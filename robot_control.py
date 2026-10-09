#!/usr/bin/env python3
"""Shared, UI-independent motor control logic for the iSV2-RS robot."""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Callable

from isv2_tool import (
    ADDR_ALARM,
    ADDR_CONTROL_WORD,
    CONTROL_JOG_LEFT,
    CONTROL_JOG_RIGHT,
    CONTROL_RESET_ALARM,
    ISV2Error,
    ModbusRTUClient,
    describe_alarm,
)


DEFAULT_BAUD = 38400
DEFAULT_LEFT_ID = 2
DEFAULT_RIGHT_ID = 1
DEFAULT_MAX_RPM = 1500
DEFAULT_START_RPM = 5
DEFAULT_ACCEL = 10000
DEFAULT_DECEL = 10000
DEFAULT_TIMEOUT = 0.4
JOG_REFRESH_INTERVAL = 0.08
JOG_INTER_DRIVE_DELAY = 0.01
SOFT_STOP_MIN_STAGE_DELAY = 0.08
SOFT_STOP_SETTLE_DELAY = 0.03
SOFT_STOP_FACTORS = (0.7, 0.45, 0.25, 0.12)

ADDR_PR0_BASE = 0x6200
ADDR_PR_TRIGGER = 0x6002
# The 0x4001/0x4002 auxiliary JOG control words use the trial-run
# parameters Pr6.04 and Pr6.25.  Pr8.39..Pr8.41 (0x6027..0x6029)
# belong to PR-mode JOG and do not affect these control words.
ADDR_JOG_SPEED = 0x0609
ADDR_JOG_ACCEL = 0x0633
ADDR_JOG_DECEL = ADDR_JOG_ACCEL  # One shared acceleration/deceleration parameter.
ADDR_MON_SPEED = 0x0B06
ADDR_MON_OVERLOAD = 0x0B0F
ADDR_MON_IO_INPUT = 0x0B11
ADDR_PR_WARNING = 0x601D

PR_MODE_SPEED = 0x0002
PR_TRIGGER_START = 0x0010
PR_TRIGGER_ESTOP = 0x0040

DIRECTION_FACTORS = {
    "forward": (1, 1),
    "backward": (-1, -1),
    "left": (-1, 1),
    "right": (1, -1),
}

DIRECTION_LABELS = {
    "forward": "前进",
    "backward": "后退",
    "left": "左转",
    "right": "右转",
    "both_positive": "双电机同转 +",
    "both_negative": "双电机同转 -",
}

MOTION_DIRECTIONS = frozenset(DIRECTION_LABELS)


@dataclass(frozen=True)
class ControllerSettings:
    left_id: int
    right_id: int
    left_inverted: bool
    right_inverted: bool
    max_rpm: int
    accel: int
    decel: int


@dataclass(frozen=True)
class ActiveMotion:
    direction: str
    settings: ControllerSettings
    rpm: int


def clamp_rpm(value: float | int, max_rpm: int = DEFAULT_MAX_RPM) -> int:
    rpm = int(round(float(value)))
    limit = abs(int(max_rpm))
    return max(-limit, min(limit, rpm))


def signed_16bit_word(value: int) -> int:
    if not -32768 <= value <= 32767:
        raise ValueError(f"signed 16-bit value out of range: {value}")
    return value & 0xFFFF


def signed_word_to_int(value: int) -> int:
    value &= 0xFFFF
    if value & 0x8000:
        return value - 0x10000
    return value


def unsigned_word(value: int, name: str) -> int:
    if not 0 <= int(value) <= 0xFFFF:
        raise ValueError(f"{name} out of 16-bit range: {value}")
    return int(value)


def format_words(values: list[int]) -> str:
    return "[" + " ".join(f"0x{value:04X}" for value in values) + "]"


def profile_time_ms(speed_rpm: int, profile_ms_per_krpm: int) -> float:
    return abs(speed_rpm) / 1000.0 * profile_ms_per_krpm


def soft_stop_speeds(speed_rpm: int) -> list[int]:
    speed = abs(int(speed_rpm))
    if speed <= 0:
        return []
    speeds = [max(1, int(round(speed * factor))) for factor in SOFT_STOP_FACTORS]
    speeds.append(1)
    result: list[int] = []
    previous: int | None = None
    for value in speeds:
        value = min(speed, value)
        if value != previous:
            result.append(value)
            previous = value
    return result


def build_pr_speed_words(
    speed_rpm: float | int,
    accel: int = DEFAULT_ACCEL,
    decel: int = DEFAULT_DECEL,
    max_rpm: int = DEFAULT_MAX_RPM,
) -> list[int]:
    speed = clamp_rpm(speed_rpm, max_rpm)
    return [
        PR_MODE_SPEED,
        0x0000,
        0x0000,
        signed_16bit_word(speed),
        unsigned_word(accel, "accel"),
        unsigned_word(decel, "decel"),
    ]


def differential_speeds(
    direction: str,
    rpm: float | int,
    *,
    left_inverted: bool = False,
    right_inverted: bool = False,
    max_rpm: int = DEFAULT_MAX_RPM,
) -> tuple[int, int]:
    if direction not in DIRECTION_FACTORS:
        raise ValueError(f"unknown direction: {direction}")
    base_rpm = abs(clamp_rpm(rpm, max_rpm))
    left_factor, right_factor = DIRECTION_FACTORS[direction]
    left = left_factor * base_rpm
    right = right_factor * base_rpm
    if left_inverted:
        left = -left
    if right_inverted:
        right = -right
    return left, right


def motion_wheel_speeds(
    direction: str,
    rpm: float | int,
    *,
    left_inverted: bool = False,
    right_inverted: bool = False,
    max_rpm: int = DEFAULT_MAX_RPM,
) -> tuple[int, int]:
    """Resolve a public motion direction to signed left/right JOG speeds."""
    speed = abs(clamp_rpm(rpm, max_rpm))
    if direction == "both_positive":
        return speed, speed
    if direction == "both_negative":
        return -speed, -speed
    return differential_speeds(
        direction,
        speed,
        left_inverted=left_inverted,
        right_inverted=right_inverted,
        max_rpm=max_rpm,
    )


def jog_control_for_speed(speed_rpm: int) -> int | None:
    if speed_rpm > 0:
        return CONTROL_JOG_RIGHT
    if speed_rpm < 0:
        return CONTROL_JOG_LEFT
    return None


def describe_jog_control(speed_rpm: int) -> str:
    control = jog_control_for_speed(speed_rpm)
    if control is None:
        return "停止"
    name = "JOG右" if control == CONTROL_JOG_RIGHT else "JOG左"
    return f"{name}/0x{control:04X}"


class RobotController:
    def __init__(
        self,
        client: ModbusRTUClient,
        *,
        left_id: int = DEFAULT_LEFT_ID,
        right_id: int = DEFAULT_RIGHT_ID,
        left_inverted: bool = False,
        right_inverted: bool = False,
        max_rpm: int = DEFAULT_MAX_RPM,
        accel: int = DEFAULT_ACCEL,
        decel: int = DEFAULT_DECEL,
    ):
        self.client = client
        self.left_id = left_id
        self.right_id = right_id
        self.left_inverted = left_inverted
        self.right_inverted = right_inverted
        self.max_rpm = max_rpm
        self.accel = accel
        self.decel = decel

    def configure(
        self,
        *,
        left_id: int,
        right_id: int,
        left_inverted: bool,
        right_inverted: bool,
        max_rpm: int,
        accel: int,
        decel: int,
    ) -> None:
        self.left_id = left_id
        self.right_id = right_id
        self.left_inverted = left_inverted
        self.right_inverted = right_inverted
        self.max_rpm = max_rpm
        self.accel = accel
        self.decel = decel

    def drive_ids(self) -> tuple[int, int]:
        return self.left_id, self.right_id

    def read_alarms(self) -> dict[int, str]:
        alarms: dict[int, str] = {}
        for drive_id in self.drive_ids():
            raw_alarm = self.client.read_registers(drive_id, ADDR_ALARM, 1)[0]
            alarms[drive_id] = describe_alarm(raw_alarm)
        return alarms

    def read_motion_status(self) -> dict[int, str]:
        status: dict[int, str] = {}
        for drive_id in self.drive_ids():
            control_mode = self.client.read_registers(drive_id, 0x0003, 1)[0]
            pr_operation = self.client.read_registers(drive_id, ADDR_PR_TRIGGER, 1)[0]
            pr0_words = self.client.read_registers(drive_id, ADDR_PR0_BASE, 6)
            jog_speed = self.client.read_registers(drive_id, ADDR_JOG_SPEED, 1)[0]
            jog_ramp = self.client.read_registers(drive_id, ADDR_JOG_ACCEL, 1)[0]
            alarm = describe_alarm(self.client.read_registers(drive_id, ADDR_ALARM, 1)[0])
            status[drive_id] = (
                f"Pr0.01=0x{control_mode:04X} ({control_mode}), "
                f"P8.02=0x{pr_operation:04X}, "
                f"JOG试机速度={jog_speed} rpm, 加减速={jog_ramp} ms/Krpm, "
                f"PR0={format_words(pr0_words)}, "
                f"报警={alarm}"
            )
        return status

    def read_diagnostics(self) -> dict[int, str]:
        diagnostics: dict[int, str] = {}
        for drive_id in self.drive_ids():
            monitor = self.client.read_registers(drive_id, ADDR_MON_SPEED, 6)
            speed = signed_word_to_int(monitor[0])
            torque = signed_word_to_int(monitor[1])
            current = signed_word_to_int(monitor[2])
            filtered_speed = signed_word_to_int(monitor[3])
            overload_rate = self.client.read_registers(drive_id, ADDR_MON_OVERLOAD, 1)[0]
            io_input = self.client.read_registers(drive_id, ADDR_MON_IO_INPUT, 1)[0]
            pr_warning = self.client.read_registers(drive_id, ADDR_PR_WARNING, 1)[0]
            alarm = describe_alarm(self.client.read_registers(drive_id, ADDR_ALARM, 1)[0])
            diagnostics[drive_id] = (
                f"速度={speed} rpm, "
                f"转矩={torque}%(|{abs(torque)}|%), "
                f"电流={current}%(|{abs(current)}|%), "
                f"滤波速度={filtered_speed} rpm, "
                f"母线={monitor[4]} V, 温度={monitor[5]}℃, "
                f"过载率={overload_rate}%, "
                f"IO输入=0x{io_input:04X}({io_input:016b}), "
                f"PR警告=0x{pr_warning:04X}, "
                f"报警={alarm}"
            )
        return diagnostics

    def reset_alarms(self) -> None:
        for drive_id in self.drive_ids():
            self.client.write_single_register(drive_id, ADDR_CONTROL_WORD, CONTROL_RESET_ALARM)

    def configure_jog_profile(self, rpm: float | int) -> int:
        speed = abs(clamp_rpm(rpm, self.max_rpm))
        ramp = unsigned_word(max(self.accel, self.decel), "JOG accel/decel")
        if ramp > 32767:
            raise ValueError(f"JOG accel/decel out of Pr6.25 range: {ramp}")
        errors: list[str] = []
        for drive_id in self.drive_ids():
            for address, value, label in (
                (ADDR_JOG_SPEED, speed, "JOG试机速度"),
                (ADDR_JOG_ACCEL, ramp, "JOG试机加减速"),
            ):
                try:
                    self.client.write_single_register(drive_id, address, value)
                except ISV2Error as exc:
                    errors.append(f"ID {drive_id} 写{label} 0x{address:04X}={value}: {exc}")
            time.sleep(JOG_INTER_DRIVE_DELAY)
        if errors:
            raise ISV2Error("; ".join(errors))
        return speed

    def set_jog_speed(self, rpm: float | int) -> int:
        speed = abs(clamp_rpm(rpm, self.max_rpm))
        errors: list[str] = []
        for drive_id in self.drive_ids():
            try:
                self.client.write_single_register(drive_id, ADDR_JOG_SPEED, speed)
            except ISV2Error as exc:
                errors.append(f"ID {drive_id} 写JOG速度 0x{ADDR_JOG_SPEED:04X}={speed}: {exc}")
            time.sleep(JOG_INTER_DRIVE_DELAY)
        if errors:
            raise ISV2Error("; ".join(errors))
        return speed

    def set_wheel_speeds(self, left_rpm: int, right_rpm: int) -> None:
        self._prepare_drive_speed(self.left_id, left_rpm)
        self._prepare_drive_speed(self.right_id, right_rpm)
        self._trigger_drive(self.left_id)
        self._trigger_drive(self.right_id)

    def debug_spin_both(self, rpm: float | int) -> tuple[int, int]:
        speed = clamp_rpm(rpm, self.max_rpm)
        self.send_wheel_jog(speed, speed)
        return speed, speed

    def jog_motion_once(self, direction: str, rpm: float | int) -> tuple[int, int]:
        left_rpm, right_rpm = differential_speeds(
            direction,
            rpm,
            left_inverted=self.left_inverted,
            right_inverted=self.right_inverted,
            max_rpm=self.max_rpm,
        )
        self.send_wheel_jog(left_rpm, right_rpm)
        return left_rpm, right_rpm

    def send_wheel_jog(self, left_rpm: int, right_rpm: int) -> None:
        left_control = jog_control_for_speed(left_rpm)
        right_control = jog_control_for_speed(right_rpm)
        errors: list[str] = []
        if left_control is not None:
            try:
                self.client.write_single_register(self.left_id, ADDR_CONTROL_WORD, left_control)
            except ISV2Error as exc:
                errors.append(f"左轮 ID {self.left_id} JOG 0x{left_control:04X}: {exc}")
        if left_control is not None and right_control is not None:
            time.sleep(JOG_INTER_DRIVE_DELAY)
        if right_control is not None:
            try:
                self.client.write_single_register(self.right_id, ADDR_CONTROL_WORD, right_control)
            except ISV2Error as exc:
                errors.append(f"右轮 ID {self.right_id} JOG 0x{right_control:04X}: {exc}")
        if errors:
            raise ISV2Error("; ".join(errors))

    def move(self, direction: str, rpm: float | int) -> tuple[int, int]:
        return self.jog_motion_once(direction, rpm)

    def stop(self) -> None:
        errors: list[str] = []
        for drive_id in self.drive_ids():
            try:
                self.client.write_single_register(drive_id, ADDR_PR_TRIGGER, PR_TRIGGER_ESTOP)
            except ISV2Error as exc:
                errors.append(f"ID {drive_id}: {exc}")
        if errors:
            raise ISV2Error("; ".join(errors))

    def jog(self, wheel: str, jog_dir: str, duration: float = 0.5, interval: float = 0.05) -> int:
        if wheel not in ("left", "right"):
            raise ValueError(f"unknown wheel: {wheel}")
        if jog_dir not in ("left", "right"):
            raise ValueError(f"unknown jog direction: {jog_dir}")
        if duration <= 0:
            raise ValueError("duration must be greater than zero")
        drive_id = self.left_id if wheel == "left" else self.right_id
        control = CONTROL_JOG_LEFT if jog_dir == "left" else CONTROL_JOG_RIGHT
        deadline = time.monotonic() + min(duration, 5.0)
        sent = 0
        while time.monotonic() < deadline:
            self.client.write_single_register(drive_id, ADDR_CONTROL_WORD, control)
            sent += 1
            remaining = deadline - time.monotonic()
            if remaining > 0:
                time.sleep(min(interval, remaining))
        return sent

    def _prepare_drive_speed(self, drive_id: int, speed_rpm: int) -> None:
        words = build_pr_speed_words(
            speed_rpm,
            accel=self.accel,
            decel=self.decel,
            max_rpm=self.max_rpm,
        )
        self.client.write_multiple_registers(drive_id, ADDR_PR0_BASE, words)

    def _trigger_drive(self, drive_id: int) -> None:
        self.client.write_single_register(drive_id, ADDR_PR_TRIGGER, PR_TRIGGER_START)


def apply_controller_settings(controller: RobotController, settings: ControllerSettings) -> None:
    controller.configure(
        left_id=settings.left_id,
        right_id=settings.right_id,
        left_inverted=settings.left_inverted,
        right_inverted=settings.right_inverted,
        max_rpm=settings.max_rpm,
        accel=settings.accel,
        decel=settings.decel,
    )


def soft_stop_motion(
    controller: RobotController,
    active_motion: ActiveMotion,
    *,
    sleep: Callable[[float], None] = time.sleep,
    stage_callback: Callable[[], None] | None = None,
) -> list[int]:
    """Ramp JOG speed to 1 rpm and then stop refreshing JOG commands."""
    settings = active_motion.settings
    speeds = soft_stop_speeds(clamp_rpm(active_motion.rpm, settings.max_rpm))
    sent_speeds: list[int] = []
    previous_speed = abs(clamp_rpm(active_motion.rpm, settings.max_rpm))
    for speed in speeds:
        if stage_callback is not None:
            stage_callback()
        apply_controller_settings(controller, settings)
        jog_speed = controller.set_jog_speed(speed)
        left_rpm, right_rpm = motion_wheel_speeds(
            active_motion.direction,
            jog_speed,
            left_inverted=settings.left_inverted,
            right_inverted=settings.right_inverted,
            max_rpm=settings.max_rpm,
        )
        controller.send_wheel_jog(left_rpm, right_rpm)
        sent_speeds.append(jog_speed)
        speed_delta = max(0, previous_speed - jog_speed)
        stage_delay = max(
            SOFT_STOP_MIN_STAGE_DELAY,
            profile_time_ms(speed_delta, settings.decel) / 1000.0
            + SOFT_STOP_SETTLE_DELAY,
        )
        sleep(stage_delay)
        previous_speed = jog_speed
    return sent_speeds
