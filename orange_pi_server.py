#!/usr/bin/env python3
"""HTTP JSON service that owns the Orange Pi RS-485 motor connection."""

from __future__ import annotations

import argparse
import glob
import json
import logging
import os
import queue
import signal
import threading
import time
import uuid
from dataclasses import asdict, dataclass, field, replace
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Callable

from isv2_tool import (
    DEFAULT_PARITY,
    DEFAULT_STOPBITS,
    ISV2Error,
    ModbusRTUClient,
    SerialConfig,
)
from robot_control import (
    ActiveMotion,
    ControllerSettings,
    DEFAULT_ACCEL,
    DEFAULT_BAUD,
    DEFAULT_DECEL,
    DEFAULT_LEFT_ID,
    DEFAULT_MAX_RPM,
    DEFAULT_RIGHT_ID,
    DEFAULT_START_RPM,
    DEFAULT_TIMEOUT,
    JOG_REFRESH_INTERVAL,
    MOTION_DIRECTIONS,
    RobotController,
    apply_controller_settings,
    motion_wheel_speeds,
    soft_stop_motion,
)


LOG = logging.getLogger("isv2-server")
API_PREFIX = "/api/v1"
MAX_JSON_BODY = 64 * 1024
CONTROL_ACTIONS = frozenset({"start_motion", "stop_motion", "force_stop"})


def _env_bool(name: str, default: bool) -> bool:
    value = os.environ.get(name)
    if value is None:
        return default
    normalized = value.strip().lower()
    if normalized in {"1", "true", "yes", "on"}:
        return True
    if normalized in {"0", "false", "no", "off"}:
        return False
    raise ValueError(f"{name} must be true or false")


def _env_int(name: str, default: int) -> int:
    value = os.environ.get(name)
    return default if value is None else int(value, 0)


def _env_float(name: str, default: float) -> float:
    value = os.environ.get(name)
    return default if value is None else float(value)


@dataclass(frozen=True)
class ServerConfig:
    bind_host: str = "192.168.2.5"
    bind_port: int = 8000
    serial_port: str = "auto"
    baud: int = DEFAULT_BAUD
    timeout: float = DEFAULT_TIMEOUT
    parity: str = DEFAULT_PARITY
    stopbits: int = DEFAULT_STOPBITS
    left_id: int = DEFAULT_LEFT_ID
    right_id: int = DEFAULT_RIGHT_ID
    left_inverted: bool = False
    right_inverted: bool = False
    max_rpm: int = DEFAULT_MAX_RPM
    default_accel: int = DEFAULT_ACCEL
    default_decel: int = DEFAULT_DECEL
    jog_refresh_interval: float = JOG_REFRESH_INTERVAL
    heartbeat_interval: float = 0.25
    watchdog_timeout: float = 1.0
    serial_retry_interval: float = 2.0

    @classmethod
    def from_env(cls) -> "ServerConfig":
        config = cls(
            bind_host=os.environ.get("ISV2_BIND_HOST", cls.bind_host),
            bind_port=_env_int("ISV2_BIND_PORT", cls.bind_port),
            serial_port=os.environ.get("ISV2_SERIAL_PORT", cls.serial_port),
            baud=_env_int("ISV2_BAUD", cls.baud),
            timeout=_env_float("ISV2_SERIAL_TIMEOUT", cls.timeout),
            parity=os.environ.get("ISV2_PARITY", cls.parity).upper(),
            stopbits=_env_int("ISV2_STOPBITS", cls.stopbits),
            left_id=_env_int("ISV2_LEFT_ID", cls.left_id),
            right_id=_env_int("ISV2_RIGHT_ID", cls.right_id),
            left_inverted=_env_bool("ISV2_LEFT_INVERTED", cls.left_inverted),
            right_inverted=_env_bool("ISV2_RIGHT_INVERTED", cls.right_inverted),
            max_rpm=_env_int("ISV2_MAX_RPM", cls.max_rpm),
            default_accel=_env_int("ISV2_DEFAULT_ACCEL", cls.default_accel),
            default_decel=_env_int("ISV2_DEFAULT_DECEL", cls.default_decel),
            jog_refresh_interval=_env_float("ISV2_JOG_REFRESH_INTERVAL", cls.jog_refresh_interval),
            heartbeat_interval=_env_float("ISV2_HEARTBEAT_INTERVAL", cls.heartbeat_interval),
            watchdog_timeout=_env_float("ISV2_WATCHDOG_TIMEOUT", cls.watchdog_timeout),
            serial_retry_interval=_env_float("ISV2_SERIAL_RETRY_INTERVAL", cls.serial_retry_interval),
        )
        config.validate()
        return config

    def validate(self) -> None:
        if not self.bind_host:
            raise ValueError("bind_host may not be empty")
        if not 1 <= self.bind_port <= 65535:
            raise ValueError("bind_port must be in range 1..65535")
        if self.parity not in {"N", "E", "O"}:
            raise ValueError("parity must be N, E, or O")
        if self.stopbits not in {1, 2}:
            raise ValueError("stopbits must be 1 or 2")
        if not 1 <= self.left_id <= 127 or not 1 <= self.right_id <= 127:
            raise ValueError("drive IDs must be in range 1..127")
        if self.left_id == self.right_id:
            raise ValueError("left_id and right_id must differ")
        if not 1 <= self.max_rpm <= 32767:
            raise ValueError("max_rpm must be in range 1..32767")
        if not 0 <= self.default_accel <= 32767 or not 0 <= self.default_decel <= 32767:
            raise ValueError("default accel/decel must be in Pr6.25 range 0..32767")
        if not 0.02 <= self.jog_refresh_interval < 0.1:
            raise ValueError("jog_refresh_interval must be >= 0.02 and < 0.1 seconds")
        if not 0.05 <= self.heartbeat_interval < self.watchdog_timeout:
            raise ValueError("heartbeat interval must be shorter than watchdog timeout")
        if self.serial_retry_interval <= 0:
            raise ValueError("serial retry interval must be positive")

    def public_dict(self, resolved_serial_port: str | None) -> dict[str, object]:
        return {
            "serial_port": self.serial_port,
            "resolved_serial_port": resolved_serial_port,
            "baud": self.baud,
            "parity": self.parity,
            "stopbits": self.stopbits,
            "left_id": self.left_id,
            "right_id": self.right_id,
            "left_inverted": self.left_inverted,
            "right_inverted": self.right_inverted,
            "max_rpm": self.max_rpm,
            "default_accel": self.default_accel,
            "default_decel": self.default_decel,
        }


class APIError(Exception):
    def __init__(self, status: int, code: str, message: str):
        super().__init__(message)
        self.status = status
        self.code = code
        self.message = message


@dataclass(frozen=True)
class MotionRequest:
    direction: str
    rpm: int
    accel: int
    decel: int


@dataclass
class WorkerRequest:
    action: str
    session_id: str | None = None
    payload: dict[str, object] = field(default_factory=dict)
    event: threading.Event = field(default_factory=threading.Event)
    result: dict[str, object] | None = None
    error: Exception | None = None


class ControllerService:
    """Owns all serial I/O and exposes thread-safe operations to HTTP handlers."""

    def __init__(
        self,
        config: ServerConfig,
        *,
        client_factory: Callable[[SerialConfig], object] = ModbusRTUClient,
        controller_factory: Callable[..., RobotController] = RobotController,
        soft_stop_sleep: Callable[[float], None] = time.sleep,
    ):
        config.validate()
        self.config = config
        self._client_factory = client_factory
        self._controller_factory = controller_factory
        self._soft_stop_sleep = soft_stop_sleep
        self._lock = threading.RLock()
        self._commands: queue.Queue[WorkerRequest] = queue.Queue()
        self._shutdown = threading.Event()
        self._worker: threading.Thread | None = None

        self._state = "serial_unavailable"
        self._last_error: str | None = None
        self._resolved_serial_port: str | None = None
        self._client_cm: object | None = None
        self._controller: RobotController | None = None
        self._active_motion: ActiveMotion | None = None
        self._desired_motion: MotionRequest | None = None
        self._next_refresh_at = 0.0
        self._next_serial_retry_at = 0.0

        self._session_id: str | None = None
        self._session_name: str | None = None
        self._last_heartbeat = 0.0

    def start(self) -> None:
        if self._worker is not None and self._worker.is_alive():
            return
        self._shutdown.clear()
        self._worker = threading.Thread(target=self._worker_loop, name="isv2-serial", daemon=True)
        self._worker.start()

    def shutdown(self, timeout: float = 8.0) -> None:
        if self._worker is None:
            return
        request = WorkerRequest("force_stop")
        self._commands.put(request)
        deadline = time.monotonic() + max(0.1, timeout - 0.5)
        request.event.wait(timeout=max(0.1, timeout - 0.5))
        while time.monotonic() < deadline:
            with self._lock:
                stopping = self._active_motion is not None or self._state == "soft_stopping"
            if not stopping:
                break
            time.sleep(0.05)
        self._shutdown.set()
        self._commands.put(WorkerRequest("wake"))
        self._worker.join(timeout=0.5)
        self._worker = None

    def snapshot(self) -> dict[str, object]:
        with self._lock:
            motion = None
            if self._active_motion is not None:
                motion = {
                    "direction": self._active_motion.direction,
                    "rpm": self._active_motion.rpm,
                    "accel": self._active_motion.settings.accel,
                    "decel": self._active_motion.settings.decel,
                }
            return {
                "ok": True,
                "service": "isv2-orange-pi-controller",
                "state": self._state,
                "serial_connected": self._controller is not None,
                "last_error": self._last_error,
                "session_active": self._session_id is not None,
                "motion": motion,
                "config": self.config.public_dict(self._resolved_serial_port),
            }

    def create_session(self, client_name: str) -> dict[str, object]:
        with self._lock:
            if self._session_id is not None:
                raise APIError(409, "session_busy", "已有上位机占用控制会话")
            if self._state in {"moving", "soft_stopping"}:
                raise APIError(409, "motor_busy", "电机仍在运动或停车中，请稍后重试")
            self._session_id = uuid.uuid4().hex
            self._session_name = client_name[:100]
            self._last_heartbeat = time.monotonic()
            session_id = self._session_id
            state = self._state
            config = self.config.public_dict(self._resolved_serial_port)
        LOG.info("control session acquired by %s", client_name or "unknown client")
        return {
            "ok": True,
            "session_id": session_id,
            "heartbeat_interval_ms": int(round(self.config.heartbeat_interval * 1000)),
            "watchdog_timeout_ms": int(round(self.config.watchdog_timeout * 1000)),
            "state": state,
            "config": config,
        }

    def heartbeat(self, session_id: str) -> dict[str, object]:
        with self._lock:
            self._assert_session_locked(session_id)
            self._last_heartbeat = time.monotonic()
            state = self._state
        return {"ok": True, "state": state}

    def delete_session(self, session_id: str) -> dict[str, object]:
        with self._lock:
            self._assert_session_locked(session_id)
            self._session_id = None
            self._session_name = None
            self._last_heartbeat = 0.0
        self._commands.put(WorkerRequest("force_stop"))
        LOG.info("control session released")
        return {"ok": True, "accepted": True}

    def start_motion(self, session_id: str, request: MotionRequest) -> dict[str, object]:
        self._require_serial_available(session_id)
        return self._submit(WorkerRequest("start_motion", session_id, {"motion": request}), 2.0)

    def stop_motion(self, session_id: str) -> dict[str, object]:
        self._assert_session(session_id)
        return self._submit(WorkerRequest("stop_motion", session_id), 2.0)

    def jog(
        self,
        session_id: str,
        *,
        wheel: str,
        direction: str,
        duration_ms: int,
        rpm: int,
        accel: int,
        decel: int,
    ) -> dict[str, object]:
        self._require_serial_available(session_id)
        return self._submit(
            WorkerRequest(
                "jog",
                session_id,
                {
                    "wheel": wheel,
                    "direction": direction,
                    "duration_ms": duration_ms,
                    "rpm": rpm,
                    "accel": accel,
                    "decel": decel,
                },
            ),
            4.0,
        )

    def read_alarms(self, session_id: str) -> dict[str, object]:
        return self._submit_serial_call(session_id, "read_alarms", 10.0)

    def reset_alarms(self, session_id: str) -> dict[str, object]:
        return self._submit_serial_call(session_id, "reset_alarms", 10.0)

    def read_motion_status(self, session_id: str) -> dict[str, object]:
        return self._submit_serial_call(session_id, "read_motion_status", 10.0)

    def read_diagnostics(self, session_id: str) -> dict[str, object]:
        return self._submit_serial_call(session_id, "read_diagnostics", 15.0)

    def _submit_serial_call(self, session_id: str, action: str, timeout: float) -> dict[str, object]:
        self._require_serial_available(session_id)
        return self._submit(WorkerRequest(action, session_id), timeout)

    def _submit(self, request: WorkerRequest, timeout: float) -> dict[str, object]:
        self._commands.put(request)
        if not request.event.wait(timeout):
            raise APIError(503, "worker_timeout", "串口工作线程未在限定时间内响应")
        if request.error is not None:
            if isinstance(request.error, APIError):
                raise request.error
            raise APIError(503, "serial_error", str(request.error))
        return request.result or {"ok": True}

    def _assert_session(self, session_id: str) -> None:
        with self._lock:
            self._assert_session_locked(session_id)

    def _assert_session_locked(self, session_id: str) -> None:
        if not session_id or self._session_id != session_id:
            raise APIError(409, "invalid_session", "控制会话不存在或已失效")

    def _require_serial_available(self, session_id: str) -> None:
        with self._lock:
            self._assert_session_locked(session_id)
            if self._controller is None or self._state in {"serial_unavailable", "fault"}:
                raise APIError(503, "serial_unavailable", "USB-RS485 串口当前不可用")

    def _worker_loop(self) -> None:
        LOG.info("serial worker started")
        while not self._shutdown.is_set():
            if self._controller is None:
                self._expire_session_if_needed()
                self._drain_commands()
                now = time.monotonic()
                if now >= self._next_serial_retry_at:
                    self._try_open_serial()
                    self._next_serial_retry_at = now + self.config.serial_retry_interval
                self._shutdown.wait(0.05)
                continue

            try:
                self._expire_session_if_needed()
                self._drain_commands()
                self._reconcile_motion()
                self._refresh_motion_if_due()
            except Exception as exc:
                self._serial_fault(exc)
            self._shutdown.wait(0.01)

        self._close_serial()
        LOG.info("serial worker stopped")

    def _serial_candidates(self) -> list[str]:
        if self.config.serial_port != "auto":
            return [self.config.serial_port]
        candidates = sorted(glob.glob("/dev/serial/by-id/*"))
        candidates.extend(path for path in sorted(glob.glob("/dev/ttyUSB*")) if path not in candidates)
        if not candidates:
            candidates.append("/dev/ttyUSB0")
        return candidates

    def _try_open_serial(self) -> None:
        errors: list[str] = []
        for port in self._serial_candidates():
            config = SerialConfig(
                port=port,
                baud=self.config.baud,
                timeout=self.config.timeout,
                parity=self.config.parity,
                stopbits=self.config.stopbits,
            )
            client_cm = None
            try:
                client_cm = self._client_factory(config)
                client = client_cm.__enter__()
                controller = self._controller_factory(
                    client,
                    left_id=self.config.left_id,
                    right_id=self.config.right_id,
                    left_inverted=self.config.left_inverted,
                    right_inverted=self.config.right_inverted,
                    max_rpm=self.config.max_rpm,
                    accel=self.config.default_accel,
                    decel=self.config.default_decel,
                )
            except Exception as exc:
                errors.append(f"{port}: {exc}")
                if client_cm is not None:
                    try:
                        client_cm.__exit__(None, None, None)
                    except Exception:
                        pass
                continue
            with self._lock:
                self._client_cm = client_cm
                self._controller = controller
                self._resolved_serial_port = port
                self._state = "idle"
                self._last_error = None
            LOG.info("serial connected: %s at %s %s8%s", port, self.config.baud, self.config.parity, self.config.stopbits)
            return

        message = "; ".join(errors) if errors else "no serial candidates"
        with self._lock:
            self._state = "serial_unavailable"
            self._last_error = message
            self._resolved_serial_port = None
        LOG.debug("serial unavailable: %s", message)

    def _close_serial(self) -> None:
        client_cm = self._client_cm
        self._client_cm = None
        self._controller = None
        self._resolved_serial_port = None
        if client_cm is not None:
            try:
                client_cm.__exit__(None, None, None)
            except Exception as exc:
                LOG.warning("error closing serial port: %s", exc)

    def _serial_fault(self, exc: Exception) -> None:
        LOG.error("serial/control fault: %s", exc)
        with self._lock:
            self._state = "fault"
            self._last_error = str(exc)
            self._active_motion = None
            self._desired_motion = None
        self._close_serial()
        self._next_serial_retry_at = time.monotonic() + self.config.serial_retry_interval

    def _expire_session_if_needed(self) -> None:
        with self._lock:
            if self._session_id is None:
                return
            if time.monotonic() - self._last_heartbeat <= self.config.watchdog_timeout:
                return
            client_name = self._session_name
            self._session_id = None
            self._session_name = None
            self._last_heartbeat = 0.0
            self._desired_motion = None
        LOG.warning("control session heartbeat expired for %s; starting soft stop", client_name)

    def _drain_commands(self, *, control_only: bool = False) -> None:
        deferred: list[WorkerRequest] = []
        while True:
            try:
                request = self._commands.get_nowait()
            except queue.Empty:
                break
            if control_only and request.action not in CONTROL_ACTIONS:
                deferred.append(request)
                continue
            self._handle_request(request)
        for request in deferred:
            self._commands.put(request)

    def _handle_request(self, request: WorkerRequest) -> None:
        try:
            if request.action == "wake":
                result = {"ok": True}
            elif request.action == "force_stop":
                self._desired_motion = None
                result = {"ok": True, "accepted": True}
            else:
                with self._lock:
                    self._assert_session_locked(request.session_id or "")
                if request.action == "start_motion":
                    if self._controller is None:
                        raise APIError(503, "serial_unavailable", "USB-RS485 串口当前不可用")
                    motion = request.payload.get("motion")
                    if not isinstance(motion, MotionRequest):
                        raise APIError(400, "invalid_request", "运动请求格式错误")
                    self._desired_motion = motion
                    result = {"ok": True, "accepted": True}
                elif request.action == "stop_motion":
                    self._desired_motion = None
                    result = {"ok": True, "accepted": True}
                else:
                    result = self._execute_serial_request(request)
        except Exception as exc:
            if isinstance(exc, OSError):
                self._serial_fault(exc)
            request.error = exc
        else:
            request.result = result
        finally:
            request.event.set()

    def _execute_serial_request(self, request: WorkerRequest) -> dict[str, object]:
        controller = self._controller
        if controller is None:
            raise APIError(503, "serial_unavailable", "USB-RS485 串口当前不可用")
        if self._active_motion is not None or self._state == "soft_stopping":
            raise APIError(409, "motor_busy", "请先停止运动，再执行诊断或点动命令")
        if request.action == "read_alarms":
            return {"ok": True, "alarms": controller.read_alarms()}
        if request.action == "reset_alarms":
            controller.reset_alarms()
            return {"ok": True}
        if request.action == "read_motion_status":
            return {"ok": True, "motion_status": controller.read_motion_status()}
        if request.action == "read_diagnostics":
            return {"ok": True, "diagnostics": controller.read_diagnostics()}
        if request.action == "jog":
            return self._execute_jog(controller, request.payload)
        raise APIError(400, "unknown_command", f"未知串口命令：{request.action}")

    def _execute_jog(self, controller: RobotController, payload: dict[str, object]) -> dict[str, object]:
        settings = self._settings(
            accel=int(payload["accel"]),
            decel=int(payload["decel"]),
        )
        apply_controller_settings(controller, settings)
        controller.configure_jog_profile(int(payload["rpm"]))
        with self._lock:
            self._state = "moving"
        try:
            frames = controller.jog(
                str(payload["wheel"]),
                str(payload["direction"]),
                duration=int(payload["duration_ms"]) / 1000.0,
            )
        finally:
            with self._lock:
                if self._state != "fault":
                    self._state = "idle"
        return {"ok": True, "frames": frames}

    def _reconcile_motion(self) -> None:
        active = self._active_motion
        desired = self._desired_motion
        if active is not None:
            active_request = MotionRequest(
                active.direction,
                active.rpm,
                active.settings.accel,
                active.settings.decel,
            )
            if desired != active_request:
                self._perform_soft_stop(active)
                self._active_motion = None
                with self._lock:
                    self._state = "idle"
                self._drain_commands()
                desired = self._desired_motion
        if self._active_motion is None and desired is not None:
            self._begin_motion(desired)

    def _begin_motion(self, request: MotionRequest) -> None:
        controller = self._controller
        if controller is None:
            raise ISV2Error("serial port is not open")
        settings = self._settings(accel=request.accel, decel=request.decel)
        apply_controller_settings(controller, settings)
        speed = controller.configure_jog_profile(request.rpm)
        active = ActiveMotion(request.direction, settings, speed)
        left_rpm, right_rpm = motion_wheel_speeds(
            request.direction,
            speed,
            left_inverted=settings.left_inverted,
            right_inverted=settings.right_inverted,
            max_rpm=settings.max_rpm,
        )
        controller.send_wheel_jog(left_rpm, right_rpm)
        self._active_motion = active
        self._next_refresh_at = time.monotonic() + self.config.jog_refresh_interval
        with self._lock:
            self._state = "moving"
        LOG.info("motion started: %s at %s rpm", request.direction, speed)

    def _refresh_motion_if_due(self) -> None:
        active = self._active_motion
        now = time.monotonic()
        if active is None or now < self._next_refresh_at:
            return
        controller = self._controller
        if controller is None:
            raise ISV2Error("serial port is not open")
        left_rpm, right_rpm = motion_wheel_speeds(
            active.direction,
            active.rpm,
            left_inverted=active.settings.left_inverted,
            right_inverted=active.settings.right_inverted,
            max_rpm=active.settings.max_rpm,
        )
        controller.send_wheel_jog(left_rpm, right_rpm)
        self._next_refresh_at = time.monotonic() + self.config.jog_refresh_interval

    def _perform_soft_stop(self, active: ActiveMotion) -> None:
        controller = self._controller
        if controller is None:
            raise ISV2Error("serial port is not open")
        with self._lock:
            self._state = "soft_stopping"
        speeds = soft_stop_motion(
            controller,
            active,
            sleep=self._soft_stop_sleep,
            stage_callback=lambda: self._drain_commands(control_only=True),
        )
        LOG.info("soft stop completed: %s", " -> ".join(str(value) for value in speeds))

    def _settings(self, *, accel: int, decel: int) -> ControllerSettings:
        return ControllerSettings(
            left_id=self.config.left_id,
            right_id=self.config.right_id,
            left_inverted=self.config.left_inverted,
            right_inverted=self.config.right_inverted,
            max_rpm=self.config.max_rpm,
            accel=accel,
            decel=decel,
        )


def _require_int(
    payload: dict[str, object], name: str, minimum: int, maximum: int, *, default: int | None = None
) -> int:
    value = payload.get(name, default)
    if isinstance(value, bool) or not isinstance(value, int):
        raise APIError(400, "invalid_parameter", f"{name} 必须是整数")
    if not minimum <= value <= maximum:
        raise APIError(400, "invalid_parameter", f"{name} 必须在 {minimum}..{maximum} 范围内")
    return value


def _require_choice(payload: dict[str, object], name: str, choices: set[str] | frozenset[str]) -> str:
    value = payload.get(name)
    if not isinstance(value, str) or value not in choices:
        allowed = ", ".join(sorted(choices))
        raise APIError(400, "invalid_parameter", f"{name} 必须是以下值之一：{allowed}")
    return value


def build_http_handler(service: ControllerService) -> type[BaseHTTPRequestHandler]:
    class Handler(BaseHTTPRequestHandler):
        server_version = "ISV2OrangePi/1.0"

        def log_message(self, fmt: str, *args: object) -> None:
            LOG.info("http %s - %s", self.client_address[0], fmt % args)

        def do_GET(self) -> None:
            self._dispatch("GET")

        def do_POST(self) -> None:
            self._dispatch("POST")

        def do_DELETE(self) -> None:
            self._dispatch("DELETE")

        def _dispatch(self, method: str) -> None:
            try:
                result = self._route(method)
            except APIError as exc:
                self._send_json(
                    exc.status,
                    {"ok": False, "error": {"code": exc.code, "message": exc.message}},
                )
            except Exception as exc:
                LOG.exception("unhandled HTTP request error")
                self._send_json(
                    500,
                    {"ok": False, "error": {"code": "internal_error", "message": str(exc)}},
                )
            else:
                self._send_json(200, result)

        def _route(self, method: str) -> dict[str, object]:
            path = self.path.split("?", 1)[0]
            if not path.startswith(API_PREFIX):
                raise APIError(404, "not_found", "接口不存在")
            route = path[len(API_PREFIX) :] or "/"
            payload = self._read_json() if method in {"POST", "DELETE"} else {}
            if method == "GET" and route == "/health":
                return service.snapshot()
            if method == "POST" and route == "/session":
                client_name = payload.get("client_name", "unknown client")
                if not isinstance(client_name, str):
                    raise APIError(400, "invalid_parameter", "client_name 必须是字符串")
                return service.create_session(client_name)

            session_id = self.headers.get("X-Control-Session", "")
            if method == "DELETE" and route == "/session":
                return service.delete_session(session_id)
            if method == "POST" and route == "/heartbeat":
                return service.heartbeat(session_id)
            if method == "POST" and route == "/motion/start":
                direction = _require_choice(payload, "direction", MOTION_DIRECTIONS)
                rpm = _require_int(payload, "rpm", 1, service.config.max_rpm)
                accel = _require_int(payload, "accel", 0, 32767, default=service.config.default_accel)
                decel = _require_int(payload, "decel", 0, 32767, default=service.config.default_decel)
                return service.start_motion(session_id, MotionRequest(direction, rpm, accel, decel))
            if method == "POST" and route == "/motion/stop":
                return service.stop_motion(session_id)
            if method == "POST" and route == "/jog":
                wheel = _require_choice(payload, "wheel", {"left", "right"})
                direction = _require_choice(payload, "direction", {"left", "right"})
                duration_ms = _require_int(payload, "duration_ms", 1, 500, default=500)
                rpm = _require_int(payload, "rpm", 1, service.config.max_rpm, default=DEFAULT_START_RPM)
                accel = _require_int(payload, "accel", 0, 32767, default=service.config.default_accel)
                decel = _require_int(payload, "decel", 0, 32767, default=service.config.default_decel)
                return service.jog(
                    session_id,
                    wheel=wheel,
                    direction=direction,
                    duration_ms=duration_ms,
                    rpm=rpm,
                    accel=accel,
                    decel=decel,
                )
            if method == "GET" and route == "/alarms":
                return service.read_alarms(session_id)
            if method == "POST" and route == "/alarms/reset":
                return service.reset_alarms(session_id)
            if method == "GET" and route == "/motion-status":
                return service.read_motion_status(session_id)
            if method == "GET" and route == "/diagnostics":
                return service.read_diagnostics(session_id)
            raise APIError(404, "not_found", "接口不存在")

        def _read_json(self) -> dict[str, object]:
            length_text = self.headers.get("Content-Length", "0")
            try:
                length = int(length_text)
            except ValueError as exc:
                raise APIError(400, "invalid_json", "Content-Length 无效") from exc
            if length < 0 or length > MAX_JSON_BODY:
                raise APIError(400, "invalid_json", "请求体过大")
            if length == 0:
                return {}
            raw = self.rfile.read(length)
            try:
                payload = json.loads(raw.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                raise APIError(400, "invalid_json", "请求体不是有效 JSON") from exc
            if not isinstance(payload, dict):
                raise APIError(400, "invalid_json", "JSON 请求体必须是对象")
            return payload

        def _send_json(self, status: int, payload: dict[str, object]) -> None:
            data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

    return Handler


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Orange Pi HTTP to iSV2-RS RS-485 controller")
    parser.add_argument("--host", help="override ISV2_BIND_HOST")
    parser.add_argument("--port", type=int, help="override ISV2_BIND_PORT")
    parser.add_argument("--serial-port", help="override ISV2_SERIAL_PORT; use auto to scan")
    parser.add_argument("--print-config", action="store_true", help="print effective config and exit")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        config = ServerConfig.from_env()
        if args.host:
            config = replace(config, bind_host=args.host)
        if args.port:
            config = replace(config, bind_port=args.port)
        if args.serial_port:
            config = replace(config, serial_port=args.serial_port)
        config.validate()
    except (TypeError, ValueError) as exc:
        print(f"configuration error: {exc}")
        return 2
    if args.print_config:
        print(json.dumps(asdict(config), ensure_ascii=False, indent=2))
        return 0

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    service = ControllerService(config)
    service.start()
    try:
        httpd = ThreadingHTTPServer((config.bind_host, config.bind_port), build_http_handler(service))
    except OSError as exc:
        service.shutdown()
        LOG.error("cannot bind %s:%s: %s", config.bind_host, config.bind_port, exc)
        return 1

    def request_shutdown(_signum: int, _frame: object) -> None:
        threading.Thread(target=httpd.shutdown, daemon=True).start()

    signal.signal(signal.SIGTERM, request_shutdown)
    signal.signal(signal.SIGINT, request_shutdown)
    LOG.info("HTTP service listening on http://%s:%s%s", config.bind_host, config.bind_port, API_PREFIX)
    try:
        httpd.serve_forever(poll_interval=0.2)
    finally:
        httpd.server_close()
        service.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
