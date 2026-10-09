#!/usr/bin/env python3
"""Command line Modbus RTU tool for Leadshine iSV2-RS drives."""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from typing import Iterable, Sequence


DEFAULT_BAUD = 9600
DEFAULT_TIMEOUT = 0.4
DEFAULT_PARITY = "N"
DEFAULT_STOPBITS = 2
MAX_READ_WORDS = 100
MAX_JOG_DURATION = 5.0

FC_READ = 0x03
FC_WRITE_SINGLE = 0x06
FC_WRITE_MULTIPLE = 0x10

ADDR_CONTROL_WORD = 0x1801
ADDR_SAVE_STATUS = 0x1901
ADDR_ALARM = 0x2203
ADDR_BAUD_SETTING = 0x053D
ADDR_AXIS_ID = 0x053F

CONTROL_RESET_ALARM = 0x1111
CONTROL_RESET_HISTORY = 0x1122
CONTROL_SAVE_EEPROM = 0x2211
CONTROL_JOG_LEFT = 0x4001
CONTROL_JOG_RIGHT = 0x4002

BAUD_SETTING_MAP = {
    0: 2400,
    1: 4800,
    2: 9600,
    3: 19200,
    4: 38400,
    5: 57600,
    6: 115200,
}

EXCEPTION_CODES = {
    0x01: "function code error",
    0x02: "address error",
    0x03: "data error or value out of range",
    0x08: "CRC error reported by drive",
}

ALARM_CODES = {
    0x000: "normal",
    0x0E0: "overcurrent",
    0x0E1: "overcurrent",
    0x100: "motor overload",
    0x101: "drive overload",
    0x102: "motor stall overload",
    0x120: "regenerative resistor overload",
    0x150: "encoder disconnected",
    0x151: "encoder data error",
    0x152: "encoder HALL signal error",
    0x153: "encoder battery error",
    0x170: "encoder data error",
    0x171: "encoder parameter error",
    0x172: "encoder parameter error",
    0x180: "position error too large",
    0x190: "motor vibration too large",
    0x1A0: "motor overspeed",
    0x1A1: "motor out of control",
    0x0D0: "undervoltage",
    0x0C0: "overvoltage",
    0x210: "input assignment duplicate or invalid",
    0x211: "input assignment duplicate or invalid",
    0x212: "input assignment duplicate or invalid",
    0x240: "parameter storage error",
    0x570: "emergency stop",
}


class ISV2Error(Exception):
    """Base exception for user-facing command failures."""


class ResponseTimeout(ISV2Error):
    pass


class CrcError(ISV2Error):
    pass


class ModbusExceptionResponse(ISV2Error):
    def __init__(self, slave_id: int, function_code: int, exception_code: int):
        message = EXCEPTION_CODES.get(exception_code, "unknown exception")
        super().__init__(
            f"ID {slave_id} exception for function 0x{function_code:02X}: "
            f"0x{exception_code:02X} ({message})"
        )
        self.slave_id = slave_id
        self.function_code = function_code
        self.exception_code = exception_code


@dataclass(frozen=True)
class SerialConfig:
    port: str
    baud: int = DEFAULT_BAUD
    timeout: float = DEFAULT_TIMEOUT
    parity: str = DEFAULT_PARITY
    stopbits: int = DEFAULT_STOPBITS


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def append_crc(data: bytes) -> bytes:
    crc = crc16_modbus(data)
    return data + bytes((crc & 0xFF, (crc >> 8) & 0xFF))


def verify_crc(frame: bytes) -> None:
    if len(frame) < 4:
        raise CrcError("response too short to contain CRC")
    expected = crc16_modbus(frame[:-2])
    actual = frame[-2] | (frame[-1] << 8)
    if actual != expected:
        raise CrcError(
            f"CRC mismatch: expected append {expected & 0xFF:02X} "
            f"{(expected >> 8) & 0xFF:02X}, got {frame[-2]:02X} {frame[-1]:02X}"
        )


def parse_int(value: str) -> int:
    try:
        return int(value, 0)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"invalid integer: {value}") from exc


def parse_word(value: str) -> int:
    number = parse_int(value)
    if not 0 <= number <= 0xFFFF:
        raise argparse.ArgumentTypeError(f"value out of 16-bit range: {value}")
    return number


def parse_address(value: str) -> int:
    number = parse_int(value)
    if not 0 <= number <= 0xFFFF:
        raise argparse.ArgumentTypeError(f"address out of range: {value}")
    return number


def parse_slave_id(value: str) -> int:
    number = parse_int(value)
    if not 1 <= number <= 127:
        raise argparse.ArgumentTypeError("drive ID must be in range 1..127")
    return number


def parse_ids(spec: str) -> list[int]:
    ids: list[int] = []
    seen: set[int] = set()
    for raw_part in spec.split(","):
        part = raw_part.strip()
        if not part:
            continue
        if "-" in part:
            start_text, end_text = part.split("-", 1)
            start = parse_slave_id(start_text)
            end = parse_slave_id(end_text)
            if start > end:
                raise argparse.ArgumentTypeError(f"invalid ID range: {part}")
            candidates = range(start, end + 1)
        else:
            candidates = (parse_slave_id(part),)
        for slave_id in candidates:
            if slave_id not in seen:
                ids.append(slave_id)
                seen.add(slave_id)
    if not ids:
        raise argparse.ArgumentTypeError("at least one ID is required")
    return ids


def hex_bytes(data: bytes) -> str:
    return " ".join(f"{byte:02X}" for byte in data)


def require_serial_module():
    try:
        import serial  # type: ignore
    except ModuleNotFoundError as exc:
        raise ISV2Error(
            "pyserial is not installed. Run: python -m pip install -r requirements.txt"
        ) from exc
    return serial


class ModbusRTUClient:
    def __init__(self, config: SerialConfig, verbose: bool = False):
        self.config = config
        self.verbose = verbose
        self._serial = None

    def __enter__(self) -> "ModbusRTUClient":
        serial = require_serial_module()
        parity_map = {
            "N": serial.PARITY_NONE,
            "E": serial.PARITY_EVEN,
            "O": serial.PARITY_ODD,
        }
        stopbits_map = {
            1: serial.STOPBITS_ONE,
            2: serial.STOPBITS_TWO,
        }
        self._serial = serial.Serial(
            port=self.config.port,
            baudrate=self.config.baud,
            bytesize=serial.EIGHTBITS,
            parity=parity_map[self.config.parity],
            stopbits=stopbits_map[self.config.stopbits],
            timeout=self.config.timeout,
            write_timeout=self.config.timeout,
        )
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        if self._serial is not None:
            self._serial.close()

    @property
    def serial(self):
        if self._serial is None:
            raise ISV2Error("serial port is not open")
        return self._serial

    def _read_exact(self, count: int) -> bytes:
        data = self.serial.read(count)
        if len(data) != count:
            raise ResponseTimeout(
                f"timeout reading response: expected {count} byte(s), got {len(data)}"
            )
        return data

    def _transact(self, slave_id: int, function_code: int, payload: bytes) -> bytes:
        request = append_crc(bytes((slave_id, function_code)) + payload)
        if self.verbose:
            print(f"TX: {hex_bytes(request)}")
        self.serial.reset_input_buffer()
        self.serial.write(request)
        self.serial.flush()

        header = self._read_exact(2)
        response_slave = header[0]
        response_fc = header[1]

        if response_slave != slave_id:
            raise ISV2Error(
                f"unexpected slave ID in response: expected {slave_id}, got {response_slave}"
            )

        if response_fc & 0x80:
            rest = self._read_exact(3)
            response = header + rest
            verify_crc(response)
            raise ModbusExceptionResponse(slave_id, response_fc & 0x7F, response[2])

        if response_fc != function_code:
            raise ISV2Error(
                f"unexpected function code: expected 0x{function_code:02X}, "
                f"got 0x{response_fc:02X}"
            )

        if function_code == FC_READ:
            byte_count = self._read_exact(1)
            rest = self._read_exact(byte_count[0] + 2)
            response = header + byte_count + rest
        elif function_code in (FC_WRITE_SINGLE, FC_WRITE_MULTIPLE):
            response = header + self._read_exact(6)
        else:
            raise ISV2Error(f"unsupported function code: 0x{function_code:02X}")

        verify_crc(response)
        if self.verbose:
            print(f"RX: {hex_bytes(response)}")
        return response

    def read_registers(self, slave_id: int, address: int, count: int) -> list[int]:
        if not 1 <= count <= MAX_READ_WORDS:
            raise ISV2Error(f"read count must be in range 1..{MAX_READ_WORDS}")
        payload = address.to_bytes(2, "big") + count.to_bytes(2, "big")
        response = self._transact(slave_id, FC_READ, payload)
        byte_count = response[2]
        if byte_count != count * 2:
            raise ISV2Error(
                f"unexpected byte count: expected {count * 2}, got {byte_count}"
            )
        values = []
        data = response[3 : 3 + byte_count]
        for offset in range(0, byte_count, 2):
            values.append(int.from_bytes(data[offset : offset + 2], "big"))
        return values

    def write_single_register(self, slave_id: int, address: int, value: int) -> None:
        payload = address.to_bytes(2, "big") + value.to_bytes(2, "big")
        response = self._transact(slave_id, FC_WRITE_SINGLE, payload)
        echo_address = int.from_bytes(response[2:4], "big")
        echo_value = int.from_bytes(response[4:6], "big")
        if echo_address != address or echo_value != value:
            raise ISV2Error(
                f"write echo mismatch: address 0x{echo_address:04X}, "
                f"value 0x{echo_value:04X}"
            )

    def write_multiple_registers(
        self, slave_id: int, address: int, values: Sequence[int]
    ) -> None:
        if not values:
            raise ISV2Error("at least one value is required")
        if len(values) > MAX_READ_WORDS:
            raise ISV2Error(f"write count must be <= {MAX_READ_WORDS}")
        data = b"".join(value.to_bytes(2, "big") for value in values)
        payload = (
            address.to_bytes(2, "big")
            + len(values).to_bytes(2, "big")
            + bytes((len(data),))
            + data
        )
        response = self._transact(slave_id, FC_WRITE_MULTIPLE, payload)
        echo_address = int.from_bytes(response[2:4], "big")
        echo_count = int.from_bytes(response[4:6], "big")
        if echo_address != address or echo_count != len(values):
            raise ISV2Error(
                f"write-multiple echo mismatch: address 0x{echo_address:04X}, "
                f"count {echo_count}"
            )

    def read_u32(self, slave_id: int, address: int) -> int:
        high, low = self.read_registers(slave_id, address, 2)
        return (high << 16) | low

    def write_u32(self, slave_id: int, address: int, value: int) -> None:
        if not 0 <= value <= 0xFFFFFFFF:
            raise ISV2Error("32-bit value must be in range 0..0xFFFFFFFF")
        self.write_multiple_registers(
            slave_id,
            address,
            ((value >> 16) & 0xFFFF, value & 0xFFFF),
        )


def serial_config_from_args(args: argparse.Namespace) -> SerialConfig:
    return SerialConfig(
        port=args.port,
        baud=args.baud,
        timeout=args.timeout,
        parity=args.parity,
        stopbits=args.stopbits,
    )


def format_value(address: int, value: int) -> str:
    return f"0x{address:04X}: 0x{value:04X} ({value})"


def command_list_ports(_: argparse.Namespace) -> int:
    try:
        from serial.tools import list_ports  # type: ignore
    except ModuleNotFoundError as exc:
        raise ISV2Error(
            "pyserial is not installed. Run: python -m pip install -r requirements.txt"
        ) from exc

    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found.")
        return 0
    for port in ports:
        details = port.description or "no description"
        hwid = port.hwid or "no hardware id"
        print(f"{port.device}\t{details}\t{hwid}")
    return 0


def command_probe(args: argparse.Namespace) -> int:
    ids = parse_ids(args.ids)
    found = 0
    with ModbusRTUClient(serial_config_from_args(args), verbose=args.verbose) as client:
        for slave_id in ids:
            try:
                axis_id = client.read_registers(slave_id, ADDR_AXIS_ID, 1)[0]
                baud_setting = client.read_registers(slave_id, ADDR_BAUD_SETTING, 1)[0]
            except ISV2Error as exc:
                if args.verbose:
                    print(f"ID {slave_id}: no response ({exc})")
                continue
            baud_text = BAUD_SETTING_MAP.get(baud_setting, "unknown")
            print(
                f"ID {slave_id}: online, Pr5.31 axis={axis_id}, "
                f"Pr5.30 baud-setting={baud_setting} ({baud_text})"
            )
            found += 1
    if found == 0:
        print(
            "No drive responded. Check COM port, A/B wiring, slave ID, baud rate, "
            "8N2 serial format, termination, and RS485 ground."
        )
        return 1
    return 0


def command_read(args: argparse.Namespace) -> int:
    with ModbusRTUClient(serial_config_from_args(args), verbose=args.verbose) as client:
        values = client.read_registers(args.id, args.addr, args.count)
    for offset, value in enumerate(values):
        print(format_value(args.addr + offset, value))
    return 0


def command_read32(args: argparse.Namespace) -> int:
    with ModbusRTUClient(serial_config_from_args(args), verbose=args.verbose) as client:
        value = client.read_u32(args.id, args.addr)
    if args.signed and value & 0x80000000:
        signed_value = value - 0x100000000
        print(f"0x{args.addr:04X}/0x{args.addr + 1:04X}: 0x{value:08X} ({signed_value})")
    else:
        print(f"0x{args.addr:04X}/0x{args.addr + 1:04X}: 0x{value:08X} ({value})")
    return 0


def command_write(args: argparse.Namespace) -> int:
    with ModbusRTUClient(serial_config_from_args(args), verbose=args.verbose) as client:
        client.write_single_register(args.id, args.addr, args.value)
    print(f"Wrote ID {args.id} 0x{args.addr:04X}=0x{args.value:04X}")
    return 0


def command_write_multiple(args: argparse.Namespace) -> int:
    with ModbusRTUClient(serial_config_from_args(args), verbose=args.verbose) as client:
        client.write_multiple_registers(args.id, args.addr, args.values)
    values = " ".join(f"0x{value:04X}" for value in args.values)
    print(f"Wrote ID {args.id} from 0x{args.addr:04X}: {values}")
    return 0


def command_write32(args: argparse.Namespace) -> int:
    with ModbusRTUClient(serial_config_from_args(args), verbose=args.verbose) as client:
        client.write_u32(args.id, args.addr, args.value)
    print(
        f"Wrote ID {args.id} 0x{args.addr:04X}/0x{args.addr + 1:04X}=0x{args.value:08X}"
    )
    return 0


def describe_alarm(raw_value: int) -> str:
    masked = raw_value & 0x0FFF
    description = ALARM_CODES.get(masked, "unknown; check manual")
    return f"raw=0x{raw_value:04X}, code=0x{masked:03X}, {description}"


def command_alarm(args: argparse.Namespace) -> int:
    with ModbusRTUClient(serial_config_from_args(args), verbose=args.verbose) as client:
        raw_value = client.read_registers(args.id, ADDR_ALARM, 1)[0]
    print(describe_alarm(raw_value))
    return 0


def command_reset_alarm(args: argparse.Namespace) -> int:
    control = CONTROL_RESET_HISTORY if args.history else CONTROL_RESET_ALARM
    with ModbusRTUClient(serial_config_from_args(args), verbose=args.verbose) as client:
        client.write_single_register(args.id, ADDR_CONTROL_WORD, control)
    action = "historical alarm" if args.history else "current alarm"
    print(f"Sent {action} reset to ID {args.id}.")
    return 0


def command_save(args: argparse.Namespace) -> int:
    deadline = time.monotonic() + args.wait
    with ModbusRTUClient(serial_config_from_args(args), verbose=args.verbose) as client:
        client.write_single_register(args.id, ADDR_CONTROL_WORD, CONTROL_SAVE_EEPROM)
        while True:
            status = client.read_registers(args.id, ADDR_SAVE_STATUS, 1)[0]
            if status == 0x5555:
                print("EEPROM save succeeded.")
                return 0
            if status == 0xAAAA:
                print("EEPROM save failed.")
                return 1
            if time.monotonic() >= deadline:
                print(f"Timed out waiting for save status; last status 0x{status:04X}.")
                return 1
            time.sleep(args.poll_interval)


def command_jog(args: argparse.Namespace) -> int:
    if args.duration <= 0:
        raise ISV2Error("duration must be greater than zero")
    if args.duration > MAX_JOG_DURATION:
        raise ISV2Error(f"duration is limited to {MAX_JOG_DURATION:.1f}s")
    if not 0.01 <= args.interval <= 0.1:
        raise ISV2Error("interval must be in range 0.01..0.10 seconds")

    control = CONTROL_JOG_LEFT if args.dir == "left" else CONTROL_JOG_RIGHT
    deadline = time.monotonic() + args.duration
    sent = 0
    print(
        f"JOG {args.dir} on ID {args.id} for {args.duration:.3f}s; "
        "release is performed by stopping refresh frames."
    )
    with ModbusRTUClient(serial_config_from_args(args), verbose=args.verbose) as client:
        try:
            while time.monotonic() < deadline:
                client.write_single_register(args.id, ADDR_CONTROL_WORD, control)
                sent += 1
                remaining = deadline - time.monotonic()
                if remaining > 0:
                    time.sleep(min(args.interval, remaining))
        except KeyboardInterrupt:
            print("Interrupted; stopped sending JOG refresh frames.")
            return 130
    print(f"JOG refresh frames sent: {sent}")
    return 0


def add_serial_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--port", required=True, help="serial port, for example COM3")
    parser.add_argument("--baud", type=parse_int, default=DEFAULT_BAUD, help="baud rate")
    parser.add_argument(
        "--timeout",
        type=float,
        default=DEFAULT_TIMEOUT,
        help="serial read/write timeout in seconds",
    )
    parser.add_argument(
        "--parity",
        choices=("N", "E", "O"),
        default=DEFAULT_PARITY,
        help="serial parity; iSV2 default is N",
    )
    parser.add_argument(
        "--stopbits",
        type=int,
        choices=(1, 2),
        default=DEFAULT_STOPBITS,
        help="serial stop bits; iSV2 default Pr5.29=5 uses 2",
    )
    parser.add_argument("--verbose", action="store_true", help="print TX/RX frames")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Modbus RTU debug tool for iSV2-RS drives over USB-RS485."
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    list_ports = subparsers.add_parser("list-ports", help="list serial ports")
    list_ports.set_defaults(func=command_list_ports)

    probe = subparsers.add_parser("probe", help="scan drive IDs by reading Pr5.31/Pr5.30")
    add_serial_args(probe)
    probe.add_argument("--ids", default="1-31", help="ID list/ranges, for example 1-31,40")
    probe.set_defaults(func=command_probe)

    read = subparsers.add_parser("read", help="read 16-bit register(s)")
    add_serial_args(read)
    read.add_argument("--id", type=parse_slave_id, required=True)
    read.add_argument("--addr", type=parse_address, required=True)
    read.add_argument("--count", type=parse_int, default=1)
    read.set_defaults(func=command_read)

    read32 = subparsers.add_parser("read32", help="read one 32-bit parameter")
    add_serial_args(read32)
    read32.add_argument("--id", type=parse_slave_id, required=True)
    read32.add_argument("--addr", type=parse_address, required=True)
    read32.add_argument("--signed", action="store_true", help="print signed decimal value")
    read32.set_defaults(func=command_read32)

    write = subparsers.add_parser("write", help="write one 16-bit register")
    add_serial_args(write)
    write.add_argument("--id", type=parse_slave_id, required=True)
    write.add_argument("--addr", type=parse_address, required=True)
    write.add_argument("--value", type=parse_word, required=True)
    write.set_defaults(func=command_write)

    write_multiple = subparsers.add_parser(
        "write-multiple", help="write consecutive 16-bit registers with function 0x10"
    )
    add_serial_args(write_multiple)
    write_multiple.add_argument("--id", type=parse_slave_id, required=True)
    write_multiple.add_argument("--addr", type=parse_address, required=True)
    write_multiple.add_argument("--values", type=parse_word, nargs="+", required=True)
    write_multiple.set_defaults(func=command_write_multiple)

    write32 = subparsers.add_parser("write32", help="write one 32-bit parameter")
    add_serial_args(write32)
    write32.add_argument("--id", type=parse_slave_id, required=True)
    write32.add_argument("--addr", type=parse_address, required=True)
    write32.add_argument("--value", type=parse_int, required=True)
    write32.set_defaults(func=command_write32)

    alarm = subparsers.add_parser("alarm", help="read current alarm register 0x2203")
    add_serial_args(alarm)
    alarm.add_argument("--id", type=parse_slave_id, required=True)
    alarm.set_defaults(func=command_alarm)

    reset_alarm = subparsers.add_parser("reset-alarm", help="reset current or historical alarm")
    add_serial_args(reset_alarm)
    reset_alarm.add_argument("--id", type=parse_slave_id, required=True)
    reset_alarm.add_argument(
        "--history", action="store_true", help="reset historical alarm instead of current alarm"
    )
    reset_alarm.set_defaults(func=command_reset_alarm)

    save = subparsers.add_parser("save", help="save all parameters to EEPROM")
    add_serial_args(save)
    save.add_argument("--id", type=parse_slave_id, required=True)
    save.add_argument("--wait", type=float, default=3.0, help="seconds to wait for status")
    save.add_argument(
        "--poll-interval", type=float, default=0.1, help="status polling interval in seconds"
    )
    save.set_defaults(func=command_save)

    jog = subparsers.add_parser("jog", help="limited JOG by refreshing control word")
    add_serial_args(jog)
    jog.add_argument("--id", type=parse_slave_id, required=True)
    jog.add_argument("--dir", choices=("left", "right"), required=True)
    jog.add_argument("--duration", type=float, default=1.0)
    jog.add_argument("--interval", type=float, default=0.05)
    jog.set_defaults(func=command_jog)

    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        return args.func(args)
    except ISV2Error as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
