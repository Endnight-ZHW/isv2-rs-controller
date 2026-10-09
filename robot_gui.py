#!/usr/bin/env python3
"""Tkinter upper-computer client for the Orange Pi motor service."""

from __future__ import annotations

import json
import queue
import threading
import time
import tkinter as tk
import urllib.error
import urllib.request
from tkinter import messagebox, ttk
from tkinter.scrolledtext import ScrolledText
from typing import Callable

from isv2_tool import CONTROL_JOG_LEFT, CONTROL_JOG_RIGHT, ISV2Error
from robot_control import (  # Re-exported for compatibility and offline tests.
    ADDR_JOG_ACCEL,
    ADDR_JOG_DECEL,
    ADDR_JOG_SPEED,
    ADDR_MON_IO_INPUT,
    ADDR_MON_OVERLOAD,
    ADDR_MON_SPEED,
    ADDR_PR0_BASE,
    ADDR_PR_TRIGGER,
    ADDR_PR_WARNING,
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
    DIRECTION_FACTORS,
    DIRECTION_LABELS,
    JOG_INTER_DRIVE_DELAY,
    JOG_REFRESH_INTERVAL,
    PR_MODE_SPEED,
    PR_TRIGGER_ESTOP,
    PR_TRIGGER_START,
    RobotController,
    SOFT_STOP_FACTORS,
    SOFT_STOP_MIN_STAGE_DELAY,
    SOFT_STOP_SETTLE_DELAY,
    build_pr_speed_words,
    clamp_rpm,
    describe_jog_control,
    differential_speeds,
    format_words,
    jog_control_for_speed,
    motion_wheel_speeds,
    profile_time_ms,
    signed_16bit_word,
    signed_word_to_int,
    soft_stop_speeds,
    unsigned_word,
)


DEFAULT_HOST = "192.168.2.5"
DEFAULT_HTTP_PORT = 8000
DEFAULT_REQUEST_TIMEOUT = 2.0
DIAGNOSTIC_REQUEST_TIMEOUT = 10.0


class RemoteAPIError(ISV2Error):
    def __init__(self, message: str, *, code: str = "remote_error", status: int = 0):
        super().__init__(message)
        self.code = code
        self.status = status


class RemoteRobotClient:
    """Small HTTP JSON client with an independent heartbeat thread."""

    def __init__(self, host: str = DEFAULT_HOST, port: int = DEFAULT_HTTP_PORT):
        self.host = host
        self.port = int(port)
        self.base_url = f"http://{host}:{self.port}/api/v1"
        self.session_id: str | None = None
        self.heartbeat_interval = 0.25
        self.watchdog_timeout = 1.0
        self.config: dict[str, object] = {}
        self._heartbeat_stop = threading.Event()
        self._heartbeat_thread: threading.Thread | None = None

    def _request(
        self,
        method: str,
        path: str,
        payload: dict[str, object] | None = None,
        *,
        require_session: bool = True,
        timeout: float = DEFAULT_REQUEST_TIMEOUT,
    ) -> dict[str, object]:
        data = None
        headers = {"Accept": "application/json", "User-Agent": "isv2-upper-computer/1"}
        if payload is not None:
            data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
            headers["Content-Type"] = "application/json; charset=utf-8"
        if require_session:
            if not self.session_id:
                raise RemoteAPIError("尚未建立控制会话", code="no_session")
            headers["X-Control-Session"] = self.session_id
        request = urllib.request.Request(
            self.base_url + path,
            data=data,
            headers=headers,
            method=method,
        )
        try:
            with urllib.request.urlopen(request, timeout=timeout) as response:
                raw = response.read()
                status = response.status
        except urllib.error.HTTPError as exc:
            try:
                raw = exc.read()
                status = exc.code
            finally:
                exc.close()
        except (urllib.error.URLError, TimeoutError, OSError) as exc:
            reason = getattr(exc, "reason", exc)
            raise RemoteAPIError(f"无法连接香橙派服务：{reason}", code="network_error") from exc

        try:
            result = json.loads(raw.decode("utf-8")) if raw else {}
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise RemoteAPIError(
                f"服务返回了无效 JSON（HTTP {status}）", code="invalid_response", status=status
            ) from exc
        if not isinstance(result, dict):
            raise RemoteAPIError("服务返回格式错误", code="invalid_response", status=status)
        if status >= 400 or not result.get("ok", False):
            error = result.get("error")
            if isinstance(error, dict):
                code = str(error.get("code", "remote_error"))
                message = str(error.get("message", f"HTTP {status}"))
            else:
                code = "remote_error"
                message = f"HTTP {status}"
            raise RemoteAPIError(message, code=code, status=status)
        return result

    def health(self) -> dict[str, object]:
        return self._request("GET", "/health", require_session=False)

    def create_session(self) -> dict[str, object]:
        result = self._request(
            "POST",
            "/session",
            {"client_name": "Tkinter upper computer"},
            require_session=False,
        )
        self.session_id = str(result["session_id"])
        self.heartbeat_interval = float(result.get("heartbeat_interval_ms", 250)) / 1000.0
        self.watchdog_timeout = float(result.get("watchdog_timeout_ms", 1000)) / 1000.0
        config = result.get("config", {})
        self.config = config if isinstance(config, dict) else {}
        return result

    def heartbeat(self) -> dict[str, object]:
        return self._request("POST", "/heartbeat", {})

    def start_heartbeat(self, on_error: Callable[[Exception], None]) -> None:
        if not self.session_id:
            raise RemoteAPIError("尚未建立控制会话", code="no_session")
        self._heartbeat_stop.clear()

        def target() -> None:
            while not self._heartbeat_stop.wait(self.heartbeat_interval):
                try:
                    self.heartbeat()
                except Exception as exc:
                    if not self._heartbeat_stop.is_set():
                        on_error(exc)
                    return

        self._heartbeat_thread = threading.Thread(target=target, daemon=True)
        self._heartbeat_thread.start()

    def start_motion(self, direction: str, rpm: int, accel: int, decel: int) -> dict[str, object]:
        return self._request(
            "POST",
            "/motion/start",
            {"direction": direction, "rpm": rpm, "accel": accel, "decel": decel},
        )

    def stop_motion(self) -> dict[str, object]:
        return self._request("POST", "/motion/stop", {})

    def jog(
        self,
        wheel: str,
        direction: str,
        duration_ms: int = 500,
        *,
        rpm: int = DEFAULT_START_RPM,
        accel: int = DEFAULT_ACCEL,
        decel: int = DEFAULT_DECEL,
    ) -> dict[str, object]:
        return self._request(
            "POST",
            "/jog",
            {
                "wheel": wheel,
                "direction": direction,
                "duration_ms": duration_ms,
                "rpm": rpm,
                "accel": accel,
                "decel": decel,
            },
            timeout=3.0,
        )

    def read_alarms(self) -> dict[str, object]:
        return self._request("GET", "/alarms", timeout=DIAGNOSTIC_REQUEST_TIMEOUT)

    def reset_alarms(self) -> dict[str, object]:
        return self._request("POST", "/alarms/reset", {}, timeout=DIAGNOSTIC_REQUEST_TIMEOUT)

    def read_motion_status(self) -> dict[str, object]:
        return self._request("GET", "/motion-status", timeout=DIAGNOSTIC_REQUEST_TIMEOUT)

    def read_diagnostics(self) -> dict[str, object]:
        return self._request("GET", "/diagnostics", timeout=DIAGNOSTIC_REQUEST_TIMEOUT)

    def close(self) -> None:
        self._heartbeat_stop.set()
        thread = self._heartbeat_thread
        if thread is not None and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=0.5)
        if not self.session_id:
            return
        try:
            self.stop_motion()
        except Exception:
            pass
        try:
            self._request("DELETE", "/session", {})
        except Exception:
            pass
        self.session_id = None


# Use the UI-independent implementation; the names remain exported here for
# compatibility with existing imports.
from remote_client import (  # noqa: E402
    DEFAULT_HOST as SHARED_DEFAULT_HOST,
    DEFAULT_HTTP_PORT as SHARED_DEFAULT_HTTP_PORT,
    RemoteAPIError as SharedRemoteAPIError,
    RemoteRobotClient as SharedRemoteRobotClient,
)

DEFAULT_HOST = SHARED_DEFAULT_HOST
DEFAULT_HTTP_PORT = SHARED_DEFAULT_HTTP_PORT
RemoteAPIError = SharedRemoteAPIError
RemoteRobotClient = SharedRemoteRobotClient


class RobotApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("iSV2-RS 双轮机器人远程上位机")
        self.root.geometry("800x560")
        self.root.minsize(740, 520)

        self.remote: RemoteRobotClient | None = None
        self.connecting = False
        self.closing = False
        self.busy_lock = threading.Lock()
        self.busy = False
        self.connection_generation = 0
        self.motion_queue: queue.Queue[tuple[int, str, dict[str, object]]] = queue.Queue()
        self.motion_worker = threading.Thread(target=self._motion_worker_loop, daemon=True)
        self.motion_worker.start()
        self.pressed_directions: list[str] = []
        self.mouse_motion: str | None = None
        self.sent_motion: str | None = None

        self.host_var = tk.StringVar(value=DEFAULT_HOST)
        self.http_port_var = tk.IntVar(value=DEFAULT_HTTP_PORT)
        self.max_rpm_var = tk.IntVar(value=DEFAULT_MAX_RPM)
        self.speed_var = tk.IntVar(value=DEFAULT_START_RPM)
        self.accel_var = tk.IntVar(value=DEFAULT_ACCEL)
        self.decel_var = tk.IntVar(value=DEFAULT_DECEL)
        self.status_var = tk.StringVar(value="未连接")
        self.hardware_var = tk.StringVar(value="硬件配置将在连接后从香橙派读取")

        self._build_ui()
        self._bind_keys()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def _build_ui(self) -> None:
        main = ttk.Frame(self.root, padding=12)
        main.grid(row=0, column=0, sticky="nsew")
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main.columnconfigure(0, weight=1)
        main.columnconfigure(1, weight=1)
        main.rowconfigure(2, weight=1)

        connection = ttk.LabelFrame(main, text="香橙派连接")
        connection.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 10))
        for col in range(8):
            connection.columnconfigure(col, weight=1)
        ttk.Label(connection, text="IP").grid(row=0, column=0, sticky="w", padx=6, pady=6)
        ttk.Entry(connection, textvariable=self.host_var, width=16).grid(row=0, column=1, padx=6)
        ttk.Label(connection, text="HTTP端口").grid(row=0, column=2, sticky="w", padx=6)
        ttk.Entry(connection, textvariable=self.http_port_var, width=8).grid(row=0, column=3, padx=6)
        ttk.Button(connection, text="连接", command=self.connect).grid(row=0, column=4, padx=6)
        ttk.Button(connection, text="断开", command=self.disconnect).grid(row=0, column=5, padx=6)
        ttk.Label(connection, textvariable=self.status_var).grid(
            row=0, column=6, columnspan=2, sticky="e", padx=6
        )
        ttk.Label(connection, textvariable=self.hardware_var).grid(
            row=1, column=0, columnspan=8, sticky="w", padx=6, pady=(0, 6)
        )

        control = ttk.LabelFrame(main, text="运动控制（按住运动，松开柔停）")
        control.grid(row=1, column=0, sticky="nsew", padx=(0, 6), pady=(0, 10))
        for col in range(4):
            control.columnconfigure(col, weight=1)
        ttk.Label(control, text="JOG速度 rpm").grid(row=0, column=0, sticky="w", padx=8, pady=6)
        self.speed_scale = ttk.Scale(
            control,
            from_=0,
            to=DEFAULT_MAX_RPM,
            variable=self.speed_var,
            orient="horizontal",
            command=self._sync_speed_label,
        )
        self.speed_scale.grid(row=0, column=1, columnspan=2, sticky="ew", padx=8, pady=6)
        self.speed_label = ttk.Label(control, text=str(self.speed_var.get()))
        self.speed_label.grid(row=0, column=3, sticky="e", padx=8)

        self._motion_button(control, "前进 W/↑", "forward", 1, 1)
        self._motion_button(control, "左转 A/←", "left", 2, 0)
        ttk.Button(control, text="停止 Space", command=self.stop).grid(
            row=2, column=1, sticky="ew", padx=8, pady=6
        )
        self._motion_button(control, "右转 D/→", "right", 2, 2)
        self._motion_button(control, "后退 S/↓", "backward", 3, 1)

        ttk.Label(control, text="服务端最高 rpm").grid(row=4, column=0, sticky="e", padx=8, pady=8)
        ttk.Label(control, textvariable=self.max_rpm_var).grid(row=4, column=1, sticky="w", padx=8)
        ttk.Label(control, text="JOG加/减速 ms/Krpm").grid(row=5, column=0, sticky="e", padx=8)
        ttk.Spinbox(control, from_=0, to=32767, textvariable=self.accel_var, width=8).grid(
            row=5, column=1, sticky="w", padx=8
        )
        ttk.Spinbox(control, from_=0, to=32767, textvariable=self.decel_var, width=8).grid(
            row=5, column=2, sticky="w", padx=8
        )

        debug = ttk.LabelFrame(main, text="远程调试")
        debug.grid(row=1, column=1, sticky="nsew", padx=(6, 0), pady=(0, 10))
        for col in range(2):
            debug.columnconfigure(col, weight=1)
        ttk.Button(debug, text="读取两轴报警", command=self.read_alarms).grid(
            row=0, column=0, columnspan=2, sticky="ew", padx=8, pady=6
        )
        ttk.Button(debug, text="复位两轴报警", command=self.reset_alarms).grid(
            row=1, column=0, columnspan=2, sticky="ew", padx=8, pady=6
        )
        ttk.Button(debug, text="读取运动状态", command=self.read_motion_status).grid(
            row=2, column=0, columnspan=2, sticky="ew", padx=8, pady=6
        )
        ttk.Button(debug, text="读取负载/IO诊断", command=self.read_diagnostics).grid(
            row=3, column=0, columnspan=2, sticky="ew", padx=8, pady=6
        )
        ttk.Button(debug, text="带载安全档", command=self.apply_loaded_safe_profile).grid(
            row=4, column=0, columnspan=2, sticky="ew", padx=8, pady=6
        )
        self._motion_button(debug, "按住同转 +", "both_positive", 5, 0)
        self._motion_button(debug, "按住同转 -", "both_negative", 5, 1)
        ttk.Button(debug, text="左轮 JOG 左 0.5s", command=lambda: self.jog("left", "left")).grid(
            row=6, column=0, sticky="ew", padx=8, pady=6
        )
        ttk.Button(debug, text="左轮 JOG 右 0.5s", command=lambda: self.jog("left", "right")).grid(
            row=6, column=1, sticky="ew", padx=8, pady=6
        )
        ttk.Button(debug, text="右轮 JOG 左 0.5s", command=lambda: self.jog("right", "left")).grid(
            row=7, column=0, sticky="ew", padx=8, pady=6
        )
        ttk.Button(debug, text="右轮 JOG 右 0.5s", command=lambda: self.jog("right", "right")).grid(
            row=7, column=1, sticky="ew", padx=8, pady=6
        )

        info = ttk.LabelFrame(main, text="日志")
        info.grid(row=2, column=0, columnspan=2, sticky="nsew")
        info.columnconfigure(0, weight=1)
        info.rowconfigure(0, weight=1)
        self.log_text = ScrolledText(info, height=12, wrap="word", state="disabled")
        self.log_text.grid(row=0, column=0, sticky="nsew", padx=8, pady=8)
        self.log("启动完成。连接香橙派后先读取报警，首次测试请让轮子离地并使用 5 rpm。")

    def _motion_button(self, parent: ttk.Frame, label: str, direction: str, row: int, col: int) -> None:
        button = ttk.Button(parent, text=label)
        button.grid(row=row, column=col, sticky="ew", padx=8, pady=6)
        button.bind("<ButtonPress-1>", lambda _event, value=direction: self._mouse_press(value))
        button.bind("<ButtonRelease-1>", lambda _event, value=direction: self._mouse_release(value))

    def _bind_keys(self) -> None:
        self.root.bind_all("<KeyPress>", self._on_key_press)
        self.root.bind_all("<KeyRelease>", self._on_key_release)
        self.root.bind("<FocusOut>", self._on_focus_out)

    @staticmethod
    def _key_direction(key: str) -> str | None:
        return {
            "w": "forward",
            "up": "forward",
            "s": "backward",
            "down": "backward",
            "a": "left",
            "left": "left",
            "d": "right",
            "right": "right",
        }.get(key.lower())

    @staticmethod
    def _is_text_input(widget: tk.Misc) -> bool:
        try:
            return widget.winfo_class() in {"Entry", "TEntry", "TSpinbox", "Text"}
        except tk.TclError:
            return False

    def _on_key_press(self, event: tk.Event) -> None:
        key = event.keysym.lower()
        if key == "space":
            self.stop()
            return
        if self._is_text_input(event.widget):
            return
        direction = self._key_direction(key)
        if direction is None or direction in self.pressed_directions:
            return
        self.pressed_directions.append(direction)
        self._apply_effective_motion()

    def _on_key_release(self, event: tk.Event) -> None:
        direction = self._key_direction(event.keysym.lower())
        if direction is None or direction not in self.pressed_directions:
            return
        self.pressed_directions.remove(direction)
        self._apply_effective_motion()

    def _mouse_press(self, direction: str) -> None:
        self.mouse_motion = direction
        self._apply_effective_motion()

    def _mouse_release(self, direction: str) -> None:
        if self.mouse_motion == direction:
            self.mouse_motion = None
        self._apply_effective_motion()

    def _on_focus_out(self, _event: tk.Event) -> None:
        self.root.after(50, self._stop_if_window_inactive)

    def _stop_if_window_inactive(self) -> None:
        if self.closing:
            return
        try:
            if self.root.focus_displayof() is None:
                self.stop(force=False)
        except tk.TclError:
            pass

    def _apply_effective_motion(self) -> None:
        direction = self.mouse_motion
        if direction is None and self.pressed_directions:
            direction = self.pressed_directions[-1]
        if direction == self.sent_motion:
            return
        self.sent_motion = direction
        if direction is None:
            self._enqueue_motion("stop", {})
        else:
            self._enqueue_motion(
                "start",
                {
                    "direction": direction,
                    "rpm": int(self.speed_var.get()),
                    "accel": int(self.accel_var.get()),
                    "decel": int(self.decel_var.get()),
                },
            )

    def _enqueue_motion(self, action: str, payload: dict[str, object]) -> None:
        if self.remote is None:
            if action == "start":
                self.log("尚未连接香橙派。")
            return
        self.motion_queue.put((self.connection_generation, action, payload))

    def _motion_worker_loop(self) -> None:
        while True:
            generation, action, payload = self.motion_queue.get()
            remote = self.remote
            if remote is None or generation != self.connection_generation:
                continue
            try:
                if action == "start":
                    remote.start_motion(
                        str(payload["direction"]),
                        int(payload["rpm"]),
                        int(payload["accel"]),
                        int(payload["decel"]),
                    )
                    direction = str(payload["direction"])
                    label = DIRECTION_LABELS.get(direction, direction)
                    message = f"{label}请求已发送：{payload['rpm']} rpm。"
                else:
                    remote.stop_motion()
                    message = "柔性停止请求已发送。"
            except Exception as exc:
                if not self.closing:
                    self.root.after(0, lambda exc=exc: self._task_failed(exc, show_dialog=False))
            else:
                if not self.closing:
                    self.root.after(0, lambda message=message: self.log(message))

    def _sync_speed_label(self, _: str) -> None:
        self.speed_label.configure(text=str(int(float(self.speed_var.get()))))

    def apply_loaded_safe_profile(self) -> None:
        self.speed_var.set(min(5, int(self.max_rpm_var.get())))
        self.accel_var.set(30000)
        self.decel_var.set(30000)
        self._sync_speed_label("")
        self.log("已切换带载安全档：速度 5 rpm，加/减速 30000 ms/Krpm。")

    def connect(self) -> None:
        if self.remote is not None:
            self.log("已经连接。")
            return
        if self.connecting:
            return
        host = self.host_var.get().strip()
        try:
            port = int(self.http_port_var.get())
            if not host or not 1 <= port <= 65535:
                raise ValueError
        except ValueError:
            messagebox.showerror("连接失败", "请输入有效的香橙派 IP 和 HTTP 端口。")
            return
        self.connecting = True
        self.status_var.set("正在连接…")

        def target() -> None:
            client = RemoteRobotClient(host, port)
            try:
                session = client.create_session()
            except Exception as exc:
                self.root.after(0, lambda exc=exc: self._connect_failed(exc))
            else:
                self.root.after(0, lambda: self._connect_succeeded(client, session))

        threading.Thread(target=target, daemon=True).start()

    def _connect_failed(self, exc: Exception) -> None:
        self.connecting = False
        self.status_var.set("连接失败")
        messagebox.showerror("连接失败", str(exc))

    def _connect_succeeded(self, client: RemoteRobotClient, session: dict[str, object]) -> None:
        if self.closing:
            client.close()
            return
        self.connecting = False
        self.remote = client
        self.connection_generation += 1
        self.sent_motion = None
        config = client.config
        max_rpm = int(config.get("max_rpm", DEFAULT_MAX_RPM))
        self.max_rpm_var.set(max_rpm)
        self.speed_scale.configure(to=max_rpm)
        if int(self.speed_var.get()) > max_rpm:
            self.speed_var.set(max_rpm)
            self._sync_speed_label("")
        serial_port = config.get("resolved_serial_port") or config.get("serial_port", "未知")
        self.hardware_var.set(
            f"串口 {serial_port} / {config.get('baud', DEFAULT_BAUD)} 8N2，"
            f"左ID {config.get('left_id', DEFAULT_LEFT_ID)}，右ID {config.get('right_id', DEFAULT_RIGHT_ID)}，"
            f"最高 {max_rpm} rpm"
        )
        state = str(session.get("state", "unknown"))
        self.status_var.set(f"已连接 / {state}")
        client.start_heartbeat(
            lambda exc, expected=client: self.root.after(
                0, lambda: self._heartbeat_failed(expected, exc)
            )
        )
        self.log(
            f"已连接 {client.host}:{client.port}；心跳 {client.heartbeat_interval * 1000:.0f} ms，"
            f"失联柔停 {client.watchdog_timeout:.1f} s。"
        )

    def _heartbeat_failed(self, client: RemoteRobotClient, exc: Exception) -> None:
        if self.remote is not client or self.closing:
            return
        self.connection_generation += 1
        self.remote = None
        self.sent_motion = None
        self.pressed_directions.clear()
        self.mouse_motion = None
        self.status_var.set("心跳中断")
        self.log(f"心跳中断：{exc}；香橙派将在看门狗超时后柔性停车。")

    def disconnect(self) -> None:
        client = self.remote
        self.connection_generation += 1
        self.remote = None
        self.sent_motion = None
        self.pressed_directions.clear()
        self.mouse_motion = None
        self.status_var.set("未连接")
        if client is None:
            self.log("已断开。")
            return

        def target() -> None:
            client.close()
            if not self.closing:
                self.root.after(0, lambda: self.log("已发送柔停并断开控制会话。"))

        threading.Thread(target=target, daemon=True).start()

    def stop(self, *, force: bool = True) -> None:
        had_motion = self.sent_motion is not None
        self.pressed_directions.clear()
        self.mouse_motion = None
        self.sent_motion = None
        if force or had_motion:
            self._enqueue_motion("stop", {})

    def jog(self, wheel: str, direction: str) -> None:
        def work(client: RemoteRobotClient) -> str:
            result = client.jog(
                wheel,
                direction,
                500,
                rpm=int(self.speed_var.get()),
                accel=int(self.accel_var.get()),
                decel=int(self.decel_var.get()),
            )
            return f"{wheel}轮 JOG {direction}完成，刷新帧 {result.get('frames', '?')} 次。"

        self._run_task(work)

    def read_alarms(self) -> None:
        def work(client: RemoteRobotClient) -> str:
            result = client.read_alarms()
            alarms = result.get("alarms", {})
            if not isinstance(alarms, dict):
                raise RemoteAPIError("报警响应格式错误")
            return "\n".join(f"ID {drive_id}: {alarm}" for drive_id, alarm in alarms.items())

        self._run_task(work)

    def reset_alarms(self) -> None:
        self.stop()
        self._run_task(lambda client: (client.reset_alarms(), "已发送两轴报警复位。")[1])

    def read_motion_status(self) -> None:
        def work(client: RemoteRobotClient) -> str:
            result = client.read_motion_status()
            status = result.get("motion_status", {})
            if not isinstance(status, dict):
                raise RemoteAPIError("运动状态响应格式错误")
            lines = ["运动状态："]
            lines.extend(f"ID {drive_id}: {value}" for drive_id, value in status.items())
            return "\n".join(lines)

        self._run_task(work)

    def read_diagnostics(self) -> None:
        def work(client: RemoteRobotClient) -> str:
            result = client.read_diagnostics()
            diagnostics = result.get("diagnostics", {})
            if not isinstance(diagnostics, dict):
                raise RemoteAPIError("诊断响应格式错误")
            lines = ["负载/IO诊断："]
            lines.extend(f"ID {drive_id}: {value}" for drive_id, value in diagnostics.items())
            return "\n".join(lines)

        self._run_task(work)

    def _run_task(self, work: Callable[[RemoteRobotClient], str]) -> None:
        client = self.remote
        if client is None:
            messagebox.showwarning("未连接", "请先连接香橙派。")
            return
        with self.busy_lock:
            if self.busy:
                self.log("上一条诊断命令还未完成，已忽略本次操作。")
                return
            self.busy = True

        def target() -> None:
            try:
                message = work(client)
            except Exception as exc:
                if not self.closing:
                    self.root.after(0, lambda exc=exc: self._task_failed(exc))
            else:
                if not self.closing:
                    self.root.after(0, lambda message=message: self.log(message))
            finally:
                with self.busy_lock:
                    self.busy = False

        threading.Thread(target=target, daemon=True).start()

    def _task_failed(self, exc: Exception, *, show_dialog: bool = True) -> None:
        self.log(f"错误：{exc}")
        if show_dialog:
            messagebox.showerror("远程控制失败", str(exc))

    def log(self, message: str) -> None:
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.configure(state="normal")
        self.log_text.insert("end", f"[{timestamp}] {message}\n")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def on_close(self) -> None:
        self.closing = True
        self.connection_generation += 1
        client = self.remote
        self.remote = None
        if client is not None:
            client.close()
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    RobotApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
