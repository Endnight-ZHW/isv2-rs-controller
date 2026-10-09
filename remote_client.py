#!/usr/bin/env python3
"""UI-independent HTTP client for the Orange Pi motor service."""

from __future__ import annotations

import json
import threading
import urllib.error
import urllib.request
from typing import Callable

from isv2_tool import ISV2Error
from robot_control import DEFAULT_ACCEL, DEFAULT_DECEL, DEFAULT_START_RPM


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
