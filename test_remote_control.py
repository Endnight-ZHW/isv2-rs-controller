from __future__ import annotations

import json
import threading
import time
import unittest
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer

import orange_pi_server
import remote_client
import robot_gui


def wait_until(predicate, timeout=2.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return bool(predicate())


class FakeSerialClient:
    def __init__(self, _config=None):
        self.calls = []
        self.closed = False

    def __enter__(self):
        return self

    def __exit__(self, _exc_type, _exc, _tb):
        self.closed = True

    def write_single_register(self, drive_id, address, value):
        self.calls.append(("single", drive_id, address, value))

    def write_multiple_registers(self, drive_id, address, values):
        self.calls.append(("multi", drive_id, address, list(values)))

    def read_registers(self, drive_id, address, count):
        self.calls.append(("read", drive_id, address, count))
        if address == 0x0B06 and count == 3:
            if drive_id == 1:
                return [0xFFCE, 120, 80]
            return [50, 110, 70]
        if address == 0x0B06 and count == 6:
            return [0, 0, 0, 0, 48, 30]
        if address == 0x6200 and count == 6:
            return [0, 0, 0, 0, 0, 0]
        if address == 0x0609 and count == 1:
            return [400]
        if address == 0x0633 and count == 1:
            return [200]
        return [0] * count


class ServiceHarness:
    def __init__(self, **overrides):
        values = {
            "bind_host": "127.0.0.1",
            "bind_port": 8000,
            "serial_port": "/dev/fake485",
            "jog_refresh_interval": 0.05,
            "heartbeat_interval": 0.05,
            "watchdog_timeout": 0.2,
            "serial_retry_interval": 0.05,
        }
        values.update(overrides)
        self.config = orange_pi_server.ServerConfig(**values)
        self.clients = []

        def factory(config):
            client = FakeSerialClient(config)
            self.clients.append(client)
            return client

        self.service = orange_pi_server.ControllerService(
            self.config,
            client_factory=factory,
            soft_stop_sleep=lambda _seconds: None,
        )
        self.service.start()
        if not wait_until(lambda: self.service.snapshot()["state"] == "idle"):
            raise AssertionError("fake serial service did not reach idle")

    def close(self):
        self.service.shutdown()


class ControllerServiceTests(unittest.TestCase):
    def setUp(self):
        self.harness = ServiceHarness()

    def tearDown(self):
        self.harness.close()

    def test_session_is_exclusive(self):
        first = self.harness.service.create_session("first")
        with self.assertRaises(orange_pi_server.APIError) as context:
            self.harness.service.create_session("second")
        self.assertEqual(context.exception.status, 409)
        self.harness.service.delete_session(first["session_id"])
        self.assertTrue(wait_until(lambda: not self.harness.service.snapshot()["session_active"]))

    def test_heartbeat_timeout_soft_stops_and_does_not_resume(self):
        session = self.harness.service.create_session("watchdog")
        self.harness.service.start_motion(
            session["session_id"],
            orange_pi_server.MotionRequest("forward", 5, 0, 0),
        )
        self.assertTrue(wait_until(lambda: self.harness.service.snapshot()["state"] == "moving"))
        self.assertTrue(
            wait_until(
                lambda: self.harness.service.snapshot()["state"] == "idle"
                and not self.harness.service.snapshot()["session_active"],
                timeout=2.0,
            )
        )
        call_count = len(self.harness.clients[0].calls)
        time.sleep(0.15)
        self.assertEqual(self.harness.service.snapshot()["state"], "idle")
        self.assertEqual(len(self.harness.clients[0].calls), call_count)

    def test_direction_change_soft_stops_then_starts_latest_direction(self):
        session = self.harness.service.create_session("direction")
        session_id = session["session_id"]
        self.harness.service.start_motion(
            session_id, orange_pi_server.MotionRequest("forward", 5, 0, 0)
        )
        self.assertTrue(wait_until(lambda: self.harness.service.snapshot()["state"] == "moving"))
        self.harness.service.heartbeat(session_id)
        self.harness.service.start_motion(
            session_id, orange_pi_server.MotionRequest("left", 5, 0, 0)
        )
        self.assertTrue(
            wait_until(
                lambda: (self.harness.service.snapshot().get("motion") or {}).get("direction")
                == "left",
                timeout=2.0,
            )
        )

    def test_server_rejects_speed_over_fixed_cap(self):
        with self.assertRaises(orange_pi_server.APIError):
            orange_pi_server._require_int({"rpm": 1501}, "rpm", 1, 1500)

class SerialReconnectTests(unittest.TestCase):
    def test_failed_open_is_retried_and_returns_idle(self):
        attempts = []

        def flaky_factory(config):
            attempts.append(config.port)
            if len(attempts) == 1:
                raise OSError("adapter missing")
            return FakeSerialClient(config)

        config = orange_pi_server.ServerConfig(
            bind_host="127.0.0.1",
            serial_port="/dev/fake485",
            jog_refresh_interval=0.05,
            heartbeat_interval=0.05,
            watchdog_timeout=0.2,
            serial_retry_interval=0.05,
        )
        service = orange_pi_server.ControllerService(
            config, client_factory=flaky_factory, soft_stop_sleep=lambda _seconds: None
        )
        service.start()
        try:
            self.assertTrue(wait_until(lambda: service.snapshot()["state"] == "idle"))
            self.assertGreaterEqual(len(attempts), 2)
            self.assertFalse(service.snapshot()["session_active"])
        finally:
            service.shutdown()

    def test_serial_io_error_closes_stale_handle_and_reconnects(self):
        clients = []

        class ReadFailureClient(FakeSerialClient):
            def read_registers(self, drive_id, address, count):
                if address == 0x2203:
                    raise OSError(5, "Input/output error")
                return super().read_registers(drive_id, address, count)

        def factory(config):
            client = ReadFailureClient(config) if not clients else FakeSerialClient(config)
            clients.append(client)
            return client

        config = orange_pi_server.ServerConfig(
            bind_host="127.0.0.1",
            serial_port="/dev/fake485",
            jog_refresh_interval=0.05,
            heartbeat_interval=0.05,
            watchdog_timeout=1.0,
            serial_retry_interval=0.05,
        )
        service = orange_pi_server.ControllerService(
            config, client_factory=factory, soft_stop_sleep=lambda _seconds: None
        )
        service.start()
        try:
            self.assertTrue(wait_until(lambda: service.snapshot()["state"] == "idle"))
            session = service.create_session("replug")
            with self.assertRaises(orange_pi_server.APIError) as context:
                service.read_alarms(session["session_id"])
            self.assertEqual(context.exception.status, 503)
            self.assertTrue(wait_until(lambda: len(clients) >= 2 and service.snapshot()["state"] == "idle"))
            self.assertTrue(clients[0].closed)
            self.assertIsNone(service.snapshot()["motion"])
        finally:
            service.shutdown()


class HTTPAPITests(unittest.TestCase):
    def setUp(self):
        self.harness = ServiceHarness(watchdog_timeout=1.0)
        self.httpd = ThreadingHTTPServer(
            ("127.0.0.1", 0),
            orange_pi_server.build_http_handler(self.harness.service),
        )
        self.thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        self.thread.start()
        self.port = self.httpd.server_address[1]

    def tearDown(self):
        self.httpd.shutdown()
        self.httpd.server_close()
        self.thread.join(timeout=1.0)
        self.harness.close()

    def request(self, method, path, payload=None, session_id=None):
        data = None if payload is None else json.dumps(payload).encode("utf-8")
        headers = {"Content-Type": "application/json"}
        if session_id:
            headers["X-Control-Session"] = session_id
        request = urllib.request.Request(
            f"http://127.0.0.1:{self.port}/api/v1{path}",
            data=data,
            headers=headers,
            method=method,
        )
        try:
            with urllib.request.urlopen(request, timeout=2.0) as response:
                return response.status, json.loads(response.read())
        except urllib.error.HTTPError as exc:
            try:
                return exc.code, json.loads(exc.read())
            finally:
                exc.close()

    def test_health_session_validation_and_remote_client(self):
        status, health = self.request("GET", "/health")
        self.assertEqual(status, 200)
        self.assertEqual(health["state"], "idle")

        client = remote_client.RemoteRobotClient("127.0.0.1", self.port)
        session = client.create_session()
        self.assertIn("session_id", session)
        self.assertEqual(client.heartbeat()["state"], "idle")
        try:
            with self.assertRaises(remote_client.RemoteAPIError) as context:
                client.start_motion("forward", 1501, 0, 0)
            self.assertEqual(context.exception.status, 400)
        finally:
            client.close()

    def test_missing_session_and_invalid_json_error_shapes(self):
        status, result = self.request("POST", "/heartbeat", {})
        self.assertEqual(status, 409)
        self.assertFalse(result["ok"])
        self.assertEqual(result["error"]["code"], "invalid_session")

        request = urllib.request.Request(
            f"http://127.0.0.1:{self.port}/api/v1/session",
            data=b"not json",
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with self.assertRaises(urllib.error.HTTPError) as context:
            urllib.request.urlopen(request, timeout=2.0)
        try:
            result = json.loads(context.exception.read())
            self.assertEqual(context.exception.code, 400)
            self.assertEqual(result["error"]["code"], "invalid_json")
        finally:
            context.exception.close()


if __name__ == "__main__":
    unittest.main()
