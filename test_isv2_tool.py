import unittest

import isv2_tool
import robot_control as robot_gui


class ProtocolTests(unittest.TestCase):
    def test_crc_read_example(self):
        frame = bytes.fromhex("01 03 00 04 00 02")
        self.assertEqual(isv2_tool.crc16_modbus(frame), 0xCA85)
        self.assertEqual(isv2_tool.append_crc(frame), bytes.fromhex("01 03 00 04 00 02 85 CA"))

    def test_crc_write_example(self):
        frame = bytes.fromhex("01 06 00 04 00 02")
        self.assertEqual(isv2_tool.crc16_modbus(frame), 0xCA49)
        self.assertEqual(isv2_tool.append_crc(frame), bytes.fromhex("01 06 00 04 00 02 49 CA"))

    def test_parse_ids(self):
        self.assertEqual(isv2_tool.parse_ids("1-3,3,5"), [1, 2, 3, 5])

    def test_alarm_description_masks_high_nibble(self):
        self.assertIn("overcurrent", isv2_tool.describe_alarm(0xF0E1))
        self.assertIn("motor stall overload", isv2_tool.describe_alarm(0x0102))


class RobotGuiProtocolTests(unittest.TestCase):
    def test_signed_16bit_speed_conversion(self):
        self.assertEqual(robot_gui.signed_16bit_word(300), 0x012C)
        self.assertEqual(robot_gui.signed_16bit_word(-300), 0xFED4)
        self.assertEqual(robot_gui.signed_word_to_int(0x012C), 300)
        self.assertEqual(robot_gui.signed_word_to_int(0xFED4), -300)

    def test_differential_mapping_with_inversion(self):
        self.assertEqual(robot_gui.differential_speeds("forward", 50), (50, 50))
        self.assertEqual(
            robot_gui.differential_speeds(
                "left",
                50,
                left_inverted=True,
                right_inverted=False,
            ),
            (50, 50),
        )
        self.assertEqual(
            robot_gui.differential_speeds(
                "right",
                50,
                left_inverted=False,
                right_inverted=True,
            ),
            (50, 50),
        )

    def test_speed_is_clamped(self):
        self.assertEqual(robot_gui.clamp_rpm(999, 300), 300)
        self.assertEqual(robot_gui.clamp_rpm(-999, 300), -300)

    def test_jog_control_for_speed(self):
        self.assertEqual(robot_gui.jog_control_for_speed(50), robot_gui.CONTROL_JOG_RIGHT)
        self.assertEqual(robot_gui.jog_control_for_speed(-50), robot_gui.CONTROL_JOG_LEFT)
        self.assertIsNone(robot_gui.jog_control_for_speed(0))
        self.assertEqual(robot_gui.describe_jog_control(50), "JOG右/0x4002")
        self.assertEqual(robot_gui.describe_jog_control(-50), "JOG左/0x4001")
        self.assertEqual(robot_gui.describe_jog_control(0), "停止")

    def test_pr_speed_register_words(self):
        self.assertEqual(
            robot_gui.build_pr_speed_words(-300, accel=50, decel=50, max_rpm=300),
            [0x0002, 0x0000, 0x0000, 0xFED4, 0x0032, 0x0032],
        )

    def test_pr_speed_write_triggers_separately(self):
        class FakeClient:
            def __init__(self):
                self.calls = []

            def write_multiple_registers(self, drive_id, address, values):
                self.calls.append(("multi", drive_id, address, list(values)))

            def write_single_register(self, drive_id, address, value):
                self.calls.append(("single", drive_id, address, value))

        fake = FakeClient()
        controller = robot_gui.RobotController(fake, left_id=1, right_id=2)
        controller.set_wheel_speeds(50, -50)
        self.assertEqual(
            fake.calls,
            [
                ("multi", 1, 0x6200, [0x0002, 0x0000, 0x0000, 0x0032, 0x2710, 0x2710]),
                ("multi", 2, 0x6200, [0x0002, 0x0000, 0x0000, 0xFFCE, 0x2710, 0x2710]),
                ("single", 1, 0x6002, 0x0010),
                ("single", 2, 0x6002, 0x0010),
            ],
        )

    def test_profile_time_ms_uses_ms_per_krpm_units(self):
        self.assertEqual(robot_gui.profile_time_ms(10, 3000), 30)
        self.assertEqual(robot_gui.profile_time_ms(5, 60000), 300)

    def test_soft_stop_speeds_step_down_to_one_rpm(self):
        self.assertEqual(robot_gui.soft_stop_speeds(10), [7, 4, 2, 1])
        self.assertEqual(robot_gui.soft_stop_speeds(5), [4, 2, 1])
        self.assertEqual(robot_gui.soft_stop_speeds(0), [])

    def test_configure_jog_profile_writes_speed_accel_decel_to_both_drives(self):
        class FakeClient:
            def __init__(self):
                self.calls = []

            def write_single_register(self, drive_id, address, value):
                self.calls.append(("single", drive_id, address, value))

        fake = FakeClient()
        controller = robot_gui.RobotController(
            fake,
            left_id=1,
            right_id=2,
            max_rpm=100,
            accel=1000,
            decel=1500,
        )
        self.assertEqual(controller.configure_jog_profile(999), 100)
        self.assertEqual(
            fake.calls,
            [
                ("single", 1, 0x0609, 100),
                ("single", 1, 0x0633, 1500),
                ("single", 2, 0x0609, 100),
                ("single", 2, 0x0633, 1500),
            ],
        )

    def test_set_jog_speed_writes_speed_only_to_both_drives(self):
        class FakeClient:
            def __init__(self):
                self.calls = []

            def write_single_register(self, drive_id, address, value):
                self.calls.append(("single", drive_id, address, value))

        fake = FakeClient()
        controller = robot_gui.RobotController(fake, left_id=1, right_id=2, max_rpm=20)
        self.assertEqual(controller.set_jog_speed(999), 20)
        self.assertEqual(
            fake.calls,
            [
                ("single", 1, 0x0609, 20),
                ("single", 2, 0x0609, 20),
            ],
        )

    def test_read_diagnostics_formats_load_and_io_status(self):
        class FakeClient:
            def read_registers(self, drive_id, address, count):
                values = {
                    (1, 0x0B06, 6): [0xFFCE, 120, 80, 0xFFCE, 48, 35],
                    (1, 0x0B0F, 1): [12],
                    (1, 0x0B11, 1): [0x0005],
                    (1, 0x601D, 1): [0x0200],
                    (1, 0x2203, 1): [0x0000],
                    (2, 0x0B06, 6): [50, 110, 70, 50, 48, 34],
                    (2, 0x0B0F, 1): [10],
                    (2, 0x0B11, 1): [0x0004],
                    (2, 0x601D, 1): [0],
                    (2, 0x2203, 1): [0x0100],
                }
                return values[(drive_id, address, count)]

        controller = robot_gui.RobotController(FakeClient(), left_id=1, right_id=2)
        diagnostics = controller.read_diagnostics()
        self.assertIn("速度=-50 rpm", diagnostics[1])
        self.assertIn("电流=80%(|80|%)", diagnostics[1])
        self.assertIn("IO输入=0x0005", diagnostics[1])
        self.assertIn("PR警告=0x0200", diagnostics[1])
        self.assertIn("overload", diagnostics[2])

    def test_debug_spin_both_uses_same_speed_on_both_drives(self):
        class FakeClient:
            def __init__(self):
                self.calls = []

            def write_multiple_registers(self, drive_id, address, values):
                self.calls.append(("multi", drive_id, address, list(values)))

            def write_single_register(self, drive_id, address, value):
                self.calls.append(("single", drive_id, address, value))

        fake = FakeClient()
        controller = robot_gui.RobotController(fake, left_id=1, right_id=2)
        self.assertEqual(controller.debug_spin_both(-50), (-50, -50))
        self.assertEqual(
            fake.calls,
            [
                ("single", 1, 0x1801, 0x4001),
                ("single", 2, 0x1801, 0x4001),
            ],
        )

    def test_jog_motion_once_uses_four_jog_commands(self):
        class FakeClient:
            def __init__(self):
                self.calls = []

            def write_single_register(self, drive_id, address, value):
                self.calls.append(("single", drive_id, address, value))

        fake = FakeClient()
        controller = robot_gui.RobotController(fake, left_id=1, right_id=2)
        self.assertEqual(controller.jog_motion_once("left", 50), (-50, 50))
        self.assertEqual(
            fake.calls,
            [
                ("single", 1, 0x1801, 0x4001),
                ("single", 2, 0x1801, 0x4002),
            ],
        )

    def test_send_wheel_jog_reports_drive_specific_errors_and_tries_both(self):
        class FakeClient:
            def __init__(self):
                self.calls = []

            def write_single_register(self, drive_id, address, value):
                self.calls.append(("single", drive_id, address, value))
                if drive_id == 1:
                    raise isv2_tool.ResponseTimeout("no reply")

        fake = FakeClient()
        controller = robot_gui.RobotController(fake, left_id=1, right_id=2)
        with self.assertRaises(isv2_tool.ISV2Error) as context:
            controller.send_wheel_jog(50, -50)
        self.assertIn("左轮 ID 1 JOG 0x4002", str(context.exception))
        self.assertEqual(
            fake.calls,
            [
                ("single", 1, 0x1801, 0x4002),
                ("single", 2, 0x1801, 0x4001),
            ],
        )


if __name__ == "__main__":
    unittest.main()
