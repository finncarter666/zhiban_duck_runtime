"""校准保持的总线顺序、确认边界及单关节失败隔离。"""

import unittest
from unittest.mock import Mock, call, patch

from zhiban_runtime.hardware.device_runtime import (
    ADDR_GOAL_POS,
    ADDR_TORQUE,
    CAL_HOLD_POS,
    RobotHardware,
    SCSBus,
)
from zhiban_runtime.messages import (
    CAL_CENTER_ONE,
    CAL_ENTER,
    CAL_EXIT,
    make_calibration_request,
)


def reply(data=b"", error=0):
    body = bytes((10, len(data) + 2, error)) + data
    return b"\xff\xff" + body + bytes(((~sum(body)) & 255,))


class CalibrationHoldTest(unittest.TestCase):
    def make_bus(self, replies):
        bus = SCSBus.__new__(SCSBus)
        bus.request = Mock(side_effect=replies)
        bus.write = Mock()
        return bus

    @patch("zhiban_runtime.hardware.device_runtime.time.sleep")
    def test_target_precedes_individual_torque_enable_and_is_verified(self, _sleep):
        bus = self.make_bus([reply(), reply(b"\x00\x08"), reply(b"\x01")])
        bus.center_and_hold(10)
        self.assertEqual(
            bus.write.call_args_list,
            [call(10, 0x0B)] * 5
            + [
                call(10, 3, bytes((ADDR_GOAL_POS, 0, 8, 0, 0, 0, 0))),
                call(10, 3, bytes((ADDR_TORQUE, 1))),
            ],
        )
        self.assertEqual(
            bus.request.call_args_list,
            [
                call(10, 1),
                call(10, 2, bytes((ADDR_GOAL_POS, 2))),
                call(10, 2, bytes((ADDR_TORQUE, 1))),
            ],
        )
        self.assertEqual(CAL_HOLD_POS, 2048)

    @patch("zhiban_runtime.hardware.device_runtime.time.sleep")
    def test_missing_error_or_mismatched_readback_is_not_success(self, _sleep):
        for responses in (
            [None],
            [reply(error=1)],
            [reply(), None],
            [reply(), reply(b"\xff\x07")],
            [reply(), reply(b"\x00\x08"), reply(b"\x00")],
            [reply(), reply(b"\x00\x08"), reply(b"\x01", error=1)],
        ):
            with self.subTest(responses=responses):
                bus = self.make_bus(responses)
                with self.assertRaises(OSError):
                    bus.center_and_hold(10)

    def make_robot(self):
        robot = RobotHardware(servo=Mock(), imu=Mock())
        robot.on_cal_reply = Mock()
        robot._process_calibration_request(make_calibration_request(CAL_ENTER))
        robot.servo.reset_mock()
        return robot

    def test_only_confirmed_joint_is_marked_and_exit_unloads(self):
        robot = self.make_robot()
        robot._process_calibration_request(
            make_calibration_request(CAL_CENTER_ONE, index=0, seq=12)
        )
        robot.servo.center_and_hold.assert_called_once_with(10)
        robot.servo.torque.assert_not_called()
        self.assertEqual(robot.cal_held_mask, 1)
        self.assertFalse(robot.enabled)
        self.assertFalse(robot._bus_torque)
        robot.on_cal_reply.assert_called_once_with({"seq": 12, "index": 0, "ok": True})
        robot._process_calibration_request(make_calibration_request(CAL_EXIT))
        robot.servo.torque.assert_called_once_with(False)
        self.assertEqual(robot.cal_held_mask, 0)
        self.assertEqual(robot.host_mode, "normal")

    def test_failure_unloads_only_selected_joint_without_losing_prior_hold(self):
        robot = self.make_robot()
        robot.cal_held_mask = 3
        robot.servo.center_and_hold.side_effect = OSError("readback failed")
        robot._process_calibration_request(
            make_calibration_request(CAL_CENTER_ONE, index=1, seq=13)
        )
        robot.servo.write_byte.assert_called_once_with(11, ADDR_TORQUE, 0)
        robot.servo.torque.assert_not_called()
        self.assertEqual(robot.cal_held_mask, 1)
        robot.on_cal_reply.assert_called_once_with({"seq": 13, "index": 1, "ok": False})

    def test_io_ticks_do_not_overwrite_or_unload_a_calibrated_joint(self):
        robot = self.make_robot()
        robot.imu.read.return_value = (True, (0, 0, -9.8), (0, 0, 0))
        robot.servo.read_joint_states.return_value = {10: (2048, 0)}
        robot._process_calibration_request(
            make_calibration_request(CAL_CENTER_ONE, index=0, seq=15)
        )
        snapshots = []

        def collect(raw):
            snapshots.append(raw)
            if len(snapshots) == 3:
                robot._stop.set()

        robot.on_raw = collect
        robot._run_io_loop()
        self.assertEqual(len(snapshots), 3)
        self.assertTrue(all(raw["held_mask"] == 1 for raw in snapshots))
        robot.servo.torque.assert_not_called()
        robot.servo.write_joint_targets.assert_not_called()

    def test_center_is_rejected_outside_calibration(self):
        robot = self.make_robot()
        robot.host_mode = "normal"
        robot._process_calibration_request(
            make_calibration_request(CAL_CENTER_ONE, index=0, seq=14)
        )
        robot.servo.center_and_hold.assert_not_called()
        self.assertFalse(robot.on_cal_reply.call_args.args[0]["ok"])


if __name__ == "__main__":
    unittest.main()
