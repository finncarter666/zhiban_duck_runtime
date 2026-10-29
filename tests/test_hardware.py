import sys
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

try:
    from zhiban_runtime.hardware.device_runtime import (
        RobotHardware,
        _encode_servo_packet,
        _monotonic_us,
    )
    from zhiban_runtime.messages import ARM, DISARM, POLICY, command
except ModuleNotFoundError:
    RobotHardware = None


class FakeServo:
    def __init__(self):
        self.torque_calls = []
        self.goals = []
        self.closed = False

    def torque(self, value):
        self.torque_calls.append(bool(value))

    def apply_motion_gains(self):
        pass

    def apply_calibration_gains(self):
        pass

    def write_joint_targets(self, values):
        self.goals.append(tuple(values))

    def read_joint_states(self):
        return {
            sid: (2047, 0)
            for sid in (10, 11, 12, 13, 14, 20, 21, 22, 23, 24, 30, 31, 32, 33, 34)
        }

    def close(self):
        self.closed = True


class FakeImu:
    def read(self):
        return True, (0.0, 0.0, -9.8), (0.0, 0.0, 0.0)

    def close(self):
        pass


@unittest.skipUnless(
    RobotHardware, "runtime dependencies are installed by requirements.txt"
)
class HardwareSafetyTest(unittest.TestCase):
    def test_scs_packet_checksum(self):
        pkt = _encode_servo_packet(10, 3, bytes((40, 1)))
        self.assertEqual(pkt[:2], b"\xff\xff")
        self.assertEqual((sum(pkt[2:-1]) + pkt[-1]) & 255, 255)

    def test_watchdog_and_explicit_rearm(self):
        servo = FakeServo()
        robot = RobotHardware(servo=servo, imu=FakeImu())
        feedback = []
        robot.set_callbacks(
            feedback=feedback.append,
            raw=lambda _x: None,
            cal_reply=lambda _x: None,
            servo_reply=lambda _x: None,
        )
        robot.start()
        try:
            targets = [float(i) for i in range(15)]
            robot.submit_command(
                command(1, _monotonic_us(), ARM, targets, ttl_us=30_000)
            )
            time.sleep(0.04)
            self.assertTrue(any(servo.torque_calls))
            self.assertTrue(servo.goals)
            time.sleep(0.05)
            self.assertFalse(feedback[-1].enabled)
            self.assertTrue(feedback[-1].flags & 4)
            robot.submit_command(command(2, _monotonic_us(), POLICY, targets))
            time.sleep(0.03)
            self.assertFalse(
                feedback[-1].enabled, "policy command must not re-arm after timeout"
            )
            robot.submit_command(command(3, _monotonic_us(), ARM, targets))
            time.sleep(0.03)
            self.assertTrue(feedback[-1].enabled)
            robot.submit_command(command(4, 0, DISARM, targets))
            time.sleep(0.03)
            self.assertFalse(feedback[-1].enabled)
        finally:
            robot.stop()


if __name__ == "__main__":
    unittest.main()
