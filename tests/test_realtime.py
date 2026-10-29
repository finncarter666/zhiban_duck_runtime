"""可重复的虚拟时钟测试，不用主机 sleep 精度冒充目标板实时性。"""

import unittest
from unittest.mock import patch

from zhiban_runtime.hardware import device_runtime as runtime
from zhiban_runtime.hardware.loop_timing import LoopTiming
from zhiban_runtime.hardware.serial_frames import ReplyBuffer
from zhiban_runtime.inference.policy_math import MOTOR_IDS
from test_hardware import FakeImu, FakeServo


def reply(sid, error=0):
    return runtime._encode_servo_packet(sid, error, b"\x00\x08\x01\x80\x00\x00")


class Clock:
    def __init__(self):
        self.value = 10.0

    def now(self):
        return self.value

    def advance(self, seconds):
        self.value += seconds


class ScheduledSerial:
    def __init__(self, clock, arrivals):
        self.clock = clock
        self.arrivals = [(clock.now() + delay, data) for delay, data in arrivals]
        self.waits = 0

    @property
    def in_waiting(self):
        return len(self.arrivals[0][1]) if self.arrivals else 0

    def read(self, _count):
        if self.arrivals and self.arrivals[0][0] <= self.clock.now():
            return self.arrivals.pop(0)[1]
        return b""

    def fileno(self):
        return 42

    def wait_readable(self, read, write, error, timeout):
        self.waits += 1
        delay = timeout
        if self.arrivals:
            delay = min(delay, max(0, self.arrivals[0][0] - self.clock.now()))
        self.clock.advance(delay)
        return read, write, error

    def reset_input_buffer(self):
        pass

    def write(self, data):
        self.request = data

    def flush(self):
        pass


class SerialDeadlineTest(unittest.TestCase):
    def read_states(self, arrivals):
        clock = Clock()
        bus = runtime.SCSBus.__new__(runtime.SCSBus)
        bus.serial = ScheduledSerial(clock, arrivals)
        with patch.object(runtime.time, "monotonic", clock.now), patch.object(
            runtime.select, "select", bus.serial.wait_readable
        ):
            states = bus.read_joint_states()
        return states, clock.now() - 10.0, bus.serial

    def test_all_replies_return_without_waiting_for_six_ms(self):
        data = b"".join(reply(sid) for sid in MOTOR_IDS)
        states, elapsed, serial = self.read_states([(0.0024, data)])
        self.assertEqual(set(states), set(MOTOR_IDS))
        self.assertEqual(states[10], (2048, -1))
        self.assertAlmostEqual(elapsed, 0.0024)
        self.assertEqual(serial.waits, 1)
        self.assertEqual(serial.request[4], 0x82)

    def test_fragmented_and_duplicate_replies_do_not_finish_early(self):
        head = reply(10)
        rest = b"".join(reply(sid) for sid in MOTOR_IDS[1:])
        states, elapsed, _ = self.read_states(
            [(0.001, head[:1]), (0.002, head[1:] + head), (0.004, rest)]
        )
        self.assertEqual(len(states), 15)
        self.assertAlmostEqual(elapsed, 0.004)

    def test_missing_joint_retains_timeout_and_partial_feedback(self):
        data = b"".join(reply(sid) for sid in MOTOR_IDS[:-1])
        states, elapsed, _ = self.read_states([(0.002, data + reply(99) + reply(10))])
        self.assertEqual(set(states), set(MOTOR_IDS[:-1]))
        self.assertAlmostEqual(elapsed, runtime.REPLY_S)

    def test_error_and_corrupted_frames_do_not_count_as_feedback(self):
        data = b"".join(reply(sid) for sid in MOTOR_IDS[:-1])
        bad = bytearray(reply(MOTOR_IDS[-1]))
        bad[-1] ^= 1
        states, elapsed, _ = self.read_states(
            [(0.001, data + bytes(bad) + reply(MOTOR_IDS[-1], error=1))]
        )
        self.assertEqual(len(states), 14)
        self.assertAlmostEqual(elapsed, runtime.REPLY_S)

    def test_no_replies_has_bounded_wait_without_polling(self):
        states, elapsed, serial = self.read_states([])
        self.assertEqual(states, {})
        self.assertAlmostEqual(elapsed, runtime.REPLY_S)
        self.assertEqual(serial.waits, 1)

    def test_parser_resynchronizes_and_keeps_split_header(self):
        parser = ReplyBuffer(expected_length=8)
        self.assertEqual(parser.feed(b"garbage\xff"), [])
        self.assertEqual(parser.feed(reply(10)[1:]), [reply(10)])
        bad_length = b"\xff\xff\x0a\xff"
        bad_checksum = reply(11)[:-1] + b"\x00"
        self.assertEqual(
            parser.feed(bad_length + bad_checksum + reply(12)), [reply(12)]
        )
        self.assertEqual(parser.pending, b"")

    def test_maintenance_collection_still_uses_full_deadline(self):
        clock = Clock()
        bus = runtime.SCSBus.__new__(runtime.SCSBus)
        bus.serial = ScheduledSerial(clock, [(0.001, reply(10))])
        with patch.object(runtime.time, "monotonic", clock.now), patch.object(
            runtime.select, "select", bus.serial.wait_readable
        ):
            frames = bus._collect_reply_frames(clock.now() + runtime.REPLY_S)
        self.assertEqual(frames, [reply(10)])
        self.assertAlmostEqual(clock.now() - 10.0, runtime.REPLY_S)


class LoopTimingTest(unittest.TestCase):
    def test_empty_window_does_not_claim_one_hundred_hz(self):
        data = LoopTiming(0.01).snapshot()
        self.assertEqual(data["target_hz"], 100)
        self.assertIsNone(data["measured_hz"])
        self.assertTrue(data["stale"])

    def test_bounded_window_and_stage_statistics(self):
        timing = LoopTiming(0.01, capacity=3)
        for index in range(5):
            timing.record(
                10 + index * 0.01,
                0.01,
                0.001,
                "normal",
                (0, 0.001, 0.001, 0.002, 0.001),
                2,
            )
        with patch(
            "zhiban_runtime.hardware.loop_timing.time.monotonic", return_value=10.05
        ):
            data = timing.snapshot()
        self.assertEqual(data["cycles_total"], 5)
        self.assertEqual(data["window_cycles"], 3)
        self.assertAlmostEqual(data["measured_hz"], 100)
        self.assertEqual(data["stages_ms"]["servo_read"]["p99"], 2)
        self.assertAlmostEqual(data["work_ms"]["max"], 5)
        self.assertEqual(data["missed_deadlines_total"], 2)
        self.assertFalse(data["stale"])

    def simulate_loop(self, read_seconds, publish_seconds=0):
        clock = Clock()
        servo = FakeServo()
        original_read = servo.read_joint_states

        def read():
            clock.advance(read_seconds)
            return original_read()

        servo.read_joint_states = read
        robot = runtime.RobotHardware(servo=servo, imu=FakeImu())
        feedback = []

        def publish(fb):
            clock.advance(publish_seconds)
            feedback.append(fb)

        class StopAfterCycles:
            def is_set(self):
                return len(feedback) >= 101

            def wait(self, timeout):
                clock.advance(timeout)

        robot._stop = StopAfterCycles()
        robot.on_feedback = publish
        with patch.object(runtime.time, "monotonic", clock.now), patch.object(
            runtime, "_monotonic_us", lambda: int(clock.now() * 1e6)
        ):
            robot._run_io_loop()
            stats = robot.timing.snapshot()
        return feedback, stats

    def test_absolute_schedule_keeps_one_hundred_hz_with_budget_remaining(self):
        feedback, stats = self.simulate_loop(0.0024, 0.001)
        self.assertEqual(len(feedback), 101)
        self.assertAlmostEqual(stats["measured_hz"], 100)
        self.assertEqual(stats["missed_deadlines_total"], 0)
        self.assertAlmostEqual(stats["stages_ms"]["feedback_publish"]["mean"], 1)

    def test_slow_bus_reports_missed_cycles_not_fake_one_hundred_hz(self):
        _, stats = self.simulate_loop(0.012)
        self.assertAlmostEqual(stats["measured_hz"], 50)
        self.assertGreater(stats["missed_deadlines_total"], 0)
        self.assertAlmostEqual(stats["stages_ms"]["servo_read"]["max"], 12)

    def test_callback_cost_is_included_in_work_budget(self):
        _, stats = self.simulate_loop(0.004, 0.007)
        self.assertAlmostEqual(stats["work_ms"]["max"], 11)
        self.assertGreater(stats["missed_deadlines_total"], 0)
        self.assertLess(stats["measured_hz"], 100)


class AcceptanceSampleTest(unittest.TestCase):
    def test_healthy_sample_and_failed_frequency(self):
        from tools.check_control_rate import check_sample

        status = {
            "mode": "policy",
            "servo_ids": MOTOR_IDS,
            "imu_ok": True,
            "hardware_timing": {
                "stale": False,
                "measured_hz": 100,
                "window_span_s": 4.99,
                "maintenance_cycles": 0,
            },
        }
        self.assertEqual(check_sample(status, 98), [])
        status["hardware_timing"]["measured_hz"] = 83
        self.assertTrue(check_sample(status, 98))

    def test_missing_data_fails_closed(self):
        from tools.check_control_rate import check_sample

        self.assertTrue(check_sample({}, 98))

    def test_maintenance_and_stale_samples_do_not_pass(self):
        from tools.check_control_rate import check_sample

        status = {
            "mode": "calibrate",
            "servo_ids": MOTOR_IDS,
            "imu_ok": True,
            "hardware_timing": {
                "stale": True,
                "measured_hz": 100,
                "window_span_s": 4.99,
                "maintenance_cycles": 3,
            },
        }
        self.assertEqual(len(check_sample(status, 98)), 3)


if __name__ == "__main__":
    unittest.main()
