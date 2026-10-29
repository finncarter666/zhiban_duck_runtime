"""Message validation at the controller/hardware boundary."""

import unittest

from zhiban_runtime.messages import (
    ARM,
    CAL_CENTER_ONE,
    PROTECTION_CURRENT_UNIT_A,
    make_calibration_request,
    command,
    unsigned_counter_delta,
    make_current_limit_request,
)


class ProtocolTest(unittest.TestCase):
    def test_command_wraps_counters_and_freezes_targets(self):
        targets = [0.0] * 15
        result = command(2**32 + 3, 2**32 + 5, ARM, targets)
        targets[0] = 100
        self.assertEqual((result.seq, result.source_us, result.ttl_us), (3, 5, 250_000))
        self.assertEqual(result.targets, (0.0,) * 15)
        self.assertEqual(unsigned_counter_delta(2, 0xFFFFFFFE), 4)

    def test_invalid_targets_and_lifetimes_are_rejected(self):
        for targets in ([0] * 14, [float("nan")] * 15, [float("inf")] * 15):
            with self.subTest(targets=targets), self.assertRaises(ValueError):
                command(0, 0, ARM, targets)
        for ttl in (0, -1, 250_001, True, 1.5):
            with self.subTest(ttl=ttl), self.assertRaises(ValueError):
                command(0, 0, ARM, [0] * 15, ttl_us=ttl)

    def test_calibration_indices_and_current_units(self):
        for index in (-1, 15, True, 1.5):
            with self.subTest(index=index), self.assertRaises(ValueError):
                make_calibration_request(CAL_CENTER_ONE, index)
        request = make_current_limit_request(1.0)
        self.assertEqual(
            request.protection_current, round(1.0 / PROTECTION_CURRENT_UNIT_A)
        )


if __name__ == "__main__":
    unittest.main()
