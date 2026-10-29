"""Observation layout, filter behavior and pre-reorganization ONNX outputs."""

import hashlib
import importlib.util
import json
import unittest
from pathlib import Path

import numpy as np

from zhiban_runtime.control.feedback_processing import FeedbackFilter
from zhiban_runtime.control.posture_recovery import RecoveryStateMachine
from zhiban_runtime.inference.policy_math import (
    DEFAULT_DEG,
    HOME_DEG,
    POLICY_INDEX,
    PolicyState,
    action_to_joint_degrees,
    build_observation,
)
from zhiban_runtime.inference.onnx_policy import OnnxPolicy
from zhiban_runtime.paths import MODEL_DIR

ROOT = Path(__file__).resolve().parents[1]


class NumericalContractTest(unittest.TestCase):
    def test_observation_layout_and_output_buffer(self):
        state = PolicyState(
            [0] * 3, [0] * 3, list(DEFAULT_DEG), list(range(15)), [0, 0, -1], [1, 2, 3]
        )
        previous = np.arange(14, dtype=np.float32)
        buffer = np.empty(61, dtype=np.float32)
        result = build_observation(
            state, previous, [4, 5, 6], [7, 8, 9, 10], out=buffer
        )
        self.assertIs(result, buffer)
        np.testing.assert_array_equal(result[:6], [1, 2, 3, 0, 0, -1])
        np.testing.assert_array_equal(result[6:20], np.zeros(14))
        np.testing.assert_allclose(
            result[20:34], np.deg2rad(np.arange(15, dtype=np.float32)[POLICY_INDEX])
        )
        np.testing.assert_array_equal(result[34:48], previous)
        np.testing.assert_array_equal(result[48:55], [4, 5, 6, 7, 8, 9, 10])
        np.testing.assert_array_equal(result[55:], np.zeros(6))

    def test_action_mapping_and_mouth_clipping(self):
        for mouth, expected in ((-1, 0), (12, 12), (100, 30)):
            targets = action_to_joint_degrees(np.zeros(14, dtype=np.float32), mouth)
            np.testing.assert_array_equal(np.asarray(targets)[POLICY_INDEX], HOME_DEG)
            self.assertEqual(targets[-1], expected)

    def test_action_slices_and_raw_previous_action(self):
        class ConstantSession:
            def run(self, outputs, feed):
                return [np.ones((1, 14), dtype=np.float32)]

        policy = OnnxPolicy()
        policy.session = ConstantSession()
        policy._outputs = ["actions"]
        policy._feed = {"observations": policy._obs.reshape(1, 61)}
        output = policy.infer_action([0, 0, -1], [0] * 3, DEFAULT_DEG, [0] * 15)
        np.testing.assert_allclose(output, [0.55] * 5 + [0.8] * 5 + [0.55] * 4)
        np.testing.assert_array_equal(policy.prev_action, np.ones(14))
        before = (policy.action_alpha, policy.action_alpha_head)
        with self.assertRaises(ValueError):
            policy.configure_action_smoothing(leg=0.1, head=2)
        self.assertEqual((policy.action_alpha, policy.action_alpha_head), before)
        policy.reset_action_history()
        np.testing.assert_array_equal(policy.prev_action, np.zeros(14))
        np.testing.assert_array_equal(policy._act_filt, np.zeros(14))

    def test_invalid_feedback_does_not_advance_fusion(self):
        signals = FeedbackFilter()
        args = ([0, 0, -9.8], [0] * 3, DEFAULT_DEG, [0] * 15)
        first = signals.update(*args, timestamp=1.0)
        self.assertTrue(first["imu_fusion_updated"])
        duplicate = signals.update(*args, timestamp=1.0)
        self.assertFalse(duplicate["imu_fusion_updated"])
        self.assertEqual(duplicate["grav"], first["grav"])
        self.assertFalse(
            signals.update(*args, timestamp=1.01, imu_ok=False)["imu_fusion_updated"]
        )
        self.assertTrue(signals.update(*args, timestamp=1.02)["imu_fusion_updated"])

    def test_recovery_thresholds_and_wall_clock_hold(self):
        recovery = RecoveryStateMachine()
        for index in range(7):
            self.assertFalse(recovery.update([1, 0, 0], index * 0.02))
        self.assertTrue(recovery.update([1, 0, 0], 0.14))
        self.assertEqual(recovery.phase, "default_pose")
        self.assertFalse(recovery.update([0, 0, -1], 1.13))
        self.assertTrue(recovery.update([0, 0, -1], 1.15))
        self.assertEqual(recovery.phase, "getup")
        for index in range(49):
            self.assertFalse(recovery.update([0, 0, -1], 1.17 + index * 0.02))
        self.assertTrue(recovery.update([0, 0, -1], 2.15))
        self.assertEqual(recovery.phase, "walk")
        self.assertEqual(recovery.transitions, 3)


@unittest.skipUnless(
    importlib.util.find_spec("onnxruntime"),
    "install runtime dependencies for ONNX tests",
)
class ModelInferenceTest(unittest.TestCase):
    def test_all_models_match_original_cpu_outputs(self):
        golden = json.loads(
            (ROOT / "tests/fixtures/inference_outputs.json").read_text()
        )
        for name, expected in golden["outputs"].items():
            with self.subTest(model=name):
                policy = OnnxPolicy()
                ok, message = policy.load(
                    name, use_model_home=name == "zhiban_roller.onnx"
                )
                self.assertTrue(ok, message)
                for observation, reference in zip(golden["observations"], expected):
                    policy._obs[:] = observation
                    actual = policy.session.run(policy._outputs, policy._feed)[0][0]
                    np.testing.assert_allclose(actual, reference, atol=1e-5, rtol=0)


@unittest.skipUnless(
    importlib.util.find_spec("onnx"),
    "install development dependencies for graph validation",
)
class ModelPayloadTest(unittest.TestCase):
    def test_only_descriptive_model_metadata_changed(self):
        import onnx

        manifest = json.loads((MODEL_DIR / "manifest.json").read_text())["models"]
        for name, entry in manifest.items():
            with self.subTest(model=name):
                model = onnx.load(str(MODEL_DIR / name))
                onnx.checker.check_model(model)
                keep = [
                    item
                    for item in model.metadata_props
                    if item.key not in ("run_path", "task_id")
                ]
                del model.metadata_props[:]
                model.metadata_props.extend(keep)
                actual = hashlib.sha256(model.SerializeToString()).hexdigest()
                self.assertEqual(actual, entry["inference_payload_sha256"])


if __name__ == "__main__":
    unittest.main()
