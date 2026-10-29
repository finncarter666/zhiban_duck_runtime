"""Exercise HTTP wiring and both CLI entry points without opening device files."""

import importlib.util
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
HAS_WEB_DEPS = all(
    importlib.util.find_spec(name)
    for name in ("fastapi", "uvicorn", "httpx", "onnxruntime")
)


@unittest.skipUnless(HAS_WEB_DEPS, "install requirements.txt for application tests")
class ApplicationTest(unittest.TestCase):
    def test_pages_api_and_hardware_lifecycle(self):
        from fastapi.testclient import TestClient
        from zhiban_runtime.application import create_app
        from zhiban_runtime.hardware.device_runtime import HardwareConfig

        with patch("zhiban_runtime.application.RobotHardware") as hardware, patch(
            "zhiban_runtime.application.MotionController.run"
        ):
            from zhiban_runtime.hardware.loop_timing import LoopTiming

            hardware.return_value.timing = LoopTiming(0.01)
            app = create_app(HardwareConfig())
            self.assertEqual(app.title, "Zhiban Duck Linux Runtime")
            hardware.return_value.set_callbacks.assert_called_once()
            with TestClient(app) as client:
                for page in ("/", "/calibrate.html", "/servo.html"):
                    with self.subTest(page=page):
                        response = client.get(page)
                        self.assertEqual(response.status_code, 200)
                        self.assertIn("Zhiban Duck", response.text)
                for asset in (
                    "app.js",
                    "calibrate.js",
                    "servo.js",
                    "style.css",
                    "servo.css",
                    "studio.js",
                    "duck-mark.svg",
                    "duck-illustration.svg",
                ):
                    self.assertEqual(client.get("/" + asset).status_code, 200)
                status = client.get("/api/status")
                self.assertEqual(status.status_code, 200)
                self.assertEqual(status.json()["mode"], "shadow")
                self.assertEqual(status.json()["platform"], "Linux")
                timing = status.json()["hardware_timing"]
                self.assertEqual(timing["target_hz"], 100)
                self.assertIsNone(timing["measured_hz"])
                self.assertTrue(timing["stale"])
                self.assertEqual(status.json()["deployment"]["control_hz"], 50)
                self.assertEqual(
                    client.post("/api/mode", json={"mode": "invalid"}).status_code, 400
                )
                self.assertEqual(
                    client.post("/api/mode", json={"mode": "hold"}).status_code, 400
                )
                self.assertEqual(
                    client.post("/api/command", json={"twist": [0, 0, 0]}).status_code,
                    200,
                )
                result = client.post("/api/filter", json={"leg": 0.4, "head": 0.3})
                self.assertEqual(result.json(), {"leg": 0.4, "head": 0.3})
                self.assertEqual(
                    client.post("/api/roller", json={"enabled": True}).status_code, 200
                )
                self.assertEqual(
                    client.get("/api/status").json()["roller"]["model"],
                    "zhiban_roller.onnx",
                )
            hardware.return_value.start.assert_called_once()
            hardware.return_value.stop.assert_called_once()

    def test_cli_entry_points_work_outside_checkout(self):
        environment = dict(os.environ)
        environment["PYTHONPATH"] = (
            str(ROOT) + os.pathsep + environment.get("PYTHONPATH", "")
        )
        with tempfile.TemporaryDirectory() as directory:
            for arguments in (
                [str(ROOT / "main.py"), "--help"],
                ["-m", "zhiban_runtime", "--help"],
            ):
                with self.subTest(arguments=arguments):
                    result = subprocess.run(
                        [sys.executable, *arguments],
                        cwd=directory,
                        env=environment,
                        capture_output=True,
                        text=True,
                        timeout=30,
                    )
                    self.assertEqual(result.returncode, 0, result.stderr)
                    self.assertIn("--servo-device", result.stdout)
                    self.assertIn("Zhiban Duck Linux runtime", result.stdout)

    def test_cli_defaults_are_unchanged(self):
        from zhiban_runtime.cli import parse_args

        with patch.object(sys, "argv", ["main.py"]):
            args = parse_args()
        self.assertEqual(
            vars(args),
            {
                "servo_device": "/dev/ttyUSB0",
                "servo_baud": 1_000_000,
                "servo_rs485": False,
                "i2c_bus": 1,
                "imu_address": None,
                "host": "0.0.0.0",
                "port": 9527,
            },
        )


if __name__ == "__main__":
    unittest.main()
