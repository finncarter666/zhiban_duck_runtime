"""检查发布文档、平台展示与统一依赖，不访问网络或设备。"""

import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


class ReleaseMaterialsTest(unittest.TestCase):
    def test_single_dependency_file_preserves_runtime_and_test_packages(self):
        requirements = (ROOT / "requirements.txt").read_text()
        self.assertFalse((ROOT / "requirements-dev.txt").exists())
        lines = [
            line.strip()
            for line in requirements.splitlines()
            if line.strip() and not line.startswith("#")
        ]
        self.assertFalse(any(line.startswith("-r") for line in lines))
        self.assertEqual(len(lines), len(set(lines)))
        self.assertEqual(
            {line.split("==")[0] for line in lines},
            {
                "fastapi",
                "numpy",
                "onnxruntime",
                "pyserial",
                "smbus2",
                "uvicorn",
                "black",
                "httpx",
                "onnx",
            },
        )
        self.assertIn('numpy==1.24.4; python_version < "3.10"', lines)
        self.assertIn('numpy==2.2.6; python_version >= "3.10"', lines)
        self.assertIn('onnxruntime==1.18.0; python_version < "3.10"', lines)
        self.assertIn('onnxruntime==1.22.1; python_version >= "3.10"', lines)

    def test_release_documents_have_no_retired_references_or_progress_reports(self):
        retired = (
            "CONTRIBUTING.md",
            "THIRD_PARTY_NOTICES.md",
            "requirements-dev.txt",
            "docs/migration.md",
        )
        for name in retired:
            self.assertFalse((ROOT / name).exists())
        for path in [ROOT / "README.md", *ROOT.joinpath("docs").glob("*.md")]:
            text = path.read_text()
            with self.subTest(path=path.name):
                for name in retired:
                    self.assertNotIn(name, text)
                for phrase in (
                    "本轮",
                    "本次",
                    "第一轮",
                    "重构前",
                    "此前",
                    "不在本次范围",
                ):
                    self.assertNotIn(phrase, text)
                for reference in re.findall(r"`([\w./-]+\.md)`", text):
                    self.assertTrue(
                        (path.parent / reference).is_file()
                        or (ROOT / reference).is_file(),
                        reference,
                    )

    def test_board_specific_text_is_confined_to_hardware_guide(self):
        paths = [
            ROOT / "README.md",
            *ROOT.joinpath("docs").glob("*.md"),
            *ROOT.joinpath("assets/web").glob("*.html"),
            *ROOT.joinpath("assets/web").glob("*.js"),
            *ROOT.joinpath("zhiban_runtime").rglob("*.py"),
        ]
        for path in paths:
            if path == ROOT / "docs/hardware.md":
                continue
            with self.subTest(path=str(path.relative_to(ROOT))):
                self.assertNotRegex(path.read_text(), r"(?i)\brdk\b|\bD-Robotics\b")
        hardware = (ROOT / "docs/hardware.md").read_text()
        for platform in ("通用 Linux PC", "Linux 单板计算机", "D-Robotics RDK X3"):
            self.assertIn(platform, hardware)


if __name__ == "__main__":
    unittest.main()
