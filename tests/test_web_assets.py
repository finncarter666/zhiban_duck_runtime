"""校验静态工作台的资源、自包含部署和控制节点，不连接真实硬件。"""

from html.parser import HTMLParser
from pathlib import Path
import unittest
from urllib.parse import urlsplit

WEB = Path(__file__).resolve().parents[1] / "assets/web"
PAGE_CONTROLS = {
    "index.html": {
        "stop",
        "message",
        "rx",
        "rl",
        "servos",
        "imu",
        "mode",
        "pick",
        "roulade",
        "roller-mode",
        "roller-state",
        "recovery-phase",
        "active-model",
        "tilt-angle",
        "recovery-timer",
        "roller-warning",
        "joy-left",
        "touch-control",
        "touch-hint",
        "joy-right",
        "left-label",
        "right-label",
        "vxValue",
        "vyValue",
        "wzValue",
        "zero",
        "motion-limits",
        "head-controls",
        "action-profile",
        "action-scale",
        "action-scale-value",
        "leg-alpha",
        "leg-alpha-value",
        "head-alpha",
        "head-alpha-value",
        "gyro-alpha",
        "gyro-alpha-value",
        "gravity-tau",
        "gravity-tau-value",
        "acc-values",
        "gyro-values",
        "gravity-values",
        "joints",
        "command-health",
        "motion",
        "telemetry",
    },
    "calibrate.html": {
        "exit",
        "exit2",
        "start",
        "message",
        "mode",
        "joints",
        "cal-completed",
        "cal-progress",
    },
    "servo.html": {
        "exit",
        "message",
        "scan",
        "scan-stop",
        "scan-status",
        "scan-progress",
        "scan-results",
        "reset-mid",
        "kp",
        "kd",
        "new-id",
        "set-gains",
        "set-id",
        "read",
        "read-out",
        "enable-protection",
        "disable-protection",
        "protection-out",
        "protection-current",
        "set-protection-current",
        "protection-current-out",
        "selected-servo",
    },
}


class Page(HTMLParser):
    def __init__(self, path):
        super().__init__()
        self.nodes = []
        self.feed(path.read_text(encoding="utf-8"))

    def handle_starttag(self, tag, attrs):
        self.nodes.append((tag, dict(attrs)))

    @property
    def ids(self):
        return [attrs["id"] for _, attrs in self.nodes if "id" in attrs]


class WebAssetsTest(unittest.TestCase):
    def test_control_nodes_are_preserved_and_unique(self):
        for name, controls in PAGE_CONTROLS.items():
            page = Page(WEB / name)
            with self.subTest(page=name):
                self.assertEqual(len(page.ids), len(set(page.ids)))
                self.assertTrue(controls <= set(page.ids), controls - set(page.ids))
                self.assertIn("connection", page.ids)
                self.assertIn("connection-label", page.ids)
                self.assertIn(("html", {"lang": "zh-CN"}), page.nodes)

    def test_page_resources_and_navigation_resolve_locally(self):
        for name in PAGE_CONTROLS:
            page = Page(WEB / name)
            for tag, attrs in page.nodes:
                for key in ("src", "href"):
                    if key not in attrs:
                        continue
                    with self.subTest(page=name, resource=attrs[key]):
                        url = urlsplit(attrs[key])
                        self.assertFalse(url.scheme, "页面资源不得依赖外部服务")
                        self.assertFalse(url.netloc)
                        relative = url.path.lstrip("/")
                        target = WEB / (
                            relative or ("index.html" if url.path else name)
                        )
                        self.assertTrue(target.is_file(), target)
                        if tag == "a" and url.fragment:
                            self.assertIn(url.fragment, Page(target).ids)

    def test_shared_presentation_loads_before_page_controller(self):
        for name, controller in (
            ("index.html", "app.js"),
            ("calibrate.html", "calibrate.js"),
            ("servo.html", "servo.js"),
        ):
            page = Page(WEB / name)
            scripts = [
                urlsplit(attrs["src"]).path
                for tag, attrs in page.nodes
                if tag == "script" and "src" in attrs
            ]
            with self.subTest(page=name):
                self.assertEqual(scripts, ["studio.js", controller])
        shared = (WEB / "studio.js").read_text()
        self.assertNotIn("fetch(", shared, "共享展示层不得增加控制请求或轮询")
        self.assertNotIn("sendBeacon(", shared)

    def test_settings_name_and_legible_font_floor(self):
        import re

        for name in PAGE_CONTROLS:
            content = (WEB / name).read_text()
            self.assertNotIn("舵机中位设置", content)
            self.assertIn("舵机设置", content)
        for name in ("style.css", "servo.css"):
            content = (WEB / name).read_text()
            sizes = re.findall(r"font-size:\s*(\d+)px", content)
            self.assertTrue(sizes)
            self.assertTrue(all(int(size) >= 14 for size in sizes), name)

    def test_unavailable_telemetry_is_not_added(self):
        for name in PAGE_CONTROLS:
            page = Page(WEB / name)
            with self.subTest(page=name):
                self.assertFalse(
                    {"battery", "temperature", "camera", "battery-level", "cpu-temp"}
                    & set(page.ids)
                )
        overview = (WEB / "index.html").read_text()
        self.assertIn("结构示意 · 非实时姿态", overview)
        for heading in ("电池状态", "电池电量", "温度监测", "摄像头"):
            self.assertNotIn(heading, overview)


if __name__ == "__main__":
    unittest.main()
