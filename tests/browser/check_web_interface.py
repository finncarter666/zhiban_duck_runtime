"""Local mock HTTP + Chrome DevTools checks; never opens robot devices."""

import base64
import copy
import hashlib
import http.server
import json
import os
from pathlib import Path
import socket
import struct
import subprocess
import tempfile
import threading
import time
import urllib.request
from urllib.parse import urlsplit
from unittest.mock import MagicMock

ROOT = Path(__file__).resolve().parents[2]
import sys

sys.path.insert(0, str(ROOT))
from zhiban_runtime.control.motion_control import MotionController

state = MotionController(MagicMock()).status()
state.update(
    feedback_age_ms=3.6,
    feedback_hz=99.8,
    inference_hz=50.0,
    servo_ids=state["motors"],
    imu_ok=True,
    enabled=False,
    control_ready=True,
    full_robot_ready=True,
    angles=[0.0] * 15,
    targets=[0.0] * 15,
    velocities=[0.0] * 15,
    acc=[0.0, 0.0, -9.8],
    gyro=[0.0, 0.0, 0.0],
    gravity=[0.0, 0.0, -1.0],
    raw_pos=[2048] * 15,
    raw_mask=32767,
    missing_joint_ids=[],
    hardware_command_seq=102,
    hardware_command_age_us=2000,
    hardware_invalid_commands=0,
    hardware_servo_debug=False,
)
state["recovery"].update(angle_deg=0.5)
calls = []
fail_status = False


class Handler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *a, **k):
        super().__init__(*a, directory=str(ROOT / "assets/web"), **k)

    def log_message(self, *a):
        pass

    def reply(self, body, code=200):
        data = json.dumps(body).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.path == "/api/status":
            self.reply(
                state if not fail_status else {"detail": "测试：状态连接中断"},
                200 if not fail_status else 503,
            )
        else:
            super().do_GET()

    def do_POST(self):
        data = json.loads(
            self.rfile.read(int(self.headers.get("Content-Length", 0))) or "{}"
        )
        calls.append((self.path, data))
        result = {}
        if self.path == "/api/mode":
            state["mode"] = data["mode"]
            state["enabled"] = data["mode"] in ("hold", "policy")
        elif self.path == "/api/command":
            pass
        elif self.path == "/api/cal/enter":
            state.update(mode="calibrate", enabled=False, cal_done=[False] * 15)
        elif self.path in ("/api/cal/exit", "/api/servo/exit"):
            state.update(
                mode="shadow",
                enabled=False,
                hardware_servo_debug=False,
                cal_done=[False] * 15,
            )
        elif self.path == "/api/cal/center":
            state["cal_done"][data["index"]] = True
        elif self.path == "/api/servo/enter":
            state.update(mode="servo_debug", hardware_servo_debug=True)
        elif self.path == "/api/servo":
            action = data["action"]
            tid = data.get("target_id", 254)
            result = dict(ok=True, target_id=tid, raw_pos=2048, kp=5, kd=10)
            if action == "ping":
                result.update(op=7, ok=tid in [10, 20, 34])
            if action == "set_protection":
                result.update(
                    op=9, broadcast=True, protection_enabled=data["protection_enabled"]
                )
            if action == "set_protection_current":
                result.update(op=10, broadcast=True, current_a=data["current_a"])
        elif self.path == "/api/roller":
            state["roller"]["enabled"] = data["enabled"]
            result = dict(
                state["deployment"], enabled=data["enabled"], model="zhiban_roller.onnx"
            )
        elif self.path == "/api/filter":
            result = data
        self.reply(result)


class DevTools:
    def __init__(self, url):
        u = urlsplit(url)
        self.sock = socket.create_connection((u.hostname, u.port), timeout=15)
        self.buffer = b""
        self.next = 0
        self.errors = []
        key = base64.b64encode(os.urandom(16)).decode()
        self.sock.sendall(
            (
                f"GET {u.path} HTTP/1.1\r\nHost: {u.netloc}\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n"
            ).encode()
        )
        while b"\r\n\r\n" not in self.buffer:
            self.buffer += self.sock.recv(4096)
        head, self.buffer = self.buffer.split(b"\r\n\r\n", 1)
        assert head.startswith(b"HTTP/1.1 101 "), head

    def read(self, n):
        while len(self.buffer) < n:
            chunk = self.sock.recv(max(4096, n - len(self.buffer)))
            if not chunk:
                raise RuntimeError("Chrome WebSocket closed")
            self.buffer += chunk
        data, self.buffer = self.buffer[:n], self.buffer[n:]
        return data

    def send(self, payload):
        body = json.dumps(payload).encode()
        n = len(body)
        mask = os.urandom(4)
        header = (
            bytes([129, 128 + n])
            if n < 126
            else (
                bytes([129, 254]) + struct.pack("!H", n)
                if n < 65536
                else bytes([129, 255]) + struct.pack("!Q", n)
            )
        )
        self.sock.sendall(
            header + mask + bytes(b ^ mask[i % 4] for i, b in enumerate(body))
        )

    def recv(self):
        first, second = self.read(2)
        n = second & 127
        if n == 126:
            n = struct.unpack("!H", self.read(2))[0]
        if n == 127:
            n = struct.unpack("!Q", self.read(8))[0]
        mask = self.read(4) if second & 128 else None
        data = self.read(n)
        if mask:
            data = bytes(b ^ mask[i % 4] for i, b in enumerate(data))
        assert first & 15 == 1, first
        return json.loads(data)

    def call(self, method, **params):
        self.next += 1
        request_id = self.next
        self.send(dict(id=request_id, method=method, params=params))
        while True:
            item = self.recv()
            if item.get("method") == "Runtime.exceptionThrown":
                self.errors.append(item["params"])
            if item.get("id") == request_id:
                if "error" in item:
                    raise RuntimeError(item)
                return item.get("result", {})

    def js(self, expression):
        result = self.call(
            "Runtime.evaluate",
            expression=expression,
            returnByValue=True,
            awaitPromise=True,
        )
        if "exceptionDetails" in result:
            raise RuntimeError(result)
        return result.get("result", {}).get("value")

    def wait(self, expression, timeout=15):
        until = time.time() + timeout
        while time.time() < until:
            if self.js("Boolean(" + expression + ")"):
                return
            time.sleep(0.1)
        raise AssertionError(
            "Timed out: "
            + expression
            + "; page="
            + str(
                self.js(
                    "[location.href,document.getElementById('message')?.textContent,document.getElementById('scan-status')?.textContent]"
                )
            )
            + "; errors="
            + str(self.errors)
            + "; calls="
            + str(calls[-10:])
        )

    def screenshot(self, name):
        result = self.call(
            "Page.captureScreenshot", format="png", captureBeyondViewport=False
        )
        (output / name).write_bytes(base64.b64decode(result["data"]))


def check_readability(c):
    problems = c.js(
        r"""(() => {
      const rgb = value => (value.match(/[\d.]+/g) || []).map(Number);
      const luminance = color => color.slice(0,3).map(v=>v/255).reduce((sum,v,i)=>sum+[.2126,.7152,.0722][i]*(v<=.04045?v/12.92:((v+.055)/1.055)**2.4),0);
      return [...document.querySelectorAll('label,button,.subtitle,.hint,.badge,.metric-label,.hero-copy>p,.hero-bottom,th,td,.cal-meta span,.cal-actions>span,.servo-readout')].flatMap(e=>{
        if (!e.getClientRects().length || e.disabled) return [];
        const style=getComputedStyle(e), fg=rgb(style.color);
        let parent=e, bg;
        do { bg=rgb(getComputedStyle(parent).backgroundColor); parent=parent.parentElement; } while(parent && (bg.length===4 && bg[3]===0));
        if (bg.length===4 && bg[3]===0) bg=[255,255,255];
        const a=luminance(fg),b=luminance(bg),ratio=(Math.max(a,b)+.05)/(Math.min(a,b)+.05);
        return parseFloat(style.fontSize)<14 || ratio<4.5 ? [{text:e.textContent.slice(0,30),font:style.fontSize,ratio}] : [];
      });
    })()"""
    )
    assert not problems, problems


def check_widths(c):
    for width in (320, 390, 768, 1024, 1440):
        c.call(
            "Emulation.setDeviceMetricsOverride",
            width=width,
            height=844,
            deviceScaleFactor=1,
            mobile=width < 680,
        )
        assert c.js("document.documentElement.scrollWidth<=innerWidth"), (
            width,
            c.js("document.documentElement.scrollWidth"),
        )
        check_readability(c)
    c.call(
        "Emulation.setDeviceMetricsOverride",
        width=390,
        height=844,
        deviceScaleFactor=1,
        mobile=True,
    )


output = Path("/tmp/zhiban-web-preview")
output.mkdir(exist_ok=True)
server = http.server.ThreadingHTTPServer(("127.0.0.1", 0), Handler)
threading.Thread(target=server.serve_forever, daemon=True).start()
base = f"http://127.0.0.1:{server.server_port}"
with tempfile.TemporaryDirectory(prefix="zhiban-chrome-") as profile:
    log = open(output / "chrome.log", "w")
    chrome = subprocess.Popen(
        [
            os.environ.get("CHROME_BINARY", "/usr/bin/google-chrome"),
            "--headless=new",
            "--no-sandbox",
            "--disable-gpu",
            "--disable-dev-shm-usage",
            "--no-first-run",
            "--no-default-browser-check",
            "--remote-debugging-port=0",
            f"--user-data-dir={profile}",
            "about:blank",
        ],
        stdout=log,
        stderr=log,
    )
    try:
        port_file = Path(profile) / "DevToolsActivePort"
        for _ in range(100):
            if port_file.exists():
                break
            time.sleep(0.1)
        port = int(port_file.read_text().splitlines()[0])
        tabs = json.load(urllib.request.urlopen(f"http://127.0.0.1:{port}/json/list"))
        c = DevTools(
            next(t["webSocketDebuggerUrl"] for t in tabs if t["type"] == "page")
        )
        c.call("Runtime.enable")
        c.call("Page.enable")
        c.call(
            "Emulation.setDeviceMetricsOverride",
            width=1440,
            height=1080,
            deviceScaleFactor=1,
            mobile=False,
        )
        c.call("Page.navigate", url=base + "/")
        c.wait("document.getElementById('rx')?.textContent === '99.8 Hz'")
        assert (
            c.js("document.getElementById('connection-label').textContent")
            == "设备已连接"
        )
        assert c.js("document.querySelectorAll('#joints tr').length") == 15
        c.js("window.firstJoint=document.querySelector('#joints tr')")
        measure = "JSON.stringify(['mode','message','servo-summary','command-health','motion','telemetry'].map(id=>{let r=document.getElementById(id).getBoundingClientRect();return [r.x,r.y,r.width,r.height]}))"
        bounds = c.js(measure)
        state.update(
            last_error="测试诊断文本：" + ("很长的通信诊断信息 / " * 40),
            missing_joint_ids=state["motors"],
            mode="policy",
            enabled=True,
        )
        c.wait(
            "document.getElementById('message').textContent.startsWith('测试诊断文本')"
        )
        assert c.js(measure) == bounds, "Live labels changed layout dimensions"
        assert c.js(
            "window.firstJoint===document.querySelector('#joints tr')"
        ), "Polling replaced a joint row"
        state.update(last_error="", missing_joint_ids=[], mode="shadow", enabled=False)
        c.wait("document.getElementById('message').textContent.startsWith('Linux')")
        assert c.js("document.documentElement.scrollWidth<=innerWidth")
        c.screenshot("overview-desktop.png")
        c.js("document.querySelector('[data-mode=policy]').click()")
        c.wait("document.getElementById('mode').textContent.includes('扭矩已使能')")
        c.call("Input.dispatchKeyEvent", type="keyDown", key="w", code="KeyW")
        time.sleep(0.2)
        assert any(p == "/api/command" and d["twist"][0] > 0 for p, d in calls)
        c.call("Input.dispatchKeyEvent", type="keyUp", key="w", code="KeyW")
        c.call("Input.dispatchKeyEvent", type="keyDown", key=" ", code="Space")
        c.wait("document.getElementById('mode').textContent.includes('已卸力')")
        assert ("/api/mode", {"mode": "off"}) in calls
        c.js(
            "document.getElementById('head0').value='0.2';document.getElementById('head0').dispatchEvent(new Event('input'))"
        )
        time.sleep(0.1)
        assert c.js("document.getElementById('head0-value').textContent") == "0.20 rad"
        assert any(p == "/api/command" and d["head"][0] == 0.2 for p, d in calls)
        # Status failures must not leave an apparently live dashboard.
        fail_status = True
        c.wait("document.getElementById('connection').dataset.state==='offline'")
        assert c.js("document.getElementById('rx').textContent") == "—"
        assert c.js(
            "window.firstJoint===document.querySelector('#joints tr')"
        ), "Disconnect replaced a joint row"
        assert c.js("document.querySelector('#joints tr').cells[1].textContent") == "—"
        assert c.js("document.querySelector('[data-mode=policy]').disabled")
        fail_status = False
        c.wait("document.getElementById('rx').textContent==='99.8 Hz'")
        c.call(
            "Emulation.setDeviceMetricsOverride",
            width=390,
            height=844,
            deviceScaleFactor=1,
            mobile=True,
        )
        assert c.js("document.documentElement.scrollWidth<=innerWidth"), c.js(
            "document.documentElement.scrollWidth"
        )
        c.js("scrollTo(0,document.body.scrollHeight)")
        time.sleep(0.3)
        assert c.js(
            "(()=>{const r=document.getElementById('stop').getBoundingClientRect();return r.top>=0 && r.bottom<=innerHeight})()"
        ), "Unload button must remain visible while scrolling"
        c.call("Emulation.setTouchEmulationEnabled", enabled=True, maxTouchPoints=2)

        def gesture(selector, dx, dy):
            c.js(
                f"document.querySelector({json.dumps(selector)}).scrollIntoView({{block:'center',behavior:'instant'}})"
            )
            time.sleep(0.4)
            point = c.js(
                f"(()=>{{let r=document.querySelector({json.dumps(selector)}).getBoundingClientRect();return [r.x+r.width/2,r.y+r.height/2]}})()"
            )
            x, y = point
            start_scroll = c.js("scrollY")
            c.call(
                "Input.dispatchTouchEvent",
                type="touchStart",
                touchPoints=[dict(x=x, y=y, id=1)],
            )
            for fraction in (0.15, 0.3, 0.5, 0.75, 1):
                c.call(
                    "Input.dispatchTouchEvent",
                    type="touchMove",
                    touchPoints=[dict(x=x + dx * fraction, y=y + dy * fraction, id=1)],
                )
                time.sleep(0.06)
            c.call("Input.dispatchTouchEvent", type="touchEnd", touchPoints=[])
            time.sleep(0.6)
            return c.js("scrollY") - start_scroll

        calls.clear()
        assert (
            abs(gesture("#joy-left", 0, -100)) > 20
        ), "Locked joystick blocked page scrolling"
        assert all(
            not any(d["twist"]) for p, d in calls if p == "/api/command"
        ), "Scrolling sent a motion command"
        original = c.js("document.getElementById('head0').value")
        calls.clear()
        assert abs(gesture("#head0", 0, -100)) > 20, "Slider blocked vertical scrolling"
        assert (
            c.js("document.getElementById('head0').value") == original
        ), "Scrolling changed head slider"
        assert all(
            d["head"][0] == float(original) for p, d in calls if p == "/api/command"
        ), "Scrolling sent an unintended head target: " + str(
            (original, [(p, d) for p, d in calls if p == "/api/command"])
        )
        gesture("#head0", 70, 0)
        assert float(c.js("document.getElementById('head0').value")) > float(
            original
        ), "Horizontal slider gesture did not adjust value"
        # Touch-session protection must not swallow later keyboard or mouse input.
        before_key = float(c.js("document.getElementById('head0').value"))
        c.call(
            "Input.dispatchKeyEvent", type="keyDown", key="ArrowLeft", code="ArrowLeft"
        )
        c.call(
            "Input.dispatchKeyEvent", type="keyUp", key="ArrowLeft", code="ArrowLeft"
        )
        assert float(c.js("document.getElementById('head0').value")) < before_key
        before_mouse = float(c.js("document.getElementById('head0').value"))
        point = c.js(
            "(()=>{let r=document.getElementById('head0').getBoundingClientRect();return [r.x+r.width*.2,r.y+r.height/2]})()"
        )
        c.call(
            "Input.dispatchMouseEvent",
            type="mousePressed",
            x=point[0],
            y=point[1],
            button="left",
            clickCount=1,
        )
        c.call(
            "Input.dispatchMouseEvent",
            type="mouseReleased",
            x=point[0],
            y=point[1],
            button="left",
            clickCount=1,
        )
        assert float(c.js("document.getElementById('head0').value")) < before_mouse
        c.js("document.getElementById('touch-control').click()")
        calls.clear()
        gesture("#joy-left", 0, -40)
        assert any(
            d["twist"][0] > 0 for p, d in calls if p == "/api/command"
        ), "Armed joystick did not send motion"
        assert (
            c.js(
                "document.getElementById('touch-control').getAttribute('aria-pressed')"
            )
            == "false"
        )
        assert [d["twist"] for p, d in calls if p == "/api/command"][-1] == [
            0,
            0,
            0,
        ], "Released joystick did not zero immediately"
        c.js('scrollTo({top:0,behavior:"instant"})')
        time.sleep(0.2)
        c.screenshot("overview-mobile.png")
        check_widths(c)
        c.call("Emulation.setTouchEmulationEnabled", enabled=False)
        for page, script in [("calibrate.html", "calibrate"), ("servo.html", "servo")]:
            c.call(
                "Emulation.setDeviceMetricsOverride",
                width=1440,
                height=1080,
                deviceScaleFactor=1,
                mobile=False,
            )
            c.call("Page.navigate", url=base + "/" + page)
            c.wait("document.readyState==='complete'")
            if script == "calibrate":
                c.wait(
                    "document.querySelector('[data-center=\"0\"]') && !document.querySelector('[data-center=\"0\"]').disabled"
                )
                c.js("document.querySelector('[data-center=\"0\"]').click()")
                c.wait("document.getElementById('cal-completed').textContent==='1'")
                assert ("/api/cal/center", {"index": 0}) in calls
                assert (
                    c.js("document.getElementById('state0').textContent") == "保持 2048"
                )
                c.screenshot("calibrate-desktop.png")
                c.js("document.getElementById('exit2').click()")
                c.wait("document.getElementById('mode').textContent==='标零已关闭'")
            else:
                c.js("document.getElementById('scan').click()")
                c.wait("document.querySelector('#scan-results button[data-id=\"10\"]')")
                c.js("document.getElementById('scan-stop').click()")
                c.wait("!document.getElementById('scan').disabled")
                c.js(
                    "document.querySelector('#scan-results button[data-id=\"10\"]').click()"
                )
                c.wait(
                    "document.getElementById('read-out').textContent.includes('2048')"
                )
                c.js("document.getElementById('reset-mid').click()")
                c.wait(
                    "document.getElementById('message').textContent.includes('已发送中位复位命令')"
                )
                assert any(
                    p == "/api/servo"
                    and d["action"] == "reset_mid"
                    and d["target_id"] == 10
                    for p, d in calls
                )
                c.js("document.getElementById('enable-protection').click()")
                c.wait(
                    "document.getElementById('protection-out').textContent.includes('广播已发送')"
                )
                c.screenshot("servo-desktop.png")
            assert c.js("document.documentElement.scrollWidth<=innerWidth")
            c.call(
                "Emulation.setDeviceMetricsOverride",
                width=390,
                height=844,
                deviceScaleFactor=1,
                mobile=True,
            )
            assert c.js("document.documentElement.scrollWidth<=innerWidth"), c.js(
                "document.documentElement.scrollWidth"
            )
            if script == "calibrate":
                c.js("document.getElementById('start').click()")
                c.wait("!document.querySelector('[data-center=\"1\"]').disabled")
                c.call(
                    "Emulation.setTouchEmulationEnabled", enabled=True, maxTouchPoints=2
                )
                calls.clear()
                gesture('[data-center="1"]', 0, -100)
                assert not any(
                    p == "/api/cal/center" for p, d in calls
                ), "Scrolling triggered calibration"
                c.call("Emulation.setTouchEmulationEnabled", enabled=False)
            c.js("scrollTo(0,0)")
            time.sleep(0.2)
            c.screenshot(script + "-mobile.png")
            check_widths(c)
            if script == "calibrate":
                c.js(
                    "document.getElementById('row0').scrollIntoView({block:'center',behavior:'instant'})"
                )
                c.screenshot("calibrate-controls-mobile.png")
        assert not c.errors, c.errors
        print(
            "PASS: stable status bounds and DOM identity; real touch scroll/slider/joystick gestures; 3 pages; desktop/mobile layout; live and disconnected states; mode and emergency unload; keyboard/head commands; calibration; scan/selection/reset; protection broadcast; no JavaScript exceptions."
        )
        print("Screenshots:", output)
        (output / "requests.json").write_text(
            json.dumps(calls, ensure_ascii=False, indent=2)
        )
    finally:
        chrome.terminate()
        try:
            chrome.wait(timeout=10)
        except subprocess.TimeoutExpired:
            chrome.kill()
            chrome.wait()
        server.shutdown()
        log.close()
