"""通过 FastAPI 连接控制接口、操作页面和运行时生命周期。"""

import sys
import threading
from contextlib import asynccontextmanager

sys.dont_write_bytecode = True
sys.setswitchinterval(0.001)

from fastapi import FastAPI, HTTPException
from fastapi.staticfiles import StaticFiles
from zhiban_runtime.control.motion_control import MotionController
from zhiban_runtime.hardware.device_runtime import HardwareConfig, RobotHardware

from zhiban_runtime.paths import WEB_DIR


def create_app(config: HardwareConfig):
    """创建硬件与控制器，绑定反馈回调、线程生命周期和 HTTP 路由。"""
    robot = RobotHardware(config)
    controller = MotionController(robot)
    robot.set_callbacks(
        feedback=controller.handle_feedback,
        raw=controller.handle_encoder_snapshot,
        cal_reply=controller.handle_calibration_reply,
        servo_reply=controller.handle_servo_reply,
    )
    worker = threading.Thread(target=controller.run, name="policy50", daemon=True)

    @asynccontextmanager
    async def lifespan(_app):
        robot.start()
        worker.start()
        try:
            yield
        finally:
            controller.stop()
            worker.join(timeout=2)
            robot.stop()

    app = FastAPI(title="Zhiban Duck Linux Runtime", lifespan=lifespan)

    def call(method, body=None):
        try:
            return method(**body) if body is not None else method()
        except (KeyError, TypeError, ValueError) as exc:
            raise HTTPException(400, str(exc))

    @app.get("/api/status")
    def status():
        result = controller.status()
        # 控制器状态锁已经释放，再复制有界样本并计算诊断统计。
        result["hardware_timing"] = robot.timing.snapshot()
        return result

    @app.post("/api/mode")
    def mode(body: dict):
        return call(controller.set_mode, {"mode": body["mode"]})

    @app.post("/api/roller")
    def roller(body: dict):
        return call(controller.set_roller_mode, {"enabled": body["enabled"]})

    @app.post("/api/command")
    def command(body: dict):
        return call(controller.command, body)

    @app.post("/api/filter")
    def filtering(body: dict):
        leg, head = call(controller.configure_action_smoothing, body)
        return {"leg": leg, "head": head}

    @app.post("/api/imu-filter")
    def imu_filtering(body: dict):
        return call(controller.configure_imu_filter, body)

    @app.post("/api/action-scale")
    def action_scale(body: dict):
        return call(controller.set_action_scale, body)

    @app.post("/api/pick")
    def pick():
        return call(controller.start_pick)

    @app.post("/api/roulade")
    def roulade():
        return call(controller.start_roulade)

    @app.post("/api/cal/enter")
    def cal_enter():
        return call(controller.set_mode, {"mode": "calibrate"})

    @app.post("/api/cal/exit")
    def cal_exit():
        return call(controller.set_mode, {"mode": "shadow"})

    @app.post("/api/cal/heartbeat")
    def cal_heartbeat():
        return call(controller.cal_heartbeat)

    @app.post("/api/cal/center")
    def cal_center(body: dict):
        return call(controller.cal_center, {"index": body["index"]})

    @app.post("/api/servo/enter")
    def servo_enter():
        return call(controller.set_mode, {"mode": "servo_debug"})

    @app.post("/api/servo/exit")
    def servo_exit():
        return call(controller.set_mode, {"mode": "shadow"})

    @app.post("/api/servo")
    def servo_action(body: dict):
        try:
            return controller.servo_op(
                body["action"],
                body.get("target_id"),
                new_id=body.get("new_id"),
                kp=body.get("kp", 5),
                kd=body.get("kd", 20),
                protection_enabled=body.get("protection_enabled", True),
                current_a=body.get("current_a", 1.0),
            )
        except (KeyError, TypeError, ValueError) as exc:
            raise HTTPException(400, str(exc))

    app.mount("/", StaticFiles(directory=WEB_DIR, html=True), name="web")
    return app
