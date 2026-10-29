"""解析 Zhiban 运行参数并启动 HTTP 服务。"""

import argparse

import uvicorn

from zhiban_runtime.application import create_app
from zhiban_runtime.hardware.device_runtime import HardwareConfig


def parse_args():
    """读取串口、I2C 与 HTTP 参数，未提供的选项沿用部署默认值。"""
    p = argparse.ArgumentParser(description="Zhiban Duck Linux runtime")
    p.add_argument("--servo-device", default="/dev/ttyUSB0")
    p.add_argument("--servo-baud", type=int, default=1_000_000)
    p.add_argument("--servo-rs485", action="store_true")
    p.add_argument("--i2c-bus", type=int, default=1)
    p.add_argument("--imu-address", type=lambda x: int(x, 0))
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--port", type=int, default=9527)
    return p.parse_args()


def main():
    """按命令行参数创建硬件配置和应用，再交由 Uvicorn 运行。"""
    a = parse_args()
    cfg = HardwareConfig(
        a.servo_device, a.servo_baud, a.servo_rs485, a.i2c_bus, a.imu_address
    )
    uvicorn.run(create_app(cfg), host=a.host, port=a.port, log_level="info")
