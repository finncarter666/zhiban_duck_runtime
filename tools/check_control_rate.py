#!/usr/bin/env python3
"""只读采样运行中的 /api/status；不使能舵机、不发送运动或维护指令。"""

import argparse
import json
import time
from urllib.request import urlopen


def check_sample(status, min_hz):
    """返回验收失败原因；目标频率不能代替实测值。"""
    timing = status.get("hardware_timing", {})
    failures = []
    if timing.get("stale", True):
        failures.append("硬件耗时样本缺失或过期")
    measured = timing.get("measured_hz")
    if measured is None or measured < min_hz:
        failures.append("实测硬件频率低于阈值")
    if timing.get("window_span_s", 0) < 3:
        failures.append("有效采样窗口不足 3 秒")
    if timing.get("maintenance_cycles", 0):
        failures.append("采样窗口包含维护/标零周期")
    if status.get("mode") in ("calibrate", "servo_debug"):
        failures.append("当前处于维护/标零模式")
    if len(status.get("servo_ids", [])) != 15:
        failures.append("舵机反馈不完整")
    if not status.get("imu_ok", False):
        failures.append("IMU 反馈无效")
    return failures


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="http://127.0.0.1:9527")
    parser.add_argument("--duration", type=float, default=60)
    parser.add_argument("--min-hz", type=float, default=98)
    args = parser.parse_args()
    if args.duration < 5 or not 0 < args.min_hz <= 100:
        parser.error("duration 至少 5 秒；min-hz 应在 (0, 100] 范围内")
    failed, initial_missed, latest_missed = False, None, None
    observed = []
    endpoint = args.url.rstrip("/") + "/api/status"
    print("只读验收：请先启动服务并预热至少 5 秒；本工具不会启动机器人运动。")
    end = time.monotonic() + args.duration
    while time.monotonic() < end:
        try:
            with urlopen(endpoint, timeout=2) as response:
                status = json.load(response)
            reasons = check_sample(status, args.min_hz)
            timing = status.get("hardware_timing", {})
            latest_missed = timing.get("missed_deadlines_total")
            if initial_missed is None:
                initial_missed = latest_missed
            elif latest_missed is None or latest_missed != initial_missed:
                reasons.append("验收期间新增漏周期，或计数器因服务重启发生变化")
            measured = timing.get("measured_hz")
            if measured is not None:
                observed.append(measured)
            stages = timing.get("stages_ms", {})
            stage_p99 = {
                key: round(value["p99"], 3) if value else None
                for key, value in stages.items()
            }
            print(
                json.dumps(
                    {
                        "measured_hz": measured,
                        "missed_deadlines_total": latest_missed,
                        "work_ms": timing.get("work_ms"),
                        "stages_p99_ms": stage_p99,
                        "failures": reasons,
                    },
                    ensure_ascii=False,
                ),
                flush=True,
            )
            failed |= bool(reasons)
        except (OSError, ValueError) as exc:
            failed = True
            print("状态读取失败：" + str(exc), flush=True)
        time.sleep(max(0, min(1, end - time.monotonic())))
    print(
        json.dumps(
            {
                "passed": not failed and bool(observed),
                "minimum_observed_hz": min(observed) if observed else None,
                "missed_deadlines_delta": (
                    latest_missed - initial_missed
                    if latest_missed is not None and initial_missed is not None
                    else None
                ),
            },
            ensure_ascii=False,
        )
    )
    return 1 if failed or not observed else 0


if __name__ == "__main__":
    raise SystemExit(main())
