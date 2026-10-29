"""有界硬件循环耗时采样；排序和统计留给 HTTP 读取线程。"""

from collections import deque
import threading
import time


class LoopTiming:
    """记录最近若干周期，区分 I/O 工作、回调阻塞和调度唤醒延迟。"""

    STAGES = ("services", "imu", "command_write", "servo_read", "feedback_publish")

    def __init__(self, period_s, capacity=500):
        self.period_s = period_s
        self._samples = deque(maxlen=capacity)
        self._lock = threading.Lock()
        self._cycles = 0
        self._missed = 0

    def record(self, start, period_s, wakeup_s, mode, stages, missed):
        # I/O 线程只存元组；持锁期间不格式化、不排序、不记录日志。
        sample = (start, period_s, wakeup_s, mode, tuple(stages))
        with self._lock:
            self._samples.append(sample)
            self._cycles += 1
            self._missed = missed

    def snapshot(self):
        with self._lock:
            samples = tuple(self._samples)
            cycles, missed = self._cycles, self._missed
        now = time.monotonic()

        def summary(seconds):
            values = sorted(value * 1000 for value in seconds)
            if not values:
                return None
            return {
                "mean": sum(values) / len(values),
                "p99": values[min(len(values) - 1, int(len(values) * 0.99))],
                "max": values[-1],
            }

        span = samples[-1][0] - samples[0][0] if len(samples) > 1 else 0
        age = max(0.0, now - samples[-1][0]) if samples else None
        return {
            "target_hz": 1.0 / self.period_s,
            "budget_ms": self.period_s * 1000,
            "measured_hz": (len(samples) - 1) / span if span > 0 else None,
            "sample_age_ms": age * 1000 if age is not None else None,
            "stale": age is None or age > max(0.1, self.period_s * 3),
            "window_cycles": len(samples),
            "window_span_s": span,
            "cycles_total": cycles,
            "missed_deadlines_total": missed,
            "maintenance_cycles": sum(s[3] != "normal" for s in samples),
            "period_ms": summary(s[1] for s in samples if s[1] > 0),
            "wakeup_lateness_ms": summary(s[2] for s in samples),
            "work_ms": summary(sum(s[4]) for s in samples),
            "stages_ms": {
                name: summary(s[4][i] for s in samples)
                for i, name in enumerate(self.STAGES)
            },
        }
