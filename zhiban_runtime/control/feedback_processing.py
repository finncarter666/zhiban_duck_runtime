"""校验反馈时间戳并整理策略观测需要的关节和惯性数据。"""

from __future__ import annotations

import math
import time
from zhiban_runtime.inference.policy_math import (
    PolicyState,
    ImuFilter,
    FEEDBACK_PERIOD,
    VEL_ALPHA,
    GYRO_ALPHA,
    GRAVITY_TAU,
)


class FeedbackFilter:
    """拒绝非有限值和时间未递增的样本，保留最近一次可用的估计结果。"""

    def __init__(self):
        self.gyro_alpha = GYRO_ALPHA
        self.vel_alpha = VEL_ALPHA
        self.gravity_tau_s = GRAVITY_TAU
        self.reset()

    def reset(self):
        """清空时间戳与输出缓存，并按当前参数重建 IMU 滤波器。"""
        self._imu = ImuFilter(self.gyro_alpha, self.vel_alpha, self.gravity_tau_s)
        self._last_t = None
        self._last = None

    def configure_imu_filter(self, gyro_alpha=None, gravity_tau_s=None):
        """先校验全部参数，再同步更新本层配置和 IMU 滤波器。"""
        gyro = self.gyro_alpha if gyro_alpha is None else float(gyro_alpha)
        tau = self.gravity_tau_s if gravity_tau_s is None else float(gravity_tau_s)
        if not math.isfinite(gyro) or not 0 <= gyro <= 0.95:
            raise ValueError("gyro alpha must be between 0 and 0.95")
        if not math.isfinite(tau) or not 0.01 <= tau <= 2.0:
            raise ValueError("gravity tau must be between 0.01 and 2 seconds")
        self.gyro_alpha = self._imu.gyro_alpha = gyro
        self.gravity_tau_s = self._imu.gravity_tau_s = tau
        return {"gyro_alpha": gyro, "gravity_tau_s": tau}

    def update(
        self, acc_ms2, gyro_dps, angles_deg, vels_deg, *, timestamp=None, imu_ok=True
    ):
        """仅对有效且时间递增的样本做轴向适配和融合；无效输入不推进融合状态。"""
        acc, gyro = list(acc_ms2), list(gyro_dps)
        now = time.perf_counter() if timestamp is None else float(timestamp)
        valid = imu_ok and all(
            math.isfinite(v) for v in (*acc, *gyro, *angles_deg, *vels_deg, now)
        )
        if self._last_t is not None and now <= self._last_t:
            valid = False
        if not valid:
            self._last_t = None
            out = dict(
                self._last
                or dict(
                    grav=[0, 0, -1],
                    gyro_rad=[0] * 3,
                    angles_deg=[0] * 15,
                    vels_deg=[0] * 15,
                    imu_fusion_ready=False,
                )
            )
            out["imu_fusion_updated"] = False
            return out
        dt = FEEDBACK_PERIOD if self._last_t is None else now - self._last_t
        if self._last_t is None:
            self._imu = ImuFilter(self.gyro_alpha, self.vel_alpha, self.gravity_tau_s)
        self._last_t = now
        # 先逆转硬件层的机体系转换，再交由通用融合器处理。
        # 整条处理链中的 [z, -x, -y] 轴向映射只能生效一次。
        st = PolicyState(
            [-acc[1], -acc[2], acc[0]],
            [-gyro[1], -gyro[2], gyro[0]],
            list(angles_deg),
            list(vels_deg),
        )
        st = self._imu.update(st, dt)
        out = dict(
            angles_deg=list(angles_deg),
            vels_deg=list(st.vels),
            grav=st.grav,
            gyro_rad=st.gyro_rad,
            imu_fusion_ready=True,
            imu_fusion_updated=True,
        )
        self._last = out
        return out
