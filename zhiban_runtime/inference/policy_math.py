"""定义电机排列、IMU 融合以及策略输入输出的数值约定。"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple
import numpy as np


GYRO_ALPHA = 0.5

VEL_ALPHA = 0.4

FEEDBACK_PERIOD = 0.01  # 硬件反馈周期为 10 ms（100 Hz）。

GRAVITY_TAU = 0.3

# 安装偏角采用机体系角度制，加速度与角速度使用同一组修正。
# 静止且机身水平时：pitch = atan2(gx, -gz)，roll = atan2(-gy, -gz)。
# 修正后的重力指向 [0, 0, -1]；仅靠重力无法确定 yaw。
IMU_BIAS_DEG = (0.0, 0.0, 0.0)  # 依次为横滚、俯仰、偏航角。

MOTOR_IDS = [
    10,
    11,
    12,
    13,
    14,
    20,
    21,
    22,
    23,
    24,
    30,
    31,
    32,
    33,
    34,
]

# 行走与回退姿态按总线顺序排列；滚轮姿态另从模型元数据读取。
DEFAULT_DEG = [
    0.0,
    -5.0,
    -22.0,
    0.0,
    22.0,
    0.0,
    5.0,
    22.0,
    0.0,
    -22.0,
    20.0,
    20.0,
    0.0,
    0.0,
    0.0,
]

POLICY_IDS = [
    10,
    11,
    12,
    13,
    14,
    30,
    31,
    32,
    33,
    20,
    21,
    22,
    23,
    24,
]

POLICY_INDEX = np.array([MOTOR_IDS.index(i) for i in POLICY_IDS])

HOME_DEG = np.array(DEFAULT_DEG, dtype=np.float32)[POLICY_INDEX]

MOUTH_INDEX = MOTOR_IDS.index(34)

MOUTH_MAX_DEG = 30.0

OBS_DIM = 61

ACTION_DIM = 14

ACTION_SCALE = 1.0

ACTION_ALPHA = 0.45

ACTION_ALPHA_HEAD = 0.2


@dataclass
class PolicyState:
    """汇集观测组装所需的传感器向量、关节样本及融合结果。"""

    acc: List[float]
    gyro: List[float]
    angles: List[float]  # 与 MOTOR_IDS 的排列顺序一致。
    vels: List[float]
    grav: List[float] = field(default_factory=lambda: [0.0, 0.0, -1.0])
    gyro_rad: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])


def _apply_mounting_rotation(vec: List[float]) -> List[float]:
    """按已配置的安装偏角修正 IMU 向量；零偏角时直接返回输入。"""
    roll, pitch, yaw = IMU_BIAS_DEG
    if roll == 0.0 and pitch == 0.0 and yaw == 0.0:
        return vec
    r, p, y = math.radians(roll), math.radians(pitch), math.radians(yaw)
    x, yv, z = vec
    if y:
        cy, sy = math.cos(y), math.sin(y)
        x, yv = x * cy - yv * sy, x * sy + yv * cy
    if r:
        cr, sr = math.cos(r), math.sin(r)
        yv, z = yv * cr - z * sr, yv * sr + z * cr
    if p:
        cp, sp = math.cos(p), math.sin(p)
        x, z = x * cp + z * sp, -x * sp + z * cp
    return [x, yv, z]


class ImuFilter:
    """以陀螺仪预测机体系重力，用按可信度加权的加速度校正，不在线估计零偏。"""

    def __init__(
        self,
        gyro_alpha: float = GYRO_ALPHA,
        vel_alpha: float = VEL_ALPHA,
        gravity_tau_s: float = GRAVITY_TAU,
    ) -> None:
        self.gyro_alpha = gyro_alpha
        self.vel_alpha = vel_alpha
        self.gravity_tau_s = gravity_tau_s
        self._gravity: Optional[List[float]] = None
        self._gyro: Optional[List[float]] = None
        self._vel: Optional[List[float]] = None

    def update(self, st: PolicyState, dt: float = FEEDBACK_PERIOD) -> PolicyState:
        """对齐传感器轴向，按采样间隔融合重力并平滑角速度和关节速度。"""
        acc_n = _apply_mounting_rotation([st.acc[2], -st.acc[0], -st.acc[1]])
        gyro_n = _apply_mounting_rotation([st.gyro[2], -st.gyro[0], -st.gyro[1]])
        norm = math.sqrt(sum(v * v for v in acc_n))
        acc_unit = [v / norm for v in acc_n] if norm > 1e-6 else None
        # 采样间隔过大时重新初始化，不用单个样本外推缺失时段的旋转。
        reset = self._gravity is None or dt > 0.1
        dt = max(0.001, min(dt, 0.1))
        if reset:
            self._gravity = acc_unit if acc_unit is not None else [0.0, 0.0, -1.0]
            self._gyro = gyro_n
            self._vel = list(st.vels)
        else:
            gx, gy, gz = self._gravity
            wx, wy, wz = [math.radians(v) for v in gyro_n]
            # 世界系固定向量在旋转机体系中满足 dg/dt = -omega × g。
            predicted = [
                gx + (wz * gy - wy * gz) * dt,
                gy + (wx * gz - wz * gx) * dt,
                gz + (wy * gx - wx * gy) * dt,
            ]
            pnorm = math.sqrt(sum(v * v for v in predicted))
            predicted = [v / pnorm for v in predicted]
            trust = max(0.0, 1.0 - abs(norm / 9.8 - 1.0) / 0.2)
            gain = (1.0 - math.exp(-dt / self.gravity_tau_s)) * trust
            if acc_unit is not None:
                predicted = [
                    (1 - gain) * p + gain * a for p, a in zip(predicted, acc_unit)
                ]
            pnorm = math.sqrt(sum(v * v for v in predicted))
            self._gravity = [v / pnorm for v in predicted]
            # 按实际经过的周期数换算平滑强度，保留 100 Hz 下的原系数含义。
            ga = self.gyro_alpha ** (dt / FEEDBACK_PERIOD)
            va = self.vel_alpha ** (dt / FEEDBACK_PERIOD)
            self._gyro = [ga * o + (1 - ga) * n for o, n in zip(self._gyro, gyro_n)]
            self._vel = [va * o + (1 - va) * n for o, n in zip(self._vel, st.vels)]
        st.grav = list(self._gravity)
        st.gyro_rad = [v * math.pi / 180.0 for v in self._gyro]
        st.vels = list(self._vel)
        return st


def joint_state_in_policy_order(
    st: PolicyState, home_deg=HOME_DEG
) -> Tuple[np.ndarray, np.ndarray]:
    """按策略顺序选择关节，将相对初始姿态的位置和关节速度转换为弧度单位。"""
    q_deg = np.asarray(st.angles, dtype=np.float32)[POLICY_INDEX]
    v_deg = np.asarray(st.vels, dtype=np.float32)[POLICY_INDEX]
    return np.deg2rad(q_deg - home_deg), np.deg2rad(v_deg)


def build_observation(
    st: PolicyState,
    prev_action: np.ndarray,
    twist: np.ndarray,
    head: np.ndarray,
    out: np.ndarray | None = None,
    home_deg=HOME_DEG,
) -> np.ndarray:
    """按模型固定切片填充 61 维观测，可复用调用方提供的输出缓存。"""
    obs = np.empty(OBS_DIM, dtype=np.float32) if out is None else out
    obs[0:3] = st.gyro_rad
    obs[3:6] = st.grav
    obs[6:20], obs[20:34] = joint_state_in_policy_order(st, home_deg)
    obs[34:48] = prev_action
    obs[48:51] = twist
    obs[51:55] = head
    obs[55:61] = 0.0
    return obs


def action_to_joint_degrees(
    action: np.ndarray,
    mouth_deg: float = 0.0,
    *,
    home_deg=HOME_DEG,
    action_scale=ACTION_SCALE,
) -> List[float]:
    """将相对初始姿态的动作映射至全部舵机目标，仅对嘴部角度做范围裁剪。"""
    q_deg = home_deg + np.rad2deg(action_scale * action)
    targets = np.zeros(len(MOTOR_IDS), dtype=np.float32)
    targets[POLICY_INDEX] = q_deg
    targets[MOUTH_INDEX] = float(np.clip(mouth_deg, 0.0, MOUTH_MAX_DEG))
    return targets.tolist()
