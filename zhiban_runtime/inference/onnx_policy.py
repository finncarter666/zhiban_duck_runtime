"""加载 ONNX 运动模型，维护每个会话的动作历史和滤波状态。"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Optional
import numpy as np

from zhiban_runtime.inference.policy_math import (
    HOME_DEG,
    ACTION_SCALE,
    ACTION_ALPHA,
    ACTION_ALPHA_HEAD,
    OBS_DIM,
    ACTION_DIM,
    PolicyState,
    build_observation as assemble_policy_observation,
    action_to_joint_degrees as map_policy_targets,
)
from zhiban_runtime.paths import MODEL_DIR


def default_model_path() -> Optional[Path]:
    """定位内置行走模型；对应文件不存在时返回 None。"""
    p = MODEL_DIR / "zhiban_walk.onnx"
    return p if p.is_file() else None


class OnnxPolicy:
    """管理单个 CPU 推理会话、模型初始姿态及上一时刻动作。"""

    def __init__(self):
        self.action_scale = ACTION_SCALE
        self.action_alpha = ACTION_ALPHA
        self.action_alpha_head = ACTION_ALPHA_HEAD
        self.unload()

    @property
    def loaded(self):
        return self.session is not None

    @property
    def default_deg(self):
        return self.action_to_joint_degrees(np.zeros(ACTION_DIM, dtype=np.float32))

    def unload(self):
        """清除会话引用，恢复默认姿态并重新初始化观测和动作缓存。"""
        self.session = None
        self.in_name = self.file = ""
        self.home_deg = HOME_DEG.copy()
        self._obs = np.empty(OBS_DIM, dtype=np.float32)
        self.reset_action_history()

    def load(self, path=None, *, use_model_home=False):
        """校验模型接口，按需读取初始姿态元数据，并完成会话预热。"""
        p = Path(path) if path else default_model_path()
        if p is None:
            return False, "default model zhiban_walk.onnx was not found"
        if not p.is_file():
            p = MODEL_DIR / p.name
        if not p.is_file():
            return False, f"not found: {p}"
        try:
            import onnxruntime as ort

            options = ort.SessionOptions()
            options.intra_op_num_threads = 1
            options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
            sess = ort.InferenceSession(
                str(p), sess_options=options, providers=["CPUExecutionProvider"]
            )
            inp, out = sess.get_inputs()[0], sess.get_outputs()[0]
            if len(inp.shape) != 2 or inp.shape[-1] != OBS_DIM:
                raise ValueError(f"expected observation shape [1,61], got {inp.shape}")
            if len(out.shape) != 2 or out.shape[-1] != ACTION_DIM:
                raise ValueError(f"expected action shape [1,14], got {out.shape}")
            self.home_deg = HOME_DEG.copy()
            if use_model_home:
                home = sess.get_modelmeta().custom_metadata_map.get("default_joint_pos")
                if home is not None:
                    home = np.array(
                        [float(v) for v in home.split(",")], dtype=np.float32
                    )
                    if home.shape != (ACTION_DIM,) or not np.isfinite(home).all():
                        raise ValueError(
                            "default_joint_pos must contain 14 finite radians in policy order"
                        )
                    self.home_deg = np.rad2deg(home)
            self.session, self.in_name = sess, inp.name
            self._outputs = [out.name]
            self.file = p.name
            self._feed = {self.in_name: self._obs.reshape(1, OBS_DIM)}
            # 在进入实时策略切换前分配推理所需的工作内存。
            result = self.warmup(12)
            if result.shape != (1, ACTION_DIM) or not np.isfinite(result).all():
                raise ValueError("model must emit a finite [1,14] action")
        except Exception as exc:
            self.unload()
            return False, str(exc)
        self.reset_action_history()
        return True, f"{self.file} in=61 out=14 threads=1"

    def warmup(self, runs: int = 1):
        """使用全零观测预热会话，不推进上一动作或动作滤波历史。"""
        if not self.loaded:
            return None
        self._obs.fill(0)
        last = None
        for _ in range(max(1, runs)):
            last = self.session.run(self._outputs, self._feed)[0]
        return last

    def build_observation(
        self, grav, gyro_rad, angles_deg, vels_deg, cmd=None, head=None
    ):
        """将反馈和可选命令写入观测缓存，并拒绝包含非有限值的结果。"""
        st = PolicyState(
            [0.0] * 3,
            [0.0] * 3,
            list(angles_deg),
            list(vels_deg),
            list(grav),
            list(gyro_rad),
        )
        twist = np.zeros(3, dtype=np.float32) if cmd is None else cmd
        head_cmd = np.zeros(4, dtype=np.float32) if head is None else head
        obs = assemble_policy_observation(
            st, self.prev_action, twist, head_cmd, out=self._obs, home_deg=self.home_deg
        )
        if not np.isfinite(obs).all():
            raise ValueError("observation contains NaN or Inf")
        return obs

    def infer_action(self, grav, gyro_rad, angles_deg, vels_deg, cmd=None, head=None):
        """保存模型原始输出作为下一次观测历史，再返回分组平滑后的动作。"""
        if not self.loaded:
            return None
        self.build_observation(grav, gyro_rad, angles_deg, vels_deg, cmd, head)
        action = self.session.run(self._outputs, self._feed)[0].reshape(ACTION_DIM)
        if not np.isfinite(action).all():
            raise ValueError("model output contains NaN or Inf")
        self.prev_action[:] = action
        # 数值约定：[5:10] 使用头部滤波系数，该切片同时包含电机 ID 20。
        for sl, alpha in (
            (slice(0, 5), self.action_alpha),
            (slice(5, 10), self.action_alpha_head),
            (slice(10, 14), self.action_alpha),
        ):
            self._act_filt[sl] *= alpha
            self._act_filt[sl] += action[sl] * (1 - alpha)
        return self._act_filt.copy()

    def configure_action_smoothing(self, *, leg=None, head=None):
        # 两个系数全部有效后再提交，任一个无效都不修改现有设置。
        """校验腿部和头部系数后一起更新，避免无效参数造成部分修改。"""
        leg = self.action_alpha if leg is None else float(leg)
        head = self.action_alpha_head if head is None else float(head)
        if any(not math.isfinite(v) or not 0 <= v <= 1 for v in (leg, head)):
            raise ValueError("action alpha must be within [0,1]")
        self.action_alpha, self.action_alpha_head = leg, head
        return leg, head

    def reset_action_history(self):
        """将原始动作历史和动作平滑累积值同时清零。"""
        self.prev_action = np.zeros(ACTION_DIM, dtype=np.float32)
        self._act_filt = np.zeros(ACTION_DIM, dtype=np.float32)

    def action_to_joint_degrees(self, action):
        """按当前初始姿态和动作缩放系数，将策略动作转换为关节角度目标。"""
        return map_policy_targets(
            np.asarray(action, dtype=np.float32),
            home_deg=self.home_deg,
            action_scale=self.action_scale,
        )
