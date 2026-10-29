"""协调反馈和运动模式，并在独立的 50 Hz 循环中执行策略。"""

import collections
import math
import threading
import time
from zhiban_runtime.inference.onnx_policy import OnnxPolicy
from zhiban_runtime.control.posture_recovery import RecoveryStateMachine
from zhiban_runtime.control.feedback_processing import FeedbackFilter
from zhiban_runtime.inference.policy_math import (
    MOTOR_IDS,
    IMU_BIAS_DEG,
    MOUTH_INDEX,
    MOUTH_MAX_DEG,
)
from zhiban_runtime.messages import (
    ALL_MASK,
    ARM,
    BROADCAST_ID,
    CAL_CENTER_ONE,
    CAL_ENTER,
    CAL_EXIT,
    CAL_HOLD_POS,
    DISARM,
    HOLD,
    POLICY,
    SERVO_CENTER_POS,
    SERVO_ENTER,
    SERVO_EXIT,
    SERVO_PING,
    SERVO_READ,
    SERVO_RESET_MID,
    SERVO_SET_ID,
    SERVO_SET_PERM_KP_KD,
    SERVO_SET_PROTECTION,
    SERVO_SET_PROTECTION_CURRENT,
    SERVO_UNLOCK,
    make_calibration_request,
    command,
    unsigned_counter_delta,
    make_current_limit_request,
    make_protection_request,
    make_servo_request,
)

EXCLUSIVE_MODES = ("calibrate", "servo_debug")
PICK_PERIOD_S = 4.0
PICK_MOUTH_CLOSE_PHI = 0.4
ROULADE_S = 1.9
ROULADE_ALPHA = 0.15
CONTROL_PERIOD = 0.02


class MotionController:
    """集中管理策略选择、命令校验及受锁保护的运动状态。"""

    def __init__(self, robot):
        self.robot = robot
        self.lock = threading.RLock()
        self.servo_op_lock = threading.Lock()
        self.cal_op_lock = threading.Lock()
        self.policy_lock = threading.Lock()
        self.policy = OnnxPolicy()
        ok, message = self.policy.load()
        if not ok:
            raise RuntimeError(message)
        self.model_message = message
        self.walk_policy = self.policy
        self.roller_policy = OnnxPolicy()
        ok, message = self.roller_policy.load("zhiban_roller.onnx", use_model_home=True)
        if not ok:
            raise RuntimeError(message)
        self.roller_policy.action_scale = 0.7
        self.roller_policy.configure_action_smoothing(leg=0.4, head=0.5)
        self.model_message += "; " + message
        self.roller_mode = False
        self.getup_policy = OnnxPolicy()
        ok, message = self.getup_policy.load("zhiban_getup.onnx")
        if not ok:
            raise RuntimeError(message)
        self.model_message += "; " + message
        self.pick_policy = OnnxPolicy()
        ok, message = self.pick_policy.load("zhiban_pick.onnx")
        if not ok:
            raise RuntimeError(message)
        self.model_message += "; " + message
        self.roulade_policy = OnnxPolicy()
        ok, message = self.roulade_policy.load("zhiban_roulade.onnx")
        if not ok:
            raise RuntimeError(message)
        self.model_message += "; " + message
        self.recovery = RecoveryStateMachine()
        self.policy_needs_reset = True
        self.pick_active = False
        self.pick_phi = 0.0
        self.roulade_until = 0.0
        self._roulade_alpha = None
        self._warm_index = 0
        self.filter = FeedbackFilter()
        self.feedback = self.filtered = None
        self.rx_time = 0.0
        self.mode = "shadow"
        self.generation = 0
        self.twist, self.head, self.mouth = [0.0] * 3, [0.0] * 4, 0.0
        self.web_time = 0.0
        self.arm_pending = False
        self.seq = self.received = self.inferred = self.sent = self.bad_frames = (
            self.gaps
        ) = 0
        self.last_error = ""
        self.stop_event = threading.Event()
        self.rx_times = collections.deque(maxlen=1000)
        self.infer_times = collections.deque(maxlen=500)
        self.infer_ms = collections.deque(maxlen=500)
        self.rx_intervals = collections.deque(maxlen=1000)
        self.loop_missed = 0
        self.last_action = [0.0] * 15
        self.started = time.monotonic()
        self.raw_pos = [SERVO_CENTER_POS] * 15
        self.cal_done = [False] * 15
        self.cal_reply = None
        self.cal_seq = 0
        self.cal_reply_event = threading.Event()
        self.cal_expected_index = None
        self.raw_mask = 0
        self.servo_reply = None
        self.servo_reply_event = threading.Event()
        self.servo_expected_reply = None

    def handle_encoder_snapshot(self, raw):
        """在状态锁内保存编码器快照和在线掩码，更新校准完成标记。"""
        try:
            pkt = raw
        except ValueError:
            return
        with self.lock:
            self.raw_pos = pkt["pos"]
            self.raw_mask = pkt["mask"]
            self.cal_done = [
                self.mode == "calibrate" and bool(pkt["held_mask"] & (1 << i))
                for i in range(15)
            ]

    def handle_calibration_reply(self, raw):
        """仅接收序号和关节索引均匹配的校准回复，并通知等待方。"""
        try:
            reply = raw
        except ValueError:
            return
        with self.lock:
            if self.cal_expected_index != (reply["seq"], reply["index"]):
                return
            self.cal_reply = reply
            self.cal_reply_event.set()

    def handle_servo_reply(self, raw):
        """仅接收操作码与目标 ID 匹配的维护回复，并通知等待方。"""
        try:
            pkt = raw
        except ValueError:
            return
        with self.lock:
            if self.servo_expected_reply != (pkt["op"], pkt["target_id"]):
                return
            self.servo_reply = pkt
            self.servo_reply_event.set()

    def handle_feedback(self, raw):
        """处理反馈序号的重复、回绕或重启，随后更新滤波结果与接收统计。"""
        try:
            fb = raw
        except ValueError:
            with self.lock:
                self.bad_frames += 1
            return
        now = time.monotonic()
        with self.lock:
            old = self.feedback
            if old:
                step = unsigned_counter_delta(fb.seq, old.seq)
                if step == 0:
                    return
                if step > 0x80000000:
                    self.mode = "shadow"
                    self.generation += 1
                    self.arm_pending = False
                    self.filter.reset()
                    self.recovery.reset()
                    self.policy_needs_reset = True
                    self._stop_pick_locked()
                    self._stop_roulade_locked()
                    self.cal_done = [False] * 15
                    self.cal_expected_index = None
                    self.cal_reply_event.set()
                else:
                    self.gaps += step - 1
                self.rx_intervals.append((now - self.rx_time) * 1000)
            self.filtered = self.filter.update(
                fb.acc,
                fb.gyro,
                fb.angles,
                fb.velocities,
                timestamp=now,
                imu_ok=fb.imu_ok,
            )
            self.feedback, self.rx_time = fb, now
            self.received += 1
            self.rx_times.append(now)

    def _require_recent_feedback(self, mask):
        """要求反馈未过期、IMU 有效且覆盖指定舵机掩码，否则拒绝操作。"""
        fb = self.feedback
        if not fb or time.monotonic() - self.rx_time > 0.15:
            raise ValueError("hardware feedback is stale")
        if not fb.imu_ok or (fb.mask & mask) != mask:
            raise ValueError("required servo feedback / IMU missing")
        return fb

    def _send_disarm_command(self):
        """取消待使能状态，递增命令序号并立即提交卸载扭矩命令。"""
        self.arm_pending = False
        self.seq = (self.seq + 1) & 0xFFFFFFFF
        sample = self.feedback.sample_us if self.feedback else 0
        self.robot.submit_command(
            command(self.seq, sample, DISARM, self.policy.default_deg)
        )

    def set_mode(self, mode):
        """保留反馈和独占服务的进入约束，完成模式切换及相关状态复位。"""
        if mode not in ("shadow", "off", "hold", "policy", "calibrate", "servo_debug"):
            raise ValueError("unknown mode")
        with self.lock:
            if mode == self.mode and mode != "calibrate":
                self.web_time = time.monotonic()
                return {"mode": self.mode}
            if mode in ("hold", "policy") and self.mode in EXCLUSIVE_MODES:
                raise ValueError("exit calibrate/servo_debug first")
            if mode == "servo_debug" and self.mode in ("hold", "policy"):
                raise ValueError("disarm before calibrate/servo_debug")
            fb = None
            if mode in ("hold", "policy"):
                fb = self._require_recent_feedback(0)
            previous = self.mode
            self.generation += 1
            self.mode = mode
            self.recovery.reset()
            self.policy_needs_reset = True
            self._stop_pick_locked()
            self._stop_roulade_locked()
            self.last_error = ""
            self.web_time = time.monotonic()
            self.twist, self.head, self.mouth = [0.0] * 3, [0.0] * 4, 0.0
            self.arm_pending = fb is not None and not fb.enabled
            if previous == "calibrate" and mode != "calibrate":
                self.cal_done = [False] * 15
                self.cal_expected_index = None
                self.cal_reply_event.set()
                self.robot.enqueue_calibration_request(
                    make_calibration_request(CAL_EXIT)
                )
            if previous == "servo_debug" and mode != "servo_debug":
                self.robot.enqueue_servo_request(make_servo_request(SERVO_EXIT, 10))
            if mode == "calibrate":
                self._send_disarm_command()
                self.cal_done = [False] * 15
                self.cal_expected_index = None
                self.cal_reply_event.set()
                self.robot.enqueue_calibration_request(
                    make_calibration_request(CAL_ENTER)
                )
            elif mode == "servo_debug":
                self._send_disarm_command()
                self.robot.enqueue_servo_request(make_servo_request(SERVO_ENTER, 10))
            elif previous in EXCLUSIVE_MODES:
                self._send_disarm_command()
        with self.policy_lock:
            for model in self._all_motion_policies():
                model.reset_action_history()
        return {"mode": mode}

    def set_roller_mode(self, enabled):
        """切换行走会话，并使切换前尚未提交的动作失效。"""
        if not isinstance(enabled, bool):
            raise ValueError("roller enabled must be true or false")
        with self.lock:
            if enabled != self.roller_mode:
                with self.policy_lock:
                    self.policy = self.roller_policy if enabled else self.walk_policy
                    self.policy.reset_action_history()
                    self.policy.warmup(1)
                self.roller_mode = enabled
                # 递增控制代次，使此前会话尚未提交的推理结果失效。
                self.generation += 1
                if (
                    self.recovery.phase == "walk"
                    and not self.pick_active
                    and not self._roulade_is_active(time.monotonic())
                ):
                    self.policy_needs_reset = True
                self.twist = [0.0] * 3
                self.web_time = time.monotonic()
            return dict(
                enabled=self.roller_mode,
                model=self.policy.file,
                action_scale=self.policy.action_scale,
                action_alpha=self.policy.action_alpha,
                action_alpha_head=self.policy.action_alpha_head,
            )

    def _all_motion_policies(self):
        """按固定顺序列出全部五个策略会话。"""
        return (
            self.walk_policy,
            self.roller_policy,
            self.getup_policy,
            self.pick_policy,
            self.roulade_policy,
        )

    def _non_roller_policies(self):
        """列出共享普通动作参数的会话，不包含滚轮策略。"""
        return (
            self.walk_policy,
            self.getup_policy,
            self.pick_policy,
            self.roulade_policy,
        )

    def _apply_action_smoothing_locked(self, leg, head):
        """在调用方持锁期间更新非滚轮策略的腿部和头部滤波系数。"""
        for model in self._non_roller_policies():
            model.configure_action_smoothing(leg=leg, head=head)

    def configure_action_smoothing(self, **values):
        with self.lock:
            with self.policy_lock:
                if self.roller_mode:
                    return self.roller_policy.configure_action_smoothing(**values)
                leg, head = self.walk_policy.configure_action_smoothing(**values)
                if self._roulade_alpha is not None:
                    self._roulade_alpha = (leg, head)
                    self._apply_action_smoothing_locked(ROULADE_ALPHA, ROULADE_ALPHA)
                else:
                    self._apply_action_smoothing_locked(leg, head)
                return leg, head

    def configure_imu_filter(self, **values):
        with self.lock:
            return self.filter.configure_imu_filter(**values)

    def set_action_scale(self, scale):
        scale = float(scale)
        if not math.isfinite(scale) or not 0 <= scale <= 2:
            raise ValueError("action scale must be within [0,2]")
        with self.lock:
            with self.policy_lock:
                models = (
                    (self.roller_policy,)
                    if self.roller_mode
                    else self._non_roller_policies()
                )
                for model in models:
                    model.action_scale = scale
        return {"scale": scale}

    def _stop_pick_locked(self):
        self.pick_active = False
        self.pick_phi = 0.0

    def _stop_roulade_locked(self):
        self.roulade_until = 0.0
        saved = self._roulade_alpha
        self._roulade_alpha = None
        if saved is not None:
            with self.policy_lock:
                self._apply_action_smoothing_locked(*saved)

    def _roulade_is_active(self, now):
        """判断翻滚截止时间是否存在且尚未到达。"""
        return now < self.roulade_until

    def start_pick(self):
        with self.lock:
            if self.mode not in ("shadow", "policy"):
                raise ValueError("pick requires shadow/policy mode")
            if self.recovery.phase != "walk" or self._roulade_is_active(
                time.monotonic()
            ):
                raise ValueError("pick only while walking upright")
            self.pick_active = True
            self.pick_phi = 0.0
            self.mouth = MOUTH_MAX_DEG
            self.policy_needs_reset = True
            self.web_time = time.monotonic()
            self.last_error = ""
        with self.policy_lock:
            self.pick_policy.reset_action_history()
            self.pick_policy.warmup(2)
        return {"ok": True, "phi": 0.0, "model": self.pick_policy.file}

    def start_roulade(self):
        with self.lock:
            if self.mode not in ("shadow", "policy"):
                raise ValueError("roulade requires shadow/policy mode")
            if self.recovery.phase != "walk" or self.pick_active:
                raise ValueError("roulade only while walking")
            if self._roulade_alpha is None:
                self._roulade_alpha = (
                    self.walk_policy.action_alpha,
                    self.walk_policy.action_alpha_head,
                )
            self.roulade_until = time.monotonic() + ROULADE_S
            self.policy_needs_reset = True
            self.web_time = time.monotonic()
            self.last_error = ""
            with self.policy_lock:
                self._apply_action_smoothing_locked(ROULADE_ALPHA, ROULADE_ALPHA)
                self.roulade_policy.reset_action_history()
                self.roulade_policy.warmup(2)
        return {"ok": True, "model": self.roulade_policy.file, "seconds": ROULADE_S}

    def command(self, twist=None, head=None, mouth=None):
        def values(seq, limits):
            if len(seq) != len(limits):
                raise ValueError("wrong command dimension")
            seq = [float(v) for v in seq]
            if any(
                not math.isfinite(v) or abs(v) > limit for v, limit in zip(seq, limits)
            ):
                raise ValueError("command exceeds bounds")
            return seq

        t = values(twist, [0.4, 0.3, 1.0]) if twist is not None else None
        h = values(head, [1.0] * 4) if head is not None else None
        m = None
        if mouth is not None:
            m = float(mouth)
            if not math.isfinite(m) or not 0 <= m <= MOUTH_MAX_DEG:
                raise ValueError("command exceeds bounds")
        with self.lock:
            if self.mode in EXCLUSIVE_MODES:
                raise ValueError("motion commands disabled in calibrate/servo_debug")
            if t is not None and not self.pick_active:
                self.twist = t
            if h is not None and not self.pick_active:
                self.head = h
            if m is not None and not self.pick_active:
                self.mouth = m
            self.web_time = time.monotonic()
        return {"ok": True}

    def cal_heartbeat(self):
        with self.lock:
            if self.mode != "calibrate":
                raise ValueError("not in calibrate mode")
            self.web_time = time.monotonic()
        return {"ok": True}

    def cal_center(self, index):
        """串行执行单关节居中请求，并等待对应的校准回复。"""
        if not isinstance(index, int) or isinstance(index, bool) or not 0 <= index < 15:
            raise ValueError("joint index out of range")
        with self.cal_op_lock:
            with self.lock:
                if self.mode != "calibrate":
                    raise ValueError("not in calibrate mode")
                if not self.feedback or time.monotonic() - self.rx_time > 0.15:
                    raise ValueError("hardware feedback is stale")
                generation = self.generation
                self.cal_seq = (self.cal_seq + 1) & 0xFFFFFFFF
                seq = self.cal_seq
                self.web_time = time.monotonic()
                self.cal_reply = None
                self.cal_reply_event.clear()
                self.cal_expected_index = (seq, index)
            try:
                self.robot.enqueue_calibration_request(
                    make_calibration_request(CAL_CENTER_ONE, index=index, seq=seq)
                )
                if not self.cal_reply_event.wait(0.75):
                    raise ValueError("centering acknowledgement timeout")
                with self.lock:
                    if self.mode != "calibrate" or self.generation != generation:
                        raise ValueError("calibration session changed")
                    if not self.cal_reply or not self.cal_reply["ok"]:
                        raise ValueError(
                            f"ID {MOTOR_IDS[index]} 中位保持未确认：请检查舵机通信、供电及保护状态"
                        )
                    self.web_time = time.monotonic()
                return {
                    "ok": True,
                    "index": index,
                    "id": MOTOR_IDS[index],
                    "hold_pos": CAL_HOLD_POS,
                }
            finally:
                with self.lock:
                    self.cal_expected_index = None

    def _require_servo_service_mode(self):
        """确认当前处于舵机维护模式，并刷新该模式的心跳时间。"""
        if self.mode != "servo_debug":
            raise ValueError("not in servo_debug mode")
        self.web_time = time.monotonic()

    def servo_op(
        self,
        action,
        target_id=None,
        new_id=None,
        kp=5,
        kd=20,
        *,
        protection_enabled=True,
        current_a=1.0,
    ):
        """在操作锁内校验并执行维护请求；修改 ID 时一并完成占用检查和读回验证。"""
        with self.servo_op_lock:
            if action != "set_id":
                return self._execute_servo_operation(
                    action,
                    target_id,
                    new_id,
                    kp,
                    kd,
                    protection_enabled=protection_enabled,
                    current_a=current_a,
                )
            target_id, new_id = int(target_id), int(new_id)
            if not 0 <= target_id <= 253 or not 0 <= new_id <= 253:
                raise ValueError("ID must be between 0 and 253")
            if not self._execute_servo_operation("ping", target_id)["ok"]:
                raise ValueError(
                    f"No servo replied at ID {target_id}; ID was not changed"
                )
            if target_id == new_id:
                return dict(
                    ok=True, op=SERVO_SET_ID, target_id=target_id, new_id=new_id
                )
            if self._execute_servo_operation("ping", new_id)["ok"]:
                raise ValueError(f"ID {new_id} is already in use; choose another ID")
            # 校验、总线写入和读回验证全程持有操作锁，防止维护请求交错。
            reply = self._execute_servo_operation(action, target_id, new_id, kp, kd)
            time.sleep(0.05)
            if not self._execute_servo_operation("ping", new_id)["ok"]:
                raise ValueError(
                    f"ID write sent, but ID {new_id} did not reply. Scan again"
                )
            if self._execute_servo_operation("ping", target_id)["ok"]:
                raise ValueError(
                    "Both old and new IDs replied. Connect only one servo and scan again"
                )
            return dict(reply, new_id=new_id)

    def _execute_servo_operation(
        self,
        action,
        target_id,
        new_id=None,
        kp=5,
        kd=20,
        *,
        protection_enabled=True,
        current_a=1.0,
    ):
        if action in ("set_protection", "set_protection_current"):
            if target_id is not None and target_id != BROADCAST_ID:
                raise ValueError(
                    "protection applies to all servos; target must be broadcast"
                )
            target_id = BROADCAST_ID
        else:
            target_id = int(target_id)
        with self.lock:
            self._require_servo_service_mode()
            if action == "unlock":
                op, payload = SERVO_UNLOCK, make_servo_request(SERVO_UNLOCK, target_id)
            elif action == "set_id":
                new_id = int(new_id)
                if not 0 <= new_id <= 253:
                    raise ValueError("invalid new id")
                op, payload = SERVO_SET_ID, make_servo_request(
                    SERVO_SET_ID, target_id, new_id=new_id
                )
            elif action == "reset_mid":
                op, payload = SERVO_RESET_MID, make_servo_request(
                    SERVO_RESET_MID, target_id
                )
            elif action == "set_gains":
                kp, kd = int(kp), int(kd)
                if not 0 <= kp <= 255 or not 0 <= kd <= 255:
                    raise ValueError("kp/kd out of range")
                op = SERVO_SET_PERM_KP_KD
                payload = make_servo_request(
                    SERVO_SET_PERM_KP_KD, target_id, kp=kp, kd=kd
                )
            elif action == "set_protection":
                op = SERVO_SET_PROTECTION
                payload = make_protection_request(protection_enabled)
            elif action == "set_protection_current":
                op = SERVO_SET_PROTECTION_CURRENT
                payload = make_current_limit_request(current_a)
            elif action == "read":
                op, payload = SERVO_READ, make_servo_request(SERVO_READ, target_id)
            elif action == "ping":
                op, payload = SERVO_PING, make_servo_request(SERVO_PING, target_id)
            elif action == "torque_off":
                op, payload = SERVO_ENTER, make_servo_request(SERVO_ENTER, target_id)
            else:
                raise ValueError("unknown servo action")
            self.servo_reply_event.clear()
            self.servo_reply = None
            self.servo_expected_reply = (op, target_id)
        try:
            self.robot.enqueue_servo_request(payload)
            return self._await_servo_reply(op, target_id=target_id)
        finally:
            with self.lock:
                self.servo_expected_reply = None

    def _await_servo_reply(self, op, timeout=0.5, target_id=None):
        """等待回复并核对操作码和目标 ID；超时或非探测操作失败时报告错误。"""
        if not self.servo_reply_event.wait(timeout):
            raise ValueError("servo reply timeout")
        with self.lock:
            reply = self.servo_reply
        if not reply or reply["op"] != op:
            raise ValueError("unexpected servo reply")
        if target_id is not None and reply["target_id"] != target_id:
            raise ValueError("unexpected servo reply ID")
        if not reply["ok"] and op != SERVO_PING:
            raise ValueError("servo operation failed")
        return reply

    def step(self):
        """完成一个控制周期，在提交动作前再次检查控制代次是否有效。"""
        with self.lock:
            fb, filt, generation, mode = (
                self.feedback,
                self.filtered,
                self.generation,
                self.mode,
            )
            age = time.monotonic() - self.rx_time
            if mode in ("hold", "policy"):
                lost = None
                if not fb or age > 0.15 or not fb.imu_ok:
                    lost = "sensor feedback lost; disarmed"
                elif time.monotonic() - self.web_time > 1.0:
                    # 浏览器无操作时仅将运动命令清零，不改变当前的扭矩模式。
                    self.twist = [0.0] * 3
                if lost:
                    self.mode = mode = "shadow"
                    self.generation += 1
                    self.arm_pending = False
                    self.last_error = lost
                    generation = self.generation
                    self.recovery.reset()
                    self.policy_needs_reset = True
                    self._stop_pick_locked()
                    self._stop_roulade_locked()
            elif mode in EXCLUSIVE_MODES:
                if time.monotonic() - self.web_time > 2.0:
                    previous = mode
                    self.mode = mode = "shadow"
                    self.generation += 1
                    self.last_error = "calibrate/servo_debug heartbeat lost; exited"
                    generation = self.generation
                    if previous == "calibrate":
                        self.cal_done = [False] * 15
                        self.cal_expected_index = None
                        self.cal_reply_event.set()
                        self.robot.enqueue_calibration_request(
                            make_calibration_request(CAL_EXIT)
                        )
                    else:
                        self.robot.enqueue_servo_request(
                            make_servo_request(SERVO_EXIT, 10)
                        )
                    self._send_disarm_command()
            twist, head, mouth = list(self.twist), list(self.head), self.mouth
            valid = bool(
                fb
                and age < 0.15
                and fb.imu_ok
                and filt
                and filt["imu_fusion_ready"]
                and filt["imu_fusion_updated"]
            )
            now = time.monotonic()
            if self.roulade_until and now >= self.roulade_until:
                self._stop_roulade_locked()
                self.recovery.fall_elapsed = 0.0
                self.policy_needs_reset = True
            rolling = self._roulade_is_active(now)
            if mode in ("shadow", "policy") and not rolling:
                changed = self.recovery.update(
                    filt["grav"] if filt else [0, 0, 0], now, valid
                )
                self.policy_needs_reset |= changed
                if changed and self.recovery.phase != "walk":
                    self._stop_pick_locked()
            phase = self.recovery.phase
            if rolling and phase != "walk":
                self._stop_roulade_locked()
                rolling = False
            picking = self.pick_active and phase == "walk" and not rolling
            if rolling:
                twist, head = [0.0] * 3, [0.0] * 4
                selected = self.roulade_policy
            elif picking:
                phi = self.pick_phi
                angle = 2.0 * math.pi * phi
                twist = [math.cos(angle), math.sin(angle), 0.0]
                head = [0.0] * 4
                mouth = MOUTH_MAX_DEG if phi < PICK_MOUTH_CLOSE_PHI else 0.0
                self.mouth = mouth
                selected = self.pick_policy
            else:
                selected = self.policy if phase == "walk" else self.getup_policy
            reset_policy = self.policy_needs_reset
            inferring = mode in ("shadow", "policy") and valid
            if inferring:
                self.policy_needs_reset = False
            exclusive = mode in EXCLUSIVE_MODES
        if exclusive:
            return
        target = self.policy.default_deg
        if inferring and phase == "default_pose":
            # 一秒姿态保持阶段使用起身模型的初始姿态，并将动作历史保持为零。
            with self.policy_lock:
                selected.reset_action_history()
                target = selected.action_to_joint_degrees([0.0] * 14)
        elif inferring:
            if phase == "getup":
                twist, head = [0.0] * 3, [0.0] * 4
            begin = time.monotonic()
            with self.policy_lock:
                if reset_policy:
                    selected.reset_action_history()
                    # 在新选中会话处理实际观测之前执行一次预热。
                    selected.warmup(1)
                action = selected.infer_action(
                    filt["grav"],
                    filt["gyro_rad"],
                    filt["angles_deg"],
                    filt["vels_deg"],
                    twist,
                    head,
                )
                target = selected.action_to_joint_degrees(action)
            finish = time.monotonic()
            with self.lock:
                self.inferred += 1
                self.infer_times.append(finish)
                self.infer_ms.append((finish - begin) * 1000)
                if picking:
                    self.pick_phi += CONTROL_PERIOD / PICK_PERIOD_S
                    if self.pick_phi >= 1.0:
                        self._stop_pick_locked()
                        self.policy_needs_reset = True
        with self.lock:
            # 若并发模式切换已使本周期失效，则丢弃其推理结果。
            if self.generation != generation:
                return
            mode = self.mode
            if mode in EXCLUSIVE_MODES:
                return
            command_mode = {
                "shadow": DISARM,
                "off": DISARM,
                "hold": HOLD,
                "policy": POLICY,
            }[mode]
            if self.arm_pending:
                command_mode = ARM
                # 首次使能的目标采用实测关节角度，而不是策略推理结果。
                target = list(fb.angles)
                self.arm_pending = False
            else:
                target = list(target)
                target[MOUTH_INDEX] = mouth
            if fb is None:
                return
            self.last_action = list(target)
            self.seq = (self.seq + 1) & 0xFFFFFFFF
            payload = command(self.seq, fb.sample_us, command_mode, target)
            self.robot.submit_command(payload)
            self.sent += 1

    def _warm_idle_policy(self):
        """对空闲会话执行一次占位推理，使未选中的策略保持预热状态。"""
        with self.lock:
            phase = self.recovery.phase
            rolling = self._roulade_is_active(time.monotonic()) and phase == "walk"
            picking = self.pick_active and phase == "walk" and not rolling
            if rolling:
                active = self.roulade_policy
            elif picking:
                active = self.pick_policy
            elif phase == "walk":
                active = self.policy
            else:
                active = self.getup_policy
            candidates = [p for p in self._all_motion_policies() if p is not active]
        if not candidates:
            return
        self._warm_index = (self._warm_index + 1) % len(candidates)
        idle = candidates[self._warm_index]
        if self.policy_lock.acquire(blocking=False):
            try:
                idle.warmup(1)
            finally:
                self.policy_lock.release()

    def run(self):
        """按控制周期调度策略，利用剩余时间预热未选中的会话。"""
        deadline = time.monotonic()
        while not self.stop_event.is_set():
            try:
                self.step()
            except Exception as exc:
                with self.lock:
                    self.last_error = str(exc)
                    self.mode = "off"
                    self.generation += 1
                    self.arm_pending = False
                    self._stop_pick_locked()
                    self._stop_roulade_locked()
            deadline += CONTROL_PERIOD
            now = time.monotonic()
            if deadline <= now:
                missed = int((now - deadline) / CONTROL_PERIOD) + 1
                self.loop_missed += missed
                deadline += missed * CONTROL_PERIOD
            else:
                # 仅利用本周期剩余的时间预热未激活会话。
                if deadline - now > 0.008:
                    try:
                        self._warm_idle_policy()
                    except Exception:
                        pass
            self.stop_event.wait(max(0.0, deadline - time.monotonic()))

    def stop(self):
        """请求控制循环结束，并发送卸载扭矩命令。"""
        self.stop_event.set()
        with self.lock:
            previous = self.mode
            self.mode = "off"
            self.generation += 1
            self.arm_pending = False
            if previous == "calibrate":
                self.cal_done = [False] * 15
                self.cal_expected_index = None
                self.cal_reply_event.set()
                self.robot.enqueue_calibration_request(
                    make_calibration_request(CAL_EXIT)
                )
            elif previous == "servo_debug":
                self.robot.enqueue_servo_request(make_servo_request(SERVO_EXIT, 10))
            self.seq += 1
            self.robot.submit_command(
                command(self.seq, 0, DISARM, self.policy.default_deg)
            )

    def status(self):
        """在状态锁内组织运行模式、反馈、推理和维护状态快照。"""
        now = time.monotonic()

        def hz(times):
            recent = [t for t in times if now - t < 5]
            if len(recent) < 2:
                return 0.0
            span = recent[-1] - recent[0]
            return (len(recent) - 1) / span if span > 1e-9 else 0.0

        def summary(samples):
            vals = sorted(samples)
            return (
                {
                    "mean": sum(vals) / len(vals),
                    "p99": vals[min(len(vals) - 1, int(len(vals) * 0.99))],
                    "max": vals[-1],
                }
                if vals
                else {}
            )

        with self.lock:
            fb = self.feedback
            result = dict(
                mode=self.mode,
                model=self.model_message,
                platform="Linux",
                hardware_loop_hz=100,
                feedback_hz=hz(self.rx_times),
                inference_hz=hz(self.infer_times),
                received=self.received,
                inferred=self.inferred,
                sent=self.sent,
                bad_frames=self.bad_frames,
                sequence_gaps=self.gaps,
                loop_missed=self.loop_missed,
                inference_ms=summary(self.infer_ms),
                receive_interval_ms=summary(self.rx_intervals),
                feedback_age_ms=(now - self.rx_time) * 1000 if fb else None,
                uptime_s=now - self.started,
                last_error=self.last_error,
                command=self.twist,
                head=self.head,
                mouth=self.mouth,
                policy_targets=self.last_action,
                motors=MOTOR_IDS,
                full_robot_ready=bool(
                    fb
                    and fb.imu_ok
                    and fb.mask == ALL_MASK
                    and now - self.rx_time < 0.15
                ),
                raw_pos=list(self.raw_pos),
                encoder_center=SERVO_CENTER_POS,
                cal_done=list(self.cal_done),
                cal_hold_pos=CAL_HOLD_POS,
                raw_mask=self.raw_mask,
            )
            result["control_ready"] = bool(
                fb
                and fb.imu_ok
                and now - self.rx_time < 0.15
                and self.mode not in EXCLUSIVE_MODES
            )
            result["servo_scan_version"] = 2
            result["roller"] = dict(
                enabled=self.roller_mode,
                model=self.roller_policy.file,
                walk_model=self.policy.file,
                default_deg=self.roller_policy.default_deg,
                action_scale=self.roller_policy.action_scale,
                action_alpha=self.roller_policy.action_alpha,
                action_alpha_head=self.roller_policy.action_alpha_head,
            )
            result["deployment"] = dict(
                reference="validated-control-contract",
                control_hz=50,
                home_deg=self.policy.home_deg.tolist(),
                default_deg=self.policy.default_deg,
                action_scale=self.policy.action_scale,
                action_alpha=self.policy.action_alpha,
                action_alpha_head=self.policy.action_alpha_head,
                gyro_alpha=self.filter.gyro_alpha,
                velocity_alpha=self.filter.vel_alpha,
                gravity_tau_s=self.filter.gravity_tau_s,
                imu_bias_deg=list(IMU_BIAS_DEG),
                filter_clock="host_receive",
                snapshot="latest",
            )
            result["recovery"] = self.recovery.status()
            result["recovery"]["active"] = self.mode in ("shadow", "policy")
            result["pick"] = dict(
                active=self.pick_active,
                phi=self.pick_phi,
                period_s=PICK_PERIOD_S,
                mouth_close_phi=PICK_MOUTH_CLOSE_PHI,
                model=self.pick_policy.file,
            )
            remain = max(0.0, self.roulade_until - now)
            result["roulade"] = dict(
                active=remain > 0,
                remaining_s=remain,
                seconds=ROULADE_S,
                model=self.roulade_policy.file,
            )
            if self.mode in ("shadow", "policy"):
                if remain > 0 and self.recovery.phase == "walk":
                    result["active_model"] = self.roulade_policy.file
                elif self.pick_active and self.recovery.phase == "walk":
                    result["active_model"] = self.pick_policy.file
                elif self.recovery.phase == "walk":
                    result["active_model"] = self.policy.file
                else:
                    result["active_model"] = self.getup_policy.file
            else:
                result["active_model"] = None
            if fb:
                result.update(
                    servo_ids=[
                        i for j, i in enumerate(MOTOR_IDS) if fb.mask & (1 << j)
                    ],
                    imu_ok=fb.imu_ok,
                    enabled=fb.enabled,
                    hardware_watchdog=bool(fb.flags & 4),
                    hardware_calibrating=fb.calibrating,
                    hardware_servo_debug=fb.servo_debugging,
                    hardware_seq=fb.seq,
                    hardware_period_us=fb.period_us,
                    hardware_work_us=fb.work_us,
                    hardware_overruns=fb.overruns,
                    hardware_report_drops=fb.report_drops,
                    hardware_invalid_commands=fb.invalid_commands,
                    hardware_command_seq=fb.command_seq,
                    hardware_command_age_us=fb.command_age_us,
                    angles=fb.angles,
                    velocities=fb.velocities,
                    targets=fb.targets,
                    acc=fb.acc,
                    gyro=fb.gyro,
                    gravity=self.filtered["grav"] if self.filtered else None,
                    missing_joint_ids=[
                        i for j, i in enumerate(MOTOR_IDS) if not fb.mask & (1 << j)
                    ],
                )
            return result
