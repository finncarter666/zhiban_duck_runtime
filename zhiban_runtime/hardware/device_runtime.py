"""在专用 Linux I/O 循环中驱动 SCS 舵机总线并采样 QMI8658。"""

from __future__ import annotations

import queue
import select
import struct
import threading
import time
from dataclasses import dataclass
from typing import Optional

from zhiban_runtime.hardware.loop_timing import LoopTiming
from zhiban_runtime.hardware.serial_frames import ReplyBuffer
from zhiban_runtime.inference.policy_math import MOTOR_IDS
from zhiban_runtime.messages import (
    ARM,
    BROADCAST_ID,
    CAL_CENTER_ONE,
    CAL_ENTER,
    CAL_EXIT,
    CAL_HOLD_POS,
    DISARM,
    Feedback,
    PROTECTION_CURRENT_UNIT_A,
    PROTECTION_ENABLED,
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
    unsigned_counter_delta,
)

SERVO_BAUD = 1_000_000
TICK_S = 0.010
REPLY_S = 0.006
CENTER = SERVO_CENTER_POS
MOTOR_SIGN = (-1.0,) * 15
ADDR_GOAL_POS, ADDR_PRESENT, PRESENT_LEN = 0x2A, 0x38, 6
ADDR_TORQUE, ADDR_TEMP_KP, ADDR_TEMP_KD = 40, 50, 51
ADDR_PERM_KP_KD, ADDR_UNLOCK, ADDR_ID = 0x15, 0x37, 0x05
ADDR_UNLOAD_CONDITION, ADDR_PROTECTION_CURRENT = 0x13, 0x1C


def _monotonic_us():
    """将单调时钟转换为微秒，保留低 32 位以匹配消息计时约定。"""
    return (time.monotonic_ns() // 1000) & 0xFFFFFFFF


def _encode_servo_packet(sid, instruction, data=b""):
    """组装 SCS 帧头、载荷长度和按字节求和后取反的校验值。"""
    body = bytes((sid, len(data) + 2, instruction)) + bytes(data)
    return b"\xff\xff" + body + bytes(((~sum(body)) & 0xFF,))


class SCSBus:
    """封装半双工 SCS 总线的读写、同步命令和设备维护操作。"""

    def __init__(self, device, baud=SERVO_BAUD, rs485=False):
        import serial

        self.serial = serial.Serial(
            device, baudrate=baud, timeout=0, write_timeout=0.02
        )
        if rs485:
            from serial.rs485 import RS485Settings

            self.serial.rs485_mode = RS485Settings(
                rts_level_for_tx=True, rts_level_for_rx=False
            )

    def close(self):
        self.serial.close()

    def drain(self):
        self.serial.reset_input_buffer()

    def write(self, sid, instruction, data=b""):
        self.serial.write(_encode_servo_packet(sid, instruction, data))
        self.serial.flush()

    def write_byte(self, sid, addr, value):
        self.write(sid, 0x03, bytes((addr, value & 255)))

    def write_word(self, sid, addr, value):
        self.write(sid, 0x03, bytes((addr, value & 255, value >> 8)))

    def torque(self, enabled):
        self.write_byte(0xFE, ADDR_TORQUE, int(enabled))

    def center_and_hold(self, sid):
        """重置单个关节中位，写入目标后使能扭矩，并核对寄存器回读。"""
        reply = self.request(sid, 1)
        if not reply or reply[3] != 2 or reply[4] != 0:
            raise OSError("servo did not acknowledge calibration")
        for _ in range(5):
            self.write(sid, 0x0B)
            time.sleep(0.01)
        # 目标、运动时间和速度一起写入，避免沿用上一次运动的目标参数。
        self.write(
            sid,
            3,
            bytes((ADDR_GOAL_POS, CAL_HOLD_POS & 255, CAL_HOLD_POS >> 8, 0, 0, 0, 0)),
        )
        self.write_byte(sid, ADDR_TORQUE, 1)
        for address, expected in (
            (ADDR_GOAL_POS, bytes((CAL_HOLD_POS & 255, CAL_HOLD_POS >> 8))),
            (ADDR_TORQUE, b"\x01"),
        ):
            reply = self.request(sid, 2, bytes((address, len(expected))))
            if (
                not reply
                or reply[3] != len(expected) + 2
                or reply[4] != 0
                or reply[5:-1] != expected
            ):
                raise OSError("servo hold register verification failed")

    def apply_motion_gains(self):
        """写入运动模式的临时增益，并单独设置嘴部舵机的比例增益。"""
        self.write_byte(0xFE, ADDR_TEMP_KP, 6)
        self.write_byte(0xFE, ADDR_TEMP_KD, 20)
        self.write_byte(34, ADDR_TEMP_KP, 10)

    def apply_calibration_gains(self):
        """写入校准模式使用的临时增益，不修改持久化参数。"""
        self.write_byte(0xFE, ADDR_TEMP_KP, 3)
        self.write_byte(0xFE, ADDR_TEMP_KD, 0)

    def write_joint_targets(self, angles):
        """按电机顺序和方向将角度转为编码器目标，再广播同步写入。"""
        data = bytearray((ADDR_GOAL_POS, 6))
        for sid, angle, sign in zip(MOTOR_IDS, angles, MOTOR_SIGN):
            raw = max(0, min(4095, CENTER + int(float(angle) / 360.0 * 4095.0 * sign)))
            data += bytes((sid, raw & 255, raw >> 8, 0, 0, 0, 0))
        self.write(0xFE, 0x83, data)

    def _collect_reply_frames(self, deadline, expected_ids=None, expected_length=None):
        """增量收帧；反馈收齐即返回，缺帧仍沿用原截止时间。"""
        parser = ReplyBuffer(expected_length)
        remaining = set(expected_ids) if expected_ids is not None else None
        frames = []
        while time.monotonic() < deadline:
            chunk = self.serial.read(self.serial.in_waiting or 1)
            if chunk:
                for frame in parser.feed(chunk):
                    frames.append(frame)
                    # 错误状态、重复回复和无关 ID 都不能充当缺失关节的反馈。
                    if remaining is not None and frame[4] == 0:
                        remaining.discard(frame[2])
                if remaining is not None and not remaining:
                    break
            else:
                remaining_s = deadline - time.monotonic()
                if remaining_s > 0:
                    # Linux 串口可等待可读事件，避免每 100 us 唤醒并查询一次。
                    select.select([self.serial.fileno()], [], [], remaining_s)
        return frames

    def read_joint_states(self):
        """同步读取已配置舵机的位置和速度，并解码速度的符号位。"""
        self.drain()
        ids = bytes(MOTOR_IDS)
        # 同步读取载荷依次包含寄存器地址、读取字节数和目标舵机 ID。
        body = bytes((0xFE, len(ids) + 4, 0x82, ADDR_PRESENT, PRESENT_LEN)) + ids
        self.serial.write(b"\xff\xff" + body + bytes(((~sum(body)) & 255,)))
        self.serial.flush()
        result = {}
        for f in self._collect_reply_frames(
            time.monotonic() + REPLY_S, MOTOR_IDS, PRESENT_LEN + 2
        ):
            if f[3] == 8 and f[4] == 0 and f[2] in MOTOR_IDS:
                pos = f[5] | (f[6] << 8)
                speed = f[7] | (f[8] << 8)
                speed = -(speed & 0x7FFF) if speed & 0x8000 else speed
                result[f[2]] = (pos, speed)
        return result

    def request(self, sid, instruction, data=b"", timeout=REPLY_S):
        self.drain()
        self.write(sid, instruction, data)
        frames = self._collect_reply_frames(time.monotonic() + timeout)
        return next((f for f in frames if f[2] == sid), None)

    def unlock(self, sid):
        for _ in range(3):
            self.write_byte(sid, ADDR_UNLOCK, 0)
            time.sleep(0.01)


class QMI8658:
    """探测并初始化 IMU，以运行时约定的机体系返回传感器数据。"""

    def __init__(self, bus_number=1, address=None):
        from smbus2 import SMBus

        self.bus = SMBus(bus_number)
        self.addr = address
        self.ok = False
        self._initialize_sensor()

    def close(self):
        self.bus.close()

    def _read(self, reg, n):
        return self.bus.read_i2c_block_data(self.addr, reg, n)

    def _write(self, reg, value):
        self.bus.write_byte_data(self.addr, reg, value)

    def _initialize_sensor(self):
        """探测指定或候选地址，完成设备识别、复位确认和寄存器配置。"""
        addresses = (self.addr,) if self.addr is not None else (0x6A, 0x6B)
        self.addr = None
        for addr in addresses:
            try:
                if self.bus.read_byte_data(addr, 0x00) == 0x05:
                    self.addr = addr
                    break
            except OSError:
                pass
        if self.addr is None:
            return
        self._write(0x60, 0xB0)
        time.sleep(0.02)
        if self.bus.read_byte_data(self.addr, 0x4D) != 0x80:
            return
        for reg, value in (
            (0x08, 0),
            (0x02, 0x40),
            (0x03, 0x04),
            (0x04, 0x64),
            (0x06, 0),
            (0x08, 3),
        ):
            self._write(reg, value)
        time.sleep(0.25)
        self.ok = True

    def read(self):
        """读取加速度与角速度并换算单位；无设备或读取失败时返回无效样本。"""
        if self.addr is None:
            return False, (0.0,) * 3, (0.0,) * 3
        try:
            data = self._read(0x35, 12)
        except OSError:
            self.ok = False
            return False, (0.0,) * 3, (0.0,) * 3
        vals = struct.unpack("<6h", bytes(data))
        ax, ay, az, gx, gy, gz = vals
        # 将传感器坐标转换为反馈融合使用的机体系约定。
        acc_raw = (az, ax, -ay)
        gyro_raw = (-gz, -gx, gy)
        self.ok = True
        return (
            True,
            tuple(v * 9.8 / 16384.0 for v in acc_raw),
            tuple(v / 32.0 for v in gyro_raw),
        )


@dataclass
class HardwareConfig:
    """保存串口和 I2C 参数；配置对象本身不打开设备。"""

    servo_device: str = "/dev/ttyUSB0"
    servo_baud: int = SERVO_BAUD
    servo_rs485: bool = False
    i2c_bus: int = 1
    imu_address: Optional[int] = None


class RobotHardware:
    """管理 100 Hz 硬件线程、最新命令交接和扭矩超时保护。"""

    def __init__(self, config=None, *, servo=None, imu=None):
        config = config or HardwareConfig()
        self.config = config
        self.servo = (
            servo
            if servo is not None
            else SCSBus(config.servo_device, config.servo_baud, config.servo_rs485)
        )
        self.imu = (
            imu if imu is not None else QMI8658(config.i2c_bus, config.imu_address)
        )
        self.on_feedback = self.on_raw = self.on_cal_reply = self.on_servo_reply = (
            lambda *_: None
        )
        self._lock = threading.Lock()
        self._latest = None
        self._pending = False
        self._services = queue.SimpleQueue()
        self._stop = threading.Event()
        self._thread = None
        self.enabled = False
        self._bus_torque = False
        self.host_mode = "normal"
        self.seq = self.overruns = self.invalid = 0
        self.command_seq = 0
        self._have_command = False
        self._last_command_at = 0.0
        self._ttl_s = 0.25
        self._last_healthy_at = 0.0
        self.timed_out = False
        self.last_targets = [0.0] * 15
        self.last_sample_us = _monotonic_us()
        self.raw_pos = [CENTER] * 15
        self.cal_held_mask = 0
        self.timing = LoopTiming(TICK_S)

    def set_callbacks(self, *, feedback, raw, cal_reply, servo_reply):
        self.on_feedback, self.on_raw = feedback, raw
        self.on_cal_reply, self.on_servo_reply = cal_reply, servo_reply

    def start(self):
        self.servo.torque(False)
        self._bus_torque = False
        self.servo.apply_motion_gains()
        self._thread = threading.Thread(
            target=self._run, name="zhiban-hardware100", daemon=True
        )
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2)
        try:
            self.servo.torque(False)
        finally:
            self.servo.close()
            self.imu.close()

    def submit_command(self, cmd):
        """覆盖尚未消费的运动命令，不将过时目标排入队列。"""
        with self._lock:
            self._latest, self._pending = cmd, True

    def enqueue_calibration_request(self, req):
        """将校准请求放入服务队列，由硬件线程按顺序处理。"""
        self._services.put(("cal", req))

    def enqueue_servo_request(self, req):
        """将舵机维护请求放入服务队列，由硬件线程按顺序处理。"""
        self._services.put(("servo", req))

    def _process_calibration_request(self, req):
        """处理校准进入、退出或单关节居中，并在居中操作后回复结果。"""
        ok = True
        if req.op == CAL_ENTER:
            self.enabled = False
            self.timed_out = False
            self.host_mode = "calibrate"
            self.cal_held_mask = 0
            self.servo.torque(False)
            self._bus_torque = False
            self.servo.apply_calibration_gains()
        elif req.op == CAL_EXIT:
            self.enabled = False
            self.timed_out = False
            self.host_mode = "normal"
            self.cal_held_mask = 0
            self.servo.torque(False)
            self._bus_torque = False
            self.servo.apply_motion_gains()
        elif req.op == CAL_CENTER_ONE and self.host_mode == "calibrate":
            sid = MOTOR_IDS[req.index]
            try:
                self.servo.center_and_hold(sid)
            except Exception:
                ok = False
                self.cal_held_mask &= ~(1 << req.index)
                # 写入或回读失败时仅卸力当前关节，不影响其他已确认的保持关节。
                try:
                    self.servo.write_byte(sid, ADDR_TORQUE, 0)
                except Exception:
                    pass
            else:
                self.cal_held_mask |= 1 << req.index
        else:
            ok = False
        if req.op == CAL_CENTER_ONE:
            self.on_cal_reply({"seq": req.seq, "index": req.index, "ok": ok})

    def _process_servo_request(self, req):
        """执行舵机维护请求，将结果或捕获的失败反馈给控制层。"""
        ok, raw, kp, kd = True, CENTER, 0, 0
        try:
            if req.op == SERVO_ENTER:
                self.enabled = False
                self.host_mode = "servo_debug"
                self.servo.torque(False)
                self._bus_torque = False
            elif req.op == SERVO_EXIT:
                self.host_mode = "normal"
                self.servo.torque(False)
                self._bus_torque = False
                self.servo.apply_motion_gains()
            elif req.op == SERVO_UNLOCK:
                self.servo.unlock(req.target_id)
            elif req.op == SERVO_SET_ID:
                self.servo.unlock(req.target_id)
                for _ in range(3):
                    self.servo.write_byte(req.target_id, ADDR_ID, req.new_id)
                    time.sleep(0.01)
            elif req.op == SERVO_RESET_MID:
                for _ in range(5):
                    self.servo.write(req.target_id, 0x0B)
                    time.sleep(0.01)
            elif req.op == SERVO_SET_PERM_KP_KD:
                self.servo.unlock(req.target_id)
                for _ in range(3):
                    self.servo.write(
                        req.target_id, 3, bytes((ADDR_PERM_KP_KD, req.kp, req.kd))
                    )
                    time.sleep(0.01)
                kp, kd = req.kp, req.kd
            elif req.op == SERVO_SET_PROTECTION:
                self.servo.unlock(BROADCAST_ID)
                self.servo.write_byte(
                    BROADCAST_ID, ADDR_UNLOAD_CONDITION, req.protection_flags
                )
            elif req.op == SERVO_SET_PROTECTION_CURRENT:
                self.servo.unlock(BROADCAST_ID)
                self.servo.write_word(
                    BROADCAST_ID, ADDR_PROTECTION_CURRENT, req.protection_current
                )
            elif req.op in (SERVO_READ, SERVO_PING):
                instruction = 1 if req.op == SERVO_PING else 2
                data = b"" if instruction == 1 else bytes((ADDR_PRESENT, PRESENT_LEN))
                frame = self.servo.request(req.target_id, instruction, data)
                ok = frame is not None
                if frame and req.op == SERVO_READ and len(frame) >= 8:
                    raw = frame[5] | (frame[6] << 8)
                    gains = self.servo.request(
                        req.target_id, 2, bytes((ADDR_PERM_KP_KD, 2))
                    )
                    if gains and gains[3] == 4 and gains[4] == 0:
                        kp, kd = gains[5], gains[6]
                    else:
                        ok = False
            else:
                ok = False
        except Exception:
            ok = False
        reply = {"op": req.op, "target_id": req.target_id, "ok": ok}
        if req.op == SERVO_SET_PROTECTION:
            reply.update(
                broadcast=True,
                protection_enabled=req.protection_flags == PROTECTION_ENABLED,
                protection_flags=req.protection_flags,
            )
        elif req.op == SERVO_SET_PROTECTION_CURRENT:
            reply.update(
                broadcast=True,
                current_raw=req.protection_current,
                current_a=req.protection_current * PROTECTION_CURRENT_UNIT_A,
            )
        else:
            reply.update(raw_pos=raw, center_pos=CENTER, kp=kp, kd=kd)
        self.on_servo_reply(reply)

    def _run(self):
        """执行硬件循环，并确保退出路径尝试关闭扭矩。"""
        try:
            self._run_io_loop()
        finally:
            self.enabled = False
            self._bus_torque = False
            try:
                self.servo.torque(False)
            except Exception:
                pass

    def _run_io_loop(self):
        """循环处理服务队列、检查命令时效并发布反馈，直到收到停止信号。"""
        deadline = time.monotonic()
        previous = deadline
        while not self._stop.is_set():
            start = time.monotonic()
            start_us = _monotonic_us()
            period_us = int((start - previous) * 1e6)
            previous = start
            while True:
                try:
                    kind, req = self._services.get_nowait()
                except queue.Empty:
                    break
                (
                    self._process_calibration_request
                    if kind == "cal"
                    else self._process_servo_request
                )(req)
            services_done = time.monotonic()
            imu_ok, acc, gyro = self.imu.read()
            imu_done = time.monotonic()
            if imu_ok:
                self._last_healthy_at = start
            with self._lock:
                cmd, got = self._latest, self._pending
                self._pending = False
            if self.host_mode == "normal" and got:
                source_age = unsigned_counter_delta(start_us, cmd.source_us)
                fresh = cmd.mode == DISARM or source_age <= 150_000
                delta = (cmd.seq - self.command_seq) & 0xFFFFFFFF
                ordered = (
                    not self._have_command
                    or cmd.mode == DISARM
                    or (delta != 0 and delta < 0x80000000)
                )
                if fresh and ordered:
                    self.command_seq = cmd.seq
                    self._have_command = True
                    self._last_command_at = start
                    self._ttl_s = cmd.ttl_us / 1e6
                    self.last_targets = list(cmd.targets)
                    if cmd.mode == DISARM:
                        self.enabled = False
                        self.timed_out = False
                    elif cmd.mode == ARM and not self.enabled and imu_ok:
                        self.servo.apply_motion_gains()
                        self.enabled = True
                        self.timed_out = False
                else:
                    self.invalid += 1
            if self.host_mode != "normal":
                self.enabled = False
                self.timed_out = False
            elif self.enabled and (
                start - self._last_command_at > self._ttl_s
                or start - self._last_healthy_at > 0.1
            ):
                self.enabled = False
                self.timed_out = True
            desired_torque = self.enabled and self.host_mode == "normal"
            if desired_torque != self._bus_torque:
                self.servo.torque(desired_torque)
                self._bus_torque = desired_torque
            if self.enabled:
                self.servo.write_joint_targets(self.last_targets)
            write_done = time.monotonic()
            states = self.servo.read_joint_states()
            read_done = time.monotonic()
            mask = 0
            angles, velocities = [0.0] * 15, [0.0] * 15
            for i, (sid, sign) in enumerate(zip(MOTOR_IDS, MOTOR_SIGN)):
                if sid not in states:
                    continue
                mask |= 1 << i
                pos, vel = states[sid]
                self.raw_pos[i] = pos
                angles[i] = (pos - CENTER) * 360.0 / 4095.0 * sign
                velocities[i] = vel * 50.0 * (360.0 / 4096.0) * sign
            self.seq = (self.seq + 1) & 0xFFFFFFFF
            self.last_sample_us = start_us
            flags = int(imu_ok) | (int(self.enabled) << 1) | (int(self.timed_out) << 2)
            flags |= (int(self.host_mode == "calibrate") << 3) | (
                int(self.host_mode == "servo_debug") << 4
            )
            work_us = int((time.monotonic() - start) * 1e6)
            fb = Feedback(
                flags,
                mask,
                self.seq,
                start_us,
                start_us,
                self.command_seq,
                (
                    int((start - self._last_command_at) * 1e6)
                    if self._have_command
                    else 0xFFFFFFFF
                ),
                period_us,
                work_us,
                self.overruns,
                0,
                self.invalid,
                tuple(acc),
                tuple(gyro),
                tuple(angles),
                tuple(velocities),
                tuple(self.last_targets),
            )
            self.on_feedback(fb)
            self.on_raw(
                {
                    "pos": tuple(self.raw_pos),
                    "mask": mask,
                    "held_mask": self.cal_held_mask,
                }
            )
            publish_done = time.monotonic()
            wakeup_s = max(0.0, start - deadline)
            deadline += TICK_S
            now = time.monotonic()
            if deadline <= now:
                missed = int((now - deadline) / TICK_S) + 1
                self.overruns += missed
                deadline += missed * TICK_S
            self.timing.record(
                start,
                period_us / 1e6 if self.seq != 1 else 0.0,
                wakeup_s,
                self.host_mode,
                (
                    services_done - start,
                    imu_done - services_done,
                    write_done - imu_done,
                    read_done - write_done,
                    publish_done - read_done,
                ),
                self.overruns,
            )
            self._stop.wait(max(0.0, deadline - time.monotonic()))
