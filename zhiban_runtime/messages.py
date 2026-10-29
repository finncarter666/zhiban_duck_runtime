"""定义控制线程与硬件线程之间的消息结构及边界校验。"""

from dataclasses import dataclass
import math

ALL_MASK = 0x7FFF
DISARM, ARM, POLICY, HOLD = range(4)
CAL_ENTER, CAL_EXIT, CAL_CENTER_ONE = range(3)
SERVO_CENTER_POS = 2047
CAL_HOLD_POS = 2048
SERVO_ENTER, SERVO_EXIT, SERVO_UNLOCK, SERVO_SET_ID = range(4)
SERVO_SET_PERM_KP_KD, SERVO_READ, SERVO_PING, SERVO_RESET_MID = range(5, 9)
SERVO_SET_PROTECTION = 9
SERVO_SET_PROTECTION_CURRENT = 10
BROADCAST_ID = 254
PROTECTION_CURRENT_UNIT_A = 0.0065
PROTECTION_DISABLED, PROTECTION_ENABLED = 0x04, 0x0D


@dataclass(frozen=True)
class RobotCommand:
    """携带模式、目标角度和有效期的不可变运动命令。"""

    seq: int
    source_us: int
    mode: int
    targets: tuple
    ttl_us: int = 250_000


@dataclass(frozen=True)
class CalRequest:
    """携带操作码、关节索引和序号的不可变校准请求。"""

    op: int
    index: int = 0
    seq: int = 0


@dataclass(frozen=True)
class ServoRequest:
    """携带目标 ID 和维护参数的不可变舵机服务请求。"""

    op: int
    target_id: int
    new_id: int = 0
    kp: int = 5
    kd: int = 20
    protection_flags: int = 0
    protection_current: int = 0


@dataclass(frozen=True)
class Feedback:
    """保存单次硬件循环的状态标志、传感器读数及执行统计。"""

    flags: int
    mask: int
    seq: int
    sample_us: int
    imu_us: int
    command_seq: int
    command_age_us: int
    period_us: int
    work_us: int
    overruns: int
    report_drops: int
    invalid_commands: int
    acc: tuple
    gyro: tuple
    angles: tuple
    velocities: tuple
    targets: tuple

    @property
    def imu_ok(self):
        return bool(self.flags & 1)

    @property
    def enabled(self):
        return bool(self.flags & 2)

    @property
    def calibrating(self):
        return bool(self.flags & 8)

    @property
    def servo_debugging(self):
        return bool(self.flags & 16)


def command(seq, source_us, mode, targets, ttl_us=250_000):
    """校验模式、15 个有限目标值和命令有效期，生成不可变运动请求。"""
    vals = tuple(float(v) for v in targets)
    if mode not in (DISARM, ARM, POLICY, HOLD):
        raise ValueError("invalid command mode")
    if len(vals) != 15 or not all(math.isfinite(v) for v in vals):
        raise ValueError("15 finite targets required")
    if (
        not isinstance(ttl_us, int)
        or isinstance(ttl_us, bool)
        or not 0 < ttl_us <= 250_000
    ):
        raise ValueError("invalid command lifetime")
    return RobotCommand(seq & 0xFFFFFFFF, source_us & 0xFFFFFFFF, mode, vals, ttl_us)


def make_calibration_request(op, index=0, seq=0):
    """校验校准操作与关节索引，并将请求序号限制为 32 位。"""
    if op not in (CAL_ENTER, CAL_EXIT, CAL_CENTER_ONE):
        raise ValueError("invalid calibration operation")
    if not isinstance(index, int) or isinstance(index, bool) or not 0 <= index < 15:
        raise ValueError("joint index out of range")
    return CalRequest(op, index, seq & 0xFFFFFFFF)


def make_servo_request(
    op, target_id, new_id=0, kp=5, kd=20, *, protection_flags=0, protection_current=0
):
    """将舵机维护参数转为整数并封装请求，不额外校验操作码。"""
    return ServoRequest(
        op,
        int(target_id),
        int(new_id),
        int(kp),
        int(kd),
        int(protection_flags),
        int(protection_current),
    )


def make_protection_request(enabled):
    """构造广播保护设置请求，根据开关选择对应的保护标志。"""
    return ServoRequest(
        SERVO_SET_PROTECTION,
        BROADCAST_ID,
        protection_flags=PROTECTION_ENABLED if enabled else PROTECTION_DISABLED,
    )


def make_current_limit_request(current_a):
    """将安培值量化为设备电流单位，校验 16 位范围后构造广播请求。"""
    raw = int(round(float(current_a) / PROTECTION_CURRENT_UNIT_A))
    if not 0 <= raw <= 65535:
        raise ValueError("protection current out of range")
    return ServoRequest(
        SERVO_SET_PROTECTION_CURRENT, BROADCAST_ID, protection_current=raw
    )


def unsigned_counter_delta(new, old):
    """计算 32 位无符号计数差，同时适用于时间戳和序号回绕。"""
    return (int(new) - int(old)) & 0xFFFFFFFF
