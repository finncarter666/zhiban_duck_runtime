# 运行时架构

## 模块边界

| 层 | 实现 | 职责 |
| --- | --- | --- |
| 入口 | `main.py`、`zhiban_runtime/cli.py` | 启动参数解析和服务启动 |
| HTTP | `application.py` | 路由、静态资源、线程生命周期和错误转译 |
| 控制 | `control/motion_control.py` | 接收反馈、模式切换、策略选择、校准和维护协调 |
| 信号 | `control/feedback_processing.py` | 时间戳有效性、反馈过滤与 IMU 轴向适配 |
| 恢复 | `control/posture_recovery.py` | 行走、默认姿态保持、起身之间的阶段转换 |
| 推理 | `inference/onnx_policy.py` | CPU ONNX 会话、元数据、预热、动作历史及滤波 |
| 数值 | `inference/policy_math.py` | 电机顺序、IMU 融合、观测组装与角度映射 |
| 硬件 | `hardware/device_runtime.py` | SCS 协议、QMI8658 采样、命令时效与扭矩输出 |
| 消息 | `messages.py` | 控制与硬件线程之间的不可变消息及输入校验 |

静态页面位于 `assets/web/`。模型和页面路径由 `paths.py` 相对仓库位置解析，不依赖启动时的工作目录。包的 `__init__.py` 不创建硬件或推理会话；调用应用工厂时会创建硬件后端，测试须注入或替换它。

## 命名与调用边界

- 控制器通过 `handle_feedback`、`handle_encoder_snapshot`、`handle_calibration_reply` 和 `handle_servo_reply` 接收硬件回调。
- `messages.py` 中的 `make_*_request` 负责构造维护请求；硬件层的 `enqueue_*_request` 负责入队，`_process_*_request` 负责执行，三者不混用。
- `_run_io_loop` 在硬件线程中执行完整循环；`read_joint_states` 和 `write_joint_targets` 分别处理舵机状态读取和目标写入。
- `configure_action_smoothing` 与 `configure_imu_filter` 分别负责动作滤波和 IMU 滤波参数的校验与更新。

## 时序与并发

- Web 页面每 50 ms 发送命令心跳。
- 控制线程目标周期为 20 ms；反馈回调和控制状态通过锁协调。
- 硬件线程目标周期为 10 ms；每轮处理维护请求、读取 IMU、消费最新命令、检查使能状态并发布反馈。
- 推理在状态锁之外执行，提交动作前检查控制代次，避免模式切换后发送旧推理结果。
- 首次 `ARM` 使用实测角度；超时后的普通 `POLICY` 命令不会隐式重使能。
- 运动命令采用单槽覆盖；维护操作采用队列。二者不能互换。

串口帧由 `hardware/serial_frames.py` 增量解析；循环耗时由 `hardware/loop_timing.py` 有界采样。HTTP 线程在取得控制器状态后计算耗时统计。详细字段见 `realtime.md`。

## 数值接口

观测为 `float32[61]`，模型输入增加 batch 维后为 `[1, 61]`：

| 切片 | 内容 |
| --- | --- |
| `[0:3]` | 角速度，rad/s |
| `[3:6]` | 机体系重力方向 |
| `[6:20]` | 策略顺序下的关节位置相对 home 偏移，rad |
| `[20:34]` | 策略顺序下的关节速度，rad/s |
| `[34:48]` | 上一时刻未经动作滤波的模型输出 |
| `[48:51]` | 三维运动命令 |
| `[51:55]` | 四维头部命令 |
| `[55:61]` | 保持为零 |

模型输出为 `[1, 14]`。动作映射后得到 15 个舵机目标；嘴部电机 34 不属于策略动作，目标范围为 0–30°。

动作滤波切片 `[0:5]` 和 `[10:14]` 使用腿部系数，`[5:10]` 使用头部系数；后者也包含电机 ID 20。参数名称不能替代实际索引定义，修改切片需要数值回归验证。

滚轮模型从 `default_joint_pos` 元数据读取 home，其他模型使用代码定义的默认姿态。ONNX 会话保持 CPU、单线程和顺序执行配置。

## 状态与动作

- `shadow`：推理但不使能；`off`：关闭扭矩；`hold`：默认姿态；`policy`：策略控制。
- `calibrate` 与 `servo_debug` 是独占维护模式，有独立进入条件和超时退出行为。
- 跌倒判定为倾角超过 55° 累计 0.15 s；默认姿态保持使用墙钟时间 1 s；起身后低于 15° 累计 1 s 回到行走。
- Pick 使用 4 s 相位周期；Roulade 使用 1.9 s 时长和临时滤波参数。动作受模式互斥和恢复状态约束。

## 平台与接口边界

硬件层依赖 Linux 串口和 I2C 接口，不调用特定板卡 SDK。`/api/status` 的 `platform` 为运行环境类别 `Linux`，不是主板自动识别结果。平台连接要求见 `hardware.md`。

HTTP 路由和启动参数是部署入口。内部 Python 模块不作为独立稳定 SDK 发布。修改控制周期、锁范围、模型、滤波、关节映射或安全策略时，应同步验证数值输出、协议与真实设备行为。
