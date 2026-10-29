# Zhiban Duck Runtime

面向 **Linux 平台** 的鸭子机器人本机运行时，通过串口访问 SCS 舵机总线、通过 I2C 读取 QMI8658，并使用 ONNX 策略执行动作推理。Web 工作台提供运动控制、舵机设置、鸭子标零与实时状态查看。

## 交流群

下面是**:Microduck技术爱好交流 1**的二维码（微信群满 200 人后无法扫码加入）。

<div align="center" style="text-align: center;">
  <img src="media/picture/wechat-group-1.png" alt=":Microduck技术爱好交流 1 微信二维码" width="280" style="width: 280px; max-width: 100%; height: auto;">
  <br>
  <sub><b>:Microduck技术爱好交流 1 · 二维码在 2026-11-04 前有效</b>（微信群码 7 天自动失效）<br>
  过期了请开个 <a href="https://github.com/finncarter666/zhiban_duck_runtime/issues">issue</a> 说一声，我会换上新的</sub>
</div>


## 功能

- 100 Hz 目标频率的硬件 I/O 循环与 50 Hz 策略控制循环。
- 行走、滚轮、Pick、Roulade 与姿态恢复策略。
- 15 个舵机的状态读取、目标写入、标零和维护。
- IMU 反馈、命令超时保护、显式使能与卸力。
- 桌面及移动端 Web 工作台，无需前端构建或外部 CDN。
- 循环耗时统计与只读频率验收工具。

## 环境要求

- Linux 系统，提供可访问的串口设备和 `/dev/i2c-*` 接口。
- Python 3.8 或更高版本；目标 Python 版本和 CPU 架构须能安装 `requirements.txt` 中的依赖。
- 兼容 SCS 协议的单线半双工舵机接口、15 个配置匹配的舵机和 QMI8658。
- 与模型匹配的机器人结构、电机方向、关节顺序及 IMU 安装方向。

项目不绑定特定主控板。通用 Linux 定位不代表任意板卡或机器人均已适配；部署前应检查设备驱动、依赖安装和控制时序。接线与平台要求见 `docs/hardware.md`。

## 安装与启动

在仓库根目录执行：

```bash
python3 -m venv .venv
. .venv/bin/activate
python -m pip install -r requirements.txt
python main.py --servo-device /dev/ttyUSB0 --i2c-bus 1
```

也可使用包入口：

```bash
python -m zhiban_runtime --servo-device /dev/ttyUSB0 --i2c-bus 1
```

浏览器访问 `http://<设备IP>:9527/`。初始模式为 `shadow`，执行推理但不使能扭矩。启动需要访问真实硬件；仅验证代码时请运行离线测试。

`requirements.txt` 统一包含运行、测试和格式检查依赖。源码部署须包含 `zhiban_runtime/`、`models/` 和 `assets/web/`；项目使用源码入口，不提供独立 wheel 安装方式。

### 启动参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `--servo-device` | `/dev/ttyUSB0` | 半双工舵机适配器设备路径 |
| `--servo-baud` | `1000000` | 舵机总线波特率 |
| `--servo-rs485` | 不启用 | 请求 pyserial RTS 方向控制，需驱动和适配器支持 |
| `--i2c-bus` | `1` | Linux I2C 总线编号 |
| `--imu-address` | 自动探测 | QMI8658 地址，可指定 `0x6a` 或 `0x6b` |
| `--host` | `0.0.0.0` | HTTP 监听地址 |
| `--port` | `9527` | HTTP 端口 |

## Web 工作台

| 功能 | 路径 | 内容 |
| --- | --- | --- |
| 实时总览 | `/` | 连接、模式、频率、舵机反馈与 IMU 状态 |
| 舵机控制 | `/#motion` | 双摇杆、键盘、头部与嘴部调节、卸力 |
| 舵机设置 | `/servo.html` | 扫描、选择舵机、中位与参数维护 |
| 鸭子标零 | `/calibrate.html` | 逐关节标零、保持与完成进度 |

## 运行安全

- **控制接口没有认证，不要暴露到公网或不可信网络。** 仅本机使用时可设置 `--host 127.0.0.1`。
- 首次上电和参数维护时固定或悬空机器人，准备可立即切断动力电源的手段。
- 首次使能由显式 `ARM` 完成，初始目标取实测角度。命令超时后的普通策略命令不会重新使能扭矩。
- 浏览器停止运动指令不等于关闭扭矩；需要卸力时使用卸力按钮并确认实际设备状态。
- 单个舵机缺失不会单独阻止其他舵机工作。使能前应确认全部 15 个舵机在线、IMU 有效。
- 打开标零页会自动进入标零并卸力。维护接口可写入设备参数，操作前须确认目标 ID 与接线。

## 实时性验收

硬件循环目标周期为 10 ms。`/api/status` 的 `hardware_timing` 提供实测频率、漏周期计数及 I/O、反馈回调和调度延迟统计。

服务预热至少 5 秒后执行：

```bash
python tools/check_control_rate.py --duration 60
```

工具仅读取状态，不启动运动。目标频率不代表实测频率，Linux 用户态循环也不提供硬实时保证；应在目标设备和实际负载下完成 `docs/realtime.md` 中的验收。

## 目录结构

```text
main.py                         # 源码启动入口
zhiban_runtime/
├── cli.py                      # 参数解析与启动
├── application.py              # HTTP 接口与生命周期
├── paths.py                    # 资源定位
├── messages.py                 # 命令、反馈和维护消息
├── control/                    # 运动控制、反馈处理和姿态恢复
├── inference/                  # ONNX 推理、观测组装和动作映射
└── hardware/                   # 舵机、IMU、串口解析与时序统计
assets/web/                     # Web 工作台和本地静态资源
models/                         # ONNX 模型与校验清单
tests/                          # 离线回归测试及测试基线
tools/                          # 只读运行检查工具
docs/                           # 部署、架构、操作与验收文档
requirements.txt                # 统一依赖清单
```

## 测试

```bash
python -m unittest discover -s tests -v
python -m black --check main.py zhiban_runtime tests tools
```

离线测试使用模拟硬件，覆盖模型推理、数值约定、消息协议、看门狗、串口解析、时序统计和 Web 接口，不替代真实设备验收。

## 文档

- `docs/hardware.md`：通用硬件要求与分平台连接说明。
- `docs/deployment.md`：安装、启动、设备权限和运行检查。
- `docs/web-interface.md`：工作台操作和数据说明。
- `docs/architecture.md`：模块职责、并发和数值接口。
- `docs/realtime.md`：100 Hz 时序、诊断字段和性能验收。
- `docs/validation.md`：离线测试与上机验收规范。


## 许可

仓库目前未指定项目级许可证。第三方代码及模型的来源、许可与分发授权信息尚未完整记录；本 README 不构成授权声明。
