# Adora Nano Feetech Kiwi 底盘 Dora 节点

将原 Adora Nano 二维导航项目中的 Rust 底盘节点接入 DORA-NAV。
通过 `feetech-servo-sdk` 访问飞特舵机串口总线，控制三轮全向底盘并发布里程计。
节点使用官方 Rust `dora-node-api`，固定版本为 `1.1.0`，无 ROS 依赖。

```text
feetech_kiwi_chassis_dora_node/
├── CMakeLists.txt
├── README.md
└── sdk_node/
    ├── Cargo.toml
    ├── Cargo.lock
    └── src/main.rs
```

## 构建

需要 Linux、Rust 1.95 或更新工具链，以及 Dora CLI 1.1.0。根工程中的其他 C++ 节点仍使用原有 Dora C SDK。
`Cargo.lock` 固定 Rust 依赖；首次构建需要获取 crates.io 依赖，或事先准备离线缓存。
`feetech-servo-sdk` 由 Cargo 管理，无需另行安装 ROS 驱动。

在 DORA-NAV 根目录构建 Adora Nano 二维建图节点：

```bash
cmake -S . -B build \
  -DBUILD_YDLIDAR_TMINI_PLUS=ON -DBUILD_KARTO_2D=ON \
  -DBUILD_FEETECH_KIWI_CHASSIS=ON -DBUILD_RANGER_CHASSIS=OFF \
  -DBUILD_LIVOX_DRIVER=OFF -DBUILD_RSLIDAR_DRIVER=OFF \
  -DBUILD_LIGHTNING_LM_MAPPING=OFF -DBUILD_HDL_GRAPH_SLAM=OFF \
  -DBUILD_LOCALIZATION=OFF -DBUILD_GLOBAL_PLANNER=OFF \
  -DBUILD_LOCAL_PLANNER=OFF -DBUILD_GOAL_PUBLISHER=OFF -DBUILD_VISUALIZATION=OFF
cmake --build build --target ydlidar_tmini_plus_dora_node \
  feetech_kiwi_chassis_dora_node karto_slam_dora_node -j4
```

`BUILD_FEETECH_KIWI_CHASSIS` 默认关闭，需要 `BUILD_CHASSIS=ON`。
`BUILD_RANGER_CHASSIS` 默认开启；上述示例关闭 Ranger，只构建 Adora Nano 所需底盘。
Cargo 中间产物位于构建目录内，不写入源码目录。

独立使用 Cargo 时，保持产物位于根构建目录：

```bash
CARGO_TARGET_DIR="$(pwd)/build/feetech-cargo" \
cargo build --release --locked \
  --manifest-path modules/chassis/feetech_kiwi_chassis_dora_node/sdk_node/Cargo.toml
```

## 数据流与接口

配置并检查设备参数后，在 DORA-NAV 根目录运行：

```bash
dora run apps/mapping_2d_adora_nano.yml
```

该数据流连接 YDLIDAR、Adora Nano 反馈里程计和 Karto，使用定时输入更新里程计。
示例不接入速度指令源；后续可将键盘或现有控制器输出连接到底盘的 `cmd_vel` 输入。

| 端口 | 内容 |
|---|---|
| 输入 `tick` | Dora 定时输入，示例周期 50 ms |
| 输入 `cmd_vel` | JSON Twist，读取 `linear.x`、`linear.y`、`angular.z`；兼容原输入名 `CmdVelTwist` |
| 输出 `Odometry` | JSON，包含 Unix 时钟 `header.stamp.sec/nanosec`、嵌套 `pose.pose` 和 `twist.twist`，Karto 已支持该格式 |

速度指令示例：

```json
{"linear":{"x":0.1,"y":0.0},"angular":{"z":0.0}}
```

指令超时后发送零速度，Dora Stop 时尝试停止所有轮子。
速度写入或超时停轮失败时，节点再次尝试停止全部轮子，并以非零状态退出。
`feedback` 模式读取反馈失败时，该次发布速度归零，避免沿用旧速度继续积分。

## 设备与标定参数

参数在 `apps/mapping_2d_adora_nano.yml` 的底盘节点 `env` 中设置。

| 环境变量 | 默认值 | 说明 |
|---|---|---|
| `SERIAL_PORT` | `/dev/ttyUSB0` | 舵机总线串口；数据流示例使用 `/dev/ttyUSB1`，需改为实际设备路径 |
| `SERIAL_BAUD` | `1000000` | 串口波特率 |
| `WHEEL_IDS` | `9,8,7` | 按轮子角度顺序排列的三个不同舵机 ID |
| `WHEEL_ANGLES_DEG` | `60,180,300` | 三个轮子的安装角度，单位度 |
| `LINEAR_TICKS_PER_MPS` | `13350` | 线速度标定系数，来自原项目 |
| `ANGULAR_TICKS_PER_RADPS` | `1660` | 角速度标定系数，来自原项目 |
| `SPEED_LIMIT` | `1500` | 单轮速度寄存器幅值上限，要求有限且大于 0、不超过 32767 |
| `ODOM_SOURCE` | `feedback` | `feedback` 使用速度反馈；`command` 使用指令估计；`auto` 优先反馈，读取失败时保留指令估计 |
| `CMD_VEL_TIMEOUT_SEC` | `0.35` | 速度指令超时，依赖持续 `tick` 输入 |
| `PID_ENABLED` | `0` | 可选轮速 PID；启用时用 `PID_KP`、`PID_KI`、`PID_KD` 设置系数 |

舵机 ID 必须互不相同且处于 0–253。角度与标定系数须有限，并能组成可逆的运动学矩阵；
非法配置在电机初始化前拒绝。
串口和轮速标定值需要按实际机器人设置；雷达与底盘必须使用不同的设备端口。
原 `apps/mapping_2d.yml` 继续使用 Ranger，Adora Nano 使用独立数据流。
