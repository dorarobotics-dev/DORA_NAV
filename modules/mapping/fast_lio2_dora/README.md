# FAST-LIO2 Dora 建图模块

以 Rust `fast-lio` 为算法核心，通过原生 `dora-node-api = 1.1.0` 接收 Livox 三维点云和 IMU。
组织方式与 Lightning 一致：算法核心、Dora 节点入口、参数文件和数据流分别存放。
运行不需要 ROS、PCL 或 Python；首次 Cargo 构建需要下载依赖或准备好 Cargo 缓存。

```text
fast_lio2_dora/
├── CMakeLists.txt
├── Cargo.toml / Cargo.lock
├── config/config.yaml
├── dataflow.yml
└── src/
    ├── main.rs              # Dora 生命周期、同步及算法调用
    ├── config.rs            # YAML 配置及检查
    ├── wire.rs              # 输入输出协议
    ├── global_map.rs        # 世界坐标体素地图及 PCD 保存
    └── core/fast_lio/       # 内置 Rust FAST-LIO2 核心库
```

## 构建与启动

需要 Rust/Cargo 1.95 或更新版本、Dora CLI 1.1.0。Livox 驱动沿用根工程的 Dora C SDK 和
Livox SDK2，需要先按仓库原有安装说明准备这两个 SDK。FAST-LIO2 Rust 节点本身不链接 C SDK。

在仓库根目录运行：

```bash
cmake -S . -B build \
  -DBUILD_FAST_LIO2=ON -DBUILD_LIVOX_DRIVER=ON \
  -DBUILD_RSLIDAR_DRIVER=OFF -DBUILD_YDLIDAR_TMINI_PLUS=OFF \
  -DBUILD_CHASSIS=OFF -DBUILD_LOCALIZATION=OFF \
  -DBUILD_LIGHTNING_LM_MAPPING=OFF -DBUILD_HDL_GRAPH_SLAM=OFF -DBUILD_KARTO_2D=OFF \
  -DBUILD_GLOBAL_PLANNER=OFF -DBUILD_LOCAL_PLANNER=OFF \
  -DBUILD_GOAL_PUBLISHER=OFF -DBUILD_VISUALIZATION=OFF
cmake --build build --target fast_lio2_dora_node livox_dora_driver -j4
dora run apps/mapping_fast_lio2.yml
```

`BUILD_FAST_LIO2` 默认关闭，启用时需要 `BUILD_MAPPING=ON`。
也可单独构建 Rust 节点：`cargo build --release --locked --manifest-path modules/mapping/fast_lio2_dora/Cargo.toml`。
单独使用模块内 `dataflow.yml` 时，仍需先按以上根工程命令构建。

首次使用设备前，编辑 Livox 的 `MID360_config.json` 网络地址，以及本模块配置中的
LiDAR→IMU 外参。默认单位矩阵和零平移只是占位值。初始化时保持传感器静止至少两秒。
首轮数据流支持一个 Livox 设备，要求 LiDAR/IMU 使用 PTP 或 GPS 同步时间；
未同步数据会被驱动拒绝，并由建图节点的超时检查报告失败。
驱动 JSON 中的 `extrinsic_parameter` 必须保持零值，外参只在 FAST-LIO2 YAML 中配置，避免重复变换。
该数据流只启动雷达和建图，不包含底盘节点或速度指令。

## 配置

`FAST_LIO2_CONFIG_FILE` 指定本模块 YAML；数据流已设置该变量，避免与其他节点的配置变量冲突。
地图保存路径相对于所选 YAML 的目录解析。参数名拼写错误或无效数值会在启动时明确报错。

- `imu.acceleration_scale`：已有 Livox 驱动输出加速度单位为 g，默认乘 `9.80665`；SI 数据源设为 `1.0`。
- `imu.time_offset_lidar_to_imu`：单位秒，**对齐时间 = IMU 时间 − 此值**。需要事先标定，不提供自动时间偏移估计。
- `extrinsics`：`p_imu = rotation * p_lidar + translation`，旋转矩阵按行排列。
- `mapping`：扫描/局部地图体素、迭代数、局部地图范围和是否在线估计外参。
- `output`：累计地图体素尺寸、发布间隔、点数上限、PCD 路径。达到上限会报错退出并尝试保存已有地图。
- `buffers`：缓存上限、采样时间间隔上限和输入超时。超时检查由数据流的 `tick` 事件驱动。
- `tracking.lost_after_frames`：连续失配多少帧后报告 `tracking_lost`，默认 5，必须大于零。
- `tracking.pose_timeout`：距离最近有效位姿的时长上限，单位秒，默认 1.0，必须为可表示的正数。

## 数据接口

| 端口 | 内容 |
|---|---|
| 输入 `pointcloud_timed` | Livox 驱动新增的 `DLIOPC01` 二进制逐点时间格式，见下文 |
| 输入 `imu` | 原有 IMU JSON：`header.timestamp` 秒、`angular_velocity` rad/s、`linear_acceleration` 由配置确定单位 |
| 输入 `tick` | Dora 定时事件，检查传感器输入超时 |
| 输出 `pose` | 6 个 little-endian double：世界坐标下雷达原点的 XYZ、roll、pitch、yaw，共 48 字节 |
| 输出 `pointcloud` | 去畸变扫描，雷达在扫描结束时的局部坐标，原有 `double 秒 + uint32 点数 + N×float32 XYZI` 格式 |
| 输出 `full_map` | 世界坐标下累计体素点云，采用相同 XYZI 格式 |
| 输出 `status` | JSON，包含初始化/建图/保存状态、接收帧数、接受帧数、地图点数及扫描结束时间 |

首次有效匹配前保持 `initializing`。首次失配报告 `degraded`；连续失配达到阈值，或超过
`pose_timeout` 没有新的有效位姿时报告 `tracking_lost`。失配扫描不发布位姿，恢复有效匹配后
立即报告 `mapping` 并清零连续失败次数。有效位姿超时也由 `tick` 检查，不依赖传感器是否仍在发数据。
状态变化、失配帧和定时检查都会更新状态，不受地图发布间隔限制。

`status` 额外包含 `tracking_state`、`pose_valid`、`consecutive_failures`、
`last_valid_pose_timestamp`、`last_valid_pose_age_sec` 和 `reason`。
最近有效时间戳采用传感器时钟（秒），年龄采用本机单调时钟（秒），首次有效位姿前两者均为 `null`。
`state: saved` 仅表示保存完成，`tracking_state` 仍保留停止时的匹配状态，避免将存图成功解释为定位有效。

输出位姿、去畸变点云和地图可接入既有 `rerun_mapping_node` 的 `pose_data`、`pointcloud`、
`full_map` 输入；启用可视化时需另行打开 `BUILD_VISUALIZATION` 并在数据流中添加该节点。
`pose` 表示雷达位姿，若用于底盘控制还需通过雷达与底盘的安装外参转换。

Livox 新输出由 `LIVOX_PUBLISH_TIMED_POINTCLOUD=true` 显式开启；已有 `pointcloud` 保持原格式。
`LIVOX_CONFIG_FILE` 可指定设备配置路径。新数据流已设置这两个变量。
`DLIOPC01` 全部字段为 little-endian：

- 24 字节头：8 字节 ASCII `DLIOPC01`、`uint64 frame_start_ns`、`uint32 count`、`uint32 flags=0`。
- 每点 24 字节：`float32 x,y,z,intensity`、`uint32 offset_ns`、`uint16 line`、`uint8 tag`、`uint8 reserved=0`。
- `offset_ns` 为同步设备采样时间减去帧起点，单位纳秒；不是接收时间，也不按点索引推算。

只接收同一时钟下的递增时间戳；无效包、时间回退、采样间隔过大、缓存溢出会使节点失败退出。
IMU 必须覆盖扫描结束时间。初始化及缺少足够匹配约束的扫描不发布有效建图位姿。

## 地图与停止

核心内部使用滑动局部地图；本节点另行累计已接受扫描，输出 `full_map` 并在停止时保存
二进制 XYZI PCD，默认路径为仓库下 `maps/fast_lio2/map.pcd`。
该地图是前端里程计累计结果，不包含回环优化，也不是导航占据栅格。

收到 Dora Stop、输入结束或处理错误后，尝试保存已有地图；写入失败或未形成有效地图时以非零状态退出。
保存先写同目录临时文件，完成后再替换目标文件。处理错误后的已保存地图是截止错误前的部分结果，进程仍保留失败状态。

## 来源

仅导入所提供的 `DoraCN/FAST-LIO` 项目中的算法核心，未导入其独立硬件驱动、规划器或底盘应用。
原始许可证和测试随核心保留，集成验证工具及运行日志放在仓库外。
来源及本地修改见 `NOTICE`。移植项目与原始 FAST_LIO 的许可证说明存在不一致，社区发布前需澄清授权依据。
