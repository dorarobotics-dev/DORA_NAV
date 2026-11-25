# Karto 二维 Dora 建图模块

接收二维 `LaserScan` 和底盘 `Odometry`，通过 Dora C API 运行 Karto 建图并保存地图。
该模块以 Adora Nano 项目的 `karto_slam_dora_node` 为基础接入 DORA_NAV，可接入 Ranger 或 Adora Nano 的底盘里程计。

```text
karto_slam_dora/
├── CMakeLists.txt
├── config/config.yaml          # 建图参数和地图保存路径
├── include/karto_dora/         # 模块接口与配置读取
└── src/                       # Dora 节点入口、建图、同步和消息适配
    └── core/karto/            # 内置 Karto 算法、karto_sdk 头文件及许可证
```

## 构建

模块内的算法与支持代码统一构建为 `karto_core` 静态库，由 Dora 节点调用。
沿用根工程的 `DORA_INCLUDE_DIR`、`DORA_NODE_API_LIB`。二维建图额外需要
Ceres、OpenSSL、Eigen、Boost serialization/thread/filesystem/system、TBB 和 yaml-cpp。
求解器沿用原工程的 `ceres::LocalParameterization` 接口，Ceres 版本需要保留该接口。

在 DORA_NAV 根目录配置和构建：

```bash
cmake -S . -B build \
  -DBUILD_YDLIDAR_TMINI_PLUS=ON -DBUILD_KARTO_2D=ON \
  -DBUILD_RANGER_CHASSIS=ON -DBUILD_FEETECH_KIWI_CHASSIS=OFF \
  -DBUILD_LIVOX_DRIVER=OFF -DBUILD_RSLIDAR_DRIVER=OFF \
  -DBUILD_LIGHTNING_LM_MAPPING=OFF -DBUILD_HDL_GRAPH_SLAM=OFF \
  -DBUILD_LOCALIZATION=OFF -DBUILD_GLOBAL_PLANNER=OFF \
  -DBUILD_LOCAL_PLANNER=OFF -DBUILD_GOAL_PUBLISHER=OFF -DBUILD_VISUALIZATION=OFF
cmake --build build --target ydlidar_tmini_plus_dora_node ranger_miniv3_node karto_slam_dora_node -j4
```

`BUILD_KARTO_2D` 默认关闭，启用时需要原有 `BUILD_MAPPING=ON`。
以上命令只启用二维驱动、Ranger 和 Karto，避免查找其他雷达和三维算法的额外依赖。
既有驱动和算法开关默认仍为开启，保留原有默认构建行为。
单独配置本模块时，可以用 `KARTO_BUILD_NODE=OFF`
仅构建核心库，省去 Dora C SDK 要求，但仍需上述算法依赖。

## Dora 数据流

```bash
# 只启动二维雷达
dora run apps/lidar_2d.yml

# 二维雷达 + 原有 Ranger 里程计 + Karto 建图
dora run apps/mapping_2d.yml

# 二维雷达 + Adora Nano 里程计 + Karto 建图
dora run apps/mapping_2d_adora_nano.yml
```

`mapping_2d.yml` 使用 Ranger 的里程计，不接任何速度指令。
[Adora Nano 数据流](../../../apps/mapping_2d_adora_nano.yml) 同样只接底盘里程计，
对应构建参数见 [Adora Nano 底盘模块](../../chassis/feetech_kiwi_chassis_dora_node/README.md)。
其他底盘可以替换里程计提供节点，但必须提供同一 Unix 时钟下的时间戳。
代码构建不需要连接设备。

## 参数

编辑 [config/config.yaml](config/config.yaml)。`KARTO_CONFIG_FILE` 可以选择另一份参数文件，
优先于兼容的 `CONFIG_FILE`。联合建图数据流为每个节点设置独立的 `CONFIG_FILE`，
从命令行替换单个节点的配置时，使用 `KARTO_CONFIG_FILE` 或驱动的
`YDLIDAR_TMINI_PLUS_CONFIG_FILE`，避免两个节点加载同一文件。
已有的大写环境变量继续作为可选覆盖方式，例如 `SLAM_MODE`、`LIDAR_X` 和 `MAP_PREFIX`。
地图路径和日志路径相对于所选 YAML 文件的目录解析，也可以使用绝对路径。

默认 `mode: new_mapping`，期望每帧 450 个角度槽，与二维驱动默认值一致。
`continue_mapping` 和 `localization` 使用本模块保存的 `.posegraph.dora` 归档；续图保留历史图约束。
如果修改雷达的点数，必须同步修改 `expected_laser_beams`。
`lidar_x`、`lidar_y`、`lidar_yaw` 是雷达相对底盘的安装外参，默认零值为占位值，设备使用前需填写实际标定值。

## 输入与输出

| 端口 | 内容 |
|---|---|
| 输入 `LaserScan` | 驱动的 JSON 扫描，`header.stamp.sec/nanosec` 为 Unix 采样时间 |
| 输入 `Odometry` | DORA_NAV 的 `pose.position/orientation`、`twist.linear/angular`、`header.timestamp`（Unix 秒）；也兼容原模块的嵌套格式 |
| 输入 `tick` | Dora 定时事件，用于处理待同步扫描的超时 |
| 输出 `pose` | 与现有规划节点兼容的三维位置和四元数 JSON，平面运动的 z 为零 |
| 输出 `slam_status` | 建图状态、待处理扫描数量及已处理扫描数量 |
| 输出 `safety_stop` | 错误或定位失效时的停机请求，供后续控制链消费 |

当前雷达输出的 `time_increment=0` 表示逐束时间未知，本接入仅保证帧级时间同步，不提供逐束运动去畸变保证。

正常收到 Dora Stop 后保存已接受的扫描，默认输出：

```text
maps/2d/map.pgm
maps/2d/map.yaml
maps/2d/map.metadata.json
maps/2d/map.posegraph.dora
```

保存失败时节点发布错误状态并以非零状态退出；重复停止会保留该失败结果。

## 来源与许可证

底层核心内置于 [src/core/karto](src/core/karto/)，
LGPL 许可证随源码保留。Ceres 求解器适配文件也保留了原始派生来源说明。
