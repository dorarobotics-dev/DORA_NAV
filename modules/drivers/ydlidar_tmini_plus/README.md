# YDLIDAR T-mini Plus 二维激光雷达驱动

使用项目内嵌的 YDLIDAR SDK 1.2.3，通过串口采集二维扫描，并作为原生 C++ Dora 节点发布 `LaserScan` JSON。

## 目录分工

沿用 `rslidar_sdk` 的模块内部布局：节点入口放在 `node/`，适配代码和内嵌厂商 SDK 放在 `src/`。

```text
ydlidar_tmini_plus/
├── CMakeLists.txt
├── README.md
├── config/
│   └── config.yaml
├── node/
│   └── ydlidar_tmini_plus_dora_node.cpp
└── src/
    ├── config_loader.hpp
    ├── scan_contract.hpp
    ├── ydlidar_tmini_plus_config.hpp
    └── ydlidar_sdk/
```

日常修改参数请编辑 [config/config.yaml](config/config.yaml)。节点入口在 `node/`，配置加载和扫描整理代码在 `src/`。

## 在 DORA_NAV 中构建

沿用根工程的 Dora C SDK 配置（`DORA_INCLUDE_DIR`、`DORA_NODE_API_LIB`），以及已有的 Threads、nlohmann-json 和 yaml-cpp 依赖。

```bash
cmake -S . -B build -DBUILD_DRIVER=ON -DBUILD_YDLIDAR_TMINI_PLUS=ON
cmake --build build --target ydlidar_tmini_plus_dora_node -j4
```

可执行文件生成在：

```text
build/modules/drivers/ydlidar_tmini_plus/ydlidar_tmini_plus_dora_node
```

`BUILD_YDLIDAR_TMINI_PLUS` 默认关闭，启用时也需要 `BUILD_DRIVER=ON`。
SDK 使用模块内的独立静态库目标构建；厂商 SDK 的源码、许可证和原始构建文件保留在 `src/ydlidar_sdk/`，不执行其中的 Python、示例、安装或打包流程。

## Dora 配置示例

下面片段可用于位于 `apps/` 下的数据流配置。节点需要 timer 输入触发采样；扫描消费者随后连接 `ydlidar_tmini_plus/LaserScan`。默认使用本模块的 `config/config.yaml`。

```yaml
nodes:
  - id: ydlidar_tmini_plus
    path: ../build/modules/drivers/ydlidar_tmini_plus/ydlidar_tmini_plus_dora_node
    inputs:
      tick: dora/timer/millis/100
    outputs:
      - LaserScan
```

## 配置与输出

参数优先顺序为：内置默认值 → YAML 文件 → 同名大写环境变量。平时直接修改 YAML 即可。
例如，`serial_port` 对应可选的 `SERIAL_PORT` 环境变量，临时覆盖只影响本次启动，不修改 YAML 文件。
设置 `YDLIDAR_TMINI_PLUS_CONFIG_FILE` 可以指定另一份配置文件，优先于兼容的 `CONFIG_FILE`；文件不存在、参数类型错误、未知参数或数值越界会报错退出。
联合建图数据流为两个节点分别设置 `CONFIG_FILE`；从命令行替换单个节点的 YAML 时，
驱动使用 `YDLIDAR_TMINI_PLUS_CONFIG_FILE`，Karto 使用 `KARTO_CONFIG_FILE`，避免配置串用。
`MASK_ANGLE_RANGES_DEG` 设为空字符串可临时关闭 YAML 中的角度屏蔽。

| YAML 参数 | 默认值 | 含义 |
|---|---|---|
| `serial_port` | `/dev/ttyUSB0` | 串口路径 |
| `serial_baud` | `230400` | 串口波特率 |
| `frame_id` | `lidar` | 扫描坐标系 |
| `output_id` | `LaserScan` | Dora 输出名称 |
| `scan_frequency_hz` | `10.0` | 扫描频率，允许 5–12 Hz |
| `target_point_count` | `450` | 每帧固定角度采样槽数量 |
| `angle_min_deg` / `angle_max_deg` | `0.0` / `360.0` | 输出角度窗口，端点为 0–360 度，支持跨越 0 度 |
| `range_min` / `range_max` | `0.05` / `12.0` | 有效距离范围，单位米 |
| `clockwise` | `false` | 保留原驱动的角度方向转换约定 |
| `mask_angle_ranges_deg` | 空 | 屏蔽角度范围，例如 `80-100` |

`RANGE_MAX` 是运行配置值，不表示已确认设备的最大实测量程。角度方向和屏蔽范围应根据实际安装配置。
扫描启动后的自动重连由厂商 SDK 管理，首次初始化失败会报错退出。已移除原来未实际使用的 `CONNECT_RETRY_MS` 配置项。

整圈扫描使用不重复的角度槽；部分角度窗口包含首末两个端点。屏蔽范围可以用逗号分隔，`0-360` 表示全圈屏蔽。

扫描输出保留迁入驱动的 JSON 契约：

- `header.stamp.sec` / `header.stamp.nanosec` 保留 SDK 提供的扫描起始采样时间（Unix 纳秒），包含 `seq` 和 `frame_id`。SDK 时间戳为零或超出可表示范围时记录错误并丢弃该帧。
- `angle_min`、`angle_max`、`angle_increment` 使用弧度；`scan_time` 使用秒，保留 SDK 的扫描周期。
- 扫描经过角度重排和重采样后，无法恢复可靠的逐束采样顺序，因此 `time_increment` 为 `0`（未知）。当前输出仅提供帧级采样时间，不能用于逐束运动去畸变。
- `ranges` 使用米，`intensities` 为对应强度；无效、超量程和被屏蔽的采样槽距离为 `0`。
- 附带 `point_count`、`raw_point_count`、`rotation_speed_hz`。

该模块输出二维扫描。`apps/mapping_2d.yml` 将其接入 Karto 二维建图；现有 HDL 节点继续消费三维点云。

## 来源与许可证

驱动以 Adora Nano 二维导航项目的 `ydlidar_tmini_plus_dora_node` 模块为基础迁入，已适配本工程的目录、YAML 配置，并修正采样时间戳及扫描边界处理。
内嵌厂商 SDK 版本为 1.2.3，完整许可证及其依赖声明位于 [src/ydlidar_sdk/LICENSE.txt](src/ydlidar_sdk/LICENSE.txt)。
