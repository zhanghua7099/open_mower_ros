# OpenMower 模块参考文档

这是 OpenMower（`open_mower_ros`）的逐模块详细参考手册：每个 ROS 包的角色、输入（订阅的话题/调用的服务与 action/参数）、输出（发布的话题/提供的服务与 action）、以及内部主要算法与执行流程（附精确到文件/行号的代码引用）。

- 想先看整体数据流水线和模块之间的调用关系，请先看 [architecture.md](architecture.md)（一页纸的全局视图）；本篇是它的深挖版，逐个模块把接口和算法讲透，供开发/调试时按需查阅。
- 所有文件路径均为仓库根目录 `open_mower_ros/` 的相对路径。
- 内容按 [architecture.md](architecture.md) 的分层组织：底层通讯 → 定位 → 地图与路径规划 → 决策（状态机）→ 导航执行与速度仲裁 → 监控与遥控。

## 目录

- [一、底层通讯层（与主控板对话）](#一底层通讯层与主控板对话)
  - [xbot_framework（xbot-service 协议基础库）](#xbot_framework协议基础库先讲这个后面几节都要引用它)
  - [mower_comms_v2](#mower_comms_v2)
  - [mower_comms_v1 与 xesc_ros](#mower_comms_v1-与-xesc_ros)
  - [mower_simulation](#mower_simulation)
  - [mower_msgs / xbot_msgs（消息定义包）](#mower_msgs--xbot_msgs纯消息服务定义包)
  - [xbot_mqtt](#xbot_mqtt)
- [二、定位](#二定位)
  - [xbot_positioning（EKF 定位核心）](#xbot_positioning)
  - [xbot_driver_gps](#xbot_driver_gps)
  - [ros_ntrip_client](#ros_ntrip_client)
- [三、地图与路径规划](#三地图与路径规划)
  - [mower_map（地图存取服务）](#mower_map)
  - [slic3r_coverage_planner（覆盖路径规划）](#slic3r_coverage_planner)
- [四、决策 / 状态机](#四决策--状态机)
  - [mower_logic（核心状态机）](#mower_logic)
- [五、导航执行与速度仲裁](#五导航执行与速度仲裁)
  - [move_base_flex (mbf_costmap_nav)](#move_base_flex-mbf_costmap_nav)
  - [ftc_local_planner（Follow The Carrot 局部规划器）](#ftc_local_planner-ftcplanner-follow-the-carrot-局部规划器)
  - [GlobalPlanner](#globalplanner)
  - [twist_mux](#twist_mux)
- [六、监控与遥控](#六监控与遥控)
  - [xbot_monitoring](#xbot_monitoring)
  - [xbot_remote](#xbot_remote)
  - [mower_utils](#mower_utils)
  - [xbot_mqtt（简述）](#xbot_mqtt简述)
  - [手柄/遥操作栈（joy / teleop_twist_joy / joy_teleop）](#手柄遥操作栈joy--teleop_twist_joy--joy_teleop)

---

## 一、底层通讯层（与主控板对话）

负责真实主板/仿真主板与 ROS 之间的数据交换：读取 IMU、轮速、GPS、电源、割草电机状态，下发速度指令、割草开关、急停控制。真实硬件（Stage-2 主板，经 `xbot-service` 协议）与仿真（`mower_simulation`）对上层来说是完全等价的通讯对端。

### xbot_framework（协议基础库，先讲这个，后面几节都要引用它）

**角色**：整个"底层板卡通讯层"的地基。它不是一个 ROS 节点，而是一套 C++ 库 + 代码生成工具，定义了"什么是一个 xbot-service"、这些 service 之间如何通过 UDP 交换数据/配置/RPC，以及怎么从一份 `services/*.json` 描述文件自动生成对应的 C++ 基类。`mower_comms_v2`（作为**客户端/Interface 侧**）与 `mower_simulation`、真实主板固件（作为**服务端/Service 侧**）都基于这套库构建，这也是为什么仿真能在 ROS 层完全替换真实硬件而上层代码零感知。

**节点**：无独立 ROS 节点，是被 `mower_comms_v2`、`mower_simulation` 链接进各自可执行文件的静态库（`xbot-service`、`xbot-service-interface`）。

#### 协议模型

一个 **service**（如 `EmergencyService`、`DiffDriveService`）是一个有固定 schema 的双向数据通道，schema 由仓库根 `services/*.json` 描述，字段包括：

- `inputs`：由「Interface 侧」（ROS 端，即 mower_comms_v2/仿真的 SimRpc 调用方）发往「Service 侧」（板卡固件/仿真的具体 xxxService）的数据项，例如 `DiffDriveService` 的 `Control Twist`（`double[6]`）。
- `outputs`：由 Service 侧发往 Interface 侧的数据项，例如 `DiffDriveService` 的 `Actual Twist`、`Wheel Ticks`。
- `registers`：配置类字段，Interface 侧在收到 `CONFIGURATION_REQUEST` 后通过一次「配置事务」写入 Service 侧（如 `GpsService` 的 `Baudrate`、`Protocol`），可以带 `default`/`optional` 标记。
- `functions`：同步 RPC 调用（请求/响应），如 `MetaService` 的 `GetFirmwareVersion`、`GetMajorVersion`。
- `enums`：普通枚举或位掩码枚举（`bitmask: true`，如 `EmergencyReason`），代码生成器会把位掩码枚举转成 `namespace X { enum Value : T {...}; }` 形式，普通枚举转成 `enum class`。

`services/service_ids.h` 定义了每个 service 的数字 ID（`EMERGENCY=1, DIFF_DRIVE=2, MOWER=3, IMU=4, POWER=5, GPS=6, INPUT=7, HIGH_LEVEL=8, BMS=9, REMOTE_GPIO=10, META=11`），两端用同一个 ID 互相寻址。

#### 线协议（wire protocol）

`src/lib/xbot_framework/include/xbot/datatypes/XbotHeader.hpp:46-88` 定义了固定 24 字节（`#pragma pack(push,1)`）的 `XbotHeader`：`protocol_version, message_type, flags, service_id, arg1, arg2, sequence_no, timestamp(ns), payload_size`。所有数据都跑在 **UDP** 之上（`src/lib/xbot_framework/include/xbot/config.hpp:16-18`：`max_packet_size=1472` 字节，正好是以太网 MTU 减去 IP/UDP 头，避免 IP 分片）。

`MessageType`（`XbotHeader.hpp:11-43`）关键取值：
- `DATA (0x01)`：单个 input/output/register 的即时更新，`arg2` = 该字段的 `target_id`。
- `CONFIGURATION_REQUEST (0x02)`：Service 侧请求 Interface 侧重新下发所有 register（例如 Service 重启后）。
- `CLAIM (0x03)`：Interface 侧"认领"一个 service，payload 为自己的 IP(uint32)+port(uint16)；Service 侧回 `CLAIM` 且 `arg1==true` 表示 ack。
- `HEARTBEAT (0x04)`：Service 侧周期性发送，证明自己还活着。
- `TRANSACTION (0x05)`：把多个 DATA/寄存器更新打包成一个包，内部用 `DataDescriptor`（`target_id, payload_size`，`XbotHeader.hpp:80-88`）逐段切分；`arg1` 区分 0=数据事务/1=配置事务。
- `RPC_CALL (0x06)` / `RPC_RESPONSE (0x07)`：同步函数调用，`arg1`=function_id，`arg2`=call_id（响应时原样带回）；`RpcStatus` 为 `SUCCESS/BUSY/ERROR`。
- `SERVICE_ADVERTISEMENT (0x80)` / `SERVICE_QUERY (0x81)`：服务发现，JSON 编码，走组播（`sd_multicast_address = 233.255.255.0`，`config.hpp:39`；普通数据组播地址 `multicast_port=4242`，`config.hpp:20`）。未被认领时按 1 秒间隔（`sd_advertisement_interval_micros_fast`）快速广播，超过 60 秒（`sd_discovery_timeout_micros`）仍未认领则退化为 10 秒慢速广播（`config.hpp:44-52`）。

#### 两侧的 C++ 基类

- **Service 侧**（板卡/固件、仿真）继承自 `xbot::service::Service`（`src/lib/xbot_framework/libxbot-service/include/xbot-service/Service.hpp:20`）。核心回调：`OnCreate/OnStart/OnStop`、`OnTransactionStart/OnTransactionEnd`、`dispatchRpcCall`（由生成代码覆盖，路由到具体 `RPCXxx()`）。发送用 `SendData()`/`StartTransaction()`+`CommitTransaction()`/`SendRpcResponse()`。内部维护一个 `Scheduler`（`Schedule`/`ManagedSchedule`）驱动心跳、服务公告、配置请求等周期任务，见 `Service.hpp:129-136`。
- **Interface 侧**（ROS 端：`mower_comms_v2`）继承自 `xbot::serviceif::ServiceInterfaceBase`（`src/lib/xbot_framework/libxbot-service-interface/include/xbot-service-interface/ServiceInterfaceBase.hpp:19`）。核心：`SendData()`/`StartTransaction()`+`CommitTransaction()`（配置事务用 `is_configuration=true`）、`SendRpc()`（阻塞式，内部用 `condition_variable` 等 `OnRpcResponse()` 回调或超时，见 `ServiceInterfaceBase.cpp:122-198`）；`OnServiceDiscovered()`（校验 `type`/`version` 匹配后才 `RegisterCallbacks`，见 `ServiceInterfaceBase.cpp:229-286`）。

#### 代码生成（codegen）

`services/*.json` → C++ 基类的生成管线：

1. `src/lib/xbot_framework/codegen/xbot_codegen/xbot_codegen.py` 的 `loadService()`（第 90-226 行）读取 JSON，做一致性检查（ID 唯一、名字唯一、类型合法），把 `inputs/outputs/registers/functions/enums` 规范化成模板可用的字典（如把名字转成驼峰 `toCamelCase`，为每个字段生成 `OnXxxChanged` 回调名和 `SendXxx`/`SetRegisterXxx` 方法名）。
2. `cog`（Python 的 `cogapp` 文字生成工具）把这份字典喂给两套 Jinja 风格模板：
   - `codegen/templates/ServiceTemplate.hpp/.cpp` → 生成 **Service 侧**基类 `XxxServiceBase`（供 `mower_simulation`、真实固件继承）。
   - `codegen/templates/ServiceInterfaceTemplate.hpp/.cpp` → 生成 **Interface 侧**基类 `XxxServiceInterfaceBase`（供 `mower_comms_v2` 继承）。
3. CMake 侧由 `codegen/cmake/AddService.cmake`（`add_service`/`target_add_service`）与 `AddServiceInterface.cmake`（`add_service_interface`/`target_add_service_interface`）驱动：对每个 `.json` 跑两条 `cog` 命令生成 `.hpp`/`.cpp` 到 `${CMAKE_CURRENT_BINARY_DIR}/generated/`，作为 `add_custom_command` 的产物纳入编译，见 `AddServiceInterface.cmake:16-22`。`mower_comms_v2/CMakeLists.txt:189-198` 与 `mower_simulation/CMakeLists.txt:178-184` 分别调用它们，为 10/7 个 service 生成对应基类。

生成出的 `ServiceInterfaceTemplate.cpp`（示例见其自身文件）展示了具体编码方式：标量 input 直接 `SendData(id, &data, sizeof(T), false)`；数组 input 用长度乘以元素大小；RPC 调用则把每个参数打包成 `DataDescriptor + 原始字节` 拼接的缓冲区，调用 `SendRpc(function_id, params_buf, ..., timeout_ms)`。

参考文档：`src/lib/xbot_framework/include/README.md`、`src/lib/xbot_framework/codegen/README.md`、`src/lib/xbot_framework/libxbot-service/README.md`。

---

### mower_comms_v2

**角色**：Stage-2（新一代）主板的 ROS 接入层，是"底层板卡通讯层"里唯一的常驻 ROS 节点（真实硬件场景）。作为 xbot-service 协议的 **Interface 侧**，同时实例化 10 个 service 的 Interface 子类，把每个 service 的 `outputs`/RPC 结果转换发布为 ROS 话题，把 ROS 端收到的指令（`/ll/cmd_vel`、割草开关、急停）转成 service 的 `inputs` 发送出去。真实主板和 `mower_simulation` 对它来说是同一个协议对端，感知不到差异。

**节点**：`mower_comms_v2`（`src/mower_comms_v2/src/mower_comms.cpp:168` `main()`），由 `src/open_mower/launch/include/_comms.launch:29-35` 在 `HARDWARE_PLATFORM==2` 时启动（`respawn="true"`）。

#### 输入

**订阅的 ROS 话题**（`mower_comms.cpp:249-256`）：
| 话题 | 类型 | 用途 |
|---|---|---|
| `ll/cmd_vel` | `geometry_msgs/Twist` | 上层（`twist_mux` 输出）下发的目标速度，回调 `velReceived`（`mower_comms.cpp:81-86`）转发给 `DiffDriveService` 的 `Control Twist` input；固件版本不兼容时（`is_firmware_compatible==false`）直接丢弃，电机不会动。 |
| `ll/position/gps/rtcm` | `rtcm_msgs/Message` | NTRIP 差分改正数据，`rtcmReceived`（`mower_comms.cpp:88-99`）攒到 1000 字节或每 0.2s 转发一次到 `GpsService` 的 `RTCM` input，避免逐字节发包刷屏。 |
| `xbot/action` | `std_msgs/String` | 转发给 `InputServiceInterface::OnAction`（`mower_comms.cpp:105-107`），用于从 ROS 侧模拟按键（payload 前缀 `mower_comms_v2/simulate_input\|\|`）。 |
| `mower_logic/current_state` | `mower_msgs/HighLevelStatus` | 分别被 `HighLevelServiceInterface`（`HighLevelServiceInterface.h:30-31`）和 `InputServiceInterface`（`InputServiceInterface.h:22-23`）各自订阅一路，前者转发给固件的 `HighLevelService`，后者用状态名（小写）作为"输入动作触发上下文"（如 `idle/short` vs `autonomous/short`）。 |

**服务客户端**：`mower_service/high_level_control`（`HighLevelControlSrv`，`highLevelClient`，`mower_comms.cpp:180`）——当前代码里声明了但主流程未调用（历史上由 V1 UI 按钮触发，V2 里预留）。

**ROS 参数**（都在 `/ll` 命名空间下，通过 `_params.launch` / `openmower_defaults_v2.yaml` / `hardware_specific/<MOWER>/params_v2.yaml` 灌入，具体默认值见下方"参数来源"）：
| 参数 | 用途 | 缺省来源 |
|---|---|---|
| `bind_ip`（默认 `"0.0.0.0"`，实际部署为 `172.16.78.1`） | xbot-service UDP 监听/绑定地址 | `mower_comms.cpp:183-185`，`openmower_defaults_v2.yaml:2` |
| `services/diff_drive/ticks_per_m`、`services/diff_drive/wheel_distance_m` | 轮速计每米脉冲数、轮距，必填，缺失则退出（`DiffDriveServiceInterface.h:25-34`） | `params_v2.yaml`（如 YardForce500: 1600.0 / 0.325） |
| `services/gps/baud_rate`、`services/gps/protocol`（`UBX`/`NMEA`）、`services/gps/port_index` | GPS 串口配置，作为 register 下发给固件 | `openmower_defaults_v2.yaml:8-10` |
| `services/gps/datum_lat/long/height` | 计算本地 ENU/UTM 原点，必填（`GpsServiceInterface.cpp:37-46`） | 硬件相关 env |
| `services/gps/absolute_coords`（默认 `true`） | 决定 `OnPositionChanged` 是把固件发来的经纬度转 UTM 还是把本地坐标转经纬度回填 NMEA | `GpsServiceInterface.cpp:135-152` |
| `services/power/battery_full_voltage`、`battery_empty_voltage`、`battery_critical_voltage`、`battery_critical_high_voltage` | 必填，作为电源保护阈值 register 下发 | `params_v2.yaml` |
| `services/power/charge_voltage/current/termination_current/pre_charge_current/re_charge_voltage/system_current/dangerously_override_hardware_charge_current_limit/log_debug` | 可选充电器高级配置 | 硬件相关 env（可为空） |
| `services/imu/axis_config`（如 `"+X-Y-Z"`） | IMU 轴映射字符串，校验格式 `((+|-)(X|Y|Z)){3}` 且三轴不重复（`ImuServiceInterface.cpp:23-45`），非法则退回 YardForce 默认 `+X-Y-Z` | `openmower_defaults_v2.yaml:14` |
| `services/input/config_file` | YAML 格式的输入（按键/急停触点）配置文件路径，用 `libfyaml` 转 JSON 再解析（`InputServiceInterface.cpp:22-58`） | `params_v2.yaml`（如 `.../YardForce500/inputs.yaml`） |
| `services/input/lift_multiple_delay`、`collision_multiple_delay`（默认 -1=不设置） | 可选，作为 register 下发给固件 InputService | — |
| `board`（读取自 `/ll/board`） | 固件名，作为 `Robot Firmware` register 下发，供固件自动识别机型；未设置只警告不阻断（Sabo/xBot 除外） | `MetaServiceInterface.h:28-31` |

**xbot-service 服务实例化清单**（`mower_comms.cpp:191-245`，都在 `Start()` 前构造完毕，避免回调打到空指针）：

| Service（ID） | 承载数据 |
|---|---|
| `MetaService`（11，最先起） | RPC：`GetFirmwareVersion() -> char[50]`、`GetMajorVersion() -> uint16_t`；register：`Robot Firmware`（把 `board` 参数写给固件） |
| `EmergencyService`（1） | input：`High Level Emergency`=`uint16_t[2]`(add,clear)；output：`Emergency Reason`（`uint16_t` 位掩码，见下方枚举） |
| `DiffDriveService`（2） | input：`Control Twist`=`double[6]`；output：`Actual Twist`、左右 ESC 状态/温度/电流、`Wheel Ticks`；register：`Wheel Ticks Per Meter`、`Wheel Distance` |
| `MowerService`（3） | input：`Mower Speed`（`float`，signed duty，符号=方向）；output：`Mower Status`、`Rain Detected`、`Mower Running`、ESC/电机温度电流转速 |
| `ImuService`（4） | output：`Axes`=`double[9]`（仅用前 6 个：加速度 xyz + 角速度 xyz）；register：`AxisRemap`=`int8_t[3]` |
| `PowerService`（5） | input：`Charging Allowed`；output：充电/电池电压电流、`Charging Status`（字符串）、`Charger Enabled`、`Battery Percentage` 等；registers：见上表 |
| `GpsService`（6） | input：`RTCM`=`uint8_t[255]`；output：`Position`(ECEF/UTM 待转换)、精度、`FixType`(字符串 `FIX`/`FLOAT`)、运动矢量/朝向；registers：`Baudrate`、`Protocol`、`Uart` |
| `BmsService`（9） | output：电压/电流/SOC/剩余容量/满容量/循环次数/温度/`BatteryStatus` 位掩码/`ExtraData`（无 register） |
| `InputService`（7） | input：`Simulated Inputs`(`uint64_t` 位图)；output：`Active Inputs`(位图)、`Input Event`=`uint8_t[2]`(idx, type)；register：压缩后的输入配置 JSON（`blob`，HeatshrinkEncode）、`Debounce/LongPress/LiftMultiple/CollisionMultiple` 延时 |
| `HighLevelService`（8） | input：状态机当前状态（`State ID`/`Name`/`SubStateName`/`GpsQuality`/`CurrentArea/Path/PathIndex`），全部来自 `mower_logic/current_state` 话题转发；output：`Action`（字符串，目前只打日志） |

#### 输出

**发布的 ROS 话题**（`mower_comms.cpp:198-234`）：
| 话题 | 类型 | 触发点 |
|---|---|---|
| `ll/emergency` | `mower_msgs/Emergency` | `EmergencyServiceInterface::PublishEmergencyState`（`EmergencyServiceInterface.cpp:76-98`），`Emergency Reason` output 变化，或服务断开时（回退为 `TIMEOUT_HIGH_LEVEL`）触发 |
| `ll/diff_drive/measured_twist` | `geometry_msgs/TwistStamped` | `DiffDriveServiceInterface::OnActualTwistChanged`（`DiffDriveServiceInterface.cpp:30-47`），`Actual Twist` output 变化 |
| `ll/diff_drive/left_esc_status`、`ll/diff_drive/right_esc_status` | `mower_msgs/ESCStatus` | 事务结束时（`OnTransactionEnd`）发布，聚合了 ESC 状态/温度/电流/`Wheel Ticks`（tacho） |
| `ll/mower_status` | `mower_msgs/Status` | `MowerServiceInterface`，事务结束时发布（含 `mow_enabled`、割草电机状态/温度/电流/RPM/雨感） |
| `ll/imu/data_raw` | `sensor_msgs/Imu` | `ImuServiceInterface::OnAxesChanged`，`Axes` 变化即发布（frame `base_link`） |
| `ll/power` | `mower_msgs/Power` | `PowerServiceInterface`，事务结束时发布 |
| `ll/bms` | `mower_msgs/Bms` | `BmsServiceInterface`，事务结束时发布（`BatteryStatus` 位掩码转成人类可读字符串，见 `BmsServiceInterface.cpp:40-50`） |
| `ll/position/gps` | `xbot_msgs/AbsolutePose` | `GpsServiceInterface`，事务结束时发布；坐标先转 UTM 再减去 datum 得到局部坐标（或反过来，取决于 `absolute_coords`） |
| `ll/position/gps/nmea` | `nmea_msgs/Sentence` | `GpsServiceInterface::SendNMEA`，每 10 秒最多发一次合成的 `$GPGGA` 语句（`GpsServiceInterface.cpp:55-101`），供其他工具/调试使用 |
| `xbot/action` | `std_msgs/String` | advertise 出来给 `InputServiceInterface` 内部使用，同时也是回环订阅点（自己发的动作字符串） |

**服务端**：
- `ll/_service/mow_enabled`（`mower_msgs/MowerControlSrv`）：`setMowEnabled`（`mower_comms.cpp:113-122`）把 `mow_enabled`+`mow_direction` 转成 signed speed（+1/-1/0）下发 `MowerService`；固件不兼容时强制 0。
- `ll/_service/emergency`（`mower_msgs/EmergencyStopSrv`）：`setEmergencyStop`（`mower_comms.cpp:76-79`）把 reason 写入 `EmergencyService` 的 High Level Emergency。

**周期性行为（`ros::Timer`）**：
- 每 0.5s（`sendEmergencyHeartbeatTimerTask`）向固件发一次高层急停心跳；固件侧若 1s 收不到心跳会自行置位 `TIMEOUT_HIGH_LEVEL`。
- 每 5s（`sendMowerEnabledTimerTask`）重发一次当前割草速度，防止丢包导致割草状态卡死。

#### 主要算法/流程

1. **启动顺序**（`mower_comms.cpp:168-260`）：先 `xbot::serviceif::Start(true, bind_ip)` 拿到全局 `Context ctx`（内部含 IO 收发线程和 ServiceDiscovery），再依次构造/`Start()` 10 个 Service Interface（`MetaService` 最先，为的是让固件尽快退出 Stage-2 等待循环），最后才注册 ROS 订阅/定时器/服务端，避免回调打到未初始化指针。
2. **固件兼容性门控**（`mower_comms.cpp:124-152`，`OnFirmwareInfoChanged`）：`MetaServiceInterface` 每 1 秒轮询一次 `CallGetMajorVersion`/`CallGetFirmwareVersion`（`MetaServiceInterface.cpp:33-51`，阻塞 RPC，必须在 ROS 定时器线程而非 xbot IO 线程调用，否则会因等待自己的响应而死锁）。只有 `major==1` 时才把全局原子量 `is_firmware_compatible` 置 true；否则通过 `EmergencyService::SetFirmwareIncompatibleEmergency(true)` 强制拉起急停，`velReceived`/`setMowEnabled` 也各自检查该标志拒绝下发指令——即旧/不兼容固件下机器人被锁死，不会意外运动。
3. **急停状态机**（`EmergencyServiceInterface.cpp`）：高层急停原因是一个 `uint16_t` 位掩码（`EmergencyReason`，`services/emergency_service.json:20-37`：`LATCH/TIMEOUT_INPUTS/STOP/LIFT/LIFT_MULTIPLE/COLLISION/COLLISION_MULTIPLE/TIMEOUT_HIGH_LEVEL/HIGH_LEVEL/SERVICE_NOT_READY/MOWER_RPM_TIMEOUT/MOWER_RPM_LIMIT/FIRMWARE_INCOMPATIBLE`）。`SetHighLevelEmergency` 发送 `(add, clear)` 二元组给固件——固件只清除明确点名的位，因此 `LATCH` 一旦置位就必须显式清除才会消失，这与真实急停按钮的锁存行为一致。`Heartbeat()` 每 0.5s 用当前缓存的 reason 重发一次，维持 `HIGH_LEVEL` 位。
4. **差速驱动的角速度约定**：ROS `geometry_msgs/Twist` 的 6 个分量（线速度 xyz + 角速度 xyz）原样映射进 `Control Twist` 的 `double[6]`（`DiffDriveServiceInterface::SendTwist`），固件/仿真只用其中的 `linear.x` 和 `angular.z`。
5. **GPS 坐标转换**（`GpsServiceInterface.cpp:130-153`）：以 `datum_lat/long/height` 通过 `RobotLocalization::NavsatConversions::LLtoUTM` 算出局部原点 `(datum_e_, datum_n_, datum_u_)`；`absolute_coords=true`（默认）时固件发来的是经纬度，节点转 UTM 后减去原点得到局部 ENU 坐标；反之固件发来的已是局部坐标，节点加上原点转回经纬度只是为了生成 NMEA 语句。`FixType` 字符串（`"FIX"`/`"FLOAT"`）映射为 `AbsolutePose` 的 RTK 标志位。
6. **输入/按键配置**（`InputServiceInterface.cpp`）：本地 YAML（`libfyaml` 解析并转成等价 JSON 字符串）描述若干输入（`_driver`、`name`、`actions` 等），节点在内存里建一份带 `_idx` 的索引数组；下发给固件的版本会剥离 `name`/`actions`（固件不需要，也保护配置细节），并用 Heatshrink 压缩后作为 `blob` register 发送。固件上报 `Input Event`（`[idx, type]`）时，节点按 `current_context_（当前 mower_logic 状态，小写）+ "/" + duration（short/long）` 或退化为纯 `current_context_` 去 `actions` 表里查一条要发布到 `xbot/action` 的动作字符串。
7. **参数缺失即拒绝启动**：`DiffDriveServiceInterface`、`PowerServiceInterface`、`GpsServiceInterface` 的构造函数在必填参数缺失时把 `param_error_` 置非 0，`main()` 里通过 `GetParamError()` 检查并直接 `return err`（进程退出），配合 launch 的 `respawn="true"` 形成"参数不对就不断重启并报错"的快速失败模式。

---

### mower_comms_v1 与 xesc_ros

**角色**：老款 V1 硬件的通讯路径，**不走 xbot-service 协议**，而是自定义的 COBS 帧 + CRC16 校验的二进制协议，直接通过一条串口跟 LL（Low Level）主控板对话；左右轮和割草电机则是各自独立的 xESC 电调，通过 `xesc_ros` 提供的驱动库再走各自的串口/协议。真实硬件、仿真都不使用这条路径（仿真只对接 V2/xbot-service）。

**节点**：`mower_comms_v1`（`src/mower_comms_v1/src/mower_comms.cpp:702` `main()`），由 `_comms.launch:8-26` 在 `HARDWARE_PLATFORM==1` 时启动。

#### 输入

**订阅的话题**：
| 话题 | 类型 | 用途 |
|---|---|---|
| `ll/cmd_vel` | `geometry_msgs/Twist` | `velReceived`（`mower_comms.cpp:471-487`）差速运动学分解为左右轮 duty cycle，`speed_r/l = linear.x ± 0.5*wheel_distance_m*angular.z`，各自 clamp 到 [-1,1] |
| `/mower_logic/current_state` | `mower_msgs/HighLevelStatus` | `highLevelStatusReceived`（`mower_comms.cpp:449-469`）打包成 `ll_high_level_state`（type+mode+gps_quality+CRC）经 COBS 编码后写串口 |

**Dynamic Reconfigure 客户端**：`reconfigClient`（`/mower_logic`，`mower_logic::MowerLogicConfig`）、`powerReconfigClient`（`/ll/services/power`，`ll::PowerConfig`）——LL 板通过配置应答包（`PACKET_ID_LL_HIGH_LEVEL_CONFIG_RSP`）把它自己保存的阈值（雨感阈值、抬升/倾斜时长、电池截止电压等）回灌给这两个 dynamic_reconfigure 服务，实现"低层配置是唯一真源"的双向同步（`handleLowLevelConfig`，`mower_comms.cpp:558-605`）。

**ROS 参数**：
| 参数 | 用途 |
|---|---|
| `~ll_serial_port` | LL 板串口设备路径，必填 |
| `~services/diff_drive/ticks_per_m`、`~services/diff_drive/wheel_distance_m` | 里程计换算 |
| `~services/diff_drive/{left,right,mower}_xesc/*`（含各自的 `xesc_type`） | 三个 `xesc_driver::XescDriver` 实例的构造参数（mower 电调是可选的，`mowerParamNh.hasParam("xesc_type")` 才创建） |
| `~services/sound/*`（`dfp_is_5v`、`volume`、`background_sounds`、`language`） | 声音模块配置，打包进 `ll_high_level_config` 下发给 LL 板 |
| `/mower_logic/ignore_charging_current` | 同上，写入 `ConfigOptions.ignore_charging_current` |

**串口帧（LL → HL 方向，从串口读到的包）**（`ll_datatypes.h`）：`PACKET_ID_LL_STATUS`(1)、`PACKET_ID_LL_IMU`(2)、`PACKET_ID_LL_UI_EVENT`(3)、`PACKET_ID_LL_HIGH_LEVEL_CONFIG_REQ/RSP`(0x11/0x12)。

#### 输出

**发布的话题**：
| 话题 | 类型 | 数据来源 |
|---|---|---|
| `ll/emergency` | `mower_msgs/Emergency` | `publishStatus()`：`active_low_level_emergency = last_ll_status.emergency_bitmask & 0xFE`，`latched=is_emergency()`（高层或低层任一置位） |
| `ll/diff_drive/measured_twist` | `geometry_msgs/TwistStamped` | 由左右 ESC 的 `tacho_absolute` 差分算出（`publishStatus():280-315`，差速轮式里程计公式） |
| `ll/diff_drive/left_esc_status`、`right_esc_status` | `mower_msgs/ESCStatus` | `convertStatus()` 从 `xesc_msgs/XescStateStamped` 转换 |
| `ll/mower_status` | `mower_msgs/Status` | 组合 LL 板状态位（`status_bitmask`：raspi 供电/充电/ESC供电/雨感/声音模块/UI板）与割草 ESC 状态 |
| `ll/imu/data_raw` | `sensor_msgs/Imu` | `handleLowLevelIMU()`（`mower_comms.cpp:619-643`），来自串口 `ll_imu` 帧（加速度/角速度，注意没有把磁力计数据填进 `sensor_msgs/Imu`，只在内部 `mower_msgs::ImuRaw` 结构体里保留） |
| `ll/power` | `mower_msgs/Power` | 来自 `ll_status` 帧的 `v_system`/`v_charge`/`charging_current` |

**服务端**：`ll/_service/mow_enabled`（`MowerControlSrv`）、`ll/_service/emergency`（`EmergencyStopSrv`，与 V2 语义一致：reason!=0 时设置急停并清掉"待清除"标志；reason==0 时设置"下次心跳清除"标志）。

**串口帧（HL → LL 方向）**：`ll_heartbeat`（每 20ms，`PACKET_ID_LL_HEARTBEAT=0x42`，含 `emergency_requested`/`emergency_release_requested`）、`ll_high_level_state`（`PACKET_ID_LL_HIGH_LEVEL_STATE=0x43`）、`ll_high_level_config`（配置请求/应答，`PACKET_ID_LL_HIGH_LEVEL_CONFIG_REQ/RSP`，弹性长度结构体，允许两端字段数量不一致时只拷贝共同前缀）。

#### 主要算法/流程

1. **COBS 帧 + CRC16 协议**：所有结构体都是 `__attribute__((packed))` 定长（或半弹性长度）二进制结构，末尾 2 字节是 `boost::crc_ccitt_type` CRC16 校验和。发送前用 `COBS::encode()`（`src/mower_comms_v1/src/COBS.h:35-62`）把数据里所有可能出现的 `0x00` 字节转义掉，末尾补一个 `0x00` 作为帧边界；接收方在主循环里逐字节读串口（`mower_comms.cpp:805-873`），遇到 `0x00` 就调用 `COBS::decode()`（`COBS.h:70-97`）还原整帧，再校验 CRC，最后按首字节 `type` 分发到 `handleLowLevelStatus/IMU/UIEvent/Config`。这一整套协议与 `xesc_2040_driver`（见下）使用的 COBS+CRC 思路完全一致，是这个项目里"简单可靠的定长二进制串口协议"的通用模式。
2. **主循环即状态机**（`main()` 内的 `while(ros::ok())` 循环，`mower_comms.cpp:784-874`）：串口未打开时先尝试 `open()`，成功后 `sleep(5s)` 等待 Arduino/LL 板完成自举（`allow_send` 在等待期间保持 false，防止过早发送指令），随后逐字节读取、找帧尾、解码、按类型分发；串口异常则关闭并重试。`publishActuatorsTimerTask`（20ms 周期定时器）负责下发心跳/占空比并发布状态，是与串口读取解耦的独立发送路径。
3. **急停超时保护**（`publishActuators()`，`mower_comms.cpp:117-164`）：`ll/cmd_vel` 超过 1 秒未更新则轮速置零，超过 25 秒未更新则连割草电机也一并置零（双重超时阈值），叠加"处于任一急停状态则强制全零"的检查。
4. **配置双向同步 + 脏检查重传**（`checkAndSendConfig()`/`handleLowLevelConfig()`/`configTracker`，`mower_comms.cpp:390-684`）：`mower_logic`/`power` 的 dynamic_reconfigure 值变化 → `reconfigCB`/`powerReconfigCB` → `checkAndSendConfig()` 用 `getNewSetChanged<T>()`模板逐字段比较差异，任何字段变化则把本地 `configTracker` 标记为"脏"（`setDirty()`，5 次重试机会），下一次 `publishLowLevelConfig(PACKET_ID_LL_HIGH_LEVEL_CONFIG_REQ)` 会真正把整份配置结构体发给 LL 板并等 0.5s 应答；反过来 LL 板发来的 `CONFIG_REQ/RSP` 会触发 `handleLowLevelConfig()` 把 LL 侧真源值写回两个 dynamic_reconfigure client。这套机制还兼顾了 LL 板重启/刷机检测：若 100ms 周期的 `ll_status` 帧超过 1 秒未到，视为 LL 已重启，主动 `setDirty()` 重新同步配置。
5. **紧急输入配置解析**（`checkAndSendConfig()` 内对 `mower_logic_config.emergency_input_config` 字符串的解析，`mower_comms.cpp:662-681`）：逗号分隔的多组 token，每组内 `!`(低电平有效前缀) + `I/L/S/U`（对应 `HallMode::OFF/LIFT_TILT/STOP/UNDEFINED`）字符序列，映射到 `llhl_config.hall_configs[]` 数组（最多 `MAX_HALL_INPUTS=10` 个霍尔/抬升传感器）。

#### xesc_ros（xESC 驱动库群）

**角色**：`xesc_driver::XescDriver`（`src/lib/xesc_ros/xesc_driver/src/xesc_driver.cpp:3-27`）是一个按 `xesc_type` 参数派发的工厂：`"xesc_2040"` → `xesc_2040_driver::Xesc2040Driver`（OpenMower 官方 RP2040 电调板，默认路径）；`"xesc_mini"` → `vesc_driver::VescDriver`（标准 VESC 协议，兼容第三方 VESC 电调）；`"xesc_yfr4"` → `xesc_yfr4_driver::XescYFR4Driver`（YardForce R4 主板电调）。三者都实现统一的 `xesc_interface::XescInterface`（`getStatus/getStatusBlocking/setDutyCycle/stop`，`src/lib/xesc_ros/xesc_interface/include/xesc_interface/xesc_interface.h`），因此 `mower_comms_v1` 对具体是哪种电调无感知。

**xesc_2040 协议**（`src/lib/xesc_ros/xesc_2040_driver/include/xesc_2040_driver/xesc_2040_datatypes.h`）：同样是 COBS 帧（复用一份几乎相同的 `COBS.h`），三种定长 `__attribute__((packed))` 结构体：`Xesc2040StatusPacket`（含固件版本、输入电压、PCB/电机温度、电流、占空比、tacho 计数、转向、`fault_code` 位掩码：未初始化/看门狗超时/欠压/过压/过流/电机过温/PCB过温/霍尔非法）、`Xesc2040ControlPacket`（占空比指令）、`Xesc2040SettingsPacket`（霍尔表 `hall_table[8]`、电流限制、加速度、温度上下限）。`Xesc2040Driver` 构造时从 ROS 参数读取这些设置项（`serial_port`、`hall_table_0..7`、`motor_current_limit`、`acceleration`、`has_motor_temp`、温度上下限）并在启动时下发给电调板。

---

### mower_simulation

**角色**：一个"假主板"，作为 xbot-service 协议的 **Service 侧**实现，暴露与真实 Stage-2 主板完全相同的 7 个 service（`Emergency/DiffDrive/Mower/Imu/Power/Gps/Meta`，唯独不实现 `Input`/`HighLevel`/`Bms`/`RemoteGPIO`——这几个在仿真中不需要，`mower_comms_v2` 对应的 Interface 只是永远发现不了对端，不影响其它功能），让 `mower_comms_v2` 以为自己在跟真硬件说话。同时内置一个简单的 2D 运动学物理模型和电池充放电模型，并通过 `xbot_mqtt` 暴露一组测试用 RPC（`sim.*`）供 App/测试脚本摆布仿真状态。它在整体架构里"替换"的是真实主板 + LL 板固件这一层，因此可以在没有任何硬件的 PC 上跑通完整的 `mower_logic` 状态机与导航链路（正是 `doc/architecture.md` 里提到的"仿真复用几乎全部真实逻辑代码"的原因）。

**节点**：`mower_simulation`（`src/mower_simulation/src/mower_simulation.cpp:49` `main()`），由 `src/mower_simulation/launch/_mower_simulation.launch:3-5` 启动（`~xb_pose_out` 重映射到 `xbot_positioning/xb_pose`），docker-simulation 环境里由 `supervisord.conf:39-47` 通过 `roslaunch mower_simulation_gui.launch` 启动。

#### 输入

**服务客户端**：`mower_map_service/get_docking_point`（`mower_map::GetDockingPointSrv`）——启动时阻塞等待该服务出现（最多 300s），再轮询最多 20 次（每次间隔 1s）读取充电桩位姿，成功则把机器人初始位置和朝向直接设为充电桩位姿（`mower_simulation.cpp:78-104`）；若一直拿不到，机器人从原点 (0,0,0) 开始。

**订阅话题**：`/joy_vel`（`geometry_msgs/Twist`，`SimRobot::OnJoyVel`，`SimRobot.cpp:129-136`）——仅在 `joy_override_` 开启（由 `sim.joy_override.set` RPC 控制）时才会覆盖正常的 `SetControlTwist()` 输入，用于手柄/App 直接摇杆遥控测试场景。

**xbot-service 数据流入（作为 Service 侧，接收 Interface 侧即 mower_comms_v2 发来的 input）**：
| Service | 接收到的 input | 处理 |
|---|---|---|
| `EmergencyService` | `High Level Emergency`=`(add, clear)` | `ApplyEmergencyUpdate()` 更新 `emergency_reasons_` 位掩码（只清除点名的位），见下方"急停位运算"算法 |
| `DiffDriveService` | `Control Twist`=`double[6]` | 只取 `[0]`(linear.x) 和 `[5]`(angular.z) 调 `robot_.SetControlTwist()` |
| `MowerService` | `Mower Speed`（signed float duty） | `mower_running_ = (new_value != 0)` |
| `PowerService` | `Charging Allowed` | 当前忽略（sim 里充电总是允许） |
| `GpsService` | `RTCM` | 未处理（GpsServiceBase 未覆盖对应回调） |

**ROS 参数**：`~publish_tf`（默认 `true`，是否自己广播 `map→base_link` TF；与真实 `xbot_positioning` 同时跑时需要设为 `false` 防止两者抢发同一条 TF）；`/xbot_positioning/antenna_offset_x`、`antenna_offset_y`（GPS 天线相对机体中心的偏移，`GpsService` 读取用于把机体中心位置换算成天线位置，见下方算法）；`dynamic_reconfigure::Server<mower_simulation::MowerSimulationConfig>`（`cfg/MowerSimulation.cfg`：`battery_voltage`、`temperature_mower`、`is_charging`、`mower_error`、`mower_running`、`wheels_stalled`、`emergency_stop`、`rain` 等调试用参数，当前代码里 `reconfig_server->setCallback` 被注释掉，即这些参数目前未接入实际逻辑）。

**xbot_mqtt RPC（`SimRpc`，见下方 xbot_mqtt 一节的协议说明）**，均注册在 `SimRpc::Start()`（`SimRpc.cpp:69-145`）：
| 方法 | 参数 | 效果 |
|---|---|---|
| `sim.emergency.set` | `{active: bool}` | true → `TriggerEmergency()`（锁存 `HIGH_LEVEL\|LATCH`）；false → `ClearEmergency()`（清空全部原因） |
| `sim.movement.set` | `{allowed: bool}` | 控制“卡死”模拟：false 时轮子仍转但地面真值位置冻结 |
| `sim.battery.set` | `{voltage: number}` | 直接设定电池电压（允许超出正常范围以模拟过压/欠压故障） |
| `sim.gps.set` | `{good: bool}` | 切换 RTK Fix（2cm 精度）/ 无 Fix（1m 精度） |
| `sim.dock.move` | 无 | 瞬移到充电桩位姿并开始充电 |
| `sim.displace` | `{dx, dy, dheading}`（均可选，默认 0） | 瞬间位移，模拟 GPS 跳变 |
| `sim.joy_override.set` | `{enabled: bool}` | 切换 `/joy_vel` 直接接管 |
| `sim.state.get` | 无 | 返回当前仿真控制状态快照（JSON） |

#### 输出

**发布话题**：
| 话题 | 类型 | 来源 |
|---|---|---|
| `~odom_out` | `nav_msgs/Odometry` | `SimRobot::PublishPosition()`（`SimRobot.cpp:295-342`），50Hz(20ms 定时器) |
| `~xb_pose_out`（launch 里重映射到 `xbot_positioning/xb_pose`） | `xbot_msgs/AbsolutePose` | 同上，`source=SOURCE_SENSOR_FUSION`，直接把仿真真值位姿当作"融合后位姿"喂给下游，绕过了真实的 EKF（因为仿真不需要再做一次定位融合） |
| `map→base_link` TF | — | 同上，仅当 `publish_tf_` 为真 |
| `/xbot_monitoring/mqtt_publish` | `xbot_mqtt/MqttPublish` | `SimRpc::PublishState()`，1Hz 定时器持续把仿真控制状态（`sim/state/json` topic，`retain=true`）广播给 MQTT/App，同时每次 RPC 调用后也立即广播一次 |

**xbot-service 数据流出（作为 Service 侧发出的 output）**——这些是喂给 `mower_comms_v2` 从而变成 `/ll/*` 话题的原始数据源，等价于真实主板会发的数据：
| Service | output | 数值来源 |
|---|---|---|
| `EmergencyService` | `Emergency Reason` | `robot_.GetEmergencyState()`，100ms tick |
| `DiffDriveService` | `Actual Twist`（`[vx,0,0,0,0,vr]`）、`Left/RightESCStatus`（固定 200=OK） | `robot_.GetTwist()`（含高斯噪声），20ms tick，与 `DiffDriveServiceInterface` 的 20ms 发送周期保持一致 |
| `MowerService` | `MowerRunning`、`RainDetected`(false)、`MowerMotorCurrent`(1.0/0.0)、`MowerMotorRPM`(4500/0)、`MowerStatus`(200)、电机/ESC温度(25.0/35.0 固定) | 1s tick，`running = !emergency && mower_running_` |
| `ImuService` | `Axes[9]`（仅 `[5]`=角速度有值，其余为 0） | 10ms tick |
| `PowerService` | `BatteryVoltage`、`ChargeVoltage`、`ChargeCurrent`、`ChargerEnabled`(true)、`ChargingStatus`（字符串状态机） | 200ms tick，来自 `robot_.GetIsCharging()` |
| `GpsService` | `FixType`("FIX"/"FLOAT")、`PositionHorizontalAccuracy`(0.02/1.0)、`Position[3]`（天线位置）、`MotionVectorENU[3]` | 200ms tick |
| `MetaService` | RPC `GetFirmwareVersion`→`"sim-1.0.0"`、`GetMajorVersion`→`1` | 固定值，保证 `mower_comms_v2` 的固件兼容性门控放行 |

#### 主要算法/流程

1. **运动学积分（差速轮式，`SimRobot::SimulationStep`，`SimRobot.cpp:198-243`）**：每 20ms tick 一次。地面真值位置只积分**指令速度**（`vx_`, `vr_`），不叠加传感器噪声，避免噪声在真实轨迹上产生随机游走；噪声只加在"上报"的轮速计/IMU 读数（`last_noisy_vx_/vr_`）上，且静止时（`vx_==0 && vr_==0`）跳过噪声，避免静止的机器人也出现虚假抖动。转弯积分用标准的圆弧运动学：`r = vx/vr`；`pos_x += r*(sin(θ+vr·dt) - sinθ)`；`pos_y -= r*(cos(θ+vr·dt) - cosθ)`；直线运动（`|vr|≈0`）则退化为 `pos += v·(cosθ, sinθ)·dt`。`movement_allowed_=false` 时地面真值位置冻结但噪声速度仍照常上报，模拟"轮子打滑/卡住但编码器还在转"的故障场景。
2. **急停位运算**：与 `mower_comms_v2` 侧完全对称——`ApplyEmergencyUpdate(add, clear)` 执行 `reasons_ = (reasons_ & ~clear) | add`（`SimRobot.cpp:47-50`），`EmergencyService::OnHighLevelEmergencyChanged` 每次收到 mower_comms_v2 的心跳都调用它；`EmergencyService::tick()`（`emergency_service.cpp:29-46`）额外实现了"高层超时"检测：若超过 1 秒未收到高层心跳消息，自动加上 `TIMEOUT_HIGH_LEVEL` 位（与真实固件行为一致），收到心跳后清掉该位。`SimRobot::SimulationStep` 里 `emergency_reasons_ != 0`（任意原因）就让上报速度归零，模拟真实急停切断电机的效果。
3. **GPS 天线偏移换算（`GpsService::tick`，`gps_service.cpp:7-42`）**：`xbot_positioning` 的观测模型假定收到的 GPS 位置是**天线**而非车体中心的位置，因此仿真必须做同样的偏移补偿：`antenna_pos = center_pos + R(heading)·antenna_offset`；速度上还要叠加刚体转动带来的切向速度分量 `v_antenna = v_center + ω × r`（代码里手工展开为 `body_vx = vx - vr*offset_y`，`body_vy = vr*offset_x`，再旋转到 ENU 系），否则原地转向时仿真会错误地让天线速度为零。
4. **充电状态机（`SimRobot::SimulationStep`，`SimRobot.cpp:245-289`）**：用离充电桩距离做**迟滞判断**（<0.02m 进入充电、>0.03m 退出充电，防止噪声导致的抖动反复触发），进入时把位置吸附到充电桩精确坐标消除漂移。充电电压/电流按 CC(恒流)→CV(恒压)→Done 三段式模型：CC 阶段电压线性爬升（+0.05V/tick）直到 `BATTERY_VOLTS_MAX`(=4.18V×7=29.26V)；CV 阶段电流以 0.99 的比例每 tick 衰减直到 <0.2A 转入 `Done`；不充电时电压以 -0.001V/tick 缓慢下降但不低于 `BATTERY_VOLTS_MIN`(=3.2V×7=22.4V)。
5. **服务发现与 tick 调度**：每个 xbot Service 子类通过 `ManagedSchedule tick_schedule_{scheduler_, IsRunning(), <微秒间隔>, ...}` 声明式地绑定周期回调（如 `DiffDriveService` 20ms、`GpsService`/`PowerService` 200ms、`ImuService` 10ms、`EmergencyService` 100ms、`MowerService` 1s），这是 `xbot::service::Service` 基类提供的通用机制（见 xbot_framework 一节），仿真的各 service 只需在自己的 `tick()` 里用 `StartTransaction()/SendXxx()/CommitTransaction()` 把最新状态批量打包发送。
6. **Meta/固件版本模拟**：`MetaService::RPCGetMajorVersion` 硬编码返回 `1`，这是让 `mower_comms_v2::OnFirmwareInfoChanged` 判定"固件兼容"、从而解锁运动与割草指令的唯一条件——如果去掉这个 service，仿真会永久卡在"固件不兼容"的急停状态。

---

### mower_msgs / xbot_msgs（纯消息/服务定义包）

两者都不含可执行节点，仅提供 `.msg`/`.srv` 给上面各通讯节点、`mower_logic`、`xbot_positioning`、`xbot_monitoring` 等共用。

#### mower_msgs（`src/mower_msgs/msg`、`src/mower_msgs/srv`）

| 文件 | 用途 |
|---|---|
| `msg/Bms.msg` | 独立 BMS（电池管理芯片）读数：电压、电流、SOC、剩余/满容量、循环次数、温度、`battery_status`（字符串化的告警位）、`extra_data` |
| `msg/DockingSensor.msg` | 充电桩对接传感器（左右探测标志），当前通讯层代码未见发布方，供硬件扩展预留 |
| `msg/Emergency.msg` | 急停状态：`active_emergency`/`latched_emergency`/`reason`（字符串） |
| `msg/ESCStatus.msg` | 单个电调状态：`status`（`DISCONNECTED=99/ERROR=100/STALLED=150/OK=200/RUNNING=201`）、电流、tacho、rpm、电机/PCB 温度 |
| `msg/HighLevelStatus.msg` | `mower_logic` 状态机广播：`state`(`IDLE/AUTONOMOUS/RECORDING`)、状态名/子状态名、`job_id`/`session_id`、当前区域/路径/路径点索引、GPS 质量百分比、电池百分比、是否充电、是否急停——是 `mower_comms_v2` 的 `HighLevelServiceInterface`/`InputServiceInterface` 的输入来源 |
| `msg/ImuRaw.msg` | V1 专用原始 IMU（含磁力计 3 轴），V2 走标准 `sensor_msgs/Imu`（无磁力计） |
| `msg/Perimeter.msg` | 电子围栏线圈信号（历史/预留功能，通讯层代码未见使用） |
| `msg/Power.msg` | 电源状态：充电/电池电压（含 ADC 原始值）、充电电流、DCDC/充电器输入电流、`charger_enabled`、`charger_status` |
| `msg/Status.msg` | 割草机整体状态：LL 板状态位（raspi 供电/充电/ESC供电/雨感/声音/UI板可用）、`mow_enabled`、割草 ESC 状态/温度/电流/电机温度/RPM |
| `srv/EmergencyStopSrv.srv` | `reason(uint16)` → 空响应；`ll/_service/emergency` |
| `srv/GPSControlSrv.srv` | `gps_enabled(uint8)` → 空响应 |
| `srv/HighLevelControlSrv.srv` | `command`：`COMMAND_START=1/HOME=2/S1=3/S2=4/RESET_EMERGENCY=254/DELETE_MAPS=255`；`mower_service/high_level_control` 的请求类型 |
| `srv/MowerControlSrv.srv` | `mow_enabled`、`mow_direction(uint8)` → 空响应；`ll/_service/mow_enabled` |
| `srv/PerimeterControlSrv.srv` | `listenOn(uint8)`：1/2 选择围栏信号，其余关闭 |

#### xbot_msgs（`src/lib/xbot_msgs/msg`、`src/lib/xbot_msgs/srv`）

| 文件 | 用途 |
|---|---|
| `msg/AbsolutePose.msg` | 通用"绝对位姿测量"消息，`source`（`SOURCE_GPS/LIGHTHOUSE/SENSOR_FUSION`）+ RTK 相关标志位（`FLAG_GPS_RTK*`）+ 位姿/精度/运动矢量/朝向；是 `ll/position/gps` 与 `xbot_positioning/xb_pose` 的公共载体 |
| `msg/ActionInfo.msg` | 一个可触发"动作"的描述（id/name/enabled），配合 `RegisterActionsSrv` |
| `msg/MapOverlay.msg`、`MapOverlayPolygon.msg` | 地图叠加多边形（用于 App/RViz 可视化，非通讯层核心） |
| `msg/MapSize.msg` | 地图尺寸/中心 |
| `msg/RobotState.msg` | 面向人类可读的整体机器人状态摘要（供监控展示） |
| `msg/SensorDataDouble.msg`、`SensorDataString.msg`、`SensorInfo.msg` | 通用传感器数据总线（热力图等扩展传感器用），含量程/告警阈值等元信息 |
| `msg/WheelTick.msg` | 通用四轮编码器计数消息（V1/其他机型可能用到，V2 走 `DiffDriveService` 的 `Wheel Ticks` output） |
| `srv/RegisterActionsSrv.srv` | 节点向某个中心（如 App/监控）注册它提供的一组 `ActionInfo` |

---

### xbot_mqtt

**角色**：一个通用的、基于 ROS 话题桥接 MQTT 的轻量 JSON-RPC 库，供 `xbot_monitoring`（把机器人暴露给 App/Home Assistant）和 `mower_simulation` 的 `SimRpc`（`sim.dock.move`、`sim.battery.set` 等测试接口）共用。它本身不直接连 MQTT broker（那是 `xbot_monitoring` 的职责），只是定义了一套 ROS 内部的请求/响应/错误话题约定，`xbot_monitoring` 负责把这些话题内容真正搬到 MQTT 上。

**节点**：无独立节点，是头文件库（`include/xbot_mqtt/{provider,publish,constants}.h`）+ 一个 `.cpp`（`src/provider.cpp`），链接进使用方的可执行文件。

#### 输入（作为库，被使用方“输入”的部分）

- **订阅**：`RpcProvider::init()`（`provider.cpp:7-20`）订阅固定话题 `/xbot/rpc/request`（`xbot_mqtt/RpcRequest`：`method`/`params`(JSON字符串)/`id`），收到后在 `handleRequest()`（`provider.cpp:34-61`）里按 `method` 查表分发到通过 `addMethod()` 注册的回调（`std::function<json(method, params)>`），找不到则静默忽略（可能是给另一个 provider 的）。
- **服务客户端**：`/xbot/rpc/register`（`xbot_mqtt/RegisterMethodsSrv`，`node_id` + `methods[]`）——启动时把自己注册的方法名列表报给一个中心注册服务（等待最多 10s，找不到只警告不中断，因为 RPC 对系统核心功能不是必需的）。

#### 输出

- **发布**：`/xbot/rpc/response`（`RpcResponse`：`result`(JSON字符串)+`id`，仅当请求带了非空 `id` 才发）；`/xbot/rpc/error`（`RpcError`：`code`+`message`+`id`，JSON-RPC 2.0 风格错误码：`INVALID_JSON=-32700/INVALID_REQUEST=-32600/METHOD_NOT_FOUND=-32601/INVALID_PARAMS=-32602/INTERNAL=-32603`，另预留 `SERVER_FIRST/LAST=-32000/-32099` 给实现方自定义错误）。
- **辅助发布接口**（`publish.h`，非 RPC，用于单向事件/状态广播）：`xbot_mqtt::publish(pub, topic, json, retain)` 把任意 JSON 打包成 `xbot_mqtt/MqttPublish`（`topic`/`payload`/`retain`）发到调用方自己 advertise 的话题（约定发到 `/xbot_monitoring/mqtt_publish`，由 `xbot_monitoring` 转发上 MQTT）；`buildEventPayload()`/`publishEvent()` 是在此之上封装的"事件"辅助函数，自动注入 `id`（`generateNanoId()`，64 字符字母表的随机 ID，与 `mower_map_service` 用的算法一致）、`t`（ROS 时间秒）、`type` 字段，统一发到 `events/json` 主题。

#### 主要算法/流程

1. **RPC 分发是单播查表，非广播共识**：所有 provider 都订阅同一个 `/xbot/rpc/request` 话题，靠方法名唯一性做路由（`handleRequest` 对未知方法直接 `return`），因此系统里任意数量的 provider 可以共存而不冲突，只要方法名不重复；`RegisterMethodsSrv` 只是"登记"用途（供 `xbot_monitoring` 知道要把哪些方法暴露到 MQTT），不参与实际的请求路由。
2. **同步风格的异步协议**：`RpcRequest`/`RpcResponse`/`RpcError` 都是 fire-and-forget 的 ROS 消息，没有内置超时/重试——调用方（如 App 通过 `xbot_monitoring` 转发）需要自己维护 `id` 到等待中请求的映射并处理超时；`SimRpc` 里的用法都是"发了就忘"（返回值在回调里直接 `return json` 由 `publishResponse` 同步发布，本地调用链是同步阻塞的，只是外部看到的 MQTT 往返是异步的）。
3. **`RpcException` 作为控制流**（`provider.h:18-30`，`provider.cpp:53-60`）：业务回调直接 `throw xbot_mqtt::RpcException(code, message)` 即可让 `handleRequest` 捕获并转成规范的 `RpcError` 响应（`SimRpc.cpp` 里 `requireBool`/`requireNumber`/`optionalNumber` 三个参数校验辅助函数就是这么做参数校验失败上报的，见 `SimRpc.cpp:20-47`）；其余未预期的 `std::exception` 会被统一包装成 `ERROR_INTERNAL`。

---

## 二、定位

融合 IMU、轮速里程计与 RTK GPS，输出机器人在 `map` 坐标系下的实时位姿，是导航与状态机决策的位姿真值来源。

### xbot_positioning

**角色**: OpenMower 定位核心节点。使用扩展卡尔曼滤波器 (EKF) 融合 IMU 角速度、轮式里程计(或差速驱动测得的线速度)与 RTK GPS 绝对位姿，估计机器人在 `map` 系下的二维位姿(x, y, θ)与线速度/角速度，并发布 `map -> base_link` 里程计变换与 `xbot_msgs/AbsolutePose`。是 OpenMower 导航栈的位姿真值来源。

**节点**: `xbot_positioning`（可执行文件 `xbot_positioning`，源码 `src/lib/xbot_positioning/src/xbot_positioning.cpp`，EKF 数学核心在 `src/lib/xbot_positioning/src/xbot_positioning_core.{h,cpp}`，模型定义在同目录 `SystemModel.hpp`、`PositionMeasurementModel.hpp`、`OrientationMeasurementModel.hpp`、`OrientationMeasurementModel2.hpp`、`SpeedMeasurementModel.hpp`）。EKF 框架来自第三方头文件库 `include/kalman/*.hpp`（`Kalman::ExtendedKalmanFilter`）。

实际部署中由 `src/open_mower/launch/include/_localization.launch` 启动，`respawn="true"`：

```xml
<node pkg="xbot_positioning" type="xbot_positioning" name="xbot_positioning" output="screen" respawn="true" respawn_delay="10">
    <remap from="~imu_in" to="/ll/imu/data_raw"/>
    <remap from="~twist_in" to="/ll/diff_drive/measured_twist"/>
    <remap from="~xb_pose_in" to="/ll/position/gps"/>
    <remap from="~xb_pose_out" to="xbot_positioning/xb_pose"/>
</node>
```

#### 输入

订阅话题（均为私有命名空间 `~`下的话题名，经 launch remap 后的实际全局名见括号）：

| 话题(参数名) | 类型 | 实际来源 | 用途 | 代码位置 |
|---|---|---|---|---|
| `~imu_in`（`/ll/imu/data_raw`） | `sensor_msgs/Imu` | 底层主控板(low-level board)IMU | 提供角速度 `angular_velocity.z`，驱动 EKF 预测步；回调 `onImu` 是整个节点的主循环节拍源（每次 IMU 消息都会 predict + 发布一次里程计） | `src/lib/xbot_positioning/src/xbot_positioning.cpp:77-174` |
| `~twist_in`（`/ll/diff_drive/measured_twist`） | `geometry_msgs/TwistStamped` | 差速驱动里程计节点，由轮速推算出的车体线速度 | 更新全局变量 `vx`（车体前向速度），作为下一次 `predict()` 的控制输入 `u.v()` | `xbot_positioning.cpp:203-205` (`onTwistIn`) |
| `~xb_pose_in`（`/ll/position/gps`） | `xbot_msgs/AbsolutePose` | GPS 驱动 (`xbot_driver_gps`，经 UTM 投影后的绝对位姿+RTK标志+速度矢量) | GPS 位置/航向更新的观测输入，见 `onPose` | `xbot_positioning.cpp:225-307` |
| `~wheel_ticks_in`（默认话题名，**未在 `_localization.launch` 中 remap，实际生产环境未接线**） | `xbot_msgs/WheelTick` | 轮编码器 | 备用的车轮里程计输入方式，将左右轮位移差分转换为 `vx`；与 `~twist_in` 是二选一的速度来源（生产配置实际用 twist） | `xbot_positioning.cpp:176-201` (`onWheelTicks`) |

服务端（本节点**提供**的服务，非订阅，但按“输入”一并列出，因为外部通过它们向 EKF 注入控制指令）：

- `xbot_positioning/set_gps_state`（`xbot_positioning/GPSControlSrv`，字段 `uint8 gps_enabled`）：运行时启用/禁用 GPS 融合，见 `setGpsState()`，`xbot_positioning.cpp:207-210`。
- `xbot_positioning/set_robot_pose`（`xbot_positioning/SetPoseSrv`，字段 `geometry_msgs/Pose robot_pose`）：外部（例如手动定位、对接点标定）强制重置 EKF 状态为给定 x,y,yaw，速度清零，协方差重置为单位阵，见 `setPose()`，`xbot_positioning.cpp:212-223`，底层调用 `xbot_positioning_core::setState()`（`xbot_positioning_core.cpp:84-96`）。

节点参数（均在私有命名空间 `~` 下，来自 `src/open_mower/launch/include/_params.launch:85-89` 及 `src/open_mower/params/openmower_defaults_v2.yaml:20-22`）：

| 参数 | 默认值(代码内) | 部署默认值 | 含义 |
|---|---|---|---|
| `skip_gyro_calibration` | `false` | 未覆盖 | 若为 `true`，跳过开机 5 秒陀螺仪零偏标定，直接使用 `gyro_offset` 参数值 |
| `gyro_offset` | `0.0` | 未覆盖 | 陀螺仪 z 轴零偏(rad/s)，标定完成后由代码计算得出；也可手工指定并配合 `skip_gyro_calibration` | 
| `min_speed` | `0.01` (m/s) | 未覆盖 | GPS 航向修正(`updateOrientation2`)生效的最小车速阈值：只有 GPS 给出的运动矢量模长 ≥ 该值时才用它修正朝向，避免静止/低速时用带噪声的速度方向"污染"航向估计 |
| `max_gps_accuracy` | `0.1` (m) | **`0.2`**（`openmower_defaults_v2.yaml`及`_params.launch`覆盖为0.2） | GPS 位置更新的精度门限：`AbsolutePose.position_accuracy` 超过该值的 GPS 消息被丢弃 |
| `debug` | `false` | `false` | 是否发布调试话题(`kalman_state`、`debug_expected_motion_vector`) |
| `antenna_offset_x` | `0.0` | 由环境变量 `OM_ANTENNA_OFFSET_X` 决定（如 YardForce500 默认 0.3m，YardForceSA650 默认 0.1m） | GPS 天线相对车体旋转中心(base_link)在车体坐标系前向(x)的偏移，用于杆臂补偿 |
| `antenna_offset_y` | `0.0` | 由环境变量 `OM_ANTENNA_OFFSET_Y` 决定（常见默认 0.0） | 同上，车体坐标系横向(y)偏移 |

注：代码里还有一段被注释掉的过程噪声设置（`xbot_positioning_core.cpp:76-82`，`c *= 0.001` 后 `sys.setCovariance(c)`），实际**从未调用**，因此系统模型 `SystemModelT` 的过程噪声协方差 `Q` 始终保持 `Kalman::StandardBase` 构造函数给出的默认值——**单位阵**（见 `include/kalman/StandardBase.hpp:83-86`）。这意味着预测步骤引入的不确定性并未针对该机器人精细调参，是潜在的调优点。

#### 输出

发布话题：

| 话题(参数名) | 类型 | 内容/用途 | 频率 |
|---|---|---|---|
| `~odom_out`（`nav_msgs/Odometry`，队列 50） | `nav_msgs/Odometry` | `header.frame_id="map"`，`child_frame_id="base_link"`；位置取 EKF 状态 x,y，姿态仅含 yaw(通过 `tf2::Quaternion(0,0,theta)` 构造，注意此处构造方式并非标准四元数轴角构造，而是把 theta 直接塞进 z 分量，等效于小角度近似——见下方"主要算法"节的说明) | 每次收到 IMU 消息时发布一次(即 IMU 频率) |
| `~xb_pose_out`（remap 为 `xbot_positioning/xb_pose`） | `xbot_msgs/AbsolutePose` | 融合后的绝对位姿，`source=SOURCE_SENSOR_FUSION`；`position_accuracy` 直接沿用最近一次 GPS 消息里的精度（若 10 秒内无 GPS 或从未收到过 GPS，则设为 999 表示不可信）；`flags` 含 `FLAG_SENSOR_FUSION_DEAD_RECKONING`，若最近 10 秒内有 GPS 数据则再加 `FLAG_SENSOR_FUSION_RECENT_ABSOLUTE_POSE`；`vehicle_heading`/`motion_heading` 均取 EKF 的 `theta` | 与 `odom_out` 同频(IMU频率) |
| `~debug_expected_motion_vector`（仅 `debug=true` 时发布） | `geometry_msgs/Vector3` | EKF 用 `OrientationMeasurementModel2::h()` 根据当前状态预测出的期望 GPS 速度矢量，便于和真实 GPS 速度矢量比对调试 | 每次收到有效 GPS 更新时 |
| `~kalman_state`（仅 `debug=true` 时发布） | `xbot_positioning/KalmanState`（`x,y,theta,vx,vr` 五个 float64） | EKF 状态向量原始值，调试用 | IMU 频率 |

TF：通过局部 `tf2_ros::TransformBroadcaster` 发布动态变换 `map -> base_link`（`odom_trans`，平移取状态 x,y,0，旋转同 `odometry.pose.pose.orientation`），在 `onImu` 回调内每次预测后广播，见 `xbot_positioning.cpp:119-128`。注意：该节点直接发布 `map->base_link`，并非 `odom->base_link`，即 OpenMower 定位不设中间 `odom` 系，EKF 输出即全局地图系下的位姿。

服务：见上方“输入”表中 `xbot_positioning/set_gps_state` 与 `xbot_positioning/set_robot_pose`（这两个是"输出"意义上的服务提供方）。

#### 主要算法/流程

**状态向量**（`SystemModel.hpp:22-56`，5 维）：
`x = [x_pos, y_pos, theta, vx, vr]`
其中 `x_pos,y_pos` 为 map 系下位置（米），`theta` 为绕 z 轴航向角（弧度，未做 wrap-to-pi），`vx` 为车体纵向速度，`vr` 为角速度。

**控制向量**（`SystemModel.hpp:67-85`，2 维）：`u = [v, dtheta]`，其中 `v` 来自轮速/twist（`onImu` 中的全局变量 `vx`），`dtheta` 来自陀螺仪 z 轴角速度减去标定零偏。

**预测步 (Predict)**：由 `onImu()` 每收到一帧 IMU 数据即调用（`xbot_positioning.cpp:106`）：
```
core.predict(vx, msg->angular_velocity.z - gyro_offset, dt)
```
`dt` = 本次与上次 IMU 消息时间戳之差。状态转移函数 `f(x,u)`（`SystemModel.hpp:123-147`）：
```
theta' = theta + dtheta*dt
x'     = x + cos(theta')*v*dt
y'     = y + sin(theta')*v*dt
vx'    = vx   (保持不变)
vr'    = vr   (保持不变)
```
即一阶欧拉积分的差速机器人运动学模型，且用的是"预测后"的新航向角(`theta'`)来计算本步位移方向（略微修正欧拉积分误差，属于中点法的近似变体）。**注意状态里的 `vx,vr` 分量并不参与位置/航向的推算**——真正驱动位置演化的是外部传入的控制量 `u.v(), u.dtheta()`，状态自带的 `vx,vr` 只是被"顺路"存储、并只能通过测量更新（`updateSpeed`/`updateOrientation2`）来改变，二者在雅可比 `F` 中也是独立对角项（`F(VX,VX)=1, F(VR,VR)=1`），与 `x,y,theta` 无耦合。

状态转移雅可比 `F`（`updateJacobians`，`SystemModel.hpp:166-194`）：
```
F(X,X)=1        F(X,THETA)=-sin(theta+dtheta*dt)*v*dt
F(Y,Y)=1        F(Y,THETA)= cos(theta+dtheta*dt)*v*dt
F(THETA,THETA)=1
F(VX,VX)=1
F(VR,VR)=1
```
噪声雅可比 `W` 恒为单位阵（代码注释里承认这是简化，TODO 待做更精细的噪声建模）。过程噪声协方差 `Q`（即 `sys.getCovariance()`）保持默认单位阵（见"输入"节末尾说明），未按运动噪声/陀螺噪声实际量级调参。

标准 EKF 预测/更新公式（`include/kalman/ExtendedKalmanFilter.hpp:107-150`，取自 Welch & Bishop 教程实现）：
```
预测: x = f(x,u);  P = F·P·Fᵀ + W·Q·Wᵀ
更新: S = H·P·Hᵀ + V·R·Vᵀ
      K = P·Hᵀ·S⁻¹
      x = x + K·(z - h(x))
      P = P - K·H·P     (非 Joseph 形式，无数值稳健化)
```

**三种/四种测量更新**（均通过 `xbot_positioning_core` 的封装方法调用 `ekf.update(model, z)`）：

1. `updatePosition(x,y,covariance)` — GPS 位置更新，`src/lib/xbot_positioning/src/PositionMeasurementModel.hpp`。测量向量 2 维 `[x_ant, y_ant]`，观测函数 `h(x)` 把机体位置按天线杆臂偏移旋转补偿到天线安装位置：
   ```
   x_ant = x + cos(theta)*offset_x - sin(theta)*offset_y
   y_ant = y + sin(theta)*offset_x + cos(theta)*offset_y
   ```
   `H` 恒为单位阵（该模型标注为"线性化"但实际未在 `updateJacobians` 里重算对 theta 的偏导，`updateJacobians` 直接 `H.setIdentity()`——**存在近似误差**：真实 `∂h/∂theta ≠ 0`（天线杆臂随航向旋转），但代码把它当线性模型处理，等效假设天线偏移引起的耦合可忽略或已被建模者忽略）。测量噪声 `R` 由调用方每次显式传入并覆盖（`xbot_positioning_core.cpp:16-27`：`c.setIdentity(); c *= covariance`），即对角同方差矩阵。首次定位（GPS 有效样本数刚过阈值）用极小协方差 `0.001`（几乎强制把状态搬到 GPS 位置），此后正常融合用协方差 `500.0`（较大，表示这里的 GPS 位置更新被故意"温和"处理，防止单次 GPS 抖动大幅牵扯滤波器；真正的可信度由外部的有效性判断和丢弃机制把关，而不是这里的协方差数值本身反映真实 GPS 精度）。

2. `updateOrientation(theta,covariance)` — 基于直接航向角(如磁罗盘/GPS双天线航向)的观测模型 `OrientationMeasurementModel.hpp`，`H(THETA,THETA)=1`。**该函数在 `xbot_positioning.cpp` 中从未被调用**：即使 GPS 驱动计算出了 `vehicle_heading`（双天线/F9R 融合航向，`AbsolutePose.orientation_valid`），定位节点也不使用它来直接修正航向，只使用下面的“运动矢量”方式。

3. `updateOrientation2(vx,vy,covariance)` — 基于 GPS 速度矢量（Doppler/位置差分给出的东向、北向速度分量，已转换到与地图同向的 x,y）修正航向和角速度，`OrientationMeasurementModel2.hpp`。观测函数：
   ```
   h.vx = vx_state*cos(theta) - sin(theta)*offset_x*vr_state - cos(theta)*offset_y*vr_state
   h.vy = vx_state*sin(theta) + cos(theta)*offset_x*vr_state - sin(theta)*offset_y*vr_state
   ```
   即把机体系速度 `(vx_state, vr_state)` 结合天线杆臂角速度分量，旋转到地图系，与 GPS 观测到的天线运动矢量比较；`H` 对 `theta` 的偏导被显式计算(`updateJacobians`)，对 `vx_state, vr_state` 的偏导直接由常数系数给出（隐含在初始 `H.setIdentity()` 中，`VX,VY` 行分别对 `VX,VR` 列有非零系数，构造函数里 `H.setIdentity()` 只是占位，真正的 `VX/VR` 列系数其实等于 1（构造时被设为单位阵)，代码没有对这两列显式覆盖，即隐含假设 `∂vx_meas/∂vx_state≈1, ∂vy_meas/∂vr_state`未定义——**这是该测量模型的一个已知简化，只精确线性化了对 theta 的偏导，其余用单位阵近似**）。调用处 `onPose()`：仅当 GPS 消息中的运动矢量模长 `sqrt(motion_vector.x²+motion_vector.y²) ≥ min_speed` 才调用，协方差固定传入 `10000.0`（很大，即"低置信度、慢慢拉"式修正）。**这正是回答"是否需要先让机器人行驶才能建立航向"的关键**：EKF 的绝对航向唯一的外部校正来源就是这个基于运动方向的更新，只有车辟低速阈值(`min_speed=0.01m/s`)以上运动时才会触发；静止或不满足 GPS 条件时，航向完全靠陀螺仪角速度积分维持（存在漂移），初始航向在节点启动时被硬编码为 0（`xbot_positioning_core.cpp:81`：`setState(0,0,0,0,0)`），除非运维通过 `xbot_positioning/set_robot_pose` 服务显式指定初始朝向。

4. `updateSpeed(vx,vr,covariance)` — `SpeedMeasurementModel.hpp`，`H(VX,VX)=1, H(VR,VR)=1`，直接用测得的车体速度/角速度去修正状态里的 `vx,vr` 两个分量。`xbot_positioning_core::updateSpeed()` 中设置协方差的代码被整体注释掉（`xbot_positioning_core.cpp:58-63`），因此实际使用的测量噪声 `R` 是模型默认的单位阵，调用方传入的 `covariance` 参数（`onImu` 中传的 `0.01`）**是死代码，不生效**。该更新在 `onImu()` 中紧跟 `predict()` 之后立即调用（`xbot_positioning.cpp:107`），使用与 predict 相同的 `(vx, gyro_rate)`，效果是让状态里的 `vx,vr` 分量迅速跟踪最新的原始控制输入，供 `updateOrientation2`/调试话题使用。

**GPS 质量/野值判定流程**（`onPose()`，`xbot_positioning.cpp:225-307`）：
1. 若 GPS 融合被服务 `set_gps_state` 关闭 (`gps_enabled=false`)，直接丢弃。
2. 若 `AbsolutePose.flags` 不含 `FLAG_GPS_RTK_FIXED`（即非 RTK 固定解），丢弃并限频打印日志——**只信任 RTK Fix，RTK Float/单点解一概不用**。
3. 若 `position_accuracy > max_gps_accuracy`（默认部署 0.2m），丢弃。
4. 若距上次采纳的 GPS 已超过 5 秒（`time_since_last_gps > 5.0`），认为 GPS 中断过，`has_gps` 置 `false`，重置 `valid_gps_samples`/`gps_outlier_count` 计数，仅记录该点为 `last_gps` 供下次比较，不立即使用——即 GPS 恢复后需要重新"预热"。
5. 若与 `last_gps` 位置距离 `< 5.0m`：视为内点(inlier)；`gps_outlier_count` 清零，`valid_gps_samples++`；
   - 若之前 `has_gps==false` 且已累计 `valid_gps_samples>10` 个连续内点：**首次定位**，调用 `updatePosition(x,y,0.001)` 用极小协方差把 EKF 强制搬到 GPS 位置，`has_gps=true`。
   - 若 `has_gps==true`（已建立稳定 GPS 定位）：正常调用 `updatePosition(x,y,500.0)`；若运动矢量足够大再调用 `updateOrientation2(...,10000.0)`。
6. 若距离 `≥5.0m`：视为野值(outlier)，`gps_outlier_count++`；连续超过 10 次野值(约 10 秒，假设 GPS 1Hz)后，**放弃拒绝**，直接把当前野值当作新基准接受（`has_gps=false`，计数清零，逻辑上等同于重新走一遍"首次定位"预热流程）——这是为了避免机器人真的发生了大跳变位移（比如被搬动、传送）时滤波器永远排斥新位置。

综上，GPS 更新采用"离散状态机 + 固定协方差"的简化融合策略，而非按每帧上报的 GPS 精度动态设置 R；GPS 精度信息 (`position_accuracy`) 只用于准入门限判断(第3步)，不直接输入协方差矩阵。

**陀螺仪零偏标定**：节点启动后前 5 秒内(`skip_gyro_calibration=false` 时)，对收到的每一帧 IMU 数据累加 `angular_velocity.z` 求平均作为零偏 `gyro_offset`，此阶段不做预测也不发布（`onImu()` 开头 `if(!has_gyro)` 分支，`xbot_positioning.cpp:78-104`），假设这 5 秒内机器人保持静止。

---

### xbot_driver_gps

**角色**: 面向真实硬件的 u-blox GNSS 接收机驱动（重点支持 u-blox F9R 等新一代芯片），通过串口/TCP 与模块通信，解析 UBX（或 NMEA）协议数据帧，转换为地图局部坐标（UTM 投影后相对给定基准点的东北天坐标），并把 RTCM 差分改正数据、轮速数据转发给 GPS 模块以支持 RTK 与传感器融合(Sensor Fusion)。是 OpenMower V1 硬件平台上 GPS 数据的唯一来源（V2 硬件由 `mower_comms_v2`/低层板直接处理，未使用本包）。

**节点**: `xbot_driver_gps`（可执行文件 `driver_gps_node`，源码 `src/lib/xbot_driver_gps/src/driver_gps_node.cpp`；协议解析在 `src/interfaces/{gps_interface,ublox_gps_interface,nmea_gps_interface}.{h,cpp}`；物理链路抽象在 `src/devices/{gps_device,serial_gps_device,tcp_gps_device}.{h,cpp}`）。

部署方式（仅 V1 硬件，`src/open_mower/launch/include/_comms.launch:13-24`）：
```xml
<node pkg="xbot_driver_gps" type="driver_gps_node" name="gps" ns="ll/services" ...>
    <remap from="/ll/services/rtcm" to="/ll/position/gps/rtcm"/>
    <remap from="/nmea" to="/ll/position/gps/nmea"/>
    <remap from="~xb_pose" to="/ll/position/gps"/>
    <remap from="~wheel_ticks" to="/mower/wheel_ticks"/>
</node>
```
即该节点消费 `/ll/position/gps/rtcm` 上的 RTCM 改正数据（由 `ros_ntrip_client` 发布），产出的绝对位姿发布到 `/ll/position/gps`（正是 `xbot_positioning` 的 `~xb_pose_in`），产出的 NMEA GGA 句子发布到 `/ll/position/gps/nmea`（供 `ros_ntrip_client` 做 VRS 反馈）。

#### 输入

订阅话题：

| 话题(参数名) | 类型 | 来源 | 用途 |
|---|---|---|---|
| `~wheel_ticks`（remap 为 `/mower/wheel_ticks`） | `xbot_msgs/WheelTick` | 底盘轮编码器 | 仅当接口为 UBX 且限频(≥90ms间隔)后，通过 `send_wheel_ticks()` 组装成 UBX-ESF-MEAS(自定义 data_type=8/9) 消息发给 u-blox F9R，用于其内部传感器融合(Sensor Fusion)定位 |
| `rtcm`（全局名，remap 为 `/ll/position/gps/rtcm`） | `rtcm_msgs/Message` | `ros_ntrip_client` | 收到即原样透传给 GPS 模块（`send_rtcm()` → `send_raw()`），为 RTK 提供差分改正数据 |

服务/参数（无 ROS service，纯参数配置，均在私有命名空间 `~`，来自 `_params.launch:59-82` 与硬件相关 `comms_gps_params.yaml`）：

| 参数 | 默认值 | 典型部署值 | 含义 |
|---|---|---|---|
| `protocol` / 兼容旧参数 `ubx_mode` | 无默认(必须提供其一) | `UBX`(`OM_GPS_PROTOCOL`，默认`UBX`) | 选择解析协议：`UBX`→`UbxGpsInterface`，`NMEA`→`NmeaGpsInterface` |
| `device_type` | `"serial"` | `serial`(`OM_GPS_DEVICE_TYPE`) | `serial`/`tcp`/`file`三选一物理链路 |
| `serial_port` | `/dev/ttyACM0` | 如 `/dev/ttyAMA2`(YardForce500) | 串口设备路径 |
| `baudrate` | `38400` | `921600`(硬件默认配置) | 串口波特率 |
| `tcp_host`/`tcp_port` | `""`/`""` | 由 `OM_GPS_TCP_HOSTNAME`/`OM_GPS_TCP_PORT`(默认 `127.0.0.1`/`2102`) | TCP 模式主机/端口 |
| `filename` | `/dev/null` | - | `file` 模式下回放的原始数据文件 |
| `mode` | `"absolute"` | `absolute`(除非 `OM_USE_RELATIVE_POSITION`) | `absolute`：需配合基准点(datum)把 UTM 坐标转换为相对局部坐标；`relative`：直接使用 u-blox 的 NAV-RELPOSNED 相对定位(不需要 datum，但该模式与 F9R 传感器融合互斥，见包 README) |
| `datum_lat`/`datum_long`/`datum_height` | 无默认(absolute 模式下必填) | `OM_DATUM_LAT`/`OM_DATUM_LONG`/`0` | 局部地图原点的 WGS84 经纬度和高度，`GpsInterface::set_datum()` 用 `RobotLocalization::NavsatConversions::LLtoUTM` 转成 UTM 东北坐标 `datum_e_,datum_n_` 作为坐标系原点 |
| `verbose` | `false` | - | 是否输出 VERBOSE 级别日志 |
| `publish_latency` | `true` | - | 是否发布轮速回环延迟调试话题(仅 UBX 接口) |

#### 输出

发布话题：

| 话题(参数名) | 类型 | 内容 | 说明 |
|---|---|---|---|
| `~pose`（`geometry_msgs/PoseWithCovariance`） | 兼容旧系统的简化位姿 | 与 `xb_pose` 中的 `pose` 字段相同内容 | 每次收到有效 NAV-PVT 解算即发布 |
| `~xb_pose`（remap 为 `/ll/position/gps`） | `xbot_msgs/AbsolutePose` | 完整绝对位姿+精度+运动矢量+RTK/DR标志 | 见下方字段映射细节；这是 `xbot_positioning` 的主要 GPS 输入 |
| `/nmea`（remap 为 `/ll/position/gps/nmea`） | `nmea_msgs/Sentence` | 由当前解算位置反算生成的 `$GPGGA` 句子（每 10 秒最多发一次，`generate_nmea()`） | 提供给 NTRIP 客户端做 VRS(虚拟参考站)/`NEAR` 选站等需要 rover 位置反馈的服务 |
| `~imu`（`sensor_msgs/Imu`） | u-blox F9R 内置 IMU 数据(仅角速度+线加速度，无姿态/协方差) | 来自 UBX-ESF-MEAS 中的陀螺/加计测量项拼装成一帧 | 仅 UBX 接口下产生 |
| `~wheel_tick_stamp_esc`/`~wheel_tick_ublox_rx`/`~wheel_tick_round_trip_host`（`std_msgs/UInt32`，仅 `publish_latency=true`） | 轮速指令时间戳/u-blox收到时的时间戳/往返总时延 | 用于诊断轮速数据到 F9R 的传输延迟(通过读回 ESF-RAW/ESF-MEAS 中 data_type=8 项 echo 校验) |

`xbot_msgs/AbsolutePose` 字段映射细节（`convert_gps_result()`，`driver_gps_node.cpp:131-190`）：
- `source = SOURCE_GPS`；`sensor_stamp`/`received_stamp` 取自 UBX `iTOW` 与本地接收时刻。
- `flags`：按 UBX NAV-PVT 的 `carrSoln`(差分解类型) 置 `FLAG_GPS_RTK`/`FLAG_GPS_RTK_FLOAT`/`FLAG_GPS_RTK_FIXED`；按 `fixType∈{DR_ONLY,GNSS_DR_COMBINED}` 置 `FLAG_GPS_DEAD_RECKONING`。
- `pose.pose.position` = `(pos_e, pos_n, pos_u)`，即相对 datum 的 UTM 东/北/高程差(米)。
- `pose.pose.orientation`：用 `vehicle_heading`(若有效)否则 `motion_heading` 构造的绕 z 四元数(同样是"把角度塞进 z 分量"的近似构造)。
- `pose.covariance`：对角阵，位置 x/y/z 三项方差都用 `position_accuracy²` 填充(各向同性近似)，姿态 roll/pitch 固定给一个很大值 `10000.0`(表示不可观测)，yaw 方差用 `headingAcc²`。
- `motion_vector` = `(vel_e, vel_n, vel_u)`，即 GPS 解出的地图系速度矢量——**正是 `xbot_positioning::updateOrientation2` 用来修正航向的数据来源**。
- `vehicle_heading`/`motion_heading`：车辆航向(双天线/DR融合，弧度)与运动方向航向。

无 TF、无服务/action 提供。

#### 主要算法/流程

**物理链路**：`GpsInterface` 用两个 POSIX 线程实现异步收发：`rx_thread()` 持续从 `GpsDevice`（串口经 `serial` 库，或 TCP 套接字）读取字节流填充 `rx_buffer_`（`std::deque<uint8_t>`），每次读取后调用协议特定的 `parse_rx_buffer()`；`tx_thread()` 等待条件变量 `tx_cv_`，把待发送数据（RTCM/轮速指令）一次性写出，若写超时或阻塞会打日志报警（拥塞检测），详见 `gps_interface.cpp:70-207`。支持从文件回放(`rx_thread_file`)用于离线调试。

**UBX 协议解析**（`ublox_gps_interface.cpp`）：
- 帧格式：`0xB5 0x62 | class(1B) | id(1B) | length(2B,LE) | payload | ck_a | ck_b`，Fletcher-8 校验和（`calculate_checksum`累加实现）。`parse_rx_buffer()` 先找到 `0xB5 0x62` 同步头，再按 `length` 字段判断是否已收全整帧（精确按需读取，降低延迟），校验和不对则丢弃并重新找同步头。
- **NAV-PVT** (`class 0x01, id 0x07`)：主解算消息。检查 `flags` 中 `gnssFixOK` 位和 `flags3` 中 `invalidLlh` 位过滤无效解；`fixType` 映射 `NO_FIX/DR_ONLY/FIX_2D/FIX_3D/GNSS_DR_COMBINED`；`flags` 的 `diffSoln`位+`carrSoln`两位判断 RTK 浮点/固定解；`lat/lon`(1e-7度)、`hMSL`(mm) 经 `LLtoUTM` 转 UTM 坐标后减去 datum 得到局部 `pos_e,pos_n,pos_u`；`hAcc`(mm)转米作为位置精度；速度 `velE/velN/velD`(mm/s)转 m/s(注意 `vel_u = -velD`，因为 UBX 的 D 轴向下)；`headVeh`(1e-5度，车辆航向)与 `headMot`(1e-5度，运动航向)各自转弧度，并做了坐标系旋转/取反(`-heading*(π/180)+π/2` 再 wrap 到 `[0,2π)`)以对齐到 ENU/ROS 航向约定；`vehicle_heading_valid` 取自 `flags` 的 headVehValid 位；同时做了主机时间与 GPS `iTOW` 的时间差合理性检查(host时间差与GPS时间差偏离>100ms 报错，用于发现总线拥塞/丢包)。解算完成后立即触发 `state_callback`(即 `gps_state_received` → 发布 `xb_pose`/`pose` 并生成反馈 GGA)。
- **ESF-MEAS** (`class 0x10, id 0x02`)：F9R 原始传感器测量帧，含多条 24bit 有符号数据项，`data_type` 标识来源：`5/13/14`=陀螺 z/y/x(单位 0.001°/s，需除以4096再转弧度，并对 x/z 轴取反以对齐坐标系约定)，`16/17/18`=加速度计 x/y/z(单位 1/1024 g→m/s²需自行换算，代码里直接按 g 数值使用)，`8`=之前发送的轮速指令回显(用于计算 `wheel_tick_round_trip_stamp` 延迟)。凑齐 6 个 IMU 字段(`imu_fields_valid_==0b111111`)后才触发 `imu_callback`，避免半帧数据。
- **轮速下发** `send_wheel_ticks()`：构造 UBX-ESF-MEAS 帧，`data_type=8`(左轮)/`9`(右轮)，24bit 数据域 `bit23`=方向、`bit0-22`=tick 计数(限制 23 位)，`driver_gps_node.cpp` 中先做 90ms 限频（避免打爆 u-blox 输入队列）并将 tick 数除以 10（降低分辨率以适配协议数据宽度）。

**RTCM 转发**：收到 `rtcm_msgs/Message` 即通过 `send_rtcm()`→`send_raw()` 原样写入发送缓冲区，不做任何解析或校验，由 tx 线程异步写出到串口/TCP。

**NMEA 反馈生成**：`generate_nmea()` 用 `GeographicLib::DMS::Encode` 把当前解算出的 `pos_lat/pos_lon` 编码成 NMEA 度分格式，拼装固定字段(定位质量=1/卫星数=8等占位值)组成 `$GPGGA` 句子，每 10 秒最多产生一次，发布到 `/nmea`，供 `ros_ntrip_client` 转发给需要 rover 概略位置的 NTRIP 挂载点(VRS/`NEAR`)。

**NMEA 协议模式**：`NmeaGpsInterface`(`interfaces/nmea_gps_interface.{h,cpp}`)作为备选解析器，解析标准 NMEA 语句(依赖 `nmeaparse` 库)，功能较 UBX 弱(无轮速/IMU 融合)，代码结构与 UBX 接口平行，此处不再展开逐行细节；生产环境固件配置(`protocol: UBX`)固定选用 UBX 路径。

---

### ros_ntrip_client

**角色**: NTRIP (Networked Transport of RTCM via Internet Protocol) 客户端，连接远程差分改正数据服务商(CORS/VRS/自建基站等 NTRIP Caster)，下载 RTCM3 改正数据流并以 ROS 话题形式转发给 GPS 驱动，从而让 u-blox 模块获得 RTK 定位所需的基准站观测数据；同时可选地把 rover 概略位置以 NMEA GGA 语句上行给需要位置相关改正(VRS/`NEAR`选站)的 Caster。

**节点**: `ntrip_client`（可执行文件 `ntrip_client_node`，源码 `src/lib/ros_ntrip_client/src/ntrip_client_node.cpp`；核心通信逻辑在可复用 C++ 库类 `ros_ntrip_client::NtripClient`，实现分散于 `ntrip_client.cpp`(生命周期/工作线程/回调分发)、`ntrip_transport.cpp`(TCP/TLS连接、HTTP请求、响应解析、重连退避)、`rtcm_framer.cpp`(RTCM3成帧与CRC校验)，公共接口见 `include/ros_ntrip_client/ntrip_client.h`）。

部署方式（`src/open_mower/launch/include/_ntrip_client.launch`，由 `_comms.launch` 在 `OM_USE_NTRIP=true`(默认) 且 GPS 未禁用时 include）：
```xml
<node name="ntrip_client" pkg="ros_ntrip_client" type="ntrip_client_node" output="screen" respawn="true" respawn_delay="10">
    <remap from="nmea" to="/ll/position/gps/nmea"/>
    <remap from="rtcm" to="/ll/position/gps/rtcm"/>
</node>
```
即：消费 `xbot_driver_gps` 生成的 GGA(`/ll/position/gps/nmea`)，产出的 RTCM 数据发布到 `/ll/position/gps/rtcm`（正是 `xbot_driver_gps` 订阅的 `rtcm` 话题），二者与 GPS 驱动构成闭环。

#### 输入

订阅话题：

| 话题(参数名) | 类型 | 来源 | 用途 |
|---|---|---|---|
| `nmea`（参数 `gga_topic`，remap 为 `/ll/position/gps/nmea`） | `nmea_msgs/Sentence` | `xbot_driver_gps` | 外部提供的 GGA 语句，缓存为“最新 GGA”，在会话建立后按 `gga_send_interval_sec` 周期或立即上行给 Caster(部分 VRS/`NEAR` 挂载点要求客户端提供概略位置才能选站或生成本地化改正) |

无服务/action 客户端。

参数（默认取自 `src/lib/ros_ntrip_client/config/ntrip_client.yaml`；OpenMower 集成时在 `_params.launch:91-106` 与 `openmower_defaults_v2.yaml:24-33` 通过环境变量覆盖部分连接与重连参数；完整加载逻辑见 `ntrip_client_node_utils.cpp`）：

| 参数 | 默认值 | 含义 |
|---|---|---|
| `host`（必填） | 空 | Caster 主机名/IP，OpenMower 中来自 `OM_NTRIP_HOSTNAME` |
| `port` | `2101` | Caster 端口，`OM_NTRIP_PORT` |
| `mountpoint`（必填） | 空 | 挂载点名称，`OM_NTRIP_ENDPOINT` |
| `username`/`password` | 空/空 | Basic 认证凭据，`OM_NTRIP_USER`/`OM_NTRIP_PASSWORD` |
| `user_agent` | `"NTRIP ros_ntrip_client/0.1"` | HTTP `User-Agent` 头 |
| `ntrip_version` | `"Ntrip/2.0"` | HTTP `Ntrip-Version` 头 |
| `tls_enabled`/`tls_verify_peer` | `false`/`true` | 是否用 TLS(NTRIP over TLS，常见端口2102)，及是否校验对端证书 |
| `tls_server_name`/`tls_ca_cert_file`/`tls_ca_cert_path`/`tls_client_cert_file`/`tls_client_key_file`/`tls_client_key_password` | 均为空 | TLS SNI/CA/客户端证书相关(基于 OpenSSL) |
| `connect_timeout_sec` | `10.0` | TCP 连接超时 |
| `read_timeout_sec` | `10.0` | 单次 socket 读超时 |
| `session_start_timeout_sec` | `15.0` | 等待会话被接受(收到200/ICY响应头)的超时 |
| `rtcm_timeout_sec` | `4.0` | 建流后若超过该时间未收到新 RTCM 数据，视为流中断；OpenMower 部署值 `4.0`(与默认一致，可用 `OM_NTRIP_RTCM_TIMEOUT_SEC` 覆盖) |
| `adaptive_reconnect` | `true`（OpenMower 显式设为 `true`） | 是否启用自适应重连限速(防止触发公共 Caster 的滥用封禁) |
| `adaptive_burst_max_attempts` | `12` | 时间窗口内允许的最大重连尝试次数 |
| `adaptive_burst_window_sec` | `60.0` | 上述窗口长度(秒)；默认策略：60秒内不超过约12次重连 |
| `adaptive_slow_after_sec` | `300.0` | 持续失败超过该时长后转入慢速重连模式 |
| `adaptive_slow_interval_sec` | `300.0` | 慢速模式下的固定重连间隔(每5分钟一次) |
| `reconnect_initial_delay_sec` | `10.0`(库默认为`5.0`，OpenMower显式设为`10.0`) | 服务端失败(认证失败/挂载点无效等)重连的初始退避延迟 |
| `reconnect_max_delay_sec` | `300.0` | 服务端失败重连的退避延迟上限 |
| `reconnect_backoff_multiplier` | `2.0` | 服务端失败重连退避的指数增长倍率 |
| `transport_reconnect_initial_delay_sec`/`_max_delay_sec`/`_backoff_multiplier` | `1.0`/`10.0`/`1.5` | 单独针对"传输层"故障(TCP连接失败、建流后连接掉线)的更轻量退避策略，与服务端故障退避历史互不干扰，用于快速从 Wi-Fi 漫游等网络抖动恢复 |
| `max_attempts` | `0`(不限) | 最大重试次数，0表示无限重试；OpenMower 显式设为 `0` |
| `send_initial_gga` | `false` | 启动会话时是否立即发送一次 GGA |
| `gga_topic` | `"nmea"` | 订阅的 GGA 话题名(节点私参，非 `NtripClientConfig` 字段，由 `node_utils` 解析) |
| `rtcm_frame_id` | 空 | 发布的 `rtcm_msgs/Message` 的 `header.frame_id` |
| `gga_send_interval_sec` | `0.0`(不周期发送) | 周期性上行 GGA 的间隔；VRS/`NEAR` 类挂载点典型取 `1.0~10.0` 秒 |
| `use_fixed_gga_position` | `false` | 是否使用固定经纬度/高程生成 GGA，而非依赖 `nmea` 话题的真实位置(用于本地/测试场景或没有先验位置时的引导) |
| `fixed_latitude_deg`/`fixed_longitude_deg`/`fixed_altitude_m` | `0.0`/`0.0`/`0.0` | 固定 GGA 位置的经纬度/椭球高 |

#### 输出

发布话题：

| 话题 | 类型 | 内容 | 说明 |
|---|---|---|---|
| `rtcm`（remap 为 `/ll/position/gps/rtcm`） | `rtcm_msgs/Message` | 从 Caster 流中提取并校验通过的单条 RTCM3 消息原始字节 | 每解析出一帧合法 RTCM3 帧即发布一次(`enqueueDataCallback`→节点里包装成消息发布，`ntrip_client_node.cpp:115-121`) |
| `ntrip_status`（`std_msgs/String`，latched） | 人类可读的状态文本 | 连接建立、认证失败、超时等所有状态事件 | 状态变化时发布 |
| `ntrip_status_code`（`std_msgs/String`，latched） | 机器可读状态码字符串 | 如 `SESSION_ACCEPTED`/`STREAM_ACTIVE`/`AUTH_FAILED`/`MOUNTPOINT_INVALID`/`RATE_LIMITED`/`RTCM_TIMEOUT`/`BACKOFF`/`TRANSPORT_HEADER_FAILED`等（完整列表见 `include/ros_ntrip_client/ntrip_client.h` 的 `StatusCode` 枚举及 `status_code.cpp`的`toString`） | 与 `ntrip_status` 同步发布 |
| `ntrip_counters`（`std_msgs/String`） | 累计计数器的格式化文本：`bytes_received`/`frames_published`/`crc_failures`/`discarded_bytes`/`buffer_trimmed_bytes` | 运行诊断 | 固定 1Hz 定时器发布(`ntrip_client_node.cpp:82-83`) |

无 TF、无服务/action。

#### 主要算法/流程

**连接与握手（NTRIP/HTTP 协议）**：工作线程 `workerLoop()` 循环执行"连接→握手→收流→(失败)重连"。`connectToCaster()` 建立 TCP 连接（可选 `configureTlsForSocket()` 包一层 OpenSSL TLS，`tls_verify_peer=true` 时校验证书链和主机名）。握手请求由 `sendRequest()` 构造（`ntrip_transport.cpp:475-503`），是标准 NTRIP v2 (基于HTTP/1.1) 请求：
```
GET /<mountpoint> HTTP/1.1
Host: <host>:<port>
User-Agent: <user_agent>
Ntrip-Version: <ntrip_version>
Connection: close
Accept: */*
Authorization: Basic <base64(username:password)>   # 仅当 username 非空
<空行>
```
响应头由 `readResponseHeaders()` 读取直到空行或超过 8KB(判定为异常)，识别 `ICY 200 OK`/`HTTP/1.0 200 OK`/`HTTP/1.1 200 OK` 之一才算会话被接受；否则按响应内容细分错误类型并映射到对应 `StatusCode`：`SOURCETABLE 200 OK`→挂载点无效(返回的是源表而非数据流)、`401`→认证失败、`403`→禁止访问、`404`→找不到挂载点、`429`→被限流、`502/504`→上游网关错误、`503`→服务暂不可用，其余→协议错误（`ntrip_transport.cpp:564-618`）。这一整套错误分类是该客户端相对基础 NTRIP 客户端实现的增强点，便于运维快速定位问题类别。

**RTCM3 数据流解析**（`rtcm_framer.cpp`）：`streamData()`持续读取的字节先送入 `processRtcmBytes()`累积到内部缓冲区(`session_.rtcm_buffer`，上限 10240 字节，超出则从头丢弃并计入 `buffer_trimmed_bytes`/`discarded_bytes`防止内存无限增长)。`extractRtcmFrame()`按 RTCM3 标准格式解帧：
```
byte0            = 0xD3 (preamble)
byte1,byte2[9:0] = message length (10 bit, 大端, 高6位保留)
payload          = length 字节
最后3字节        = CRC-24Q 校验值(大端)
```
用查表法 CRC-24Q(`kRtcmCrcLookup`，多项式对应标准 RTCM/高精度 GNSS 用 CRC-24Q)校验，校验失败则丢弃 1 字节重新找同步头(`crc_failures`计数)，通过则整帧(`preamble+length+payload+crc`)作为一条消息通过 `enqueueDataCallback` 送入回调分发线程，并把 `frames_published`/`last_rtcm_frame_at`等计数更新；首次成功出帧会重置失败重连历史并发出 `StreamRecovered`/`StreamActive` 状态。若建流后超过 `rtcm_timeout_sec`未再收到新帧，判定流超时需要重连。

**重连退避与"文明礼让"策略**：区分两类失败——
- *传输层故障*(`FailureCategory::Transport`，如 TCP 连接失败、TLS 握手失败、已建流后连接掉线)：使用更激进/更快的 `transport_reconnect_*` 参数(默认初始1秒、最大10秒、倍率1.5)独立退避，且**不计入**服务端失败历史，用于快速从网络抖动(如 WiFi 漫游)恢复。
- *服务端/协议故障*(`FailureCategory::Service`，如认证失败、挂载点无效、超时未建流、RTCM 超时)：使用 `reconnect_*` 参数指数退避(默认初始10秒、最大300秒、倍率2.0)，并计入 `adaptive_reconnect` 自适应限速历史：默认在 60 秒滑动窗口内最多尝试约 12 次(`adaptive_burst_max_attempts`/`adaptive_burst_window_sec`)，若持续失败超过 300 秒(`adaptive_slow_after_sec`)则强制降速到每 300 秒(`adaptive_slow_interval_sec`)一次，超出限速部分发出 `StatusCode::Backoff`并额外等待，是为了遵守公共 NTRIP Caster(如 rtk2go.com)的公平使用政策，避免被封禁。`max_attempts=0` 时不限制总重试次数(OpenMower 生产配置即如此，配合自适应限速做"无限重试但不轰炸")。

**线程模型**（README 已详述，源码核对一致）：三类执行上下文——(1) worker 线程：独占 socket/TLS I/O，负责连接、发请求、解析响应头、读流、判定超时、执行重连、在合适时机把最新 GGA 写出；(2) 回调分发线程：从内部队列取出 RTCM 数据/状态事件，在**非 worker 线程**上调用用户回调(`DataCallback`/`StatusCallback`)，因此节点里 `rtcm_pub.publish()`等操作不会阻塞底层收流；(3) 调用者/ROS 线程：只允许调用 `start()`/`stop()`/`updateGgaSentence()`/`getCounters()`，均为线程安全操作，不直接碰 socket。`updateGgaSentence()`只更新缓存的 GGA 字符串并通过一个"唤醒管道"(pipe)通知 worker，真正的发送仍在 worker 线程完成(`sendQueuedGgaIfNeeded()`/`sendCurrentGga()`)。

**GGA 上行**：仅当会话已被 Caster 接受(`session_.uplink_ready`)后才会尝试发送 GGA；来源二选一——若 `use_fixed_gga_position=true`，由 `ntrip_client_node_utils.cpp` 的 `positionToGga()`根据固定经纬度/高程周期性(`gga_send_interval_sec`)重新生成 `$GPGGA`；否则转发从 `nmea` 话题收到的最新真实 GGA。`gga_send_interval_sec=0` 表示不做周期性重发(仅在 `send_initial_gga=true` 或收到新 GGA 时发一次)。这一机制正是 VRS/`NEAR` 类挂载点(如 CentipedeRTK 的 `NEAR`、SAPOS 的 VRS 服务)动态选站/生成本地化改正所必需的。

---

## 三、地图与路径规划

存取割草区域/导航区域/障碍物/充电桩（`map.json`），并把区域轮廓转换成机器人可执行的覆盖路径。这个项目没有传统 SLAM 建图，地图完全靠人工开一圈录制（见 [architecture.md](architecture.md) "地图是哪来的"一节）。

### mower_map

**角色**: 地图持久化与查询服务。保存/加载割草区域（mow）、导航区域（nav，即允许通行但不割草的区域，如通道）、障碍物（obstacle）轮廓以及一个充电桩位姿（docking point），以 `map.json` 文件形式持久化在磁盘上；同时把这些多边形栅格化为 `nav_msgs/OccupancyGrid`，作为 `move_base_flex`/costmap_2d 的静态层（static layer）数据源。

**节点**: `mower_map_service`（可执行文件与节点名相同，包名 `mower_map`），源码 `src/mower_map/src/mower_map_service.cpp`。典型启动方式：`<node pkg="mower_map" type="mower_map_service" name="map_service"/>`（见 `src/open_mower/launch/open_mower.launch:11`、`src/open_mower/launch/sim_mower_logic.launch:19`）。仿真中通过 docker-compose 设置环境变量 `ROS_HOME=/data/ros`（`docker-simulation/docker-compose.yaml:111`），节点以相对路径 `map.json`/`map.bag` 读写文件（`src/mower_map/src/mower_map_service.cpp:61-62`），因此实际落盘位置为 `$ROS_HOME/map.json`，对应仓库中的示例文件 `docker-simulation/data/ros/map.json`。

#### 输入

**订阅的话题**: 无（该节点不订阅任何话题）。

**提供的服务（Service Server）**，均挂在命名空间 `mower_map_service/` 下：

1. `mower_map_service/add_mowing_area`（`AddMowingAreaSrv`，`src/mower_map/srv/AddMowingAreaSrv.srv`）
   - 请求字段：
     - `MapArea area`（`src/mower_map/msg/MapArea.msg`）：
       - `string id` — 区域 ID，为空时服务端自动生成 32 位随机 nanoid（`generateNanoId()`，`mower_map_service.cpp:172-179`）
       - `string name` — 显示名称
       - `bool active` — 是否启用
       - `geometry_msgs/Polygon area` — 区域外轮廓（世界坐标系 `map`下的 x,y，z 未使用）
       - `geometry_msgs/Polygon[] obstacles` — 该区域内部的障碍物多边形列表（随区域一并添加，各自成为独立的 `type=obstacle` 记录）
       - `float64 angle` — 该区域专属的割草方向覆盖值（弧度），NaN 表示使用自动检测角度
       - `int32 outline_count` / `int32 outline_overlap_count` — 区域专属的outline圈数/与填充重叠圈数覆盖值，-1 表示使用全局配置
       - `float64 outline_offset` — 区域专属的外轮廓额外偏移量覆盖值，NaN 表示使用全局配置
     - `bool isNavigationArea` — true 则该区域类型记为 `"nav"`（可通行不割草），false 则记为 `"mow"`（割草区域）
   - 响应：空（仅返回成功/失败布尔值）
   - 处理逻辑（`mower_map_service.cpp:575-586`）：把 `area`（及其携带的 obstacles）分别转换为内部 `MapArea` 结构追加到全局 `map_data.areas`（障碍物不做与已有区域的关系校验，直接原样加入，类型固定为 `"obstacle"`），然后调用 `saveMapToFile()` 落盘并调用 `buildMap()` 重建栅格地图与可视化。该服务只做“追加”，没有更新/合并同名区域的逻辑。

2. `mower_map_service/get_mowing_area`（`GetMowingAreaSrv`，`src/mower_map/srv/GetMowingAreaSrv.srv`）
   - 请求：`uint32 index` — 在“所有 `type=="mow"` 的区域”列表中的下标（`MapData::getMowingAreas()`，`mower_map_service.cpp:100-106`，按 `map_data.areas` 中出现顺序过滤）
   - 响应：`MapArea area`，包含该割草区域自身的 outline/name/angle/outline_count/outline_overlap_count/outline_offset，并且 `obstacles` 字段被重新填充为**地图中所有 `active && type=="obstacle"` 的区域**（不区分该障碍物原本属于哪个区域，是全局障碍物集合，`mower_map_service.cpp:599-602`）
   - 若 `index` 越界返回 false（ROS service call 失败）

3. `mower_map_service/set_docking_point`（`SetDockingPointSrv`，`src/mower_map/srv/SetDockingPointSrv.srv`）
   - 请求：`geometry_msgs/Pose docking_pose`（位置 x,y + 四元数朝向）
   - 响应：空
   - 逻辑：四元数转 yaw 得到 `heading`，**清空**已有 `docking_stations` 列表后插入唯一一条记录（即全局只保留一个充电桩），保存并重建地图（`mower_map_service.cpp:607-628`）

4. `mower_map_service/get_docking_point`（`GetDockingPointSrv`，`src/mower_map/srv/GetDockingPointSrv.srv`）
   - 请求：空
   - 响应：`geometry_msgs/Pose docking_pose`
   - 逻辑：若 `docking_stations` 为空返回 false；否则取 `front()`，heading 转四元数返回（`mower_map_service.cpp:630-647`）

5. `mower_map_service/set_nav_point`（`SetNavPointSrv`，`src/mower_map/srv/SetNavPointSrv.srv`）
   - 请求：`geometry_msgs/Pose nav_pose`
   - 响应：空
   - 逻辑：不写入 `map_data`/不持久化。仅在内存中设置一个"假障碍物"（`fake_obstacle_pose`，一个箭头形八边形多边形，围绕给定 pose 前后左右展开，`mower_map_service.cpp:470-521`），并在 `buildMap()` 的栅格图上把该形状标记为占据。用途是临时阻挡机器人从某个方向靠近某点（例如让机器人绕开自身当前位置去对接坞或反向进入路径起点）。

6. `mower_map_service/clear_nav_point`（`ClearNavPointSrv`，无字段）：关闭 `show_fake_obstacle` 标志并重建地图。

7. `mower_map_service/clear_map`（`ClearMapSrv`，无字段）：清空 `map_data`（areas + docking_stations），保存空地图并重建。

**通过 xbot_mqtt RPC 提供的方法**（非 ROS service，是基于 MQTT 的 RPC，`xbot_mqtt::RpcProvider`，`mower_map_service.cpp:203-219`）：
- `map.replace`：参数为单个 JSON 对象（完整 `MapData` 结构），用于云端/App 一次性整体替换地图数据，校验通过后保存并重建。

**参数**: 该节点本身几乎无 ROS 参数（未见 `ros::NodeHandle("~")` 读取任何 `~param`）；行为完全由服务调用驱动。

#### 输出

**发布的话题**（均为 latched，`n.advertise(..., N, true)`）：

- `mower_map_service/json_map`（`std_msgs/String`，`data` 为 `map_data` 序列化后的 JSON 字符串，与磁盘上 `map.json` 内容相同）——供前端/监控展示整份地图。
- `mower_map_service/map`（`nav_msgs/OccupancyGrid`）——**costmap 静态层数据源**，值域经 `grid_map::GridMapRosConverter::toOccupancyGrid(map, "navigation_area", 0.0, 1.0, msg)` 转换，0 表示自由，1 表示占据（含模糊边界后的中间值）。在 `src/open_mower/params/costmap_common_params.yaml` 中，`static_layer.map_topic: mower_map_service/map`，被 `costmap_2d::StaticLayer` 直接订阅使用。
- `mower_map_service/map_viz`（`visualization_msgs/MarkerArray`）——RViz 可视化：割草区域轮廓画绿色线（`LineMarker`），障碍物轮廓画红色线，充电桩位姿画蓝色箭头（`mower_map_service.cpp:319-383`）。
- `mower_map_service/map_size`（`xbot_msgs/MapSize`）——地图的宽/高（米）及中心点 x/y，供云端热力图等功能使用。

**服务响应格式**：见"输入"一节中各服务的响应字段说明。

**磁盘文件 `map.json` 格式**：JSON 对象，两个顶层数组 `areas` 和 `docking_stations`（由 `NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(MapData, areas, docking_stations)` 生成，`mower_map_service.cpp:165`）：

```json
{
  "areas": [
    {
      "id": "poaPQtkBIlva20ddsFDx3zTDg3aQQGNJ",
      "properties": {
        "name": "Liegefläche",
        "type": "mow"
        // 可选: "active": false, "angle": 1.23,
        //       "outline_count": 4, "outline_overlap_count": 1, "outline_offset": 0.05
      },
      "outline": [
        { "x": 0.6787848470266908, "y": 1.6896718749776483 },
        { "x": 0.7715408210642636, "y": 1.7413816442713141 }
        // ... 逐点闭合多边形 ...
      ]
    },
    {
      "id": "wZEN4hur1vvCMPYyxoQneO4q2u9pTbqu",
      "properties": { "name": "Apfelbaum", "type": "obstacle" },
      "outline": [ { "x": -16.516, "y": 1.789 }, /* ... */ ]
    }
  ],
  "docking_stations": [
    {
      "id": "AiFtinqNajHA6sH6XZLDh0R5s4TW1cim",
      "properties": { "name": "Docking Station" },
      "position": { "x": -3.1873125018117, "y": 1.5199206012938002 },
      "heading": 2.037528526559177
    }
  ]
}
```

字段说明（`to_json`/`from_json`，`mower_map_service.cpp:117-163`）：
- `areas[].properties.type` ∈ `{"mow", "nav", "obstacle", "draft"}`（未识别/缺省视为 `"draft"`，`buildMap()` 会忽略 `draft`）。
- `properties.active` 只有为 `false` 时才写出（默认 `true` 省略以节省空间）；`angle`/`outline_count`/`outline_overlap_count`/`outline_offset` 只有非默认（非 NaN / >=0）时才写出。
- `outline` 是有序点列表（无独立的“最后一点=第一点”闭合要求，由 grid_map::Polygon 使用时自动按多边形处理）。
- 障碍物没有独立的“属于哪个区域”的字段——`obstacles` 只在 `AddMowingAreaSrv`/`GetMowingAreaSrv` 的 REST 层面按“当前活跃障碍物全集”处理，落盘后与 mow/nav 区域是同级的、类型为 `obstacle` 的独立记录。
- 历史遗留格式 `map.bag`（rosbag，topic `mowing_areas`/`navigation_areas`/`docking_point`，消息类型 `mower_map/MapAreaLegacy`）在找不到 `map.json` 但存在 `map.bag` 时会被自动一次性转换为 `map.json`（`convertLegacyMapToJson()`，`mower_map_service.cpp:721-778`）。

#### 主要算法/流程

- **坐标系**：所有区域/障碍物/充电桩坐标均在全局固定坐标系 `map`（`frame_id="map"`，`mower_map_service.cpp:431`）下，单位为米，不做任何 TF 变换（充电桩朝向由四元数与 `map` 系下的 yaw 互转，`tf2::Matrix3x3::getRPY`）。

- **多边形 → 占据栅格（costmap 静态层）流程**，`buildMap()`（`mower_map_service.cpp:394-536`）：
  1. 遍历 `map_data.areas` 中所有 `active` 且类型属于 `{mow, nav, obstacle}` 的区域，取其所有顶点的 x/y 最小最大值，得到地图包围盒；四周各外扩 1 米（保证模糊处理后边界仍完全占据）；若地图为空则退化为以原点为中心的 10×10 m 空地图。
  2. 用 `grid_map::GridMap`（单层 `"navigation_area"`，分辨率 **0.05 m/格**）以包围盒中心为原点建图，`map.clearAll()` 后整张图先填 `1.0`（**全部视为占据**）。
  3. 第一遍：把所有 `active` 且 `type ∈ {mow, nav}` 的多边形区域用 `grid_map::PolygonIterator` 遍历其覆盖的栅格，置为 `0.0`（**自由**）——即割草区域和导航区域内部先被"打通"。
  4. 第二遍：把所有 `active && type=="obstacle"` 的多边形同样用 `PolygonIterator` 置回 `1.0`（**占据**，在自由区域内"挖洞"）。因此障碍物总是覆盖/优先于割草区域，无论 JSON 中出现的先后顺序。
  5. 若 `show_fake_obstacle` 为真（`set_nav_point` 服务触发），额外绘制一个围绕给定位姿的箭头状八边形并置为占据。
  6. 用 OpenCV 把该层转成 `CV_8UC1` 图像，做 `cv::blur(cv_map, cv_map, cv::Size(5, 5))`（5×5 均值模糊），再转换回 GridMap layer——这一步让区域/障碍边界附近产生**代价梯度**（不是硬边界），使 costmap 认为“靠近边界更贵、但不是绝对禁止”，配合 `costmap_2d::InflationLayer` 使路径规划更平滑地远离边界。
  7. 最终通过 `grid_map::GridMapRosConverter::toOccupancyGrid(map, "navigation_area", 0.0, 1.0, msg)` 转成标准 `nav_msgs/OccupancyGrid` 并发布到 `mower_map_service/map`；`costmap_2d::StaticLayer` 订阅该 topic 作为静态代价层的数据来源（见 `src/open_mower/params/costmap_common_params.yaml`）。

- **存储/查询逻辑**：内存中始终维护单一权威数据结构 `MapData map_data { areas: vector<MapArea>, docking_stations: vector<DockingStation> }`（进程内全局变量，无数据库，无并发锁），每次写操作（`add_mowing_area`/`set_docking_point`/`clear_map`/RPC `map.replace`）都是"修改内存 → `saveMapToFile()` 整体覆写 `map.json` → `buildMap()` 全量重建栅格图与可视化"的同步流程，没有增量更新。`get_mowing_area` 是唯一的纯读查询，按“仅 mow 类型的顺序下标”索引，并把当前地图内所有 obstacle 一并附加返回给调用方（供 `slic3r_coverage_planner` 使用）。ID 生成使用 32 字符的 base62 风格随机 nanoid（`generateNanoId()`），保证请求中不带 id 时也能稳定持久化引用。

---

### slic3r_coverage_planner

**角色**: 覆盖路径规划器。输入一块区域的外轮廓多边形（可带若干障碍物“洞”），输出用于割草作业的完整路径集合：区域外轮廓走线（outline passes）、障碍物绕行走线（obstacle avoidance passes）以及区域内部的“弓字形”（boustrophedon）往返填充线（infill）。核心算法直接复用 3D 打印切片器 Slic3r 的 `libslic3r` 中的周界生成（`PerimeterGenerator`）与填充（`Fill`/`FillRectilinear`/`FillConcentric`）代码，把“逐层打印实心区域”的算法挪用到“逐区域割草覆盖”场景。

**节点**: `slic3r_coverage_planner`（包 `slic3r_coverage_planner`），源码 `src/lib/slic3r_coverage_planner/src/coverage_planner.cpp`；辅助文件 `src/lib/slic3r_coverage_planner/src/OutlinePoly.h/.cpp`（当前是空壳类，未承载实际逻辑）。`libslic3r` 本身不在本仓库中维护，而是通过 CMake `ExternalProject`（`src/lib/slic3r_coverage_planner/CMakeLists.txt:234-249`）在构建时拉取源码并编译为 `liblibslic3r.a` 静态库链接进来；本次分析基于构建产物目录 `build/lib/slic3r_coverage_planner/Slic3r/src/Slic3r/xs/src/libslic3r/` 下拉取到的实际源码（`Fill/Fill.cpp`、`Fill/FillRectilinear.cpp`、`Fill/FillConcentric.cpp`、`PerimeterGenerator.hpp` 等）。启动方式：`<node pkg="slic3r_coverage_planner" type="slic3r_coverage_planner" name="slic3r_coverage_planner"/>`（`src/open_mower/launch/open_mower.launch:14`），私有参数 `~visualize_plan`（bool，默认 `true`；如 `src/open_mower/launch/planner.launch` 显式设为 `false`）控制是否额外发布可视化 Marker。

#### 输入

**订阅的话题**: 无。

**提供的服务（Service Server）**: `slic3r_coverage_planner/plan_path`，类型 `PlanPath`（`src/lib/slic3r_coverage_planner/srv/PlanPath.srv`），处理函数 `planPath()`（`coverage_planner.cpp:321`）。

请求字段（`PlanPathRequest`）：

| 字段 | 类型 | 含义 |
|---|---|---|
| `fill_type` | uint8，常量 `FILL_LINEAR=0` / `FILL_CONCENTRIC=1` | 选择填充算法：0=矩形/直线往返填充（`Slic3r::FillRectilinear`），1=同心轮廓偏移填充（`Slic3r::FillConcentric`）。`mower_logic` 目前恒用 `FILL_LINEAR`（`MowingBehavior.cpp:229`）。 |
| `angle` | float64（弧度） | 填充方向角。经 `Fill::_infill_direction()` 内部再加 `π/2` 才是真正扫线方向（见下）。 |
| `distance` | float64（米） | 相邻两条 outline 圈之间、以及 fill 扫线之间的目标间距（约等于割草工具宽度）。 |
| `outer_offset` | float64（米） | 最外圈相对原始 `outline` 的向内收缩偏移（例如安全边距）。 |
| `outline_count` | uint8 | 需要生成的“沿边界走线”圈数（含最外圈）。 |
| `outline_overlap_count` | uint8 | 这些圈中有多少圈需要与内部 fill 区域重叠（越大，填充起始位置越靠内）。 |
| `skip_area_outline` | bool | true 则不生成区域外轮廓走线。 |
| `skip_obstacle_outlines` | bool | true 则不生成障碍物绕行走线。 |
| `skip_fill` | bool | true 则不生成内部弓字形填充线。 |
| `outline` | `geometry_msgs/Polygon` | 待覆盖区域的外轮廓（世界坐标 `map` 系，米，非缩放整数）。 |
| `holes` | `geometry_msgs/Polygon[]` | 障碍物多边形数组（洞）。 |

**调用方（Service Client）**：`mower_logic`（`MowingBehavior::create_mowing_plan()`，`src/mower_logic/src/mower_logic/behaviors/MowingBehavior.cpp:160-269`）。该客户端在 `mower_logic.cpp:742` 创建：`pathClient = n->serviceClient<slic3r_coverage_planner::PlanPath>("slic3r_coverage_planner/plan_path")`。实际填充逻辑（`MowingBehavior.cpp:216-231`）：
- 先通过 `mower_map_service/get_mowing_area` 拿到该区域的 `MapArea`（含其 `area`/`obstacles`/角度与圈数覆盖字段）。
- 割草方向 `angle`：若区域自带 `angle`（非 NaN）直接使用；否则遍历轮廓顶点，找到第一个与起点距离 > 2 m 的点，用 `atan2` 算出方向；再叠加动态配置 `mow_angle_offset`（度，默认 0，`dynamic_reconfigure`，`MowerLogic.cfg`）以及“完成一整张地图后自动递增”的 `mow_angle_increment`（度，默认 0）；若 `mow_angle_offset_is_absolute` 为真则忽略自动检测角度，直接用偏移量作为绝对角度（相对正东方向）。
- `outline_count`/`outline_overlap_count`：区域级覆盖优先，否则用全局 `dynamic_reconfigure` 值（默认分别为 **3** 圈 / **0** 圈，`MowerLogic.cfg`）。
- `outer_offset`：区域覆盖优先，否则全局 `outline_offset`（默认 **0.0** m，范围 [-1,1]）。
- `distance`：恒等于全局 `tool_width`（割草宽度，默认 **0.14** m）。
- `fill_type` 恒为 `FILL_LINEAR`；三个 `skip_*` 均未显式赋值（ROS 消息布尔字段默认 `false`），即默认三部分全部生成。
- `outline`/`holes` 直接来自 `mapSrv.response.area.area` / `.obstacles`。
调用成功后 `currentMowingPaths = pathSrv.response.paths`，随后按顺序（outline → obstacle → fill）依次交给底层路径执行（FTC local planner）。

**参数**: 仅 `~visualize_plan`（bool），控制是否发布 RViz Marker，不影响规划结果本身。

#### 输出

**发布的话题**：`slic3r_coverage_planner/path_marker_array`（`visualization_msgs/MarkerArray`，仅当 `~visualize_plan=true` 时 advertise，`coverage_planner.cpp:672-675`）——每次 `plan_path` 调用后可视化：输入轮廓（球点串，蓝色系首色）、每条输出路径的线带（依次轮换 7 种预定义颜色）与起点箭头/球标记（`createMarkers()`，`coverage_planner.cpp:31-190`）。

**服务响应格式**（`PlanPathResponse`）：`Path[] paths`。`Path`（`src/lib/slic3r_coverage_planner/msg/Path.msg`）：
- `uint8 is_outline` — 1 表示这是一条"走边界"的路径（区域外轮廓或障碍物绕行），0 表示这是内部弓字形填充线。
- `nav_msgs/Path path` — 标准 `PoseStamped` 序列，`header.frame_id="map"`；每个点的朝向（四元数 yaw）由该点指向下一个点的方向角 `atan2(dy,dx)` 计算得到，最后一个点复用倒数第二个点的朝向（`determinePathForOutline()` 与 fill 段落，`coverage_planner.cpp:274-318`, `607-637`）。点间距经过 `equally_spaced_points(scale_(0.1))` 重采样为约 **0.1 m** 等间距。

`res.paths` 中元素的追加顺序（决定了 mower_logic 执行时的默认走线顺序）：
1. 若 `!skip_area_outline`：按“由外到内”的顺序追加区域外轮廓的各圈走线（每圈是一条 `is_outline=1` 的 Path）。
2. 若 `!skip_obstacle_outlines`：按“距离上一条路径终点最近优先”的顺序追加各障碍物的绕行走线（每个障碍物一条 `is_outline=1` 的 Path，若干嵌套圈已被拼成一条连续路径）。
3. 若 `!skip_fill`：追加内部弓字形填充线，每条独立扫线段是一条 `is_outline=0` 的 Path（顺序即 `Fill` 算法产出 `Polylines` 的原始顺序，未做额外的 TSP/最近邻排序）。

#### 主要算法/流程

整体分三段：**(A) 生成同心偏移“走线圈”（outline loops）并从中派生出区域外轮廓路径与障碍物绕行路径**；**(B) 对走线圈收缩后剩下的最内层区域做弓字形/同心填充**；**(C) 后处理**（等距重采样、方向角计算、圈间/障碍物间的路径拼接与排序）。代码位置：`src/lib/slic3r_coverage_planner/src/coverage_planner.cpp:321-661`。

**0. 坐标缩放与洞的裁剪**（`coverage_planner.cpp:323-350`）
- 输入的 `geometry_msgs/Polygon`（浮点米制坐标）通过 `scale_()`（`libslic3r` 定点整数缩放，通常 1 单位=1e-6 m）转成 `Slic3r::Polygon`（整数坐标，Clipper 库要求整数运算避免浮点误差）。
- 外轮廓强制 `make_counter_clockwise()`（CCW），构造成 `Slic3r::ExPolygon expoly(outline_poly)`（带洞多边形容器）。
- 每个障碍物 `hole` 先 `make_clockwise()`（洞必须 CW，与 Clipper/Slic3r 的方向约定一致），然后**先与外轮廓做交集 `intersection(outline_poly, hole_poly)` 再加入 `expoly.holes`**——这是本仓库对上游 Slic3r 逻辑的修正：若障碍物跨出区域边界或只部分重叠，未裁剪的洞可能持有整个路径集合的“极值顶点”，导致 Clipper 的 `ClipperOffset::FixOrientations` 把所有路径（包括外轮廓本身）方向翻转，造成规划器在障碍物内部填充、在区域内部留空的错误结果（注释见 `coverage_planner.cpp:340-344`）。

**A. Outline 圈生成 = 借用 Slic3r 的“周界生成”算法（`PerimeterGenerator` 的核心偏移/嵌套逻辑，内联实现而非直接调用 `PerimeterGenerator::process()`）**（`coverage_planner.cpp:362-481`）：
1. `loop_number = outline_count - 1`（0-indexed，例如 `outline_count=3` → 生成第 0、1、2 三圈）；`inner_loop_number = loop_number - outline_overlap_count`。
2. 从 `last = expoly` 开始迭代 `i = 0..loop_number`：
   - 第 0 圈（最外圈）用 `offset(last, -outer_offset)` 向内收缩 `outer_offset`（对应请求里的 `outer_offset`，即安全边距）；
   - 之后每一圈用 `offset(last, -distance)` 继续向内收缩 `distance`（工具宽度，即相邻走线圈间距 = 割草宽度，圈与圈之间无重叠、刚好首尾相接达到全覆盖）；
   - 每次 `offset()`（Clipper 多边形内缩）结果为空则提前终止（区域已被完全消耗）。
   - `i <= inner_loop_number` 时把该圈结果保存为 `inner`——这是**留给填充算法使用的“最内层可用区域”**：`outline_overlap_count` 越大，`inner_loop_number` 越小，填充区域就从更靠外的一圈开始（即让填充线与外圈走线有重叠，避免中间漏割）。
3. 每一圈的每个连通多边形按方向分类为“轮廓”（`is_contour = polygon->is_counter_clockwise()`，即区域本体，CCW）或“洞”（CW，即障碍物在该圈的投影），封装为 `PerimeterGeneratorLoop{polygon, depth=i}`，分别放入 `contours[i]`/`holes[i]`（`std::vector<PerimeterGeneratorLoops>`，按深度 i 索引）。
4. **嵌套（nesting）**：
   - 洞按深度从浅到深遍历，把深度 d 的洞挂到"深度 > d 且能 `contains()` 该洞任一点"的更深洞下面（`children`），构建洞的层级树（对应障碍物的多圈嵌套关系，比如障碍物本身+其外扩安全圈）。
   - 轮廓按深度从深到浅遍历，把深度 d 的轮廓挂到"深度 < d 且能 `contains()` 该轮廓某点"的更浅轮廓下面，构建区域轮廓的层级树（最外圈是根）。
5. **由树生成走线路径序列**：
   - 区域外轮廓：若 `!skip_area_outline`，调用 `traverse_from_right(contours[0], area_outlines)`——**从最内层子节点开始向外**深度优先遍历（`traverse_from_right`：反向遍历 `children` 再 push 当前节点，`coverage_planner.cpp:203-212`），得到 `area_outlines: vector<Polygons>`，每组 `Polygons` 是“从内到外”排好序的同心圈序列，供后续 `determinePathForOutline()` 拼成一条螺旋状路径（这样机器人先走最内圈再往外走，或反之，取决于后处理如何使用该顺序——见下方"路径拼接"）。
   - 障碍物绕行：若 `!skip_obstacle_outlines`，对每个深度的 `holes[d]` 调用 `traverse_from_left`（正向遍历 `children` 再 push 当前节点，与 `traverse_from_right` 相反的子树优先方向），得到 `obstacle_outlines`；然后把每组内所有多边形的点序 `reverse`（因为洞在 Clipper 约定下是 CW，为了让最终路径方向与“靠近障碍物走”的直觉一致而反转）。

**B. 填充（Boustrophedon / 弓字形）算法 = 直接复用 `Slic3r::Fill`/`FillRectilinear`**（`coverage_planner.cpp:484-521`，算法本体在 `libslic3r` 构建产物 `Fill/FillRectilinear.cpp`）：
1. 对 outline 阶段算出的 `inner`（最内层剩余区域）做 `union_ex()` 合并成 `ExPolygons`。
2. 对每个连通子区域构造 `Slic3r::Surface`（type=`stBottom`），设置 `Fill` 对象参数：`angle=req.angle`、`min_spacing=req.distance`（未缩放的米制间距）、`density=1.0`（**满密度**，即扫线间距就等于 `min_spacing`，不留空隙）、`dont_adjust=true`（不为了对齐边界而微调间距）、`dont_connect=false`（允许在扫线端点沿多边形边界连接相邻扫线，形成连续弓字形而非分离线段）、`link_max_length=0`（不限制连接线长度）、`complete=false`。
3. 根据 `fill_type` 选择 `FillRectilinear`（直线往返扫描）或 `FillConcentric`（同心偏移环）。
4. 调用 `fill->fill_surface(surface)` 得到 `Polylines`，追加到 `fill_lines`。

**FillRectilinear 算法细节**（`Fill/FillRectilinear.cpp:19-449`，方法 `_fill_single_direction`）：
- 先做 `expolygon.rotate(-direction.first)` 把整个多边形**旋转到扫线方向与 Y 轴平行、即可用“竖直线”扫描的坐标系**；扫描方向角 `direction.first` 来自 `Fill::_infill_direction()`（`Fill.cpp:76-108`）：取请求角 `angle`（若非“桥接”场景，`bridge_angle` 恒为默认值不启用；本规划器未设置 `layer_id`，故不触发逐层交替角度逻辑），再整体加 `π/2`——即请求里的 `angle` 语义是"割草前进方向"，内部会转成与之垂直的"扫描线切割方向"。
- `line_spacing = min_spacing / density`（density=1.0 时等于 `min_spacing`，即精确等于 `distance` 请求值，不做额外重叠或稀疏）。
- 用一个 `map<x, map<y, IntersectionPoint>>` 的网格结构，把多边形所有边与等间距竖直线（间隔 `line_spacing`）的交点全部求出，标记每个交点是某条竖直线段的“下端点/上端点”（lower/upper）。
- 若 `!dont_connect`（默认允许连接）：对同一条竖直扫描线的相邻交点序列，把每个交点与“沿多边形边界方向遇到的、x 更大的下一个交点”用多边形边界上的顶点序列相连（`ip.next`），这样就能把原本独立的多条竖直线段沿多边形边界拼接成连续折线——这正是 Boustrophedon（牛耕式/弓字形）路径的产生机制：直线切割 + 边界转弯连接。
- 主循环 `while (!grid.empty())`：每次从最左侧竖直线的最下端点出发，向上/向下配对找到该竖直线段的另一端点，加入当前 `Polyline`；若该端点有 `next` 连接（即与相邻竖直线通过边界相连），继续沿连接线走到下一竖直线段，如此反复直到没有更多连接，输出一条完整折线；然后处理下一条尚未使用的竖直线，直至耗尽整个网格。每条折线在起止点上按 `endpoints_overlap`（本规划器设为 0）做长度微调。
- 最终 `it->rotate(direction.first)` 把所有产出折线旋转回原始坐标系。
- `FillConcentric`（`fill_type=1` 时使用，`Fill/FillConcentric.cpp:10-61`）：不支持旋转，反复对多边形做 `offset2(last, -(distance+min_spacing/2), +min_spacing/2)` 向内收缩生成一层层同心闭合环，直到收缩至空；再用 `union_pt_chained` 把所有环按"最近邻"拼接顺序排好（从外到内，避免细小中心环带来的走位问题），依次通过 `split_at_index` + 最近点搜索拆成开放折线序列。

**障碍物处理（如何在填充中"挖洞"）**：障碍物洞在第 0 步就被裁剪进 `expoly.holes`，因此从 A 步的第 0 圈偏移开始，所有 `offset()`/`offset2()` 调用本身就是对“带洞多边形”的操作——Clipper 会自动保证洞内区域不被填充线覆盖（填充算法看到的 `inner`/`expp` 已经是挖掉障碍物安全边界后的可通行区域）。障碍物本身的边界路径（供机器人绕障碍物走一圈）则完全来自 A 步中 `holes[]` 树的独立产出，与 fill 阶段解耦。

**路径排序/连接（后处理，`coverage_planner.cpp:524-641`）**：
1. `determinePathForOutline()`（`coverage_planner.cpp:214-319`）把一组“由内到外”排序的同心多边形圈（`Polygons`，来自 A 步的 `area_outlines`/`obstacle_outlines` 分组）拼接成一条带朝向的 `nav_msgs/Path`：
   - 每个环先用 `equally_spaced_points(scale_(0.1))` 重采样为约 0.1 m 等距点；
   - 相邻两圈之间需要找一个“过渡点”：在当前圈上找离上一圈终点最近的点（`closest_idx`），并尝试从 `closest_idx` 顺移 3 个点（`smooth_transition_idx`，让过渡更平滑、类似螺旋线而非生硬的径向跳转）；如果这条平滑过渡连线与“下一层外圈”（或整体 outline）发生几何相交（碰撞检测），则放弃平滑、退回到最近点直接过渡；
   - 用 `std::rotate` 把该圈起点旋转到过渡点，保证前后两圈拼接后是连续路径。
   - 每个点的朝向由“指向下一个点”的方向角决定；对障碍物（`isObstacle=true`）方向反转（因为障碍物路径最后会整体 `reverse`）。
2. 区域外轮廓路径：对每组 `area_outlines` 调用一次 `determinePathForOutline`（内→外顺序直接使用），非空则加入 `res.paths`，并记录 `areaLastPoint`（供障碍物路径排序作为起始参考点）。
3. 障碍物路径排序：**不使用 Slic3r 原有“先扫 X 轴再扫 Y 轴”的 3D 打印顺序**（3D 打印的排序方式对割草机效率很差），改为贪心最近邻排序——维护一个 `prev_point`（初始为区域外轮廓终点，若无外轮廓则用第一个障碍物的起点），每次从剩余障碍物中选出“外圈起点离 `prev_point` 最远”的（代码用 `sort` 降序后 `pop_back()`，等价于每轮取距离最近的一个），生成该障碍物路径后更新 `prev_point` 为其终点，循环直至所有障碍物排序完毕（`coverage_planner.cpp:547-576`，注释明确说明这是相对于原版 Slic3r 排序逻辑的割草场景优化）。每条障碍物路径生成后整体 `std::reverse` 点序，使机器人以"从远处驶向障碍物"而非"贴着障碍物开始"的方式接近（更符合避障场景下的进入姿态）。
4. 内部填充线：`fill_lines` 中每条 `Polyline` 去重（`remove_duplicate_points`）、等距重采样（0.1 m）、计算逐点朝向后，直接按 Fill 算法输出的原始顺序追加为独立的 `is_outline=0` 的 `Path`（**未做额外的扫线间 TSP 优化或方向翻转**，即哪条扫线先在 grid 中被弹出就先输出，实际顺序由 `FillRectilinear` 内部 `grid`（`std::map`，按 x 升序）的遍历顺序决定，近似“从一侧到另一侧”的顺序）。
5. 最终 `res.paths` 依次为：区域外轮廓（若干条，通常 1 条，因为多圈已拼成一条路径）→ 排序后的障碍物绕行路径（每个障碍物 1 条）→ 若干条独立填充扫线。`mower_logic` 按此顺序依次执行。

**关键参数与效果总结**：
- `distance`（= `tool_width`，默认 0.14 m）：控制 outline 圈间距与 fill 扫线间距，即割草宽度，决定覆盖是否有重叠/空隙（本规划器按 1:1 精确对齐，无冗余重叠，除非通过 `outline_overlap_count` 主动制造重叠）。
- `outline_count`（默认 3）/`outline_overlap_count`（默认 0）：先走几圈边界，再从第几圈开始做内部填充（重叠圈数越大，填充区域越小、边界覆盖越充分，适合边缘障碍多的场景）。
- `outer_offset`（默认 0.0 m）：仅作用于最外圈，独立于 `distance`，可用于安全收边而不改变内部走线间距。
- `angle`：割草前进方向；内部固定加 `π/2` 转成扫描线方向；`mower_logic` 侧支持自动检测（首条长边方向）、固定偏移、绝对角度三种模式，并支持“每次全图割完后自动递增角度”避免草痕单一方向导致的地面损伤。

---

## 四、决策 / 状态机

`mower_logic` 是整个系统的"大脑"：一个持续运行的状态机，根据传感器安全状态和外部指令在 IDLE / MOWING / DOCKING / UNDOCKING / AREA_RECORDING 等行为之间切换，驱动地图查询、路径规划、导航执行与割草电机开关。

### mower_logic

**角色**：`mower_logic` 是 OpenMower 软件栈的中枢状态机（"大脑"）。它本身不做定位、不做路径规划、不直接驱动电机，而是持续读取安全相关传感器状态（急停、电池、GPS 质量、电机温度、雨量等），根据配置和外部指令在若干"行为"（`Behavior`）之间切换，通过调用 `mower_map_service`（要区域轮廓/充电桩位姿）、`slic3r_coverage_planner`（要覆盖路径）、`move_base_flex`（执行导航 action）、`mower_comms_v2`/底层板卡（开关割草电机、清急停）等服务和 action 来驱动整台机器完成"割草 / 回充 / 脱离充电桩 / 录制地图 / 待命"等任务，并把当前状态、事件通过话题和 MQTT 上报给监控/App 层。

**节点**：包 `mower_logic` 下有两个可执行节点：
- `mower_logic`（源码 `src/mower_logic/src/mower_logic/mower_logic.cpp`）：核心状态机节点，ROS 节点名 `mower_logic`。
- `monitoring`（源码 `src/mower_logic/src/monitoring/monitoring.cpp`）：一个轻量的旁路节点，ROS 节点名 `monitoring`，负责把电池/ESC/GPS 精度等原始传感器数据转成 `xbot_msgs::SensorInfo`/`SensorDataDouble`/`SensorDataString`，发布到 `xbot_monitoring/sensors/<id>/{info,data}`，并把 `mower_logic/current_state` 转发为 `xbot_msgs::RobotState`（`xbot_monitoring/robot_state`），供 App/MQTT 监控展示用。它不参与状态机决策。

两者都在 `src/open_mower/launch/open_mower.launch`（真实硬件，第12行 `mower_logic` node，第35行 `monitoring` node）和 `src/open_mower/launch/sim_mower_logic.launch`（仿真，第20行/第57行）中被启动，`respawn="true" respawn_delay="10"`。运行参数由 `src/open_mower/launch/include/_params.launch` 中一大批 `<param name="mower_logic/...">`（源自 `OM_*` 环境变量，即用户的 `mower_config.sh`）和 `<param name="ll/services/power/...">` 注入，仿真环境额外由 `docker-simulation/data/params/custom_params.yaml` 中的 `mower_logic:` 和 `ll: services: power:` 段覆盖。

#### 输入

##### 订阅的话题

| 话题 | 类型 | 发布方 | 用途 |
| --- | --- | --- | --- |
| `/ll/emergency` | `mower_msgs/Emergency` | `mower_comms_v2`（或仿真 `mower_simulation`） | 急停/锁存急停状态；通过 `StateSubscriber` 包装（`mower_logic.cpp:93`），`checkSafety()` 每 0.5s 检查 `latched_emergency`，触发时把当前行为置为暂停（`PAUSE_EMERGENCY`），必要时清除。 |
| `/ll/mower_status` | `mower_msgs/Status` | `mower_comms_v2` | 割草电机状态：`mow_enabled`、`mower_motor_temperature`、`mower_motor_rpm`、`rain_detected` 等。用于电机热保护、下雨检测、转速起转检测（`wait_for_mower_spinup`）。 |
| `/ll/power` | `mower_msgs/Power` | `mower_comms_v2` | 电源状态：`battery_voltage[_adc]`、`charge_voltage[_adc]`、`charge_current` 等。用于电量百分比计算、是否在充电桩上判定（`charge_voltage>5V`）、低电压回充判定。 |
| `/ll/bms` | `mower_msgs/Bms` | `mower_comms_v2` | 智能电池管理系统（BMS）电压/电流等，作为电压读数的一个候选来源（`utils::GetFirstValid`）。 |
| `/ll/diff_drive/left_esc_status` | `mower_msgs/ESCStatus` | `mower_comms_v2` | 左轮电调状态，`checkSafety()` 检查 `status <= ESC_STATUS_ERROR` 触发紧急模式。 |
| `/ll/diff_drive/right_esc_status` | `mower_msgs/ESCStatus` | `mower_comms_v2` | 右轮电调状态，同上。 |
| `/xbot_positioning/xb_pose` | `xbot_msgs/AbsolutePose` | `xbot_positioning`（EKF 定位） | 机器人在 `map` 系下的融合位姿、`position_accuracy`、`orientation_valid`、`FLAG_SENSOR_FUSION_RECENT_ABSOLUTE_POSE` 标志位。用于 GPS 质量判定（`isGpsGood()`，`mower_logic.cpp:425`）、docking/undocking/mowing 中的目标位姿计算，`AreaRecordingBehavior` 录制轨迹点。 |
| `/joy_vel` | `geometry_msgs/Twist` | `xbot_monitoring`（App 遥控）、`xbot_remote`（网页摇杆）或本地手柄 `joy_teleop` | 手动遥控速度指令。仅当当前行为 `redirect_joystick()==true`（目前只有 `AreaRecordingBehavior`）时被透传到 `/logic_vel`（`joyVelReceived`，`mower_logic.cpp:704`）。若超过 10s 未收到但行为仍要求透传，则主动停车避免下游超时。 |
| `xbot/action` | `std_msgs/String` | App / 前端（通过 `xbot_monitoring`） | 通用"动作"指令字符串，格式 `"<node_prefix>:<state>/<action_id>"`，例如 `mower_logic:mowing/pause`。特殊值 `"mower_logic/reset_emergency"` 直接清急停；其余转发给 `currentBehavior->handle_action()`。 |
| `/joy`（仅 `AreaRecordingBehavior` enter 时订阅） | `sensor_msgs/Joy` | 本地手柄 `joy` 节点 | 录制地图时用手柄按键控制：B=开始/停止录制多边形，X=记录充电桩点，Y+↑=保存为导航区域，Y+↓=保存为割草区域，RB=手动采点，LB+RB=切换自动/手动采点。 |
| `/record_dock`、`/record_polygon`、`/record_mowing`、`/record_navigation`、`/record_auto_point_collecting`、`/record_collect_point`（仅 `AreaRecordingBehavior`） | `std_msgs/Bool` | 外部脚本/工具（例如命令行录制辅助） | 与手柄按键等价的录制控制话题，供非手柄方式（如键盘/脚本）驱动区域录制流程。 |
| `/xbot_positioning/xb_pose`（`AreaRecordingBehavior` 内部第二次订阅） | `xbot_msgs/AbsolutePose` | `xbot_positioning` | 供多边形录制读取当前位姿（`pose_received`）。 |
| `/mower/perimeter`（仅 `PerimeterBase::setupConnections()`，电磁围栏对接模式） | `mower_msgs/Perimeter` | `mower_comms_v2`（若硬件支持电磁围栏信号） | 左右线圈信号，用于沿边界线搜索/对接充电桩（老式电磁围栏方案，`perimeter_signal` 参数非0时才启用）。 |

##### 调用的服务

| 服务名 | 类型 | 何时/为何调用 |
| --- | --- | --- |
| `slic3r_coverage_planner/plan_path` | `slic3r_coverage_planner/PlanPath` | `MowingBehavior::create_mowing_plan()`（`MowingBehavior.cpp:232`）：把区域轮廓/障碍物/割草角度/刀宽等交给覆盖路径规划器，取回一组 `Path`（往返条纹路径）。 |
| `mower_map_service/get_mowing_area` | `mower_map/GetMowingAreaSrv` | `IdleBehavior::execute()` 检查是否已配置地图；`MowingBehavior::create_mowing_plan()` 按 `area_index` 取某块区域的轮廓/障碍物/名字/是否启用/角度覆盖等。 |
| `mower_map_service/clear_map` | `mower_map/ClearMapSrv` | `highLevelCommand()` 收到 `COMMAND_DELETE_MAPS` 时调用，清空 `map.json`。 |
| `mower_map_service/get_docking_point` | `mower_map/GetDockingPointSrv` | `IdleBehavior`、`DockingBehavior::enter()`、`UndockingBehavior::enter()` 获取充电桩位姿（`map` 系）。 |
| `mower_map_service/add_mowing_area` | `mower_map/AddMowingAreaSrv` | `AreaRecordingBehavior` 录制完一个区域后保存（`isNavigationArea` 区分割草区/导航区）。 |
| `mower_map_service/set_docking_point` | `mower_map/SetDockingPointSrv` | `AreaRecordingBehavior::getDockingPosition()` 记录两次位姿算出朝向后保存充电桩位姿。 |
| `mower_map_service/set_nav_point` | `mower_map/SetNavPointSrv` | `MowingBehavior` 在 `add_fake_obstacle=true` 且当前路径段是轮廓（`path.is_outline`）时，在起点位置放一个"假障碍物"帮助路径规划绕过它，避免直接怼上轮廓。 |
| `mower_map_service/clear_nav_point` | `mower_map/ClearNavPointSrv` | 到达轮廓起点后清除上面设置的假障碍物。 |
| `xbot_positioning/set_gps_state` | `xbot_positioning/GPSControlSrv` | `setGPS(enabled)`（`mower_logic.cpp:218`）：控制 EKF 是否使用 GPS 观测。IDLE/DOCKING 阶段关闭 GPS（用相对里程计），MOWING/UNDOCKING/录制阶段打开。 |
| `xbot_positioning/set_robot_pose` | `xbot_positioning/SetPoseSrv` | `setRobotPose()`：机器人确认停靠在充电桩上时，把 EKF 位姿硬性同步为充电桩位姿（消除累积漂移），用于 IDLE 刚检测到已停靠、以及 `UndockingBehavior::enter()` 里。 |
| `xbot/register_actions` | `xbot_msgs/RegisterActionsSrv` | `registerActions(prefix, actions)`：每个行为进入/退出时把自己支持的"动作"列表（如 `start_mowing`、`pause`、`abort_docking`……）及其 `enabled` 状态注册给前端，供 App 动态生成按钮。 |
| `ll/_service/mow_enabled` | `mower_msgs/MowerControlSrv` | `setMowerEnabled()`：开关割草电机，`mow_direction` 位若 `randomize_mow_motor_direction=true` 则按启动秒数奇偶交替（用于双向刀片磨损均衡），否则固定为 1（正转）。 |
| `ll/_service/emergency` | `mower_msgs/EmergencyStopSrv` | `setEmergencyMode(reason)`：设置/清除急停原因（位掩码，见下文 `EmergencyReason`）。 |
| `/move_base_flex/FTCPlanner/planner_get_progress` | `ftc_local_planner/PlannerGetProgress` | `MowingBehavior::getCurrentMowPathIndex()`：轮询当前 `exe_path` action 执行到路径的第几个点，用于进度上报和 30s 一次的检查点持久化。 |
| `/mower_service/perimeter_listen`（仅电磁围栏模式） | `mower_msgs/PerimeterControlSrv` | `PerimeterBase::setupConnections()` / `shutdownConnections()`：开关围栏线圈信号监听（`listenOn` = 1/2 或其他值禁用）。 |

##### 调用的 Action（actionlib SimpleActionClient）

| Action | 类型 | 何时/为何调用 |
| --- | --- | --- |
| `/move_base_flex/move_base` | `mbf_msgs/MoveBaseAction` | 点到点导航：割草时先开到路径段第一个点（`MowingBehavior::execute_mowing_plan()`），对接时先开到充电桩后方的"接近点"（`DockingBehavior::approach_docking_point()`）。`controller` 固定用 `"FTCPlanner"`。 |
| `/move_base_flex/exe_path` | `mbf_msgs/ExePathAction` | 沿指定 `nav_msgs/Path` 精确跟随：割草覆盖路径本体、对接接近段/直线入桩段、脱离充电桩的路径。跟随时 `controller` 为 `"FTCPlanner"`（割草/接近）或 `"DockingFTCPlanner"`（直线对接/脱离，专用参数集，通常更保守）。 |
| `/move_base_flex/recovery` | `mbf_msgs/RecoveryAction` | `MowingBehavior::execute_mowing_plan()` 中路径跟随失败（非用户暂停）且 GPS 良好时，依次尝试 `/move_base_flex/recovery_behaviors` 参数里配置的各个恢复行为，直到某个成功或全部失败（`getConfiguredRecoveryBehaviors()`，`MowingBehavior.cpp:301`）。若 MBF 未配置恢复行为则跳过（no-op）。 |

##### 参数（`cfg/MowerLogic.cfg`，dynamic_reconfigure，ROS 命名空间 `/mower_logic/...`）

| 参数名 | 类型 | 默认值 | 取值范围 | 含义 |
| --- | --- | --- | --- | --- |
| `automatic_mode` | int | 0 | 0–2 | 0=手动（Manual，仅响应 START/HOME 命令），1=半自动（Semiautomatic，完成一次任务后停止），2=全自动（Automatic，充满电/时机合适就自动出发）。`eAutoMode` 枚举定义在 `Behavior.h:29`。 |
| `undock_distance` | double | 2 | 0–100 | 脱离充电桩第一阶段：直线倒退的距离（m）。 |
| `undock_angled_distance` | double | 0 | 0–100 | 脱离充电桩第二阶段：带角度倒退的距离（m）。 |
| `undock_angle` | double | 0 | -90–90 | 第二阶段的转向角度（度）。 |
| `undock_fixed_angle` | bool | True | — | true=固定使用 `undock_angle`；false=在 `[-|undock_angle|, +|undock_angle|]` 内随机取角度（用于避免每次沿同一条路径走出同一条草痕）。 |
| `undock_use_curve` | bool | True | — | true=第二阶段角度随距离线性递增形成弧线；false=一开始就用全角度直线倒退。 |
| `docking_distance` | double | 2 | 0–100 | 对接最后阶段：从充电桩位姿开始沿桩朝向正向行驶（撞入）的距离（m）。 |
| `docking_approach_distance` | double | 1.5 | 0–5 | 对接前先导航到充电桩正后方多远的"接近点"，再从那里沿直线接近。 |
| `docking_retry_count` | int | 4 | 0–50 | 对接失败（接近失败或未检测到充电电压）时的最大重试次数。 |
| `docking_extra_time` | double | 0 | 0–1.0 | 检测到充电电压后继续多等一小段时间（s）确保接触良好，再取消导航目标。 |
| `docking_redock` | bool | False | — | true 时，若已进入 `IDLE(DOCKED)` 但充电电压又消失（接触不良），自动重新走一次脱离+对接（`UndockingBehavior::RETRY_INSTANCE`）。 |
| `docking_waiting_time` | double | 0 | 0–60 | 到达接近点后、开始走接近直线段前的等待时间（s）。 |
| `undocking_waiting_time` | double | 0 | 0–60 | 开始脱离前的等待时间（s）。 |
| `perimeter_signal` | int | 0 | -2–2 | 0=不使用电磁围栏对接；正数=逆时针沿边对接并给出信号编号，负数=顺时针，绝对值为信号编号。 |
| `outline_count` | int | 3 | 0–255 | 覆盖路径规划前先沿轮廓走几圈（描边），再铺内部条纹填充。 |
| `outline_overlap_count` | int | 0 | 0–255 | 内部条纹与描边重叠的圈数。 |
| `outline_offset` | double | 0.0 | -1.0–1.0 | 额外轮廓内缩（正值，更安全留边）/外扩（负值，扩大覆盖）的偏移量（m）。 |
| `mow_angle_offset` | double | 0 | -180–180 | 割草条纹角度的偏移量（度）。 |
| `mow_angle_offset_is_absolute` | bool | False | — | true=偏移是相对正东的绝对角度；false=在自动检测的区域主方向基础上叠加偏移。 |
| `mow_angle_increment` | double | 0 | 0–180 | 每完成一次全图割草任务后，自动给 `mow_angle_offset` 叠加的角度增量（用于让相邻两次割草条纹交叉，减少压痕）。累积值 `currentMowingAngleIncrementSum` 存在检查点里。 |
| `tool_width` | double | 0.14 | 0.1–2 | 割草刀盘/机身有效宽度（m），决定覆盖路径条纹间距。 |
| `enable_mower` | bool | False | — | 割草电机总开关（硬件保险丝级别的开关）；即使行为要求开启，若此项为 false 也不会真正开启（见 `setMowerEnabled()`）。 |
| `manual_pause_mowing` | bool | False | — | 手动暂停标志；`checkSafety()` 检测到为 true 会触发回充（`dockingReason="Manual pause"`）。 |
| `motor_hot_temperature` | double | 70.0 | 20–150 | 割草电机温度超过此值（°C）触发回充。 |
| `motor_cold_temperature` | double | 40.0 | 20–150 | 电机温度低于此值才允许 IDLE 状态判定"可以开始割草"（`mower_ready`）。 |
| `max_position_accuracy` | double | 0.2 | 0.01–1.0 | GPS/定位精度阈值（m），`position_accuracy` 优于此值才算"GPS 良好"。 |
| `gps_wait_time` | double | 10.0 | 0–60 | 脱离充电桩后拿到"良好 GPS"信号后，再额外等待多久（s）让 EKF 滤波收敛才正式放行。 |
| `gps_timeout` | double | 10.0 | 0–60 | 允许在没有良好 GPS 的情况下继续行驶多长时间（s），超时则停车/暂停（依赖行为的 `needs_gps()`）。 |
| `add_fake_obstacle` | bool | False | — | true 时在驶向覆盖路径轮廓起点前，先在该点设置一个临时导航障碍物，帮助规划器绕开、更平滑地接近轮廓。 |
| `ignore_gps_errors` | bool | False | — | **仅用于仿真**：true 时忽略 GPS 精度/新鲜度检查，始终视为良好。 |
| `max_first_point_attempts` | int | 3 | 1–10 | 到达路径段第一个点失败后的重试次数上限，超过后开始"裁边"。 |
| `max_first_point_trim_attempts` | int | 3 | 1–10 | 裁边（跳过路径起始点）的最大次数，超过后彻底放弃该路径段并 `abort()`。 |
| `rain_mode` | int | 0 | 0–3 | 0=忽略下雨；1=检测到下雨立即回充；2=回充直到变干（`IdleBehavior` 的 `rain_delay` 逻辑）；3=暂停自动模式（把 `manual_pause_mowing` 置 true）。 |
| `rain_delay_minutes` | int | 1（注：min=30，范围写法特殊） | 30–1440 | 雨停后延迟多久（分钟）才允许恢复割草。注意 `.cfg` 里默认值 1 小于其声明的最小值 30，属于源码里的既有小瑕疵，实际会被 dynamic_reconfigure 钳制到 30。 |
| `rain_check_seconds` | int | 0（同样，min=20） | 20–300 | 需要连续检测到下雨多长时间（s）才真正判定"在下雨"（防抖），为 0 表示每次收到即触发判定。 |
| `cu_rain_threshold` | int | -1 | — | Stock-CoverUI 雨量传感器的湿/干阈值（数值越大越干），-1 表示不使用（供 `mower_comms`/CoverUI 硬件层用，非 `mower_logic` 自身逻辑直接消费）。 |
| `emergency_lift_period` | int | -1 | — | 多个轮/霍尔传感器同时被触发多长时间（ms）才计为"举升"急停，用于过滤颠簸地面误触发；-1 走底层默认（供固件/`mower_comms` 使用）。 |
| `emergency_tilt_period` | int | -1 | — | 单个轮/霍尔传感器触发多长时间（ms）才计为"倾斜/举升"急停。 |
| `emergency_input_config` | string | "" | — | 逗号分隔的急停输入配置串，每路传感器可配置为 `[!]I忽略｜U未使用｜S立即停｜L举升检测` 模式。 |
| `shutdown_esc_max_pitch` | int | 0 | 0–180 | 允许执行电调关机的最大俯仰角（度），0=禁用该功能（供底层使用）。 |
| `randomize_mow_motor_direction` | bool | False | — | true 时每次开机/启动割草在正反转间交替（`started.sec & 0x1`），用于均衡双向可逆刀片磨损；仅对称/可逆刀片可用。 |
| `mower_spinup_rpm` | int | 2000 | 0–5000 | 开始沿覆盖路径行驶前，要求割草电机先达到的最小转速（RPM）；0=禁用起转检测。 |
| `mower_spinup_timeout` | double | 10.0 | 1–60 | 等待电机达到 `mower_spinup_rpm` 的超时时间（s），超时则发布 `MOW_MOTOR_SPINUP_FAILED` 事件并进入急停（`EmergencyReason::MOWER_RPM_TIMEOUT`）。 |

另有一组**未走 dynamic_reconfigure、直接用 `ros::param` 静态读取**的参数（在 `_params.launch` 中同样以 `mower_logic/xxx` 命名空间下发，但代码里用 `paramNh->param(...)` 读一次）：
- `mower_logic/ignore_charging_current`（bool，默认 false）：monitoring 节点用它决定是否上报"充电电流"传感器（`monitoring.cpp:83`），`mower_logic` 状态机本身不使用它。

`ll/services/power/*` 命名空间下（`cfg/Power.cfg`，`ll::PowerConfig`，节点内通过 `ros::NodeHandle powerNodeHandle("/ll/services/power")` 一次性读取，非动态刷新其自身逻辑但可被 `dynamic_reconfigure::Client` 远端改写，`monitoring` 节点会跟随更新）：

| 参数名 | 默认值 | 含义 |
| --- | --- | --- |
| `battery_critical_voltage` | -1（必填，无默认时回退为 `battery_empty_voltage`） | 电压低于此值立即回充（不等 20s 平均）。 |
| `battery_empty_voltage` | -1（必填） | 20s 平均电压低于此值判定"电量耗尽"回充；也是 IDLE 判定"是否已充满"的下限参考。 |
| `battery_full_voltage` | -1（必填） | IDLE 中 `mower_ready` 要求电压高于此值才允许出发割草；也是电量百分比计算的上限。 |
| `battery_critical_high_voltage` | -1 | 充电上限保护电压（供充电管理，`mower_logic` 状态机自身不直接使用）。 |
| `charge_critical_high_voltage` | -1 | 充电口电压过高保护阈值。 |
| `charge_critical_high_current` | -1 | 充电电流过高保护阈值。 |

仿真环境实测值（`docker-simulation/data/params/custom_params.yaml`）：`battery_full_voltage=29.26`，`battery_empty_voltage=22.4`，`battery_critical_voltage=22.0`（对应 7 串电芯 3.2–4.18 V/芯的电池模型）。

> 备注：`src/mower_logic/cfg/MowerOdometry.cfg`（`ll::MowerOdometry`/`mower_logic::MowerOdometryConfig`）虽由 `mower_logic` 包的 CMake 生成，但在当前源码里没有任何 `.cpp` 文件包含或使用其生成的头（`grep MowerOdometryConfig` 无匹配），是历史遗留（早期里程计融合节点）的死配置，可视为未使用。

#### 输出

##### 发布的话题

| 话题 | 类型 | 用途 | 频率 |
| --- | --- | --- | --- |
| `/logic_vel` | `geometry_msgs/Twist` | `mower_logic` 自身要发出的速度指令（`cmd_vel_pub`，`mower_logic.cpp:737`）：`stopMoving()` 发零速；`AreaRecordingBehavior` 透传的 `/joy_vel`；电磁围栏对接/搜索模式下的巡边速度。经 `twist_mux` 与导航 `nav_vel`、遥控 `override_vel` 仲裁后送到 `/ll/cmd_vel`。 | 事件触发，非固定周期。 |
| `mower_logic/current_state` | `mower_msgs/HighLevelStatus` | 对外广播当前高层状态：`state`/`state_name`/`sub_state_name`、`job_id`/`session_id`、当前区域/路径/路径点索引、`gps_quality_percent`、`battery_percent`、`is_charging`、`emergency`。是 App 和 `monitoring` 节点了解机器人状态的主要入口（latched=true）。 | 1 Hz（`ui_timer`，`mower_logic.cpp:980`）。 |
| `/xbot_monitoring/mqtt_publish` | `xbot_mqtt/MqttPublish` | 把离散"事件"（见下）通过 MQTT 转发给 App/Home Assistant：`publishMowerEvent(type, details)` 统一打包为 JSON（含 `id`/`t`/`type`/机器人当前 x,y、`job_id`、`session_id`），主题固定 `events/json`。 | 事件触发。 |
| `xbot/register_actions`（服务调用而非话题，另列于"调用的服务") | — | — | — |
| `area_recorder/progress_visualization` / `area_recorder/progress_visualization_array` | `visualization_msgs/Marker` / `MarkerArray` | 仅 `AreaRecordingBehavior`：RViz 中实时显示正在录制的多边形折线（蓝色进行中，绿色=已保存轮廓，红色=已保存障碍物）。 | 每新增一个采样点。 |
| `xbot_monitoring/map_overlay` | `xbot_msgs/MapOverlay` | 仅 `AreaRecordingBehavior`：把录制中的多边形以 App 可理解的叠加图层格式实时推送。 | 每新增一个采样点/切换区域时清空。 |

`monitoring` 节点额外发布（不属于 `mower_logic` 主节点，但同包同 launch）：`xbot_monitoring/sensors/<id>/info`（`xbot_msgs/SensorInfo`，latched）与 `.../data`（`SensorDataDouble`/`SensorDataString`，2 Hz 限流）覆盖电池电压、充电电压、充电电流、左右电调温度、割草电机温度/电流/转速、GPS 精度等传感器；以及 `xbot_monitoring/robot_state`（`xbot_msgs/RobotState`，汇总位姿+高层状态）。

##### 提供的服务

| 服务名 | 类型 | 用途 | 合法 `command` 取值 |
| --- | --- | --- | --- |
| `mower_service/high_level_control` | `mower_msgs/HighLevelControlSrv` | 外部（App 按钮、命令行 `rosservice call`）向状态机下达高层指令的唯一入口，`highLevelCommand()`（`mower_logic.cpp:643`）。 | `COMMAND_START=1`（开始割草/继续，转发 `currentBehavior->command_start()`）、`COMMAND_HOME=2`（回充/暂停，`command_home()`）、`COMMAND_S1=3`（次要指令1：IDLE 下进入 `AreaRecordingBehavior`，MOWING 下请求暂停，`command_s1()`）、`COMMAND_S2=4`（次要指令2：MOWING 下跳过当前区域，`command_s2()`）、`COMMAND_RESET_EMERGENCY=254`（清急停）、`COMMAND_DELETE_MAPS=255`（仅 IDLE/AreaRecording 下合法，清空地图并重启当前行为）。 |

节点还监听 `xbot/action`（话题而非服务）承载更细粒度的、按当前行为动态注册/使能的"动作"（见输入表），本质上是 `high_level_control` 的补充/扩展通道，不受限于固定的 6 个枚举值。

#### 状态机 / 主要流程

##### 顶层执行循环与行为切换机制

`main()`（`mower_logic.cpp:990`）里是一个简单但关键的循环：

```
while (ros::ok()) {
  currentBehavior->start(config, shared_state);   // 重置计时器/aborted/paused等，调用 enter()
  Behavior* newBehavior = currentBehavior->execute();  // 阻塞执行，直到需要切状态
  currentBehavior->exit();
  currentBehavior = newBehavior;
}
```
`Behavior` 基类（`src/mower_logic/src/mower_logic/behaviors/Behavior.h`）定义了统一接口：`enter()/execute()/exit()/reset()`、`abort()`（异步请求尽快退出 `execute()`）、`requestPause()/requestContinue()`（`PAUSE_MANUAL`/`PAUSE_EMERGENCY` 两个可独立叠加的暂停原因位）、`needs_gps()`、`mower_enabled()`、`redirect_joystick()`、`command_home/start/s1/s2()`、`handle_action()`。每个具体行为类是**单例**（静态成员 `INSTANCE`），`execute()` 返回值即"下一个行为"的指针（可以是自己）。

会话（session）/任务（job）ID 由 `mower_logic.cpp:996-1011` 在切换行为时统一维护：进入 `UndockingBehavior::INSTANCE`、`PerimeterUndockingBehavior::INSTANCE`，或从空闲直接进入 `MowingBehavior::INSTANCE`（未走脱离流程，说明本来就没停靠）时开启新 `session_id`（若 `job_id` 为空也一并生成）；回到任一 `IdleBehavior` 实例时结束 `session_id`，若 `MowingBehavior::reset()` 标记过 `current_job_finished` 则同时清空 `job_id`。

安全定时器 `checkSafety()`（0.5 Hz，`mower_logic.cpp:438`）与状态执行循环并行运行（独立 `ros::Timer` + `AsyncSpinner`），它不直接切换 `currentBehavior` 指针，而是通过 `requestPause/requestContinue`、`abortExecution()`、`setEmergencyMode()`、`stopBlade()/stopMoving()` 等手段影响当前行为的内部状态，是真正的"看门狗"。

##### `checkSafety()` 安全检查详细逻辑（`mower_logic.cpp:438-636`，0.5 Hz）

依次执行（任一项触发提前 `return` 的会跳过后续检查）：
1. **急停锁存**：`emergency.latched_emergency` 为真时，对当前行为 `requestPause(PAUSE_EMERGENCY)`；若当前行为是 `AreaRecordingBehavior`/`IdleBehavior::INSTANCE`/`IdleBehavior::DOCKED_INSTANCE` 且正在充电，则认为安全，自动 `setEmergencyMode(0)` 清除（因为这些状态下机器人不会移动）。否则 `requestContinue(PAUSE_EMERGENCY)`。
2. **位姿超时**：`now - pose_time > 1.0s` → `stopBlade()+stopMoving()` 并 `return`（不进入急停，只是刹停，因为位姿丢失通常是暂时的 GPS/EKF 抖动）。
3. **状态/电源超时**：`now - status_time > 3s` 或 `now - power_time > 3s` → 直接 `setEmergencyMode(HIGH_LEVEL)` 并 `return`（这些数据理论上不该丢，丢了说明通讯链路故障，情节更严重）。
4. **电调错误**：左右电调 `status <= ESC_STATUS_ERROR` → `setEmergencyMode(HIGH_LEVEL)` 并 `return`。
5. **GPS 质量判定** `isGpsGood()`（`mower_logic.cpp:425`）：`orientation_valid && position_accuracy < max_position_accuracy && FLAG_SENSOR_FUSION_RECENT_ABSOLUTE_POSE` 全部满足，或 `ignore_gps_errors=true`，则刷新 `last_good_gps` 时间戳并计算 `gps_quality_percent = 1 - min(1, position_accuracy/max_position_accuracy)`；否则质量记为 0（若定向都无效则记为 -1，用于 UI 区分"完全没数据"和"精度差"）。
6. **GPS 超时判定**：`now - last_good_gps > gps_timeout` → `gpsTimeout=true`，质量清零，并通过 `publishMowerEvent("GPS", {available:false/true})` 上报可用性变化边沿。若当前行为 `needs_gps()==true`，把 `gpsTimeout` 状态同步给行为（`setGoodGPS`），超时则 `stopBlade()+stopMoving()` 并 `return`（不进急停，等待恢复）。
7. **手柄遥控超时兜底**：若当前行为 `redirect_joystick()==true` 且超过 10s 未收到 `/joy_vel`，主动 `stopMoving()`，避免下游 `mower_comms` 因 cmd_vel 超时自行报警。
8. **割草电机使能**：`setMowerEnabled(mowerAllowed && currentBehavior->mower_enabled())`——`mowerAllowed` 在本函数开头恒置 true，只要以上安全检查未提前 return 就保持 true；最终是否真正开启还要看行为自身逻辑 (`mower_enabled()`) 与全局开关 `enable_mower`。
9. **电量计算**：电压来源优先级 `battery_voltage_adc > bms.voltage > battery_voltage`（`utils::GetFirstValid`），归一化到 `[battery_empty_voltage, battery_full_voltage]` 区间并钳制到 `[0,1]`。
10. **是否需要回充**（`dockingNeeded`，多个原因可任一触发，取第一个匹配的文案）：
    - `manual_pause_mowing == true` → "Manual pause"；
    - 电压低于 `battery_critical_voltage`（瞬时值，立即触发，防止 BMS 欠压保护）→ "Battery voltage critical"；
    - 每 20s 统计一次窗口内最高电压 `max_v_battery_seen`（忽略短时电流尖峰导致的电压跌落），若低于 `battery_empty_voltage` → "Battery average voltage low"；
    - 电机温度 `>= motor_hot_temperature` → "Mow motor over temp"；
    - 下雨检测（见下）且 `rain_mode != 0` → "Rain detected"，若 `rain_mode==3` 额外把 `manual_pause_mowing` 置 true（暂停整个自动模式，而不仅是这一次回充）。
    - 下雨检测逻辑：`rain_detected` 初值为 true，只有连续 `rain_check_seconds` 秒内每次都读到 `rain_detected` 才保持 true（防抖，见 `mower_logic.cpp:606-627`），窗口结束后重置为 true 等待下一轮采样；同时维护 `rain_resume`（雨停后再等 `rain_delay_minutes` 分钟才允许恢复，供 `IdleBehavior` 里 `rain_mode==2` 时判断 `rain_delay`）。
11. 若 `dockingNeeded` 且当前不处于 `DockingBehavior`/`UndockingBehavior::RETRY_INSTANCE`/两种 `IdleBehavior` 之一，则发布 `DOCKING` 事件并调用 `abortExecution()`（`currentBehavior->abort()`），使当前行为尽快退出并在其 `execute()` 里返回 `&DockingBehavior::INSTANCE`。

##### 行为（状态）一览与转移图

| 行为类 | `state_name()` | `HighLevelStatus.state` | `needs_gps()` | `mower_enabled()` | `redirect_joystick()` |
| --- | --- | --- | --- | --- | --- |
| `IdleBehavior::INSTANCE`（未停靠） | `IDLE` | `HIGH_LEVEL_STATE_IDLE` | false | false | false |
| `IdleBehavior::DOCKED_INSTANCE`（已停靠） | `IDLE` | `HIGH_LEVEL_STATE_IDLE` | false | false | false |
| `MowingBehavior::INSTANCE` | `MOWING`（暂停时显示 `PAUSED`） | `HIGH_LEVEL_STATE_AUTONOMOUS` | true | 动态（跟随 `mowerEnabled`） | false |
| `DockingBehavior::INSTANCE` | `DOCKING` | `HIGH_LEVEL_STATE_AUTONOMOUS` | 动态（仅接近段为 true） | false | false |
| `UndockingBehavior::INSTANCE` / `::RETRY_INSTANCE` | `UNDOCKING` | `HIGH_LEVEL_STATE_AUTONOMOUS` | 动态（脱离直线段 false，等待 GPS 后 true） | false | false |
| `AreaRecordingBehavior::INSTANCE` | `AREA_RECORDING` | `HIGH_LEVEL_STATE_RECORDING` | false | 动态（`manual_mowing`） | **true** |
| `PerimeterSearchBehavior`/`PerimeterDockingBehavior`（电磁围栏对接） | `DOCKING` | `HIGH_LEVEL_STATE_AUTONOMOUS` | false | false | false |
| `PerimeterUndockingBehavior`/`PerimeterMoveToGpsBehavior`（电磁围栏脱离） | `UNDOCKING` | `HIGH_LEVEL_STATE_AUTONOMOUS` | false | false | false |

初始行为：`currentBehavior = &IdleBehavior::INSTANCE`（`mower_logic.cpp:110`）。

**主要转移路径**（省略电磁围栏的等价支路）：

```
                    ┌──────────────────────────────────────────────┐
                    │                                                │
     启动 ──────────▶ IdleBehavior(::INSTANCE / ::DOCKED_INSTANCE)   │
                    │  IdleBehavior.cpp:50 execute()                │
                    └───────┬──────────────┬───────────┬────────────┘
       无地图/无充电桩配置 ↓        command_start /       command_s1
        (IdleBehavior.cpp:52-64)  自动模式条件满足↓        (start_area_recorder)
                    │                       │                       │
                    ▼                       ▼                       ▼
          AreaRecordingBehavior      未停靠→MowingBehavior      AreaRecordingBehavior
          (AreaRecordingBehavior     已停靠→UndockingBehavior   (AreaRecordingBehavior.cpp:36
           .cpp:36 execute())         (IdleBehavior.cpp:126-134)  execute())
                    │                       │
        command_home/                       ▼
        exit_recording_mode           UndockingBehavior
        → abort() → IdleBehavior      (UndockingBehavior.cpp:51 execute())
                                           │ 成功且GPS就绪
                                           ▼
                                      MowingBehavior (或 DockingBehavior，若来自 RETRY_INSTANCE)
                                      (MowingBehavior.cpp:64 execute())
                                           │ 无法创建覆盖计划 / 被abort（人工HOME/急停/下雨/低电量…）
                                           ▼
                                      DockingBehavior
                                      (DockingBehavior.cpp:199 execute())
                                           │ 成功对接
                                           ▼
                                      IdleBehavior::DOCKED_INSTANCE
                                           │ 对接失败次数超限 或 approach失败超限
                                           ▼
                                      IdleBehavior::INSTANCE (放弃，回IDLE)
```

各转移的触发条件与代码位置：

- **IDLE → AreaRecordingBehavior**：`mower_map_service/get_mowing_area` 或 `get_docking_point` 调用失败（未配置地图/充电桩），见 `IdleBehavior.cpp:52-64`；或收到 `COMMAND_S1`（`command_s1()` 置 `start_area_recorder=true`），见 `IdleBehavior.cpp:136-138`。
- **IDLE → UndockingBehavior::INSTANCE**：`manual_start_mowing`（收到 `COMMAND_START`）或（`automatic_mode==AUTO` 或 半自动任务未完成）且 `mower_ready`（电压>`battery_full_voltage` 且电机温度<`motor_cold_temperature` 且非手动暂停且非雨天延迟）为真，且 `currently_docked`（充电电压>5V）为真，见 `IdleBehavior.cpp:126-129`。若配置了电磁围栏脱离（`PerimeterUndockingBehavior::configured()`）则改走 `PerimeterUndockingBehavior::INSTANCE`。
- **IDLE → MowingBehavior::INSTANCE**：同上条件但 `currently_docked==false`（本来就没停靠，比如被人搬到草坪上开机），直接 `setGPS(true)` 后进入割草，见 `IdleBehavior.cpp:131-133`。
- **IDLE(DOCKED) → UndockingBehavior::RETRY_INSTANCE**：`docking_redock=true` 且已停靠标记 `stay_docked` 但充电电压又跌破 5V（接触不良），见 `IdleBehavior.cpp:145-148`。
- **UndockingBehavior → MowingBehavior / DockingBehavior**：`UndockingBehavior` 构造时绑定了 `nextBehavior`（`INSTANCE` 绑 `MowingBehavior::INSTANCE`；`RETRY_INSTANCE` 绑 `DockingBehavior::INSTANCE`，`UndockingBehavior.cpp:34-35`）。脱离路径（先直线退后再按 `undock_angle` 转弯/弧线退出，随后等待 GPS 精度达标 + 额外 `gps_wait_time` 秒收敛）成功后跳转到 `nextBehavior`；路径失败或等不到 GPS 则回 `IdleBehavior::INSTANCE`（`UndockingBehavior.cpp:122-158`）。
- **UndockingBehavior → IdleBehavior::INSTANCE**：`command_home()` 被调用（`abort()`）；`exe_path` 未成功；等 GPS 时被 abort 或 ROS 关闭。
- **MowingBehavior 主循环**（`MowingBehavior.cpp:64-103`）：若当前无缓存路径 (`currentMowingPaths.empty()`) 就调用 `create_mowing_plan(currentMowingArea)`；若该区域被跳过（`active=false`）则前进到下一区域索引继续循环，不产生实际路径；若规划失败（多半是区域索引超出，说明所有区域都割完了）则 `reset()`（清零索引、生成 `JOB_COMPLETE` 事件、累加 `mow_angle_increment`、写检查点）并转 `DockingBehavior::INSTANCE`；否则调用 `execute_mowing_plan()` 执行当前区域所有路径段，执行完（返回 true）则区域索引 +1 继续下一区域；循环体外层 `while` 条件是 `!aborted`，一旦 `aborted`（急停/低电量/人工 HOME/雨天等触发 `abortExecution()`）跳出循环并统一转 `DockingBehavior::INSTANCE`。
- **DockingBehavior → IdleBehavior::DOCKED_INSTANCE**：进入时若已检测到充电电压（例如被人为搬回充电桩）直接判定"已对接"；否则等 GPS 良好 → `approach_docking_point()` 导航到充电桩正后方 `docking_approach_distance` 处再沿直线精确接近 → 关闭 GPS → `dock_straight()` 沿桩朝向正向行驶 `docking_distance`（或电磁围栏模式下改走 `PerimeterSearchBehavior`），过程中一旦充电电压 >5V 就立即取消目标、多等 `docking_extra_time` 秒、判定成功。
- **DockingBehavior → DockingBehavior::INSTANCE（重试）**：接近失败且重试次数 `retryCount <= docking_retry_count`。
- **DockingBehavior → UndockingBehavior::RETRY_INSTANCE**：直线对接失败（超时/规划错误）且重试次数未超限——先退出来再重新对接一次，而不是原地重试。
- **DockingBehavior → IdleBehavior::INSTANCE**：重试次数用尽（放弃对接，留在场地里等待人工介入）或用户 abort。
- **AreaRecordingBehavior → IdleBehavior::INSTANCE**：始终作为循环外层的最终返回值（`AreaRecordingBehavior.cpp:152`），即 `aborted==true`（用户点击"退出录制"或 `command_home()`）时跳出主循环返回 IDLE。

##### 各行为算法细节

**`IdleBehavior`**（`IdleBehavior.cpp`）：25 Hz 轮询循环。每轮先 `stopMoving()+stopBlade()`（保证空闲态绝对不动），然后：
- 计算 `mower_ready`：电压 > `battery_full_voltage` 且电机温度 < `motor_cold_temperature` 且非手动暂停且非雨天延迟。
- 检测"充满电"边沿（`currently_charging && seen_not_charging && mower_ready` 时发 `FULLY_CHARGED` 事件），用 `seen_not_charging` 记录是否观察到过"不在充电"，避免刚启动就误报。
- **停靠位姿同步**：首次检测到 `currently_docked`（充电电压>5V）时，若未配置电磁围栏脱离，调用 `setRobotPose(docking_pose_stamped.pose)` 把 EKF 位姿强制对齐充电桩位姿——这一步提前到"进入 IDLE 后立刻做"而不是等到要脱离时才做，避免长时间停靠期间 EKF 位姿陈旧/无效。
- 满足出发条件见上文转移表。`enter()` 时会 `DockingBehavior::INSTANCE.reset()`（清零对接重试计数，允许下次重新累计）并注册 `start_mowing`/`start_area_recording` 两个动作。

**`MowingBehavior`**（`MowingBehavior.cpp`）：覆盖割草的核心算法。

1. **建立割草计划** `create_mowing_plan(area_index)`（`MowingBehavior.cpp:160`）：
   - 拉取区域轮廓/障碍物；若区域被标记 `!active` 则直接返回 true 但不产生路径（上层据此跳过该区域）。
   - **割草角度**确定：若区域 `angle` 字段非 NaN（人工在地图里显式指定了方向）直接用；否则自动检测——遍历轮廓点，找到第一个与起点距离 > 2m 的点，用该向量的 `atan2` 作为主方向（近似区域最长边方向）。
   - 叠加 `mow_angle_offset + currentMowingAngleIncrementSum`（后者随每次任务完成按 `mow_angle_increment` 累加，取模 360 后规整到 `[-180,180]`）；若 `mow_angle_offset_is_absolute=true` 则最终角度直接等于这个偏移值（相对正东的绝对角），否则叠加到自动检测角度上。
   - 调用 `slic3r_coverage_planner/plan_path`：`outline_count`/`outline_overlap_count`/`outer_offset` 可被该区域的 per-area 字段覆盖（-1/NaN 表示"用全局配置"），`distance=tool_width`，`fill_type=FILL_LINEAR`。
   - **计划摘要与断点续传**：对返回的所有路径点做 SHA-256 摘要（`CryptoPP`），若摘要与上次记录的 `currentMowingPlanDigest` 相同，说明是"重启后恢复同一份计划"，保留 `currentMowingPath`/`currentMowingPathIndex` 从检查点继续；否则视为全新计划，索引清零，记录新摘要。
2. **执行计划** `execute_mowing_plan()`（`MowingBehavior.cpp:354`）：对 `currentMowingPaths` 里每个路径段（`Path`，含 `is_outline` 标记）依次：
   - **暂停处理**：`requested_pause_flag` 非零（急停或人工暂停）时进入等待循环，`mowerEnabled=false`，直到标志清零；随后额外等待 `hasGoodGPS()` 恢复（GPS 中断造成的暂停要等信号恢复才继续，即使人工暂停已解除）。
   - **走到路径段起点**（"FIRST POINT"阶段）：可选先放一个假导航点（`add_fake_obstacle` 且是轮廓段）；用 `move_base` action（`controller="FTCPlanner"`）导航过去；轮询状态，途中响应 `skip_area`/`skip_path`/`aborted`/`requested_pause_flag`；若失败则计数重试，`max_first_point_attempts` 次后开始"裁边"（`currentMowingPathIndex++`，跳过刚才怎么也到不了的第一个点，再给 `max_first_point_attempts` 次机会），`max_first_point_trim_attempts` 次裁边仍失败则彻底 `abort()`。
   - **沿路径行驶**（"MOW"阶段）：到达起点后才真正 `mowerEnabled=true` 并 `wait_for_mower_spinup()`（若 `mower_spinup_rpm>0`，轮询 `/ll/mower_status.mower_motor_rpm` 直到达到目标转速或 `mower_spinup_timeout` 超时——超时会直接 `setEmergencyMode(MOWER_RPM_TIMEOUT)`，因为电机不转还继续开车会拉草不割）。随后用 `exe_path` action（`controller="FTCPlanner"`，`angle_tolerance=5°`，`dist_tolerance=0.2m`）执行从当前索引到段尾的子路径；执行中每收到一次 `FTCPlanner` 进度（`getCurrentMowPathIndex()`）就更新 `currentMowingPathIndex`，并每 30s 落一次检查点（`checkpoint()`，写 `checkpoint.bag`）。
   - **路径跟随失败处理**：若剩余点数 < 5（视为"基本走完"）直接当作该段完成，前进到下一段；否则视为真正失败——非用户请求的暂停时，先尝试 MBF `recovery` action（依次跑 `/move_base_flex/recovery_behaviors` 里配置的每个恢复行为直到成功或用尽），然后统一进入 `paused=true` 等待（GPS 恢复 + 人工/安全定时器放行）并发布 `NAVIGATION_ERROR` 事件。
   - 全部路径段走完（`currentMowingPath >= size()`）返回 true，交由外层前进到下一区域。
3. **检查点持久化**（`checkpoint()`/`restore_checkpoint()`，`MowingBehavior.cpp:815-880`）：用 `rosbag` 把 `mower_logic/CheckPoint`（`job_id`、当前区域/路径/路径点索引、计划摘要、角度增量累计）写到进程工作目录下的 `checkpoint.bag`；节点启动时（`MowingBehavior` 构造函数）尝试恢复，文件不存在/损坏则从头开始。这是"断电重启后能从上次割到哪继续"的机制，但只有摘要匹配（同一份区域配置生成的同一份路径）才会真正续接，否则会从该区域第一条路径重新开始。
4. **人工控制**：`command_home()`（回充：若在半自动任务中先标 `manual_pause_mowing`，再 `abort()`；若正暂停则先 `requestContinue()` 让阻塞的等待循环退出再立即 abort）、`command_start()`（继续：解除暂停，若是半自动任务的人工暂停也一并清除 `manual_pause_mowing`）、`command_s1()`（暂停）、`command_s2()`（跳过当前区域）。

**`DockingBehavior`**（`DockingBehavior.cpp`）：见上文转移表已覆盖主流程；关键细节：
- `approach_docking_point()`：先算出充电桩朝向 `yaw`，目标点 = 充电桩位姿沿 `-yaw` 方向后退 `docking_approach_distance`；用 `move_base` 导航过去，再等待 `docking_waiting_time`，然后按 0.1m 间隔生成从接近点到充电桩位姿正前方的一串路径点（`exe_path`，`angle_tolerance=1°`，`dist_tolerance=0.1m`，`controller="FTCPlanner"`），精确排正对准充电桩。
- `dock_straight()`：从充电桩位姿沿其朝向正向（撞入方向）按 0.1m 间隔生成 `docking_distance` 长的路径，`exe_path`（`controller="DockingFTCPlanner"`——注意换成了对接专用的局部规划器参数集）；边走边轮询充电电压，一旦 >5V 立即 `cancelGoal()` 并成功，即便 action 还在 `ACTIVE`/`PENDING`；若走到路径终点（`SUCCEEDED`）仍未检测到电压才算失败。
- `needs_gps()` 只在 `inApproachMode=true`（进桩前的导航/接近阶段）时为 true；一进入直线对接阶段就 `setGPS(false)`（近距离对接不依赖 GPS，避免绝对位姿噪声干扰最后几十厘米的精确对准）。

**`UndockingBehavior`**（`UndockingBehavior.cpp`）：两级路径：
- 第一级：从当前位姿沿反方向（`-yaw`）直线倒退，共 3 个点（FTC 规划器最少需要 3 点），总长 `undock_distance`。
- 第二级：在第一级终点基础上继续倒退 `undock_angled_distance`，方向在 `yaw` 基础上叠加一个角度：`undock_fixed_angle=true` 用固定的 `undock_angle`；否则用 `rand()`（首次调用时用当前 ROS 时间做随机种子）在 `[-|undock_angle|, |undock_angle|]` 内取值。`undock_use_curve=true` 时该角度随 10 个采样点线性从 0 渐变到目标角（弧线倒车），否则从第一个采样点就用满角度（折线倒车）。整条路径用一次 `exe_path`（`controller="DockingFTCPlanner"`）执行。
- 成功后 `waitForGPS()`：`setGPS(true)`，轮询 `isGpsGood()` 直到达标，再额外等待 `gps_wait_time` 秒让 EKF 收敛，随后把 `gpsRequired` 置 true（此后 `needs_gps()` 才返回 true，交由 `checkSafety()` 的 GPS 超时机制接管）。成功后跳转到构造时绑定的 `nextBehavior`。
- `enter()` 里若检测到当前仍在充电（`charge_voltage>5V`），先 `setRobotPose()` 把位姿钉在充电桩位姿上，确保脱离路径的起点是准确的。

**`AreaRecordingBehavior`**（"没有 SLAM 建图，靠人工开一圈手动录制多边形"）：
- `execute()` 外层循环：每轮先清空可视化叠加图层，内层循环持续响应手柄/话题/App 动作，直到 `finished_all` 或 `error` 或 `aborted`。
- **充电桩位姿记录** `getDockingPosition()`（两阶段）：第一次调用只记录当前 `/xbot_positioning/xb_pose` 位置为 `first_docking_pos`，返回 false（未完成）；第二次调用记录第二个位置，用两点连线的 `atan2` 算出朝向（`yaw`），拼成完整 `Pose` 后返回 true，随即调用 `mower_map_service/set_docking_point` 保存。典型操作：先把机器人开到充电桩里触发一次，再沿离开方向挪动一点触发第二次。
- **多边形录制** `recordNewPolygon()`（10 Hz 采样 `/xbot_positioning/xb_pose`）：第一个点直接采集；之后每当新位置与上一个记录点的距离 `> NEW_POINT_MIN_DISTANCE(0.1m)` 且（`auto_point_collecting=true`，自动模式）或（`collect_point` 标志被手动置位，手动模式）时才追加新点；`poly_recording_enabled` 变 false 时结束——若已有 >2 个点，自动把首点追加到末尾闭合多边形，否则视为失败（点数不足）。第一个录制的多边形自动作为区域**外轮廓**（`has_outline=true`），之后再录的追加为**障碍物**（`obstacles`）。全过程同步发布 RViz `Marker`/`MapOverlay` 供人工核对录制轨迹。
- **保存**：`finished_all=true` 时，若 `has_outline` 且（`is_mowing_area` 或 `is_navigation_area`），调用 `mower_map_service/add_mowing_area`（`isNavigationArea=!is_mowing_area`，`area.active=true`，`angle`/`outline_count`/`outline_overlap_count`/`outline_offset` 全部置为"未指定"哨兵值，交给 `mower_logic` 全局配置或自动检测决定）。丢弃（`finish_discard`）则不调用保存服务，直接清空状态重新开始录制循环。
- **手柄按键映射**（`joy_received`，Xbox 风格手柄索引）：B=开始/停止录制当前多边形；X=记录充电桩点（两次）；Y+↑=完成并保存为导航区域；Y+↓=完成并保存为割草区域；RB=手动采点；LB+RB=切换自动/手动采点模式。同等效果也可通过 `/record_*` 话题或 `xbot/action`（`mower_logic:area_recording/*`）触发，供 App/脚本使用。
- `mower_enabled()` 返回 `manual_mowing`（通过 `start_manual_mowing`/`stop_manual_mowing` 动作切换）——允许一边手动开着割草刀一边录制路线（例如清理杂草的同时顺手记录草坪轮廓），与"是否在录制多边形"完全解耦。
- `redirect_joystick()` 恒返回 true：这是唯一一个手柄/遥控速度指令会被透传到 `/logic_vel`（进而到 `/ll/cmd_vel`）驱动机器人真正移动的行为，因为录制阶段本来就要靠人开着机器人走。

**电磁围栏对接分支（`PerimeterDocking.h/.cpp`，仅 `perimeter_signal != 0` 时启用，老式硬件方案）**：
- `PerimeterSearchBehavior`（对接开始时，若 `PerimeterSearchBehavior::configured()`）：先 `setupConnections()` 订阅 `/mower/perimeter` 并调用 `perimeter_listen` 服务开启监听；等待初始信号、判定电缆极性（假设当前在围栏外，信号应为负）、校准左右线圈灵敏度；然后直行直到检测到进入围栏内（信号变正），再多走 2 秒确认轮子完全压过边界线，转入 `PerimeterDockingBehavior`。
- `PerimeterFollowBehavior`（`PerimeterDockingBehavior`/`PerimeterMoveToGpsBehavior` 共用基类）：核心是三态巡边控制（`FOLLOW`/`TURN_IN`/`TURN_OUT`），用中央线圈信号估计偏离边界线的横向偏移 `y0`、换算成偏转角 `alpha0`，用一个简单的一阶低通（`exp(-dt/5s)` 权重）估计"直行角速度漂移"`drift` 并叠加 `alpha0/2` 作为角速度指令，限幅在 `±ANGULAR_SPEED(0.5rad/s)`；线速度固定 `SEARCH_SPEED(0.1m/s)`。`PerimeterDockingBehavior::arrived()` 用行驶里程 `travelled` 超过 `docking_distance` 判失败（回 IDLE），或连续 2 次检测到充电电压判成功（回 `IdleBehavior::DOCKED_INSTANCE`）。
- `PerimeterUndockingBehavior`：先倒退 0.5m，原地转 180°（转到线圈信号翻转），切换巡边方向，转入 `PerimeterMoveToGpsBehavior`（沿边巡线开出去，行驶里程接近 `undock_distance` 时切回 GPS 并转入 `MowingBehavior`）。

#### 关键参数一览表

| 参数 | 默认值 | 影响 |
| --- | --- | --- |
| `automatic_mode` | 0（手动） | 0=全手动出发，1=半自动（一次任务后停）、2=全自动（满足条件自动出发割草） |
| `tool_width` | 0.14 m | 覆盖路径条纹间距（刀盘/机身有效宽度） |
| `outline_count` / `outline_offset` | 3 / 0.0 m | 割草前先描边几圈；轮廓内缩(+)/外扩(-)安全边距 |
| `mow_angle_offset` / `mow_angle_offset_is_absolute` / `mow_angle_increment` | 0 / False / 0 | 割草条纹方向控制，`increment` 用于逐次任务旋转条纹角度减少压痕 |
| `max_position_accuracy` | 0.2 m | 判定"GPS良好"的精度阈值，直接决定能否行驶 |
| `gps_timeout` | 10 s | 允许无良好GPS继续行驶的最长时间，超时刹停 |
| `gps_wait_time` | 10 s | 脱离充电桩后等待EKF收敛的额外时间 |
| `battery_full_voltage` (`ll/services/power`) | 需配置 | IDLE判定"电量足够可出发割草"的电压门槛 |
| `battery_empty_voltage` (`ll/services/power`) | 需配置 | 20s平均电压低于此值触发回充 |
| `battery_critical_voltage` (`ll/services/power`) | 需配置（缺省=empty） | 瞬时电压低于此值立即回充，防BMS欠压保护 |
| `motor_hot_temperature` / `motor_cold_temperature` | 70℃ / 40℃ | 电机过热触发回充 / 电机需冷却到此温度以下才允许再次出发 |
| `docking_approach_distance` / `docking_distance` | 1.5 m / 2 m | 对接接近点距离 / 最后直线入桩距离 |
| `docking_retry_count` | 4 | 对接失败最大重试次数，超过则放弃回IDLE |
| `docking_redock` | False | 停靠后接触不良（电压丢失）是否自动重新对接 |
| `undock_distance` / `undock_angled_distance` / `undock_angle` | 2 m / 0 m / 0° | 脱离充电桩两阶段路径的距离与转角 |
| `undock_fixed_angle` / `undock_use_curve` | True / True | 脱离转角是否固定（否则随机）；是否走弧线（否则折线） |
| `rain_mode` | 0（忽略） | 1=检测到雨立即回充，2=回充直到变干，3=暂停自动模式 |
| `rain_delay_minutes` / `rain_check_seconds` | 1(钳制到30) / 0 | 雨停后恢复延迟；判定"在下雨"所需的连续检测时长 |
| `mower_spinup_rpm` / `mower_spinup_timeout` | 2000 / 10 s | 起转检测转速门槛与超时（0=禁用），超时进入急停 |
| `randomize_mow_motor_direction` | False | 每次启动割草电机在正反转间随机切换，均衡刀片磨损 |
| `max_first_point_attempts` / `max_first_point_trim_attempts` | 3 / 3 | 到达路径段起点失败的重试与裁边上限，决定何时彻底放弃一条路径 |
| `add_fake_obstacle` | False | 是否在驶向轮廓起点前放置临时假障碍物辅助绕行 |
| `ignore_gps_errors` | False | **仅仿真**：忽略所有GPS质量/超时检查 |
| `perimeter_signal` | 0（不用） | 非0启用老式电磁围栏巡边对接/脱离方案 |

---

## 五、导航执行与速度仲裁

把 `mower_logic` 下发的目标/路径转换成实际的 `cmd_vel` 速度指令，并在导航、状态机、手动遥控等多路速度来源之间做优先级仲裁。

### move_base_flex (mbf_costmap_nav)

**角色**：导航执行框架。取代传统 `move_base`，以 actionlib 形式对外提供 `move_base` / `exe_path` / `get_path` / `recovery` 等动作，内部驱动全局规划器（`GlobalPlanner`）、局部控制器（`ftc_local_planner`）、全局/局部代价地图（costmap_2d）以及恢复行为（`BackwardForwardRecovery`）。它本身是系统安装的 ROS 包（`mbf_costmap_nav`，未随本仓库源码 vendor），仓库中只包含其 launch 与 yaml 配置。

**节点**：`move_base_flex`（`pkg="mbf_costmap_nav" type="mbf_costmap_nav"`），由 `src/open_mower/launch/include/_move_base.launch:3` 启动，`respawn="true" respawn_delay="10"`。同一 launch 文件里还启动了 `move_base_legacy_relay`（`pkg="mbf_costmap_nav" type="move_base_legacy_relay.py"`，见 `_move_base.launch:15-18`），把旧版 `move_base` 风格的话题/参数（`base_local_planner=FTCPlanner`、`base_global_planner=GlobalPlanner`）转发到 mbf 上，用于兼容依赖旧 `move_base` 接口的第三方工具（如 rviz 插件）。

#### 输入

**订阅的话题**
- 机器人里程计/TF（`map -> odom -> base_link`，由定位模块提供，costmap 用其获取机器人位姿）。
- 传感器代价地图数据由 `static_layer` 和 `obstacle_layer` 提供（见下）。

**Action / Service 服务端**（标准 mbf_costmap_nav 接口，供 `mower_logic` 调用）：
- `/move_base_flex/move_base`（`mbf_msgs/MoveBaseAction`）：完整的"从当前位置规划并执行到目标点"流程（get_path + exe_path 串联，含 `recovery_behaviors`）。`mower_logic` 中 `mbfClient`（`mower_logic.cpp:761`）即连接此 action，被 `DockingBehavior.cpp:62` 等用于前往目标点。
- `/move_base_flex/exe_path`（`mbf_msgs/ExePathAction`）：仅执行一条已经算好的路径（不重新做全局规划），`mower_logic.cpp:762` 的 `mbfClientExePath` 用于让 `slic3r_coverage_planner` 生成的覆盖路径、以及对接/脱离桩路径被逐段执行（见 `MowingBehavior.cpp:544`、`DockingBehavior.cpp:71/93/128`、`UndockingBehavior.cpp:62/120`）。目标里的 `controller` 字段用来选择具体控制器插件：割草/常规导航用 `"FTCPlanner"`，进出桩用 `"DockingFTCPlanner"`（同一插件类型的两份不同参数实例，见下文 ftc_local_planner 一节）。
- `/move_base_flex/get_path`（`mbf_msgs/GetPathAction`）：仅调用全局规划器（`GlobalPlanner`）算一条路径而不执行。
- `/move_base_flex/recovery`（`mbf_msgs/RecoveryAction`）：执行指定的恢复行为，`mower_logic.cpp:763` 的 `mbfClientRecovery` 及 `MowingBehavior.cpp:658` 用它触发 `BackwardForwardRecovery`。
- `/move_base_flex/FTCPlanner/planner_get_progress`（`ftc_local_planner/PlannerGetProgress` service）：查询局部规划器当前跟踪到全局路径的第几个点（`mower_logic.cpp:756`）。

**参数（来自 `src/open_mower/params/move_base_flex.yaml`）**：
```yaml
planners:
  - name: GlobalPlanner
    type: global_planner/GlobalPlanner
controllers:
  - name: FTCPlanner
    type: ftc_local_planner/FTCPlanner
  - name: DockingFTCPlanner
    type: ftc_local_planner/FTCPlanner
recovery_behaviors:
  - name: BackwardForwardRecovery
    type: ftc_local_planner/BackwardForwardRecovery
controller_frequency: 1.0      # 局部控制器循环调用频率 (Hz)
controller_patience: 30.0      # 控制器持续失败多久后判定为失败 (s)
planner_frequency: 1.0         # 全局规划器循环调用频率 (Hz)
planner_patience: 5.0          # 全局规划失败容忍时间 (s)
recovery_enabled: true         # 是否允许触发恢复行为
recovery_patience: 7.0
oscillation_timeout: 10.0      # 震荡检测超时 (s)
oscillation_distance: 0.2      # 震荡检测位移阈值 (m)
```
注意：`controller_frequency` 与 `planner_frequency` 都设为 `1.0`（比很多默认配置低很多），意味着 mbf 主循环（get_path/局部控制调用）只有 1 Hz；真正的高频率速度指令由 `ftc_local_planner` 在被 mbf 每次调用 `computeVelocityCommands` 时自行按 `dt` 做积分/前瞻计算并输出（见下节），但 mbf 每秒只调用一次该函数——实际控制平滑度依赖 `ftc_local_planner` 对 `dt` 的自适应处理和下游硬件侧的插值，而不是靠 mbf 的高频重算。

**代价地图配置**（`costmap_common_params.yaml`，同时以 `ns="global_costmap"` 和 `ns="local_costmap"` 加载）：
```yaml
robot_radius: 0.0          # 未设置多边形 footprint，圆形半径 0（即未主动配置足迹尺寸，依赖上游/机器人描述其他方式提供）
global_frame: map
robot_base_frame: base_link
plugins:
  - name: static_layer     costmap_2d::StaticLayer
  - name: obstacle_layer   costmap_2d::ObstacleLayer
  - name: inflation_layer  costmap_2d::InflationLayer
static_layer:
  map_topic: mower_map_service/map   # 静态层地图来自 mower_map 提供的割草区域/禁行区栅格地图服务话题
obstacle_layer:
  max_obstacle_height: 1.0
  obstacle_range: 3.0
  raytrace_range: 4.0
```
全局/局部代价地图各自的差异化参数：
```yaml
# global_costmap_params.yaml
global_costmap:
  update_frequency: 1.0
  publish_frequency: 1.0
  rolling_window: false        # 全局地图为固定坐标系静态窗口，不跟随机器人滚动
  resolution: 0.10              # 10 cm/格
  inflation_layer:
    inflation_radius: 1.0       # 障碍物膨胀半径 1 m

# local_costmap_params.yaml
local_costmap:
  update_frequency: 5.0
  publish_frequency: 2.0
  rolling_window: true          # 局部地图跟随机器人滚动窗口
  width: 1.5
  height: 1.5
  resolution: 0.15              # 15 cm/格，比全局地图更粗
  inflation_layer:
    inflation_radius: 0.0       # 局部层不做额外膨胀（避免局部规划过度保守）
```

#### 输出

- `cmd_vel`（`geometry_msgs/Twist`）被 remap 为 `/nav_vel`（`_move_base.launch:4`）——这是 mbf 内部 `ExePath`/`MoveBase` 执行时，局部控制器（ftc_local_planner）计算出的速度指令最终发布的话题名，随后进入 `twist_mux` 参与仲裁。
- 上述四个 action 的标准 result/feedback。
- 代价地图可视化话题（`/move_base_flex/global_costmap/costmap` 等，标准 costmap_2d 发布）。

#### 主要算法/流程

1. `mower_logic` 通过 `MoveBaseAction` 或 `ExePathAction` 下发目标/路径，并在 goal 中指定 `controller` 字段选择 `FTCPlanner`（常规导航/割草覆盖路径执行）或 `DockingFTCPlanner`（进出桩，参数更保守精细，见下节）。
2. mbf 用 `GlobalPlanner`（system 包，见下节）在 `global_costmap`（10 cm 分辨率、1 Hz 更新、`static_layer`+障碍物层+1 m 膨胀）上算出全局路径。
3. mbf 以 `controller_frequency=1.0` Hz 周期调用所选控制器插件的 `computeVelocityCommands()`，控制器在 `local_costmap`（15 cm 分辨率、5 Hz 更新、1.5×1.5 m 滚动窗口、无额外膨胀）上做避障检测并输出 `cmd_vel`。
4. 若控制器连续失败超过 `controller_patience=30s`、或全局规划失败超过 `planner_patience=5s`、或检测到震荡（`oscillation_timeout=10s` 内位移小于 `oscillation_distance=0.2m`），且 `recovery_enabled=true`，则按 `recovery_behaviors` 列表依次执行恢复行为（本仓库只配置了一个 `BackwardForwardRecovery`），恢复耐心 `recovery_patience=7.0s`。
5. `move_base_legacy_relay` 节点把新版 mbf action 接口适配为旧版 `move_base` 的 topic/param 接口（`base_local_planner`/`base_global_planner` 参数指向同样的 `FTCPlanner`/`GlobalPlanner`），供仍使用旧接口的可视化/监控工具使用；不改变实际控制链路。

---

### ftc_local_planner (FTCPlanner / "Follow The Carrot" 局部规划器)

**角色**：`mbf_costmap_core::CostmapController` 插件，是实际计算 `cmd_vel` 的局部路径跟踪控制器。源码位于 `src/lib/ftc_local_planner`（本仓库自带，非系统包）。同一份代码在 `move_base_flex.yaml` 中被注册为两个不同参数实例：`FTCPlanner`（用于常规导航/覆盖割草）和 `DockingFTCPlanner`（用于精确对接充电桩），两者仅动态参数不同。另附带一个恢复行为插件 `BackwardForwardRecovery`（`nav_core::RecoveryBehavior`）。

**节点**：不是独立节点，作为插件库（`lib/libftc_local_planner.so`，`mbf_plugin.xml`；`lib/libbackward_forward_recovery.so`，`recovery_plugin.xml`）被 `move_base_flex` 进程加载运行。

#### 输入

**插件接口输入**（来自 mbf 框架调用）：
- `setPlan(std::vector<geometry_msgs::PoseStamped> plan)`：mbf 下发的全局路径（`ftc_planner.cpp:71`）。要求路径至少 3 个点，否则直接判定 `FINISHED` 并报警（`ftc_planner.cpp:93-107`）。若路径足够长，会复制最后一个点并让倒数第二个点的朝向沿用倒数第三个点的朝向（`ftc_planner.cpp:96-98`），避免末端朝向跳变影响插值。
- `computeVelocityCommands(pose, velocity, cmd_vel, message)`：mbf 每个控制周期传入机器人当前位姿/速度，要求插件返回 `cmd_vel` 和状态码（`ftc_planner.cpp:155`）。
- `isGoalReached()` / `cancel()`：标准查询/取消接口。
- 机器人在 `map` 到 `base_link` 的 TF（`tf_buffer->lookupTransform("base_link","map",...)`，`ftc_planner.cpp:424`），用于把全局路径上的"控制点"变换到机器人局部坐标系计算误差。
- 局部代价地图（`costmap_2d::Costmap2DROS`），用于障碍检测和机器人实际足迹碰撞检测。

**动态参数（`dynamic_reconfigure`，定义于 `src/lib/ftc_local_planner/cfg/FTCPlanner.cfg`）及本仓库两套实例的实际取值**：

| 参数 | 含义 | cfg 默认值 | `FTCPlanner`（`ftc_local_planner.yaml`） | `DockingFTCPlanner`（`docking_ftc_local_planner.yaml`） |
|---|---|---|---|---|
| `speed_fast` | 直线段允许的最大巡航速度 (m/s) | 0.5 | **0.4** | **0.1** |
| `speed_fast_threshold` | 前方多远仍为"直线"才允许用 `speed_fast` (m) | 1.5 | **1.0** | **5.0** |
| `speed_fast_threshold_angle` | 判定前方路径仍为直线的最大角度偏差 (deg) | 5.0 | **20.0** | **5.0** |
| `speed_slow` | 弯道/接近目标时的慢速 (m/s) | 0.2 | **0.15** | **0.1** |
| `speed_angular` | 控制点沿路径推进时允许的最大角速度 (deg/s，用于控制点推进而非机器人本体) | 20.0 | **45.0** | **40.0** |
| `acceleration` | 巡航速度斜坡加减速率 (m/s²) | 1.0 | **0.1**（很慢的加减速） | **1.0** |
| `lateral_priority_distance` | 横向误差达到此值时角度环增益降到最小 (m，0 关闭) | 0.1 | 0.1 | 0.1 |
| `lateral_priority_min_gain` | 上述增益下限 | 0.1 | 0.1 | 0.1 |
| `kp_lon`/`ki_lon`/`kd_lon` | 纵向（前进方向）速度 PID | 1.0/0/0 | **4.0**/0/0 | **4.0**/0/0 |
| `ki_lon_max` | 纵向积分限幅 | 10.0 | 10.0（未显式覆盖，取默认） | 10.0 |
| `kp_lat`/`ki_lat`/`kd_lat` | 横向误差 PID（作用到角速度） | 1.0/0/0 | **25.0**/**0.06153**/**1.23** | **6.15**/0/0 |
| `ki_lat_max` | 横向积分限幅 | 10.0 | **62.0** | 10.0 |
| `kp_ang`/`ki_ang`/`kd_ang` | 航向角误差 PID | 1.0/0/0 | **3.0**/0/**0.5** | **1.0**/0/0 |
| `ki_ang_max` | 航向积分限幅 | 10.0 | **62.0** | 10.0 |
| `forward_only` | 是否只允许前进 | true | **true** | **false**（对接允许倒车） |
| `restore_defaults` | 恢复出厂参数触发位 | false | false | false |
| `debug_pid` | 是否发布 `debug_pid` 话题 | false | **true** | false |
| `max_cmd_vel_speed` | 输出线速度上限 (m/s) | 2.0 | **1.0** | **0.6** |
| `max_cmd_vel_ang` | 输出角速度上限 (rad/s) | 2.0 | **3.1** | **3.1** |
| `max_goal_distance_error` | 判定"已到达目标位置"的距离阈值 (m) | 1.0 | **0.2** | **0.1**（对接要求更精确） |
| `max_goal_angle_error` | 判定"已到达目标朝向"的角度阈值 (deg) | 10.0 | **20.0** | **20.0** |
| `goal_timeout` | 上述位置/朝向判定的超时 (s)，超时则放弃继续尝试或直接结束 | 5.0 | **20.0** | **20.0** |
| `max_follow_distance` | 机器人偏离路径控制点超过此距离视为"撞了/丢失路径" (m) | 1.0 | **1.0** | **1.0** |
| `oscillation_recovery` | 是否开启末端旋转震荡检测/恢复 | true | **false** | **false** |
| `oscillation_v_eps`/`oscillation_omega_eps`/`oscillation_recovery_min_duration` | 震荡检测速度/角速度死区与持续时间 | 5.0/5.0/5.0 | 未覆盖（取默认，但因 `oscillation_recovery=false`不生效） | 同左 |
| `check_obstacles` | 是否在局部代价地图上检查前方障碍并停止 | true | **false** | **false** |
| `obstacle_lookahead` | 前瞻检查的路径点数 | 5 | 未覆盖（默认 5，因 `check_obstacles=false` 不生效） | 同左 |
| `obstacle_footprint` | 是否检查机器人当前足迹碰撞 | true | 未覆盖（不生效同上） | 同左 |
| `debug_obstacle` | 是否发布障碍调试 marker | true | 未覆盖 | 同左 |

即：本仓库两套配置都**关闭了局部避障停车**（`check_obstacles: false`），说明 FTC 规划器本身不做避障，避障完全依赖上游代价地图规划/静态地图里预先标出的禁行区，以及机械层面（碰撞传感器等）。

**恢复行为 `BackwardForwardRecovery` 参数**（`nav_core::RecoveryBehavior`，通过私有命名空间 `~/BackwardForwardRecovery/*` 的普通 ROS 参数配置，非 dynamic_reconfigure；`backward_forward_recovery.cpp:33-43`，本仓库未在 yaml 中覆盖，均为源码默认值）：
- `max_distance` = 0.5 m：尝试倒车/前进的最大位移。
- `linear_vel` = 0.3 m/s：恢复动作速度。
- `check_frequency` = 10.0 Hz：执行循环频率。
- `max_cost_threshold` = `costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 10`：路径上代价地图格子允许的最大代价（超过则视为前方有障碍，中止该方向尝试）。
- `obstacle_check_distance` = 0.5 m：沿运动方向前方检测碰撞的距离。
- `timeout` = 3.0 s：单方向尝试超时。

#### 输出

- `cmd_vel`（`geometry_msgs::TwistStamped`，通过 mbf `computeVelocityCommands()` 的输出参数返回给 mbf，再由 mbf 发布到（remap 后的）`/nav_vel`）。`BackwardForwardRecovery` 单独直接发布到 `/cmd_vel`（`backward_forward_recovery.cpp:31`，其私有命名空间下的话题，实际由 mbf remap 规则决定最终话题名）。
- `~/<controller_name>/global_point`（`geometry_msgs/PoseStamped`）：当前"carrot"控制点在全局坐标系下的位姿，可视化用（`ftc_planner.cpp:27,419-422`）。
- `~/<controller_name>/global_plan`（`nav_msgs/Path`，latched）：`setPlan()` 收到并处理过的全局路径（`ftc_planner.cpp:28,91-108`）。
- `~/<controller_name>/costmap_marker`（`visualization_msgs/Marker`）：障碍检测调试点云（仅 `debug_obstacle=true` 且 `check_obstacles=true` 时有意义；本仓库两套配置 `check_obstacles=false`，该 marker 实际不会因障碍检测触发）。
- `~/<controller_name>/debug_pid`（`ftc_local_planner/PID`，latched，仅 `FTCPlanner` 实例开启 `debug_pid: true`）：逐周期输出 P/I/D 各分量在纵向/横向/角度三个误差通道上的贡献值、三个误差原始值、最终线速度/角速度。消息字段定义于 `src/lib/ftc_local_planner/msg/PID.msg`：`kp_lon_set/kp_lat_set/kp_ang_set`、`ki_*_set`、`kd_*_set`、`lon_err/lat_err/ang_err`、`ang_speed/lin_speed`。
- `~/<controller_name>/planner_get_progress`（service，`ftc_local_planner/PlannerGetProgress`）：返回 `current_index`——当前跟踪到全局路径数组中的第几个点（`ftc_planner.cpp:593-597`），供 `mower_logic` 判断覆盖路径执行进度。

#### 主要算法/流程

**状态机**（`ftc_planner.h:32-39`，`ftc_planner.cpp:225-300`）：`PRE_ROTATE → FOLLOWING → WAITING_FOR_GOAL_APPROACH → POST_ROTATE → FINISHED`。
- `PRE_ROTATE`：控制点直接固定在路径起点 `global_plan[0]`（`ftc_planner.cpp:308`），机器人原地转向对准该点朝向；`angle_error`（弧度转角度）小于 `max_goal_angle_error` 时进入 `FOLLOWING`；若在 `goal_timeout` 内未对准则判定 `is_crashed=true` 直接 `FINISHED`（失败）。
- `FOLLOWING`：正常路径跟踪（详见下文控制点推进/PID 部分）。若机器人到控制点的局部距离 `local_control_point.translation().norm()` 超过 `max_follow_distance` 判定"丢失路径/撞了"（`is_crashed=true`）。当控制点索引推进到倒数第二个点（`current_index == global_plan.size()-2`）时切到 `WAITING_FOR_GOAL_APPROACH`。
- `WAITING_FOR_GOAL_APPROACH`：控制点保持不动（等于终点前一状态的位置），只等机器人位置误差小于 `max_goal_distance_error`，或等待 `goal_timeout` 超时后强行进入 `POST_ROTATE`。
- `POST_ROTATE`：控制点固定为路径最后一个点的完整位姿，原地转到终点朝向，误差小于 `max_goal_angle_error` 或超时则 `FINISHED`。
- `FINISHED`：输出速度恒为 0。

**"胡萝卜"控制点（control point）推进算法**（`update_control_point()`，`ftc_planner.cpp:302-430`，仅在 `FOLLOWING` 状态推进）：
1. 用 `distanceLookahead()`（`ftc_planner.cpp:124-153`）沿当前 `current_index` 之后的路径点累加直线距离：只要相邻点朝向差（四元数夹角）不超过 `speed_fast_threshold_angle`，就持续累加，一旦累计距离超过 `speed_fast_threshold` 或转角超阈值就停止。这个"前方还有多长一段直路"的量决定巡航速度挡位：距离 ≥ `speed_fast_threshold` 用 `speed_fast`，否则用 `speed_slow`。
2. 目标速度挡位与当前 `current_movement_speed` 之间用 `acceleration` 做斜坡限幅（每周期 `dt*acceleration` 增减，`ftc_planner.cpp:324-337`）实现加减速平滑。
3. 本周期允许控制点沿路径推进的弧长 `distance_to_move = dt * current_movement_speed`，允许的姿态旋转量 `angle_to_move = dt * speed_angular(rad)`。
4. 用一个 while 循环把控制点从 `current_index` 开始，沿相邻路径点间做**线性插值推进**：计算到下一个路径点的剩余距离/剩余角度，如果本周期额度够（`distance_to_move`/`angle_to_move` 都超过剩余量）就把 `current_index++`，从新段继续消耗剩余额度；否则用剩余额度按比例更新 `current_progress`（`0~1`，取距离和角度两者推进比例的较小值 `fmin`，即整个推进过程同时受"最大直线速度"和"最大控制点角速度"两个上限约束，谁先到瓶颈就按谁）。
5. 最终控制点位姿：在 `current_index` 与 `current_index+1` 两点间，位置按 `(1-progress)*p0 + progress*p1` 线性插值，姿态用四元数 `slerp(progress)` 插值（`ftc_planner.cpp:393-406`）。
6. 把控制点（`current_control_point`，`map` 系）通过 TF 变换到 `base_link` 系得到 `local_control_point`（`ftc_planner.cpp:424-425`），读出三个误差量：`lon_error`=局部 x（纵向/前进方向误差）、`lat_error`=局部 y（横向误差）、`angle_error`=局部姿态绕 z 轴分量（航向误差）。

**速度指令计算（三通道独立 PID，互相耦合）**（`calculate_velocity_commands()`，`ftc_planner.cpp:432-591`）：
- 每个误差通道（纵向 lon、横向 lat、角度 ang）分别累计积分误差并按 `ki_*_max` 限幅，微分项为 `(误差-上次误差)/dt`。
- **线速度**（仅 `FOLLOWING` 状态非零）：`lin_speed = lon_error*kp_lon + i_lon_error*ki_lon + d_lon*kd_lon`；若 `forward_only=true` 且算出负值则钳为 0（不允许倒退）；否则按 `max_cmd_vel_speed` 双向限幅；若最终判定为倒车（`lin_speed<0`），额外把 `lat_error` 取反，使倒车时横向纠偏方向随之翻转（保持相对于运动方向的横向修正意义不变）。
- **角速度**：
  - `FOLLOWING` 状态下为**横向误差 + 航向误差组合的耦合 PID**（"follow the carrot" 的核心）：`ang_speed = ang_gain_factor*(angle_error*kp_ang + i_angle_error*ki_ang + d_angle*kd_ang) + lat_error*kp_lat + i_lat_error*ki_lat + d_lat*kd_lat`，即横向偏差本身也直接贡献角速度指令（把机器人"拉回"路径），航向误差项则修正机器人朝向。`ang_gain_factor` 是"横向优先"机制：当 `lateral_priority_distance>0` 时，`ang_gain_factor = (lateral_priority_distance - |lat_error|) / lateral_priority_distance`，横向误差越大，航向 PID 的权重越被压低（但不低于 `lateral_priority_min_gain`），使得机器人横向偏差很大时优先横向纠偏而不是原地大幅调头。最终按 `max_cmd_vel_ang` 限幅。
  - 非 `FOLLOWING`（即 `PRE_ROTATE`/`POST_ROTATE` 原地转向阶段）：角速度只用纯 `angle_error` 的 PID（`kp_ang/ki_ang/kd_ang`），限幅后额外做震荡检测：若 `oscillation_recovery=true` 且 `FailureDetector`（`oscillation_detector.h`，复用 move_base 同款震荡检测算法，基于速度符号在缓冲区内的翻转次数判断）判定正在震荡且持续超过 `oscillation_recovery_min_duration`，则强制把角速度设为 `max_cmd_vel_ang`（固定转向速度冲出震荡）。本仓库两套实例都设 `oscillation_recovery: false`，因此该机制在当前配置下不生效。
  - `debug_pid=true` 时把上述各 P/I/D 分量和误差、最终速度打包发布到 `debug_pid` 话题。

**障碍检测**（`checkCollision()`，`ftc_planner.cpp:599-669`，本仓库两套实例均 `check_obstacles:false`，故实际关闭）：若开启，先检查机器人当前朝向足迹（`getOrientedFootprint`）在局部代价地图上是否有格子代价 ≥ `LETHAL_OBSTACLE`；再沿全局路径未来 `obstacle_lookahead` 个点检查代价地图代价，若代价 >127 且比前一个采样点代价更高（代价递增趋势，暗示正驶向障碍）则判定即将碰撞，`is_crashed=true`，`computeVelocityCommands` 返回 `RET_BLOCKED(109)` 并把速度清零。

**对接精度（Docking）的实现方式**：并非 FTC 规划器内部有专门的"对接模式"分支，而是通过给 `DockingFTCPlanner` 这个独立参数实例配置**更小的速度上限**（`max_cmd_vel_speed=0.6`、`speed_fast=0.1`、`speed_slow=0.1`）、**更严格的到位判据**（`max_goal_distance_error=0.1m`）、**允许倒车**（`forward_only=false`）、**更低比例增益但仍精细的横向 PID**（`kp_lat=6.15`，无积分/微分项，避免对接时震荡）来实现慢速高精度贴桩，控制算法与常规导航完全相同。

**恢复行为 `BackwardForwardRecovery`**（`backward_forward_recovery.cpp`）：被 mbf 判定需要恢复时调用 `runBehavior()`：先尝试沿当前朝向反方向（倒车）移动最多 `max_distance`（每步以 `check_frequency` 频率发送 `linear_vel` 速度，并用 `isPathClear()` 沿运动方向以代价地图分辨率步进采样到 `obstacle_check_distance`，若采样点代价超过 `max_cost_threshold` 则立即停止判定该方向失败），若倒车失败再尝试同样距离的前进；`timeout` 秒内未达到 `max_distance` 也判定该方向失败。两个方向都失败则恢复行为本身失败，交回上层重试/放弃逻辑。

---

### GlobalPlanner

**角色**：`mbf_costmap_nav` 使用的全局路径规划器插件（system 包 `global_planner`，未随本仓库 vendor 源码），在 `global_costmap` 上用 Dijkstra/A* 类算法算出从起点到目标点的全局路径，供 `ftc_local_planner` 跟踪。主要用于点到点导航场景（例如前往/离开充电桩附近的过渡段、`mower_logic` 用 `MoveBaseAction` 直接指定单点目标时），覆盖割草区域内部的"车道式"路径本身是由 `slic3r_coverage_planner` 预先算好整条路径后通过 `ExePathAction` 直接执行，不经过 `GlobalPlanner` 逐点重新规划。

**节点**：无独立节点，作为插件被 `move_base_flex` 加载（`planners` 列表中 `name: GlobalPlanner, type: global_planner/GlobalPlanner`，`move_base_flex.yaml:1-3`）。

#### 输入
- 全局代价地图（`global_costmap`，见 move_base_flex 一节的配置：10cm 分辨率、`static_layer`+`obstacle_layer`+`inflation_layer`，1m 膨胀半径）。
- `mbf_msgs/GetPathAction` 或作为 `MoveBaseAction` 内部子步骤被调用，输入为目标位姿（`geometry_msgs/PoseStamped`）。
- 参数（`src/open_mower/params/global_planner_params.yaml`，仓库中仅覆盖了一项）：
```yaml
GlobalPlanner:
  orientation_mode: 1
```
`orientation_mode=1` 对应上游 `global_planner` 的 `PointPath`（沿路径切线方向为每个路径点赋朝向，而不是全部朝向目标点或全部保持起点朝向），使输出路径每一点的朝向大致贴合路径走向，便于 `ftc_local_planner` 做姿态插值跟踪。其余参数（如 `use_dijkstra`、`use_grid_path`、`old_navfn_behavior`、代价缩放系数等）本仓库未覆盖，使用上游 `global_planner` 包自身默认值。

#### 输出
- `nav_msgs/Path`，作为 `GetPathAction`/`MoveBaseAction` 的结果路径返回给 `move_base_flex`，再交给所选局部控制器（`setPlan()`）跟踪。

#### 主要算法/流程
本仓库未 vendor 其源码，仅通过上表 yaml 配置。行为上是标准 `global_planner` 包（基于代价地图做 Dijkstra 或 A* 全局最短路搜索，可选样条平滑），本仓库只定制了路径点朝向的生成方式（`orientation_mode: 1`）。在 OpenMower 整体流程中，它主要用于"去程/回程"这类点到点导航（如 `DockingBehavior`/`UndockingBehavior`/`MowingBehavior` 中通过 `MoveBaseAction` 发起、目标为单个位姿的场景），实际覆盖区域内的锯齿状割草路径由 `slic3r_coverage_planner` 一次性生成、通过 `ExePathAction` 直接执行，不经过这里的逐点全局搜索。

---

### twist_mux

**角色**：多路 `geometry_msgs/Twist` 速度指令的优先级仲裁器（system 包 `twist_mux`，未 vendor 源码，只在本仓库配置其话题/锁定 yaml）。它是导航执行链路的最终关口：`ftc_local_planner`（经由 `move_base_flex` 的 `/nav_vel`）、`mower_logic`（`/logic_vel`、以及未来可能使用的 `/override_vel`）等多个速度来源都汇入这里，按优先级和超时规则选出一路转发给底盘。

**节点**：`twist_mux`（`pkg="twist_mux" type="twist_mux"`），在多个 launch 文件中启动：`src/open_mower/launch/open_mower.launch:16-20`（正式运行，`respawn="true" respawn_delay="10"`，并 `remap from="cmd_vel_out" to="/ll/cmd_vel"`）、以及 `move_around.launch`、`sim_mower_logic.launch`、`sim_navigation.launch` 中的仿真/调试等价配置（均加载同一份 `twist_mux_topics.yaml`，但后三者未显式 remap 输出话题，使用默认 `cmd_vel_out`）。

#### 输入

**订阅的话题**（`src/open_mower/params/twist_mux_topics.yaml`）：
```yaml
topics:
  - name: navigation
    topic: nav_vel
    timeout: 0.5
    priority: 10
  - name: logic
    topic: logic_vel
    timeout: 0.5
    priority: 50
  - name: logic
    topic: override_vel
    timeout: 0.5
    priority: 100

locks: []
```
逐项说明：
- `navigation` / `/nav_vel`（优先级 **10**，超时 **0.5s**）：来自 `move_base_flex`（`ftc_local_planner` 计算的导航速度，`_move_base.launch:4` 把控制器输出 remap 到此话题）。三路中优先级最低——正常自动导航/割草时使用，但可被更高优先级的手动/急停指令随时抢占。
- `logic` / `/logic_vel`（优先级 **50**，超时 **0.5s**）：由 `mower_logic` 节点直接发布（`mower_logic.cpp:737`：`cmd_vel_pub = n->advertise<Twist>("/logic_vel", 1)`）。用途有二：(1) 状态机自身在某些行为（如未通过 `move_base_flex` 的简单动作）中直接下发速度；(2) 摇杆/遥控（`/joy_vel`，来自 `xbot_monitoring`/`xbot_remote`/`_teleop.launch` 的 joy_node）在 `mower_logic.cpp:704-709` 的 `joyVelReceived()` 回调中被**原样转发**到 `/logic_vel`——但仅当 `currentBehavior && currentBehavior->redirect_joystick()` 为真时才转发，即只有当前行为状态机允许摇杆接管才会透传。注意 `/joy_vel` 并未作为独立 topic 项配置进 `twist_mux_topics.yaml`，它是先被 `mower_logic` 判断/转发到 `/logic_vel` 后，再间接参与 twist_mux 仲裁的，因此摇杆遥控和 `mower_logic` 自身指令共享同一优先级（50）、同一 `logic_vel` 话题。
- `logic` / `/override_vel`（优先级 **100**，超时 **0.5s**）：三路中最高优先级，预留的"覆盖"通道。经全仓库搜索，当前代码中**没有任何节点发布** `/override_vel`（`mower_logic`、xbot 系列均未引用该话题名），推测是为外部安全覆盖/未来功能（如紧急遥控抢占）预留的接口，当前版本未启用。
- `locks: []`：空的锁定话题列表。`4f31a93`（"Add missing locks field to twist_mux_topics.yaml"）提交专门补上了这个字段——较新版本的 `ros-noetic-twist-mux` 若配置文件缺少 `locks` 键会直接报 FATAL 参数错误无法启动（导致完全没有速度指令能到达底盘），即使本项目并不使用锁定话题机制，也必须显式给出空列表。

**动态参数**：twist_mux 自身另支持 `dynamic_reconfigure` 调整超时等，但本仓库未额外配置，使用 yaml 静态值。

#### 输出
- `cmd_vel_out`（remap 为 `/ll/cmd_vel`，仅在 `open_mower.launch:17` 中显式 remap；其余 launch 文件中为默认话题名 `cmd_vel_out`）：仲裁后选出的最终 `geometry_msgs/Twist`，直接发往底盘驱动（`mower_comms_v2`/`ll` 底层接口），是整条导航执行链路的最终出口。

#### 主要算法/流程
twist_mux 标准仲裁逻辑（未在本仓库改动，源码不在仓库中，行为按上游实现）：对配置的每个输入话题维护"最后收到消息的时间戳"，每个控制周期在**所有未超时**（距今 < 对应 `timeout`，本仓库三路均为 0.5s）的话题里选**优先级数字最大**的一路，把其 `Twist` 原样转发到 `cmd_vel_out`；若该路数据本身超过 0.5s 未更新则视为失效自动降级到次优先级的有效话题；若所有话题都超时，则输出零速度（或不发布，取决于上游实现细节）。本仓库的优先级设计体现了一条清晰的安全/控制层级：
1. **导航自动驾驶**（`/nav_vel`，优先级 10）——最低，日常割草/导航使用。
2. **状态机指令与摇杆遥控**（`/logic_vel`，优先级 50）——`mower_logic` 状态机可随时通过发布到该话题抢占导航指令（例如对接微调、紧急停止动作等自定义逻辑），且当状态机允许时，人工摇杆输入经 `mower_logic` 透传到同一话题，与状态机指令同级。
3. **保留覆盖通道**（`/override_vel`，优先级 100）——最高优先级，当前代码未使用，为未来外部强制覆盖预留。
`locks` 机制（基于布尔/传感器话题临时完全锁死某些或所有速度源）在本仓库未启用（空列表），仅为满足新版 twist_mux 的强制参数要求而声明。

---

## 六、监控与遥控

把机器人状态/传感器数据/事件通过 MQTT 暴露给 App 等监控端，并提供手柄、网页摇杆、App 端等多种手动遥控输入通道。

### xbot_monitoring

**角色**：机器人状态/传感器数据/事件上报与远程控制入口。通过本地 MQTT broker（Mosquitto，`docker/assets/mosquitto.conf`）把机器人状态、传感器数据、地图、事件等以 JSON/BSON 形式发布到一组结构化的 MQTT 主题上，供 OpenMowerApp（通过 `ws://<host>:9001` 的 WebSocket-MQTT）、Home Assistant 或其它监控系统消费；同时反向接收 App/其它客户端下发的驱动指令、动作指令（开始/停止割草等）和通用 JSON-RPC 请求。**它自身不是 MQTT broker**，而是 broker 的一个普通 TCP 客户端（连接 `127.0.0.1:1883`），真正对外暴露 WebSocket（9001端口，`protocol websockets`）的是 Mosquitto（见 `docker/assets/mosquitto.conf:4-10`）。

**节点**：
- `xbot_monitoring`（`src/lib/xbot_monitoring/src/xbot_monitoring.cpp`）— 主监控/MQTT 网关节点。
- `heatmap_generator`（`src/lib/xbot_monitoring/src/heatmap_generator.cpp`）— 可选的传感器热力图节点，仅当设置了 `OM_HEATMAP_SENSOR_IDS` 时才在 `open_mower.launch:27-29` 中启动。
- `xbot_sensor_example`（`src/lib/xbot_monitoring/src/xbot_sensor_example.cpp`）— 示例/测试节点，演示如何向 `xbot_monitoring` 注册一个自定义 double 传感器，不在生产 launch 文件中使用。

#### 输入

**ROS 话题订阅**（`xbot_monitoring.cpp:864-873`）：
| 话题 | 消息类型 | 用途 |
|---|---|---|
| `xbot_monitoring/robot_state` | `xbot_msgs/RobotState` | 电量、GPS质量、当前状态机状态/子状态、当前动作进度、位姿等 → 转发为 `robot_state/json` |
| `mower_map_service/json_map` | `std_msgs/String`（JSON字符串） | 地图 GeoJSON → 转发为 `map/json` |
| `xbot_monitoring/map_overlay` | `xbot_msgs/MapOverlay` | 叠加多边形（如规划路径可视化）→ 转发为 `map_overlay/json` |
| `/xbot_positioning/xb_pose` | `xbot_msgs/AbsolutePose` | 高频定位位姿，进入 150ms 采样缓冲 → `position/json` |
| `/xbot_monitoring/mqtt_publish` | `xbot_mqtt/MqttPublish`（自定义，`topic`/`payload`/`retain`） | **通用 MQTT 转发通道**：任何 ROS 节点都可以发这个话题，让 `xbot_monitoring` 原样把 `payload` 发布到指定 MQTT `topic`（用于事件、扩展消息等），见下方"事件"机制 |
| `xbot_monitoring/sensors/<id>/info`（动态发现，正则匹配 `/xbot_monitoring/sensors/.*/info`） | `xbot_msgs/SensorInfo` | 传感器元数据（量纲、单位、阈值等），10Hz 轮询 `ros::master::getTopics()` 发现新话题 |
| `xbot_monitoring/sensors/<id>/data`（发现 info 后动态订阅） | `xbot_msgs/SensorDataDouble` 或 `xbot_msgs/SensorDataString`（依据 `SensorInfo::value_type`） | 传感器实时数据 |
| `xbot_mqtt::TOPIC_RESPONSE` = `/xbot/rpc/response` | `xbot_mqtt/RpcResponse` | 其它节点对 RPC 请求的应答 |
| `xbot_mqtt::TOPIC_ERROR` = `/xbot/rpc/error` | `xbot_mqtt/RpcError` | 其它节点对 RPC 请求的错误应答 |

**ROS 服务提供**：
- `xbot/register_actions`（`xbot_msgs/RegisterActionsSrv`，`xbot_monitoring.cpp:864,735-746`）：其它节点（如 `mower_logic`）用它注册一组"动作"（action_id/action_name/enabled），按 `node_prefix` 分组存储，注册后立即重新发布 `actions/json`。
- `xbot_mqtt::SERVICE_REGISTER_METHODS` = `/xbot/rpc/register`（`xbot_mqtt/RegisterMethodsSrv`，`xbot_monitoring.cpp:881,821-826`）：其它节点用它注册自己能处理的 JSON-RPC 方法名列表（`register_methods()`），用于路由/校验传入的 RPC 请求。

**MQTT 订阅**（内部 MQTT 客户端订阅，连接成功时执行，`xbot_monitoring.cpp:94-104`）：
- 前缀为空字符串（本地 client）：订阅 `teleop`、`command`、`action`、`rpc/request`（以及为了兼容旧版本同时订阅带前导 `/` 的 `/teleop`、`/command`、`/action` —— 标注为 deprecated）。
- 若启用外部 MQTT（`external_mqtt_enable=true`），第二个客户端 `client_external_` 以 `external_mqtt_topic_prefix` 为前缀订阅同样的一组主题。

**MQTT 消息 → ROS 的精确映射**（`MqttCallback::message_arrived`，`xbot_monitoring.cpp:112-139`）：
| MQTT 主题（相对前缀） | payload 格式 | 处理 |
|---|---|---|
| `teleop`（旧版 `/teleop`已废弃保留兼容） | **BSON** 编码的 JSON 对象，字段 `vx`（float，线速度）、`vz`（float，角速度） | 解析后构造 `geometry_msgs/Twist{linear.x=vx, angular.z=vz}`，发布到 ROS 话题 `xbot_monitoring/remote_cmd_vel`（在 `open_mower.launch:24` 被 remap 为 `/joy_vel`） |
| `action`（旧版 `/action` 已废弃保留兼容，会打印 WARN） | 纯文本字符串（动作 ID，如 `mower_logic:area_recording/record_dock`，或者 `mower_logic:reset_emergency` 等） | 原样封装为 `std_msgs/String`，发布到 `xbot/action`（`mower_logic` 等节点订阅后按 action_id 前缀分发处理，如开始/停止割草、清除急停、进入录制模式等，属于 `mower_logic` 模块行为，此处不展开） |
| `command`（订阅了但代码里 `message_arrived` 未处理，无对应 `else if` 分支——当前实现中该主题的消息会被静默丢弃） | — | 无操作（历史遗留主题，未接入任何处理逻辑） |
| `rpc/request` | JSON-RPC 2.0 请求字符串 | 交给 `rpc_request_callback()` 处理（见下方"主要算法"） |

**launch 参数**（`src/open_mower/launch/include/_params.launch:160-167`，均来自环境变量 `optenv`）：
- `xbot_monitoring/external_mqtt_enable`（`OM_MQTT_ENABLE`，默认 false）
- `xbot_monitoring/external_mqtt_hostname`（`OM_MQTT_HOSTNAME`）
- `xbot_monitoring/external_mqtt_port`（`OM_MQTT_PORT`，默认 1883）
- `xbot_monitoring/external_mqtt_username` / `external_mqtt_password`（`OM_MQTT_USER`/`OM_MQTT_PASSWORD`）
- `xbot_monitoring/external_mqtt_topic_prefix`（`OM_MQTT_TOPIC_PREFIX`）
- `xbot_monitoring/software_version`（`OM_SOFTWARE_VERSION`）

#### 输出

**MQTT 主题**（本地 broker `127.0.0.1:1883`；若启用外部 MQTT，同样内容再加 `external_mqtt_topic_prefix` 前缀发到外部 broker，见 `try_publish()`，`xbot_monitoring.cpp:275-299`）。命名规则：无固定命名空间前缀（本地客户端 `mqtt_topic_prefix=""`），主题名即为下表左列；很多主题成对存在 `json`（文本/UTF-8）版本和 `bson`（二进制）版本，`json` 版本供网页/调试工具直接读，`bson` 版本供 App 高效解析，二者内容相同（`bson` 版本额外包一层 `{"d": ...}`）：

| MQTT 主题 | 触发时机 | retain | 内容/格式 |
|---|---|---|---|
| `capabilities/json` | MQTT 连接建立时 | 是 | `capabilities.h` 中静态声明的能力表：`{"rpc":1,"map:json":1,"mqtt:params":1,"events":1,"position":1}`，供客户端探测服务端支持哪些功能 |
| `version/json`、`version`（bson） | 连接建立时 | 是 | `{"version": "<OM_SOFTWARE_VERSION>"}` |
| `params/json` | 连接建立时 | 是 | 全部 ROS 参数（`ros::param::getParamNames`）转成 JSON；名字里含 `password` 的参数值被替换为 `null`（脱敏），XML-RPC 值经 `xmlrpc_to_json()` 递归转换 |
| `sensor_infos/json`、`sensor_infos/bson` | 连接建立时；发现新传感器或传感器元数据变化时 | 是 | 已发现的全部传感器的 `SensorInfo`（id、名称、类型、单位、量纲描述、min/max、critical 阈值）数组 |
| `map/json`、`map/bson` | 连接建立时；`mower_map_service/json_map` 收到新地图时 | 是 | 地图 GeoJSON（原样转发自 `mower_map_service`） |
| `map_overlay/json`、`map_overlay/bson` | 连接建立时；`xbot_monitoring/map_overlay` 收到新叠加层时 | 是 | `{"polygons":[{"poly":[{x,y},...],"is_closed":bool,"line_width":..,"color":..}, ...]}` |
| `actions/json`、`actions/bson` | 连接建立时；`xbot/register_actions` 服务被调用后 | 是 | `[{"action_id":"<node_prefix>/<action.action_id>","action_name":..,"enabled":..}, ...]` |
| `robot_state/json`、`robot_state/bson` | 每次收到 `xbot_monitoring/robot_state` | 否 | 电量%、GPS质量%、当前动作进度、当前状态/子状态、当前区域、当前路径及索引、急停标志、充电中标志、下雨检测标志，以及 `pose.{x,y,heading,pos_accuracy,heading_accuracy,heading_valid}` |
| `position/json` | 每 150ms（`MQTT_POSITION_PUBLISH_INTERVAL`）一次，`pose_publish_timer_callback` | 否 | 对 150ms 窗口内收到的若干个 `/xbot_positioning/xb_pose` 采样做**逐分量中位数滤波**（x、y、heading 分别取中位数），得到 `{"x","y","heading","attributes":{job_id,session_id,blades}}`；`attributes` 来自 `PositionHistory` 当前跟踪的作业属性 |
| `sensors/<sensor_id>/data`、`sensors/<sensor_id>/bson` | 每次收到对应传感器的数据话题 | 否 | 单个 double（转成字符串）或 string 值；bson 版本包 `{"d": value}` |
| `events/json` | 每次调用 `publish_event()`（内部）或收到经 `/xbot_monitoring/mqtt_publish` 转发且主题为 `events/json` 的消息 | 否 | JSON-RPC 风格事件对象：`{"id":<32位随机NanoID>,"t":<ROS时间,秒(float)>,"type":"<事件类型>", ...其它字段}`；目前节点自身仅在启动/关闭时发布 `BOOTED`/`SHUTDOWN`（`xbot_monitoring.cpp:888,939`），其余事件（如 `STATE`、`BLADES`）由 `mower_logic` 等节点经 `/xbot_monitoring/mqtt_publish` 话题转发过来 |
| `rpc/response` | 每次收到 RPC 响应/错误，或本地立即处理的错误 | 否 | JSON-RPC 2.0 响应：`{"jsonrpc":"2.0","result":..,"id":..}` 或 `{"jsonrpc":"2.0","error":{"code":..,"message":..},"id":..}` |
| 由 `/xbot_monitoring/mqtt_publish` 转发的任意主题 | 任何节点发布该话题时 | 由消息的 `retain` 字段决定 | 消息体透传，`payload` 原样作为 MQTT payload |

**ROS 话题发布**：
- `xbot_monitoring/remote_cmd_vel`（`geometry_msgs/Twist`）— 由 MQTT `teleop` 消息解码而来；在生产 launch 文件中被 remap 为 `/joy_vel`（`open_mower.launch:24`、`sim_mower_logic.launch:50`）。
- `xbot/action`（`std_msgs/String`）— 由 MQTT `action`/`/action` 消息转发而来，供 `mower_logic` 等节点消费。
- `xbot_mqtt::TOPIC_REQUEST` = `/xbot/rpc/request`（`xbot_mqtt/RpcRequest`）— 由 MQTT `rpc/request` 消息校验、路由后转发。

**RPC 方法**（`xbot_monitoring` 自己作为一个 `xbot_mqtt::RpcProvider`，node_id=`xbot_monitoring`，`xbot_monitoring.cpp:159-208`）：
- `rpc.ping` → `"pong"`
- `rpc.methods` → 返回全系统所有已注册 RPC 方法名（合并所有节点通过 `register_methods` 服务注册的方法，排序后输出）
- `events.history` / `events.history.list` / `events.history.delete` → 读取/列出/删除按日期（`YYYYMMDD`）存档的事件历史（见下）
- `position.history` / `position.history.list` / `position.history.delete` → 读取/列出/删除按 `job_id` 存档的位置历史轨迹（见下）

#### 主要算法/流程

1. **双 MQTT 客户端架构**（`setupMqttClient()`，`xbot_monitoring.cpp:211-273`）：始终创建一个本地客户端（client id `xbot_monitoring`，连接 `tcp://127.0.0.1:1883`），若 `external_mqtt_enable` 为真，再创建一个 `ext_xbot_monitoring` 客户端连接外部 broker（可带用户名密码）。两个客户端都设置 `set_automatic_reconnect(true)`、`set_clean_session(true)`、`keep_alive_interval=1000s`、`max_inflight=10`。`try_publish()`/`try_publish_binary()` 对两个客户端都尝试发布，任何一个抛出 `mqtt::exception`（通常是未连接）都被静默吞掉，不阻塞主流程——这是断线容错机制，没有消息队列/重试，断线期间的非 retained 消息会丢失。
2. **重连即重新推送全量快照**：`MqttCallback::connected()`（`xbot_monitoring.cpp:84-105`）在（重）连接成功时立即重新发布 capabilities、传感器元数据、地图、地图叠加层、动作列表、版本、参数（这些都是 `retain=true` 的"配置类"数据），保证新客户端/重连客户端立刻拿到完整快照，而无需等待下一次数据更新。
3. **动态传感器发现**（`main()` 主循环，`xbot_monitoring.cpp:890-938`）：以 10Hz 轮询 `ros::master::getTopics()`，用正则 `^/xbot_monitoring/sensors/.*/info$` 匹配，发现新话题后订阅其 `SensorInfo`；首次收到或内容变化时更新 `found_sensors` 并（若是新传感器）动态创建对该传感器 `data` 话题的订阅（`subscribe_to_sensor()`），然后重新发布 `sensor_infos/json`。传感器 info 订阅永久保留（代码注释说明是为了支持阈值等动态变化）。
4. **位置采样与中位数滤波**（`pose_callback` + `pose_publish_timer_callback`，`xbot_monitoring.cpp:575-611`）：`/xbot_positioning/xb_pose` 以远高于 150ms 的频率到来时，先缓冲进 `pose_buffer`；定时器每 150ms 触发一次，把缓冲区 swap 出来，对 x/y/heading 三个分量分别用 `std::nth_element` 求中位数（而非平均），以抑制野值抖动，同时把该中位数点喂给 `PositionHistory::addPoint()` 做轨迹压缩存档，再发布 `position/json`。
5. **JSON-RPC 2.0 请求路由**（`rpc_request_callback()`，`xbot_monitoring.cpp:755-803`）：解析 payload→校验是对象、`id`（若有）必须是字符串、`jsonrpc=="2.0"`、`method` 是字符串；`meta.*` 和 `ext.*` 前缀的方法被静默忽略（约定由其它服务处理，如 App 自身内建方法）；在 `registered_methods`（由各节点通过 `/xbot/rpc/register` 服务登记）中查找该方法，找不到则回 `ERROR_METHOD_NOT_FOUND`；找到则包成 `xbot_mqtt::RpcRequest` 发到 `/xbot/rpc/request`，由实际实现该方法的节点（可能是 `xbot_monitoring` 自己，通过本地 `RpcProvider` 直接处理；也可能是外部节点，通过订阅 `/xbot/rpc/request` 处理后经 `/xbot/rpc/response`(`/xbot/rpc/error`) 回传）异步处理，最终统一由 `rpc_response_callback`/`rpc_error_callback` 封装成 JSON-RPC 响应发到 MQTT `rpc/response`。这是一个中心化的"MQTT↔ROS RPC 网关"模式：MQTT 侧只认识一个请求/一个响应主题，路由与鉴权（是否已注册）都在 `xbot_monitoring` 完成。
6. **事件历史持久化**（`EventHistory`，`src/lib/xbot_monitoring/src/EventHistory.h`）：按天分文件，`event_history/<YYYYMMDD>.jsonl`，每行一条 JSON。`add()` 把新事件同时写入内存缓冲区（仅保留当天）和追加写入磁盘；跨天时清空内存缓冲、在目录索引前插入新日期。`getAll(date)` 优先从内存服务当天数据，历史日期则直接读盘解析。`deleteHistory` 支持删单日或全部。
7. **位置历史两阶段压缩算法**（`PositionHistory`，`src/lib/xbot_monitoring/src/PositionHistory.h`，注释中说明是**镜像 OpenMowerApp 前端的 `useTrack.ts`/`decimate.ts`/`rdp.ts` 实现**，保证前后端轨迹渲染结果一致）：
   - 只有 `session_id` 非空（即处于一次作业会话）时，`addPoint()` 才把中位数滤波后的点加入原始缓冲 `rel_buffer_`；
   - 缓冲超过 `COMPACTION_THRESHOLD=60` 点即触发 `compact()`：先做**抽稀（decimate）**——按距离阈值 `DIST_THRESHOLD=10cm`（噪声阈值 `NOISE_DIST_THRESHOLD=3cm`）、朝向变化阈值 `HEADING_THRESHOLD=8°`、心跳点数 `HEARTBEAT_POSITIONS=10`（超过这么多点没被保留就强制保留一个）筛选关键点；再对抽稀结果做 **Ramer–Douglas–Peucker 简化**（`RDP_EPSILON=1cm`）；简化后的点中标记为 `committable` 的（排除用于平滑上下文的最后 `CONTEXT_WINDOW=10` 个点，flush 时除外）被提交进当前 `history_segments_` 的末尾 segment；
   - 事件驱动的 segment 切分（`onEvent()`）：解析事件 JSON，`STATE` 事件更新 `job_id`/`session_id`，`BLADES` 事件更新刀片开关状态；`job_id` 变化 ⇒ 强制 flush 全部缓冲、关闭当前文件、（若新 job_id 非空）重置状态并开启新文件；仅 `session_id`/`blades` 变化 ⇒ flush 缓冲并在同一文件内开新 segment，新 segment 会以上一个 segment 的最后一点作为"桥接点"（bridge vertex）保证轨迹连续不断线；
   - 持久化文件命名 `position_history/<unix_epoch>_<job_id>.jsonl`，每行是 `{"type":"new_segment","attributes":{job_id,session_id,blades},"points":[[x,y],...]}` 或 `{"type":"append_points","points":[[x,y],...]}` 两种记录之一；用"已写入 segment 数/已写入点数"两个水位线（`written_seg_count_`/`written_point_count_`）做增量写入，避免重复写盘；每 30 秒（`POSITION_HISTORY_FLUSH_INTERVAL`）由定时器触发一次 `periodicFlush()`，ROS 关闭时再做一次全量 `flush()`。
   - 节点重启时 `initializeFromDisk()` 会加载最新的 job 文件、恢复 `job_id`（但**不**恢复 `session_id`/`blades`，注释说明是为了避免开机后在"未挂钩"状态下误记录轨迹），并另起一个不带桥接点的新 segment。
8. **手柄/App 驱动指令的最终去向**：`xbot_monitoring/remote_cmd_vel` remap 成 `/joy_vel` 后，被 `mower_logic`（不在本模块文档范围，仅提示数据流向）通过 `n->subscribe("/joy_vel", ...)` 订阅（`src/mower_logic/src/mower_logic/mower_logic.cpp:773`），在 `joyVelReceived()`（`mower_logic.cpp:704-709`）中：只有当前行为（behavior）的 `redirect_joystick()==true` 时才把收到的 `Twist` 转发到 `/logic_vel`（`mower_logic.cpp:737` 广告的话题），同时记录 `joy_vel_time` 用作 10 秒超时保护——若 `redirect_joystick()` 为真但超过 10 秒没有新的 `/joy_vel` 消息，`mower_logic` 主循环会主动调用 `stopMoving()`（`mower_logic.cpp:554-558`），防止遥控信号丢失后机器人持续运动（这是唯一的"看门狗"/死人开关逻辑，位于 `mower_logic` 而非 `xbot_monitoring`/`xbot_remote` 本身）。`/logic_vel` 最终经 `twist_mux`（优先级 50，超时 0.5s，见 `src/open_mower/params/twist_mux_topics.yaml`）汇入 `/ll/cmd_vel`。
   - 备注：`twist_mux_topics.yaml` 中还定义了一个优先级更高（100）的 `override_vel` 输入话题，架构文档 `doc/architecture.md:99` 描述手柄/App 遥控"殊途同归都汇入 `override_vel`"，但**实际代码中未发现任何节点发布到 `/override_vel`**（全仓库检索无匹配）——当前实现里手柄/App 指令实际是经由 `/logic_vel`（优先级 50）汇入的，`override_vel` 目前是预留但未使用的输入。
9. **未实现 Home Assistant 原生 MQTT Discovery**：代码中没有发现 `homeassistant/...` 前缀的 discovery 主题或 `device`/`config` payload（HA 官方 MQTT Discovery 协议的标志性特征）。`xbot_monitoring` 只是把状态/传感器数据发布到上述通用 JSON/BSON 主题，Home Assistant 若要接入需要用户手动配置通用 `MQTT sensor`/`MQTT switch` 等 YAML 集成去订阅这些主题，或依赖第三方桥接；没有自动发现机制。

---

### xbot_remote

**角色**：一个独立的、极简的 WebSocket 服务器，监听 **9002 端口**，为手机/浏览器上的摇杆提供低延迟遥控通道，解析后直接发布为 ROS `Twist` 消息，最终汇入 `/joy_vel`。与 `xbot_monitoring` 的 MQTT `teleop` 通道是两条独立的驱动指令入口，二者殊途同归都 remap 到 `/joy_vel`。

**节点**：`xbot_remote`（`src/lib/xbot_remote/src/xbot_remote.cpp`），单一可执行文件，无子节点。

#### 输入

**WebSocket 客户端 → 服务端消息**（`on_message()`，`xbot_remote.cpp:35-47`）：
- 协议：原生 WebSocket（`websocketpp` + `asio_no_tls`，即**未加密**，无 TLS/WSS），无自定义握手/鉴权/子协议协商，任何能连上 9002 端口的客户端即可发送指令。
- 消息体：**BSON** 编码（`nlohmann::json::from_bson(msg->get_payload())`）的二进制帧，须包含至少两个字段：
  - `vx`（数值，线速度）
  - `vz`（数值，角速度）
- 无其它字段/消息类型支持——只处理一种消息格式，没有握手、心跳、ping/pong 业务层协议，也没有区分"连接建立"/"断开"事件的业务逻辑（依赖 websocketpp 底层的连接管理）。
- 解析异常（字段缺失、BSON 格式错误等）会被 `catch(std::exception&)` 捕获并打印 `ROS_ERROR_STREAM`，不会导致服务崩溃，也不会断开该连接。

**launch 参数**：无（未声明任何 ROS 参数，硬编码端口 9002）。

#### 输出

**ROS 话题发布**：
- `xbot_remote/cmd_vel`（`geometry_msgs/Twist`，队列长度1，`xbot_remote.cpp:102`）— `t.linear.x = vx; t.angular.z = vz;`，每收到一帧 WebSocket 消息就发布一次。生产 launch 文件中被 remap 为 `/joy_vel`（`open_mower.launch:32`、`sim_mower_logic.launch:54`）。

**WebSocket 服务端 → 客户端**：代码中未注册任何出站消息回调/发送逻辑——服务器不向客户端回送任何数据（既没有状态回执，也没有心跳/keepalive 消息），是纯单向的"输入通道"。

#### 主要算法/流程

1. **服务器启动**（`server_thread()`，`xbot_remote.cpp:49-94`）：在独立 pthread 中运行 `websocketpp::server<asio>`。先尝试双栈监听 `listen(9002, ec)`；如果失败（通常因为系统缺 IPv6），回退为显式 IPv4 监听 `listen(tcp::v4(), 9002, ec)`；两次都失败则 `ROS_FATAL` 并抛异常（`main` 中未捕获，会导致 `exit(1)`）。日志渠道 `set_access_channels(alevel::all)` 但清除了 `frame_payload` 通道（不把每帧内容打到日志里，避免刷屏/泄露）。`set_reuse_addr(true)` 允许快速重启复用端口。
2. **单线程 ASIO 事件循环**：`echo_server.run()` 阻塞运行在该 pthread 内，`on_message` 回调在这个线程上下文中直接调用 `cmd_vel_pub.publish()`——发布操作发生在非 ROS 主线程，但 `roscpp` 的 Publisher::publish 是线程安全的，因此没有额外加锁。
3. **无死人开关（deadman）/超时保护**：`xbot_remote` 本身**不做**任何速度指令超时清零——只要收到消息就发布一次 `Twist`，断线或客户端停止发送后不会主动发布零速度。真正的安全网是下游 `mower_logic::joyVelReceived()` 的 10 秒超时+`stopMoving()`（见上文 `xbot_monitoring` 小节第 8 点，`mower_logic.cpp:554-558,704-709`），以及 `twist_mux_topics.yaml` 中各输入话题 0.5 秒超时后清零速度的机制（`twist_mux` 层面的超时对 `/joy_vel` 本身不适用，因为 `/joy_vel` 并未直接接入 `twist_mux`，而是先经 `mower_logic` 转发为 `/logic_vel`，`/logic_vel` 才是 `twist_mux` 里超时 0.5s 的话题）。
4. **优雅关闭**：`main()` 中 `ros::spin()` 返回后（收到 SIGINT 等）调用 `echo_server.stop()` 并 `pthread_join` 等待服务线程退出。
5. **无鉴权/加密**：整个模块没有任何身份验证、Token、TLS，安全性完全依赖网络层隔离（通常只在局域网内暴露）。

---

### mower_utils

**角色**：一组零散的小工具节点集合，目前只包含两个可执行文件，服务于可视化调试与规划器离线测试。

**节点**：
- `xbot_pose_converter`（`src/mower_utils/src/xbot_pose_converter.cpp`）— 把 `xbot_msgs/AbsolutePose` 转换为标准 `geometry_msgs/PoseWithCovarianceStamped`，供 RViz 等标准 ROS 可视化工具直接显示（`xbot_msgs/AbsolutePose` 是 OpenMower 自定义消息，RViz 原生不认识）。
- `planner_test`（`src/mower_utils/src/planner_test.cpp`）— 命令行离线测试工具：从 `mower_map_service` 拉取指定编号的割草区域，调用 `slic3r_coverage_planner` 生成覆盖路径，发布到 `mower_logic/mowing_path`（`nav_msgs/Path`，latched）供 RViz 查看效果，不参与运行时系统。

`src/mower_utils/CMakeLists.txt:155-164` 中仅声明并编译这两个可执行文件（`planner_test`、`xbot_pose_converter`），没有其它节点。`src/mower_utils/launch/planner_test.launch` 和 `src/mower_utils/rviz/planner_test.rviz` 是配套的启动文件和 RViz 预设。

#### xbot_pose_converter 详细说明

##### 输入
- ROS 参数（私有命名空间 `~`）：
  - `topic`（**必需**，无默认值，缺失则 `ROS_ERROR` 并 `return 1` 直接退出）— 要转换的源话题名，例如 `/ll/position/gps` 或 `/xbot_positioning/xb_pose`
  - `frame`（默认 `"frame"`）— 输出 `PoseWithCovarianceStamped.header.frame_id` 覆盖值，实际部署中传 `"map"`
  - `use_motion_vector_orientation`（bool，默认 `false`）— 是否用运动矢量方向代替消息自带朝向
- 订阅：`<topic>`（`xbot_msgs/AbsolutePose`），队列深度 0（即无缓冲，仅处理最新）

##### 输出
- 发布：`<topic>/converted`（即私有命名空间下的 `converted`，`xbot_pose_converter.cpp:65` 用 `paramNh.advertise`，实际全局话题名是 `<node_private_ns>/converted`）— `geometry_msgs/PoseWithCovarianceStamped`，队列10，非 latched

##### 主要算法/流程
- `pose_received()`（`xbot_pose_converter.cpp:29-45`）：直接拷贝 `header` 和 `pose`（含协方差），把 `position.z` 强制置 0（约束到地面平面，便于 2D 可视化叠加）；
- 若 `use_motion_vector_orientation=true`：不使用消息自带的 `orientation`（该字段对某些数据源如原始 GPS fix 并无有意义的朝向，只有对地航向的估计），而是用 `msg->motion_vector.{x,y}` 算出偏航角 `yaw=atan2(motion_vector.y, motion_vector.x)`，转换成四元数（仅绕Z轴，`orientation={x:0,y:0,z:sin(yaw/2),w:cos(yaw/2)}`）替换朝向；
- 最后用参数 `frame` 覆盖 `header.frame_id`（因为源消息里的 frame_id 可能是传感器坐标系，需要统一到地图坐标系用于 RViz 叠加显示），发布。
- 实际用法示例（`test_playback.launch:9-16`、`sim_mower_logic.launch:24`）：为 `/ll/position/gps`（原始GPS AbsolutePose）和 `/xbot_positioning/xb_pose`（融合定位输出）分别起一个 `xbot_pose_converter` 实例，都转换到 `frame=map`，从而在 RViz 中同时对比原始GPS轨迹与融合后轨迹。

---

### xbot_mqtt（简述）

**角色**：`xbot_monitoring` 所依赖的通用 MQTT-based RPC 库/消息定义包（`src/lib/xbot_mqtt`），不是一个独立运行的节点，而是被其它节点 `#include` 使用的库+ROS消息/服务定义集合。除 `xbot_monitoring` 外，`src/mower_simulation` 的 `SimRpc`（暴露 `sim.dock.move`/`sim.battery.set` 等测试用 RPC 接口）也基于它实现。

- **消息/服务定义**（`msg/`、`srv/`）：`MqttPublish.msg`（topic/payload/retain，用于任意节点向 MQTT 转发任意消息）、`RpcRequest.msg`/`RpcResponse.msg`/`RpcError.msg`（JSON-RPC 请求/响应/错误的 ROS 化表示）、`RegisterMethodsSrv.srv`（节点向中心注册自己支持的 RPC 方法名）。
- **`xbot_mqtt::RpcProvider`**（`include/xbot_mqtt/provider.h` + `src/provider.cpp`）：供业务节点声明一组 `{方法名 → 回调}` 映射（`RPC_METHOD` 宏简化写法），`init()` 时订阅 `/xbot/rpc/request`、广告 `/xbot/rpc/response`/`/xbot/rpc/error`，并调用 `/xbot/rpc/register` 服务把自己的方法名列表登记到 `xbot_monitoring`（若注册服务10秒内不存在，仅打 ERROR 日志不中止，因为 RPC 对系统非关键）；收到请求后按方法名查表执行回调，回调可抛 `RpcException{code,message}` 转成标准错误响应。
- **常量**（`include/xbot_mqtt/constants.h`）：统一定义话题名 `/xbot/rpc/request`、`/xbot/rpc/response`、`/xbot/rpc/error` 和服务名 `/xbot/rpc/register`。
- **辅助函数**（`include/xbot_mqtt/publish.h`）：`MQTT_PUBLISHER` 宏快速声明发布到 `/xbot_monitoring/mqtt_publish` 的 Publisher；`generateNanoId()`（32位随机ID，字母表与 `mower_map_service` 的 `generateNanoId()` 算法一致）；`buildEventPayload()`/`publishEvent()` 统一生成/发布带 `id`/`t`(时间戳)/`type` 字段的事件 JSON 到 `events/json` 主题。

实际上，`xbot_mqtt` 把"业务节点自己实现具体功能，中心节点（`xbot_monitoring`）统一做 MQTT⇄ROS 网关和方法路由/事件归档"这一模式抽象成了可复用库，值得在专门的 `xbot_mqtt` 文档中深入展开（如另一位协作者所述，此处从简）。

---

### 手柄/遥操作栈（joy / teleop_twist_joy / joy_teleop）

这些是**系统自带的标准 ROS 包**（不在本仓库 `src/lib` 下自研），仓库中只提供其配置/启动方式，未自研核心逻辑。相关文件均为 `src/open_mower/launch/include/_teleop.launch` 及 `src/open_mower/params/gamepads/*.yaml`。

#### 输入
- 物理游戏手柄的原始 HID/joystick 设备输入（由 `joy` 包的 `joy_node` 读取，发布标准 `sensor_msgs/Joy`）。
- 环境变量 `OM_MOWER_GAMEPAD`（默认 `xbox360`）决定走哪条配置分支。

#### 输出（`_teleop.launch:1-30`）
两种互斥的启动分支：

1. **`OM_MOWER_GAMEPAD == "xbox360"`**（默认分支）：
   - `joy_node`（`~autorepeat_rate=10.0`、`~coalesce_interval=0.06`）发布 `/joy`（`sensor_msgs/Joy`）；
   - `teleop_twist_joy` 的 `teleop_node` 订阅 `/joy`，参数 `scale_linear=0.5`、`scale_angular=1.5`、`scale_linear_turbo=1.0`、`scale_angular_turbo=3.0`、`enable_turbo_button=4`（按钮4按住时用 turbo 倍率），输出话题 `cmd_vel` 被 remap 为 `/joy_vel`。此分支未配置任何"录制"相关按钮映射（Xbox360 手柄目前只做行驶，不做地图录制按键绑定）。

2. **其它手柄型号**（`shield`/`ps3`/`switch_pro`/`steam_touch`/`steam_stick` 等，对应 `src/open_mower/params/gamepads/<name>.yaml`）：
   - `joy_node` 把 `/joy` remap 为 `/teleop_joy`；
   - 加载对应手柄的 YAML 配置（`rosparam ... $(optenv OM_MOWER_GAMEPAD).yaml`）；
   - 用通用的 `joy_teleop`（Python 节点 `joy_teleop.py`）读取 `/teleop_joy`，按 YAML 中 `teleop.<action>` 各条目的 `axis_mappings`/`deadman_buttons`/`message_value` 生成对应消息。以 `shield.yaml`（`src/open_mower/params/gamepads/shield.yaml`）为例：
     - `move`：`geometry_msgs/Twist`，话题 `joy_vel`，死人开关按钮 `[0]`（A键，需按住才有效），轴0→`angular.z`，轴1→`linear.x`，比例均为1
     - `record_polygon`：`std_msgs/Bool`，话题 `record_polygon`，死人开关 `[1]`（B键），按下发布 `data=1`
     - `record_dock`：`std_msgs/Bool`，话题 `record_dock`，死人开关 `[2]`（X键）
     - `record_mowing`：`std_msgs/Bool`，话题 `record_mowing`，死人开关 `[3,4]`（Y+L1 组合键）
     - `record_navigation`：`std_msgs/Bool`，话题 `record_navigation`，死人开关 `[3,6]`（Y+Play 组合键）
   - 所有 `move` 类输出话题名都叫 `joy_vel`（各 YAML 中一致），launch 里没有额外 remap（因为已经直接命名为 `joy_vel`），最终与 `xbot_monitoring`/`xbot_remote` 的输出汇合到同一个全局话题 `/joy_vel`。

#### 与 mower_logic 的关系（仅作提示，详细行为见 mower_logic 相关文档）
- `/joy_vel` 三路合一后统一由 `mower_logic::joyVelReceived()` 订阅转发（见上文），是唯一真正消费 `/joy_vel` 的节点。
- 在**地图录制模式**（`AREA_RECORDING`，`src/mower_logic/src/mower_logic/behaviors/AreaRecordingBehavior.cpp`）下，`mower_logic` 并不通过 `/joy_vel` 获取录制操作，而是**直接订阅** `record_dock`(`/record_dock`)、`record_polygon`(`/record_polygon`)、`record_mowing`(`/record_mowing`)、`record_navigation`(`/record_navigation`) 四个 `std_msgs/Bool` 话题（`AreaRecordingBehavior.cpp:184-187`），这些话题正是上面 `joy_teleop` 按 YAML 配置直接产生的——即手柄的录制按键不经过 `/joy_vel`，而是被 `joy_teleop` 转成独立的 Bool 话题后由 `AreaRecordingBehavior` 直接读取消费，属于 `mower_logic` 模块的行为细节，本文档不再深入。

---

### 附：模块间数据流小结

```
                         ┌─────────────────────────────┐
物理手柄 --HID--> joy_node --/joy(或/teleop_joy)--> teleop_twist_joy / joy_teleop.py ---\
                                                                                          \
OpenMowerApp --MQTT(ws://<host>:9001, BSON teleop主题)--> Mosquitto --tcp:1883--> xbot_monitoring --/xbot_monitoring/remote_cmd_vel-->\  
                                                                                          |-- remap 全部到 --> /joy_vel --> mower_logic::joyVelReceived()
手机/浏览器摇杆 --WebSocket ws://<host>:9002 (BSON)--> xbot_remote --/xbot_remote/cmd_vel--/                (仅当 currentBehavior->redirect_joystick())
                                                                                          |
                                                                                          v
                                                                                    /logic_vel (10s无消息则 mower_logic 主动 stopMoving)
                                                                                          |
                                                                                          v
                                                                                     twist_mux (0.5s超时清零, priority=50)
                                                                                          |
                                                                                          v
                                                                                      /ll/cmd_vel --> mower_comms_v2 --> 电机
```

状态/传感器/事件上行链路：各节点 --ROS话题--> `xbot_monitoring` --MQTT(JSON+BSON, 主题如 `robot_state/json`、`sensors/<id>/data`、`events/json`、`position/json` 等)--> Mosquitto(1883本地 / 9001 WebSocket对外) --> OpenMowerApp / 手动订阅的 Home Assistant 等。
