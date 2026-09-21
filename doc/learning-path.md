# 学习路径：为接手 VSLAM / 建图任务打基础

这份文档是给"要在这个项目基础上做 VSLAM 和建图"这个目标定制的学习顺序，不是泛泛地介绍代码。目的是让你在看到具体需求之前，先把"现在的定位/建图是怎么做的"、"新东西大概会从哪里接进去"搞清楚，接到活儿的时候不会一头雾水。

先说结论：**这个项目现在完全没有视觉，定位靠 GPS+IMU+轮速的 EKF，"地图"靠人手动开一圈录 GPS 多边形**——这意味着你要做的 VSLAM/建图，八成不是在一个已有的视觉子系统上改，而是要新增一整条视觉定位/建图能力，然后想办法接进现有的这套 EKF + 状态机 + 导航流水线。下面的顺序就是照着"先看懂现状 → 再看懂接口 → 再看懂怎么验证"这个思路排的。

## 第 0 步：跑起来，建立直觉（花半天）

不看代码，先把仿真跑起来，亲手把机器人从充电桩开出去、录一块新地图、看它自己割草。

1. 按 [running-native.md](running-native.md) 把本地仿真跑起来（推荐本地版，改代码验证快，反正你接下来要写新代码）。
2. 照 [getting-started.md](getting-started.md) 里的命令让它实际动起来：Start、Dock、清急停。
3. 照 [architecture.md](architecture.md) 里"地图是哪来的"那节，自己用 `rosservice call ... "command: 3"` 进一次地图录制模式，手动开一圈录一块新地图，感受一下现在的"建图"到底是什么（提前说：非常原始，就是记 GPS 轨迹点连成多边形，没有任何视觉/激光）。
4. 跑的时候开着 `rqt_graph`、`rqt_console`，对着 [architecture.md](architecture.md) 的流水线图，把图里的每个方块对应到实际跑起来的节点上。

这一步的目的不是学会用这个项目，是让后面看代码的时候，脑子里有一个"这段代码在实际运行时是什么样"的画面。

## 第 1 步：定位这条线——你最需要看懂的部分

VSLAM 要解决的问题，本质上跟现在 `xbot_positioning` 在做的事是同一件事："给出机器人在地图里的位置"。所以这是整个项目里跟你关系最大的模块，要看到能讲清楚每一行在干什么的程度。

**看什么：**
- `src/lib/xbot_positioning/src/xbot_positioning.cpp`——节点入口。重点看：
  - 订阅了什么（`imu_in`、`twist_in`、`wheel_ticks_in`、**`xb_pose_in`**）、发布了什么（`odom_out`、**`xb_pose_out`**）。
  - 两个服务：`xbot_positioning/set_gps_state`、`xbot_positioning/set_robot_pose`——这是外部**手动干预/重置**这个 EKF 的入口，做视觉定位联调时很可能用得上（比如出问题了强制把位姿掰回来）。
- `src/lib/xbot_positioning/src/SystemModel.hpp` + `PositionMeasurementModel.hpp` + `OrientationMeasurementModel*.hpp` + `SpeedMeasurementModel.hpp`——EKF 的运动模型和几个测量模型。别被 Kalman 库的模板语法吓到，核心逻辑就是"预测下一步位置"+"来一个新的位置/朝向/速度测量时怎么修正"。**`PositionMeasurementModel` 这个类不关心测量值是从哪来的**——它是一个纯粹的"给定一个 (x, y) 绝对位置测量，怎么修正状态"的模型，历史上喂给它的是 GPS 天线位置，但模型本身跟"GPS"没有耦合。这一点很重要：说明这条 EKF 融合链路本来就是"传感器无关"设计的。
- `src/lib/xbot_msgs/msg/AbsolutePose.msg`——整条流水线里描述"一次绝对位姿测量"的通用消息格式。**注意里面的 `source` 枚举**：
  ```
  uint8 SOURCE_GPS=1
  uint8 SOURCE_LIGHTHOUSE=2
  uint8 SOURCE_SENSOR_FUSION=100
  ```
  `SOURCE_LIGHTHOUSE` 是给某种室内定位技术（激光基站/Vive 一类）预留的——说明这个消息格式从设计上就没打算只服务于 GPS。这大概率就是你的 VSLAM 位姿要塞进去的地方：让你的定位节点发一个 `xbot_msgs/AbsolutePose`（`source` 可以是 `SOURCE_SENSOR_FUSION` 或者申请一个新值），喂给 `xbot_positioning` 的 `xb_pose_in`（在 `_localization.launch` 里目前 remap 到 `/ll/position/gps`），或者直接旁路整个 EKF、自己发一份等价于 `xb_pose_out` 的位姿——两种做法都要在看懂这一节之后才好决定。

**练习**：跑着仿真的时候，`rostopic echo /xbot_positioning/xb_pose`，观察 `position_accuracy`、`orientation_valid` 这些字段怎么随 GPS 状态变化。想一下：如果这个消息是你的 VSLAM 节点发的，这几个字段应该怎么填才不会把下游（`mower_logic` 的 `gps_wait_time`/`gps_timeout` 之类的判断）搞乱。

## 第 2 步：地图这条线——理解现在的"地图"跟你要做的"地图"不是一个东西

**看什么：**
- `src/mower_logic/src/mower_logic/behaviors/AreaRecordingBehavior.cpp`——现在的"建图"实现，全靠人开车 + 记 GPS 轨迹点。已经在 [architecture.md](architecture.md) 里讲过流程，直接看代码印证一遍。
- `src/mower_map/src/mower_map_service.cpp`——地图的存储和服务层。重点看：
  - `map.json` 的读写（`addMowingArea`/`getMowingArea` 等）——这是多边形数据结构。
  - 大概 500-540 行附近，多边形怎么被**栅格化成 `nav_msgs::OccupancyGrid`**（`mower_map_service/map` 话题），这个格子地图才是喂给导航 costmap 的东西（`global_costmap`/`local_costmap` 的 `static_layer`）。
- 标准 ROS 导航栈的 costmap 插件概念（`costmap_2d`，不是这个仓库的代码，是系统 ROS 包）：`static_layer`（上面说的那份栅格地图）叠加 `obstacle_layer`（实时传感器数据，目前这个项目里基本没有真的障碍物传感器输入）。**`obstacle_layer` 是标准的可扩展点**——真实的避障传感器（激光、深度相机、超声波）通常就是接成一个新的 `obstacle_layer` 插件或者往它订阅的话题发数据，这跟 VSLAM/建图关系不大，但跟"基于视觉的实时避障"关系很大，如果你的任务里包含避障，这是要看的地方。

搞清楚这一层的意义在于：现在这个"地图"是纯拓扑/边界性质的（哪块地能割、充电桩在哪），不是一份可以直接拿来做视觉重定位、回环检测的稠密地图。如果任务是"用 VSLAM 生成一份点云/网格地图"，这份地图很可能是**平行于** `map.json` 存在的新东西，而不是改造 `mower_map` 本身；但如果任务包含"用视觉自动生成割草区域轮廓"，那最终产物还是要落回 `AddMowingAreaSrv` 这个接口，变成 `mower_map` 认识的多边形。

## 第 3 步：决策层——搞清楚新东西该在状态机的哪个阶段介入

**看什么：**
- `src/mower_logic/src/mower_logic/mower_logic.cpp` 的整体结构（状态机怎么切换、`Behavior` 基类怎么工作），不用逐行看，看懂"状态机"这个骨架和 `IdleBehavior`/`MowingBehavior`/`DockingBehavior` 几个具体状态怎么互相跳转。
- `src/mower_logic/src/mower_logic/behaviors/Behavior.h`——每个状态要实现哪些回调（`execute`/`enter`/`exit`/`needs_gps` 等）。**`needs_gps()` 这个接口值得注意**——如果你的 VSLAM 要替代/补充 GPS，这类"当前状态是否需要绝对定位"的判断点就是要跟着改的地方。

## 第 4 步：怎么在仿真里验证一个新的定位/建图源（不需要真相机）

真正接手任务之前，建议自己动手做一次最小验证，而不是等真的有相机数据了才第一次测试集成：

1. 写一个最小的 ROS 节点（哪怕是纯 Python），按固定频率发布 `xbot_msgs/AbsolutePose` 消息到一个新话题（比如 `/vslam/pose`），位置随便给一个跟当前仿真机器人差不多的坐标（可以先读一遍 `/xbot_positioning/xb_pose` 抄个初始值）。
2. 改一下 `_localization.launch`（或者本地临时改）把 `xb_pose_in` 的 remap 从 `/ll/position/gps` 换成你的新话题，或者干脆开两个 `xbot_positioning` 实例做对比。
3. 观察 `/xbot_positioning/xb_pose`、RViz 里的位姿，看你发的假位姿有没有被正确融合进去、协方差/精度字段填错了会有什么后果。
4. 参考 [running-native.md](running-native.md) 的调试一节，需要打断点就用 gdb/IDE 挂到 `xbot_positioning` 进程上，单步看 EKF 更新那一段。

这个练习本身不产出任何真实功能，纯粹是让你在接到具体需求之前，先摸清楚"我的新模块发出去的消息，下游会怎么处理、错了会怎么炸"，比拿到真实任务后一边啃相机 SDK 一边啃这套 EKF 效率高。

## 参考顺序总结

1. [running-native.md](running-native.md) + [getting-started.md](getting-started.md)——跑起来，建立直觉。
2. [architecture.md](architecture.md)——建立模块地图和全局流水线的心智模型。
3. `xbot_positioning`（本文档第 1 步）——现有定位实现，是你未来改动最直接相关的部分。
4. `AreaRecordingBehavior` + `mower_map`（本文档第 2 步）——现有"建图"实现，理清它跟你要做的建图不是一回事。
5. `mower_logic` 状态机骨架（本文档第 3 步）——新模块要在哪个阶段介入。
6. 自己做一次最小的仿真验证（本文档第 4 步）——在接到具体需求前先摸清楚集成点的脾气。

到这一步，接手具体的 VSLAM/建图任务时，至少不会再问"这个项目到底是怎么定位的""地图是从哪来的""我这个新模块应该往哪塞"这几个最基础的问题。
