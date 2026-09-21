# 怎么用这个项目

这份文档是入口，串一下"跑起来"到"实际操作机器人"的整条链路。仓库本身是 OpenMower 的 ROS 软件栈（详见 [README.md](../README.md)），控制一台真实/仿真的自主割草机器人：定位、导航、覆盖路径规划、割草逻辑都在这里。

## 第一步：跑一个仿真起来

不用真实硬件也能跑完整个软件栈，机器人的底层板卡（驱动、割草电机、IMU、电源、GPS）由 `mower_simulation` 节点模拟。两条路线选一个：

- **[Docker 版](running-docker.md)**——`cd docker-simulation && ./sim.sh up`。不用装 ROS，几条命令就能跑起来，还带一个网页 App 界面。只是想看看效果、不打算改代码，选这个。
- **[本地编译版](running-native.md)**——要装 ROS Noetic、`catkin_make` 编译。麻烦一点，但改代码后验证快得多，还能直接上 gdb/IDE 打断点调试。日常开发调试选这个。

两条路线跑起来之后,机器人状态是一样的:一个在 `IDLE`(待命)状态的仿真机器人。它*应该*是停在充电桩里正在充电,但仿真启动时机器人对充电桩具体位置的查询有时会超时,导致它其实落在地图原点而没有真正贴合充电桩、电量偏低——这是已知限制,不是你操作出的问题,见下面"遇到问题"里的说明。

## 第二步：让机器人动起来

跑起来之后,怎么让它开始割草、看它的状态、让它回充,两条路线操作方式不一样:

### Docker 版：网页 App

- http://localhost:3000（新版,带地图编辑器）
- http://localhost:8080（旧版）

在网页里点 Start / Dock 之类的按钮就行,不需要碰命令行。

### 本地编译版：没有网页 App，用命令行

本地这条路线没有网页 App(那两个是 Docker 专属容器,`sim_mower_logic.launch` 没启动它们)。可视化只有 RViz(+ 一个 `rqt_reconfigure` 小窗口),控制机器人靠一个 ROS service:

```bash
# 先确认服务在(排查用,不是必须)
rosservice list | grep high_level_control
```

| 想做什么 | 命令 |
| --- | --- |
| 开始割草（从 IDLE 状态） | `rosservice call /mower_service/high_level_control "command: 1"` |
| 回充 / 中止当前任务 | `rosservice call /mower_service/high_level_control "command: 2"` |
| 清除急停（Emergency） | `rosservice call /mower_service/high_level_control "command: 254"` |

看当前状态（`state_name` 会在 `IDLE` / `AUTONOMOUS` 等之间切换）：

```bash
rostopic echo /mower_logic/current_state
```

发出开始割草的命令后,回头看 RViz 窗口,机器人应该会沿着规划好的覆盖路径开始移动。

## 遇到问题时看哪个文档

- 仿真起不来、`roslaunch`/`docker compose` 报错、环境变量踩坑 → [running-docker.md](running-docker.md) 或 [running-native.md](running-native.md) 里各自的"排查问题"部分。
- 点了 Start，机器人开到草坪就卡住/又被派回充电桩 → 两份文档里都有对应的"已知问题/已知限制"小节：一个是 `enable_mower` 配置默认值的问题（已修复），一个是仿真电池电量偏低、充不上电的已知限制（有应对办法）。
- 要改代码、打断点、看日志 → [running-native.md](running-native.md) 的"调试代码"一节（这些只在本地编译版下才好用，Docker 发布镜像通常没有调试符号）。
