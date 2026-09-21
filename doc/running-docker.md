# 运行 / 停止仿真栈（Docker 版）

本文档说明如何用 `docker-simulation/` 下的 Docker Compose 栈启动、停止和管理 OpenMower 的机器人仿真。这是最省心的方式：不需要本机装 ROS、不需要 `catkin_make`。

> `roslaunch open_mower open_mower.launch` 是给真实硬件用的，直接跑会因为连不上真实主板而起不来。想快速看一眼仿真效果、不打算改代码调试，用本文说的 Docker 方式；如果要改代码、打断点调试，见 [running-native.md](running-native.md)。

## 前置条件

- 已安装 Docker 和 Docker Compose（`docker --version`、`docker compose version` 能正常输出）。

## 启动

```bash
cd docker-simulation
./sim.sh up
```

- 首次启动会本地构建 `mower_simulation_gui` 镜像（模拟底层主板 + RViz + noVNC），可能要几分钟。
- 之后再执行 `./sim.sh up` 不会重新构建，只是启动/复用已有镜像。

启动后打开：

| 地址 | 内容 |
| --- | --- |
| http://localhost:6080 | 仿真画面（RViz，浏览器内 noVNC，免密码） |
| http://localhost:3000 | OpenMowerApp（新版，带地图编辑器） |
| http://localhost:8080 | OpenMowerApp（旧版 Flutter 版本） |
| localhost:5900 | 原生 VNC 客户端连接地址（不想用浏览器时用） |

## 停止

```bash
./sim.sh down
```

只停止并删除容器，`./data/` 下的地图、参数、日志等文件保留，下次 `./sim.sh up` 还在。

如果想连同 Docker 命名卷一起清理：

```bash
./sim.sh down-volumes
```

## 常用命令

```bash
./sim.sh ps                    # 查看容器状态，重点看 mower_simulation_gui 是否 healthy
./sim.sh logs                  # 跟踪全部容器日志
./sim.sh logs open_mower_ros   # 只看 open_mower_ros 容器日志
./sim.sh restart               # down + up，并重新拉取镜像
./sim.sh rebuild               # 强制重新构建 mower_simulation_gui（改了要打进镜像的代码/配置后用这个）
./sim.sh reset                 # 丢弃 data/ 下的地图和参数改动，恢复成仓库自带的初始地图
./sim.sh clean                 # 危险操作：清空 ./data/ 下的所有内容（日志、录制、地图、参数），会二次确认
./sim.sh shell                 # 进入 open_mower_ros 容器的 bash
./sim.sh help                  # 查看完整命令列表
```

## 修改本地代码后如何生效

`docker-simulation` 默认拉取发布好的镜像，本地 `src/` 下的代码改动**不会**自动同步进容器（源码是构建镜像时 `COPY` 进去的，不是挂载）。要测试本地改动，第一次要手动构建一遍、并把 `.env` 指向本地镜像：

```bash
docker compose --profile build-from-source build build_from_source
```

然后在 `docker-simulation/.env` 里取消注释：

```
BASE_IMAGE=local/open_mower_ros:local
```

**做完这次性设置之后**，`./sim.sh rebuild` 会自动检测到 `BASE_IMAGE=local/open_mower_ros:local`，自己重新跑一遍源码构建、重建 `mower_simulation_gui`、再重启容器——不需要每次都手动再跑一遍 `build build_from_source`。所以后续每次改完代码，只需要一条命令：

```bash
./sim.sh rebuild
```

这个循环比本地 `catkin_make` 慢很多（要走一遍 Docker build），改代码、打断点这种高频调试建议走 [running-native.md](running-native.md)，Docker 仿真只在阶段性验证"这个功能在完整系统里跑得对不对"时用。

## 已知问题：发布镜像里电机不响应（已在本分支源码修复）

截至 `docker-simulation/.env` 里 `VERSION=1.3.0` 这次改动为止，`open_mower_ros` 官方发布的 `edge` / `v1.4.0` 镜像存在一个上游 bug：`mower_comms_v2` 新增的 `MetaService`（固件版本检查）机制，仿真用的 `mower_simulation` 节点没有实现，导致仿真里 `MetaService not connected`，电机被永久禁用，点 Start 没有任何反应。

**本分支已经在源码里修好了这个 bug**（以及另外三个连带发现的问题，见 [running-native.md](running-native.md) 里的完整列表），但官方还没发布带这个修复的镜像。所以：

- **用发布镜像（默认路径，不用本地构建）**：保持 `.env` 里 `VERSION=1.3.0`（该版本早于 `MetaService` bug 引入之前）。改回 `edge` 或最新 tag 会重新踩坑，除非官方已经合并修复。
- **用 `build-from-source`**：因为镜像是从本分支的 `src/` 现场编译的，修复已经在里面，`VERSION` 这时候不生效（`BASE_IMAGE` 优先），不需要再关心版本号。

## 已知问题：点 Start 后机器人开到草坪就停下（配置问题，已修复，不用重新构建镜像）

`data/params/custom_params.yaml` 里 `mower_logic.enable_mower` 之前默认是 `false`——这是给真实硬件用的安全默认值（先确认急停传感器正常，再手动打开割草电机），仿真里没有真实刀片，这个默认值只会让 Start 之后机器人开到草坪却因为电机被硬禁用而卡住。已经把这个改成了 `true`。

这份文件是通过 `docker-compose.yaml` 里的 bind mount（`./data/params:/data/params`）挂进容器的，**不是**打包进镜像里的，所以这个修复不需要 `./sim.sh rebuild`，哪怕你用的是发布镜像（`VERSION=1.3.0`）也会立即生效——`./sim.sh restart` 或者干脆重新 `./sim.sh up` 就够了。如果你之前用 `./sim.sh reset` 恢复过初始数据，记得再跑一遍确认 `data/params/custom_params.yaml` 里这一行是 `true`。

## 已知限制：仿真电池可能一直很低、充不上电

如果发现机器人反复被派回充电桩，很可能是仿真启动时机器人没有精确落在充电桩坐标上（对充电桩位置的查询有时会在 `open_mower_ros` 那一侧服务还没起来之前超时），导致电量一直很低、充不上电，`mower_logic` 判断"不适合割草"就会主动折返。这是仿真本身的已知限制，不是这次改动引入的问题，详见 [running-native.md](running-native.md) 里的说明（包含用 RViz 手动摆位的应对办法，以及 `mower_simulation` 自带的 `sim.dock.move` / `sim.battery.set` 测试接口）。

## 排查问题

```bash
./sim.sh ps                             # 确认 mower_simulation_gui 是 healthy
./sim.sh logs mower_simulation_gui
./sim.sh logs open_mower_ros
```

如果栈已经在跑，但改动没生效，大概率是需要重新构建：`./sim.sh rebuild`（普通 `up` 不会自动重建）。
