# 运行 / 调试仿真（本地编译版，不用 Docker）

本文档说明如何在本机直接编译、运行仿真，以及怎么调试代码。这条路线比 [Docker 版](running-docker.md)麻烦一些（要装 ROS、要自己配环境变量），但换来的是快得多的"改代码 → 验证"循环，以及能直接上 gdb/IDE 打断点——适合日常开发调试；只是想看一眼效果、不打算碰代码，用 Docker 版更省心。

> 本分支已经修好了几个会导致仿真"点 Start 没反应/开到草坪就停下"的问题（三个源码 bug + 一个配置默认值，见文末），按本文步骤跑应该能直接工作。如果你是在更早的 commit 上跑遇到问题，先确认已经拉到这几个修复。

## 前置条件

- ROS Noetic（`roscore`、`roslaunch`、`catkin_make` 能跑）。
- 拉好依赖、编译过一次工作区：

```bash
sudo rosdep init      # 第一次用 rosdep 才需要
rosdep update
git submodule update --init --recursive
rosdep install --from-paths src --ignore-src --default-yes
catkin_make
```

## 环境变量：先排查 shell 里已有的 ROS 配置

如果你这台机器之前配过别的 ROS 项目/真实机器人，`~/.bashrc` 或 `~/.zshrc` 里很可能已经写了 `ROS_MASTER_URI` 指向别的地址（比如某个远程机器人的 IP）。跑本地仿真前检查一下：

```bash
echo $ROS_MASTER_URI
```

如果不是 `http://localhost:11311`，下面的启动脚本会显式覆盖它，但如果你自己手动跑命令，记得也覆盖，否则 `roslaunch` 会尝试连到别的机器上的 ROS Master，看起来像"卡住不动"。

## 启动

先 `cd` 到仓库根目录，然后**按顺序**依次执行下面这几条命令（都是普通的终端命令，不是脚本；`.native-sim/` 只需要建一次，`source`/`export` 那几行只在当前终端窗口生效，关掉窗口或开新窗口要重新执行一遍）：

```bash
cd /path/to/open_mower_ros   # 换成你自己的仓库路径

# 1. 准备一份仿真用的地图和参数（只需要执行一次）。
#    sim_mower_logic.launch 用的是新版（V2）参数体系，得靠环境变量告诉它去哪读配置，
#    这里复用仓库里 docker-simulation/data/ 下已经配好的起始地图和参数
#    （有割草区域、有充电桩位置，不是空地图）。用独立目录（这里叫 .native-sim/，
#    不要提交到 git）是为了不让本地跑仿真产生的日志、位置历史污染
#    docker-simulation/data/ 里那份提交进仓库的起始数据。
mkdir -p .native-sim/ros .native-sim/params
cp docker-simulation/data/ros/map.json .native-sim/ros/
cp docker-simulation/data/params/custom_params.yaml .native-sim/params/

# 2. 加载系统 ROS（给你 roslaunch、rviz、joy 这些标准 ROS 包）
source /opt/ros/noetic/setup.bash

# 3. 加载这个仓库自己编译出来的包（open_mower、mower_logic 等，系统 ROS 里没有）
source devel/setup.bash   # zsh 用户用 devel/setup.zsh

# 4. 覆盖 ROS Master 地址，避免连到你机器上其他项目配置的远程地址
export ROS_MASTER_URI=http://localhost:11311
unset ROS_HOSTNAME

# 5. sim_mower_logic.launch 读配置用的几个环境变量
export MOWER=CUSTOM
export PARAMS_PATH="$(pwd)/.native-sim/params"
export ROS_HOME="$(pwd)/.native-sim/ros"

# 6. 启动
roslaunch open_mower sim_mower_logic.launch
```

下次再跑，第 1 步（`.native-sim/` 已经建好）可以跳过，从第 2 步开始就行。

正常情况下会弹出 RViz 窗口（需要一个可用的 `DISPLAY`；纯远程/无图形环境的话在第 6 步前加一句 `export OM_START_RVIZ=False` 跳过 RViz）。用下面命令确认状态健康：

```bash
rostopic echo -n1 /ll/emergency          # active_emergency 应该是 False，reason 是空字符串
rostopic echo -n1 /mower_logic/current_state   # state_name 应该是 "IDLE"
```

## 停止

`Ctrl+C` 发 `SIGINT` 给 `roslaunch` 进程即可，它会把拉起来的所有节点一起关掉。

## 改代码后如何生效

`catkin_make` 编译产物在 `devel/`，`roslaunch` 用的就是这份产物，中间没有 Docker 镜像层这道"隔离墙"：

```bash
catkin_make                     # 增量编译，只重编改过的包
# 回到跑着 roslaunch 的那个终端窗口，Ctrl+C 停掉，
# 上面第 2-5 步的 source/export 还在这个窗口里生效，直接重新执行第 6 步 roslaunch 即可
```

**已经在跑的进程不会自动换成新编译出来的二进制**，必须重启节点/整个 launch 才会生效。

## 调试代码

### 先用日志

- 节点默认 `output="screen"`，终端直接能看到日志。
- 代码里用 `ROS_DEBUG(...)` 打印更细的信息，运行前设置：
  ```bash
  export ROSCONSOLE_MIN_SEVERITY=DEBUG
  ```
- 图形化查看/过滤日志：`rqt_console`。
- 看话题数据流、节点关系，很多时候比打断点更快定位问题：
  ```bash
  rqt_graph                  # 节点/话题连接图
  rostopic echo /ll/cmd_vel   # 看某个话题的实时数据
  rostopic hz /ll/cmd_vel     # 看发布频率
  rosservice call ...         # 手动调用服务测试
  ```

### gdb 打断点

先保证编译带调试符号：

```bash
catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo   # 或 Debug（完全不优化，符号最全，跑起来更慢）
```

单独调某一个节点（推荐，其余节点照常跑）：

```bash
rosnode kill /mower_logic
rosrun --prefix 'gdb -ex run --args' mower_logic mower_logic
```

或者让 roslaunch 直接用 gdb 拉起某个节点，在对应 `.launch` 文件里给那个 `<node>` 加：

```xml
<node pkg="mower_logic" type="mower_logic" name="mower_logic"
      launch-prefix="xterm -e gdb -ex run --args" />
```

改完记得把 `launch-prefix` 去掉，不然仿真/上线也会带着它跑。

### IDE 调试

- **CLion**：按 [README.md](../README.md#how-to-build-using-clion-ide) 里的说明，`source devel/setup.bash` 后启动 CLion，打开 `src` 目录。之后可以直接对某个可执行文件配 Run/Debug Configuration 打断点，或者用 "Attach to Process" 挂到已经在跑的节点进程上（`ps aux | grep <节点名>` 先找到 PID）。
- **VSCode**：装 C/C++ 扩展，编译时加 `-DCMAKE_EXPORT_COMPILE_COMMANDS=1` 生成 `compile_commands.json` 获得准确跳转/补全；断点调试配一个 `"request": "attach"` 的 launch config 挂到节点 PID 上。

## 本分支修复过的问题

跑本地仿真时如果遇到"点 Start 没反应"或卡在某个状态，很可能是下面这几个问题（前三个是仓库本身的 bug，本分支已修复；第四个是配置默认值，也已经改掉）：

1. **`mower_simulation` 没实现 `MetaService`**——`mower_comms_v2` 新增的固件版本检查机制，仿真板卡没有回应，电机永久禁用（日志：`MetaService not connected ... Motors disabled`）。修复：`src/mower_simulation/src/services/meta_service/`。
2. **`sim_mower_logic.launch` 缺少定位节点**——没有 include `_localization.launch`，`xbot_positioning` 不会启动，`mower_logic` 卡在"等 GPS 服务"永远到不了 `IDLE`。
3. **`twist_mux_topics.yaml` 缺 `locks` 字段**——较新版本的 `ros-noetic-twist-mux`（apt 装的，跟 Docker 镜像里打包的版本可能不一致）强制要求这个字段存在，没有就直接 FATAL 崩溃，速度指令传不到底盘。修复：加了个空的 `locks: []`。
4. **`custom_params.yaml` 里 `mower_logic.enable_mower` 默认是 `false`**——这是给真实硬件用的安全默认值（先确认急停传感器正常，再手动打开割草电机），但仿真里没有真实刀片，这个默认值只会让"点 Start"的机器人开到草坪之后因为电机被硬禁用而卡住（可能表现为 `MOWER_RPM_TIMEOUT` 急停，也可能表现为悄无声息地回到 `IDLE`）。修复：`docker-simulation/data/params/custom_params.yaml` 里改成了 `true`。如果你在这个改动之前就已经跑起来了仿真，改完文件要重新走一遍第 1 步（重新 `cp` 一份到 `.native-sim/params/`）再重启，或者直接在当前会话里 `rosparam set /mower_logic/enable_mower true` 临时生效。

## 已知限制：仿真电池可能一直很低、充不上电

跑起来之后如果发现机器人反复被派回充电桩（`current_state` 是 `DOCKING`），查一下 `rostopic echo -n1 /ll/power` 里的 `battery_voltage`——如果它一直卡在 `battery_empty_voltage`（`custom_params.yaml` 里配的空电阈值）附近、`is_charging` 也一直是 `False`，说明电量太低，`mower_logic` 判断"不适合割草"就会主动折返，这不是本分支引入的 bug,是仿真本身的已知限制：

- 仿真启动时,`mower_simulation` 对"充电桩具体位置"的查询有时会在 `open_mower_ros` 那一侧的服务还没起来之前就超时,导致机器人实际生成在地图原点而不是真正贴合充电桩的位置,不在桩上自然充不了电（[docker-simulation/README.md](../docker-simulation/README.md) 里也提到过这一点）。
- `mower_simulation` 本身有一套通过 MQTT RPC 直接摆位/改电量的测试接口（`sim.dock.move`、`sim.battery.set`，见 `src/mower_simulation/src/SimRpc.cpp`），但这套接口挂在 MQTT 上,本地这条路线没有起 mosquitto,直接用不了。
- 简单的应对办法：在 RViz 里用 "2D Pose Estimate" 工具把机器人重新摆到充电桩位置上，等它显示 `is_charging: True` 之后再等一会儿把电充起来，再重新 Start。
