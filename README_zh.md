# Safe Valley 实验功能包 (safe_valley_exp)

中文版 | [English](README.md)

这是一个基于模块化设计的 ROS 集群无人机控制框架，具备分布式通信、时空一致性同步以及高精度集群控制算法。

## 项目概述
本功能包旨在实现多机部署的“零配置”迁移。通过将核心逻辑拆分为四个独立模块，实现了控制状态机、参数加载、底层通信与数学算法的彻底解耦，极大方便了后续算法的迭代与实机部署。

## 模块化架构
四个核心模块：
- **`safe_flock_main.py`**: 主入口与状态机。负责管理 `hover` (悬停)、`form` (编队) 和 `navi` (导航/集群) 模式的切换逻辑。
- **`flock_config.py`**: 身份识别与参数中心。支持通过 `roslaunch`、主机名自动识别本机 ID，并从 YAML 加载所有算法增益。
- **`flock_comm.py`**: 通信管理模块。实现了基于 GPS 授时 (`TimeReference`) 的系统时钟偏差校准，以及基于 Leader 广播的全球原点同步。
- **`flock_method.py`**: 方法库。封装了高精度 Reynolds 集群算法（凝聚、对齐、分离）以及带“侧向逃逸”逻辑的动态椭圆势场避障模型。

若干辅助模块：
- **`submode_publisher.py`**: 子模式发布模块。负责将当前子模式（如 `hover`、`form`、`navi`）发布到 `/submode` 主题，供其他节点订阅。
- **`wait_mavros.py`**: 等待 MAVROS 连接模块。负责在主节点启动前，等待 MAVROS 连接，确保所有无人机都能正常通信。

## 关键技术特性
- **时间一致性**：通过计算系统时钟与 GPS 时间的 Bias，实时修正他机 Odom 时间戳，解决跨机分布式系统的时间抖动问题。
- **空间对齐**：Leader 自动获取全球定位并广播，Follower 同步设置 `set_gp_origin`，确保集群在统一的 ENU 坐标系下运行。
- **垂直投影避碰**：在集群跟随过程中，将 Leader 视为垂直方向的虚拟轴（投影），从物理逻辑上规避了从机与领航者在高度层上的碰撞。
- **动态椭圆避障**：离心率随速度动态变化的椭圆模型，包含“侧向避让”逻辑，有效解决速度与相对位置共线时的避障死锁。

## 安装与编译
兼容 ROS Melodic/Noetic 环境。
```bash
# 推荐使用 catkin build
catkin build safe_valley_exp
# 或者使用 catkin_make
catkin_make --pkg safe_valley_exp
```

## 配置说明
所有算法参数均在 `config/flock.yaml` 中定义：
- `control`: 存储安全半径、最大速/加速度及各类算法增益。
- `leader`: 设置 Leader 名称、轨迹参数（圆心、半径、速度）及 RC 通道映射。
- `topology`: 定义集群的邻居拓扑关系（用于对齐算法）。

## 运行方式 (仿真环境)

本功能包支持**多 ROS Master 隔离仿真**，以模拟“每机一套机载 ROS Master”的实机部署形态，并用 QGC 同时遥测多机。

仿真建议采用“**仿真层 + 机载层**”两层结构（最少命令，最少耦合）：
- **仿真层（1 个 Master）**：仅负责 Gazebo+ 多个 PX4 SITL 实例（多机）。不跑算法。
- **机载层（N 个 Master）**：每台无人机一个独立 ROS Master，只跑 MAVROS + `swarm_topology_bridge` + `safe_valley_exp`。通过 `fcu_url` 用 UDP 连接到仿真层对应 PX4 实例。


### 2. 启动仿真层（一次即可，无界面）
**终端 A（仿真 Master：11300）**
```bash
export ROS_MASTER_URI=http://localhost:11300
export ROS_HOSTNAME=localhost
export GAZEBO_MASTER_URI=http://localhost:11345
roslaunch safe_valley_exp multi_uav_sim.launch
```

### 3. 启动机载层（每机一个 Master）
端口与系统号按 ID 偏移：
- UAV6 (ID=0)：`fcu_url=udp://:24540@localhost:34580`，`tgt_system=1`
- UAV7 (ID=1)：`fcu_url=udp://:24541@localhost:34581`，`tgt_system=2`

**终端B1 UAV6（机载 Master：11311）**
```bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
roslaunch safe_valley_exp uav_offboard_sim.launch uav_name:=UAV6 tgt_system:=1
```

**终端B2 UAV7（机载 Master：11312）**
```bash
export ROS_MASTER_URI=http://localhost:11312
export ROS_HOSTNAME=localhost
roslaunch safe_valley_exp uav_offboard_sim.launch uav_name:=UAV7 tgt_system:=2
```

**终端B3 UAV8（机载 Master：11313）**
```bash
export ROS_MASTER_URI=http://localhost:11313
export ROS_HOSTNAME=localhost
roslaunch safe_valley_exp uav_offboard_sim.launch uav_name:=UAV8 tgt_system:=3
```

**终端B4 UAV9（机载 Master：11314）**
```bash
export ROS_MASTER_URI=http://localhost:11314
export ROS_HOSTNAME=localhost
roslaunch safe_valley_exp uav_offboard_sim.launch uav_name:=UAV9 tgt_system:=4
```

### 4. QGC 遥测要点
QGC 看到“第二台标签但没有位置/图标”，通常表示心跳已通但该 PX4 实例没有接入仿真器产生位置数据。优先检查：
- 对应 PX4 实例是否通过 `single_vehicle_spawn_xtd.launch` 成功启动并生成模型
- 对应 MAVROS 的 `/mavros/state` 是否 `connected: True`

---

## 运行方式 (实机部署)
在真实的机载电脑上，首先通过串口连接到飞控Telem端口，接着调试mavros功能包的px4.launch文件参数完成连接，然后运行以下launch指令，会自动启动mavros节点和算法程序：
```bash
# 程序会自动识别 hostname (如主机名为 UAV6，则自动以 UAV6 身份运行)
roslaunch safe_valley_exp uav_offboard_real.launch
```

---
若需实现新算法：
1. 在 `flock.yaml` 中添加所需参数。
2. 在 `flock_config.py` 中补充参数加载代码。
3. 在 `flock_method.py` 中编写新的数学逻辑函数。
4. 在 `safe_flock_main.py` 的执行循环中调用新函数并发布指令。
5. 在 `submode_publisher.py` 中添加对新算法的模拟输入逻辑。
