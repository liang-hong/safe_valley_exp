# Safe Valley 实验功能包 (safe_valley_exp)

这是一个基于模块化设计的 ROS 集群无人机控制框架，具备分布式通信、时空一致性同步以及高精度集群控制算法。

## 项目概述
本功能包旨在实现多机部署的“零配置”迁移。通过将核心逻辑拆分为四个独立模块，实现了控制状态机、参数加载、底层通信与数学算法的彻底解耦，极大方便了后续算法的迭代与实机部署。

## 模块化架构
程序由以下四个核心模块组成：
- **`safe_flock_main.py`**: 主入口与状态机。负责管理 `hover` (悬停)、`form` (编队) 和 `navi` (导航/集群) 模式的切换逻辑。
- **`flock_config.py`**: 身份识别与参数中心。支持通过 `roslaunch`、主机名自动识别本机 ID，并从 YAML 加载所有算法增益。
- **`flock_comm.py`**: 通信管理模块。实现了基于 GPS 授时 (`TimeReference`) 的系统时钟偏差校准，以及基于 Leader 广播的全球原点同步。
- **`flock_math.py`**: 数学算法库。封装了高精度 Reynolds 集群算法（凝聚、对齐、分离）以及带“侧向逃逸”逻辑的动态椭圆势场避障模型。

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

## 运行方式
### 实机部署
```bash
roslaunch safe_valley_exp safe_flock.launch own_name:=UAV1
```
### 仿真测试 (多 Master 模式)
配合 `swarm_topology_bridge` 功能包，支持单机隔离运行多台无人机节点：
```bash
roslaunch safe_valley_exp test_sim_swarm.launch
```

## 算法扩展指南
若需实现新算法：
1. 在 `flock.yaml` 中添加所需参数。
2. 在 `flock_config.py` 中补充参数加载代码。
3. 在 `flock_math.py` 中编写新的数学逻辑函数。
4. 在 `safe_flock_main.py` 的执行循环中调用新函数并发布指令。
