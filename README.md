# safe_valley_exp

当前用途（implementation_plan_26082916 §6 已清理 flock legacy 示例）：

- **GPS 时钟偏置节点**（`gps_bias_node.py`）：订阅 `/mavros/time_reference`（GPS PPS），
  滑动窗口均值估计本机 ROS 时钟与 GPS 绝对时间偏差，以固定频率发布 `gps_bias`（latch）。
  邻机经 topology bridge 转发 bias，接收端用 `T_own = T_other - bias_other + bias_own`
  修正机间时间戳（planner intent 时间对齐）。
- **机载 offboard EGO 启动**（`uav_offboard_ego.launch` + `startup_offboard_ego.sh`）：
  MAVROS + `uav_executor_ego.launch` + Group A topology bridge + GPS bias 节点。
- **15-UAV SITL 仿真启动**（`multi_uav_ego_15sim.launch` / `multi_uav_ego_4sim.launch` /
  `multi_uav_sim*.launch`）：Gazebo 世界 spawn iris_0..iris_14。
- **起飞/MAVROS 等待**：`offboard_takeoff_15.py`、`wait_mavros.py`。

## 目录结构

```
safe_valley_exp/
├── config/gps_bias_defaults.yaml   # GPS bias 参数唯一来源（window_s/publish_rate_hz/lockout_s）
├── launch/uav_offboard_ego.launch  # 机载 offboard EGO 启动（MAVROS+EGO+bridge+bias）
├── launch/multi_uav_ego_*.launch   # 15/4 机 EGO 仿真（Gazebo spawn）
├── launch/multi_uav_sim*.launch    # 通用多机仿真
├── scripts/gps_bias_node.py        # GPS 时钟偏置估计/发布节点
├── scripts/offboard_takeoff_15.py  # 15 机 arm/OFFBOARD 与 PX4 参数设置
├── scripts/wait_mavros.py          # MAVROS 连接等待
├── scripts/submode_publisher.py    # 子模式发布（测试辅助）
├── startup_offboard_ego.sh         # 机载层启动脚本（UAV1..UAV15 -> 11311..11325）
└── test/                           # gps_bias 单测 + wiring + rostest
```

## GPS bias 参数

参数唯一来源 `config/gps_bias_defaults.yaml`（`gps_bias_node` 私有 namespace）：

| 参数 | 默认 | 说明 |
|---|---|---|
| `window_s` | 10.0 | 滑动均值窗口（秒） |
| `publish_rate_hz` | 1.0 | 发布频率（Hz） |
| `lockout_s` | 3.0 | GPS 失锁判定阈值（秒） |

校验：所有 double finite；`window_s`/`publish_rate_hz` >0、`lockout_s` >=0；
非法配置节点 fail-fast。

## 启动

```bash
# 机载层（每机一个 Master）
export ROS_HOME=/home/ub20tg/catkin_swarm6-2/.ros_home
export ROS_LOG_DIR=/home/ub20tg/catkin_swarm6-2/.ros_home/log
bash startup_offboard_ego.sh
```

15 机 SITL 仿真世界（终端 A，Master 11300）：

```bash
roslaunch safe_valley_exp multi_uav_ego_15sim.launch
```

## 测试

```bash
catkin run_tests safe_valley_exp
```

- `test/test_gps_bias.py`：`BiasEstimator` 纯逻辑单测（无需 ROS）。
- `test/test_gps_bias_wiring.py`：launch 加载 canonical YAML + 脚本权限静态检查。
- `test/gps_bias_no_reference.test`：rostest（无参考时钟失锁行为）。
