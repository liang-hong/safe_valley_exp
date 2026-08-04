# Safe Valley Experiment Package (safe_valley_exp)

English | [中文版](README_zh.md)

This is a modular ROS swarm-drone control framework with distributed communication, spatiotemporal consistency synchronization, and high-precision flocking algorithms.

## Project Overview
This package aims for “zero-configuration” migration for multi-UAV deployment. By splitting the core logic into four independent modules, it fully decouples the control state machine, parameter loading, communication layer, and math algorithms, which makes iteration and real-world deployment easier.

## Modular Architecture
Four core modules:
- **`safe_flock_main.py`**: Entry point and state machine. Manages transitions among `hover` (hover), `form` (formation), and `navi` (navigation/flocking) modes.
- **`flock_config.py`**: Identity resolution and parameter center. Resolves the local UAV ID via `roslaunch`/hostname, and loads all gains from YAML.
- **`flock_comm.py`**: Communication manager. Implements GPS `TimeReference`-based clock bias correction and Leader-led global origin synchronization.
- **`flock_method.py`**: Method library. Includes high-precision Reynolds flocking (cohesion, alignment, separation) and a dynamic elliptical potential-field obstacle avoidance model with “side nudge”.

Auxiliary modules:
- **`submode_publisher.py`**: Submode publisher. Publishes the current submode (e.g. `hover`, `form`, `navi`) to the `/offb_submode` topic.
- **`wait_mavros.py`**: MAVROS readiness waiter. Waits for MAVROS connection before the main node starts, ensuring all UAVs can communicate normally.

## Program Relationships

```text
multi_uav_sim.launch    # Copy this file into px4/launch/ before use
├── include → gazebo_ros/launch/empty_world.launch  # Load Gazebo world
└── include → px4/launch/single_vehicle_spawn_xtd.launch  # Spawn 4 UAV instances
    ├── group ns=iris_0 (ID=0, mavlink_tcp_port=4560, udp_gimbal_port=13030)
    ├── group ns=iris_1 (ID=1, mavlink_tcp_port=4561, udp_gimbal_port=13031)
    ├── group ns=iris_2 (ID=2, mavlink_tcp_port=4562, udp_gimbal_port=13032)
    └── group ns=iris_3 (ID=3, mavlink_tcp_port=4563, udp_gimbal_port=13033)

uav_offboard_sim.launch
├── include → mavros/launch/px4.launch          # Start MAVROS
└── include → safe_flock_sim.launch
    ├── node safe_flock: wait_mavros.py         # Wait for MAVROS connection
    │   └── execv → safe_flock_main.py          # Main algorithm program
    ├── node submode_publisher: submode_publisher.py  # Submode button simulate
    ├── rospackage: swarm_topology_bridge       # Communication package
    │   └── node swarm_bridge: bridge_node.py
    └── node rosbag_record: rosbag_record.py    # Data recording

uav_offboard_real.launch
├── include → mavros/launch/px4.launch          # Start MAVROS
└── include → safe_flock_real.launch
    ├── node safe_flock: wait_mavros.py         # Wait for MAVROS connection
    │   └── execv → safe_flock_main.py          # Main algorithm program
    ├── rospackage: swarm_topology_bridge       # Communication package
    │   └── node swarm_bridge: bridge_node.py
    └── node rosbag_record: rosbag_record.py    # Data recording
```

## Key Technical Features
- **Time consistency**: Computes the bias between system clock and GPS time to correct other-UAV odom timestamps and reduce distributed time jitter.
- **Space alignment**: The Leader acquires global position and broadcasts it; Followers set `set_gp_origin` accordingly to ensure a shared ENU frame.
- **Vertical projection collision avoidance**: Treats the Leader as a virtual vertical axis (projection) during flocking to avoid altitude-layer collisions.
- **Dynamic elliptical obstacle avoidance**: Uses a speed-dependent eccentricity ellipse with “side nudge” logic to break collinear deadlocks.

## Installation & Build
Compatible with ROS Melodic/Noetic.
```bash
# Recommended: catkin build
catkin build safe_valley_exp
# Or: catkin_make
catkin_make --pkg safe_valley_exp
```

## Configuration
All algorithm parameters are defined in `config/flock.yaml`:
- `control`: safety radius, max speed/accel, and other gains.
- `leader`: Leader name, trajectory params (center/radius/speed), and RC channel mapping.
- `topology`: swarm neighborhood topology (used by alignment).

## Usage (Simulation)
This package supports **multi ROS master isolated simulation** to mimic “one onboard ROS master per UAV” deployment, and uses QGC to view telemetry for multiple UAVs simultaneously.

We recommend a two-layer structure for simulation (**simulation layer + onboard layer**) to minimize commands and coupling:
- **Simulation layer (1 master)**: runs Gazebo + multiple PX4 SITL instances (multi-UAV). Does not run the algorithm.
- **Onboard layer (N masters)**: one ROS master per UAV, runs only MAVROS + `swarm_topology_bridge` + `safe_valley_exp`. Connects to the simulation-layer PX4 instance via UDP `fcu_url`.

### 1. Start the simulation UI

Copy `multi_uav_sim.launch` into the `launch` directory of the `px4` package, then start the simulation UI.

**Terminal A (simulation master: 11300)**
```bash
# Copy multi_uav_sim.launch
roscd px4/launch
cp ~/catkin_ws/src/safe_valley_exp/launch/multi_uav_sim.launch .

export ROS_MASTER_URI=http://localhost:11300
export ROS_HOSTNAME=localhost
export GAZEBO_MASTER_URI=http://localhost:11345
roslaunch px4 multi_uav_sim.launch
```

### 2. Start the onboard programs

`source` the `devel/setup.bash` of the workspace that contains `safe_valley_exp`, then start the onboard programs.
Ports and system IDs are offset by ID:
- UAV6 (ID=0): `fcu_url=udp://:24540@localhost:34580`, `tgt_system=1`
- UAV7 (ID=1): `fcu_url=udp://:24541@localhost:34581`, `tgt_system=2`

**Terminal B1 UAV6 (onboard master: 11311)**
```bash
# source the workspace, adjust the path to your environment
source ~/catkin_ws/devel/setup.bash
# run offboard program
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
roslaunch safe_valley_exp uav_offboard_sim.launch uav_name:=UAV6 tgt_system:=1
```

**Terminal B2 UAV7 (onboard master: 11312)**
```bash
# source the workspace, adjust the path to your environment
source ~/catkin_ws/devel/setup.bash
# run offboard program
export ROS_MASTER_URI=http://localhost:11312
export ROS_HOSTNAME=localhost
roslaunch safe_valley_exp uav_offboard_sim.launch uav_name:=UAV7 tgt_system:=2
```

**Terminal B3 UAV8 (onboard master: 11313)**
```bash
# source the workspace, adjust the path to your environment
source ~/catkin_ws/devel/setup.bash
# run offboard program
export ROS_MASTER_URI=http://localhost:11313
export ROS_HOSTNAME=localhost
roslaunch safe_valley_exp uav_offboard_sim.launch uav_name:=UAV8 tgt_system:=3
```

**Terminal B4 UAV9 (onboard master: 11314)**
```bash
# source the workspace, adjust the path to your environment
source ~/catkin_ws/devel/setup.bash
# run offboard program
export ROS_MASTER_URI=http://localhost:11314
export ROS_HOSTNAME=localhost
roslaunch safe_valley_exp uav_offboard_sim.launch uav_name:=UAV9 tgt_system:=4
```


### Rosbag Recording
`uav_offboard_sim.launch` and `uav_offboard_real.launch` enable rosbag recording by default and write bags to `~/rosbagrec` (the directory is auto-created if missing).

Topics recorded:
- `/mavros/setpoint_velocity/cmd_vel`
- `/mavros/setpoint_position/local`
- `/mavros/global_position/set_gp_origin`
- `/mavros/state`
- `/mavros/local_position/odom`
- `/offb_submode`

Disable or change output directory:
```bash
# Disable recording
roslaunch safe_valley_exp uav_offboard_sim.launch uav_name:=UAV6 tgt_system:=1 enable_rosbag:=false

# Change output directory
roslaunch safe_valley_exp uav_offboard_sim.launch uav_name:=UAV6 tgt_system:=1 rosbag_dir:=/home/ub20tg/rosbagrec
```

### 4. QGC Telemetry Notes
If QGC shows “a second vehicle tab but no position/icon”, it typically means the heartbeat is connected but the PX4 instance is not receiving simulation-generated position data. Check:
- whether the PX4 instance was started and its model was spawned via `single_vehicle_spawn_xtd.launch`
- whether `/mavros/state` reports `connected: True`

---

## Usage (Real-world Deployment)
On the real onboard computer, first connect to the FCU Telem port via a serial link, then configure the MAVROS `px4.launch` parameters to establish the connection. After that, run the launch file below, which starts both MAVROS and the control algorithm:
```bash
# source the workspace, adjust the path to your environment
source ~/catkin_ws/devel/setup.bash
# run offboard program. The program auto-detects hostname (e.g. UAV6) and runs with that identity
roslaunch safe_valley_exp uav_offboard_real.launch
```

---

To add a new algorithm:
1. Add the required parameters to `flock.yaml`.
2. Extend parameter loading in `flock_config.py`.
3. Implement new math logic in `flock_method.py`.
4. Call the new function in the main loop of `safe_flock_main.py` and publish the command.
5. Extend simulated input logic for the new algorithm in `submode_publisher.py`.
