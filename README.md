# Safe Valley Experiment (safe_valley_exp)

[中文版](README_zh.md) | English

A modularized ROS framework for swarm drone control, featuring decentralized communication, time/origin synchronization, and high-fidelity flocking algorithms.

## Project Overview
This package is designed for "zero-configuration" deployment across multiple drones. It decouples the core logic into four distinct modules, allowing for easy algorithm migration and testing in both simulation (SITL) and real-world experiments.

## Modular Architecture
The codebase is structured into four main components:
- **`safe_flock_main.py`**: The entry point and master state machine. It manages transitions between `hover`, `formation`, and `navigation` modes.
- **`flock_config.py`**: Handles identity resolution (UAV name) and parameter loading from YAML. It ensures high priority for `roslaunch` parameters to support multi-master simulation.
- **`flock_comm.py`**: Manages all ROS subscribers and publishers. Implements GPS-based clock bias correction and Leader-led global origin synchronization.
- **`flock_math.py`**: The algorithm core. Contains high-fidelity Reynolds flocking (Cohesion, Alignment, Separation) and an elliptical potential field model for dynamic obstacle avoidance.

## Key Technical Features
- **Time Consistency**: Automatically corrects system clock jitter by calculating the bias between ROS System Time and GPS `TimeReference`.
- **Space Alignment**: Synchronizes the local ENU origin across the swarm via the Leader's broadcasted GPS fix.
- **Safety-First Flocking**: Uses a vertical projection method for Leader following to prevent physical collisions between followers and the leader.
- **Elliptical Obstacle Avoidance**: A dynamic eccentricity model that adjusts the safety buffer based on velocity, including a "Side Nudge" logic to break collinear deadlocks.

## Installation & Build
This package is compatible with ROS Melodic/Noetic.
```bash
# Recommended build tool
catkin build safe_valley_exp
# Or use catkin_make
catkin_make --pkg safe_valley_exp
```

## Configuration
All algorithm gains and swarm topologies are defined in `config/flock.yaml`.
- `control`: Algorithm constants (r_safe, v_max, gains).
- `leader`: Leader identity, trajectory parameters, and RC channel mappings.
- `topology`: Defines the neighborhood relationship for the alignment algorithm.

## Usage (Simulation)

This package supports **Multi-ROS Master Isolated Simulation**, which accurately mimics real-world deployment where each drone has its own onboard computer. Communication between masters is handled by `swarm_topology_bridge` (ZeroMQ).

### 1. Environment Setup (Execute in Every Terminal)
Ensure ROS and PX4 paths are loaded before starting any simulation terminal:
```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
# Configure PX4 path (adjust according to your setup)
export PX4_DIR=~/PX4_Firmware
source $PX4_DIR/Tools/setup_gazebo.bash $PX4_DIR $PX4_DIR/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$PX4_DIR:$PX4_DIR/Tools/sitl_gazebo
```

### 2. Start UAV6 (Leader)
**Terminal 1: Simulation Core (PX4 + MAVROS + Gazebo)**
```bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
# Uses ID 0, corresponding to ports 14540 (Send) / 14580 (Receive)
roslaunch px4 mavros_posix_sitl.launch x:=0 y:=0 z:=0.5 fcu_url:=udp://:14540@localhost:14580 gui:=true interactive:=true
```
*Wait for PX4 to display `EKF alignment complete` and the Gazebo window to appear.*

**Terminal 2: Algorithm & Bridge**
```bash
export ROS_MASTER_URI=http://localhost:11311
roslaunch safe_valley_exp safe_flock_sim.launch uav_name:=UAV6
```

### 3. Start UAV7 (Follower)
**Terminal 3: Follower Simulation Core**
```bash
export ROS_MASTER_URI=http://localhost:11312
export ROS_HOSTNAME=localhost
export GAZEBO_MASTER_URI=http://localhost:11345  # Connects to the Gazebo Server started by UAV6
# Uses ID 1, automatically shifting ports to 14541 / 14581
roslaunch px4 single_vehicle_spawn_sdf.launch x:=1 y:=0 z:=0.5 ID:=1 vehicle:=iris sdf:=iris interactive:=true &
sleep 2
roslaunch mavros px4.launch fcu_url:=udp://:14541@localhost:14581
```

**Terminal 4: Algorithm & Bridge**
```bash
export ROS_MASTER_URI=http://localhost:11312
roslaunch safe_valley_exp safe_flock_sim.launch uav_name:=UAV7
```

---

## Usage (Real-world Deployment)
On a real onboard computer, simply connect to the flight controller via MAVROS and run:
```bash
source ~/catkin_ws/devel/setup.bash
# The program automatically identifies the hostname (e.g., if hostname is UAV6, it runs as UAV6)
roslaunch safe_valley_exp safe_flock_real.launch
```

---
To implement a new algorithm:
1. Define new parameters in `flock.yaml`.
2. Update `flock_config.py` to load these parameters.
3. Implement the mathematical logic as a new method in `flock_math.py`.
4. Call the new method within the state machine in `safe_flock_main.py`.
