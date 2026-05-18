# Safe Valley Experiment (safe_valley_exp)

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

## Usage
### Real-world Deployment
```bash
roslaunch safe_valley_exp safe_flock.launch own_name:=UAV1
```
### Simulation (Multi-Master)
When used with `swarm_topology_bridge`, you can run multiple instances on one machine:
```bash
roslaunch safe_valley_exp test_sim_swarm.launch
```

## Extension Guide
To implement a new algorithm:
1. Define new parameters in `flock.yaml`.
2. Update `flock_config.py` to load these parameters.
3. Implement the mathematical logic as a new method in `flock_math.py`.
4. Call the new method within the state machine in `safe_flock_main.py`.
