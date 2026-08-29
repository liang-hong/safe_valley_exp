# safe_valley_exp

Current scope (flock legacy examples removed per implementation_plan_26082916 §6):

- **GPS clock-bias node** (`gps_bias_node.py`): subscribes `/mavros/time_reference`
  (GPS PPS), estimates the local ROS-clock to GPS-time offset via a sliding-window
  mean, and publishes `gps_bias` (latched). Peers forward the bias over the topology
  bridge; receivers correct inter-aircraft timestamps with
  `T_own = T_other - bias_other + bias_own`.
- **Onboard offboard EGO launch** (`uav_offboard_ego.launch` + `startup_offboard_ego.sh`):
  MAVROS + `uav_executor_ego.launch` + Group A topology bridge + GPS bias node.
- **15-UAV SITL simulation** (`multi_uav_ego_15sim.launch` / `multi_uav_ego_4sim.launch` /
  `multi_uav_sim*.launch`): spawns iris_0..iris_14 in Gazebo.
- **Takeoff/MAVROS wait**: `offboard_takeoff_15.py`, `wait_mavros.py`.

## GPS bias parameters

Sole source `config/gps_bias_defaults.yaml` (private namespace of `gps_bias_node`):

| param | default | meaning |
|---|---|---|
| `window_s` | 10.0 | sliding mean window (s) |
| `publish_rate_hz` | 1.0 | publish rate (Hz) |
| `lockout_s` | 3.0 | GPS-lockout threshold (s) |

Validation: all doubles finite; `window_s`/`publish_rate_hz` >0, `lockout_s` >=0;
node fails fast on invalid configuration.

## Launch

```bash
export ROS_HOME=/home/ub20tg/catkin_swarm6-2/.ros_home
export ROS_LOG_DIR=/home/ub20tg/catkin_swarm6-2/.ros_home/log
bash startup_offboard_ego.sh
```

15-UAV SITL world (terminal A, master 11300):

```bash
roslaunch safe_valley_exp multi_uav_ego_15sim.launch
```

## Tests

```bash
catkin run_tests safe_valley_exp
```

- `test/test_gps_bias.py`: `BiasEstimator` pure-logic unit tests (no ROS).
- `test/test_gps_bias_wiring.py`: launch canonical-YAML wiring + script permission.
- `test/gps_bias_no_reference.test`: rostest (lockout without reference clock).
