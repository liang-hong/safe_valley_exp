# Safe Valley TODO List

## Completed Work ✅

### Core Functionality
- [x] Implemented safe_valley algorithm based on flocking architecture
- [x] Completed modular config, comm, and main modules
- [x] Implemented GPS clock synchronization calibration
- [x] Modularized main program with configurable submode channel mapping
- [x] Modified own_name reading logic to support both simulation and real hardware

### State Machine & Navigation
- [x] Standardized submode initial state recording
- [x] Fixed formation movement logic
- [x] Fixed leader navigation trajectory calculation logic
- [x] Fixed leader_fix_origin subscription error
- [x] Modified leader circular trajectory to clockwise circle 10m east of current position

### Simulation Support
- [x] Added environment launch for simulation environment and multi-UAV nodes
- [x] Integrated submode_publisher into algorithm launch for simulating RCIN submode switching
- [x] Added combined launch for mavros and algorithm program with wait_mavros script dependency
- [x] Cohesion flock effect verified in simulation, alignment working

### Real Hardware Support
- [x] Modified RCin-submode 3-position switch boundary values
- [x] Modified submode RC channel mapping
- [x] Updated flock communication configuration for new cluster (6 7 9 10, N E S W)
- [x] Adjusted safety radius (UAV radius) for real hardware
- [x] Subscribe to leader's RCIn
- [x] Real hardware launch configuration verification
- [x] Separation behavior testing and verification

### Scripts & Tools
- [x] Renamed flock_math to flock_method
- [x] Added startup script
- [x] Added execution permission to startup script
- [x] Added rosbag recording wrapper script
- [x] Added rosbag recording launch configuration support

### Documentation
- [x] Added Chinese and English README
- [x] Added cross-links between Chinese and English README
- [x] Updated Chinese and English README
- [x] Fixed real hardware operation instructions in README
- [x] Fixed submode topic name and added rosbag recording documentation

### Parameter Tuning
- [x] Adjusted parameters, restored speed limit
- [x] Updated formation velocity parameter in flock.yaml
- [x] Updated target and leader heights parameters

### Logging & Debugging
- [x] Added launch pose logging for each execution mode
- [x] Fixed submode_publisher warnings in simulation, resolved exit issues

---

## In Progress Work ⏳

---

## Pending Work ⏳

### Core Functionality
- [ ] When the UAV is in disarm state, trigger algorithm reset through a specified RC channel switch, which returns the program to the unified coordinate system initialization step and continuously corrects the coordinate reference. This feature is used to handle scenarios such as coordinate jumps during GPS-RTK positioning switching, allowing manual restart of the algorithm process to re-establish correct coordinate reference

### Testing & Verification
- [ ] Coordinate reconstruction functionality testing and verification

---

## Maintenance & Improvements
- [ ] Continuous flocking parameter optimization
