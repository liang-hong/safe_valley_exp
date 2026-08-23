#!/bin/bash
# 机载层顺序启动：MAVROS + ego_swarm executor（每个仿真无人机一个独立 ROS Master）
#
# 用法（在 safe_valley_exp 所在工作空间 root 运行）：
#   source devel/setup.bash
#   bash src/safe_valley_exp/startup_offboard_ego.sh 1 2 3        # 顺序启动 UAV1/UAV2/UAV3 (tgt_system 1/2/3)
#   bash src/safe_valley_exp/startup_offboard_ego.sh               # 默认启动 UAV1 (tgt_system 1)
#
# 每个 UAV 一个独立 Master：UAV$idx -> 11310+idx
# 端口公式（与 uav_offboard_sim.launch 一致）：
#   udp_port = 24540 - 1 + tgt_system
#   gcs_port = 34580 - 1 + tgt_system
set -u

# 将 ROS 运行日志重定向到当前工作空间 .ros_home
export ROS_HOME=/home/ub20tg/catkin_swarm6-2/.ros_home
export ROS_LOG_DIR=/home/ub20tg/catkin_swarm6-2/.ros_home/log
mkdir -p "$ROS_LOG_DIR"

WS=/home/ub20tg/catkin_swarm6-2
# 本脚本运行日志/pid 统一放工作空间 .tmp（不使用 /tmp）
mkdir -p "$WS/.tmp"
PX4_ROOT=/home/ub20tg/PX4_Firmware
PX4_BUILD="$PX4_ROOT/build/px4_sitl_default"
source /opt/ros/noetic/setup.bash
source "$WS/devel/setup.bash"
# The PX4 launch files used by uav_offboard_ego.launch are not in the catkin
# workspace.  Load the same Gazebo/PX4 environment used for the simulation
# layer after sourcing catkin, otherwise each background roslaunch exits before
# creating its ROS master and leaves an empty log.
source "$PX4_ROOT/Tools/setup_gazebo.bash" "$PX4_ROOT" "$PX4_BUILD"
export ROS_PACKAGE_PATH="$ROS_PACKAGE_PATH:$PX4_ROOT:$PX4_ROOT/Tools/sitl_gazebo"

STARTUP_TIMEOUT_S=${STARTUP_TIMEOUT_S:-120}

start_uav() {
  local idx=$1
  local master_port=$((11310 + idx))
  local uav_name="UAV$idx"
  local uav_id="UAV$idx"
  local neighbor_odom_topics=""
  local neighbor_idx
  for neighbor_idx in $(seq 1 15); do
    if [ "$neighbor_idx" -eq "$idx" ]; then
      continue
    fi
    if [ -n "$neighbor_odom_topics" ]; then
      neighbor_odom_topics+=","
    fi
    neighbor_odom_topics+="/UAV${neighbor_idx}/mavros/local_position/odom"
  done
  export ROS_MASTER_URI="http://localhost:$master_port"
  export ROS_HOSTNAME=localhost
  nohup roslaunch safe_valley_exp uav_offboard_ego.launch \
    uav_name:=$uav_name uav_id:=$uav_id tgt_system:=$idx \
    neighbor_odom_topics:=$neighbor_odom_topics \
    interfaces_version:=sitl-ego-combined \
    > "$WS/.tmp/${uav_name}_offboard_ego.log" 2>&1 &
  echo "$!" > "$WS/.tmp/${uav_name}_offboard_ego.pid"
  echo "started $uav_name (master=$master_port, tgt_system=$idx, pid=$!)"

  echo "waiting $uav_name ROS Master and MAVROS state (timeout=${STARTUP_TIMEOUT_S}s)"
  if ! timeout "$STARTUP_TIMEOUT_S" bash -c \
    "until ROS_MASTER_URI=http://localhost:$master_port rosnode list >/dev/null 2>&1; do sleep 1; done"; then
    echo "ERROR: $uav_name ROS Master not ready within ${STARTUP_TIMEOUT_S}s" >&2
    return 1
  fi
  if ! timeout "$STARTUP_TIMEOUT_S" bash -c \
    "until ROS_MASTER_URI=http://localhost:$master_port rostopic echo -n 1 /mavros/state >/dev/null 2>&1; do sleep 1; done"; then
    echo "ERROR: $uav_name /mavros/state not ready within ${STARTUP_TIMEOUT_S}s" >&2
    return 1
  fi
  echo "ready $uav_name (master=$master_port, MAVROS state available)"
}

stop_all() {
  for idx in "${@:-}"; do
    if [ -f "$WS/.tmp/UAV${idx}_offboard_ego.pid" ]; then
      kill "$(cat "$WS/.tmp/UAV${idx}_offboard_ego.pid")" 2>/dev/null
      rm -f "$WS/.tmp/UAV${idx}_offboard_ego.pid"
    fi
  done
  sleep 1
  ps aux | grep -E '[u]av_offboard_ego|[m]avros|[u]av_executor|[e]go_planner_driver_node' \
    | awk '{print $2}' | xargs -r kill 2>/dev/null || true
}

if [ "$#" -eq 0 ]; then
  set -- 1
fi

case "$1" in
  stop)
    shift
    stop_all "${@:-}"
    echo "offboard ego stopped"
    ;;
  *)
    for idx in "$@"; do
      if ! start_uav "$idx"; then
        echo "startup stopped before UAV$idx; inspect $WS/.tmp/UAV${idx}_offboard_ego.log" >&2
        exit 1
      fi
    done
    echo "offboard ego started sequentially: $*"
    ;;
esac
