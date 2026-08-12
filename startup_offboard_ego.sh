#!/bin/bash
# 机载层批量启动：MAVROS + ego_swarm executor（每个仿真无人机一个独立 ROS Master）
#
# 用法（在 safe_valley_exp 所在工作空间 root 运行）：
#   source devel/setup.bash
#   bash src/safe_valley_exp/startup_offboard_ego.sh 1 2 3        # 启动 UAV1/UAV2/UAV3 (tgt_system 1/2/3)
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
source /opt/ros/noetic/setup.bash
source "$WS/devel/setup.bash"

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
    > "/tmp/${uav_name}_offboard_ego.log" 2>&1 &
  echo "$!" > "/tmp/${uav_name}_offboard_ego.pid"
  echo "started $uav_name (master=$master_port, tgt_system=$idx, pid=$!)"
}

stop_all() {
  for idx in "${@:-}"; do
    if [ -f "/tmp/UAV${idx}_offboard_ego.pid" ]; then
      kill "$(cat /tmp/UAV${idx}_offboard_ego.pid)" 2>/dev/null
      rm -f "/tmp/UAV${idx}_offboard_ego.pid"
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
      start_uav "$idx"
    done
    echo "offboard ego started: $*"
    ;;
esac
