#!/usr/bin/env python3
"""多机起飞前只读诊断工具。

历史版本会直接 arm、切 OFFBOARD，并依赖 ego_planner_driver 的旧 TAKEOFF
状态机。当前正式语义由 UavTask PREPARE/START 与机载 executor 唯一负责：
MOVE_TO 首次执行时才 arm，垂直上升 5 m 并稳定 2 s。为避免两个程序同时
拥有飞控模式，本脚本保留原文件名和多 ROS master 调用方式，但不再写 PX4
参数、不调用 arm/set_mode 服务，也不发布位置目标。
"""
import argparse
import os
import subprocess
import sys
import time

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, String

ROS_SETUP = "/opt/ros/noetic/setup.bash"
WS = "/home/ub20tg/catkin_swarm6-2"


class PreflightCheck:
    def __init__(self, idx, timeout_s):
        self.idx = idx
        self.master_port = 11310 + idx
        self.timeout_s = float(timeout_s)
        self.state = None
        self.pose = None
        self.global_position = None
        self.origin_confirmed = False
        self.exec_state = None

    def run(self):
        os.environ["ROS_MASTER_URI"] = "http://localhost:%d" % self.master_port
        os.environ["ROS_HOSTNAME"] = "localhost"
        rospy.init_node("preflight_uav%d" % self.idx, anonymous=True)
        rospy.Subscriber("/mavros/state", State,
                         lambda msg: setattr(self, "state", msg), queue_size=1)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped,
                         lambda msg: setattr(self, "pose", msg), queue_size=1)
        rospy.Subscriber("/mavros/global_position/global", NavSatFix,
                         lambda msg: setattr(self, "global_position", msg), queue_size=1)
        rospy.Subscriber("/gp_origin_confirmed", Bool,
                         lambda msg: setattr(self, "origin_confirmed", bool(msg.data)),
                         queue_size=1)
        rospy.Subscriber("/exec_state", String,
                         lambda msg: setattr(self, "exec_state", msg.data), queue_size=1)

        deadline = time.monotonic() + self.timeout_s
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if (self.state is not None and self.pose is not None and
                    self.global_position is not None and self.exec_state is not None):
                break
            time.sleep(0.1)

        errors = []
        if self.state is None or not self.state.connected:
            errors.append("MAVROS未连接")
        elif self.state.armed:
            errors.append("飞机已经arm，统一原点门禁要求arm前完成")
        if self.pose is None:
            errors.append("无local_position/pose")
        if self.global_position is None:
            errors.append("无global_position/global")
        if not self.origin_confirmed:
            errors.append("gp_origin尚未回读确认")
        if self.exec_state is None:
            errors.append("ego_planner_driver无exec_state")

        if errors:
            rospy.logerr("UAV%d 起飞前检查失败: %s", self.idx, "; ".join(errors))
            return 1
        rospy.loginfo(
            "UAV%d 起飞前检查通过: mode=%s armed=%s exec_state=%s local=(%.2f,%.2f,%.2f)",
            self.idx, self.state.mode, self.state.armed, self.exec_state,
            self.pose.pose.position.x, self.pose.pose.position.y,
            self.pose.pose.position.z)
        return 0


def _single_command(idx, timeout_s):
    shell = (
        "source {ros} && source {ws}/devel/setup.bash && "
        "export ROS_MASTER_URI=http://localhost:{port} && "
        "exec python3 {script} --single {idx} --timeout {timeout}"
    ).format(ros=ROS_SETUP, ws=WS, port=11310 + idx, script=__file__,
             idx=idx, timeout=float(timeout_s))
    return ["bash", "-c", shell]


def main():
    parser = argparse.ArgumentParser(
        description="检查15机MAVROS、位置、原点回读和EGO状态；不执行arm或起飞")
    parser.add_argument("--start", type=int, default=1)
    parser.add_argument("--end", type=int, default=15)
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--individual", action="store_true", help="逐机串行检查")
    args = parser.parse_args()

    indices = list(range(args.start, args.end + 1))
    results = {}
    if args.individual:
        for idx in indices:
            result = subprocess.run(_single_command(idx, args.timeout), check=False)
            results[idx] = result.returncode
    else:
        processes = {
            idx: subprocess.Popen(_single_command(idx, args.timeout)) for idx in indices
        }
        for idx, process in processes.items():
            try:
                results[idx] = process.wait(timeout=args.timeout + 10.0)
            except subprocess.TimeoutExpired:
                process.terminate()
                results[idx] = 124

    for idx in indices:
        print("UAV%-2d: %s" % (idx, "READY" if results[idx] == 0 else "FAIL"))
    return 0 if all(code == 0 for code in results.values()) else 1


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--single":
        single_parser = argparse.ArgumentParser()
        single_parser.add_argument("--single", type=int, required=True)
        single_parser.add_argument("--timeout", type=float, default=20.0)
        single_args = single_parser.parse_args()
        sys.exit(PreflightCheck(single_args.single, single_args.timeout).run())
    sys.exit(main())
