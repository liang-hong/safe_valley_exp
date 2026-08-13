#!/usr/bin/env python3
"""
15 机无头 SITL 批量 arm + OFFBOARD 脚本（每机独立 ROS Master 11311..11325）。

职责（配合正确启动流程）：
  0. 先启动机载层（offboard 程序）：MAVROS + ego driver + executor + bridge，
     统一坐标系并待命；ego driver 启动后处于 TAKEOFF 状态，在 IDLE 下已持续
     30Hz 发布 /mavros/setpoint_position/local（经 setpoint_relay）。
  1. 本脚本对选中的每架 UAV：
     a. arm 无人机
     b. 切 OFFBOARD
     c. 成功进入 OFFBOARD 后立即退出；后续 TAKEOFF/HOLD/IDLE
         状态转换完全由 ego_planner_driver 负责

用法：
  python3 offboard_takeoff_15.py 1 2 3
  不写编号 -> 默认 1..15；默认逐机运行。
  加 --parallel -> 所有选中 UAV 并发运行。

注意:
  - 无 RC 的 SITL 必须设 COM_RCL_EXCEPT=4，否则 RC 丢失触发 failsafe（RTL）
  - 起飞高度由 ego_planner_driver 的 takeoff_height_m 参数控制（默认 5.0m）
  - 本脚本不再调用 /mavros/cmd/takeoff（MAV_CMD_NAV_TAKEOFF），
    避免 PX4 preflight 在 set_gp_origin 修改 EKF2 origin 后拦截
"""
import argparse
import os
import sys

import rospy
from mavros_msgs.srv import CommandBool, SetMode

ROS_SETUP = "/opt/ros/noetic/setup.bash"
WS = "/home/ub20tg/catkin_swarm6-2"


class TakeoffUAV:
    def __init__(self, idx):
        self.idx = idx
        self.master_port = 11310 + idx

    def run(self):
        os.environ["ROS_MASTER_URI"] = "http://localhost:%d" % self.master_port
        os.environ["ROS_HOSTNAME"] = "localhost"
        rospy.init_node("takeoff_uav%d" % self.idx, anonymous=True)
        rospy.loginfo("UAV%d: master=%d", self.idx, self.master_port)

        # 1) arm。ego_planner_driver 自己看 MAVROS 状态，触发 TAKEOFF。
        try:
            rospy.wait_for_service("/mavros/cmd/arming", timeout=10)
            arm = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        except rospy.ROSException as exc:
            rospy.logerr("UAV%d: arm 服务不可用: %s", self.idx, exc)
            return 1

        resp = arm(True)
        rospy.loginfo("UAV%d: arm result=%s", self.idx, resp.success)
        if not resp.success:
            rospy.logerr("UAV%d: arm 被拒绝", self.idx)
            return 1

        # 2) 切 OFFBOARD。只看服务响应，不等状态回报。
        try:
            rospy.wait_for_service("/mavros/set_mode", timeout=10)
            set_mode = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        except rospy.ROSException as exc:
            rospy.logerr("UAV%d: set_mode 服务不可用: %s", self.idx, exc)
            return 1
        resp = set_mode(0, "OFFBOARD")
        rospy.loginfo("UAV%d: OFFBOARD mode_sent=%s", self.idx, resp.mode_sent)
        if not resp.mode_sent:
            rospy.logerr("UAV%d: OFFBOARD 请求失败", self.idx)
            return 1
        rospy.loginfo("UAV%d: arm + OFFBOARD 请求完成；脚本退出", self.idx)
        return 0


def main():
    parser = argparse.ArgumentParser(
        description="选定 UAV arm + OFFBOARD（默认逐机，ego driver 自动软起飞）")
    parser.add_argument("uavs", nargs="*", type=int, metavar="UAV",
                        help="UAV 编号，例如：1 2 3；不写则运行 1..15")
    parser.add_argument("--parallel", action="store_true",
                        help="并发运行；默认逐机运行")
    parser.add_argument("--individual", action="store_true",
                        help="兼容旧命令；默认就是逐机运行")
    args = parser.parse_args()

    results = {}
    idxs = args.uavs or list(range(1, 16))
    if any(idx < 1 or idx > 15 for idx in idxs):
        parser.error("UAV 编号必须在 1..15")
    if len(set(idxs)) != len(idxs):
        parser.error("UAV 编号不能重复")

    if not args.parallel:
        # 逐机串行：每个 UAV 在独立子进程内运行 --single，避免
        # rospy.init_node 同一进程重复调用的问题。
        import subprocess
        for idx in idxs:
            cmd = [
                "bash", "-c",
                "source %s && source %s/devel/setup.bash && "
                "export ROS_MASTER_URI=http://localhost:%d && "
                "exec python3 %s --single %d"
                % (ROS_SETUP, WS, 11310 + idx, __file__, idx),
            ]
            r = subprocess.run(cmd)
            results[idx] = "OK" if r.returncode == 0 else "FAIL(%d)" % r.returncode
    else:
        import subprocess
        procs = {}
        for idx in idxs:
            cmd = [
                "bash", "-c",
                "source %s && source %s/devel/setup.bash && "
                "export ROS_MASTER_URI=http://localhost:%d && "
                "exec python3 %s --single %d"
                % (ROS_SETUP, WS, 11310 + idx, __file__, idx),
            ]
            p = subprocess.Popen(cmd)
            procs[idx] = p
        for idx in idxs:
            p = procs[idx]
            p.wait()
            results[idx] = "OK" if p.returncode == 0 else "FAIL(%d)" % p.returncode

    print("=== arm+OFFBOARD 汇总 ===")
    for idx in idxs:
        print("UAV%-2d (%d): %s" % (idx, 11310 + idx, results.get(idx, "?")))
    ok = [i for i, v in results.items() if v == "OK"]
    print("成功 %d/%d: %s" % (len(ok), len(idxs), sorted(ok)))
    return 0 if len(ok) == len(idxs) else 1


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--single":
        p = argparse.ArgumentParser()
        p.add_argument("--single", type=int)
        a, _ = p.parse_known_args()
        sys.exit(0 if TakeoffUAV(a.single).run() == 0 else 1)
    sys.exit(main())