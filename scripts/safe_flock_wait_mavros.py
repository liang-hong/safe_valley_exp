#!/usr/bin/env python3

import os
import sys
import rospy
from mavros_msgs.msg import State


def wait_for_mavros_ready(timeout_s: float) -> bool:
    deadline = rospy.Time.now() + rospy.Duration.from_sec(timeout_s) if timeout_s > 0 else None
    last_log = rospy.Time(0)

    while not rospy.is_shutdown():
        if deadline is not None and rospy.Time.now() > deadline:
            return False

        try:
            state = rospy.wait_for_message("/mavros/state", State, timeout=1.0)
            if state.connected:
                return True
        except rospy.ROSException:
            pass

        if rospy.Time.now() - last_log > rospy.Duration.from_sec(2.0):
            rospy.loginfo("[Wait] Waiting for MAVROS connected...")
            last_log = rospy.Time.now()

    return False


def main():
    rospy.init_node("safe_flock_wait_mavros", anonymous=True)

    timeout_s = float(rospy.get_param("~timeout", 60.0))
    ok = wait_for_mavros_ready(timeout_s)
    if not ok:
        rospy.logerr("[Wait] MAVROS not ready (timeout).")
        return

    rospy.loginfo("[Wait] MAVROS ready, continue.")
    script_dir = os.path.dirname(os.path.realpath(__file__))
    target = os.path.join(script_dir, "safe_flock_main.py")
    argv = [sys.executable, target] + sys.argv[1:]
    os.execv(sys.executable, argv)


if __name__ == "__main__":
    main()
