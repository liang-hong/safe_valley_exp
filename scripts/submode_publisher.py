#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import sys
import select
import signal

_should_exit = False

def _handle_sigint(signum, frame):
    global _should_exit
    _should_exit = True
    rospy.signal_shutdown('SIGINT')

def main():
    rospy.init_node('submode_publisher', anonymous=True, disable_signals=True)
    signal.signal(signal.SIGINT, _handle_sigint)
    pub = rospy.Publisher('/offb_submode', String, queue_size=10)
    
    valid_submodes = {'form', 'hover', 'navi'}
    
    rospy.loginfo("Submode publisher ready. Valid submodes: form, hover, navi")
    rospy.loginfo("Enter submode and press Enter to publish (Ctrl+C or 'exit' to quit):")
    
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        try:
            if _should_exit:
                break

            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if not rlist:
                continue

            submode = sys.stdin.readline()
            if submode == '':
                break
            submode = submode.strip().lower()
            
            if not submode:
                continue

            if submode in {'exit', 'quit'}:
                break
            
            if submode in valid_submodes:
                msg = String(data=submode)
                pub.publish(msg)
                rospy.loginfo(f"Published: {submode}")
            else:
                rospy.logwarn(f"Invalid submode: {submode}")
                rospy.logwarn(f"Valid options: {', '.join(valid_submodes)}")
                
        except (EOFError, KeyboardInterrupt):
            break
        
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
