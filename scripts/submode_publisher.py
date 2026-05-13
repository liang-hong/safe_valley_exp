#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def main():
    rospy.init_node('submode_publisher', anonymous=True)
    pub = rospy.Publisher('/offb_submode', String, queue_size=10)
    
    valid_submodes = {'hover', 'navi', 'back'}
    
    rospy.loginfo("Submode publisher ready. Valid submodes: hover, navi, back")
    rospy.loginfo("Enter submode and press Enter to publish (Ctrl+C to exit):")
    
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        try:
            submode = input().strip().lower()
            
            if not submode:
                continue
            
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
