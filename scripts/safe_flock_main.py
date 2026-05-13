#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, TwistStamped
from flock_config import FlockConfig
from flock_math import FlockMath
from flock_comm import FlockComm

class SafeFlockMain:
    def __init__(self):
        rospy.init_node("safe_flock_main", anonymous=True)
        
        # 1. 加载配置
        self.cfg = FlockConfig()
        
        # 2. 初始化数学库
        self.math = FlockMath(self.cfg)
        
        # 3. 初始化通信与原点同步
        self.comm = FlockComm(self.cfg)
        self.comm.sync_origin()
        
        # 4. 状态变量
        self.hover_pose = None
        self.leader_start_time = None
        self._last_mode = ""
        self._last_submode = ""
        
        rospy.loginfo(f"Safe Flock Main initialized for {self.cfg.own_name}")
        self.run()

    def check_state_change(self):
        curr_mode = self.comm.own_state.mode
        curr_submode = self.comm.offb_submode.data
        if curr_mode != self._last_mode or curr_submode != self._last_submode:
            rospy.loginfo(f"State: {curr_mode} | Submode: {curr_submode}")
            self.hover_pose = None # 切换模式时重置悬停点
        self._last_mode = curr_mode
        self._last_submode = curr_submode

    def run(self):
        rate = rospy.Rate(self.cfg.control_rate)
        while not rospy.is_shutdown():
            self.check_state_change()
            
            # 只有在 OFFBOARD 模式下才执行控制
            if self.comm.own_state.mode == "OFFBOARD":
                submode = self.comm.offb_submode.data
                if submode == "form":
                    self.execute_formation()
                elif submode == "navi":
                    self.execute_navigation()
                else:
                    self.execute_hover()
            else:
                self.execute_hover()
                
            rate.sleep()

    def execute_hover(self):
        if self.hover_pose is None:
            self.hover_pose = self.comm.own_pose
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.pose = self.hover_pose.pose
        self.comm.local_pos_pub.publish(msg)

    def execute_formation(self):
        if self.hover_pose is None:
            self.hover_pose = self.comm.own_pose

        if self.cfg.own_name == self.cfg.leader_name:
            self.execute_hover() # Leader 在编队阶段保持悬停
        else:
            # Follower 向编队位置移动
            leader_odom = self.comm.drones_data.get(self.cfg.leader_name, {}).get('odom')
            if not leader_odom:
                self.execute_hover() # 安全回退：数据未准备好时就地悬停
                return
            
            target_p = np.array([leader_odom.pose.pose.position.x, 
                                leader_odom.pose.pose.position.y, 
                                leader_odom.pose.pose.position.z]) + self.cfg.form_offset
            
            current_p = np.array([self.comm.own_pose.pose.position.x,
                                 self.comm.own_pose.pose.position.y,
                                 self.comm.own_pose.pose.position.z])
            
            p_error = target_p - current_p
            dist = np.linalg.norm(p_error)
            
            if dist < 0.5:
                # 误差足够小，直接发目标位置
                desired_p = target_p
            else:
                # 距离较远，以 vel_form 匀速靠近
                desired_p = current_p + (p_error/dist) * (self.cfg.vel_form / self.cfg.control_rate)

            msg = PoseStamped()
            msg.header.stamp = rospy.Time.now()
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = desired_p
            self.comm.local_pos_pub.publish(msg)

    def execute_navigation(self):
        if self.hover_pose is None:
            self.hover_pose = self.comm.own_pose

        if self.cfg.own_name == self.cfg.leader_name:
            # Leader 沿圆轨迹飞行
            if self.leader_start_time is None: self.leader_start_time = rospy.Time.now()
            target_p = self.math.get_leader_circle_position(self.leader_start_time, rospy.Time.now())
            msg = PoseStamped()
            msg.header.stamp = rospy.Time.now()
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = target_p
            self.comm.local_pos_pub.publish(msg)
        else:
            # Follower 基于集群算法跟随
            own_pos = np.array([self.comm.own_pose.pose.position.x, 
                               self.comm.own_pose.pose.position.y, 
                               self.comm.own_pose.pose.position.z])
            own_vel = np.array([self.comm.own_vel.twist.linear.x, 
                               self.comm.own_vel.twist.linear.y, 
                               self.comm.own_vel.twist.linear.z])
            
            # 计算四层分量
            v_cohe = self.math.cohe_control(own_pos, self.comm.drones_data)
            v_align = self.math.align_control(own_vel, self.comm.drones_data)
            v_sepa = self.math.sepa_control(own_pos, self.comm.drones_data, self.cfg.obstacles)
            
            # Follower 跟随预测的 Leader 位置
            pred_leader = self.math.predict_leader_position(self.comm.leader_pos_history, rospy.Time.now())
            if pred_leader is not None:
                v_flock = (pred_leader + self.cfg.form_offset - own_pos) * 0.5
            else:
                # 如果预测失败（如 Leader 数据丢失），退回到悬停保护逻辑
                self.execute_hover()
                return
            
            # 综合速度
            desired_v = v_cohe + v_align + v_sepa + v_flock
            # 限制速度与加速度
            limited_v = self.math.apply_limits(desired_v, own_vel)
            
            # 发布速度指令
            msg = TwistStamped()
            msg.header.stamp = rospy.Time.now()
            msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = limited_v
            self.comm.local_vel_pub.publish(msg)

if __name__ == '__main__':
    try:
        SafeFlockMain()
    except rospy.ROSInterruptException:
        pass
