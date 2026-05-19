#!/usr/bin/env python3
import rospy
import os
import sys

# 解决在某些环境下无法找到同目录下模块的问题
script_dir = os.path.dirname(os.path.realpath(__file__))
if script_dir not in sys.path:
    sys.path.insert(0, script_dir) # 使用 insert(0, ...) 确保优先加载源码目录而非 devel 空间的 relay 脚本

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
        while not self.comm.origin_set and not rospy.is_shutdown(): 
            rospy.loginfo("[Main] Wait for origin sync...")
            rate.sleep()

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
        hover_msg = PoseStamped()
        hover_msg.header.stamp = rospy.Time.now()
        hover_msg.pose = self.hover_pose.pose
        self.comm.local_pos_pub.publish(hover_msg)

    def execute_formation(self):
        if self.hover_pose is None:
            self.hover_pose = self.comm.own_pose

        if self.cfg.own_name == self.cfg.leader_name:
            # Leader 目标：保持进入模式时的 XY，上升到 leader_height
            target_p = np.array([
                self.hover_pose.pose.position.x,
                self.hover_pose.pose.position.y,
                self.cfg.leader_height
            ])
        else:
            # Follower 目标：跟随 Leader 当前位置并加上编队偏移
            leader_odom = self.comm.drones_data.get(self.cfg.leader_name, {}).get('odom')
            if not leader_odom:
                self.execute_hover() # 安全回退：数据未准备好时就地悬停
                return
            
            target_p = np.array([
                leader_odom.pose.pose.position.x, 
                leader_odom.pose.pose.position.y, 
                leader_odom.pose.pose.position.z
            ]) + self.cfg.form_offset
        
        # 统一的移动逻辑：匀速靠近 + 近距离直接发位置
        current_p = np.array([
            self.comm.own_pose.pose.position.x,
            self.comm.own_pose.pose.position.y,
            self.comm.own_pose.pose.position.z
        ])
        
        p_error = target_p - current_p
        dist = np.linalg.norm(p_error)
        
        if dist < 0.5:
            # 误差足够小，直接发目标位置
            desired_p = target_p
        else:
            # 距离较远，以 vel_form 匀速靠近
            desired_p = current_p + (p_error/dist) * (self.cfg.vel_form * self.cfg.dt)

        # 发布位置指令
        form_msg = PoseStamped()
        form_msg.header.stamp = rospy.Time.now()
        form_msg.pose.position.x, form_msg.pose.position.y, form_msg.pose.position.z = desired_p
        form_msg.pose.orientation = self.hover_pose.pose.orientation
        self.comm.local_pos_pub.publish(form_msg)

    def execute_navigation(self):
        if self.hover_pose is None:
            self.hover_pose = self.comm.own_pose

        if self.cfg.own_name == self.cfg.leader_name:
            # Leader 沿圆轨迹飞行
            if self.leader_start_time is None: self.leader_start_time = rospy.Time.now()
            target_p = self.math.get_leader_circle_position(self.leader_start_time, rospy.Time.now())
            navi_msg = PoseStamped()
            navi_msg.header.stamp = rospy.Time.now()
            navi_msg.pose.position.x, navi_msg.pose.position.y, navi_msg.pose.position.z = target_p
            self.comm.local_pos_pub.publish(navi_msg)
        else:
            # Follower 基于集群算法跟随
            leader_data = self.comm.drones_data.get(self.cfg.leader_name)
            if not leader_data or 'odom' not in leader_data:
                self.execute_hover()
                return
            
            leader_odom = leader_data['odom']
            leader_pos = np.array([
                leader_odom.pose.pose.position.x,
                leader_odom.pose.pose.position.y,
                leader_odom.pose.pose.position.z
            ])
            leader_vel = np.array([
                leader_odom.twist.twist.linear.x,
                leader_odom.twist.twist.linear.y,
                leader_odom.twist.twist.linear.z
            ])

            own_pos = np.array([
                self.comm.own_pose.pose.position.x, 
                self.comm.own_pose.pose.position.y, 
                self.comm.own_pose.pose.position.z
            ])
            own_vel = np.array([
                self.comm.own_vel.twist.linear.x, 
                self.comm.own_vel.twist.linear.y, 
                self.comm.own_vel.twist.linear.z
            ])
            own_ori = self.comm.own_pose.pose.orientation
            
            # 计算四层分量
            neighbors = self.cfg.topology.get(self.cfg.own_name, [])
            v_cohe = self.math.cohe_control(own_pos, leader_pos)
            v_align = self.math.align_control(own_pos, own_vel, self.comm.drones_data, neighbors)
            v_sepa = self.math.sepa_control(own_pos, own_vel, own_ori, self.comm.drones_data, self.cfg.obstacles, self.cfg.leader_name)
            v_flock = self.math.flock_control(own_pos, leader_pos, leader_vel)
            
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
