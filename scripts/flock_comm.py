#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, RCIn
from nav_msgs.msg import Odometry
from geographic_msgs.msg import GeoPointStamped
from sensor_msgs.msg import NavSatFix, TimeReference
from std_msgs.msg import String
import numpy as np

def ensure_global(name):
    return name if name.startswith('/') else '/' + name

class FlockComm:
    def __init__(self, config):
        self.cfg = config
        self.drones_data = {}
        self.own_state = State()
        self.own_odom = Odometry()
        self.own_pose = PoseStamped()
        self.own_vel = TwistStamped()
        self.own_time_ref = TimeReference()
        self.offb_submode = String(data="hover")
        self.leader_rcin = RCIn()
        
        self.leader_pos_history = []
        self.origin_set = False
        
        # 时钟同步偏差 (Bias = System_Time - GPS_Time)
        self.own_bias = rospy.Duration(0)

        # 初始化发布器
        self.init_publishers()
        # 初始化订阅器
        self.init_subscribers()
        rospy.sleep(1)

    def init_publishers(self):
        self.local_vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
        self.local_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.set_origin_pub = rospy.Publisher("/mavros/global_position/set_gp_origin", GeoPointStamped, queue_size=10)
        # 如果是 Leader，还需要发布原点给其他人
        if self.cfg.own_name == self.cfg.leader_name:
            self.leader_fix_origin_pub = rospy.Publisher("/leader_fix_origin", NavSatFix, queue_size=10)

    def init_subscribers(self):
        # 本机基础状态
        rospy.Subscriber("/mavros/state", State, self.own_state_cb)
        rospy.Subscriber("/mavros/local_position/odom", Odometry, self.own_odom_cb)
        rospy.Subscriber("/mavros/time_reference", TimeReference, self.own_time_ref_cb)
        
        # 子模式订阅
        rospy.Subscriber("/offb_submode", String, self.offb_submode_cb)

        # 根据拓扑订阅邻居
        if self.cfg.own_name in self.cfg.topology:
            for other_name in self.cfg.topology[self.cfg.own_name]:
                self.subscribe_other_topics(other_name)

        # 确保订阅了 Leader (用于获取 RC 指令或原点)
        if self.cfg.leader_name != self.cfg.own_name:
            self.subscribe_leader_special_topics()
            self.subscribe_other_topics(self.cfg.leader_name)

    def subscribe_other_topics(self, other_name):
        prefix = ensure_global(other_name)
        for topic_cfg in self.cfg.topics:
            t_name = prefix + ensure_global(topic_cfg['name'])
            t_type = topic_cfg['type']
            if 'Odometry' in t_type:
                rospy.Subscriber(t_name, Odometry, self.other_odom_cb, other_name)
            elif 'State' in t_type:
                rospy.Subscriber(t_name, State, self.other_state_cb, other_name)
            elif 'TimeReference' in t_type:
                rospy.Subscriber(t_name, TimeReference, self.other_time_ref_cb, other_name)

    def subscribe_leader_special_topics(self):
        prefix = ensure_global(self.cfg.leader_name)
        # 订阅 Leader 的 RCIn (用于同步切换模式)
        rospy.Subscriber(prefix + "/mavros/rc/in", RCIn, self.leader_rcin_cb)
        # 订阅 Leader 的原点
        rospy.Subscriber(prefix + "/leader_fix_origin", NavSatFix, self.leader_fix_origin_cb)

    # --- Callbacks ---
    def own_state_cb(self, msg): self.own_state = msg
    def own_odom_cb(self, msg):
        self.own_odom = msg
        self.own_pose.header = msg.header
        self.own_pose.pose = msg.pose.pose
        self.own_vel.header = msg.header
        self.own_vel.twist = msg.twist.twist
    
    def own_time_ref_cb(self, msg):
        self.own_time_ref = msg
        # 计算本机系统时钟与 GPS 时间的偏差
        self.own_bias = msg.header.stamp - msg.time_ref
    
    def offb_submode_cb(self, msg): self.offb_submode = msg
    
    def other_state_cb(self, msg, name):
        if name not in self.drones_data: self.drones_data[name] = {}
        self.drones_data[name]['state'] = msg

    def other_odom_cb(self, msg, name):
        if name not in self.drones_data: self.drones_data[name] = {}
        
        # 修正时间戳：将他机系统时间转换成本机系统时间
        # 公式：T_own = T_other - Bias_other + Bias_own
        if 'bias' in self.drones_data[name]:
            msg.header.stamp = msg.header.stamp - self.drones_data[name]['bias'] + self.own_bias
            
        self.drones_data[name]['odom'] = msg
        # if name == self.cfg.leader_name:
        #     self.leader_pos_history.append((rospy.Time.now().to_sec(), msg.pose.pose.position))
        #     if len(self.leader_pos_history) > self.cfg.history_max_size:
        #         self.leader_pos_history.pop(0)

    def other_time_ref_cb(self, msg, name):
        if name not in self.drones_data: self.drones_data[name] = {}
        self.drones_data[name]['time_reference'] = msg
        # 计算该他机系统时钟与 GPS 时间的偏差
        self.drones_data[name]['bias'] = msg.header.stamp - msg.time_ref

    def leader_rcin_cb(self, msg):
        self.leader_rcin = msg
        # 将配置文件中的 1-based 通道号转为 0-based 索引
        ch_idx = self.cfg.submode_channel - 1
        if 0 <= ch_idx < len(self.leader_rcin.channels):
            val = self.leader_rcin.channels[ch_idx]
            if val < 1300:
                self.offb_submode.data = "form"
            elif 1400 <= val <= 1600:
                self.offb_submode.data = "hover"
            elif val >= 1700:
                self.offb_submode.data = "navi"

    def leader_fix_origin_cb(self, msg):
        if self.cfg.leader_name not in self.drones_data: self.drones_data[self.cfg.leader_name] = {}
        self.drones_data[self.cfg.leader_name]['leader_fix_origin'] = msg

    # --- Origin Sync Logic ---
    def sync_origin(self):
        if self.cfg.own_name == self.cfg.leader_name:
            self._set_leader_origin()
        else:
            self._set_follower_origin()

    def _set_leader_origin(self):
        rospy.loginfo("[Comm] Leader wait for global fix...")
        leader_fix = NavSatFix()
        leader_fix.status.status = -1
        while not rospy.is_shutdown():
            try:
                leader_fix = rospy.wait_for_message("/mavros/global_position/global", NavSatFix, timeout=5.0)
                # 增加状态提示：-1=No Fix, 0=Fix, 1=SBAS, 2=GBAS(RTK)
                if leader_fix.status.status >= self.cfg.min_gps_status:
                    leader_gp_origin = GeoPointStamped()
                    leader_gp_origin.position.latitude = leader_fix.latitude
                    leader_gp_origin.position.longitude = leader_fix.longitude
                    leader_gp_origin.position.altitude = leader_fix.altitude
                    self.set_origin_pub.publish(leader_gp_origin)
                    self.leader_fix_origin = leader_fix
                    self.origin_set = True
                    rospy.Timer(rospy.Duration(1.0), lambda e: self.leader_fix_origin_pub.publish(self.leader_fix_origin))
                    rospy.loginfo(f"[Comm] Leader gp_origin set (status: {leader_fix.status.status}) and fix_origin broadcast.")
                    break
                else:
                    rospy.logwarn(f"[Comm] GPS status is {leader_fix.status.status}, waiting for >= {self.cfg.min_gps_status}...")
            except Exception as e:
                rospy.logwarn(f"[Comm] Leader global unfix or timeout, retry... Error: {e}")
            rospy.sleep(1.0)

    def _set_follower_origin(self):
        rospy.loginfo(f"[Comm] Follower wait for leader fix_origin (required >= {self.cfg.min_gps_status})...")
        leader_fix = NavSatFix()
        leader_fix.status.status = -1
        while not rospy.is_shutdown():
            leader_fix = self.drones_data.get(self.cfg.leader_name, {}).get('leader_fix_origin')
            if leader_fix and leader_fix.status.status >= self.cfg.min_gps_status:
                follower_gp_origin = GeoPointStamped()
                follower_gp_origin.position.latitude = leader_fix.latitude
                follower_gp_origin.position.longitude = leader_fix.longitude
                follower_gp_origin.position.altitude = leader_fix.altitude
                self.set_origin_pub.publish(follower_gp_origin)
                self.origin_set = True
                rospy.loginfo(f"[Comm] Follower gp_origin sync with leader (status: {leader_fix.status.status}).")
                break
            else:
                curr_status = leader_fix.status.status if leader_fix else "None"
                rospy.logwarn(f"[Comm] Leader GPS status is {curr_status}, waiting for >= {self.cfg.min_gps_status}...")
            rospy.sleep(1.0)
