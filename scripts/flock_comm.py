#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped, GeoPointStamped
from mavros_msgs.msg import State, RCIn
from nav_msgs.msg import Odometry
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
        self.offb_submode = String(data="hover")
        self.leader_rcin = RCIn()
        
        self.leader_pos_history = []
        self.origin_set = False

        # 初始化发布器
        self.init_publishers()
        # 初始化订阅器
        self.init_subscribers()

    def init_publishers(self):
        self.local_vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
        self.local_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.set_origin_pub = rospy.Publisher("/mavros/global_position/set_gp_origin", GeoPointStamped, queue_size=10)
        # 如果是 Leader，还需要发布原点给其他人
        if self.cfg.own_name == self.cfg.leader_name:
            self.leader_origin_pub = rospy.Publisher("/leader_gp_origin", NavSatFix, queue_size=10)

    def init_subscribers(self):
        # 本机基础状态
        rospy.Subscriber("/mavros/state", State, self.own_state_cb)
        rospy.Subscriber("/mavros/local_position/odom", Odometry, self.own_odom_cb)
        
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

    def subscribe_leader_special_topics(self):
        prefix = ensure_global(self.cfg.leader_name)
        # 订阅 Leader 的 RCIn (用于同步切换模式)
        rospy.Subscriber(prefix + "/mavros/rc/in", RCIn, self.leader_rcin_cb)
        # 订阅 Leader 的原点
        rospy.Subscriber("/leader_gp_origin", NavSatFix, self.leader_gp_origin_cb)

    # --- Callbacks ---
    def own_state_cb(self, msg): self.own_state = msg
    def own_odom_cb(self, msg):
        self.own_odom = msg
        self.own_pose.pose = msg.pose.pose
        self.own_vel.twist = msg.twist.twist
    
    def offb_submode_cb(self, msg): self.offb_submode = msg
    
    def other_state_cb(self, msg, name):
        if name not in self.drones_data: self.drones_data[name] = {}
        self.drones_data[name]['state'] = msg

    def other_odom_cb(self, msg, name):
        if name not in self.drones_data: self.drones_data[name] = {}
        self.drones_data[name]['odom'] = msg
        if name == self.cfg.leader_name:
            self.leader_pos_history.append((rospy.Time.now().to_sec(), msg.pose.pose.position))
            if len(self.leader_pos_history) > self.cfg.history_max_size:
                self.leader_pos_history.pop(0)

    def leader_rcin_cb(self, msg):
        self.leader_rcin = msg
        # 将配置文件中的 1-based 通道号转为 0-based 索引
        ch_idx = self.cfg.submode_channel - 1
        if 0 <= ch_idx < len(self.leader_rcin.channels):
            val = self.leader_rcin.channels[ch_idx]
            if val < 1200:
                self.offb_submode.data = "form"
            elif 1300 <= val <= 1500:
                self.offb_submode.data = "hover"
            elif val >= 1600:
                self.offb_submode.data = "navi"

    def leader_gp_origin_cb(self, msg):
        if self.cfg.leader_name not in self.drones_data: self.drones_data[self.cfg.leader_name] = {}
        self.drones_data[self.cfg.leader_name]['leader_origin'] = msg

    # --- Origin Sync Logic ---
    def sync_origin(self):
        if self.cfg.own_name == self.cfg.leader_name:
            self._set_leader_origin()
        else:
            self._set_follower_origin()

    def _set_leader_origin(self):
        rospy.loginfo("[Comm] Leader waiting for global fix...")
        while not rospy.is_shutdown():
            try:
                fix = rospy.wait_for_message("/mavros/global_position/global", NavSatFix, timeout=5.0)
                if fix.status.status >= 2:
                    geo = GeoPointStamped()
                    geo.position.latitude = fix.latitude
                    geo.position.longitude = fix.longitude
                    geo.position.altitude = fix.altitude
                    self.set_origin_pub.publish(geo)
                    self.leader_origin_fix = fix
                    self.origin_set = True
                    rospy.Timer(rospy.Duration(1.0), lambda e: self.leader_origin_pub.publish(self.leader_origin_fix))
                    rospy.loginfo("[Comm] Leader origin set and broadcasting.")
                    break
            except: rospy.logwarn("[Comm] Leader fix unavailable, retrying...")
            rospy.sleep(1.0)

    def _set_follower_origin(self):
        rospy.loginfo("[Comm] Follower waiting for leader origin...")
        while not rospy.is_shutdown():
            leader_fix = self.drones_data.get(self.cfg.leader_name, {}).get('leader_origin')
            if leader_fix and leader_fix.status.status >= 2:
                geo = GeoPointStamped()
                geo.position.latitude = leader_fix.latitude
                geo.position.longitude = leader_fix.longitude
                geo.position.altitude = leader_fix.altitude
                self.set_origin_pub.publish(geo)
                self.origin_set = True
                rospy.loginfo("[Comm] Follower origin synchronized with leader.")
                break
            rospy.sleep(1.0)
