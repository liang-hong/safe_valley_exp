#!/usr/bin/env python3

import rospy
import socket
import numpy as np
import math
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, RCIn
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geographic_msgs.msg import GeoPointStamped
from sensor_msgs.msg import NavSatFix, TimeReference

def ensure_global(name):
    return name if name.startswith('/') else '/' + name

class SafeFlockController:
    def __init__(self, own_name=None):
        rospy.init_node("safe_flock_real", anonymous=True)

        # ========== 1. 确定本机无人机名称 ==========
        self.determine_own_name(own_name)

        # ========== 2. 加载集群配置和算法参数 ==========
        self.load_config()

        # ========== 3. 检查节点（topology中所有跟自身相关的节点） ==========
        self.check_topology_nodes()

        # ========== 4. 初始化订阅和发布 ==========
        self.init_subscribers()
        self.init_publishers()
        rospy.sleep(1.0)

        # ========== 5. 统一坐标系 ==========
        self.set_uni_origin(self.leader_name, self.own_name)

        rospy.sleep(1.0)
        rospy.loginfo("Safe Flock initialized")

        # ========== 7. 进入状态机主循环 ==========
        self.run_state_machine()

    def determine_own_name(self, own_name):
        """确定本机无人机名称（不含/前缀）"""
        if own_name is not None:
            self.own_name = own_name.strip('/')
            rospy.loginfo(f"own_name: {self.own_name}")
        else:
            hostname = socket.gethostname()
            if "UAV" in hostname and any(c.isdigit() for c in hostname):
                self.own_name = hostname.strip('/')
                rospy.loginfo(f"hostname: {self.own_name}")
            else:
                own_name_param = rospy.get_param('~own_name', None)
                if own_name_param is not None:
                    self.own_name = own_name_param.strip('/')
                    rospy.loginfo(f"config own_name: {self.own_name}")
                else:
                    rospy.logerr(f"own_name unresolved: hostname '{hostname}'")
                    raise RuntimeError("own_name not resolved")

    def load_config(self):
        """从 flock.yaml 加载集群配置和算法参数"""
        self.init_positions = rospy.get_param('~init_position', {})
        self.control_params = rospy.get_param('~control', {})
        self.leader_params = rospy.get_param('~leader', {})
        self.topics = rospy.get_param('~topics', [])
        self.topology = rospy.get_param('~topology', {})
        self.obstacles = rospy.get_param('~obstacles', [])

        self.leader_name = self.leader_params.get('name', 'UAV6').strip('/')
        self.leader_topics = self.leader_params.get('leader_topics', [])

        self.control_rate = self.control_params.get('rate', 30.0)
        self.rate = rospy.Rate(self.control_rate)

        self.init_control_params()
        self.init_state_machine_vars()

        rospy.loginfo(f"leader: {self.leader_name}, rate: {self.control_rate}")
        rospy.loginfo(f"topics: {len(self.topics)}, topology: {self.topology.get(self.own_name, [])}")

    def init_control_params(self):
        """初始化控制参数"""
        # ========== cohe 参数 (凝聚) ==========
        self.r_safe = self.control_params.get('r_safe', 0.5)
        self.r_pack = self.control_params.get('r_pack', 2.1)
        self.phi_sc = self.control_params.get('phi_sc', 0.523)
        self.p_cohe = self.control_params.get('p_cohe', 1.5)

        # ========== align 参数 (对齐) ==========
        self.a_max = self.control_params.get('a_max', 5.0)
        self.v_max = self.control_params.get('v_max', 1.5)
        self.dv_max = self.control_params.get('dv_max', 0.167)
        self.r_align = self.control_params.get('r_align', 10.0)
        self.p_align = self.control_params.get('p_align', 0.6)

        # ========== sepa 参数 (分离/避障) ==========
        self.b = self.control_params.get('b', 1.0)
        self.r_sen = self.control_params.get('r_sen', 5.0)
        self.e_max = self.control_params.get('e_max', 0.95)
        self.p_sepa_quad = self.control_params.get('p_sepa_quad', 0.3)
        self.p_sepa_obs = self.control_params.get('p_sepa_obs', 0.05)

        # ========== flock 参数 (跟随 leader) ==========
        self.target_height = self.control_params.get('target_height', 2.0)
        self.kp_flock_z = self.control_params.get('kp_flock_z', 0.5)
        self.max_climb_vel = self.control_params.get('max_climb_vel', 1.0)

        # ========== formation 参数 (编队形成) ==========
        self.vel_form = self.control_params.get('vel_form', 2.0)
        self.kp_form = self.control_params.get('kp_form', 0.5)
        self.kd_form = self.control_params.get('kd_form', 0.1)
        self.min_height = self.control_params.get('min_height', 2.0)

        # 从 init_position 加载本机编队偏移
        if self.own_name in self.init_positions:
            self.form_offset = np.array(self.init_positions[self.own_name])
        else:
            self.form_offset = np.array([0.0, 0.0, 0.0])

        # ========== leader 参数 (领航轨迹) ==========
        self.leader_height = self.leader_params.get('leader_height', 5.0)
        self.circle_center = np.array(self.leader_params.get('circle_center', [10.0, 0.0]))
        self.circle_radius = self.leader_params.get('circle_radius', 10.0)
        self.circle_speed = self.leader_params.get('circle_speed', 0.5)
        self.clockwise = self.leader_params.get('clockwise', True)
        self.hover_on_start = self.leader_params.get('hover_on_start', True)

        self.drones_data = {}
        self.dt = 1.0 / self.control_rate

    def init_state_machine_vars(self):
        """初始化状态机变量"""
        self.offb_submode = String(data="hover")
        self._last_mode = ""
        self._last_submode = ""

        self.hover_pose = None
        self.origin_set = False

        self.leader_pos_history = []
        self.prediction_window = 0.1
        self.history_max_size = int(self.prediction_window * self.control_rate)

        self.formation_start_pose = None

    def check_topology_nodes(self):
        """检查 topology 中所有跟自身相关的节点"""
        nodes_to_check = set()
        if self.own_name in self.topology:
            nodes_to_check.update(self.topology[self.own_name])
        nodes_to_check.discard(self.own_name)

        for node_name in nodes_to_check:
            self.check_node(node_name)

        if self.leader_name != self.own_name:
            self.check_node(self.leader_name)

    def check_node(self, uav_name):
        """检查指定UAV的节点是否存在并已连接"""
        is_own = (uav_name == self.own_name)
        topic_prefix = "" if is_own else f"/{uav_name}"

        topic_name = "/mavros/state"
        uav_type = "own" if is_own else "other"
        rospy.loginfo(f"checking {uav_type} {uav_name}{topic_name}")

        state_msg = State()
        while not rospy.is_shutdown():
            try:
                state_msg = rospy.wait_for_message(topic_prefix + topic_name, State, timeout=5.0)
                if state_msg.connected:
                    rospy.loginfo(f"{uav_name} connected")
                    break
                else:
                    rospy.logwarn(f"{uav_name} not connected")
            except rospy.ROSException:
                rospy.logwarn(f"{uav_name}{topic_name} unavailable")
            rospy.sleep(1.0)

    def init_subscribers(self):
        """初始化订阅器"""
        self.own_state = State()
        rospy.Subscriber("/mavros/state", State, self.own_state_cb)

        self.own_odom = Odometry()
        self.own_pose = PoseStamped()
        self.own_vel = TwistStamped()
        rospy.Subscriber("/mavros/local_position/odom", Odometry, self.own_odom_cb)
        rospy.Subscriber("/mavros/time_reference", TimeReference, self.own_time_ref_cb)

        self.subscribe_leader_topics()

        if self.own_name in self.topology:
            for other_name in self.topology[self.own_name]:
                self.subscribe_other_topics(other_name)

        if self.leader_name != self.own_name:
            self.subscribe_other_topics(self.leader_name)

        rospy.Subscriber("/offb_submode", String, self.offb_submode_cb)

    def subscribe_leader_topics(self):
        """订阅 leader 的专属话题（rcin 和 leader_fix_origin）"""
        for topic_cfg in self.leader_topics:
            topic_name = ensure_global(topic_cfg['name'])
            topic_type = topic_cfg['type']

            if self.leader_name == self.own_name:
                full_topic = topic_name
            else:
                full_topic = f"/{self.leader_name}{topic_name}"

            rospy.loginfo(f"Sub: {full_topic}")

            if 'RCIn' in topic_type:
                rospy.Subscriber(full_topic, RCIn, self.leader_rcin_cb)
            elif 'NavSatFix' in topic_type:
                rospy.Subscriber(full_topic, NavSatFix, lambda msg, name=self.leader_name: self.leader_fix_origin_cb(msg, name))

    def subscribe_other_topics(self, other_name):
        """订阅其他无人机的topics"""
        for topic_cfg in self.topics:
            topic_name = ensure_global(topic_cfg['name'])
            topic_type = topic_cfg['type']
            full_topic = f"/{other_name}{topic_name}"

            rospy.loginfo(f"Sub: {full_topic}")

            if 'State' in topic_type:
                rospy.Subscriber(full_topic, State, lambda msg, name=other_name: self.other_state_cb(msg, name))
            elif 'TimeReference' in topic_type:
                rospy.Subscriber(full_topic, TimeReference, lambda msg, name=other_name: self.other_time_ref_cb(msg, name))
            elif 'Odometry' in topic_type:
                rospy.Subscriber(full_topic, Odometry, lambda msg, name=other_name: self.other_odom_cb(msg, name))

    def init_publishers(self):
        """初始化发布器"""
        self.local_vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
        self.local_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.set_origin_pub = rospy.Publisher("/mavros/global_position/set_gp_origin", GeoPointStamped, queue_size=10)

    def set_uni_origin(self, leader_name, own_name):
        """设置统一坐标系原点"""
        if leader_name == own_name:
            self.set_leader_origin()
        else:
            self.set_follower_origin()

    def set_leader_origin(self):
        """Leader: 发布自身原点到 leader_fix_origin"""
        global_topic = "/mavros/global_position/global"
        leader_origin_topic = "/leader_fix_origin"

        rospy.loginfo("Waiting leader global fix")
        leader_origin_fix = NavSatFix()
        leader_origin_fix.status.status = -1
        while not rospy.is_shutdown() and leader_origin_fix.status.status < 2:
            try:
                leader_origin_fix = rospy.wait_for_message(global_topic, NavSatFix, timeout=5.0)
                if leader_origin_fix.status.status >= 2:
                    rospy.loginfo("Leader global fix")
                    break
            except rospy.ROSException:
                rospy.logwarn("Leader fix unavailable")
            rospy.sleep(1.0)

        leader_origin = GeoPointStamped()
        leader_origin.position.latitude = leader_origin_fix.latitude
        leader_origin.position.longitude = leader_origin_fix.longitude
        leader_origin.position.altitude = leader_origin_fix.altitude
        
        self.set_origin_pub.publish(leader_origin)
        rospy.loginfo(f"Set leader {self.own_name} origin")

        self.leader_origin_fix = leader_origin_fix
        self.leader_origin_pub = rospy.Publisher(leader_origin_topic, NavSatFix, queue_size=10)
        rospy.sleep(1.0)
        self.origin_set = True
        rospy.loginfo("Origin saved, start publishing")

        self.origin_timer = rospy.Timer(rospy.Duration(1.0), self.publish_origin_cb)

    def publish_origin_cb(self, event=None):
        """Timer callback for periodic origin publishing"""
        if self.origin_set and hasattr(self, 'leader_origin_fix'):
            self.leader_origin_pub.publish(self.leader_origin_fix)

    def set_follower_origin(self):
        """Follower: 从 leader 订阅 leader_fix_origin 并设置本地原点"""
        rospy.loginfo("Waiting leader origin fix")
        leader_origin_fix = NavSatFix()
        leader_origin_fix.status.status = -1
        while not rospy.is_shutdown() and leader_origin_fix.status.status < 2:
            if self.leader_name in self.drones_data and 'leader_fix_origin' in self.drones_data[self.leader_name]:
                leader_origin_fix = self.drones_data[self.leader_name]['leader_fix_origin']
                if leader_origin_fix.status.status >= 2:
                    rospy.loginfo("Leader fix origin received")
                    break
            rospy.sleep(1.0)

        follower_origin = GeoPointStamped()
        follower_origin.position.latitude = leader_origin_fix.latitude
        follower_origin.position.longitude = leader_origin_fix.longitude
        follower_origin.position.altitude = leader_origin_fix.altitude
        
        self.set_origin_pub.publish(follower_origin)
        rospy.loginfo(f"Set follower {self.own_name} origin")
        self.origin_set = True

    # ========== 回调函数 ==========
    def own_state_cb(self, msg):
        self.own_state = msg

    def own_odom_cb(self, msg):
        self.own_odom = msg
        self.own_pose.header = self.own_odom.header
        self.own_pose.pose = self.own_odom.pose.pose
        self.own_vel.header = self.own_odom.header
        self.own_vel.twist = self.own_odom.twist.twist

    def own_time_ref_cb(self, msg):
        self.own_time_ref = msg

    def other_state_cb(self, msg, src_name):
        if src_name not in self.drones_data:
            self.drones_data[src_name] = {}
        self.drones_data[src_name]['state'] = msg

    def other_odom_cb(self, msg, src_name):
        if src_name not in self.drones_data:
            self.drones_data[src_name] = {}
        self.drones_data[src_name]['odom'] = msg

        if src_name == self.leader_name:
            self.leader_pos_history.append((rospy.Time.now().to_sec(), msg.pose.pose.position))
            if len(self.leader_pos_history) > self.history_max_size:
                self.leader_pos_history.pop(0)

    def other_time_ref_cb(self, msg, src_name):
        if src_name not in self.drones_data:
            self.drones_data[src_name] = {}
        self.drones_data[src_name]['time_ref'] = msg

    def leader_fix_origin_cb(self, msg, src_name):
        if src_name not in self.drones_data:
            self.drones_data[src_name] = {}
        self.drones_data[src_name]['leader_fix_origin'] = msg

    def leader_rcin_cb(self, msg):
        """RC输入回调, 用于模式切换"""
        self.leader_rcin = msg
        if len(self.leader_rcin.channels) > 5:
            ch6 = self.leader_rcin.channels[5]
            if ch6 < 1200:
                self.offb_submode.data = "form"
            elif 1300 <= ch6 <= 1500:
                self.offb_submode.data = "hover"
            elif ch6 >= 1600:
                self.offb_submode.data = "navi"

    def offb_submode_cb(self, msg):
        """子模式订阅回调"""
        self.offb_submode.data = msg.data

    def check_mode_changes(self):
        """检查 mode 和 submode 是否有变化，如有变化则处理"""
        old_mode = self._last_mode
        old_submode = self._last_submode

        mode_changed = self.own_state.mode != old_mode
        submode_changed = self.offb_submode.data != old_submode

        if mode_changed:
            rospy.loginfo(f"Mode: {old_mode} -> {self.own_state.mode}")
            self.hover_pose = None

        if submode_changed:
            rospy.loginfo(f"Submode: {old_submode} -> {self.offb_submode.data}")
            self.hover_pose = None

        if mode_changed or submode_changed:
            rospy.loginfo(f"Mode: {self.own_state.mode}, Submode: {self.offb_submode.data}")

        self._last_mode = self.own_state.mode
        self._last_submode = self.offb_submode.data

    def get_leader_circle_position(self):
        """计算 leader 圆轨迹上的目标位置 [0, 2pi)"""
        if not hasattr(self, 'leader_start_time') or self.leader_start_time is None:
            self.leader_start_time = rospy.Time.now()

        elapsed = (rospy.Time.now() - self.leader_start_time).to_sec()
        angular_vel = self.circle_speed / self.circle_radius

        direction = -1 if self.clockwise else 1
        angle = direction * angular_vel * elapsed
        angle = angle % (2 * math.pi)

        x = self.circle_center[0] + self.circle_radius * math.cos(angle)
        y = self.circle_center[1] + self.circle_radius * math.sin(angle)
        z = self.leader_height

        return np.array([x, y, z])

    # ========== 四层控制算法 ==========
    def cohe_control(self):
        """凝聚控制 - 向邻居中心移动"""
        v_cohe = np.array([0.0, 0.0, 0.0])

        if not self.drones_data:
            return v_cohe

        neighbor_positions = []
        for drone_name, data in self.drones_data.items():
            if drone_name != self.own_name:
                odom = data.get('odom')
                if odom:
                    if hasattr(odom, 'pose'):
                        pos = odom.pose.pose.position
                    else:
                        pos = odom.position
                    neighbor_positions.append(np.array([pos.x, pos.y, pos.z]))

        if neighbor_positions:
            center = np.mean(neighbor_positions, axis=0)
            own_pos = np.array([self.own_odom.pose.pose.position.x,
                               self.own_odom.pose.pose.position.y,
                               self.own_odom.pose.pose.position.z])
            v_cohe = self.p_cohe * (center - own_pos)

        return v_cohe

    def align_control(self):
        """对齐控制 - 速度方向一致"""
        v_align = np.array([0.0, 0.0, 0.0])

        if not self.drones_data:
            return v_align

        neighbor_velocities = []
        for drone_name, data in self.drones_data.items():
            if drone_name != self.own_name:
                odom = data.get('odom')
                if odom:
                    if hasattr(odom, 'twist'):
                        vel = odom.twist.linear
                    else:
                        vel = odom.linear
                    neighbor_velocities.append(np.array([vel.x, vel.y, vel.z]))

        if neighbor_velocities:
            avg_vel = np.mean(neighbor_velocities, axis=0)
            own_vel = np.array([self.own_odom.twist.linear.x,
                               self.own_odom.twist.linear.y,
                               self.own_odom.twist.linear.z])
            v_align = self.p_align * (avg_vel - own_vel)

        return v_align

    def sepa_control(self):
        """分离控制 - 避障"""
        v_sepa = np.array([0.0, 0.0, 0.0])
        own_pos = np.array([self.own_odom.pose.pose.position.x,
                           self.own_odom.pose.pose.position.y,
                           self.own_odom.pose.pose.position.z])

        for obs in self.obstacles:
            obs_pos = np.array([obs['x'], obs['y'], obs['z']])
            dist = np.linalg.norm(own_pos - obs_pos)

            if dist < self.r_sen and dist > 0:
                obs_radius = obs.get('radius', 0.5)
                if dist < obs_radius + self.r_safe:
                    v_sepa += self.p_sepa_obs * (own_pos - obs_pos) / dist

        for drone_name, data in self.drones_data.items():
            if drone_name != self.own_name:
                odom = data.get('odom')
                if odom:
                    if hasattr(odom, 'pose'):
                        pos = odom.pose.pose.position
                    else:
                        pos = odom.position
                    drone_pos = np.array([pos.x, pos.y, pos.z])
                    dist = np.linalg.norm(own_pos - drone_pos)

                    if dist < self.r_sen and dist > 0:
                        if dist < self.r_safe:
                            v_sepa += self.p_sepa_quad * (own_pos - drone_pos) / dist

        return v_sepa

    def flock_control(self):
        """跟随 leader 控制"""
        v_flock = np.array([0.0, 0.0, 0.0])

        if self.leader_name in self.drones_data:
            leader_data = self.drones_data[self.leader_name]
            odom = leader_data.get('odom')
            if odom:
                if hasattr(odom, 'pose'):
                    leader_pos = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z])
                else:
                    leader_pos = np.array([odom.position.x, odom.position.y, odom.position.z])
            else:
                leader_pos = self.get_leader_circle_position()
        else:
            leader_pos = self.get_leader_circle_position()

        own_pos = np.array([self.own_odom.pose.pose.position.x,
                            self.own_odom.pose.pose.position.y,
                            self.own_odom.pose.pose.position.z]) 

        to_leader = leader_pos - own_pos
        dist_to_leader = np.linalg.norm(to_leader)

        if dist_to_leader > 0:
            v_flock[0:2] = to_leader[0:2] * 0.3

        height_diff = self.target_height - own_pos[2]
        v_flock[2] = self.kp_flock_z * height_diff

        return v_flock

    def predict_leader_position(self):
        """预测 leader 未来位置"""
        if not self.leader_pos_history:
            return self.get_leader_circle_position()

        if len(self.leader_pos_history) >= 2:
            t1, pos1 = self.leader_pos_history[-2]
            t2, pos2 = self.leader_pos_history[-1]

            dt = t2 - t1
            if dt > 0:
                vel_x = (pos2.x - pos1.x) / dt
                vel_y = (pos2.y - pos1.y) / dt
                vel_z = (pos2.z - pos1.z) / dt

                pred_x = pos2.x + vel_x * self.prediction_window
                pred_y = pos2.y + vel_y * self.prediction_window
                pred_z = pos2.z + vel_z * self.prediction_window

                return np.array([pred_x, pred_y, pred_z])

        return self.get_leader_circle_position()

    def vel_control(self):
        """速度控制 - 综合四层控制"""
        v_cohe = self.cohe_control()
        v_align = self.align_control()
        v_sepa = self.sepa_control()
        v_flock = self.flock_control()

        desired_v = v_cohe + v_align + v_sepa + v_flock

        v_norm = np.linalg.norm(desired_v)
        if v_norm > self.v_max:
            desired_v = desired_v / v_norm * self.v_max

        current_vel = np.array([self.own_odom.twist.linear.x,
                                self.own_odom.twist.linear.y,
                                self.own_odom.twist.linear.z])
        dv = desired_v - current_vel
        dv_norm = np.linalg.norm(dv)
        if dv_norm > self.dv_max:
            dv = dv / dv_norm * self.dv_max
            desired_v = current_vel + dv

        desired_vel = TwistStamped()
        desired_vel.header.stamp = rospy.Time.now()
        desired_vel.twist.linear.x = desired_v[0]
        desired_vel.twist.linear.y = desired_v[1]
        desired_vel.twist.linear.z = desired_v[2]

        return desired_vel

    # ========== 模式更新 ==========
    def update_hover(self):
        """悬停模式更新"""
        if self.hover_pose is None:
            self.hover_pose = self.own_pose

        hover_msg = PoseStamped()
        hover_msg.header.stamp = rospy.Time.now()
        hover_msg.pose = self.hover_pose.pose
        self.local_pos_pub.publish(hover_msg)

    def update_leader_form(self):
        """Leader 编队模式 - 悬停等待编队形成"""
        if self.own_name != self.leader_name:
            return
        
        self.update_hover()

    def update_leader_navi(self):
        """Leader 导航模式 - 沿圆轨迹飞行"""
        if self.own_name != self.leader_name:
            return

        if self.hover_pose is None:
            self.hover_pose = self.own_pose

        target_pos = self.get_leader_circle_position()

        navi_msg = PoseStamped()
        navi_msg.header.stamp = rospy.Time.now()
        navi_msg.pose.position.x = target_pos[0]
        navi_msg.pose.position.y = target_pos[1]
        navi_msg.pose.position.z = target_pos[2]
        self.local_pos_pub.publish(navi_msg)

    def update_follower_form(self):
        """Follower 编队模式 - 插值法逐渐到达编队中的本机位置（相对leader悬停点）

        误差 > 2m: 全速 vel_form 向目标移动
        误差 0.5-2m: 平滑减速到 1/3*vel_form
        误差 < 0.5m: 直接发布目标位置
        """
        if self.own_name == self.leader_name:
            return

        if self.formation_start_pose is None:
            self.hover_pose = self.own_pose
        
        self.formation_start_pose = self.hover_pose
        rospy.loginfo(f"Start Formation")

        if self.leader_name not in self.drones_data:
            return

        leader_odom = self.drones_data[self.leader_name].get('odom')
        if not leader_odom:
            return

        leader_pos = np.array([
            leader_odom.pose.pose.position.x,
            leader_odom.pose.pose.position.y,
            leader_odom.pose.pose.position.z
        ])

        target_p = leader_pos + self.form_offset

        current_p = np.array([
            self.own_pose.pose.position.x,
            self.own_pose.pose.position.y,
            self.own_pose.pose.position.z
        ])

        p_error = target_p - current_p
        p_error_norm = np.linalg.norm(p_error)

        if p_error_norm > 2.0:
            direction = p_error / p_error_norm
            desired_vel = direction * self.vel_form
        elif p_error_norm > 0.5:
            ratio = (p_error_norm - 0.5) / 1.5
            speed = self.vel_form * (1.0 - 2.0/3.0 * (1.0 - ratio))
            direction = p_error / p_error_norm
            desired_vel = direction * speed
        else:
            navi_msg = PoseStamped()
            navi_msg.header.stamp = rospy.Time.now()
            navi_msg.pose.position.x = target_p[0]
            navi_msg.pose.position.y = target_p[1]
            navi_msg.pose.position.z = max(target_p[2], self.min_height)
            self.local_pos_pub.publish(navi_msg)
            return

        desired_p = current_p + desired_vel / self.control_rate

        navi_msg = PoseStamped()
        navi_msg.header.stamp = rospy.Time.now()
        navi_msg.pose.position.x = desired_p[0]
        navi_msg.pose.position.y = desired_p[1]
        navi_msg.pose.position.z = max(desired_p[2], self.min_height)
        self.local_pos_pub.publish(navi_msg)

    def update_follower_navi(self):
        """Follower 导航模式 - 基于行为算法跟随 leader"""
        if self.hover_pose is None:
            self.hover_pose = self.own_pose

        desired_vel = self.vel_control()
        self.local_vel_pub.publish(desired_vel)

    def run_state_machine(self):
        """状态机主循环
        统一子模式: form(编队), hover(悬停), navi(导航)
        - Leader: form 悬停等待follower编队, navi 沿固定航迹飞行
        - Follower: form 形成编队, navi 导航跟随
        - hover 为过渡模式, 悬停当前点
        """
        while not rospy.is_shutdown():
            self.check_mode_changes()

            if self.own_state.mode == "OFFBOARD":
                if self.offb_submode.data == "hover":
                    self.update_hover()
                elif self.offb_submode.data == "form":
                    if self.own_name == self.leader_name:
                        self.update_leader_form()
                    else:
                        self.update_follower_form()
                elif self.offb_submode.data == "navi":
                    if self.own_name == self.leader_name:
                        self.update_leader_navi()
                    else:
                        self.update_follower_navi()
                else:
                    self.update_hover()
            else:
                self.update_hover()

            self.rate.sleep()

def main():
    try:
        SafeFlockController()
    except rospy.ROSInterruptException:
        pass
    except RuntimeError as e:
        rospy.logerr(f"Init failed: {e}")

if __name__ == '__main__':
    main()