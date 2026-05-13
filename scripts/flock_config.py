#!/usr/bin/env python3
import rospy
import socket
import numpy as np

class FlockConfig:
    def __init__(self, own_name_input=None):
        # 1. 确定本机无人机名称（不含/前缀）
        self.own_name = self.determine_own_name(own_name_input)
        
        # 2. 从参数服务器加载配置
        self.load_config()

    def determine_own_name(self, own_name_input):
        if own_name_input is not None:
            name = own_name_input.strip('/')
        else:
            hostname = socket.gethostname()
            if "UAV" in hostname and any(c.isdigit() for c in hostname):
                name = hostname.strip('/')
            else:
                own_name_param = rospy.get_param('~own_name', None)
                if own_name_param is not None:
                    name = own_name_param.strip('/')
                else:
                    rospy.logerr(f"own_name unresolved: hostname '{hostname}'")
                    raise RuntimeError("own_name not resolved")
        rospy.loginfo(f"[Config] Own UAV name: {name}")
        return name

    def load_config(self):
        # 从 flock.yaml 加载集群配置和算法参数
        self.init_positions = rospy.get_param('~init_position', {})
        self.control_params = rospy.get_param('~control', {})
        self.leader_params = rospy.get_param('~leader', {})
        self.topics = rospy.get_param('~topics', [])
        self.topology = rospy.get_param('~topology', {})
        self.obstacles = rospy.get_param('~obstacles', [])

        self.leader_name = self.leader_params.get('name', 'UAV6').strip('/')
        self.leader_topics = self.leader_params.get('leader_topics', [])

        # 控制率
        self.control_rate = self.control_params.get('rate', 30.0)
        self.dt = 1.0 / self.control_rate

        # 算法参数映射
        self.r_safe = self.control_params.get('r_safe', 0.5)
        self.r_pack = self.control_params.get('r_pack', 2.1)
        self.phi_sc = self.control_params.get('phi_sc', 0.523)
        self.p_cohe = self.control_params.get('p_cohe', 1.5)

        self.a_max = self.control_params.get('a_max', 5.0)
        self.v_max = self.control_params.get('v_max', 1.5)
        self.dv_max = self.control_params.get('dv_max', 0.167)
        self.r_align = self.control_params.get('r_align', 10.0)
        self.p_align = self.control_params.get('p_align', 0.6)

        self.b = self.control_params.get('b', 1.0)
        self.r_sen = self.control_params.get('r_sen', 5.0)
        self.e_max = self.control_params.get('e_max', 0.95)
        self.p_sepa_quad = self.control_params.get('p_sepa_quad', 0.3)
        self.p_sepa_obs = self.control_params.get('p_sepa_obs', 0.05)

        self.target_height = self.control_params.get('target_height', 2.0)
        self.kp_flock_z = self.control_params.get('kp_flock_z', 0.5)
        self.max_climb_vel = self.control_params.get('max_climb_vel', 1.0)

        self.vel_form = self.control_params.get('vel_form', 2.0)
        self.kp_form = self.control_params.get('kp_form', 0.5)
        self.kd_form = self.control_params.get('kd_form', 0.1)
        self.min_height = self.control_params.get('min_height', 2.0)

        # 本机编队偏移
        if self.own_name in self.init_positions:
            self.form_offset = np.array(self.init_positions[self.own_name])
        else:
            self.form_offset = np.array([0.0, 0.0, 0.0])

        # Leader 轨迹参数
        self.leader_height = self.leader_params.get('leader_height', 5.0)
        self.circle_center = np.array(self.leader_params.get('circle_center', [10.0, 0.0]))
        self.circle_radius = self.leader_params.get('circle_radius', 10.0)
        self.circle_speed = self.leader_params.get('circle_speed', 0.5)
        self.clockwise = self.leader_params.get('clockwise', True)
        self.hover_on_start = self.leader_params.get('hover_on_start', True)
        self.submode_channel = self.leader_params.get('submode_channel', 6)

        # 预测窗口
        self.prediction_window = 0.1
        self.history_max_size = int(self.prediction_window * self.control_rate)
