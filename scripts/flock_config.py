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
        # 优先顺序：1. 构造函数直接传入 > 2. roslaunch 参数 (~own_name) > 3. 物理主机名 (hostname)
        if own_name_input is not None:
            name = own_name_input.strip('/')
        else:
            # 尝试从 ROS 参数服务器获取（通常由 launch 文件传入）
            own_name_param = rospy.get_param('~own_name', None)
            if own_name_param is not None:
                name = own_name_param.strip('/')
            else:
                # 最后尝试使用物理主机名
                hostname = socket.gethostname()
                if "UAV" in hostname and any(c.isdigit() for c in hostname):
                    name = hostname.strip('/')
                else:
                    rospy.logerr(f"own_name unresolved: hostname '{hostname}' and no ~own_name param")
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
        self.r_pack_gain = self.control_params.get('r_pack_gain', 2.1)
        self.phi_sc = self.control_params.get('phi_sc', 0.52359877559)
        self.p_cohe = self.control_params.get('p_cohe', 1.5)

        self.a_max = self.control_params.get('a_max', 5.0)
        self.v_max = self.control_params.get('v_max', 1.5)
        self.dv_max = self.a_max / self.control_rate
        
        # 对齐算法参数补全 (参考仿真 L130-132)
        self.r_res = 2.0 * self.r_safe
        self.v_res = 0.1 * self.v_max
        self.r_align = (2 * self.v_max)**2 / (self.a_max) + 2 * self.r_safe + self.r_res

        self.p_align = self.control_params.get('p_align', 0.6)

        # 避障算法参数补全 (参考仿真 L136-143)
        self.b = self.control_params.get('b', 1.0)
        self.r_sen = self.control_params.get('r_sen', 5.0)
        
        # 动态计算 e_max (解析解取代 sympy)
        # 确保 K > 1，防止 e_max 出现负值或导致除零错误
        K = max(1.001, ((self.r_sen - self.r_safe) / self.b) ** 2)
        self.e_max = (K - 1) / (K + 1)
        
        self.p_sepa_quad = self.control_params.get('p_sepa_quad', 0.3)
        self.p_sepa_obs = self.control_params.get('p_sepa_obs', 0.05)

        self.target_height = self.control_params.get('target_height', 2.0)
        self.kp_flock_z = self.control_params.get('kp_flock_z', 0.5)
        self.max_climb_vel = self.control_params.get('max_climb_vel', 1.0)

        # 凝聚与跟随参数 (基于集群数量动态计算)
        n = len(self.topology) if self.topology else 1
        r_pack = self.r_pack_gain * self.r_safe
        self.r_in_floc = r_pack * (n / self.phi_sc) ** (1.0/3.0)
        self.r_out_floc = 2.0 * self.r_in_floc

        self.vel_form = self.control_params.get('vel_form', 2.0)
        self.kp_form = self.control_params.get('kp_form', 0.5)
        self.kd_form = self.control_params.get('kd_form', 0.1)
        self.min_height = self.control_params.get('min_height', 1.0)

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
