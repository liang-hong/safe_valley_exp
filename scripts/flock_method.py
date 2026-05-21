#!/usr/bin/env python3
import numpy as np
import math
import rospy

class FlockMethod:
    def __init__(self, config):
        self.cfg = config

    def get_leader_circle_position(self, start_time, current_time):
        """计算 leader 圆轨迹上的目标位置"""
        elapsed = (current_time - start_time).to_sec()
        angular_vel = self.cfg.circle_speed / self.cfg.circle_radius

        direction = -1 if self.cfg.clockwise else 1
        angle = (direction * angular_vel * elapsed + math.pi) % (2 * math.pi)

        x = self.cfg.circle_center[0] + self.cfg.circle_radius * math.cos(angle)
        y = self.cfg.circle_center[1] + self.cfg.circle_radius * math.sin(angle)
        z = 0.0

        return np.array([x, y, z])

    def cohe_control(self, own_pos, leader_pos):
        """凝聚控制 - 向 Leader 的垂直投影靠拢"""
        v_cohe = np.array([0.0, 0.0, 0.0])
        
        # 计算水平向量和水平距离
        r_i0_xy = own_pos[0:2] - leader_pos[0:2]
        dist_xy = np.linalg.norm(r_i0_xy)
        
        if self.cfg.r_in_floc < dist_xy < self.cfg.r_out_floc:
            gain = ((dist_xy - self.cfg.r_in_floc) / (self.cfg.r_out_floc - self.cfg.r_in_floc))**(0.5)
            v_cohe[0:2] = - self.cfg.p_cohe * gain * r_i0_xy / dist_xy
            rospy.loginfo(f"cohe_control enable")
            
        return v_cohe

    def flock_control(self, own_pos, leader_pos, leader_vel):
        """跟随控制 - 匹配 Leader 水平速度并控制到目标高度"""
        # 计算水平距离
        r_i0_xy = own_pos[0:2] - leader_pos[0:2]
        dist_xy = np.linalg.norm(r_i0_xy)
        
        # 垂直控制：驱动无人机到达配置文件中的 target_height
        z_error = own_pos[2] - self.cfg.target_height
        v_z = - self.cfg.kp_flock_z * z_error
        v_z = np.clip(v_z, -self.cfg.max_climb_vel, self.cfg.max_climb_vel)
        
        if self.cfg.r_in_floc < dist_xy < self.cfg.r_out_floc:
            # 在有效半径内，匹配 Leader 的水平速度
            v_flock = np.array([
                leader_vel[0],
                leader_vel[1],
                v_z
            ])
            rospy.loginfo(f"flock_control enable")
        else:
            # 半径外仅保持高度控制
            v_flock = np.array([0.0, 0.0, v_z])
            
        return v_flock

    def align_control(self, own_pos, own_vel, drones_data, neighbor_names):
        """对齐控制 - 仿真高精度版本 (参考仿真 L389-423)"""
        v_align = np.array([0.0, 0.0, 0.0])
        
        for name in neighbor_names:
            data = drones_data.get(name)
            if not (data and 'odom' in data):
                continue
                
            r_j = np.array([
                data['odom'].pose.pose.position.x,
                data['odom'].pose.pose.position.y,
                data['odom'].pose.pose.position.z
            ])
            r_ij = own_pos - r_j
            r_ij_norm = np.linalg.norm(r_ij)
            
            # 计算该邻居对应的最大允许相对速度 v_ij_max
            if r_ij_norm <= 2.0 * self.cfg.r_safe:
                v_ij_max = 0.0
            elif r_ij_norm <= self.cfg.r_res:
                v_ij_max = self.cfg.v_res
            elif r_ij_norm < self.cfg.r_align:
                # 刹车距离公式：v = sqrt(2*a*d)
                v_ij_max = np.sqrt(max(self.cfg.v_res**2, 2 * self.cfg.a_max * (r_ij_norm - self.cfg.r_res)))
            else:
                continue
            
            v_j = np.array([
                data['odom'].twist.twist.linear.x,
                data['odom'].twist.twist.linear.y,
                data['odom'].twist.twist.linear.z
            ])
            v_ij = own_vel - v_j
            v_ij_norm = np.linalg.norm(v_ij)
            
            # 如果相对速度超过阈值，产生对齐拉力
            if v_ij_norm > v_ij_max:
                gain = (v_ij_norm - v_ij_max) / (2 * self.cfg.v_max - v_ij_max)
                v_align += - self.cfg.p_align * gain * v_ij / v_ij_norm
                rospy.loginfo(f"align_control enable for {name}")
        
        return v_align

    def sepa_control(self, own_pos, own_vel, own_ori, drones_data, obstacles, leader_name):
        """分离控制 - 仿真高精度椭圆模型 (参考仿真 L426-523)"""
        v_sepa = np.array([0.0, 0.0, 0.0])
        
        # 1. 确定本机前进方向和离心率
        v_i_norm = np.linalg.norm(own_vel)
        if v_i_norm < 0.05 * self.cfg.v_max:
            # 速度过小时使用机头指向 (由四元数计算 R[:3,0])
            x, y, z, w = own_ori.x, own_ori.y, own_ori.z, own_ori.w
            v_i_hat = np.array([
                1 - 2*y**2 - 2*z**2,
                2*x*y + 2*z*w,
                2*x*z - 2*y*w
            ])
            e_i = 0.05
        else:
            v_i_hat = own_vel / v_i_norm
            e_i = self.cfg.e_max * v_i_norm / self.cfg.v_max
            
        # 2. 避开其他无人机 (排除虚拟 Leader)
        v_sepa_quad = np.array([0.0, 0.0, 0.0])
        for drone_name, data in drones_data.items():
            if drone_name == leader_name or 'odom' not in data:
                continue
                
            r_quad = np.array([
                data['odom'].pose.pose.position.x, 
                data['odom'].pose.pose.position.y, 
                data['odom'].pose.pose.position.z
            ])
            r_iquad = own_pos - r_quad
            r_iquad_norm = np.linalg.norm(r_iquad)
            if r_iquad_norm < 1e-3: continue
            
            r_iquad_hat = r_iquad / r_iquad_norm
            cos_theta = np.dot(v_i_hat, -r_iquad_hat)
            
            # 椭圆安全边界计算
            r_off = 2.0 * self.cfg.r_safe
            r_free = self.cfg.b * np.sqrt(1 - e_i**2) / (1 - e_i * cos_theta) + r_off
            
            if r_iquad_norm < r_free:
                r_free_min = self.cfg.b * np.sqrt(1 - e_i**2) / (1 + e_i)
                # 避障受力方向 (垂直于相对方向的逃逸向量)
                perp_vec = v_i_hat + r_iquad_hat
                perp_norm = np.linalg.norm(perp_vec)
                
                if perp_norm < 1e-3:
                    # 【打破平衡】共线对冲情况：速度正对障碍物中心，perp_vec 抵消为 0
                    # 尝试与 Z 轴叉乘得到一个水平侧向逃逸方向
                    side_vec = np.cross(v_i_hat, [0, 0, 1])
                    if np.linalg.norm(side_vec) < 1e-3:
                        # 如果本身就是垂直飞行，则与 X 轴叉乘得到逃逸方向
                        side_vec = np.cross(v_i_hat, [1, 0, 0])
                    perp_vec = side_vec
                    perp_norm = np.linalg.norm(perp_vec)
                
                v_sepa_quad += self.cfg.p_sepa_quad * np.sqrt((r_free - r_iquad_norm) / r_free_min) * (perp_vec / perp_norm)
                rospy.loginfo(f"sepa_quad enable for {drone_name}")

        # 3. 避开障碍物
        v_sepa_obs = np.array([0.0, 0.0, 0.0])
        for obs in obstacles:
            # 简化为球体：Z 轴强制设定为巡航高度 target_height，不考虑圆柱实际高度
            r_obs = np.array([obs['x'], obs['y'], self.cfg.target_height])
            r_iobs = own_pos - r_obs
            r_iobs_norm = np.linalg.norm(r_iobs)
            if r_iobs_norm > self.cfg.r_sen or r_iobs_norm < 1e-3:
                continue
                
            r_iobs_hat = r_iobs / r_iobs_norm
            cos_theta = np.dot(v_i_hat, -r_iobs_hat)
            
            r_off = self.cfg.r_safe + obs.get('radius', 0.5)
            r_free = self.cfg.b * np.sqrt(1 - e_i**2) / (1 - e_i * cos_theta) + r_off
            
            if r_iobs_norm < r_free:
                r_free_min = self.cfg.b * np.sqrt(1 - e_i**2) / (1 + e_i)
                perp_vec = v_i_hat + r_iobs_hat
                perp_norm = np.linalg.norm(perp_vec)
                
                if perp_norm < 1e-3:
                    # 【打破平衡】共线对冲情况
                    side_vec = np.cross(v_i_hat, [0, 0, 1])
                    if np.linalg.norm(side_vec) < 1e-3:
                        side_vec = np.cross(v_i_hat, [1, 0, 0])
                    perp_vec = side_vec
                    perp_norm = np.linalg.norm(perp_vec)
                
                v_sepa_obs += self.cfg.p_sepa_obs * np.sqrt((r_free - r_iobs_norm) / r_free_min) * (perp_vec / perp_norm)
                rospy.loginfo(f"sepa_obs enable for {obs['id']}")

        return v_sepa_quad + v_sepa_obs

    # def predict_leader_position(self, history, current_time):
    #     """预测 leader 未来位置"""
    #     if not history:
    #         return None

    #     if len(history) >= 2:
    #         t1, pos1 = history[-2]
    #         t2, pos2 = history[-1]
    #         dt = t2 - t1
    #         if dt > 0:
    #             vel = np.array([(pos2.x - pos1.x)/dt, (pos2.y - pos1.y)/dt, (pos2.z - pos1.z)/dt])
    #             pred_pos = np.array([pos2.x, pos2.y, pos2.z]) + vel * self.cfg.prediction_window
    #             return pred_pos
        
    #     last_pos = history[-1][1]
    #     return np.array([last_pos.x, last_pos.y, last_pos.z])

    def apply_limits(self, desired_v, current_v):
        """应用速度和加速度限制"""
        # 速度幅值限制
        v_norm = np.linalg.norm(desired_v)
        if v_norm > self.cfg.v_max:
            desired_v = desired_v / v_norm * self.cfg.v_max

        # 加速度（速度变化量）限制
        dv = desired_v - current_v
        dv_norm = np.linalg.norm(dv)
        if dv_norm > self.cfg.dv_max:
            dv = dv / dv_norm * self.cfg.dv_max
            desired_v = current_v + dv
            
        return desired_v
