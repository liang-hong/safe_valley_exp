#!/usr/bin/env python3
import numpy as np
import math

class FlockMath:
    def __init__(self, config):
        self.cfg = config

    def get_leader_circle_position(self, start_time, current_time):
        """计算 leader 圆轨迹上的目标位置"""
        elapsed = (current_time - start_time).to_sec()
        angular_vel = self.cfg.circle_speed / self.cfg.circle_radius

        direction = -1 if self.cfg.clockwise else 1
        angle = (direction * angular_vel * elapsed) % (2 * math.pi)

        x = self.cfg.circle_center[0] + self.cfg.circle_radius * math.cos(angle)
        y = self.cfg.circle_center[1] + self.cfg.circle_radius * math.sin(angle)
        z = self.cfg.leader_height

        return np.array([x, y, z])

    def cohe_control(self, own_pos, drones_data):
        """凝聚控制 - 向邻居中心移动"""
        v_cohe = np.array([0.0, 0.0, 0.0])
        neighbor_positions = []
        
        for drone_name, data in drones_data.items():
            odom = data.get('odom')
            if odom:
                pos = odom.pose.pose.position
                neighbor_positions.append(np.array([pos.x, pos.y, pos.z]))

        if neighbor_positions:
            center = np.mean(neighbor_positions, axis=0)
            v_cohe = self.cfg.p_cohe * (center - own_pos)

        return v_cohe

    def align_control(self, own_vel, drones_data):
        """对齐控制 - 速度方向一致"""
        v_align = np.array([0.0, 0.0, 0.0])
        neighbor_velocities = []
        
        for drone_name, data in drones_data.items():
            odom = data.get('odom')
            if odom:
                vel = odom.twist.twist.linear
                neighbor_velocities.append(np.array([vel.x, vel.y, vel.z]))

        if neighbor_velocities:
            avg_vel = np.mean(neighbor_velocities, axis=0)
            v_align = self.cfg.p_align * (avg_vel - own_vel)

        return v_align

    def sepa_control(self, own_pos, drones_data, obstacles):
        """分离控制 - 避障"""
        v_sepa = np.array([0.0, 0.0, 0.0])

        # 避开障碍物
        for obs in obstacles:
            obs_pos = np.array([obs['x'], obs['y'], obs['z']])
            dist = np.linalg.norm(own_pos - obs_pos)
            if 0 < dist < self.cfg.r_sen:
                obs_radius = obs.get('radius', 0.5)
                # 越近排斥力越大
                v_sepa += self.cfg.p_sepa_obs * (own_pos - obs_pos) / (dist**2)

        # 避开其他无人机
        for drone_name, data in drones_data.items():
            odom = data.get('odom')
            if odom:
                drone_pos = np.array([odom.pose.pose.position.x, 
                                     odom.pose.pose.position.y, 
                                     odom.pose.pose.position.z])
                dist = np.linalg.norm(own_pos - drone_pos)
                if 0 < dist < self.cfg.r_sen:
                    v_sepa += self.cfg.p_sepa_quad * (own_pos - drone_pos) / (dist**2)

        return v_sepa

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
