"""controller.py — 变道控制器（横向路径跟踪 + 纵向速度 PID）

对应课程报告 4.4 节 Lane Change Controller：
  - 横向：融合虚拟车道中心（横向偏差）+ 预瞄曲率（期望航向），输出转向角
  - 纵向：速度跟踪，并限制纵向急动度（加加速度）
此处用几何/反馈控制近似 MPC 的协同效果，便于无 MATLAB 环境复现。
"""

import numpy as np


class LaneChangeController:
    def __init__(self, L=2.7, K_yaw=1.2, K_lat=0.9, max_steer=np.deg2rad(35)):
        self.L = L
        self.K_yaw = K_yaw
        self.K_lat = K_lat
        self.max_steer = max_steer

    def lateral_control(self, v, y, yaw, y_des, y_des_dot):
        """横向（转向）控制。

        v        : 当前纵向速度 (m/s)
        y, yaw   : 当前横向位置与航向
        y_des    : 期望横向位置（来自 Frenet 轨迹）
        y_des_dot: 期望横向速度
        返回转向角 (rad)，正=左转。
        """
        desired_yaw = np.arctan2(y_des_dot, max(v, 1e-3))
        e_yaw = desired_yaw - yaw          # 航向误差（需增大航向时为正）
        e_y = y_des - y                    # 横向位置误差
        steer = self.K_yaw * e_yaw + self.K_lat * e_y / max(v, 1e-3)
        return float(np.clip(steer, -self.max_steer, self.max_steer))

    def longitudinal_control(self, v, v_des, prev_accel, dt,
                              max_accel=5.0, max_jerk=5.0):
        """纵向（加速度）控制，含急动度限制。

        v, v_des : 当前/目标速度
        prev_accel : 上一步加速度（用于计算 jerk）
        返回限幅后的加速度指令 (m/s^2)。
        """
        accel_cmd = 1.0 * (v_des - v)
        accel_cmd = float(np.clip(accel_cmd, -max_accel, max_accel))
        # 限制纵向急动度 |da/dt| <= max_jerk
        da = accel_cmd - prev_accel
        if abs(da) / dt > max_jerk:
            accel_cmd = prev_accel + np.sign(da) * max_jerk * dt
        return float(np.clip(accel_cmd, -max_accel, max_accel))
