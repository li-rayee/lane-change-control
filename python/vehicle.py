"""vehicle.py — 车辆动力学（自行车模型）

对应课程报告 4.5 节 Vehicle Dynamics（双轴耦合自行车模型 / ISO855 坐标系）。
这里采用运动学自行车模型，输出本车六自由度状态中的关键项：
  x, y（位置）、yaw（横摆角）、v（速度），以及横向加速度（用于急动度评估）。
"""

import numpy as np


class BicycleModel:
    def __init__(self, L=2.7, dt=0.05):
        self.L = L        # 轴距 (m)
        self.dt = dt      # 仿真步长 (s)
        # 状态: [x, y, yaw(rad), v(m/s)]
        self.state = np.array([0.0, 0.0, 0.0, 0.0])

    def reset(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.state = np.array([x, y, yaw, v], dtype=float)

    def step(self, steering, accel, max_steer=np.deg2rad(35), max_accel=5.0):
        """前向积分一步。

        steering : 方向盘转角 (rad)，正=左转
        accel    : 纵向加速度指令 (m/s^2)
        返回当前步的物理量字典。
        """
        steering = float(np.clip(steering, -max_steer, max_steer))
        accel = float(np.clip(accel, -max_accel, max_accel))

        x, y, yaw, v = self.state
        v = max(v + accel * self.dt, 0.0)                 # 速度非负
        yaw += (v / self.L) * np.tan(steering) * self.dt  # 横摆角更新
        x += v * np.cos(yaw) * self.dt
        y += v * np.sin(yaw) * self.dt
        self.state = np.array([x, y, yaw, v])

        # 横向加速度（用于 MetricsAssessment）：a_lat = v^2 * curvature
        curvature = np.tan(steering) / self.L
        lat_acc = v * v * curvature
        return dict(x=x, y=y, yaw=yaw, v=v, lat_acc=lat_acc)


if __name__ == "__main__":
    veh = BicycleModel(dt=0.1)
    veh.reset(yaw=0.0, v=15.0)
    for _ in range(50):  # 持续左转，应走出圆弧
        veh.step(steering=np.deg2rad(5), accel=0.0)
    x, y, yaw, v = veh.state
    print("50 步后: x=%.2f y=%.2f yaw=%.2f rad v=%.2f" % (x, y, yaw, v))
