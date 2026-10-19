"""frenet.py — Frenet 坐标系横向轨迹生成（五次多项式）

对应技术报告 4.3 节 Highway Lane Change Planner：
  - 在道路参考系（Frenet）下，用五次多项式拟合横向位移 d(t)
  - 给定起止位置/速度/加速度边界条件，求解系数
  - 结合多目标成本函数（横向偏差 / 时间 / 速度跟踪）可生成候选轨迹并选优
"""

import numpy as np


class QuinticPolynomial:
    """d(t) = a0 + a1 t + a2 t^2 + a3 t^3 + a4 t^4 + a5 t^5

    给定两端的位置、速度、加速度边界条件，求解 6 个系数。
    """

    def __init__(self, d0, d0_dot, d0_ddot, d1, d1_dot, d1_ddot, T):
        self.T = float(T)
        # 边界条件构成的线性方程组 A * coeffs = b
        T_ = self.T
        A = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 2, 0, 0, 0],
            [1, T_, T_**2, T_**3, T_**4, T_**5],
            [0, 1, 2 * T_, 3 * T_**2, 4 * T_**3, 5 * T_**4],
            [0, 0, 2, 6 * T_, 12 * T_**2, 20 * T_**3],
        ], dtype=float)
        b = np.array([d0, d0_dot, d0_ddot, d1, d1_dot, d1_ddot], dtype=float)
        self.coeffs = np.linalg.solve(A, b)  # [a0, a1, a2, a3, a4, a5]

    def _basis(self, t, order):
        t = float(min(t, self.T))
        if order == 0:
            return np.array([1, t, t**2, t**3, t**4, t**5])
        if order == 1:
            return np.array([0, 1, 2 * t, 3 * t**2, 4 * t**3, 5 * t**4])
        if order == 2:
            return np.array([0, 0, 2, 6 * t, 12 * t**2, 20 * t**3])
        if order == 3:  # 加加速度（jerk）
            return np.array([0, 0, 0, 6, 24 * t, 60 * t**2])
        raise ValueError("order must be 0..3")

    def eval(self, t):
        return float(self._basis(t, 0) @ self.coeffs)

    def eval_dot(self, t):
        return float(self._basis(t, 1) @ self.coeffs)

    def eval_ddot(self, t):
        return float(self._basis(t, 2) @ self.coeffs)

    def eval_jerk(self, t):
        return float(self._basis(t, 3) @ self.coeffs)


def plan_lateral_trajectory(d_start, d_end, T, v0=0.0, v1=0.0):
    """生成从 d_start 到 d_end、时长 T、两端速度为 v0/v1 且加速度为 0 的平滑横向轨迹。

    返回 QuinticPolynomial，可调用 eval / eval_dot / eval_jerk 取任意时刻的
    位置、速度、加速度、加加速度。
    """
    return QuinticPolynomial(d_start, v0, 0.0, d_end, v1, 0.0, T)


def evaluate_trajectory_cost(traj, T, lat_dev_cost=1.0, time_cost=-1.0, speed_cost=1.0,
                             v_nominal=20.0):
    """多目标成本函数（对应报告 4.2 CostWeights）。

    - 横向偏差成本：末端位置与期望偏移（这里以 jerk 积分近似平滑度）
    - 时间成本：偏好更短完成时间（time_cost 为负）
    - 速度成本：偏离标称速度的惩罚
    返回标量成本，越小越优。
    """
    ts = np.linspace(0, T, 21)
    jerk_rms = np.sqrt(np.mean([traj.eval_jerk(t) ** 2 for t in ts]))
    cost = (lat_dev_cost * jerk_rms
            + time_cost * T
            + speed_cost * abs(traj.eval_dot(T) - v_nominal) / v_nominal)
    return cost


if __name__ == "__main__":
    # 自测：3.5 m 横向变道，4 s 完成
    traj = plan_lateral_trajectory(0.0, 3.5, 4.0)
    print("t=0.0  d=%.3f  d_dot=%.3f" % (traj.eval(0), traj.eval_dot(0)))
    print("t=2.0  d=%.3f  d_dot=%.3f" % (traj.eval(2), traj.eval_dot(2)))
    print("t=4.0  d=%.3f  d_dot=%.3f" % (traj.eval(4), traj.eval_dot(4)))
    print("成本 = %.4f" % evaluate_trajectory_cost(traj, 4.0))
