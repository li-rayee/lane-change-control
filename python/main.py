"""main.py — 高速路变道控制主仿真（Python 复现）

流程：感知（障碍物）-> 规划（Frenet 横向轨迹）-> 控制（转向/加速度）
      -> 动力学（自行车模型）-> 指标评估（MetricsAssessment）

运行：
    pip install -r requirements.txt
    python main.py
输出：
    - 控制台打印安全性指标判定
    - lane_change_result.png （横向位置 / 转向角 / 横摆角 曲线）
    - lane_change_log.csv     （逐帧数据）
"""

import numpy as np
import matplotlib
matplotlib.use("Agg")  # 无显示环境下保存图片
import matplotlib.pyplot as plt

from frenet import plan_lateral_trajectory
from vehicle import BicycleModel
from controller import LaneChangeController
from metrics import MetricsAssessment, SAFETY
from scenario import Scenario


def main():
    dt = 0.05
    T_total = 18.0

    sc = Scenario()
    ego_y0 = sc.lane_center_y(sc.ego_start_lane)   # 0（中间车道）
    target_y = sc.lane_center_y(sc.target_lane)    # +3.5（右侧车道）

    veh = BicycleModel(L=2.7, dt=dt)
    veh.reset(x=0.0, y=ego_y0, yaw=0.0, v=15.0)
    ctrl = LaneChangeController(L=2.7)
    metrics = MetricsAssessment()

    v_des = 20.0          # 规划器设定速度（报告 SetSpeed=20 m/s）
    trigger_t = 2.0       # 感知到慢车后，规划器决策变道时刻
    change_dur = 4.0      # 变道时长

    traj = None
    t_change_start = None
    prev_accel = 0.0
    log = []

    for k in range(int(T_total / dt)):
        t = k * dt
        x, y, yaw, v = veh.state

        # ---- 规划：触发后生成 Frenet 横向轨迹 ----
        if t >= trigger_t and traj is None:
            t_change_start = t
            traj = plan_lateral_trajectory(ego_y0, target_y, change_dur,
                                           v0=0.0, v1=0.0)
        if traj is not None:
            tt = t - t_change_start
            if tt <= change_dur:
                y_des = traj.eval(tt)
                y_des_dot = traj.eval_dot(tt)
                changed = False
            else:
                y_des, y_des_dot = target_y, 0.0
                changed = True
        else:
            y_des, y_des_dot = ego_y0, 0.0
            changed = False

        # ---- 控制 ----
        steer = ctrl.lateral_control(v, y, yaw, y_des, y_des_dot)
        accel = ctrl.longitudinal_control(v, v_des, prev_accel, dt, max_jerk=2.0)
        prev_accel = accel

        # ---- 动力学 ----
        out = veh.step(steer, accel)
        lat_acc = out["lat_acc"]

        # ---- 指标 ----
        # 相对"当前目标车道中心"的横向偏差
        current_target = target_y if changed else (y_des if traj else ego_y0)
        lat_dev = abs(y - current_target)

        # 仅当障碍物与 ego 处于同一车道时才计算 TimeGap / TTC / 安全间隙
        obs = sc.obstacle_positions(t)
        ego_lane = sc.nearest_lane(y)
        time_gap = ttc = 999.0
        front_gap = 999.0
        for name, (ox, oy, ov) in obs.items():
            if sc.nearest_lane(oy) == ego_lane:
                gap = ox - x
                if gap > 0:
                    front_gap = min(front_gap, gap)
                    time_gap = min(time_gap, gap / max(v, 1e-3))
                    if (v - ov) > 0:
                        ttc = min(ttc, gap / (v - ov))

        metrics.record(t, lat_dev, accel, lat_acc, time_gap, ttc, front_gap)
        log.append([t, x, y, yaw, v, steer, accel, lat_dev, time_gap])

    # ---- 结果输出 ----
    s = metrics.summary()
    print("=" * 50)
    print("高速路变道控制 — Python 复现仿真结果")
    print("=" * 50)
    print(f"最大横向偏差      : {s['max_lat_dev']:.3f} m   (阈值 {SAFETY['lat_dev_max']} m)  "
          f"{'OK' if s['lat_dev_ok'] else 'FAIL'}")
    print(f"最大纵向急动度    : {s['max_lon_jerk']:.3f} m/s^3 (阈值 {SAFETY['lon_jerk_max']})  "
          f"{'OK' if s['lon_jerk_ok'] else 'FAIL'}")
    print(f"最大横向急动度    : {s['max_lat_jerk']:.3f} m/s^3 (阈值 {SAFETY['lat_jerk_max']})  "
          f"{'OK' if s['lat_jerk_ok'] else 'FAIL'}")
    print(f"最小 TimeGap      : {s['min_time_gap']:.3f} s   (阈值 >{SAFETY['time_gap_min']})  "
          f"{'OK' if s['time_gap_ok'] else 'FAIL'}")
    print(f"最小前车安全间隙  : {s['min_front_gap']:.3f} m   (阈值 >{SAFETY['front_gap_min']})  "
          f"{'OK' if s['front_gap_ok'] else 'FAIL'}")
    print("=" * 50)

    # ---- 绘图 ----
    arr = np.array(log)
    fig, ax = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    ax[0].plot(arr[:, 0], arr[:, 2], label="y (lateral)")
    ax[0].axhline(target_y, color="r", ls="--", label="target lane center")
    ax[0].axvline(trigger_t, color="g", ls=":", label="lane-change trigger")
    ax[0].set_ylabel("Lateral y (m)")
    ax[0].legend(loc="upper left")
    ax[1].plot(arr[:, 0], np.degrees(arr[:, 5]), label="steering (deg)")
    ax[1].set_ylabel("Steering (deg)")
    ax[1].legend(loc="upper left")
    ax[2].plot(arr[:, 0], np.degrees(arr[:, 3]), label="yaw (deg)")
    ax[2].set_xlabel("Time (s)")
    ax[2].set_ylabel("Yaw (deg)")
    ax[2].legend(loc="upper left")
    fig.suptitle("Highway Lane Change — Python Reproduce")
    fig.tight_layout()
    fig.savefig("lane_change_result.png", dpi=120)
    print("已保存 lane_change_result.png")

    np.savetxt("lane_change_log.csv", arr, delimiter=",",
               header="t,x,y,yaw,v,steering,accel,lat_dev,time_gap",
               comments="")
    print("已保存 lane_change_log.csv")


if __name__ == "__main__":
    main()
