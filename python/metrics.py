"""metrics.py — 安全性指标评估

对应技术报告 4.6 节 Metrics Assessment：
  - CollisionDetection：TimeGap = 车距 / 本车速度；TTC 碰撞时间
  - JerkMetrics：纵向急动度 / 横向急动度（加加速度）
  - VehicleSafety：综合安全指标
安全阈值取自报告 5.1~5.3 节需求表。
"""

SAFETY = dict(
    lon_jerk_max=5.0,     # m/s^3
    lat_jerk_max=5.0,     # m/s^3
    time_gap_min=0.8,     # s
    front_gap_min=20.0,   # m
    rear_gap_min=5.0,     # m
    lat_dev_max=0.5,      # m
)


class MetricsAssessment:
    def __init__(self):
        self.log = []   # 每步记录

    def record(self, t, lat_dev, lon_accel, lat_accel, time_gap, ttc, front_gap):
        if self.log:
            dt = t - self.log[-1]["t"]
            lon_jerk = (lon_accel - self.log[-1]["lon_accel"]) / dt if dt > 0 else 0.0
            lat_jerk = (lat_accel - self.log[-1]["lat_accel"]) / dt if dt > 0 else 0.0
        else:
            lon_jerk = lat_jerk = 0.0
        self.log.append(dict(t=t, lat_dev=lat_dev, lon_accel=lon_accel,
                             lat_accel=lat_accel, lon_jerk=lon_jerk,
                             lat_jerk=lat_jerk, time_gap=time_gap,
                             ttc=ttc, front_gap=front_gap))

    def summary(self):
        if not self.log:
            return {}
        lat_dev = [r["lat_dev"] for r in self.log]
        lon_j = [r["lon_jerk"] for r in self.log]
        lat_j = [r["lat_jerk"] for r in self.log]
        tg = [r["time_gap"] for r in self.log]
        fg = [r["front_gap"] for r in self.log]
        return dict(
            max_lat_dev=max(lat_dev),
            max_lon_jerk=max(lon_j),
            max_lat_jerk=max(lat_j),
            min_time_gap=min(tg),
            min_front_gap=min(fg),
            lat_dev_ok=max(lat_dev) <= SAFETY["lat_dev_max"],
            lon_jerk_ok=max(lon_j) <= SAFETY["lon_jerk_max"],
            lat_jerk_ok=max(lat_j) <= SAFETY["lat_jerk_max"],
            time_gap_ok=min(tg) >= SAFETY["time_gap_min"],
            front_gap_ok=min(fg) >= SAFETY["front_gap_min"],
        )
