"""scenario.py — 高速变道场景定义

对应技术报告 5.1 节 Scenario_LC_15_StopnGo_Curved 的简化示意：
  - 3 车道高速，车道宽 3.5 m
  - 本车（ego）起始位于中间车道，速度 15 m/s
  - 前方同车道有一辆慢车（10 m/s），触发向右变道避让
  - 左侧车道有一辆快车后随（34 m/s），故变道目标选右侧车道
"""


class Scenario:
    def __init__(self, lane_width=3.5, n_lanes=3):
        self.lane_width = lane_width
        self.n_lanes = n_lanes
        # 车道中心 y 坐标：lane 0 最左，lane n-1 最右
        self.lane_centers = [(i - (n_lanes - 1) / 2) * lane_width
                             for i in range(n_lanes)]
        # 慢车领跑（中间车道，最危险）
        self.lead = dict(lane=1, x0=80.0, v=10.0)
        # 快车后随（左侧车道）
        self.fast_rear = dict(lane=0, x0=-60.0, v=34.0)
        self.ego_start_lane = 1
        self.target_lane = 2  # 向右变道，远离快车

    def lane_center_y(self, lane):
        return self.lane_centers[lane]

    def nearest_lane(self, y):
        idx = int(round(y / self.lane_width + (self.n_lanes - 1) / 2))
        return max(0, min(self.n_lanes - 1, idx))

    def obstacle_positions(self, t):
        lead_x = self.lead["x0"] + self.lead["v"] * t
        lead_y = self.lane_center_y(self.lead["lane"])
        rear_x = self.fast_rear["x0"] + self.fast_rear["v"] * t
        rear_y = self.lane_center_y(self.fast_rear["lane"])
        return dict(lead=(lead_x, lead_y, self.lead["v"]),
                    rear=(rear_x, rear_y, self.fast_rear["v"]))
