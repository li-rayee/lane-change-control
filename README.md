# 高速路变道控制 / Highway Lane Change Control

> 深圳大学《机器人学》课程项目 · 李瑞宇 · 电子信息工程 · 2025.06
> 关键词：自动驾驶 · 轨迹规划 · Frenet 坐标系 · MPC 路径跟踪 · 安全性验证

本项目围绕**高速公路自动驾驶变道**展开：在动态交通流中实时决策变道时机、生成平滑且满足车辆运动学约束的轨迹，并通过模型预测控制（MPC）精准跟踪，最后用量化指标验证安全合规性。

仿真基于 **MATLAB/Simulink + Automated Driving / Navigation / Model Predictive Control Toolbox**，使用 MathWorks 官方示例模型 `Highway Lane Change Planner And Controller Test Bench`。本仓库同时提供：

- `matlab/`：来自课程报告的真实测试台架与可视化代码（需 MATLAB 授权）
- `python/`：**可脱离 MATLAB 运行的 Python 复现版**（Frenet 轨迹生成 + 路径跟踪控制器 + 自行车模型 + 指标评估），适合快速理解算法与做二次开发

---

## 1. 背景

在自动驾驶技术迅猛发展的背景下，高速公路变道动作的规划与控制算法已成为提升行车安全与效率的关键。本项目专注于开发一套高效、可靠的变道系统，核心在于实时处理复杂交通环境，通过仿真和算法优化，确保车辆安全、平滑地完成变道。

- **直道超车**：智能识别关键目标，生成最优轨迹实现左/右侧超车，减少追尾风险；
- **弯道变道**：利用 Frenet 坐标系简化轨迹计算，确保车辆在曲率变化路段稳定变道；
- **可扩展**：支持导入高精度地图，模拟真实道路环境。

---

## 2. 核心问题

| 问题 | 挑战 | 本项目解法 |
|---|---|---|
| **复杂场景决策** | 动态交通流中实时决策变道时机（超车/避障），平衡安全与效率 | 行为层（CC/LCF/LC）状态机 + MIO（Most Important Object）识别 |
| **轨迹规划挑战** | 平滑路径满足运动学约束（最大转向角、横向加速度），Frenet 下轨迹优化 | Frenet 坐标系五次多项式拟合 + 多目标成本函数 |
| **控制鲁棒性** | 精准跟踪非线性轨迹，抵抗弯道离心力、路面附着变化 | 纵向/横向协同 MPC 控制 |
| **安全验证** | 功能安全要求：TimeGap、急动度（Jerk）、碰撞风险 | MetricsAssessment 子系统全维度量化评估 |

---

## 3. 系统架构

模型由 6 个子系统构成闭环控制：

```
传感器与环境 ──▶ 规划器(Highway Lane Change Planner) ──▶ 控制器(Lane Change Controller)
     ▲                                                          │
     │                                                          ▼
     └──────────── 车辆动力学(Vehicle Dynamics) ◀── 转向角 / 加速度指令
                                  │
                                  ▼
                       指标评估(Metrics Assessment)
```

1. **Scenario and Environment**：从 `drivingScenario` 对象读取道路场景、参与者与地图数据。
2. **Planner Configuration Parameters**：规划器/控制器/车辆参数与行为模式开关（见 `matlab/planner_config.m`）。
3. **Highway Lane Change Planner**（核心）：
   - `Frenet State Converter` 将世界坐标 → Frenet 坐标；
   - `Find MIOs` 识别最关键障碍物；
   - `Terminal State Sampler` + `Motion Prediction` 采样并预测终端状态；
   - `Motion Planner` 生成多条候选轨迹，综合**成本/可行性/碰撞风险**选定最优路径并输出跟踪点。
4. **Lane Change Controller**：`Virtual Lane Center` + `Preview Curvature` + `Path Following Controller`（MPC）协同输出转向角与加速度。
5. **Vehicle Dynamics**：双轴耦合自行车模型（ISO855 坐标系），输出六自由度状态。
6. **Metrics Assessment**：`CollisionDetection` + `JerkMetrics` → `VehicleSafety` 综合安全指标。

---

## 4. 仿真环境

- **Navigation Toolbox**：参考路径生成、Frenet 轨迹优化、碰撞检测（validator）。
- **Model Predictive Control Toolbox**：MPC 跟踪变道轨迹。
- **Automated Driving Toolbox**：Driving Scenario Designer 创建/仿真场景。
- **软件版本**：MATLAB R2020a 或更新。

---

## 5. 仿真场景与结果

通过 `helperSLHighwayLaneChangeSetup` 调用 **15 个预定义场景**，覆盖三类测试环境：

- **直道**（`_straightroad`）：基础超车决策、紧急切入避撞
- **弯道**（`_curvedroad`）：曲率变化段 Frenet 优化、控制鲁棒性
- **高精地图**（`_importedfronHDmap`）：真实路网全局规划

场景清单：`LC_01`~`LC_15`（慢车超车、故障车绕行、切入制动、连续双变道、匝道汇入等）。

### 5.1 Scenario_LC_15_StopnGo_Curved（弯道启停流）

| 安全性需求 | 目标值 | 仿真结果 |
|---|---|---|
| Longitudinal Jerk | < 5 m/s³ | 始终 ±0.05 m/s³ ✅ |
| Lateral Jerk | < 5 m/s³ | 峰值 0.3 m/s³ ✅ |
| Time gap | > 0.8 s | 满足 ✅ |
| Front/Rear Safety Gap | > 20m / > 5m | 满足 ✅ |
| 曲率 Curvature | ≤ 1.0 m⁻¹ | 全程 ±0.02 rad/m ✅ |

方向盘转角变化 < 1°，转向控制精准抑制车身摆动。

### 5.2 Scenario_LC_06_DoubleLaneChange（连续双变道，直道）

横向偏差全程 ±0.2 m（安全阈值 ±0.5 m）；转向角波动 < 1°；纵向急动度 ±0.05 m/s³；横向急动度变道瞬间峰值 0.3 m/s³，符合 ISO2631 晕动阈值。系统平稳完成两次变道避让。

### 5.3 Scenario_LC_11_MergingCar_HighwayEntry（匝道汇入）

汇入车辆切入时保持横向稳定性，TimeGap > 0.8 s，横向偏差 < 0.15 m，满足纵向安全要求。

---

## 6. 讨论：车辆质量对变道的影响

| 质量 | 现象 | 根因 |
|---|---|---|
| **1575 kg**（标称） | 正常完成变道 | 动力学参数匹配 |
| **6000 kg**（超载） | 行车摇摆、末姿态错误 | 横向急动度尖峰 ±3 m/s³（超 ISO2631 阈值），高惯性矩导致修正性超调 |
| **100 kg**（过轻） | 偏航失控、19 s 仿真终止 | 横摆率 18 s 达 -40°/s 极限，横向偏差超 ±0.8 m，模型失配 |

结论：**车重必须保持在合理区间**，超载或盲目减重都会引发严重失稳。

---

## 7. 代码说明

### MATLAB（官方工具箱，需授权）

| 文件 | 内容 |
|---|---|
| `matlab/run_test_case.m` | 测试台架：运行指定场景用例并生成仿真报告（来自报告第 5 章） |
| `matlab/simulate_and_visualize.m` | 逐帧仿真与可视化（来自报告第 6.1 节） |
| `matlab/planner_config.m` | 规划器/控制器/车辆参数与行为模式配置示例 |

> 运行 MATLAB 代码需安装上述三个 Toolbox，并加载 MathWorks 示例模型
> `HLCPlannerAndControllerTestBench`（见 Automated Driving Toolbox 文档）。

### Python 复现（可运行，零商业依赖）

`python/` 用纯 Python + numpy + matplotlib 复现了核心算法链路，便于无 MATLAB 环境时
学习与二次开发：

| 文件 | 内容 |
|---|---|
| `python/frenet.py` | Frenet 横向轨迹：五次多项式 + 多目标成本函数选优 |
| `python/vehicle.py` | 自行车模型（kinematic bicycle）车辆动力学 |
| `python/controller.py` | 横向（路径跟踪）+ 纵向（速度 PID）协同控制器 |
| `python/metrics.py` | 安全性指标：横向偏差、纵/横向急动度、TimeGap、TTC、安全间隙 |
| `python/scenario.py` | 场景定义（车道、慢车领跑、快车后随） |
| `python/main.py` | 主仿真：触发变道 → 闭环控制 → 输出指标与曲线图 |

---

## 8. 快速运行（Python）

```bash
cd python
pip install -r requirements.txt
python main.py
```

脚本会输出变道全过程指标（横向偏差、急动度、TimeGap 等）并保存 `lane_change_result.png` 与
`lane_change_log.csv`，直观展示「轨迹平滑、急动度受控、安全间隙达标」。

---

## 9. 安全与合规说明

- 本仓库为**课程学习成果展示**，代码与数据为脱敏/示意版本，不含任何真实路测或企业数据。
- MATLAB 相关模型与 helper 函数版权归 MathWorks 所有，请遵守其许可证；本仓库 MATLAB 代码
  仅作学习参考，运行需合法授权。
- 自动驾驶算法严禁直接用于实车，仅用于仿真与教学。
