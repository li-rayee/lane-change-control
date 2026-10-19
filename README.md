# 高速路变道控制 / Highway Lane Change Control

深圳大学《机器人学》课程项目 · 李瑞宇 · 电子信息工程 · 2025.06
关键词：自动驾驶 · 轨迹规划 · Frenet 坐标系 · MPC 路径跟踪 · 安全性验证

这个项目就做一件事：在动态交通流里实时决定变道时机、生成既平滑又满足车辆运动学约束的轨迹，再用模型预测控制（MPC）精准跟踪，最后用量化指标卡一遍安全合规。

仿真站在 MATLAB/Simulink 加 Automated Driving、Navigation、Model Predictive Control 三个工具箱上，用的是 MathWorks 官方示例模型 `Highway Lane Change Planner And Controller Test Bench`。仓库里同时放了 `matlab/` 和 `python/` 两套：`matlab/` 是课程报告里的真实测试台架和可视化代码（得有 MATLAB 授权），`python/` 是不依赖 MATLAB 就能跑的复现版——Frenet 轨迹生成、路径跟踪控制器、自行车模型、指标评估都在里面，适合快速吃透算法再做二次开发。

## 背景

自动驾驶这块，高速变道这种动作的规划和控制算法直接关系行车安全和通行效率，一直是重点。我在这个项目里要做的是一套能实时处理复杂交通环境的变道系统：直道超车、弯道变道、还能接高精度地图模拟真实路网。Frenet 坐标系把轨迹计算压到很轻量，弯道里的曲率变化也就稳住了。

## 核心问题

| 问题 | 挑战 | 本项目解法 |
|---|---|---|
| 复杂场景决策 | 动态交通流里实时决定变道时机（超车、避障），平衡安全和效率 | 行为层（CC/LCF/LC）状态机加 MIO（Most Important Object）识别 |
| 轨迹规划挑战 | 平滑路径要满足运动学约束（最大转向角、横向加速度），在 Frenet 下做轨迹优化 | Frenet 坐标系五次多项式拟合加多目标成本函数 |
| 控制鲁棒性 | 精准跟踪非线性轨迹，扛住弯道离心力和路面附着变化 | 纵向横向协同的 MPC 控制 |
| 安全验证 | 功能安全要求：TimeGap、急动度（Jerk）、碰撞风险 | MetricsAssessment 子系统全维度量化评估 |

## 系统架构

模型由六个子系统闭环组成。传感器与环境先读场景，规划器算完轨迹交给控制器，控制器输出转向角和加速度指令给车辆动力学，车辆动力学再喂回闭环，最后指标评估那一层把安全指标量出来。

1. Scenario and Environment：从 `drivingScenario` 对象读道路场景、参与者和地图数据。
2. Planner Configuration Parameters：规划器、控制器、车辆参数和行为模式开关，见 `matlab/planner_config.m`。
3. Highway Lane Change Planner 是核心：Frenet State Converter 把世界坐标换成 Frenet 坐标；Find MIOs 找出最关键障碍物；Terminal State Sampler 加 Motion Prediction 采样并预测终端状态；Motion Planner 生成多条候选轨迹，综合成本、可行性、碰撞风险挑出最优路径并输出跟踪点。
4. Lane Change Controller：Virtual Lane Center 加 Preview Curvature 加 Path Following Controller（MPC）一起输出转向角和加速度。
5. Vehicle Dynamics：双轴耦合自行车模型（ISO855 坐标系），输出六自由度状态。
6. Metrics Assessment：CollisionDetection 加 JerkMetrics 接到 VehicleSafety，给出综合安全指标。

## 仿真环境

Navigation Toolbox 负责参考路径生成、Frenet 轨迹优化和碰撞检测（validator）。Model Predictive Control Toolbox 用 MPC 跟踪变道轨迹。Automated Driving Toolbox 用 Driving Scenario Designer 创建和仿真场景。软件版本 MATLAB R2020a 或更新。

## 仿真场景与结果

通过 `helperSLHighwayLaneChangeSetup` 调出 15 个预定义场景，覆盖三类测试环境：直道（`_straightroad`，基础超车决策、紧急切入避撞）、弯道（`_curvedroad`，曲率变化段 Frenet 优化、控制鲁棒性）、高精地图（`_importedfronHDmap`，真实路网全局规划）。场景清单 `LC_01` 到 `LC_15`，慢车超车、故障车绕行、切入制动、连续双变道、匝道汇入都在内。

### LC_15_StopnGo_Curved（弯道启停流）

| 安全性需求 | 目标值 | 仿真结果 |
|---|---|---|
| Longitudinal Jerk | < 5 m/s³ | 始终 ±0.05 m/s³ |
| Lateral Jerk | < 5 m/s³ | 峰值 0.3 m/s³ |
| Time gap | > 0.8 s | 满足 |
| Front/Rear Safety Gap | > 20m / > 5m | 满足 |
| 曲率 Curvature | ≤ 1.0 m⁻¹ | 全程 ±0.02 rad/m |

方向盘转角变化不到 1 度，转向控制把车身摆动压得很干净。

### LC_06_DoubleLaneChange（连续双变道，直道）

横向偏差全程 ±0.2 m，安全阈值 ±0.5 m；转向角波动小于 1 度；纵向急动度 ±0.05 m/s³；横向急动度在变道瞬间峰值 0.3 m/s³，落在 ISO2631 晕动阈值内。系统平稳地完成了两次变道避让。

### LC_11_MergingCar_HighwayEntry（匝道汇入）

汇入车辆切入时保持横向稳定，TimeGap 大于 0.8 秒，横向偏差小于 0.15 m，纵向安全要求满足。

## 讨论：车重对变道的影响

| 质量 | 现象 | 根因 |
|---|---|---|
| 1575 kg（标称） | 正常完成变道 | 动力学参数匹配 |
| 6000 kg（超载） | 行车摇摆、末姿态错误 | 横向急动度尖峰 ±3 m/s³，超 ISO2631 阈值，高惯性矩导致修正性超调 |
| 100 kg（过轻） | 偏航失控、19 秒仿真终止 | 横摆率 18 秒达 −40°/s 极限，横向偏差超 ±0.8 m，模型失配 |

实际跑下来看得很清楚：车重必须落在合理区间，超载或者盲目减重都会把系统带进严重失稳。

## 代码说明

### MATLAB（官方工具箱，需授权）

| 文件 | 内容 |
|---|---|
| `matlab/run_test_case.m` | 测试台架：跑指定场景用例并生成仿真报告（来自报告第 5 章） |
| `matlab/simulate_and_visualize.m` | 逐帧仿真和可视化（来自报告第 6.1 节） |
| `matlab/planner_config.m` | 规划器、控制器、车辆参数和行为模式配置示例 |

跑 MATLAB 代码得装上面三个 Toolbox，还要加载 MathWorks 示例模型 `HLCPlannerAndControllerTestBench`（见 Automated Driving Toolbox 文档）。

### Python 复现（可运行，零商业依赖）

`python/` 用纯 Python 加 numpy 加 matplotlib 把核心算法链路复现了一遍，没有 MATLAB 环境也能学习和做二次开发：

| 文件 | 内容 |
|---|---|
| `python/frenet.py` | Frenet 横向轨迹：五次多项式加多目标成本函数选优 |
| `python/vehicle.py` | 自行车模型（kinematic bicycle）车辆动力学 |
| `python/controller.py` | 横向（路径跟踪）加纵向（速度 PID）协同控制器 |
| `python/metrics.py` | 安全性指标：横向偏差、纵横急动度、TimeGap、TTC、安全间隙 |
| `python/scenario.py` | 场景定义（车道、慢车领跑、快车后随） |
| `python/main.py` | 主仿真：触发变道、闭环控制、输出指标和曲线图 |

## 快速运行（Python）

```bash
cd python
pip install -r requirements.txt
python main.py
```

脚本会打出变道全过程的指标——横向偏差、急动度、TimeGap 这些，并保存 `lane_change_result.png` 和 `lane_change_log.csv`，把轨迹平滑、急动度受控、安全间隙达标这件事直观摊开。

## 安全与合规说明

这个仓库是课程学习成果展示，代码和数据都是脱敏或示意版本，不含任何真实路测或企业数据。MATLAB 相关的模型和 helper 函数版权归 MathWorks，遵守它的许可证；仓库里的 MATLAB 代码只作学习参考，运行需要合法授权。自动驾驶算法严禁直接上实车，只能用于仿真和教学。
