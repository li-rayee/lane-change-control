% planner_config.m
% -------------------------------------------------------------------------
% 高速路变道控制 —— 规划器 / 控制器 / 车辆 参数配置示例
% 整理自技术报告 4.2 节（Planner Configuration Parameters）与 5.1~5.3 节
% 各场景的安全需求表。数值为示例场景（LC_15 / LC_06 / LC_11）的共同配置。
%
% 在 Simulink 模型中，这些变量通过 Constant + BusCreator 注入对应子系统。
% -------------------------------------------------------------------------

% ============ 规划参数 PlanningParameters ============
PlanningParameters.timeResolution = 0.1;   % 轨迹时间分辨率 (s)
PlanningParameters.replanRate     = 10;    % 重规划频率 (Hz)
PlanningParameters.timeHorizon    = 1:3;   % 时间范围窗口 (s)

% ============ 安全参数 SafetyParameters ============
SafetyParameters.frontSafetyGap   = 20;    % 前安全间隙 (m)
SafetyParameters.rearSafetyGap    = 5;     % 后安全间隙 (m)
SafetyParameters.egoFrontExt      = 1;     % 本车碰撞模型前向缩放
SafetyParameters.targetFrontExt   = 1;     % 目标碰撞模型前向缩放
SafetyParameters.egoTTC           = 2;     % 本车道碰撞时间阈值 (s)
SafetyParameters.nextTTC          = 5;     % 邻车道碰撞时间阈值 (s)

% ============ 成本权重 CostWeights ============
CostWeights.latDevCost = 1.0;   % 横向偏差权重
CostWeights.timeCost   = -1.0;  % 通行时间权重（负向=鼓励更快）
CostWeights.speedCost  = 1.0;   % 速度跟踪权重

% ============ 可行性约束 FeasibilityParameters ============
FeasibilityParameters.maxAccel     = 5.0;   % 最大加速度 (m/s^2)
FeasibilityParameters.maxCurvature = 1.0;   % 最大道路曲率 (m^-1)
FeasibilityParameters.maxYawRate   = 80.0;  % 最大横摆角速度 (rad/s)
FeasibilityParameters.maxVelocity  = 20.0;  % 最大车速 (m/s)
FeasibilityParameters.minVelocity  = 0.0;   % 最小车速 (m/s)

% ============ 行为模式开关 BehaviorMode ============
BehaviorMode.enableCCBehavior  = true;  % 巡航控制模式
BehaviorMode.enableLCFBehavior = true;  % 跟车模式
BehaviorMode.enableLCBehavior  = true;  % 主动变道模式

% ============ 场景级设定（示例：LC_15 弯道启停） ============
SetSpeed      = 20;    % 设定速度 (m/s)
PreferredLane = 2;     % 偏好车道序号

fprintf("规划器配置已加载：timeHorizon=%s, SetSpeed=%.1f m/s, PreferredLane=%d\n", ...
        mat2str(PlanningParameters.timeHorizon), SetSpeed, PreferredLane);
