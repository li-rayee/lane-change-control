% simulate_and_visualize.m
% -------------------------------------------------------------------------
% 高速路变道控制 —— 逐帧仿真与可视化
% 功能：分时段运行测试台架模型，观察车辆行为（巡航 CC -> 变道 LC -> 协商），
%       最后用 HelperAnalyzeLCPlannerSimulationResults 生成可视化界面。
% 来源：技术报告第 6.1 节
%
% 前置条件：已加载 HLCPlannerAndControllerTestBench 模型及对应场景设置。
% -------------------------------------------------------------------------

% 1) 初始化场景（弯道启停流）
helperSLHLCPlannerAndControllerSetup('scenarioFcnName', 'scenario_LC_15_StopnGo_Curved');
mpcverbosity('off');   % 关闭 MPC 冗余输出

% 2) 5 秒仿真：规划器生成初始导航轨迹（此时应为 CC 巡航）
sim('HLCPlannerAndControllerTestBench', 'StopTime', '5');
hLCPlot = findobj('Type', 'Figure', 'Name', 'LaneChangeStatusPlot');
if ~isempty(hLCPlot)
    close(hLCPlot);
end

% 3) 8 秒仿真：规划器生成绕行慢车的轨迹（此时应为 LC 变道，右->左）
sim('HLCPlannerAndControllerTestBench', 'StopTime', '8');
hLCPlot = findobj('Type', 'Figure', 'Name', 'LaneChangeStatusPlot');
if ~isempty(hLCPlot)
    close(hLCPlot);
end

% 4) 18 秒仿真：完整执行两次变道避让，并保持设定速度
simout = sim('HLCPlannerAndControllerTestBench', 'StopTime', '18');

% 5) 生成可视化界面（追踪视图 / 轨迹信息 / MIO / 行为模式 / 速度）
visualizatonObj = HelperAnalyzeLCPlannerSimulationResults(simout.logsout);
visualizatonObj.visualizeSimulationData;

fprintf("仿真与可视化完成。\n");
