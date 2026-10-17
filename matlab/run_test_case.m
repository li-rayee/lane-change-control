% run_test_case.m
% -------------------------------------------------------------------------
% 高速路变道控制 —— 测试台架
% 功能：调用 MATLAB Test Manager 中的指定场景用例并运行，随后生成 PDF 报告。
% 来源：课程报告第 5 章（Scenario_LC_15 / LC_06 / LC_11 等）
%
% 前置条件：
%   1. 已加载 MathWorks 示例模型 HLCPlannerAndControllerTestBench
%      （Automated Driving Toolbox -> Highway Lane Change Planner and Controller）
%   2. 已通过 helperSLHighwayLaneChangeSetup 初始化 testFile / testSuite
%
% 使用：将下方 scenarioName 改为目标场景名后直接运行本脚本。
% -------------------------------------------------------------------------

% 选择要运行的场景（报告 5.1 / 5.2 / 5.3 分别对应下列名称）
scenarioName = "scenario_LC_15_StopnGo_Curved";
% scenarioName = "scenario_LC_06_DoubleLaneChange";
% scenarioName = "scenario_LC_11_MergingCar_HighwayEntry";

% 从测试文件中取得测试套件与用例
testSuite = getTestSuiteByName(testFile, "TestScenarios");
testCase  = getTestCaseByName(testSuite, scenarioName);

% 运行该用例，返回结果对象
resultObj = run(testCase);

% 生成仿真报告（含 MATLAB 图形、错误信息，不含逐条测试结果）
sltest.testmanager.report(resultObj, "Report.pdf", ...
    Title = "HighwayLaneChange", ...
    IncludeMATLABFigures = true, ...
    IncludeErrorMessages = true, ...
    IncludeTestResults = false, ...
    LaunchReport = true);

fprintf("场景 %s 运行完成，报告已生成。\n", scenarioName);
