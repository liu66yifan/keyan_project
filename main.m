clear all
clc
close all

%% Constants
dtr = pi/180;
rtd = 180/pi;
Re = 6371173;         %地球半径
miu = 398600.4418e9;  %地球标准引力参数

%% Set Date
%场景历园 epoch起始时间、epochend结束时间
%orbit是自定义结构体
orbit.epoch.year = 2025;        
orbit.epoch.month = 12;
orbit.epoch.day = 20;
orbit.epoch.hour = 10;
orbit.epoch.minute = 00;
orbit.epoch.second = 0;

orbit.epochend.year = 2025;        
orbit.epochend.month = 12;
orbit.epochend.day = 20;
orbit.epochend.hour = 11;
orbit.epochend.minute = 30;
orbit.epochend.second = 0;

%% STK Initialized
app = actxserver('STK11.Application');                        %启动STK，创建场景
root = app.Personality2;                                      %获取STK对象，用来控制STK
scenario = root.Children.New('escenario','Scenario688');      %STK创建一个新对象场景,场景名称叫Scenario688
STK_Basefcn_SetDefaultUnit(root);                             % 设置量纲
flag = STK_Basefcn_SetEpoch(root,orbit.epoch,orbit.epochend); % STK_Basefcn_SetEpoch是一个初始设置函数

%% Create facility
Struct_Facility.Num = 1;
Struct_Facility.Name = ["New_York"];
Struct_Facility.Lon = [-74.0060];       %地面站经度
Struct_Facility.Lat = [40.7128];        %地面站纬度
Struct_Facility.High = [0];             %地面站高度
MinElevation = 10;                      %地面站最小仰角
AllFacility = STK_Basefcn_CreateFacility(root, Struct_Facility, MinElevation);

%% Seed Satellite and constellation parameters
% 550e3=550km + 地球半径 = 轨道半径
% 0.00001是偏心率 几乎 == 圆轨道 
% 53是轨道倾角
% 0 升交点赤经RAAN
% 0 近地点幅角
% 10 真近点角
SeedSat = [550e3 + Re,0.00001,53 ,0,0,10;].';   % 基准卫星轨道根数列向量(.'是转置)
NumConstellation = 1;                             % 星座数量
P = [3];         % Number of Plane
N = [4];         % Number of Satellite   
F = [1.0];       % 相位因子(控制不同轨道面之间的相位错开)
FirstName = "C";
Flag_DS = 1;     % 1:使用walker Delta  0:使用walker Star
Structure_Walker = fun_SetSatStructure(SeedSat, P, N, F, FirstName, Flag_DS);
AllSatHandle = STK_Basefcn_CreateSatellite_J2(root, Structure_Walker);

%% % 建立 每个站 与 每个星的 access
DataStep = 15.0; % 时间步长 s
StructAccess = STK_Basefcn_GetAccess(scenario, AllSatHandle, AllFacility, DataStep);

%% Get data from STK
% 这段代码是将stk获取的给定时间范围的不同数据，并将这些数据保存到.mat文件中
Time_start =  STK_Basefcn_Time2String(orbit.epoch);
Time_end =  STK_Basefcn_Time2String(orbit.epochend);
time_step = 15.0;  % 步长s
for p = 1:Structure_Walker.TNum 
    J2000Data{p,1} = STK_Basefcn_GetJ2000(AllSatHandle(p,1), Time_start, Time_end, time_step);            % 得到 J2000坐标下的X Y Z
    FixedLLAData{p,1} = STK_Basefcn_GetFixedLLA(AllSatHandle(p,1),Time_start,Time_end,time_step);         % 得到 经度 纬度 高度
    ClassicalEleData{p,1} = STK_Basefcn_GetClassicalEle(AllSatHandle(p,1),Time_start,Time_end,time_step); % 得到 卫星经典半长轴等数据
end
save("DataFromSTK.mat",'J2000Data','ClassicalEleData','FixedLLAData');

%% Save as Excel

% IAgStkObjectRoot root: STK Object Model root
aircraft = root.CurrentScenario.Children.New('eAircraft', 'MyAircraft');
% IAgAircraft aircraft: Aircraft object
route = aircraft.Route;
ptsArray = {40.5378,116.2207,104.80,100,2;
            40.2602,123.5517,104.80,100,2;
            30.2602,123.5517,104.80,100,2;
            30.2602,116.5517,104.80,100,2;
            35.2602,116.5517,104.80,100,2};
route.SetPointsSmoothRateAndPropagate(ptsArray);

% Propagate the route
route.Propagate;



