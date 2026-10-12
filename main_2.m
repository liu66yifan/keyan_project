clear all
clc
close all
%% Constants
dtr = pi/180;
rtd = 180/pi;
Re = 6371173;
miu = 398600.4418e9;

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
orbit.epochend.hour = 13;
orbit.epochend.minute = 30;
orbit.epochend.second = 0;

%% STK Initialized
app = actxserver('STK11.Application');
root = app.Personality2;
scenario = root.Children.New('escenario','Scenario688');
STK_Basefcn_SetDefaultUnit(root);   % 设置量纲
flag = STK_Basefcn_SetEpoch(root,orbit.epoch,orbit.epochend); 

%% Create facility
Struct_Facility.Num = 1;
Struct_Facility.Name = ["New_York"];
Struct_Facility.Lon = [116.2207];       %地面站经度
Struct_Facility.Lat = [40.5378];        %地面站纬度
Struct_Facility.High = [104.80];        %地面站高度
MinElevation = 10;                      %地面站最小仰角
AllFacility = STK_Basefcn_CreateFacility(root, Struct_Facility, MinElevation);

%% Set UAV
% IAgStkObjectRoot root: STK Object Model root
Aircraft_Handle = root.CurrentScenario.Children.New('eAircraft', 'MyAircraft');
% IAgAircraft aircraft: Aircraft object
route = Aircraft_Handle.Route;
ptsArray = {40.5378,116.2207,104.80,100,2;
            40.2602,123.5517,104.80,100,2;
            30.2602,123.5517,104.80,100,2;
            30.2602,116.5517,104.80,100,2;
            35.2602,116.5517,104.80,100,2};
route.SetPointsSmoothRateAndPropagate(ptsArray);

% Propagate the route
route.Propagate;

%% Seed Satellite and constellation parameters
% 550e3=550km + 地球半径 = 轨道半径
% 0.00001是偏心率 几乎 == 圆轨道 
% 53是轨道倾角
% 0 升交点赤经RAAN
% 0 近地点幅角
% 10 真近点角
SeedSat = [550e3 + Re,0.00001,53 ,0,0,10;].';   % 基准卫星轨道根数列向量(.'是转置)
NumConstellation = 1;                             % 星座数量
P = [25];         % Number of Plane
N = [40];         % Number of Satellite   
F = [1.0];       % 相位因子(控制不同轨道面之间的相位错开)
FirstName = "C";
Flag_DS = 1;     % 1:使用walker Delta  0:使用walker Star
Structure_Walker = fun_SetSatStructure(SeedSat, P, N, F, FirstName, Flag_DS);
AllSatHandle = STK_Basefcn_CreateSatellite_J2(root, Structure_Walker);

%% % 建立 无人机 与 每个星 的 access
%DataStep = 15.0; % s
%StructAccess2 = STK_Basefcn_GetAccess(scenario, AllSatHandle, Aircraft_Handle, DataStep);

%% % 建立 每个站 与 每个星的 access
DataStep = 15.0; % 时间步长 s
StructAccess = STK_Basefcn_GetAccess(scenario, AllSatHandle, AllFacility, DataStep);
%% Get data from STK(卫星)
Time_start =  STK_Basefcn_Time2String(orbit.epoch);
Time_end =  STK_Basefcn_Time2String(orbit.epochend);
time_step = 15.0;
for p = 1:Structure_Walker.TNum 
    J2000Data{p,1} = STK_Basefcn_GetJ2000(AllSatHandle(p,1), Time_start, Time_end, time_step);
    FixedLLAData{p,1} = STK_Basefcn_GetFixedLLA(AllSatHandle(p,1),Time_start,Time_end,time_step);
    ClassicalEleData{p,1} = STK_Basefcn_GetClassicalEle(AllSatHandle(p,1),Time_start,Time_end,time_step);
end
save("DataFromSTK.mat",'J2000Data','ClassicalEleData','FixedLLAData');

%% Get data from STK（动态无人机）
% ========== 2. 新增：保存无人机数据（和卫星格式完全一致） ==========
% 调用和卫星相同的函数，获取无人机J2000坐标（时间范围/步长和卫星一致）
%UAV_J2000Data{1,1} = STK_Basefcn_GetJ2000(Aircraft_Handle, Time_start, Time_end, 15);
% 获取无人机FixedLLA坐标（经纬度高，地固系）
%UAV_FixedLLAData{1,1} = STK_Basefcn_GetFixedLLA(Aircraft_Handle, Time_start, Time_end, 15);

% ========== 3. 保存数据（包含卫星+无人机） ==========
%save("DataFromSTK.mat",...
%    'J2000Data','ClassicalEleData','FixedLLAData',...  % 原有卫星数据
%    'UAV_J2000Data','UAV_FixedLLAData');              % 新增无人机数据

