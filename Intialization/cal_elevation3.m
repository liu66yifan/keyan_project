clc; 
%%代码解释
%代码功能：计算无人机和所有卫星在所有时刻的仰角

%目前是固定无人机左边，如果无人机运动，只要把输入坐标改一下就可以

%函数输出保存：results中！！！！

%% ====================== 1. 基础配置（固定参数） ======================
% 无人机LLA（你提供的精准数据，固定不变）
drone_lat = [40.3478083753442];   % 纬度（°）
drone_lon = [116.220700000000];   % 经度（°）
drone_Rad = [6369251.94837469];   % 地心距（米）

% WGS84参数（和STK完全一致，核心不变）
WGS84_a = 6378137.0;
WGS84_f = 1/298.257223563;
WGS84_e2 = 2*WGS84_f - WGS84_f^2;

% 无人机离地高度（核心不变）
drone_alt = drone_Rad - WGS84_a;  

% 配置：卫星总数+最小仰角阈值
total_sat = 1000;                % 750颗卫星！！！！！！！！！！！！！！！！（记得改）
MinElevation = 10;              % 可连接仰角阈值

%% ====================== 2. 加载数据+提取所有公共时刻 ======================
load("DataFromSTK.mat");

% 步骤1：提取无人机的所有时刻（作为基准时刻）
%uav_lla = UAV_FixedLLAData{1,1};
all_times = FixedLLAData{1, 1}.Time;       % 全部时刻数组
total_times = length(all_times);% 总时刻数

% 步骤2：预分配结构化存储（核心：每个时刻对应一个结构体）
% 结构体字段说明：
%   time: 该时刻的datetime对象
%   connect_sat_count: 该时刻可连接卫星数
%   sat_list: 该时刻可连接卫星的子结构体数组（包含sat_id、elevation）
results = struct('time', [], 'connect_sat_count', 0, 'sat_list', []);
results(total_times) = results(1);  % 预分配内存

fprintf('? 开始遍历全部时刻（共%d个时刻）\n', total_times);
fprintf('可连接条件：仰角≥%d°\n', MinElevation);
fprintf('----------------------------------------\n');

%% ====================== 3. 遍历全部时刻+100颗卫星 ======================
for t_idx = 1:total_times
    % 步骤1：获取当前时刻
    current_time = all_times(t_idx);
    results(t_idx).time = current_time;
    
    % 步骤2：初始化当前时刻的可连接卫星列表
    sat_list = struct('sat_id', [], 'elevation', []);
    connect_count = 0;
    
    % 步骤3：遍历100颗卫星计算当前时刻的仰角
    for sat_id = 1:total_sat
        try
            % 提取第sat_id颗卫星的LLA数据
            sat_lla = FixedLLAData{sat_id,1};
            
            % 精准匹配当前时刻的索引
            time_idx = find(sat_lla.Time == current_time, 1);
            if isempty(time_idx)
                continue;
            end
            
            % 提取卫星在当前时刻的LLA
            sat_lat = sat_lla.Lat(time_idx);     
            sat_lon = sat_lla.Lon(time_idx);     
            sat_Rad = sat_lla.Rad(time_idx);     
            sat_alt = sat_Rad - WGS84_a;         
            
            % 核心计算（完全复用验证通过的函数）
            [E, N, U] = LLA2ENU_STK_Exact(sat_lat, sat_lon, sat_alt, drone_lat, drone_lon, drone_alt, WGS84_a, WGS84_e2);
            
            % 计算仰角
            slant_range = sqrt(E^2 + N^2 + U^2);
            elevation = rad2deg(asin(U / slant_range));
            elevation = max(elevation, 0);  
            
            % 筛选可连接卫星并存储
            if elevation >= MinElevation
                connect_count = connect_count + 1;
                sat_list(connect_count).sat_id = sat_id;
                sat_list(connect_count).elevation = elevation;
            end
        catch ME
            continue;
        end
    end
    
    % 步骤4：保存当前时刻的结果到结构体
    results(t_idx).connect_sat_count = connect_count;
    results(t_idx).sat_list = sat_list;
    
    % 打印当前时刻汇总（可选，减少冗余输出）
    fprintf('时刻 %d/%d：%s → 可连接卫星数：%d颗\n', ...
        t_idx, total_times, datestr(current_time), connect_count);
end

%% ====================== 4. 结果保存+使用示例 ======================
% 保存结构体到mat文件（后续可直接load使用）
save('全时刻可连接卫星数据.mat', 'results');
fprintf('----------------------------------------\n');
fprintf('? 所有时刻数据已保存到：全时刻可连接卫星数据.mat\n');

% ---------------------- 后续使用示例（复制即可调用） ----------------------
fprintf('\n? 后续使用示例：\n');
% 示例1：获取第5个时刻的可连接卫星
t_example = 1;
if t_example <= total_times
    fprintf('示例1：时刻%s的可连接卫星：\n', datestr(results(t_example).time));
    for i = 1:length(results(t_example).sat_list)
        sat_id = results(t_example).sat_list(i).sat_id;
        elev = results(t_example).sat_list(i).elevation;
        fprintf('  卫星ID=%d，仰角=%.2f°\n', sat_id, elev);
    end
end

% 示例2：获取所有时刻中卫星91可连接的时刻
fprintf('\n示例2：卫星91可连接的所有时刻：\n');
for t_idx = 1:total_times
    sat_list = results(t_idx).sat_list;
    sat_ids = [sat_list.sat_id];  % 提取该时刻所有可连接卫星ID
    if ismember(91, sat_ids)
        % 找到卫星91在该时刻的仰角
        idx = find(sat_ids == 91);
        elev = sat_list(idx).elevation;
        fprintf('  时刻%s → 仰角=%.2f°\n', datestr(results(t_idx).time), elev);
    end
end

%% ====================== 核心函数：完全保留你验证通过的版本（无任何改动） ======================
function [E, N, U] = LLA2ENU_STK_Exact(sat_lat, sat_lon, sat_alt, ref_lat, ref_lon, ref_alt, a, e2)
    % 转弧度
    ref_lat_rad = deg2rad(ref_lat);
    ref_lon_rad = deg2rad(ref_lon);
    sat_lat_rad = deg2rad(sat_lat);
    sat_lon_rad = deg2rad(sat_lon);
    
    % 计算无人机ECEF坐标
    N_ref = a / sqrt(1 - e2*sin(ref_lat_rad)^2);
    ref_x = (N_ref + ref_alt) * cos(ref_lat_rad) * cos(ref_lon_rad);
    ref_y = (N_ref + ref_alt) * cos(ref_lat_rad) * sin(ref_lon_rad);
    ref_z = (N_ref*(1 - e2) + ref_alt) * sin(ref_lat_rad);
    
    % 计算卫星ECEF坐标
    N_sat = a / sqrt(1 - e2*sin(sat_lat_rad)^2);
    sat_x = (N_sat + sat_alt) * cos(sat_lat_rad) * cos(sat_lon_rad);
    sat_y = (N_sat + sat_alt) * cos(sat_lat_rad) * sin(sat_lon_rad);
    sat_z = (N_sat*(1 - e2) + sat_alt) * sin(sat_lat_rad);
    
    % 相对ECEF向量
    dx = sat_x - ref_x;
    dy = sat_y - ref_y;
    dz = sat_z - ref_z;
    
    % STK官方ENU旋转矩阵
    R = [
        -sin(ref_lon_rad),          cos(ref_lon_rad),         0;
        -sin(ref_lat_rad)*cos(ref_lon_rad), -sin(ref_lat_rad)*sin(ref_lon_rad), cos(ref_lat_rad);
        cos(ref_lat_rad)*cos(ref_lon_rad),  cos(ref_lat_rad)*sin(ref_lon_rad), sin(ref_lat_rad)
    ];
    
    % 转换为ENU
    enu_vec = R * [dx; dy; dz];
    E = enu_vec(1);
    N = enu_vec(2);
    U = enu_vec(3);
end