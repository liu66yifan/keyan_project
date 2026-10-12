clc; 
%函数输出保存：results2中（uav--sat距离）
%% 1. 固定无人机LLA坐标（悬停时不变）+ 预计算固定ECEF（只算一次，提升效率）
drone_lat = [40.3478083753442];   % 纬度（°）
drone_lon = [116.220700000000];   % 经度（°）
drone_Rad = [6369251.94837469];   % 地心距（米）
drone_alt = drone_Rad - 6378137.0;  % 离地高度（米）

% 预计算无人机固定ECEF坐标（只算一次，无需重复转换）
drone_ecef = LLA2ECEF(drone_lat, drone_lon, drone_alt);

%% 2. 加载数据+提取所有公共时刻（以无人机时间序列为基准）
load("DataFromSTK.mat");
total_sat = 1000;  % 750颗卫星！！！！！！！！！！！！！！！！！！！！！(记得改)

% 提取所有时刻（无人机悬停，时间序列完整）
%all_times = UAV_FixedLLAData{1,1}.Time;
total_times = length(all_times);
fprintf('=== 开始遍历所有时刻（共%d个时刻） ===\n', total_times);

%% 3. 预分配结构体保存结果（命名为result2，与仰角的result区分）
% 结构体字段：time(时刻), sat_distances(卫星距离数组，sat_distances(sat_id)=对应距离)
result2 = struct('time', [], 'sat_distances', zeros(1, total_sat));
result2(total_times) = result2(1);  % 预分配内存

%% 4. 遍历所有时刻+800颗卫星计算距离
for t_idx = 1:total_times
    current_time = all_times(t_idx);
    result2(t_idx).time = current_time;  % 保存当前时刻
    
    % 打印进度（可选，避免输出冗余）
    fprintf('正在计算时刻 %d/%d：%s\n', t_idx, total_times, datestr(current_time));
    
    % 遍历750颗卫星
    for sat_id = 1:total_sat
        try
            % 提取第sat_id颗卫星的LLA数据
            sat_lla_data = FixedLLAData{sat_id,1};
            
            % 定位当前时刻在卫星时间序列中的索引
            time_idx = find(sat_lla_data.Time == current_time, 1);
            if isempty(time_idx)
                result2(t_idx).sat_distances(sat_id) = NaN;  % 无数据标为NaN
                continue;
            end
            
            % 提取卫星当前时刻的LLA
            sat_lat = sat_lla_data.Lat(time_idx);
            sat_lon = sat_lla_data.Lon(time_idx);
            sat_Rad = sat_lla_data.Rad(time_idx);
            sat_alt = sat_Rad - 6378137.0;  % 卫星离地高度
            
            % 卫星LLA转ECEF
            sat_ecef = LLA2ECEF(sat_lat, sat_lon, sat_alt);
            
            % 计算3D距离（复用欧几里得公式）
            distance = satellite_distance_core(drone_ecef, sat_ecef);
            result2(t_idx).sat_distances(sat_id) = distance;  % 保存距离
            
        catch ME
            result2(t_idx).sat_distances(sat_id) = NaN;  % 出错标为NaN
            continue;
        end
    end
end

%% 5. 结果保存+简单验证（保存为result2）
save('无人机-卫星全时刻距离数据.mat', 'result2');
fprintf('=== 所有时刻计算完成！结果已保存为：无人机-卫星全时刻距离数据.mat ===\n');

% 验证：打印第一个时刻的前5颗卫星距离
fprintf('\n【验证】2025-12-20 10:00:00 前5颗卫星距离：\n');
first_time_idx = find([result2.time] == datetime('2025-12-20 10:00:00'), 1);
for sat_id = 1:5
    fprintf('卫星ID=%d：距离=%.1fm\n', sat_id, result2(first_time_idx).sat_distances(sat_id));
end

%% ====================== 核心函数（复用/新增） ======================
% 函数1：LLA转ECEF（WGS84，和STK一致）
function ecef = LLA2ECEF(lat, lon, alt)
    a = 6378137.0;        % 赤道半径（米）
    e2 = 0.00669437999014;% 第一偏心率平方
    
    lat_rad = deg2rad(lat);
    lon_rad = deg2rad(lon);
    
    N = a / sqrt(1 - e2 * sin(lat_rad)^2);  % 卯酉圈曲率半径
    x = (N + alt) * cos(lat_rad) * cos(lon_rad);
    y = (N + alt) * cos(lat_rad) * sin(lon_rad);
    z = (N * (1 - e2) + alt) * sin(lat_rad);
    ecef = [x, y, z];
end

% 函数2：欧几里得距离计算（复用你原有的核心逻辑）
function dist = satellite_distance_core(xyz1, xyz2)
    delta_x = xyz2(1) - xyz1(1);
    delta_y = xyz2(2) - xyz1(2);
    delta_z = xyz2(3) - xyz1(3);
    dist = sqrt(delta_x^2 + delta_y^2 + delta_z^2);
    dist = round(dist, 1);
end