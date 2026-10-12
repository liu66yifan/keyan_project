% ====================== 原始卫星数据 J2000Data ======================
% J2000Data 是一个 cell 数组（长度=卫星总数，比如25），每个 cell 对应一颗卫星的完整结构体，
% 包含字段：Time（卫星完整时间序列）、X（X坐标序列）、Y（Y坐标序列）、Z（Z坐标序列）

% 函数生成：all_sat_interval_data (生成我们需要时间段内所有卫星的坐标xyz)


% ====================== 步骤1：配置参数（可自定义，也可复用ground_sat_core_info的时间） ======================
% 方式1：手动指定时间区间（如果你想单独设置）
% t_start = datetime('2025-12-20 10:00:00');  % 起始时间
% t_end = datetime('2025-12-20 10:21:34');    % 结束时间

% 方式2：（推荐）从ground_sat_core_info中自动获取所有卫星的时间范围（无需手动输入，贴合你的场景）
t_start = min([ground_sat_core_info.start_time]);  % 所有卫星的最早开始时间
t_end = max([ground_sat_core_info.stop_time]);    % 所有卫星的最晚结束时间
total_sat = length(J2000Data);  % 自动获取卫星总数（比如25颗）

fprintf('? 准备提取卫星坐标，时间区间：%s ~ %s\n', datestr(t_start), datestr(t_end));
fprintf('? 待处理卫星总数：%d颗\n', total_sat);

% ====================== 步骤2：初始化存储结构（生成all_sat_interval_data的容器） ======================
all_sat_interval_data = cell(total_sat, 1);  % 初始化N×1 cell数组（N=卫星总数）

% ====================== 步骤3：循环遍历每颗卫星，筛选并存储时间+坐标 ======================
for sat_id = 1:total_sat
    % 1. 取出当前卫星的原始完整数据
    current_sat_raw = J2000Data{sat_id};  % 从原始J2000Data中取出第sat_id颗卫星
    
    % 2. 筛选：当前卫星在指定时间区间内的时刻（增加0.5秒误差兼容，避免边界时刻丢失）
    time_mask = (current_sat_raw.Time >= t_start - seconds(0.5)) & (current_sat_raw.Time <= t_end + seconds(0.5));
    
    % 3. 提取筛选后的时间和XYZ坐标
    sat_time_interval = current_sat_raw.Time(time_mask);  % 该卫星在时间区间内的所有时刻
    sat_x_interval = current_sat_raw.X(time_mask);        % 对应时刻的X坐标
    sat_y_interval = current_sat_raw.Y(time_mask);        % 对应时刻的Y坐标
    sat_z_interval = current_sat_raw.Z(time_mask);        % 对应时刻的Z坐标
    
    % 4. 封装成结构体，存入all_sat_interval_data（核心步骤）
    all_sat_interval_data{sat_id} = struct( ...
        'SatelliteID', sat_id, ...          % 卫星编号（唯一标识）
        'Time_Interval', sat_time_interval, ...  % 筛选后的时间序列
        'X_Interval', sat_x_interval, ...        % 筛选后的X坐标序列
        'Y_Interval', sat_y_interval, ...        % 筛选后的Y坐标序列
        'Z_Interval', sat_z_interval ...          % 筛选后的Z坐标序列
    );
    
    % 打印进度，方便你查看每颗卫星的提取结果
    sample_count = length(sat_time_interval);
    fprintf('第%d颗卫星提取完成，共获取%d个时间-坐标采样点\n', sat_id, sample_count);
end

fprintf('\n?all_sat_interval_data 生成完成！\n');
fprintf('该变量是 %d×1 cell数组，每个cell对应一颗卫星的时间+XYZ坐标\n', total_sat);