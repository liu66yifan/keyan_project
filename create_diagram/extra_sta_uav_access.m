% ====================== 提取：3个核心星地数据 ======================
% 元素1：sat_id（地面站连接的卫星编号）
% 元素2：start_time/stop_time（开始时间+结束时间，合并为结构体字段）
% 元素3：range_data（星地距离）

% 函数结果：存储在 ground_sat_core_info

ground_sat_core_info = struct( ...
    'sat_id', [], ...          % 核心1：卫星编号
    'start_time', [], ...      % 核心2：开始时间
    'stop_time', [], ...       % 核心2：结束时间
    'range_data', [] ...       % 核心3：星地距离
);
% 初始化结构体数组（长度等于StructAccess条目数）
ground_sat_core_info = repmat(ground_sat_core_info, length(StructAccess), 1);
valid_count = 0;

% 遍历StructAccess，精准提取3个核心元素
for i = 1:length(StructAccess)
    current_struct = StructAccess(i);  % 简化当前条目引用
    current_sat_id = [];
    current_start = [];
    current_stop = [];
    current_range = [];

    % ------------- 1. 提取核心：卫星编号 -------------
    if isfield(current_struct, 'Sat') && ~isempty(current_struct.Sat)
        current_sat_id = current_struct.Sat;
    else
        continue;  % 无卫星编号，跳过该条目
    end

    % ------------- 2. 提取核心：开始时间 + 结束时间（修正字段引用bug，二选一适配你的字段） -------------
    % 情况A：如果你的字段是「驼峰式」（StartTimeDate/StopTimeDate，无空格）【推荐，更稳定】
    if isfield(current_struct, 'StartTimeDate') && ~isempty(current_struct.StartTimeDate)
        current_start = current_struct.StartTimeDate;
    end
    if isfield(current_struct, 'StopTimeDate') && ~isempty(current_struct.StopTimeDate)
        current_stop = current_struct.StopTimeDate;
    end

    % 情况B：如果你的字段是「带空格」（Start Time Date/Stop Time Date），取消下面注释启用
    % if isfield(current_struct, 'Start Time Date') && ~isempty(current_struct.('Start Time Date'))
    %     current_start = current_struct.('Start Time Date');
    % end
    % if isfield(current_struct, 'Stop Time Date') && ~isempty(current_struct.('Stop Time Date'))
    %     current_stop = current_struct.('Stop Time Date');
    % end

    % 无有效起止时间，跳过
    if isempty(current_start) || isempty(current_stop)
        continue;
    end
    % 统一转换为datetime格式（确保格式一致）
    if ~isa(current_start, 'datetime')
        current_start = datetime(current_start);
    end
    if ~isa(current_stop, 'datetime')
        current_stop = datetime(current_stop);
    end

    % ------------- 3. 提取核心：星地距离（accessAER_Range） -------------
    if isfield(current_struct, 'accessAER_Range') && ~isempty(current_struct.accessAER_Range)
        current_range_cell = current_struct.accessAER_Range;
        % cell数组转数值数组（兼容直接是数值数组的情况）
        if iscell(current_range_cell)
            current_range = cell2mat(current_range_cell);
        else
            current_range = current_range_cell;
        end
    else
        continue;  % 无距离数据，跳过该条目
    end

    % ------------- 存储有效核心数据 -------------
    valid_count = valid_count + 1;
    ground_sat_core_info(valid_count).sat_id = current_sat_id;
    ground_sat_core_info(valid_count).start_time = current_start;
    ground_sat_core_info(valid_count).stop_time = current_stop;
    ground_sat_core_info(valid_count).range_data = current_range;
end

% 剔除无效条目，只保留有效星地数据
ground_sat_core_info(valid_count+1:end) = [];

% 有效性校验
if valid_count == 0
    error('StructAccess中无有效星地核心数据（卫星编号+起止时间+距离）！');
end

% 打印提取结果，方便核对
fprintf('? 成功提取%d颗卫星的核心星地数据！\n', valid_count);
fprintf('提取的核心元素：\n');
fprintf('1. 卫星编号：');
for i = 1:valid_count
    fprintf('%d ', ground_sat_core_info(i).sat_id);
end
fprintf('\n2. 每颗卫星对应起止时间和星地距离（存储在ground_sat_core_info中）\n');