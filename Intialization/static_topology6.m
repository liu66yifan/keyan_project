%% 全卫星 静态拓扑生成逻辑（无窗口、无阈值，直接填充真实距离）
% 静态拓扑核心规则（完全匹配你的需求）：
% 1. 同轨道：环形连接（30颗/轨道）→ 卫星i连i-1（前）和i+1（后），轨道首连尾、尾连首
% 2. 相邻轨道：环形相邻（轨道1?2/25，轨道2?1/3，...，轨道25?24/1）→ 仅连相邻轨道的「同位置卫星」
%    - 例：卫星1（轨道1-位置1）→ 轨道2-位置1（31）、轨道25-位置1（721）
%    - 例：卫星30（轨道1-位置30）→ 轨道2-位置30（60）、轨道25-位置30（750）
% 存储格式：
% - 结构体数组result_static（25×1），每个元素含time（datetime）和sat_data（750×9矩阵）
% - sat_data每一行：卫星编号 | 链路1ID（跨轨道） | 链路1距离 | 链路2ID（跨轨道） | 链路2距离 | 同轨道ID1 | 同轨道ID1距离 | 同轨道ID2 | 同轨道ID2距离
% 核心特点：无窗口逻辑、无距离阈值，直接填充计算出的真实距离，不做任何筛选

clc;
% 运行一下把result_static --- > 361*1
%% 1. 核心配置（仅保留静态拓扑必需参数，移除窗口和阈值）
orbit_sat_num = 40;               % 每轨道40颗卫星（固定）
guidao_num = 25;
total_sat_num = guidao_num * orbit_sat_num; % 总卫星数：1000（25轨道×40）

fprintf('? 全卫星静态拓扑配置（无窗口、无阈值）：\n');
fprintf('  1. 星座规模：25轨道×40颗/轨道 = %d颗卫星\n', total_sat_num);
fprintf('  2. 静态连接规则：\n');
fprintf('     - 同轨道：环形（如卫星1连2&30，卫星30连29&1）\n');
fprintf('     - 跨轨道：相邻轨道同位置（如轨道1-位置1连轨道2-位置1&轨道25-位置1）\n');
fprintf('  3. 存储格式：结构体数组result_static（每个时刻750×9矩阵）\n');
fprintf('  4. 无距离阈值、无窗口逻辑，直接填充真实距离\n\n');

%% 2. 加载数据 + 提取所有时刻（保持原有逻辑，需卫星坐标计算真实距离）
try
    load('DataFromSTK.mat', 'J2000Data');
    fprintf('? 成功加载J2000Data（%d颗卫星）\n', size(J2000Data,1));
catch
    error('? 请确保DataFromSTK.mat文件和代码在同一文件夹下！');
end

% 提取所有时刻（任意卫星时刻一致）
ref_sat_data = J2000Data{1, 1};
sat_times = ref_sat_data.Time;      % datetime
times = length(all_times);
total_times = times;                   % 总时刻数
if length(sat_times) < total_times
    total_times = length(sat_times);
    warning('?? 数据实际时刻数不足25，自动调整为%d个时刻', total_times);
end
fprintf('? 共%d个时刻，开始全卫星静态拓扑生成（直接填充真实距离）...\n\n', total_times);

%% 3. 初始化结构化存储（result_static，保持750×9格式）
result_static = struct('time', [], 'sat_data', []);
result_static = repmat(result_static, total_times, 1); % 预分配时刻数×1结构体

%% 4. 逐时刻 + 逐卫星生成静态拓扑（核心：固定连接ID，填充真实距离）
for t_idx = 1:total_times
    current_time = sat_times(t_idx);
    fprintf('=============================================\n');
    fprintf('正在生成时刻%d/%d | %s\n', t_idx, total_times, datestr(current_time));
    fprintf('=============================================\n');

    % 初始化当前时刻750×9矩阵（第1列固定为卫星编号）
    sat_data = inf(total_sat_num, 9);
    sat_data(:, 1) = 1:total_sat_num; % 第1列：卫星编号1-750

    % 遍历所有750颗卫星，生成静态连接
    for target_sat_id = 1:total_sat_num
        % -------- 步骤1：提取当前卫星坐标（用于计算真实距离） --------
        sat_rec = J2000Data{target_sat_id, 1};
        sat_coord = [sat_rec.X(t_idx), sat_rec.Y(t_idx), sat_rec.Z(t_idx)];

        % -------- 步骤1.1：静态同轨道连接（环形逻辑，固定ID+真实距离） --------
        orbit_id = ceil(target_sat_id / orbit_sat_num); % 当前卫星轨道ID（1-25）
        same_orbit_sats = ((orbit_id-1)*orbit_sat_num + 1) : (orbit_id*orbit_sat_num); % 本轨道范围
        % 同轨道前相邻（首→尾）
        if target_sat_id == min(same_orbit_sats)
            prev_orbit_sat = max(same_orbit_sats);
        else
            prev_orbit_sat = target_sat_id - 1;
        end
        % 同轨道后相邻（尾→首）
        if target_sat_id == max(same_orbit_sats)
            next_orbit_sat = min(same_orbit_sats);
        else
            next_orbit_sat = target_sat_id + 1;
        end
        % 计算同轨道真实距离（不做任何筛选，是多少填多少）
        prev_sat_data = J2000Data{prev_orbit_sat, 1};
        prev_sat_coord = [prev_sat_data.X(t_idx), prev_sat_data.Y(t_idx), prev_sat_data.Z(t_idx)];
        prev_orbit_dist = sqrt(sum((prev_sat_coord - sat_coord).^2));
        next_sat_data = J2000Data{next_orbit_sat, 1};
        next_sat_coord = [next_sat_data.X(t_idx), next_sat_data.Y(t_idx), next_sat_data.Z(t_idx)];
        next_orbit_dist = sqrt(sum((next_sat_coord - sat_coord).^2));
        % 填充同轨道信息（第6-9列，固定位置）
        sat_data(target_sat_id, 6) = prev_orbit_sat;  % 同轨道ID1（前）
        sat_data(target_sat_id, 7) = prev_orbit_dist;% 同轨道ID1真实距离
        sat_data(target_sat_id, 8) = next_orbit_sat;  % 同轨道ID2（后）
        sat_data(target_sat_id, 9) = next_orbit_dist;% 同轨道ID2真实距离

        % -------- 步骤1.2：静态跨轨道连接（固定ID+真实距离） --------
        % 步骤1.2.1：计算当前卫星在轨道内的位置（1-30，用于匹配同位置）
        pos_in_orbit = target_sat_id - (orbit_id - 1) * orbit_sat_num; % 位置1~30

        % 步骤1.2.2：计算相邻轨道ID（环形相邻：轨道1?2/25，轨道25?24/1）
        next_cross_orbit = orbit_id + 1; % 后相邻轨道
        if next_cross_orbit > guidao_num
            next_cross_orbit = 1;
        end
        prev_cross_orbit = orbit_id - 1; % 前相邻轨道
        if prev_cross_orbit < 1
            prev_cross_orbit = guidao_num;
        end

        % 步骤1.2.3：计算相邻轨道的同位置卫星ID（固定不变）
        static_cross1_sat = (next_cross_orbit - 1) * orbit_sat_num + pos_in_orbit; % 链路1：后相邻轨道同位置
        static_cross2_sat = (prev_cross_orbit - 1) * orbit_sat_num + pos_in_orbit; % 链路2：前相邻轨道同位置
        % 保险：避免双链路连接同一卫星（静态规则下不会出现，仅防异常）
        if static_cross1_sat == static_cross2_sat
            static_cross2_sat = inf;
        end

        % -------- 步骤1.3：计算跨轨道真实距离（不做任何筛选，是多少填多少） --------
        % 链路1（后相邻轨道同位置）真实距离
        cross1_data = J2000Data{static_cross1_sat, 1};
        cross1_coord = [cross1_data.X(t_idx), cross1_data.Y(t_idx), cross1_data.Z(t_idx)];
        cross1_dist = sqrt(sum((cross1_coord - sat_coord).^2));
        % 链路2（前相邻轨道同位置）真实距离
        cross2_data = J2000Data{static_cross2_sat, 1};
        cross2_coord = [cross2_data.X(t_idx), cross2_data.Y(t_idx), cross2_data.Z(t_idx)];
        cross2_dist = sqrt(sum((cross2_coord - sat_coord).^2));

        % -------- 步骤2：填充跨轨道信息（第2-5列，固定位置） --------
        sat_data(target_sat_id, 2) = static_cross1_sat; % 链路1ID（静态跨轨道）
        sat_data(target_sat_id, 3) = cross1_dist;       % 链路1真实距离
        sat_data(target_sat_id, 4) = static_cross2_sat; % 链路2ID（静态跨轨道）
        sat_data(target_sat_id, 5) = cross2_dist;       % 链路2真实距离
    end

    % -------- 步骤3：将当前时刻矩阵存入result_static --------
    result_static(t_idx).time = current_time;
    result_static(t_idx).sat_data = sat_data;
    fprintf('? 时刻%d静态拓扑生成完成（750×9矩阵，真实距离直接填充）\n', t_idx);
    fprintf('=============================================\n\n');
end

%% 5. 保存静态拓扑结果（文件名：result_static）
save('ISL_AllSat_Static_Topology_Result.mat', 'result_static', 'total_times');
fprintf('=============================================\n');
fprintf('? 全卫星静态拓扑数据已保存！\n');
fprintf('  - 文件名：ISL_AllSat_Static_Topology_Result.mat\n');
fprintf('  - 结构体数组：result_static（%d×1）\n', total_times);
fprintf('  - 每个时刻数据：750×9矩阵（卫星编号+静态跨轨道+静态同轨道）\n');
fprintf('  - 数据特点：无窗口、无阈值，所有距离为真实计算值\n');
fprintf('  - 总数据量：%d×%d×9=%.0f个值（约19MB）\n', total_times, total_sat_num, total_times*total_sat_num*9);
fprintf('=============================================\n');

%% 6. 快速访问示例（验证静态连接和真实距离）
fprintf('=============================================\n');
fprintf('          静态拓扑验证示例（真实距离）\n');
fprintf('=============================================\n');
% 加载静态拓扑结果
load('ISL_AllSat_Static_Topology_Result.mat', 'result_static');

% 示例1：验证卫星1（轨道1-位置1）的静态连接（应连31、721、30、2）
t_example = 1;
sat_example1 = 1;
sat_info1 = result_static(t_example).sat_data(sat_example1, :);
fprintf('? 第%d时刻卫星%d（轨道1-位置1）静态连接：\n', t_example, sat_example1);
fprintf('  卫星编号：%d\n', sat_info1(1));
fprintf('  跨轨道链路1：卫星%d（真实距离=%.2f米）→ 轨道2-位置1\n', sat_info1(2), sat_info1(3));
fprintf('  跨轨道链路2：卫星%d（真实距离=%.2f米）→ 轨道25-位置1\n', sat_info1(4), sat_info1(5));
fprintf('  同轨道链路1：卫星%d（真实距离=%.2f米）→ 轨道1-位置30\n', sat_info1(6), sat_info1(7));
fprintf('  同轨道链路2：卫星%d（真实距离=%.2f米）→ 轨道1-位置2\n', sat_info1(8), sat_info1(9));

% 示例2：验证卫星31（轨道2-位置1）的静态连接（应连32、1、60、30）
sat_example2 = 41;
sat_info2 = result_static(t_example).sat_data(sat_example2, :);
fprintf('\n? 第%d时刻卫星%d（轨道2-位置1）静态连接：\n', t_example, sat_example2);
fprintf('  跨轨道链路1：卫星%d（真实距离=%.2f米）→ 轨道3-位置1\n', sat_info2(2), sat_info2(3));
fprintf('  跨轨道链路2：卫星%d（真实距离=%.2f米）→ 轨道1-位置1\n', sat_info2(4), sat_info2(5));
fprintf('  同轨道链路1：卫星%d（真实距离=%.2f米）→ 轨道2-位置30\n', sat_info2(6), sat_info2(7));
fprintf('  同轨道链路2：卫星%d（真实距离=%.2f米）→ 轨道2-位置2\n', sat_info2(8), sat_info2(9));
fprintf('=============================================\n');