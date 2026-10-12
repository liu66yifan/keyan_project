% 生成星地+星座的图(包含星间ISL、星地距离access)
% 这个代码就是那篇文章的第一个版本

% 函数生成：final_valid_graphs图


% 获取卫星坐标的参考时间序列（和你单个时间片生成时用的逻辑一致）
reference_sat_times = [];
reference_sat_X = [];
reference_sat_Y = [];
reference_sat_Z = [];
total_sat = length(all_sat_interval_data);

% 取任意一颗有效卫星的完整数据（比如第1颗，你也可以指定其他卫星）
for sat_id = 1:total_sat
    current_sat_data = all_sat_interval_data{sat_id};
    if ~isempty(current_sat_data.Time_Interval) && length(current_sat_data.Time_Interval) > 0
        reference_sat_times = current_sat_data.Time_Interval;  % 卫星坐标的所有时刻
        % 顺便获取这颗卫星的XYZ坐标（你单个时间片时也会提取）
        reference_sat_X = current_sat_data.X_Interval;
        reference_sat_Y = current_sat_data.Y_Interval;
        reference_sat_Z = current_sat_data.Z_Interval;
        fprintf('卫星%d的总时间序列，共%d个时刻\n', sat_id, length(reference_sat_times));
        break;
    end
end
% 初始化存储：存放所有有效时间段内生成的图（最终结果）
final_valid_graphs = [];  % 存储所有有效图
final_valid_graph_times = [];  % 存储每个图对应的时刻（方便你对应）

% 遍历每颗卫星（利用ground_sat_core_info的起止时间）
for info_idx = 1:length(ground_sat_core_info)
    % 提取当前卫星的专属时间段（你说的“每一段时间的开始时间和结束时间”）
    current_sat_id = ground_sat_core_info(info_idx).sat_id;
    current_start = ground_sat_core_info(info_idx).start_time;  % 开始时间
    current_stop = ground_sat_core_info(info_idx).stop_time;    % 结束时间
    current_range = ground_sat_core_info(info_idx).range_data;  % 星地距离（和你单个时间片时用的一致）

    fprintf('\n===== 处理卫星%d：时间段 %s ~ %s =====\n', current_sat_id, datestr(current_start), datestr(current_stop));

    % 关键：筛选出卫星坐标中，落在当前时间段内的所有时刻（这是多个时间片和单个的唯一区别）
    time_mask = (reference_sat_times >= current_start) & (reference_sat_times <= current_stop);
    valid_times_in_period = reference_sat_times(time_mask);  % 该时间段内的所有卫星坐标时刻
    valid_time_indices = find(time_mask);  % 这些时刻在卫星坐标总序列中的索引

    if isempty(valid_times_in_period)
        fprintf('该时间段内无卫星坐标时刻，跳过！\n');
        continue;
    end
    fprintf('筛选出该时间段内的卫星坐标时刻共%d个\n', length(valid_times_in_period));

    % 循环遍历这些时刻，逐个生成图（和你单个时间片生成逻辑完全一致，只是多了个for循环）
    for t_idx = 1:length(valid_time_indices)
        % 1. 提取当前时刻的索引和具体时间（单个时间片时你也会指定索引/时间）
        current_time_idx = valid_time_indices(t_idx);
        current_time = valid_times_in_period(t_idx);

        % 2. 提取当前时刻所有卫星的坐标（和你单个时间片时的提取逻辑一致）
        sat_coords_current = zeros(total_sat, 3);
        for sat_id = 1:total_sat
            sat_data = all_sat_interval_data{sat_id};
            if length(sat_data.X_Interval) >= current_time_idx
                sat_coords_current(sat_id, :) = [sat_data.X_Interval(current_time_idx), ...
                                                 sat_data.Y_Interval(current_time_idx), ...
                                                 sat_data.Z_Interval(current_time_idx)];
            else
                sat_coords_current(sat_id, :) = [NaN, NaN, NaN];
            end
        end

        % 3. 初始化邻接矩阵（26×26，和你单个时间片时的初始化一致）
        total_nodes = 1 + total_sat;
        adj_matrix = ones(total_nodes, total_nodes) * Inf;
        adj_matrix(1, 1) = 0;  % 地面站自身
        for i = 2:total_nodes
            adj_matrix(i, i) = 0;  % 卫星自身
        end

        % 4. 填充星间连接（和你单个时间片时的填充逻辑一致）
        sat_per_orbit = 5;
        total_orbit = ceil(total_sat / sat_per_orbit);
        for sat_id = 1:total_sat
            if any(isnan(sat_coords_current(sat_id, :)))
                continue;
            end
            % 计算轨道和邻居（和你单个时间片时一致）
            n = ceil(sat_id / sat_per_orbit);
            m = mod(sat_id - 1, sat_per_orbit) + 1;
            m_prev = m - 1; if m_prev < 1, m_prev = sat_per_orbit; end
            sat_prev = (n - 1) * sat_per_orbit + m_prev;
            m_next = m + 1; if m_next > sat_per_orbit, m_next = 1; end
            sat_next = (n - 1) * sat_per_orbit + m_next;
            prev_orbit = n - 1; if prev_orbit < 1, prev_orbit = total_orbit; end
            sat_po = (prev_orbit - 1) * sat_per_orbit + m;
            next_orbit = n + 1; if next_orbit > total_orbit, next_orbit = 1; end
            sat_no = (next_orbit - 1) * sat_per_orbit + m;
            % 过滤邻居
            all_neighbors = unique([sat_prev, sat_next, sat_po, sat_no]);
            all_neighbors = all_neighbors(all_neighbors >= 1 & all_neighbors <= total_sat);
            % 计算星间距离（和你单个时间片时一致）
            x1 = sat_coords_current(sat_id, 1);
            y1 = sat_coords_current(sat_id, 2);
            z1 = sat_coords_current(sat_id, 3);
            for neighbor_id = all_neighbors
                if any(isnan(sat_coords_current(neighbor_id, :)))
                    continue;
                end
                x2 = sat_coords_current(neighbor_id, 1);
                y2 = sat_coords_current(neighbor_id, 2);
                z2 = sat_coords_current(neighbor_id, 3);
                dist = sqrt((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2);
                adj_matrix(sat_id + 1, neighbor_id + 1) = dist;
                adj_matrix(neighbor_id + 1, sat_id + 1) = dist;
            end
        end

        % 5. 填充星地连接
        % 匹配当前时刻对应的星地距离（按索引匹配，确保一一对应）
        if t_idx <= length(current_range)
            current_ground_dist = current_range(t_idx);
            adj_matrix(1, current_sat_id + 1) = current_ground_dist;
            adj_matrix(current_sat_id + 1, 1) = current_ground_dist;
        end

        % 6. 存储当前时刻的图（收集多个时间片的结果）
        if isempty(final_valid_graphs)
            final_valid_graphs = {adj_matrix};  % 初始化cell数组
        else
            final_valid_graphs{end+1} = adj_matrix;  % 追加新的图
        end
            final_valid_graph_times = [final_valid_graph_times; current_time];  % 记录对应时刻

            fprintf('生成卫星%d在时刻%s的图（第%d个图）\n', current_sat_id, datestr(current_time), length(final_valid_graphs));
    end
end

fprintf('\n所有有效时间段内的图生成完成！共生成%d个图（对应你的3000多有效时刻）\n', length(final_valid_graphs));