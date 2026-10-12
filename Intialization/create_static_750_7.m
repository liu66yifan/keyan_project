clc;
%生成adj_matrices_static 25*751*751
% 这个代码需要改sat_id_xianzhi
%% ====================== 1. 核心配置（时间区间：12个时刻） ======================
start_time = 1;        % 第一个时刻，t_idx = 1
end_time = 841;         % 最后一个时刻，t_idx = 12（总共12个时刻）
ground_station_node = 0;   % 地面站节点（0开始）
sat_id_xianzhi = 1000; % !!!!!!!!!!!!!!!!!

%% ====================== 2. 加载数据 ======================
try
    load('无人机-卫星速率数据_result3.mat', 'result3');
    load('ISL_AllSat_Static_Topology_Result.mat', 'result_static');
    fprintf('数据加载成功\n');
catch ME
    error('请确认数据文件在当前路径！原始错误：%s', ME.message);
end

%% ====================== 3. 创建三维矩阵来存储所有时刻的邻接矩阵 ======================
% 改数量要记得改矩阵！！！！！！！！！！！！！！！！！！！！！！！！！！！！！
adj_matrices_static = inf(end_time, 1001, 1001);  % 创建一个 12×751×751 的三维矩阵，用来存储每个时刻的邻接矩阵

for t_idx = start_time:end_time
    fprintf('正在处理第 %d 时刻的邻接矩阵...\n', t_idx);

    %% 调用 create_diagram_dynamic 方法生成邻接矩阵
    [adj_matrix, has_valid_links] = create_diagram_dynamic(t_idx, result3, result_static, ground_station_node);

    %% 4. 验证构图结果（数值输出）
    fprintf('\n=============================================\n');
    fprintf('第%d时刻 构图函数输出验证：\n', t_idx);

    if isempty(adj_matrix)
        fprintf('  邻接矩阵为空！\n');
        continue;  % 跳过此时刻，处理下一个时刻
    end

    fprintf('  邻接矩阵维度：%d×%d\n', size(adj_matrix, 1), size(adj_matrix, 2));
    fprintf('  has_valid_links = %d\n', has_valid_links);

    % 无人机接入卫星链路验证
    drone_sat_ids = result3(t_idx).sat_ids(:);
    drone_sat_dists = result3(t_idx).dists_m(:);

    fprintf('  无人机接入卫星数量：%d颗\n', numel(drone_sat_ids));

    if ~isempty(drone_sat_ids)
        fprintf('  地面站→接入卫星链路详情：\n');
        for i = 1:numel(drone_sat_ids)
            sat_id = drone_sat_ids(i);
            if isfinite(sat_id) && sat_id == floor(sat_id) && sat_id >= 1 && sat_id <= sat_id_xianzhi
                link_dist = adj_matrix(ground_station_node + 1, sat_id + 1);
                fprintf('    地面站(0) → 卫星(%d)：邻接矩阵=%.1f米，result_test=%.1f米\n', ...
                    sat_id, link_dist, drone_sat_dists(i));
            else
                fprintf('    跳过非法sat_id：%g\n', sat_id);
            end
        end
    end

    % 随机验证1颗卫星的星间链路（取第1颗接入卫星）
    if ~isempty(drone_sat_ids) && isfinite(drone_sat_ids(1))
        test_sat_id = drone_sat_ids(1);
        if test_sat_id >= 1 && test_sat_id <= sat_id_xianzhi
            sat_info = result_static(t_idx).sat_data(test_sat_id, :);

            fprintf('  验证卫星(%d)的星间链路（邻接矩阵 vs result_static）：\n', test_sat_id);

            link1_sat = sat_info(2); link1_dist = sat_info(3);
            if isfinite(link1_sat) && isfinite(link1_dist)
                adj_link1_dist = adj_matrix(test_sat_id + 1, link1_sat + 1);
                fprintf('    跨轨道链路1：%d→%d：result_static=%.1f，adj=%.1f\n', ...
                    test_sat_id, link1_sat, link1_dist, adj_link1_dist);
            end

            orbit1_sat = sat_info(6); orbit1_dist = sat_info(7);
            if isfinite(orbit1_sat) && isfinite(orbit1_dist)
                adj_orbit1_dist = adj_matrix(test_sat_id + 1, orbit1_sat + 1);
                fprintf('    同轨道链路1：%d→%d：result_static=%.1f，adj=%.1f\n', ...
                    test_sat_id, orbit1_sat, orbit1_dist, adj_orbit1_dist);
            end
        end
    end

    fprintf('=============================================\n');
    
    %% 5. 保存邻接矩阵到三维矩阵
    adj_matrices_static(t_idx, :, :) = adj_matrix;  % 保存当前时刻的邻接矩阵

end

