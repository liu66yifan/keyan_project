%% ===== 0. 安全初始化与数据保护 =====
clc;

%% ===== 1. 参数设置与 Sweep 设置 =====
% 只比较两个 alpha
alpha_compare = [0, 0.7];
num_compare = numel(alpha_compare);

% 固定扰动强度
current_sigma_s0 = 2.2;

% 横轴：冗余度 n
n_redundancy_list = [1, 2, 3, 4];
num_n = numel(n_redundancy_list);

num_iterations = 1;      % 蒙特卡洛迭代次数
t_idx = 1;
target_sat = 402;           % 目标卫星（0开始编号）
L = 4;                      % 协同接入卫星数
M_bits = 1e6;               % 传输数据量
min_dist_limit = 600e3;     % 硬截断阈值

dist_matrix = adj_matrices_static_temp;   % 纯距离图 / 时延图
c_light = 299792458;        % 光速
rng(1);                     % 固定随机种子

% 存储 PDR 结果
pdr_a0_rmpr       = zeros(num_n, 1);   % a=0, RMPR不相交
pdr_a07_rmpr      = zeros(num_n, 1);   % a=0.7, RMPR不相交
pdr_single_path   = zeros(num_n, 1);   % 单一路径
pdr_overlap_path  = zeros(num_n, 1);   % 可重叠多路径（这里按 a=0）

% 存储时延结果（单位：秒）
lat_a0_rmpr       = zeros(num_n, 1);
lat_a07_rmpr      = zeros(num_n, 1);
lat_single_path   = zeros(num_n, 1);
lat_overlap_path  = zeros(num_n, 1);

%% ===== 2. 预生成 a=0 和 a=0.7 两个代价矩阵 =====
W_mats = cell(num_compare, 1);
T_maxs = zeros(num_compare, 1);
R_refs = zeros(num_compare, 1);

for a_idx = 1:num_compare
    [W_mats{a_idx}, T_maxs(a_idx), R_refs(a_idx)] = ...
        build_weighted_cost_matrix(dist_matrix, current_sigma_s0, alpha_compare(a_idx));
end

%% ===== 3. 先单独计算 single-path（只算一次，与 n 理论上无关） =====
fprintf('\n>>> 正在测试 single-path 基线, sigma_s0 = %.2f ', current_sigma_s0);

success_count_single = 0;
latency_sum_single = 0;
latency_count_single = 0;

for i = 1:num_iterations
    % 获取当前时刻接入点
    drone_sat_ids = result3(t_idx).sat_ids(:);
    drone_sat_ids = drone_sat_ids(isfinite(drone_sat_ids));
    Sources = drone_sat_ids(1:min(end, L));
    if isempty(Sources), continue; end

    % 当前迭代真实执行图
    adj_exec = simulate_link_failure_oisl(dist_matrix, current_sigma_s0, min_dist_limit, false);

    try
        [success_sp, selected_source_sp, selected_path_sp, selected_distance_sp, ...
            all_candidate_paths_sp, all_candidate_distances_sp] = ...
            run_single_path(dist_matrix, adj_exec, Sources, target_sat); %#ok<ASGLU>

        % ---- 时延 ----
        if ~isempty(selected_path_sp)
            d_path = 0;
            for e = 1:numel(selected_path_sp)-1
                d_path = d_path + dist_matrix(selected_path_sp(e)+1, selected_path_sp(e+1)+1);
            end

            T_prop_phys = d_path / c_light;

            idx_r = find(result3(t_idx).sat_ids == selected_source_sp, 1);
            T_trans_phys = M_bits / (result3(t_idx).rates_Mbps(idx_r) * 1e6);

            current_latency_single = T_prop_phys + T_trans_phys;
            latency_sum_single = latency_sum_single + current_latency_single;
            latency_count_single = latency_count_single + 1;
        end

        % ---- PDR ----
        if success_sp == 1
            success_count_single = success_count_single + 1;
        end
    catch
        % 跳过
    end

    if mod(i, 100) == 0, fprintf('.'); end
end

single_path_const_pdr = success_count_single / num_iterations;
single_path_const_lat = latency_sum_single / max(latency_count_single, 1);

pdr_single_path(:) = single_path_const_pdr;
lat_single_path(:) = single_path_const_lat;

fprintf(' 完成');
fprintf(' | single-path PDR = %.4f | single-path Lat(ms) = %.4f\n', ...
    single_path_const_pdr, single_path_const_lat * 1000);

%% ===== 4. 仿真主循环：遍历冗余度 n =====
for n_idx = 1:num_n
    n_redundancy = n_redundancy_list(n_idx);
    fprintf('\n>>> 正在测试冗余度 n = %d, sigma_s0 = %.2f ', n_redundancy, current_sigma_s0);

    % 成功次数统计
    success_count_a0_rmpr      = 0;
    success_count_a07_rmpr     = 0;
    success_count_overlap      = 0;

    % 时延累计
    latency_sum_a0_rmpr      = 0;
    latency_sum_a07_rmpr     = 0;
    latency_sum_overlap      = 0;

    % 实际统计到时延的次数
    latency_count_a0_rmpr    = 0;
    latency_count_a07_rmpr   = 0;
    latency_count_overlap    = 0;

    for i = 1:num_iterations
        % 获取当前时刻接入点
        drone_sat_ids = result3(t_idx).sat_ids(:);
        drone_sat_ids = drone_sat_ids(isfinite(drone_sat_ids));
        Sources = drone_sat_ids(1:min(end, L));
        if isempty(Sources), continue; end

        % 当前迭代真实执行图
        adj_exec = simulate_link_failure_oisl(dist_matrix, current_sigma_s0, min_dist_limit, false);

        %% ===== 方法1：a = 0, RMPR（不相交）=====
        try
            curr_alpha = 0;
            curr_W = W_mats{1};
            curr_Tmax = T_maxs(1);
            curr_Rref = R_refs(1); %#ok<NASGU>

            in = rmpr_build_waterfill_inputs_multi_source( ...
                t_idx, dist_matrix, curr_W, result3, Sources, ...
                target_sat, L, M_bits, curr_alpha, curr_Tmax, false);

            [w_vec, ~] = solve_routing_optimization(in.q_sec, in.gamma_sec, n_redundancy);

            % ---- 时延 ----
            path_physical_delays = zeros(1, in.valid_L);
            for k = 1:in.valid_L
                pk = in.paths{k};

                d_path = 0;
                for e = 1:numel(pk)-1
                    d_path = d_path + dist_matrix(pk(e)+1, pk(e+1)+1);
                end

                T_prop_phys = d_path / c_light;

                idx_r = find(result3(t_idx).sat_ids == in.access_sats(k), 1);
                T_trans_phys = M_bits / (result3(t_idx).rates_Mbps(idx_r) * 1e6);

                path_physical_delays(k) = T_prop_phys + w_vec(k) * T_trans_phys;
            end

            valid_idx = (w_vec > 1e-4);
            if any(valid_idx)
                current_latency = max(path_physical_delays(valid_idx));
                latency_sum_a0_rmpr = latency_sum_a0_rmpr + current_latency;
                latency_count_a0_rmpr = latency_count_a0_rmpr + 1;
            end

            % ---- PDR ----
            received_sum = 0;
            for k = 1:numel(w_vec)
                if is_path_alive(in.paths{k}, adj_exec)
                    received_sum = received_sum + w_vec(k);
                end
            end

            if received_sum >= 1.0 - 1e-12
                success_count_a0_rmpr = success_count_a0_rmpr + 1;
            end
        catch
            % 跳过
        end

        %% ===== 方法2：a = 0.7, RMPR（不相交，改进算法）=====
        try
            curr_alpha = 0.7;
            curr_W = W_mats{2};
            curr_Tmax = T_maxs(2);
            curr_Rref = R_refs(2); %#ok<NASGU>

            in = rmpr_build_waterfill_inputs_multi_source( ...
                t_idx, dist_matrix, curr_W, result3, Sources, ...
                target_sat, L, M_bits, curr_alpha, curr_Tmax, false);

            [w_vec, ~] = solve_routing_optimization(in.q_sec, in.gamma_sec, n_redundancy);

            % ---- 时延 ----
            path_physical_delays = zeros(1, in.valid_L);
            for k = 1:in.valid_L
                pk = in.paths{k};

                d_path = 0;
                for e = 1:numel(pk)-1
                    d_path = d_path + dist_matrix(pk(e)+1, pk(e+1)+1);
                end

                T_prop_phys = d_path / c_light;

                idx_r = find(result3(t_idx).sat_ids == in.access_sats(k), 1);
                T_trans_phys = M_bits / (result3(t_idx).rates_Mbps(idx_r) * 1e6);

                path_physical_delays(k) = T_prop_phys + w_vec(k) * T_trans_phys;
            end

            valid_idx = (w_vec > 1e-4);
            if any(valid_idx)
                current_latency = max(path_physical_delays(valid_idx));
                latency_sum_a07_rmpr = latency_sum_a07_rmpr + current_latency;
                latency_count_a07_rmpr = latency_count_a07_rmpr + 1;
            end

            % ---- PDR ----
            received_sum = 0;
            for k = 1:numel(w_vec)
                if is_path_alive(in.paths{k}, adj_exec)
                    received_sum = received_sum + w_vec(k);
                end
            end

            if received_sum >= 1.0 - 1e-12
                success_count_a07_rmpr = success_count_a07_rmpr + 1;
            end
        catch
            % 跳过
        end

        %% ===== 方法3：Overlap multi-path（按你当前版本，a=0）=====
        try
            curr_alpha = 0;
            curr_W = W_mats{1};
            curr_Tmax = T_maxs(1);
            curr_Rref = R_refs(1); %#ok<NASGU>

            in_overlap = rmpr_build_waterfill_inputs_multi_source_overlap( ...
                t_idx, dist_matrix, curr_W, result3, Sources, ...
                target_sat, L, M_bits, curr_alpha, curr_Tmax, false);

            [w_vec_overlap, ~] = solve_routing_optimization( ...
                in_overlap.q_sec, in_overlap.gamma_sec, n_redundancy);

            % ---- 时延 ----
            path_physical_delays = zeros(1, in_overlap.valid_L);
            for k = 1:in_overlap.valid_L
                pk = in_overlap.paths{k};

                d_path = 0;
                for e = 1:numel(pk)-1
                    d_path = d_path + dist_matrix(pk(e)+1, pk(e+1)+1);
                end

                T_prop_phys = d_path / c_light;

                idx_r = find(result3(t_idx).sat_ids == in_overlap.access_sats(k), 1);
                T_trans_phys = M_bits / (result3(t_idx).rates_Mbps(idx_r) * 1e6);

                path_physical_delays(k) = T_prop_phys + w_vec_overlap(k) * T_trans_phys;
            end

            valid_idx = (w_vec_overlap > 1e-4);
            if any(valid_idx)
                current_latency = max(path_physical_delays(valid_idx));
                latency_sum_overlap = latency_sum_overlap + current_latency;
                latency_count_overlap = latency_count_overlap + 1;
            end

            % ---- PDR ----
            received_sum_overlap = 0;
            for k = 1:numel(w_vec_overlap)
                if is_path_alive(in_overlap.paths{k}, adj_exec)
                    received_sum_overlap = received_sum_overlap + w_vec_overlap(k);
                end
            end

            if received_sum_overlap >= 1.0 - 1e-12
                success_count_overlap = success_count_overlap + 1;
            end
        catch
            % 跳过
        end

        % 每100次打印进度点
        if mod(i, 100) == 0, fprintf('.'); end
    end

    % 记录PDR
    pdr_a0_rmpr(n_idx)      = success_count_a0_rmpr   / num_iterations;
    pdr_a07_rmpr(n_idx)     = success_count_a07_rmpr  / num_iterations;
    pdr_overlap_path(n_idx) = success_count_overlap   / num_iterations;

    % 记录时延
    lat_a0_rmpr(n_idx)      = latency_sum_a0_rmpr   / max(latency_count_a0_rmpr, 1);
    lat_a07_rmpr(n_idx)     = latency_sum_a07_rmpr  / max(latency_count_a07_rmpr, 1);
    lat_overlap_path(n_idx) = latency_sum_overlap   / max(latency_count_overlap, 1);

    fprintf(' 完成\n');
    fprintf('  PDR    | a=0 RMPR=%.4f | a=0.7 RMPR=%.4f | single=%.4f | overlap=%.4f\n', ...
        pdr_a0_rmpr(n_idx), pdr_a07_rmpr(n_idx), pdr_single_path(n_idx), pdr_overlap_path(n_idx));
    fprintf('  Lat(ms)| a=0 RMPR=%.4f | a=0.7 RMPR=%.4f | single=%.4f | overlap=%.4f\n', ...
        lat_a0_rmpr(n_idx)*1000, lat_a07_rmpr(n_idx)*1000, ...
        lat_single_path(n_idx)*1000, lat_overlap_path(n_idx)*1000);
end

%% ===== 5. 绘图结果展现：PDR vs n =====
figure('Color', 'w', 'Position', [100, 100, 900, 650]);

plot(n_redundancy_list, pdr_a0_rmpr, ...
    'LineStyle', '-', ...
    'Marker', 'o', ...
    'LineWidth', 2, ...
    'MarkerSize', 8, ...
    'DisplayName', 'a = 0（RMPR）');
hold on;

plot(n_redundancy_list, pdr_a07_rmpr, ...
    'LineStyle', '--', ...
    'Marker', 's', ...
    'LineWidth', 2, ...
    'MarkerSize', 8, ...
    'DisplayName', 'a = 0.7（改进算法）');

plot(n_redundancy_list, pdr_single_path, ...
    'LineStyle', '-.', ...
    'Marker', '^', ...
    'LineWidth', 2, ...
    'MarkerSize', 8, ...
    'DisplayName', 'Single-path（单一路径）');

plot(n_redundancy_list, pdr_overlap_path, ...
    'LineStyle', ':', ...
    'Marker', 'd', ...
    'LineWidth', 2, ...
    'MarkerSize', 8, ...
    'DisplayName', 'Overlap multi-path（可重叠多路径）');

grid on;
xlabel('冗余度 n', 'FontSize', 12);
ylabel('数据包投递率 (PDR)', 'FontSize', 12);
title('\sigma_{s0}=2.2 时，不同算法的 PDR 随冗余度变化', 'FontSize', 14);
legend('Location', 'best', 'FontSize', 10);
ylim([0 1.1]);
set(gca, 'XTick', n_redundancy_list);

%% ===== 6. 绘图结果展现：Latency vs n =====
figure('Color', 'w', 'Position', [120, 120, 900, 650]);

plot(n_redundancy_list, lat_a0_rmpr * 1000, ...
    'LineStyle', '-', ...
    'Marker', 'o', ...
    'LineWidth', 2, ...
    'MarkerSize', 8, ...
    'DisplayName', 'a = 0（RMPR）');
hold on;

plot(n_redundancy_list, lat_a07_rmpr * 1000, ...
    'LineStyle', '--', ...
    'Marker', 's', ...
    'LineWidth', 2, ...
    'MarkerSize', 8, ...
    'DisplayName', 'a = 0.7（改进算法）');

plot(n_redundancy_list, lat_single_path * 1000, ...
    'LineStyle', '-.', ...
    'Marker', '^', ...
    'LineWidth', 2, ...
    'MarkerSize', 8, ...
    'DisplayName', 'Single-path（单一路径）');

plot(n_redundancy_list, lat_overlap_path * 1000, ...
    'LineStyle', ':', ...
    'Marker', 'd', ...
    'LineWidth', 2, ...
    'MarkerSize', 8, ...
    'DisplayName', 'Overlap multi-path（可重叠多路径）');

grid on;
xlabel('冗余度 n', 'FontSize', 12);
ylabel('平均方案执行时延 (ms)', 'FontSize', 12);
title('\sigma_{s0}=2.2 时，不同算法的时延随冗余度变化', 'FontSize', 14);
legend('Location', 'best', 'FontSize', 10);
set(gca, 'XTick', n_redundancy_list);

%% ===== 7. 控制台打印汇总 =====
fprintf('\n================ 结果汇总 ================\n');
for n_idx = 1:num_n
    fprintf('n = %d\n', n_redundancy_list(n_idx));
    fprintf('  PDR    | a=0 RMPR=%.4f | a=0.7 RMPR=%.4f | single=%.4f | overlap=%.4f\n', ...
        pdr_a0_rmpr(n_idx), pdr_a07_rmpr(n_idx), pdr_single_path(n_idx), pdr_overlap_path(n_idx));
    fprintf('  Lat(ms)| a=0 RMPR=%.4f | a=0.7 RMPR=%.4f | single=%.4f | overlap=%.4f\n', ...
        lat_a0_rmpr(n_idx)*1000, lat_a07_rmpr(n_idx)*1000, ...
        lat_single_path(n_idx)*1000, lat_overlap_path(n_idx)*1000);
end
fprintf('==========================================\n');

%% ===== 8. 辅助函数 (脚本底部) =====
function A = simulate_link_failure_oisl(dist_mat, sigma_s0, min_dist_threshold, verbose)
    A = dist_mat;
    A(A < min_dist_threshold) = inf; % 硬截断
    [ii, jj] = find(triu(isfinite(A) & A > 0, 1));
    for k = 1:numel(ii)
        p_f = oisl_failure_prob(A(ii(k), jj(k)), sigma_s0);
        if rand() < p_f
            A(ii(k), jj(k)) = inf;
            A(jj(k), ii(k)) = inf;
        end
    end
end

function alive = is_path_alive(p, A)
    alive = true;
    for k = 1:numel(p)-1
        if ~isfinite(A(p(k)+1, p(k+1)+1))
            alive = false;
            return;
        end
    end
end
