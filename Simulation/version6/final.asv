%% ===== 0. 安全初始化与数据保护 =====
clc;

%% ===== 1. 参数设置 =====
sigma_s0_list = [2.15];
num_s0 = numel(sigma_s0_list);

num_iterations = 1;     % 每个时刻、每个sigma下的蒙特卡洛次数

% 系统参数
target_sat = 402;         % 目标卫星
L = 4;                    % 协同接入卫星数
M_bits = 1e6;             % 总传输数据量 (bits)
n_redundancy = 2;         % 冗余度
min_dist_limit = 600e3;   % 硬截断阈值
ground_station_node = 0;  % 地面站节点（0开始）
c_light = 299792458;      % 光速

% 时刻数
time_diedai = 2;

% 当前保留的5种方法
% 1 -> Single-path
% 2 -> a=0.7 RMPR（不考虑节点失效）
% 3 -> a=0.7 + omega=0.5 Node-weight RMPR
% 4 -> a=0 RMPR
% 5 -> Overlap multi-path（无节点加权）
num_methods = 5;

% 节点权重法参数
alpha_plain = 0.7;
alpha_node_method = 0.7;
omega_node_method = 0.5;
alpha_a0_method = 0.0;
background_age = 1.0;     % 非热点背景节点年龄
hotspot_age = 13.5;       % 当前时刻指定热点节点年龄

% ===== 热点节点抽奖概率 =====
enable_hotspot_random_failure = true;
hotspot_fail_prob = 0.3;   % 当前时刻热点节点在执行阶段失效概率 p

%% ===== 1.1 预先写好的每个时刻热点节点（0-based）=====
hotspots_by_time = cell(time_diedai, 1);

% ===== 在这里提前写好 =====
if 1 <= time_diedai,  hotspots_by_time{1} = [569 529 489 647]; end
if 2 <= time_diedai,  hotspots_by_time{2} = [648 489 449]; end
if 6 <= time_diedai,  hotspots_by_time{6} = [650]; end
if 11 <= time_diedai, hotspots_by_time{11} = [528 488 448 566 448]; end
if 15 <= time_diedai, hotspots_by_time{15} = [488 448 408 568 528]; end

% 下两项原来会越界，这里加保护
if 255 <= time_diedai, hotspots_by_time{255} = [672 673 674]; end
if 281 <= time_diedai, hotspots_by_time{281} = [668 669 670]; end
if 296 <= time_diedai, hotspots_by_time{296} = [968 967 966]; end

%% ===== 1.2 按时刻设置“新发生失效”的节点（节点编号从0开始）=====
fail_events_by_time = cell(time_diedai, 1);

% 示例：
% fail_events_by_time{3} = [409];
% fail_events_by_time{5} = [100, 120];
% fail_events_by_time{10} = [610];

rng(1);

%% ===== 2. 加载数据 =====
if ~exist('result3', 'var')
    load('无人机-卫星速率数据_result3.mat', 'result3');
end

if ~exist('result_static', 'var')
    error('工作区中没有 result_static，请先加载 result_static。');
end

%% ===== 3. 结果存储 =====
% PDR: [sigma, method, time]
pdr_matrix = zeros(num_s0, num_methods, time_diedai);

% Delay: [sigma, method, time]，单位：秒
delay_matrix = nan(num_s0, num_methods, time_diedai);

% 保存固定规划结果
last_paths = cell(num_s0, num_methods, time_diedai);
last_w_vec = cell(num_s0, num_methods, time_diedai);
last_access_sats = cell(num_s0, num_methods, time_diedai);

% 保存每个时刻预设的热点节点（0-based）
hotspots_selected_by_time = hotspots_by_time;

% 保存每个时刻实际抽中失效的热点节点（0-based）
hotspots_sampled_failed_by_time = cell(time_diedai, 1);

% 保存每个时刻 node-weight 方法的热点命中数
weighted_hot_hits_record = nan(num_s0, time_diedai);

% 保存每个时刻平均每次抽中失效的热点节点数
avg_hotspot_failed_record = nan(num_s0, time_diedai);

% 历史累计失效节点（从下一时刻开始进入规划图）
failed_nodes_persistent = [];

%% ===== 4. 主循环：遍历所有时刻 =====
for t_idx = 1:time_diedai
    fprintf('\n==================================================\n');
    fprintf('开始处理时刻 t_idx = %d / %d\n', t_idx, time_diedai);
    fprintf('当前累计失效节点 = %s\n', mat2str(failed_nodes_persistent));
    fprintf('==================================================\n');

    %% --- 4.1 提取当前时刻静态图 ---
    result_static_temp = result_static(t_idx);
    result_static_for_build = repmat(result_static_temp, t_idx, 1);

    [adj_matrices_static_temp, has_valid_links] = create_diagram_dynamic( ...
        t_idx, result3, result_static_for_build, ground_station_node);

    if isempty(adj_matrices_static_temp) || ~has_valid_links
        fprintf('时刻 %d: 未生成有效邻接矩阵，跳过当前时刻。\n', t_idx);
        failed_nodes_persistent = unique([failed_nodes_persistent, fail_events_by_time{t_idx}]);
        continue;
    end

    dist_matrix_raw = adj_matrices_static_temp;

    % 当前规划图：去掉历史累计失效节点
    dist_matrix = remove_failed_nodes_from_graph(dist_matrix_raw, failed_nodes_persistent);

    % 当前时刻接入卫星
    drone_sat_ids = result3(t_idx).sat_ids(:);
    drone_sat_ids = drone_sat_ids(isfinite(drone_sat_ids));
    Sources = drone_sat_ids(1:min(end, L));

    if isempty(Sources)
        fprintf('时刻 %d: 没有可用接入卫星，跳过。\n', t_idx);
        failed_nodes_persistent = unique([failed_nodes_persistent, fail_events_by_time{t_idx}]);
        continue;
    end

    %% --- 4.2 当前时刻直接读取预设热点节点 ---
    hotspot_nodes_0based = hotspots_by_time{t_idx};
    if isempty(hotspot_nodes_0based)
        hotspot_nodes_0based = [];
    end

    fprintf('时刻 %d 预设热点节点 = %s\n', t_idx, mat2str(hotspot_nodes_0based));

    %% --- 4.3 对每个 sigma_s0 分别仿真 ---
    for s_idx = 1:num_s0
        current_sigma_s0 = sigma_s0_list(s_idx);
        fprintf('\n>>> 时刻 %d: sigma_s0 = %.2f m\n', t_idx, current_sigma_s0);

        % -----------------------------
        % A. 预生成 a=0.7 和 a=0 的权重矩阵
        % -----------------------------
        [W_plain, T_max_plain, ~] = build_weighted_cost_matrix( ...
            dist_matrix, current_sigma_s0, alpha_plain);

        [W_a0, T_max_a0, ~] = build_weighted_cost_matrix( ...
            dist_matrix, current_sigma_s0, alpha_a0_method);

        curr_last_paths = cell(1, num_methods);
        curr_last_w_vec = cell(1, num_methods);
        curr_last_access_sats = cell(1, num_methods);
        valid_method = false(1, num_methods);

        % ===== 方法1：Single-path =====
        try
            curr_W = W_a0; % 单路径按 a=0 权重图
            [success_sp_plan, ~, selected_path_sp, ~, ~, ~] = ...
                run_single_path(curr_W, curr_W, Sources, target_sat);

            if success_sp_plan == 1 && ~isempty(selected_path_sp)
                curr_last_paths{1} = {selected_path_sp};
                curr_last_w_vec{1} = 1;
                curr_last_access_sats{1} = selected_path_sp(1); % 路径首节点视为接入卫星
                valid_method(1) = true;
            end
        catch
            curr_last_paths{1} = [];
            curr_last_w_vec{1} = [];
            curr_last_access_sats{1} = [];
        end

        % ===== 方法2：a=0.7 RMPR（不考虑节点失效）=====
        try
            in_plain = rmpr_build_waterfill_inputs_multi_source( ...
                t_idx, dist_matrix, W_plain, result3, Sources, ...
                target_sat, L, M_bits, alpha_plain, T_max_plain, false);

            [w_vec_plain, ~] = solve_routing_optimization( ...
                in_plain.q_sec, in_plain.gamma_sec, n_redundancy);

            curr_last_paths{2} = in_plain.paths;
            curr_last_w_vec{2} = w_vec_plain;
            curr_last_access_sats{2} = in_plain.access_sats;
            valid_method(2) = true;
        catch
            curr_last_paths{2} = [];
            curr_last_w_vec{2} = [];
            curr_last_access_sats{2} = [];
        end

        % -----------------------------
        % B. 构造当前时刻节点年龄图
        % -----------------------------
        NumNodes = size(dist_matrix, 1);
        node_ages_special = background_age * ones(1, NumNodes);

        if ~isempty(hotspot_nodes_0based)
            hotspot_idx = hotspot_nodes_0based + 1;
            hotspot_idx = hotspot_idx(hotspot_idx >= 1 & hotspot_idx <= NumNodes);
            node_ages_special(hotspot_idx) = hotspot_age;
        else
            hotspot_idx = [];
        end

        if ~isempty(hotspot_idx)
            fprintf('热点节点平均年龄 = %.2f\n', mean(node_ages_special(hotspot_idx)));
        else
            fprintf('当前时刻未设置热点节点。\n');
        end

        bg_idx = setdiff(1:NumNodes, hotspot_idx);
        if ~isempty(bg_idx)
            fprintf('背景节点平均年龄 = %.2f\n', mean(node_ages_special(bg_idx)));
        end

        % ===== 方法3：Node-weight RMPR =====
        try
            [W_node, T_max_node, ~, ~] = build_weighted_cost_matrix_node( ...
                dist_matrix, current_sigma_s0, node_ages_special, ...
                alpha_node_method, omega_node_method);

            in_node = rmpr_build_waterfill_inputs_multi_source( ...
                t_idx, dist_matrix, W_node, result3, Sources, ...
                target_sat, L, M_bits, alpha_node_method, T_max_node, false);

            [w_vec_node, ~] = solve_routing_optimization( ...
                in_node.q_sec, in_node.gamma_sec, n_redundancy);

            curr_last_paths{3} = in_node.paths;
            curr_last_w_vec{3} = w_vec_node;
            curr_last_access_sats{3} = in_node.access_sats;
            valid_method(3) = true;

            % 记录热点命中数
            weighted_hot_hits = 0;
            for k = 1:numel(w_vec_node)
                p = in_node.paths{k};
                p_mid = p(2:end-1);
                hot_hits = sum(ismember(p_mid, hotspot_nodes_0based));
                weighted_hot_hits = weighted_hot_hits + w_vec_node(k) * hot_hits;
            end
            weighted_hot_hits_record(s_idx, t_idx) = weighted_hot_hits;
        catch
            curr_last_paths{3} = [];
            curr_last_w_vec{3} = [];
            curr_last_access_sats{3} = [];
        end

        % ===== 方法4：a=0 RMPR =====
        try
            in_a0 = rmpr_build_waterfill_inputs_multi_source( ...
                t_idx, dist_matrix, W_a0, result3, Sources, ...
                target_sat, L, M_bits, alpha_a0_method, T_max_a0, false);

            [w_vec_a0, ~] = solve_routing_optimization( ...
                in_a0.q_sec, in_a0.gamma_sec, n_redundancy);

            curr_last_paths{4} = in_a0.paths;
            curr_last_w_vec{4} = w_vec_a0;
            curr_last_access_sats{4} = in_a0.access_sats;
            valid_method(4) = true;
        catch
            curr_last_paths{4} = [];
            curr_last_w_vec{4} = [];
            curr_last_access_sats{4} = [];
        end

        % ===== 方法5：Overlap multi-path（无节点加权）=====
        try
            curr_W = W_a0; % overlap 按 a=0 权重图
            in_overlap = rmpr_build_waterfill_inputs_multi_source_overlap( ...
                t_idx, dist_matrix, curr_W, result3, Sources, ...
                target_sat, L, M_bits, alpha_plain, T_max_a0, false);

            [w_vec_overlap, ~] = solve_routing_optimization( ...
                in_overlap.q_sec, in_overlap.gamma_sec, n_redundancy);

            curr_last_paths{5} = in_overlap.paths;
            curr_last_w_vec{5} = w_vec_overlap;
            curr_last_access_sats{5} = in_overlap.access_sats;
            valid_method(5) = true;
        catch
            curr_last_paths{5} = [];
            curr_last_w_vec{5} = [];
            curr_last_access_sats{5} = [];
        end

        % -----------------------------
        % D. 打印5个方法当前时刻最终固定路径
        % -----------------------------
        fprintf('\n----- 时刻 %d 的 5 个方法固定路径与分配结果 -----\n', t_idx);

        method_names = { ...
            'Single-path', ...
            'a=0.7 RMPR', ...
            'a=0.7 + omega=0.5 Node-weight RMPR', ...
            'a=0 RMPR', ...
            'Overlap multi-path'};

        for m_idx = 1:num_methods
            fprintf('算法 %s:\n', method_names{m_idx});

            if isempty(curr_last_paths{m_idx}) || isempty(curr_last_w_vec{m_idx})
                fprintf('  该方法当前时刻规划失败。\n');
                continue;
            end

            if m_idx == 1
                fprintf('  路径: %s\n', mat2str(curr_last_paths{m_idx}{1}));
                fprintf('  权重 = %.6f\n', curr_last_w_vec{m_idx});
                fprintf('  分配数据 = %.2f bits\n', curr_last_w_vec{m_idx} * M_bits);
            else
                for k = 1:numel(curr_last_w_vec{m_idx})
                    p_now = curr_last_paths{m_idx}{k};
                    fprintf('  路径 %d: %s\n', k, mat2str(p_now));
                    fprintf('          权重 = %.6f\n', curr_last_w_vec{m_idx}(k));
                    fprintf('          分配数据 = %.2f bits\n', curr_last_w_vec{m_idx}(k) * M_bits);

                    if m_idx == 3
                        p_mid = p_now(2:end-1);
                        hot_hits = sum(ismember(p_mid, hotspot_nodes_0based));
                        fprintf('          中间节点 = %s\n', mat2str(p_mid));
                        fprintf('          命中当前热点次数 = %d\n', hot_hits);
                    end
                end
            end
        end

        % -----------------------------
        % D.1 计算5个方法当前时刻的固定规划时延
        % 完成时延 = 传播时延 + 传输时延
        % 方案时延 = 所有活动路径中最慢一条
        % -----------------------------
        for m_idx = 1:num_methods
            if valid_method(m_idx)
                delay_matrix(s_idx, m_idx, t_idx) = compute_scheme_delay( ...
                    curr_last_paths{m_idx}, ...
                    curr_last_w_vec{m_idx}, ...
                    curr_last_access_sats{m_idx}, ...
                    dist_matrix, result3, t_idx, M_bits, c_light);
            else
                delay_matrix(s_idx, m_idx, t_idx) = NaN;
            end
        end

        % -----------------------------
        % E. Monte Carlo：当前时刻执行阶段抽奖
        %    注意：这里不重规划
        % -----------------------------
        success_counts = zeros(1, num_methods);
        hotspot_failed_counter_total = 0;
        sampled_failed_hotspots_union = [];

        for i = 1:num_iterations
            new_failed_nodes_now = fail_events_by_time{t_idx};

            [adj_exec, sampled_failed_hotspots_now] = simulate_link_failure_oisl( ...
                dist_matrix, current_sigma_s0, min_dist_limit, new_failed_nodes_now, false, ...
                hotspot_nodes_0based, enable_hotspot_random_failure, hotspot_fail_prob);

            hotspot_failed_counter_total = hotspot_failed_counter_total + numel(sampled_failed_hotspots_now);
            sampled_failed_hotspots_union = unique([sampled_failed_hotspots_union, sampled_failed_hotspots_now]);

            % ===== 方法1：Single-path =====
            try
                if valid_method(1)
                    selected_path_sp = curr_last_paths{1}{1};
                    if is_path_alive(selected_path_sp, adj_exec)
                        success_counts(1) = success_counts(1) + 1;
                    end
                end
            catch
            end

            % ===== 方法2：a=0.7 RMPR =====
            try
                if valid_method(2)
                    received_sum = 0;
                    w_vec = curr_last_w_vec{2};
                    path_set = curr_last_paths{2};

                    for k = 1:numel(w_vec)
                        if is_path_alive(path_set{k}, adj_exec)
                            received_sum = received_sum + w_vec(k);
                        end
                    end

                    if received_sum >= 1.0 - 1e-12
                        success_counts(2) = success_counts(2) + 1;
                    end
                end
            catch
            end

            % ===== 方法3：Node-weight RMPR =====
            try
                if valid_method(3)
                    received_sum = 0;
                    w_vec = curr_last_w_vec{3};
                    path_set = curr_last_paths{3};

                    for k = 1:numel(w_vec)
                        if is_path_alive(path_set{k}, adj_exec)
                            received_sum = received_sum + w_vec(k);
                        end
                    end

                    if received_sum >= 1.0 - 1e-12
                        success_counts(3) = success_counts(3) + 1;
                    end
                end
            catch
            end

            % ===== 方法4：a=0 RMPR =====
            try
                if valid_method(4)
                    received_sum = 0;
                    w_vec = curr_last_w_vec{4};
                    path_set = curr_last_paths{4};

                    for k = 1:numel(w_vec)
                        if is_path_alive(path_set{k}, adj_exec)
                            received_sum = received_sum + w_vec(k);
                        end
                    end

                    if received_sum >= 1.0 - 1e-12
                        success_counts(4) = success_counts(4) + 1;
                    end
                end
            catch
            end

            % ===== 方法5：Overlap multi-path =====
            try
                if valid_method(5)
                    received_sum = 0;
                    w_vec = curr_last_w_vec{5};
                    path_set = curr_last_paths{5};

                    for k = 1:numel(w_vec)
                        if is_path_alive(path_set{k}, adj_exec)
                            received_sum = received_sum + w_vec(k);
                        end
                    end

                    if received_sum >= 1.0 - 1e-12
                        success_counts(5) = success_counts(5) + 1;
                    end
                end
            catch
            end

            if mod(i, 100) == 0
                fprintf('.');
            end
        end

        % -----------------------------
        % F. 存储 PDR、路径、热点抽中统计
        % -----------------------------
        pdr_matrix(s_idx, :, t_idx) = success_counts / num_iterations;
        avg_hotspot_failed_record(s_idx, t_idx) = hotspot_failed_counter_total / num_iterations;
        hotspots_sampled_failed_by_time{t_idx} = sampled_failed_hotspots_union;

        fprintf('\n完成\n');
        fprintf(' | PDR(Single-path)=%.4f', pdr_matrix(s_idx, 1, t_idx));
        fprintf(' | PDR(a=0.7 RMPR)=%.4f', pdr_matrix(s_idx, 2, t_idx));
        fprintf(' | PDR(Node-weight)=%.4f', pdr_matrix(s_idx, 3, t_idx));
        fprintf(' | PDR(a=0 RMPR)=%.4f', pdr_matrix(s_idx, 4, t_idx));
        fprintf(' | PDR(Overlap)=%.4f\n', pdr_matrix(s_idx, 5, t_idx));
        fprintf(' | Delay(Single)=%.3f ms', 1000*delay_matrix(s_idx, 1, t_idx));
        fprintf(' | Delay(a=0.7)=%.3f ms', 1000*delay_matrix(s_idx, 2, t_idx));
        fprintf(' | Delay(Node)=%.3f ms', 1000*delay_matrix(s_idx, 3, t_idx));
        fprintf(' | Delay(a=0)=%.3f ms', 1000*delay_matrix(s_idx, 4, t_idx));
        fprintf(' | Delay(Overlap)=%.3f ms\n', 1000*delay_matrix(s_idx, 5, t_idx));
        fprintf(' | 平均每次抽中失效热点数 = %.4f\n', avg_hotspot_failed_record(s_idx, t_idx));
        fprintf(' | 本时刻所有 Monte Carlo 中曾被抽中失效的热点节点并集 = %s\n', ...
            mat2str(hotspots_sampled_failed_by_time{t_idx}));

        if ~isnan(weighted_hot_hits_record(s_idx, t_idx))
            fprintf(' | Node-weight 方法加权热点命中数 = %.6f\n', weighted_hot_hits_record(s_idx, t_idx));
        end

        for m_idx = 1:num_methods
            last_paths{s_idx, m_idx, t_idx} = curr_last_paths{m_idx};
            last_w_vec{s_idx, m_idx, t_idx} = curr_last_w_vec{m_idx};
            last_access_sats{s_idx, m_idx, t_idx} = curr_last_access_sats{m_idx};
        end
    end

    % -----------------------------
    % G. 当前时刻结束后：把“本时刻抽中失效的热点节点”并入累计失效
    %    从下一时刻开始，规划自动避开它们
    % -----------------------------
    failed_nodes_persistent = unique([ ...
        failed_nodes_persistent, ...
        fail_events_by_time{t_idx}, ...
        hotspots_sampled_failed_by_time{t_idx} ...
        ]);

    fprintf('时刻 %d 结束后，累计失效节点更新为 = %s\n', ...
        t_idx, mat2str(failed_nodes_persistent));
end

%% ===== 5. 绘图：PDR =====
figure('Color', 'w', 'Position', [100, 100, 1300, 720]);
hold on;

time_axis = 0:(time_diedai-1);

color_single = [0.4660, 0.6740, 0.1880];
color_plain  = [0.8500, 0.3250, 0.0980];
color_node   = [0.6350, 0.0780, 0.1840];
color_a0     = [0.0000, 0.4470, 0.7410];
color_ov     = [0.4940, 0.1840, 0.5560];

for s_idx = 1:num_s0
    plot(time_axis, squeeze(pdr_matrix(s_idx, 1, :))', ...
        'LineStyle', '-.', 'Marker', '^', 'Color', color_single, ...
        'LineWidth', 2, 'MarkerSize', 4, ...
        'DisplayName', sprintf('\\sigma_{s0}=%.2f, Single-path', sigma_s0_list(s_idx)));

    plot(time_axis, squeeze(pdr_matrix(s_idx, 2, :))', ...
        'LineStyle', '--', 'Marker', 's', 'Color', color_plain, ...
        'LineWidth', 2, 'MarkerSize', 4, ...
        'DisplayName', sprintf('\\sigma_{s0}=%.2f, a=0.7 RMPR', sigma_s0_list(s_idx)));

    plot(time_axis, squeeze(pdr_matrix(s_idx, 3, :))', ...
        'LineStyle', '-', 'Marker', 'p', 'Color', color_node, ...
        'LineWidth', 2, 'MarkerSize', 5, ...
        'DisplayName', sprintf('\\sigma_{s0}=%.2f, Node-weight RMPR', sigma_s0_list(s_idx)));

    plot(time_axis, squeeze(pdr_matrix(s_idx, 4, :))', ...
        'LineStyle', '-', 'Marker', 'o', 'Color', color_a0, ...
        'LineWidth', 2, 'MarkerSize', 4, ...
        'DisplayName', sprintf('\\sigma_{s0}=%.2f, a=0 RMPR', sigma_s0_list(s_idx)));

    plot(time_axis, squeeze(pdr_matrix(s_idx, 5, :))', ...
        'LineStyle', ':', 'Marker', 'd', 'Color', color_ov, ...
        'LineWidth', 2, 'MarkerSize', 4, ...
        'DisplayName', sprintf('\\sigma_{s0}=%.2f, Overlap', sigma_s0_list(s_idx)));
end

grid on;
xlabel('时刻 t', 'FontSize', 12);
ylabel('数据包投递率 (PDR)', 'FontSize', 12);
title('五种算法在一周期下的 PDR', 'FontSize', 14);
legend('Location', 'best', 'FontSize', 9);
ylim([0 1]);
xlim([0, time_diedai-1]);

if time_diedai <= 20
    set(gca, 'XTick', 0:(time_diedai-1));
elseif time_diedai <= 60
    set(gca, 'XTick', 0:5:(time_diedai-1));
elseif time_diedai <= 120
    set(gca, 'XTick', 0:10:(time_diedai-1));
else
    set(gca, 'XTick', 0:20:(time_diedai-1));
end

%% ===== 5.1 绘图：五种算法的平均端到端时延 =====
mean_delay_ms = squeeze(mean(delay_matrix(1, :, :), 3, 'omitnan')) * 1000;  % 秒 -> ms

figure('Color', 'w', 'Position', [120, 120, 1000, 480]);
b = bar(mean_delay_ms, 0.65);
b.FaceColor = 'flat';
b.CData(1,:) = color_single;
b.CData(2,:) = color_plain;
b.CData(3,:) = color_node;
b.CData(4,:) = color_a0;
b.CData(5,:) = color_ov;
b.EdgeColor = [0 0 0];
b.LineWidth = 1.0;

grid on;
box off;
set(gca, 'FontSize', 11, 'LineWidth', 1.0, 'TickDir', 'out');
ylabel('Average end-to-end delay (ms)', 'FontSize', 12);
title('Average end-to-end delay of five methods', 'FontSize', 14);
set(gca, 'XTick', 1:num_methods, ...
         'XTickLabel', {'Single-path','a=0.7 RMPR','Node-weight','a=0 RMPR','Overlap'});

%% ===== 6. 绘图：Node-weight 方法热点命中数 =====
figure('Color', 'w', 'Position', [150, 150, 1000, 420]);
plot(time_axis, weighted_hot_hits_record(1, :), 'b-o', 'LineWidth', 2.5, 'MarkerSize', 7);
grid on;
xlabel('时刻 t', 'FontSize', 12);
ylabel('加权热点节点命中数', 'FontSize', 12);
title('Node-weight 方法对当前时刻热点节点的命中程度', 'FontSize', 14);

%% ===== 7. 绘图：平均每次抽中失效的热点节点数 =====
figure('Color', 'w', 'Position', [180, 180, 1000, 420]);
plot(time_axis, avg_hotspot_failed_record(1, :), 'r-s', 'LineWidth', 2.5, 'MarkerSize', 7);
grid on;
xlabel('时刻 t', 'FontSize', 12);
ylabel('平均每次抽中失效热点数', 'FontSize', 12);
title('当前时刻热点节点执行阶段随机失效强度', 'FontSize', 14);

%% ===== 8. 辅助函数 =====
function [A, sampled_failed_hotspots_0based] = simulate_link_failure_oisl( ...
    dist_mat, sigma_s0, min_dist_threshold, failed_nodes, verbose, ...
    hotspot_nodes_0based, enable_hotspot_random_failure, hotspot_fail_prob)

    A = dist_mat;
    A(A < min_dist_threshold) = inf;

    sampled_failed_hotspots_0based = [];

    % ---------------------------------------------------------
    % 1) 链路随机失效
    % ---------------------------------------------------------
    [ii, jj] = find(triu(isfinite(A) & A > 0, 1));
    for k = 1:numel(ii)
        p_f = oisl_failure_prob(A(ii(k), jj(k)), sigma_s0);
        if rand() < p_f
            A(ii(k), jj(k)) = inf;
            A(jj(k), ii(k)) = inf;
        end
    end

    % ---------------------------------------------------------
    % 2) 当前时刻热点节点按概率随机失效
    % ---------------------------------------------------------
    if enable_hotspot_random_failure && ~isempty(hotspot_nodes_0based)
        for kk = 1:numel(hotspot_nodes_0based)
            node_0based = hotspot_nodes_0based(kk);
            idx = node_0based + 1;

            if idx >= 1 && idx <= size(A,1)
                if rand() < hotspot_fail_prob
                    A(idx, :) = inf;
                    A(:, idx) = inf;
                    A(idx, idx) = 0;
                    sampled_failed_hotspots_0based(end+1) = node_0based; %#ok<AGROW>
                end
            end
        end
    end

    % ---------------------------------------------------------
    % 3) 本时刻手动指定的新失效节点立即生效
    % ---------------------------------------------------------
    if ~isempty(failed_nodes)
        for kk = 1:numel(failed_nodes)
            failed_idx = failed_nodes(kk) + 1;

            if failed_idx >= 1 && failed_idx <= size(A, 1)
                A(failed_idx, :) = inf;
                A(:, failed_idx) = inf;
                A(failed_idx, failed_idx) = 0;

                if verbose
                    fprintf('指定失效节点 %d 已断开所有关联边。\n', failed_nodes(kk));
                end
            else
                warning('failed_node=%d 超出邻接矩阵范围，已忽略该节点失效设置。', failed_nodes(kk));
            end
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

function A = remove_failed_nodes_from_graph(A, failed_node_ids)
    if isempty(failed_node_ids)
        return;
    end

    for kk = 1:numel(failed_node_ids)
        node_id = failed_node_ids(kk);
        idx = node_id + 1;

        if idx >= 1 && idx <= size(A, 1)
            A(idx, :) = inf;
            A(:, idx) = inf;
            A(idx, idx) = 0;
        else
            warning('failed_node=%d 超出邻接矩阵范围，已忽略该节点失效设置。', node_id);
        end
    end
end

function scheme_delay = compute_scheme_delay(path_set, w_vec, access_sats, dist_matrix, result3, t_idx, M_bits, c_light)
% 计算某个方法在某时刻固定规划后的方案时延
% 方案时延 = 所有活动路径中最慢一条完成时间
%
% 单条路径完成时间：
%   propagation delay + transmission delay
% = sum(d_ij)/c + (w_k * M_bits)/rate_k

    scheme_delay = NaN;

    if isempty(path_set) || isempty(w_vec)
        return;
    end

    num_paths = numel(w_vec);
    path_delays = nan(1, num_paths);

    for k = 1:num_paths
        if w_vec(k) <= 1e-12
            continue;
        end

        path_now = path_set{k};

        % --- 1) 传播时延 ---
        d_total = 0;
        for e = 1:(numel(path_now)-1)
            d_ek = dist_matrix(path_now(e)+1, path_now(e+1)+1);
            if ~isfinite(d_ek)
                d_total = NaN;
                break;
            end
            d_total = d_total + d_ek;
        end

        if ~isfinite(d_total)
            continue;
        end

        prop_delay = d_total / c_light;

        % --- 2) 传输时延 ---
        acc_sat = path_now(1); % 默认用路径首节点做兜底
        if ~isempty(access_sats)
            if numel(access_sats) == 1
                acc_sat = access_sats;
            elseif numel(access_sats) >= k && ~isempty(access_sats(k))
                acc_sat = access_sats(k);
            end
        end

        rate_bps = get_access_rate_bps(result3, t_idx, acc_sat);

        if ~isfinite(rate_bps) || rate_bps <= 0
            continue;
        end

        tx_delay = (w_vec(k) * M_bits) / rate_bps;

        path_delays(k) = prop_delay + tx_delay;
    end

    valid_idx = isfinite(path_delays);
    if any(valid_idx)
        scheme_delay = max(path_delays(valid_idx));
    end
end

function rate_bps = get_access_rate_bps(result3, t_idx, acc_sat)
% 尽量兼容不同字段名
    rate_bps = NaN;

    if ~isfield(result3(t_idx), 'sat_ids')
        return;
    end

    idx_rate = find(result3(t_idx).sat_ids == acc_sat, 1);
    if isempty(idx_rate)
        return;
    end

    % 优先找 Mbps 字段
    candidate_fields = {'rates_Mbps', 'rate_Mbps', 'rates', 'downlink_rate_Mbps', 'downlink_rates_Mbps'};
    for i = 1:numel(candidate_fields)
        f = candidate_fields{i};
        if isfield(result3(t_idx), f)
            val = result3(t_idx).(f);
            if numel(val) >= idx_rate && isfinite(val(idx_rate))
                rate_bps = val(idx_rate) * 1e6;
                return;
            end
        end
    end
end
