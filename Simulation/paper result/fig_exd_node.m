%% ===== 0. 安全初始化与数据保护 （路径图）=====
clc;

%% ===== 1. 参数设置 =====
sigma_s0_list = [2.15];
num_s0 = numel(sigma_s0_list);

num_iterations = 400;     % 每个时刻、每个sigma下的蒙特卡洛次数

% 系统参数
target_sat = 402;         % 目标卫星
L = 4;                    % 协同接入卫星数
M_bits = 1e6;             % 总传输数据量
n_redundancy = 2;         % 冗余度
min_dist_limit = 600e3;   % 硬截断阈值
ground_station_node = 0;  % 地面站节点（0开始）

% 时刻数
time_diedai = 3;

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
hotspots_by_time{1} = [569 529 489 647];
hotspots_by_time{2} = [648 489 449];
hotspots_by_time{6} = [650];
hotspots_by_time{11} = [528 488 448 566 448];
hotspots_by_time{15} = [488 448 408 568 528];
hotspots_by_time{45} = [488 448 408 568 528];
hotspots_by_time{76} = [602 562 523];
hotspots_by_time{86} = [522 482 443];
hotspots_by_time{97} = [482 563];

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
% 规划失败时记为 NaN
pdr_matrix = nan(num_s0, num_methods, time_diedai);

% 记录规划是否成功
plan_success_matrix = false(num_s0, num_methods, time_diedai);

% 保存固定规划结果
last_paths = cell(num_s0, num_methods, time_diedai);
last_w_vec = cell(num_s0, num_methods, time_diedai);

% 保存每个时刻预设的热点节点（0-based）
hotspots_selected_by_time = hotspots_by_time;

% 保存每个时刻实际抽中失效的热点节点（0-based）
% 这里只做诊断记录，不再并入下一时刻永久失效
hotspots_sampled_failed_by_time = cell(time_diedai, 1);

% 保存每个时刻 node-weight 方法的热点命中数
weighted_hot_hits_record = nan(num_s0, time_diedai);

% 保存每个时刻平均每次抽中失效的热点节点数
avg_hotspot_failed_record = nan(num_s0, time_diedai);

% 历史累计失效节点（从下一时刻开始进入规划图）
% 只由 fail_events_by_time 驱动
failed_nodes_persistent = [];

% ===== 新增：保存每个时刻可视化所需信息 =====
% 保存每个时刻的规划图（去掉永久失效节点之后）
dist_matrix_saved = cell(num_s0, time_diedai);

% 保存每个时刻的原始静态图（可选）
dist_matrix_raw_saved = cell(num_s0, time_diedai);

% 保存每个时刻接入卫星
sources_saved = cell(num_s0, time_diedai);

% 保存每个时刻目标卫星
target_saved = nan(num_s0, time_diedai);

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
        valid_method = false(1, num_methods);

        % ===== 方法1：Single-path =====
        try
            curr_W = W_a0;
            [success_sp_plan, ~, selected_path_sp, ~, ~, ~] = ...
                run_single_path(curr_W, curr_W, Sources, target_sat);

            if success_sp_plan == 1 && ~isempty(selected_path_sp)
                curr_last_paths{1} = {selected_path_sp};
                curr_last_w_vec{1} = 1;
                valid_method(1) = true;
            end
        catch
            curr_last_paths{1} = [];
            curr_last_w_vec{1} = [];
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
            valid_method(2) = true;
        catch
            curr_last_paths{2} = [];
            curr_last_w_vec{2} = [];
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
            valid_method(4) = true;
        catch
            curr_last_paths{4} = [];
            curr_last_w_vec{4} = [];
        end

        % ===== 方法5：Overlap multi-path（无节点加权）=====
        try
            curr_W = W_a0;
            in_overlap = rmpr_build_waterfill_inputs_multi_source_overlap( ...
                t_idx, dist_matrix, curr_W, result3, Sources, ...
                target_sat, L, M_bits, alpha_a0_method, T_max_a0, false);

            [w_vec_overlap, ~] = solve_routing_optimization( ...
                in_overlap.q_sec, in_overlap.gamma_sec, n_redundancy);

            curr_last_paths{5} = in_overlap.paths;
            curr_last_w_vec{5} = w_vec_overlap;
            valid_method(5) = true;
        catch
            curr_last_paths{5} = [];
            curr_last_w_vec{5} = [];
        end

        % ===== 新增：保存当前时刻可视化所需图信息 =====
        dist_matrix_saved{s_idx, t_idx} = dist_matrix;
        dist_matrix_raw_saved{s_idx, t_idx} = dist_matrix_raw;
        sources_saved{s_idx, t_idx} = Sources;
        target_saved(s_idx, t_idx) = target_sat;

        % 记录规划是否成功
        plan_success_matrix(s_idx, :, t_idx) = valid_method;

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
        % E. Monte Carlo：当前时刻执行阶段抽奖
        %    注意：这里不重规划
        %    先统一生成一套随机失效样本，5种方法共用
        % -----------------------------
        success_counts = zeros(1, num_methods);

        adj_exec_bank = cell(num_iterations, 1);
        hotspot_failed_counter_total = 0;
        sampled_failed_hotspots_union = [];

        for i = 1:num_iterations
            new_failed_nodes_now = fail_events_by_time{t_idx};

            [adj_exec_bank{i}, sampled_failed_hotspots_now] = simulate_link_failure_oisl( ...
                dist_matrix, current_sigma_s0, min_dist_limit, new_failed_nodes_now, false, ...
                hotspot_nodes_0based, enable_hotspot_random_failure, hotspot_fail_prob);

            hotspot_failed_counter_total = hotspot_failed_counter_total + numel(sampled_failed_hotspots_now);
            sampled_failed_hotspots_union = unique([sampled_failed_hotspots_union, sampled_failed_hotspots_now]); %#ok<AGROW>
        end

        % 用同一批样本评估5种方法
        for i = 1:num_iterations
            adj_exec = adj_exec_bank{i};

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
        for m_idx = 1:num_methods
            if valid_method(m_idx)
                pdr_matrix(s_idx, m_idx, t_idx) = success_counts(m_idx) / num_iterations;
            else
                pdr_matrix(s_idx, m_idx, t_idx) = NaN;
            end
        end

        avg_hotspot_failed_record(s_idx, t_idx) = hotspot_failed_counter_total / num_iterations;
        hotspots_sampled_failed_by_time{t_idx} = sampled_failed_hotspots_union;

        fprintf('\n完成\n');
        if valid_method(1)
            fprintf(' | PDR(Single-path)=%.4f', pdr_matrix(s_idx, 1, t_idx));
        else
            fprintf(' | PDR(Single-path)=NaN');
        end
        if valid_method(2)
            fprintf(' | PDR(a=0.7 RMPR)=%.4f', pdr_matrix(s_idx, 2, t_idx));
        else
            fprintf(' | PDR(a=0.7 RMPR)=NaN');
        end
        if valid_method(3)
            fprintf(' | PDR(Node-weight)=%.4f', pdr_matrix(s_idx, 3, t_idx));
        else
            fprintf(' | PDR(Node-weight)=NaN');
        end
        if valid_method(4)
            fprintf(' | PDR(a=0 RMPR)=%.4f', pdr_matrix(s_idx, 4, t_idx));
        else
            fprintf(' | PDR(a=0 RMPR)=NaN');
        end
        if valid_method(5)
            fprintf(' | PDR(Overlap)=%.4f\n', pdr_matrix(s_idx, 5, t_idx));
        else
            fprintf(' | PDR(Overlap)=NaN\n');
        end

        fprintf(' | 平均每次抽中失效热点数 = %.4f\n', avg_hotspot_failed_record(s_idx, t_idx));
        fprintf(' | 本时刻所有 Monte Carlo 中曾被抽中失效的热点节点并集（仅诊断，不写回永久状态） = %s\n', ...
            mat2str(hotspots_sampled_failed_by_time{t_idx}));

        if ~isnan(weighted_hot_hits_record(s_idx, t_idx))
            fprintf(' | Node-weight 方法加权热点命中数 = %.6f\n', weighted_hot_hits_record(s_idx, t_idx));
        end

        for m_idx = 1:num_methods
            last_paths{s_idx, m_idx, t_idx} = curr_last_paths{m_idx};
            last_w_vec{s_idx, m_idx, t_idx} = curr_last_w_vec{m_idx};
        end
    end

    % -----------------------------
    % G. 当前时刻结束后：只把手动指定的真实失效节点并入累计失效
    % -----------------------------
    failed_nodes_persistent = unique([ ...
        failed_nodes_persistent, ...
        fail_events_by_time{t_idx} ...
        ]);

    fprintf('时刻 %d 结束后，累计失效节点更新为 = %s\n', ...
        t_idx, mat2str(failed_nodes_persistent));
end

%% ===== 5. 仅保留三种方法的路径可视化，并排合成一张图 =====
% 方法顺序：
% 2 -> a=0.7 RMPR（Link-weighted）
% 4 -> a=0 RMPR
% 3 -> a=0.7 + omega=0.5 Node-weight RMPR（Link-node weighted）

t_plot_compare = 2;   % 你想看的时刻
s_plot_compare = 1;   % sigma索引

figure('Color','w','Position',[100,100,1450,430]);

% --------------------- 子图 1：Link-weighted ---------------------
subplot(1,3,1);
plot_routing_avoidance_snapshot_subplot( ...
    dist_matrix_saved{s_plot_compare, t_plot_compare}, ...
    last_paths{s_plot_compare, 2, t_plot_compare}, ...
    last_w_vec{s_plot_compare, 2, t_plot_compare}, ...
    hotspots_by_time{t_plot_compare}, ...
    sources_saved{s_plot_compare, t_plot_compare}, ...
    target_saved(s_plot_compare, t_plot_compare), ...
    '', '');

% --------------------- 子图 2：RMPR ---------------------
subplot(1,3,2);
plot_routing_avoidance_snapshot_subplot( ...
    dist_matrix_saved{s_plot_compare, t_plot_compare}, ...
    last_paths{s_plot_compare, 4, t_plot_compare}, ...
    last_w_vec{s_plot_compare, 4, t_plot_compare}, ...
    hotspots_by_time{t_plot_compare}, ...
    sources_saved{s_plot_compare, t_plot_compare}, ...
    target_saved(s_plot_compare, t_plot_compare), ...
    '', '');

% --------------------- 子图 3：Link-node weighted ---------------------
subplot(1,3,3);
plot_routing_avoidance_snapshot_subplot( ...
    dist_matrix_saved{s_plot_compare, t_plot_compare}, ...
    last_paths{s_plot_compare, 3, t_plot_compare}, ...
    last_w_vec{s_plot_compare, 3, t_plot_compare}, ...
    hotspots_by_time{t_plot_compare}, ...
    sources_saved{s_plot_compare, t_plot_compare}, ...
    target_saved(s_plot_compare, t_plot_compare), ...
    '', '');

% 如果 MATLAB 支持 sgtitle 就加总标题；不支持也没关系
if exist('sgtitle','file') == 2 || exist('sgtitle','builtin') == 5
    sgtitle(sprintf('Routing Path Comparison at t = %d', t_plot_compare), ...
        'FontSize', 14, 'FontWeight', 'normal');
end

% 如需导出 eps，取消下面注释
% print(gcf, 'img/pic/fig15', '-depsc', '-r600');


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

%% ===== 8. 可视化：某个时刻路径如何绕开热点节点 =====
% method_plot:
% 1 -> Single-path
% 2 -> a=0.7 RMPR
% 3 -> Node-weight RMPR
% 4 -> a=0 RMPR
% 5 -> Overlap multi-path
t_plot = 2;
s_plot = 1;
method_plot = 3;   % 默认看 Node-weight RMPR

plot_routing_avoidance_snapshot( ...
    dist_matrix_saved{s_plot, t_plot}, ...
    last_paths{s_plot, method_plot, t_plot}, ...
    last_w_vec{s_plot, method_plot, t_plot}, ...
    hotspots_by_time{t_plot}, ...
    sources_saved{s_plot, t_plot}, ...
    target_saved(s_plot, t_plot), ...
    t_plot, method_plot);

%% ===== 9. 可视化：三种路径方案合并成一张图 =====
% 这里建议顺序与论文中保持一致：
% 左：Link-weighted
% 中：RMPR
% 右：Link-node weighted

t_plot_compare = 2;
s_plot_compare = 1;

figure('Color','w','Position',[100,100,1450,430]);

tl = tiledlayout(1,3,'TileSpacing','compact','Padding','compact');

% --------------------- 子图 1：Link-weighted ---------------------
nexttile;
plot_routing_avoidance_snapshot_subplot( ...
    dist_matrix_saved{s_plot_compare, t_plot_compare}, ...
    last_paths{s_plot_compare, 2, t_plot_compare}, ...
    last_w_vec{s_plot_compare, 2, t_plot_compare}, ...
    hotspots_by_time{t_plot_compare}, ...
    sources_saved{s_plot_compare, t_plot_compare}, ...
    target_saved(s_plot_compare, t_plot_compare), ...
    '', 'Link-weighted');

% --------------------- 子图 2：RMPR ---------------------
nexttile;
plot_routing_avoidance_snapshot_subplot( ...
    dist_matrix_saved{s_plot_compare, t_plot_compare}, ...
    last_paths{s_plot_compare, 4, t_plot_compare}, ...
    last_w_vec{s_plot_compare, 4, t_plot_compare}, ...
    hotspots_by_time{t_plot_compare}, ...
    sources_saved{s_plot_compare, t_plot_compare}, ...
    target_saved(s_plot_compare, t_plot_compare), ...
    '', 'RMPR');

% --------------------- 子图 3：Link-node weighted ---------------------
nexttile;
plot_routing_avoidance_snapshot_subplot( ...
    dist_matrix_saved{s_plot_compare, t_plot_compare}, ...
    last_paths{s_plot_compare, 3, t_plot_compare}, ...
    last_w_vec{s_plot_compare, 3, t_plot_compare}, ...
    hotspots_by_time{t_plot_compare}, ...
    sources_saved{s_plot_compare, t_plot_compare}, ...
    target_saved(s_plot_compare, t_plot_compare), ...
    '', 'Link-node weighted');

% 如果你想导出成 eps：
% print(gcf, 'img/pic/fig15', '-depsc', '-r600');


%% ===== 10. 辅助函数 =====
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

function plot_routing_avoidance_snapshot(A, path_set, w_vec, hotspot_nodes_0based, Sources, target_sat, t_plot, method_plot)
    if isempty(A)
        error('当前时刻的邻接矩阵为空，无法绘图。');
    end

    if isempty(path_set)
        error('当前方法在该时刻没有规划成功，无法绘图。');
    end

    hotspot_nodes_0based = unique(hotspot_nodes_0based(:)');

    N = size(A, 1);

    % -------- 1) 构造图 --------
    A_plot = A;
    A_plot(~isfinite(A_plot)) = 0;
    A_plot = triu(A_plot, 1) + triu(A_plot, 1)';   % 保证对称且无自环

    G = graph(A_plot > 0);

    figure('Color', 'w', 'Position', [120, 80, 1250, 900]);
    p = plot(G, ...
        'Layout', 'force', ...
        'UseGravity', true, ...
        'Iterations', 120, ...
        'NodeLabel', {}, ...
        'MarkerSize', 2, ...
        'EdgeColor', [0.75 0.75 0.75], ...
        'LineWidth', 0.5);

    hold on;

    % -------- 2) 默认所有节点浅灰 --------
    highlight(p, 1:N, ...
        'NodeColor', [0.75 0.75 0.75], ...
        'MarkerSize', 2);

    % -------- 3) 标热点节点 --------
    hotspot_idx = hotspot_nodes_0based + 1;
    hotspot_idx = hotspot_idx(hotspot_idx >= 1 & hotspot_idx <= N);

    if ~isempty(hotspot_idx)
        highlight(p, hotspot_idx, ...
            'NodeColor', [1 0 0], ...
            'MarkerSize', 8);
    end

    % -------- 4) 标接入卫星 --------
    if ~isempty(Sources)
        src_idx = Sources + 1;
        src_idx = src_idx(src_idx >= 1 & src_idx <= N);
        highlight(p, src_idx, ...
            'NodeColor', [0 0.4470 0.7410], ...
            'MarkerSize', 9);
    else
        src_idx = [];
    end

    % -------- 5) 标目标卫星 --------
    tar_idx = target_sat + 1;
    if tar_idx >= 1 && tar_idx <= N
        highlight(p, tar_idx, ...
            'NodeColor', [0.4660 0.6740 0.1880], ...
            'MarkerSize', 10);
    end

    % -------- 6) 画路径 --------
    cmap = lines(max(numel(path_set), 5));

    for k = 1:numel(path_set)
        path0 = path_set{k};
        path1 = path0 + 1;

        for ii = 1:numel(path1)-1
            u = path1(ii);
            v = path1(ii+1);

            if u>=1 && u<=N && v>=1 && v<=N
                highlight(p, u, v, ...
                    'EdgeColor', cmap(k,:), ...
                    'LineWidth', 2.8);
            end
        end

        valid_nodes = path1(path1>=1 & path1<=N);
        highlight(p, valid_nodes, ...
            'NodeColor', cmap(k,:), ...
            'MarkerSize', 5);

        if ~isempty(w_vec) && numel(w_vec) >= k
            mid_idx = path1(ceil(numel(path1)/2));
            text(p.XData(mid_idx), p.YData(mid_idx), ...
                sprintf('  w=%.3f', w_vec(k)), ...
                'Color', cmap(k,:), ...
                'FontSize', 10, ...
                'FontWeight', 'bold');
        end
    end

    % -------- 7) 给热点节点写编号 --------
    for i = 1:numel(hotspot_idx)
        node_i = hotspot_idx(i);
        text(p.XData(node_i), p.YData(node_i), ...
            sprintf('  H:%d', node_i-1), ...
            'Color', 'r', ...
            'FontSize', 10, ...
            'FontWeight', 'bold');
    end

    % 给源节点写编号
    for i = 1:numel(src_idx)
        node_i = src_idx(i);
        text(p.XData(node_i), p.YData(node_i), ...
            sprintf('  S:%d', node_i-1), ...
            'Color', [0 0.4470 0.7410], ...
            'FontSize', 10, ...
            'FontWeight', 'bold');
    end

    % 给目标节点写编号
    if tar_idx >= 1 && tar_idx <= N
        text(p.XData(tar_idx), p.YData(tar_idx), ...
            sprintf('  T:%d', tar_idx-1), ...
            'Color', [0.4660 0.6740 0.1880], ...
            'FontSize', 10, ...
            'FontWeight', 'bold');
    end

    % -------- 8) 统计每条路径是否命中热点 --------
    hot_hit_info = cell(numel(path_set), 1);
    for k = 1:numel(path_set)
        p0 = path_set{k};
        p_mid = p0(2:end-1);
        hit_nodes = intersect(p_mid, hotspot_nodes_0based);
        hot_hit_info{k} = hit_nodes;
    end

    method_names = { ...
        'Single-path', ...
        'a=0.7 RMPR', ...
        'Node-weight RMPR', ...
        'a=0 RMPR', ...
        'Overlap multi-path'};

    title(sprintf('t = %d, method = %s : routing vs hotspot nodes', ...
        t_plot, method_names{method_plot}), 'FontSize', 14);

    fprintf('\n==================================================\n');
    fprintf('时刻 t = %d, 方法 = %s\n', t_plot, method_names{method_plot});
    fprintf('热点节点(0-based) = %s\n', mat2str(hotspot_nodes_0based));
    for k = 1:numel(path_set)
        fprintf('路径 %d = %s\n', k, mat2str(path_set{k}));
        if ~isempty(w_vec) && numel(w_vec) >= k
            fprintf('       权重 = %.6f\n', w_vec(k));
        end
        fprintf('       中间节点 = %s\n', mat2str(path_set{k}(2:end-1)));
        fprintf('       命中热点 = %s\n', mat2str(hot_hit_info{k}));
    end
    fprintf('==================================================\n');
end

function plot_routing_avoidance_snapshot_subplot(A, path_set, w_vec, hotspot_nodes_0based, Sources, target_sat, ~, plot_name)
    if isempty(A) || isempty(path_set)
        title(sprintf('%s (Planning Failed)', plot_name), 'FontSize', 12, 'FontWeight', 'normal');
        axis off;
        return;
    end

    hotspot_nodes_0based = unique(hotspot_nodes_0based(:)');
    N = size(A,1);

    A_plot = A;
    A_plot(~isfinite(A_plot)) = 0;
    A_plot = triu(A_plot,1) + triu(A_plot,1)';   % 保证对称
    G = graph(A_plot > 0);

    p = plot(G, ...
        'Layout', 'force', ...
        'UseGravity', true, ...
        'Iterations', 120, ...
        'NodeLabel', {}, ...
        'MarkerSize', 2, ...
        'EdgeColor', [0.8 0.8 0.8], ...
        'LineWidth', 0.5);
    hold on;

    % 默认所有节点浅灰
    highlight(p, 1:N, 'NodeColor', [0.75 0.75 0.75], 'MarkerSize', 2);

    % 热点节点
    hotspot_idx = unique(hotspot_nodes_0based + 1);
    hotspot_idx = hotspot_idx(hotspot_idx >= 1 & hotspot_idx <= N);
    if ~isempty(hotspot_idx)
        highlight(p, hotspot_idx, 'NodeColor', [1 0 0], 'MarkerSize', 8);
    end

    % 接入卫星
    src_idx = Sources + 1;
    src_idx = src_idx(src_idx >= 1 & src_idx <= N);
    if ~isempty(src_idx)
        highlight(p, src_idx, 'NodeColor', [0 0.4470 0.7410], 'MarkerSize', 9);
    end

    % 目标卫星
    tar_idx = target_sat + 1;
    if tar_idx >= 1 && tar_idx <= N
        highlight(p, tar_idx, 'NodeColor', [0.4660 0.6740 0.1880], 'MarkerSize', 10);
    end

    % 画路径
    cmap = lines(max(numel(path_set), 5));
    for k = 1:numel(path_set)
        path1 = path_set{k} + 1;

        for ii = 1:numel(path1)-1
            highlight(p, path1(ii), path1(ii+1), ...
                'EdgeColor', cmap(k,:), 'LineWidth', 2.6);
        end

        valid_nodes = path1(path1 >= 1 & path1 <= N);
        highlight(p, valid_nodes, ...
            'NodeColor', cmap(k,:), ...
            'MarkerSize', 5);

        if ~isempty(w_vec) && numel(w_vec) >= k
            mid_idx = path1(ceil(numel(path1)/2));
            text(p.XData(mid_idx), p.YData(mid_idx), ...
                sprintf('  %.3f', w_vec(k)), ...
                'Color', cmap(k,:), ...
                'FontSize', 9, ...
                'FontWeight', 'bold');
        end
    end

    % 热点节点编号
    for i = 1:numel(hotspot_idx)
        node_i = hotspot_idx(i);
        text(p.XData(node_i), p.YData(node_i), ...
            sprintf('  H:%d', node_i-1), ...
            'Color', 'r', ...
            'FontSize', 9, ...
            'FontWeight', 'bold');
    end

    % 源节点编号
    for i = 1:numel(src_idx)
        node_i = src_idx(i);
        text(p.XData(node_i), p.YData(node_i), ...
            sprintf('  S:%d', node_i-1), ...
            'Color', [0 0.4470 0.7410], ...
            'FontSize', 9, ...
            'FontWeight', 'bold');
    end

    % 目标节点编号
    if tar_idx >= 1 && tar_idx <= N
        text(p.XData(tar_idx), p.YData(tar_idx), ...
            sprintf('  T:%d', tar_idx-1), ...
            'Color', [0.4660 0.6740 0.1880], ...
            'FontSize', 9, ...
            'FontWeight', 'bold');
    end

    title(plot_name, 'FontSize', 13, 'FontWeight', 'normal');
    axis off;
    set(gca, 'LooseInset', max(get(gca, 'TightInset'), 0.02));
end
