%% =========================================================
%  四种算法对比：固定 L=4，比较 n=2,3,4 下的
%  1) Average Delay
%  2) Average PDR
%  3) Average Hop Count
%
%  算法（真实存储顺序）：
%   1) Single-path
%   2) Node-weight RMPR
%   3) RMPR
%   4) Overlap multi-path
%
%  说明：
%  - task size 固定为 M_bits = 1e6
%  - Node-weight RMPR 使用 a=0.7 + 节点年龄权重构造
%  - RMPR 这里对应 a=0 的构造
%  - Single-path 在每个时刻、每个 sigma 下只规划一次，不随 n 变化
%  - 时延定义：所有活动路径中最慢完成路径的完成时间
%  - 跳数定义：按分配权重加权的平均跳数
%  - PDR 定义：在“成功规划”的前提下，Monte Carlo 下成功交付比例
%  - 新增：Planning Success Rate（规划成功率）
%% =========================================================
clc;

%% ===== 1. 参数设置 =====
sigma_s0_list = [2.15];
num_s0 = numel(sigma_s0_list);

num_iterations = 400;      % 每个时刻、每个 sigma、每个 n 的蒙特卡洛次数

target_sat = 402;          % 目标卫星
L_fixed = 4;               % 固定 L=4
n_list = [2, 3, 4];
num_n = numel(n_list);

M_bits = 1e6;              % 固定任务大小
min_dist_limit = 600e3;    % 硬截断阈值
ground_station_node = 0;   % 地面站节点（0-based）
time_diedai = 100;         % 时间片数
c_light = 299792458;       % 光速

% ===== 四种算法（真实存储顺序）=====
% 1) Single-path
% 2) Node-weight RMPR
% 3) RMPR
% 4) Overlap multi-path
num_methods = 4;
raw_method_names = { ...
    'Single path', ...
    'LNW-RMPR', ...
    'RMPR', ...
    'Overlapped multipath'};

% 画图顺序：把 Node-weight 放最左
plot_order = [2, 1, 3, 4];
plot_method_names = raw_method_names(plot_order);

% ===== 权重参数 =====
alpha_node_method = 0.7;
omega_node_method = 0.5;
alpha_rmpr_method = 0.0;   % RMPR 对应 a=0

background_age = 1.0;
hotspot_age = 13.5;

% ===== 热点节点随机失效 =====
enable_hotspot_random_failure = true;
hotspot_fail_prob = 0.3;

% ===== 输出控制 =====
enable_detail_print = false;   % true: 打印每条路径和权重；false: 只打印摘要

rng(1);

%% ===== 1.1 热点节点设置（0-based）=====
hotspots_by_time = cell(time_diedai, 1);

% ===== 手动填写 =====
hotspots_by_time{1} = [569 529 489 647];
hotspots_by_time{2} = [648 489 449];
hotspots_by_time{6} = [650];
hotspots_by_time{11} = [528 488 448 566 448];
hotspots_by_time{15} = [488 448 408 568 528];
hotspots_by_time{45} = [488 448 408 568 528];
hotspots_by_time{76} = [602 562 523];
hotspots_by_time{86} = [522 482 443];
hotspots_by_time{97} = [482 563];

%% ===== 1.2 手动指定必失效节点（0-based）=====
fail_events_by_time = cell(time_diedai, 1);

% 示例：
% fail_events_by_time{1} = [409];
% fail_events_by_time{2} = [610];


%% ===== 2. 加载数据 =====
if ~exist('result3', 'var')
    load('无人机-卫星速率数据_result3.mat', 'result3');
end

if ~exist('result_static', 'var')
    error('工作区中没有 result_static，请先加载 result_static。');
end

%% ===== 3. 结果存储 =====
% 维度: [sigma, n_idx, method, time]
pdr_matrix   = nan(num_s0, num_n, num_methods, time_diedai);   % 仅对成功规划样本统计
delay_matrix = nan(num_s0, num_n, num_methods, time_diedai);   % 单位：秒
hop_matrix   = nan(num_s0, num_n, num_methods, time_diedai);   % 加权平均跳数

% 规划成功率
plan_success_matrix = false(num_s0, num_n, num_methods, time_diedai);

last_paths = cell(num_s0, num_n, num_methods, time_diedai);
last_w_vec = cell(num_s0, num_n, num_methods, time_diedai);

weighted_hot_hits_record = nan(num_s0, num_n, time_diedai);
avg_hotspot_failed_record = nan(num_s0, num_n, time_diedai);

% 仅做诊断记录，不再写回 persistent
hotspots_sampled_failed_diag_by_time = cell(time_diedai, 1);

% 历史累计失效节点：只由 fail_events_by_time 驱动
failed_nodes_persistent = [];

%% ===== 4. 主循环 =====
for t_idx = 1:time_diedai
    fprintf('\n==================================================\n');
    fprintf('开始处理时刻 t_idx = %d / %d\n', t_idx, time_diedai);
    fprintf('当前累计失效节点 = %s\n', mat2str(failed_nodes_persistent));
    fprintf('==================================================\n');

    %% --- 4.1 构建当前时刻邻接矩阵 ---
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
    dist_matrix = remove_failed_nodes_from_graph(dist_matrix_raw, failed_nodes_persistent);

    %% --- 4.2 当前时刻接入卫星 ---
    drone_sat_ids = result3(t_idx).sat_ids(:);
    drone_sat_ids = drone_sat_ids(isfinite(drone_sat_ids));
    Sources = drone_sat_ids(1:min(end, L_fixed));

    if isempty(Sources)
        fprintf('时刻 %d: 没有可用接入卫星，跳过。\n', t_idx);
        failed_nodes_persistent = unique([failed_nodes_persistent, fail_events_by_time{t_idx}]);
        continue;
    end

    %% --- 4.3 当前时刻热点节点 ---
    hotspot_nodes_0based = hotspots_by_time{t_idx};
    if isempty(hotspot_nodes_0based)
        hotspot_nodes_0based = [];
    end
    fprintf('时刻 %d 预设热点节点 = %s\n', t_idx, mat2str(hotspot_nodes_0based));

    %% --- 4.4 构造节点年龄图 ---
    NumNodes = size(dist_matrix, 1);
    node_ages_special = background_age * ones(1, NumNodes);

    if ~isempty(hotspot_nodes_0based)
        hotspot_idx = hotspot_nodes_0based + 1;
        hotspot_idx = hotspot_idx(hotspot_idx >= 1 & hotspot_idx <= NumNodes);
        node_ages_special(hotspot_idx) = hotspot_age;
        fprintf('热点节点平均年龄 = %.2f\n', mean(node_ages_special(hotspot_idx)));
    else
        hotspot_idx = [];
        fprintf('当前时刻未设置热点节点。\n');
    end

    %% --- 4.5 遍历 sigma ---
    for s_idx = 1:num_s0
        current_sigma_s0 = sigma_s0_list(s_idx);
        fprintf('\n>>> 时刻 %d: sigma_s0 = %.2f m\n', t_idx, current_sigma_s0);

        % 预生成两种权重矩阵
        % 1) RMPR / Overlap / Single-path 使用
        [W_rmpr, T_max_rmpr, ~] = build_weighted_cost_matrix( ...
            dist_matrix, current_sigma_s0, alpha_rmpr_method);

        % 2) Node-weight RMPR（节点+链路，a=0.7）使用
        [W_node, T_max_node, ~, ~] = build_weighted_cost_matrix_node( ...
            dist_matrix, current_sigma_s0, node_ages_special, ...
            alpha_node_method, omega_node_method);

        %% ======================================================
        % 关键修改 1：先固定一套 Monte Carlo 失效样本，所有 n/方法复用
        %% ======================================================
        adj_exec_bank = cell(num_iterations, 1);
        sampled_failed_hotspots_bank = cell(num_iterations, 1);
        hotspot_failed_counter_total_bank = 0;
        sampled_failed_hotspots_union_bank = [];

        for mc = 1:num_iterations
            new_failed_nodes_now = fail_events_by_time{t_idx};

            [adj_exec_bank{mc}, sampled_failed_hotspots_now] = simulate_link_failure_oisl( ...
                dist_matrix, current_sigma_s0, min_dist_limit, new_failed_nodes_now, false, ...
                hotspot_nodes_0based, enable_hotspot_random_failure, hotspot_fail_prob);

            sampled_failed_hotspots_bank{mc} = sampled_failed_hotspots_now;
            hotspot_failed_counter_total_bank = hotspot_failed_counter_total_bank + numel(sampled_failed_hotspots_now);
            sampled_failed_hotspots_union_bank = unique([sampled_failed_hotspots_union_bank, sampled_failed_hotspots_now]); %#ok<AGROW>
        end

        hotspots_sampled_failed_diag_by_time{t_idx} = sampled_failed_hotspots_union_bank;
        avg_hotspot_failed_this_slot = hotspot_failed_counter_total_bank / num_iterations;

        %% ======================================================
        % 关键修改 2：Single-path 在当前时刻、当前 sigma 下只规划一次
        % 不随 n = 2,3,4 变化
        %% ======================================================
        single_path_fixed = [];
        single_w_fixed = [];
        single_access_fixed = [];
        single_valid_fixed = false;

        try
            curr_W_sp = W_rmpr;
            [success_sp_plan, ~, selected_path_sp, ~, ~, ~] = ...
                run_single_path(curr_W_sp, curr_W_sp, Sources, target_sat);

            if success_sp_plan == 1 && ~isempty(selected_path_sp)
                single_path_fixed = {selected_path_sp};
                single_w_fixed = 1;
                single_access_fixed = [];
                single_valid_fixed = true;
            end
        catch
            single_path_fixed = [];
            single_w_fixed = [];
            single_access_fixed = [];
            single_valid_fixed = false;
        end

        %% --- 4.6 遍历 n = 2,3,4 ---
        for n_idx = 1:num_n
            n_redundancy = n_list(n_idx);

            fprintf('\n------------------------------\n');
            fprintf('当前参数: L = %d, n = %d\n', L_fixed, n_redundancy);
            fprintf('------------------------------\n');

            curr_last_paths = cell(1, num_methods);
            curr_last_w_vec = cell(1, num_methods);
            curr_access_sats = cell(1, num_methods);
            valid_method = false(1, num_methods);

            %% ===== 方法1：Single-path（固定，不随 n 变化）=====
            if single_valid_fixed
                curr_last_paths{1} = single_path_fixed;
                curr_last_w_vec{1} = single_w_fixed;
                curr_access_sats{1} = single_access_fixed;
                valid_method(1) = true;
            else
                curr_last_paths{1} = [];
                curr_last_w_vec{1} = [];
                curr_access_sats{1} = [];
            end

            %% ===== 方法2：Node-weight RMPR =====
            try
                in_node = rmpr_build_waterfill_inputs_multi_source( ...
                    t_idx, dist_matrix, W_node, result3, Sources, ...
                    target_sat, L_fixed, M_bits, alpha_node_method, T_max_node, false);

                [w_vec_node, ~] = solve_routing_optimization( ...
                    in_node.q_sec, in_node.gamma_sec, n_redundancy);

                curr_last_paths{2} = in_node.paths;
                curr_last_w_vec{2} = w_vec_node;
                curr_access_sats{2} = in_node.access_sats;
                valid_method(2) = true;

                % 记录热点命中数
                weighted_hot_hits = 0;
                for k = 1:numel(w_vec_node)
                    p = in_node.paths{k};
                    p_mid = p(2:end-1);
                    hot_hits = sum(ismember(p_mid, hotspot_nodes_0based));
                    weighted_hot_hits = weighted_hot_hits + w_vec_node(k) * hot_hits;
                end
                weighted_hot_hits_record(s_idx, n_idx, t_idx) = weighted_hot_hits;

            catch
                curr_last_paths{2} = [];
                curr_last_w_vec{2} = [];
                curr_access_sats{2} = [];
            end

            %% ===== 方法3：RMPR =====
            try
                in_rmpr = rmpr_build_waterfill_inputs_multi_source( ...
                    t_idx, dist_matrix, W_rmpr, result3, Sources, ...
                    target_sat, L_fixed, M_bits, alpha_rmpr_method, T_max_rmpr, false);

                [w_vec_rmpr, ~] = solve_routing_optimization( ...
                    in_rmpr.q_sec, in_rmpr.gamma_sec, n_redundancy);

                curr_last_paths{3} = in_rmpr.paths;
                curr_last_w_vec{3} = w_vec_rmpr;
                curr_access_sats{3} = in_rmpr.access_sats;
                valid_method(3) = true;
            catch
                curr_last_paths{3} = [];
                curr_last_w_vec{3} = [];
                curr_access_sats{3} = [];
            end

            %% ===== 方法4：Overlap multi-path =====
            try
                in_overlap = rmpr_build_waterfill_inputs_multi_source_overlap( ...
                    t_idx, dist_matrix, W_rmpr, result3, Sources, ...
                    target_sat, L_fixed, M_bits, alpha_rmpr_method, T_max_rmpr, false);

                [w_vec_overlap, ~] = solve_routing_optimization( ...
                    in_overlap.q_sec, in_overlap.gamma_sec, n_redundancy);

                curr_last_paths{4} = in_overlap.paths;
                curr_last_w_vec{4} = w_vec_overlap;
                curr_access_sats{4} = in_overlap.access_sats;
                valid_method(4) = true;
            catch
                curr_last_paths{4} = [];
                curr_last_w_vec{4} = [];
                curr_access_sats{4} = [];
            end

            %% ===== 记录规划成功 =====
            for m_idx = 1:num_methods
                plan_success_matrix(s_idx, n_idx, m_idx, t_idx) = valid_method(m_idx);
            end

            %% ===== 打印摘要 =====
            fprintf('\n----- 时刻 %d, n=%d 的 4 个算法规划摘要 -----\n', t_idx, n_redundancy);
            for m_idx = 1:num_methods
                fprintf('算法 %s:\n', raw_method_names{m_idx});

                if ~valid_method(m_idx)
                    fprintf('  规划失败。\n');
                    continue;
                end

                w_now = curr_last_w_vec{m_idx};
                fprintf('  规划成功 | sum(w)=%.6f | active_paths=%d | max(w)=%.6f\n', ...
                    sum(w_now), sum(w_now > 1e-10), max(w_now));

                if enable_detail_print
                    for k = 1:numel(w_now)
                        p_now = curr_last_paths{m_idx}{k};
                        fprintf('  路径 %d: %s\n', k, mat2str(p_now));
                        fprintf('          权重 = %.6f\n', w_now(k));
                        fprintf('          分配数据 = %.2f bits\n', w_now(k) * M_bits);
                        fprintf('          跳数 = %d\n', numel(p_now)-1);
                    end
                end
            end

            %% ===== 计算四种算法的固定方案时延、固定方案平均跳数 =====
            for m_idx = 1:num_methods
                if ~valid_method(m_idx)
                    continue;
                end

                curr_paths_m = curr_last_paths{m_idx};
                curr_w_vec_m = curr_last_w_vec{m_idx};
                curr_access_sats_m = curr_access_sats{m_idx};

                % ---------- 平均跳数：按权重加权 ----------
                weighted_hops = 0;
                valid_weight_sum_for_hops = 0;

                for k = 1:numel(curr_w_vec_m)
                    if curr_w_vec_m(k) <= 1e-10
                        continue;
                    end
                    hop_k = numel(curr_paths_m{k}) - 1;
                    weighted_hops = weighted_hops + curr_w_vec_m(k) * hop_k;
                    valid_weight_sum_for_hops = valid_weight_sum_for_hops + curr_w_vec_m(k);
                end

                if valid_weight_sum_for_hops > 1e-10
                    hop_matrix(s_idx, n_idx, m_idx, t_idx) = weighted_hops / valid_weight_sum_for_hops;
                end

                % ---------- 平均时延：方案完成时间 = 活动路径最慢完成时间 ----------
                path_delays = nan(1, numel(curr_w_vec_m));

                for k = 1:numel(curr_w_vec_m)
                    if curr_w_vec_m(k) <= 1e-10
                        continue;
                    end

                    path = curr_paths_m{k};

                    % 1) 传播时延
                    d_path = 0;
                    for e = 1:length(path)-1
                        d_path = d_path + dist_matrix(path(e)+1, path(e+1)+1);
                    end
                    prop_delay = d_path / c_light;

                    % 2) 传输时延
                    rate_bps = NaN;

                    if ~isempty(curr_access_sats_m) && numel(curr_access_sats_m) >= k
                        access_sat_now = curr_access_sats_m(k);
                        idx_rate = find(result3(t_idx).sat_ids == access_sat_now, 1);
                        if ~isempty(idx_rate)
                            rate_bps = result3(t_idx).rates_Mbps(idx_rate) * 1e6;
                        end
                    end

                    if ~isfinite(rate_bps)
                        source_sat_now = path(1);
                        idx_rate = find(result3(t_idx).sat_ids == source_sat_now, 1);
                        if ~isempty(idx_rate)
                            rate_bps = result3(t_idx).rates_Mbps(idx_rate) * 1e6;
                        end
                    end

                    if isfinite(rate_bps) && rate_bps > 0
                        tx_delay = (curr_w_vec_m(k) * M_bits) / rate_bps;
                        path_delays(k) = prop_delay + tx_delay;
                    end
                end

                valid_idx = isfinite(path_delays);
                if any(valid_idx)
                    delay_matrix(s_idx, n_idx, m_idx, t_idx) = max(path_delays(valid_idx));
                end
            end

            %% ===== Monte Carlo：执行阶段抽奖，统计 PDR =====
            success_counts = zeros(1, num_methods);

            for mc = 1:num_iterations
                adj_exec = adj_exec_bank{mc};

                for m_idx = 1:num_methods
                    if ~valid_method(m_idx)
                        continue;
                    end

                    received_sum = 0;
                    w_vec = curr_last_w_vec{m_idx};
                    path_set = curr_last_paths{m_idx};

                    for k = 1:numel(w_vec)
                        if is_path_alive(path_set{k}, adj_exec)
                            received_sum = received_sum + w_vec(k);
                        end
                    end

                    if received_sum >= 1.0 - 1e-12
                        success_counts(m_idx) = success_counts(m_idx) + 1;
                    end
                end
            end

            %% ===== 存储当前结果 =====
            avg_hotspot_failed_record(s_idx, n_idx, t_idx) = avg_hotspot_failed_this_slot;

            for m_idx = 1:num_methods
                if valid_method(m_idx)
                    pdr_matrix(s_idx, n_idx, m_idx, t_idx) = success_counts(m_idx) / num_iterations;
                else
                    pdr_matrix(s_idx, n_idx, m_idx, t_idx) = NaN;
                end

                last_paths{s_idx, n_idx, m_idx, t_idx} = curr_last_paths{m_idx};
                last_w_vec{s_idx, n_idx, m_idx, t_idx} = curr_last_w_vec{m_idx};
            end

            fprintf('\n完成 | 时刻=%d | n=%d\n', t_idx, n_redundancy);
            for m_idx = 1:num_methods
                if valid_method(m_idx)
                    fprintf('  %-24s | Plan=1 | PDR = %.4f | Delay = %.4f ms | Hops = %.4f\n', ...
                        raw_method_names{m_idx}, ...
                        pdr_matrix(s_idx, n_idx, m_idx, t_idx), ...
                        delay_matrix(s_idx, n_idx, m_idx, t_idx) * 1000, ...
                        hop_matrix(s_idx, n_idx, m_idx, t_idx));
                else
                    fprintf('  %-24s | Plan=0 | PDR = NaN | Delay = NaN | Hops = NaN\n', ...
                        raw_method_names{m_idx});
                end
            end
            fprintf('  平均每次抽中失效热点数 = %.4f\n', avg_hotspot_failed_record(s_idx, n_idx, t_idx));
        end

        fprintf('本时刻 Monte Carlo 中曾被抽中失效的热点节点并集（仅诊断，不写回状态）= %s\n', ...
            mat2str(hotspots_sampled_failed_diag_by_time{t_idx}));
    end

    %% ======================================================
    % 真实系统状态只由 fail_events_by_time 更新
    % 不再把 Monte Carlo 假想失败样本并回 persistent
    %% ======================================================
    failed_nodes_persistent = unique([ ...
        failed_nodes_persistent, ...
        fail_events_by_time{t_idx} ...
        ]);

    fprintf('时刻 %d 结束后，累计失效节点更新为 = %s\n', ...
        t_idx, mat2str(failed_nodes_persistent));
end

%% ===== 5. 对时间求平均，得到最终画图数据 =====
% 条件平均：仅在成功规划的时隙上平均
mean_pdr = squeeze(mean(pdr_matrix(1,:,:,:), 4, 'omitnan'));         % [n, method]
mean_delay_ms = squeeze(mean(delay_matrix(1,:,:,:), 4, 'omitnan')) * 1000;
mean_hops = squeeze(mean(hop_matrix(1,:,:,:), 4, 'omitnan'));

% 规划成功率
mean_plan_success = squeeze(mean(plan_success_matrix(1,:,:,:), 4));   % [n, method]

% 重排画图顺序
mean_pdr = mean_pdr(:, plot_order);
mean_delay_ms = mean_delay_ms(:, plot_order);
mean_hops = mean_hops(:, plot_order);
mean_plan_success = mean_plan_success(:, plot_order);

%% ===== 6. 绘图：组合成 1x3 大图，顶部共享 legend =====
figure('Color','w','Position',[120,120,1200,360]);
% ---------------- 子图 1：Average Delay ----------------
ax1 = subplot(1,3,1);
b1 = bar(ax1, mean_delay_ms, 'grouped');
grid(ax1, 'on');
box(ax1, 'on');
set(ax1, 'FontSize', 11, 'LineWidth', 1, 'TickDir', 'out');
set(ax1, 'XTick', 1:num_n, 'XTickLabel', compose('n=%d', n_list));
ylabel(ax1, 'Average Delay (ms)');
ylim(ax1, [0, 80]);
yticks(ax1, 0:20:80);                 % 4 格

% ---------------- 子图 2：Average PDR ----------------
ax2 = subplot(1,3,2);
b2 = bar(ax2, mean_pdr, 'grouped');
grid(ax2, 'on');
box(ax2, 'on');
set(ax2, 'FontSize', 11, 'LineWidth', 1, 'TickDir', 'out');
set(ax2, 'XTick', 1:num_n, 'XTickLabel', compose('n=%d', n_list));
ylabel(ax2, 'Average PDR');
ylim(ax2, [0, 1]);
yticks(ax2, 0:0.25:1);                % 4 格

% ---------------- 子图 3：Average Hop Count ----------------
ax3 = subplot(1,3,3);
b3 = bar(ax3, mean_hops, 'grouped');
grid(ax3, 'on');
box(ax3, 'on');
set(ax3, 'FontSize', 11, 'LineWidth', 1, 'TickDir', 'out');
set(ax3, 'XTick', 1:num_n, 'XTickLabel', compose('n=%d', n_list));
ylabel(ax3, 'Average Hop Count');
ylim(ax3, [0, 20]);
yticks(ax3, 0:5:20);                  % 4 格

% ---------------- 统一调整三个子图位置 ----------------
ax_all = [ax1, ax2, ax3];
for i = 1:3
    pos = ax_all(i).Position;
    % 往下压一点，给顶部 legend 留空间；同时底部留一点位置给 (a)(b)(c)
    ax_all(i).Position = [pos(1), pos(2)+0.03, pos(3), pos(4)-0.08];
end

% ---------------- 顶部共享 legend（两列） ----------------
lgd = legend(ax2, b2, plot_method_names, ...
    'NumColumns', 4, ...
    'Orientation', 'horizontal', ...
    'Location', 'northoutside', ...
    'Box', 'on', ...
    'FontSize', 10);

set(lgd, 'Units', 'normalized');
drawnow;                     % 先让 MATLAB 计算出 legend 实际大小
pos = lgd.Position;          % [left, bottom, width, height]
pos(1) = (1 - pos(3)) / 2;   % 按当前宽度自动水平居中
pos(2) = 0.89;               % 纵向位置你自己定
lgd.Position = pos;

% lgd.Position = [0.12, 0.89, 0.78, 0.09];




% ---------------- 子图下标，与图 2 那种风格对齐 ----------------
text(ax1, 0.5, -0.28, '(a) Average Delay', ...
    'Units', 'normalized', 'HorizontalAlignment', 'center', 'FontSize', 11);

text(ax2, 0.5, -0.28, '(b) Average PDR', ...
    'Units', 'normalized', 'HorizontalAlignment', 'center', 'FontSize', 11);

text(ax3, 0.5, -0.28, '(c) Average Hop Count', ...
    'Units', 'normalized', 'HorizontalAlignment', 'center', 'FontSize', 11);

% ---------------- 导出（可选） ----------------
% print(gcf, 'fig_redundancy_levels', '-depsc', '-r600');


%% ===== 7. 打印最终平均结果 =====
fprintf('\n\n================ 条件平均结果（仅成功规划时隙） ================\n');
for n_idx = 1:num_n
    fprintf('\n>>> n = %d\n', n_list(n_idx));
    for p_idx = 1:num_methods
        m_idx = plot_order(p_idx);
        fprintf('  %-24s | PlanRate = %.4f | Avg Delay = %.4f ms | Avg PDR = %.4f | Avg Hops = %.4f\n', ...
            raw_method_names{m_idx}, ...
            mean_plan_success(n_idx, p_idx), ...
            mean_delay_ms(n_idx, p_idx), ...
            mean_pdr(n_idx, p_idx), ...
            mean_hops(n_idx, p_idx));
    end
end

%% =========================================================
%% 8. 辅助函数
%% =========================================================

function [A, sampled_failed_hotspots_0based] = simulate_link_failure_oisl( ...
    dist_mat, sigma_s0, min_dist_threshold, failed_nodes, verbose, ...
    hotspot_nodes_0based, enable_hotspot_random_failure, hotspot_fail_prob)

    A = dist_mat;
    A(A < min_dist_threshold) = inf;

    sampled_failed_hotspots_0based = [];

    % 1) 链路随机失效
    [ii, jj] = find(triu(isfinite(A) & A > 0, 1));
    for k = 1:numel(ii)
        p_f = oisl_failure_prob(A(ii(k), jj(k)), sigma_s0);
        if rand() < p_f
            A(ii(k), jj(k)) = inf;
            A(jj(k), ii(k)) = inf;
        end
    end

    % 2) 当前时刻热点节点按概率随机失效
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

    % 3) 本时刻手动指定的新失效节点立即生效
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
