%% =========================================================
%  Node-weight RMPR:
%  固定 n = 2，比较 L = 2, 3, 4
%  横轴：task size = 100, 200, 300, 400, 500 Mb
%
%  指标：
%   1) Average Delay
%   2) Average PDR
%% =========================================================
clc;

%% ===== 1. 参数设置 =====
sigma_s0_list = [2.15];
num_s0 = numel(sigma_s0_list);

num_iterations = 1;      % 每个时刻、每个 sigma、每个 task、每个 L 的蒙特卡洛次数

target_sat = 402;          % 目标卫星
L_list = [2, 3, 4];        % 比较的接入卫星数
num_L = numel(L_list);

n_fixed = 2;               % 固定冗余度
task_size_list_Mb = [1 2 3 4];
num_tasks = numel(task_size_list_Mb);

min_dist_limit = 600e3;    % 硬截断阈值
ground_station_node = 0;   % 地面站节点（0-based）
time_diedai = 1;           % 时间片数
c_light = 299792458;       % 光速

% ===== Node-weight 参数 =====
alpha_node_method = 0.7;
omega_node_method = 0.5;
background_age = 1.0;
hotspot_age = 13.5;

% ===== 热点随机失效 =====
enable_hotspot_random_failure = true;
hotspot_fail_prob = 0.3;

rng(1);

%% ===== 1.1 热点节点设置（0-based）=====
hotspots_by_time = cell(time_diedai, 1);

% ===== 在这里按你的实验手动填写 =====
hotspots_by_time{1} = [569 529 489 647];
hotspots_by_time{2} = [648 489 449];

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
% 维度: [sigma, task_idx, L_idx, time]
pdr_matrix   = nan(num_s0, num_tasks, num_L, time_diedai);
delay_matrix = nan(num_s0, num_tasks, num_L, time_diedai);   % 单位：秒

last_paths = cell(num_s0, num_tasks, num_L, time_diedai);
last_w_vec = cell(num_s0, num_tasks, num_L, time_diedai);
last_access_sats = cell(num_s0, num_tasks, num_L, time_diedai);

weighted_hot_hits_record = nan(num_s0, num_tasks, num_L, time_diedai);
avg_hotspot_failed_record = nan(num_s0, num_tasks, num_L, time_diedai);

hotspots_sampled_failed_by_time = cell(time_diedai, 1);

% 历史累计失效节点：从下一时刻开始进入规划图
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

    %% --- 4.2 当前时刻热点节点 ---
    hotspot_nodes_0based = hotspots_by_time{t_idx};
    if isempty(hotspot_nodes_0based)
        hotspot_nodes_0based = [];
    end
    fprintf('时刻 %d 预设热点节点 = %s\n', t_idx, mat2str(hotspot_nodes_0based));

    %% --- 4.3 构造节点年龄图 ---
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

    %% --- 4.4 遍历 sigma ---
    for s_idx = 1:num_s0
        current_sigma_s0 = sigma_s0_list(s_idx);
        fprintf('\n>>> 时刻 %d: sigma_s0 = %.2f m\n', t_idx, current_sigma_s0);

        % 当前时刻 Node-weight 权重矩阵
        [W_node_base, T_max_node_base, ~, ~] = build_weighted_cost_matrix_node( ...
            dist_matrix, current_sigma_s0, node_ages_special, ...
            alpha_node_method, omega_node_method);

        sampled_failed_hotspots_union_alltasks = [];

        %% --- 4.5 遍历 task size ---
        for task_idx = 1:num_tasks
            M_bits = task_size_list_Mb(task_idx) * 1e6;

            fprintf('\n==============================\n');
            fprintf('当前 task size = %d Mb\n', task_size_list_Mb(task_idx));
            fprintf('==============================\n');

            sampled_failed_hotspots_union_onetask = [];

            %% --- 4.6 遍历 L = 2,3,4 ---
            for L_idx = 1:num_L
                L_now = L_list(L_idx);

                fprintf('\n----- 当前参数: task=%d Mb, L=%d, n=%d -----\n', ...
                    task_size_list_Mb(task_idx), L_now, n_fixed);

                % 当前接入卫星
                drone_sat_ids = result3(t_idx).sat_ids(:);
                drone_sat_ids = drone_sat_ids(isfinite(drone_sat_ids));
                Sources = drone_sat_ids(1:min(end, L_now));

                if isempty(Sources)
                    fprintf('当前组合无可用接入卫星，跳过。\n');
                    continue;
                end

                %% ===== 固定规划一次 =====
                curr_paths = [];
                curr_w_vec = [];
                curr_access_sats = [];
                valid_method = false;

                try
                    in_node = rmpr_build_waterfill_inputs_multi_source( ...
                        t_idx, dist_matrix, W_node_base, result3, Sources, ...
                        target_sat, L_now, M_bits, alpha_node_method, T_max_node_base, false);

                    [w_vec_node, ~] = solve_routing_optimization( ...
                        in_node.q_sec, in_node.gamma_sec, n_fixed);

                    curr_paths = in_node.paths;
                    curr_w_vec = w_vec_node;
                    curr_access_sats = in_node.access_sats;
                    valid_method = true;
                catch
                    fprintf('Node-weight RMPR 在当前参数下规划失败。\n');
                    continue;
                end

                %% ===== 固定方案时延 =====
                path_delays = nan(1, numel(curr_w_vec));

                for k = 1:numel(curr_w_vec)
                    if curr_w_vec(k) <= 1e-10
                        continue;
                    end

                    path = curr_paths{k};

                    % 1) 传播时延
                    d_path = 0;
                    for e = 1:length(path)-1
                        d_path = d_path + dist_matrix(path(e)+1, path(e+1)+1);
                    end
                    prop_delay = d_path / c_light;

                    % 2) 传输时延
                    rate_bps = NaN;

                    if ~isempty(curr_access_sats) && numel(curr_access_sats) >= k
                        access_sat_now = curr_access_sats(k);
                        idx_rate = find(result3(t_idx).sat_ids == access_sat_now, 1);
                        if ~isempty(idx_rate)
                            rate_bps = result3(t_idx).rates_Mbps(idx_rate) * 1e6;
                        end
                    end

                    % fallback：用路径首节点去找接入速率
                    if ~isfinite(rate_bps)
                        source_sat_now = path(1);
                        idx_rate = find(result3(t_idx).sat_ids == source_sat_now, 1);
                        if ~isempty(idx_rate)
                            rate_bps = result3(t_idx).rates_Mbps(idx_rate) * 1e6;
                        end
                    end

                    if isfinite(rate_bps) && rate_bps > 0
                        tx_delay = (curr_w_vec(k) * M_bits) / rate_bps;
                        path_delays(k) = prop_delay + tx_delay;
                    end
                end

                valid_idx = isfinite(path_delays);
                if any(valid_idx)
                    delay_matrix(s_idx, task_idx, L_idx, t_idx) = max(path_delays(valid_idx));
                end

                %% ===== Monte Carlo：执行阶段抽奖，统计 PDR =====
                success_count = 0;
                hotspot_failed_counter_total = 0;
                sampled_failed_hotspots_union = [];

                for mc = 1:num_iterations
                    new_failed_nodes_now = fail_events_by_time{t_idx};

                    [adj_exec, sampled_failed_hotspots_now] = simulate_link_failure_oisl( ...
                        dist_matrix, current_sigma_s0, min_dist_limit, new_failed_nodes_now, false, ...
                        hotspot_nodes_0based, enable_hotspot_random_failure, hotspot_fail_prob);

                    hotspot_failed_counter_total = hotspot_failed_counter_total + numel(sampled_failed_hotspots_now);
                    sampled_failed_hotspots_union = unique([sampled_failed_hotspots_union, sampled_failed_hotspots_now]);

                    if valid_method
                        received_sum = 0;
                        for k = 1:numel(curr_w_vec)
                            if is_path_alive(curr_paths{k}, adj_exec)
                                received_sum = received_sum + curr_w_vec(k);
                            end
                        end

                        if received_sum >= 1 - 1e-12
                            success_count = success_count + 1;
                        end
                    end

                    if mod(mc, 100) == 0
                        fprintf('.');
                    end
                end

                %% ===== 存储结果 =====
                pdr_matrix(s_idx, task_idx, L_idx, t_idx) = success_count / num_iterations;
                avg_hotspot_failed_record(s_idx, task_idx, L_idx, t_idx) = ...
                    hotspot_failed_counter_total / num_iterations;

                last_paths{s_idx, task_idx, L_idx, t_idx} = curr_paths;
                last_w_vec{s_idx, task_idx, L_idx, t_idx} = curr_w_vec;
                last_access_sats{s_idx, task_idx, L_idx, t_idx} = curr_access_sats;

                sampled_failed_hotspots_union_onetask = unique([ ...
                    sampled_failed_hotspots_union_onetask, ...
                    sampled_failed_hotspots_union]);

                fprintf('\n完成 | task=%d Mb | L=%d | n=%d | PDR = %.4f | Delay = %.4f ms\n', ...
                    task_size_list_Mb(task_idx), L_now, n_fixed, ...
                    pdr_matrix(s_idx, task_idx, L_idx, t_idx), ...
                    delay_matrix(s_idx, task_idx, L_idx, t_idx) * 1000);
            end

            sampled_failed_hotspots_union_alltasks = unique([ ...
                sampled_failed_hotspots_union_alltasks, ...
                sampled_failed_hotspots_union_onetask]);
        end

        hotspots_sampled_failed_by_time{t_idx} = sampled_failed_hotspots_union_alltasks;
        fprintf('本时刻所有 task/L 中曾被抽中失效的热点节点并集 = %s\n', ...
            mat2str(hotspots_sampled_failed_by_time{t_idx}));
    end

    %% --- 当前时刻结束后：累计失效节点更新 ---
    failed_nodes_persistent = unique([ ...
        failed_nodes_persistent, ...
        fail_events_by_time{t_idx}, ...
        hotspots_sampled_failed_by_time{t_idx} ...
        ]);

    fprintf('时刻 %d 结束后，累计失效节点更新为 = %s\n', ...
        t_idx, mat2str(failed_nodes_persistent));
end

%% ===== 5. 对时间求平均，得到最终画图数据 =====
mean_pdr = squeeze(mean(pdr_matrix(1,:,:,:), 4, 'omitnan'));         % [task, L]
mean_delay_ms = squeeze(mean(delay_matrix(1,:,:,:), 4, 'omitnan')) * 1000;

%% ===== 6. 绘图：两个子图 =====
figure('Color','w','Position',[120,100,1100,420]);

% -------- (a) Average Delay --------
subplot(1,2,1);
bar(mean_delay_ms, 'grouped');
grid on; box on;
set(gca, 'FontSize', 11, 'LineWidth', 1, 'TickDir', 'out');
set(gca, 'XTick', 1:num_tasks, 'XTickLabel', compose('%d', task_size_list_Mb));
xlabel('Task size (Mb)');
ylabel('Average Delay (ms)');
title('(a) Average Delay');
legend(compose('L=%d', L_list), 'Location', 'northwest');

% -------- (b) Average PDR --------
subplot(1,2,2);
bar(mean_pdr, 'grouped');
grid on; box on;
set(gca, 'FontSize', 11, 'LineWidth', 1, 'TickDir', 'out');
set(gca, 'XTick', 1:num_tasks, 'XTickLabel', compose('%d', task_size_list_Mb));
xlabel('Task size (Mb)');
ylabel('Average PDR');
title('(b) Average PDR');
legend(compose('L=%d', L_list), 'Location', 'southwest');
ylim([0 1.05]);

sgtitle(sprintf('Node-weight RMPR under Different Task Sizes (n = %d)', n_fixed), ...
    'FontSize', 14);

%% ===== 7. 打印最终平均结果 =====
fprintf('\n\n================ 最终平均结果（对时刻求平均） ================\n');
for task_idx = 1:num_tasks
    fprintf('\n>>> Task size = %d Mb\n', task_size_list_Mb(task_idx));
    for L_idx = 1:num_L
        fprintf('  L = %d | Avg Delay = %.4f ms | Avg PDR = %.4f\n', ...
            L_list(L_idx), ...
            mean_delay_ms(task_idx, L_idx), ...
            mean_pdr(task_idx, L_idx));
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