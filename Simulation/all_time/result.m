
%%%这个是第一阶段的全周期代码！！

%% ===== 0. 安全初始化与数据保护 =====
clc;

%% ===== 1. 参数设置 =====
% 只比较两个算法：a=0（原算法）和 a=0.8（改进算法）
alpha_list = [0, 0.7];
num_alphas = numel(alpha_list);

% 扫描不同初始抖动强度 sigma_s0
sigma_s0_list = [2.2];
num_s0 = numel(sigma_s0_list);

% Monte Carlo 参数
num_iterations = 10;     % 每个时刻、每个sigma下的蒙特卡洛次数

% 系统参数
target_sat = 402;         % 目标卫星
L = 4;                    % 协同接入卫星数
M_bits = 1e6;             % 总传输数据量
n_redundancy = 2;         % 冗余度
min_dist_limit = 600e3;   % 硬截断阈值
ground_station_node = 0;  % 地面站节点（0开始）

% 固定随机种子，保证可复现
rng(1);

%% ===== 2. 加载数据 =====
if ~exist('result3', 'var')
    load('无人机-卫星速率数据_result3.mat', 'result3');
end

if ~exist('result_static', 'var')
    error('工作区中没有 result_static，请先加载 result_static。');
end

time_diedai = 2;

% 存储结果：维度 = [sigma_s0, method, t_idx]
% method:
% 1 -> a=0, RMPR disjoint
% 2 -> a=0.8, RMPR disjoint
% 3 -> Single-path
% 4 -> Overlap multi-path
pdr_matrix = zeros(num_s0, 4, time_diedai);

% 保存最后一次成功求解结果
last_paths = cell(num_s0, 4, time_diedai);
last_w_vec = cell(num_s0, 4, time_diedai);

%% ===== 3. 主循环：遍历所有时刻 =====
for t_idx = 1:time_diedai
    fprintf('\n==================================================\n');
    fprintf('开始处理时刻 t_idx = %d / %d\n', t_idx, time_diedai);
    fprintf('==================================================\n');

    %% --- 3.1 提取当前时刻的静态图 dist_matrix ---
    result_static_temp = result_static(t_idx);

    % 兼容 create_diagram_dynamic
    result_static_for_build = repmat(result_static_temp, t_idx, 1);

    [adj_matrices_static_temp, has_valid_links] = create_diagram_dynamic( ...
        t_idx, result3, result_static_for_build, ground_station_node);

    if isempty(adj_matrices_static_temp) || ~has_valid_links
        fprintf('时刻 %d: 未生成有效邻接矩阵，跳过当前时刻。\n', t_idx);
        continue;
    end

    dist_matrix = adj_matrices_static_temp;

    %% --- 3.2 对每个 sigma_s0 分别仿真 ---
    for s_idx = 1:num_s0
        current_sigma_s0 = sigma_s0_list(s_idx);
        fprintf('\n>>> 时刻 %d: 正在测试抖动强度 sigma_s0 = %.2f m ', t_idx, current_sigma_s0);

        % 预生成当前 sigma_s0 下两个 alpha 的代价矩阵
        W_mats = cell(num_alphas, 1);
        T_maxs = zeros(num_alphas, 1);
        R_refs = zeros(num_alphas, 1);

        for a_idx = 1:num_alphas
            [W_mats{a_idx}, T_maxs(a_idx), R_refs(a_idx)] = ...
                build_weighted_cost_matrix(dist_matrix, current_sigma_s0, alpha_list(a_idx));
        end

        % 成功次数统计：
        % [a=0 RMPR, a=0.8 RMPR, single-path, overlap]
        success_counts = zeros(1, 4);

        % 临时记录“最后一次成功求解”的结果
        curr_last_paths = cell(1, 4);
        curr_last_w_vec = cell(1, 4);

        %% --- 3.3 Monte Carlo 仿真 ---
        for i = 1:num_iterations
            % 获取当前时刻接入点
            drone_sat_ids = result3(t_idx).sat_ids(:);
            drone_sat_ids = drone_sat_ids(isfinite(drone_sat_ids));
            Sources = drone_sat_ids(1:min(end, L));

            if isempty(Sources)
                continue;
            end

            % 生成当前迭代下真实的物理失效环境
            % 同一次 Monte Carlo 中，所有方法共用同一张 adj_exec
            adj_exec = simulate_link_failure_oisl(dist_matrix, current_sigma_s0, min_dist_limit, false);

            %% ===== 方法1：a = 0, RMPR 不相交 =====
            try
                curr_alpha = 0;
                curr_W = W_mats{1};
                curr_Tmax = T_maxs(1);
                curr_Rref = R_refs(1); %#ok<NASGU>

                in = rmpr_build_waterfill_inputs_multi_source( ...
                    t_idx, dist_matrix, curr_W, result3, Sources, ...
                    target_sat, L, M_bits, curr_alpha, curr_Tmax, false);

                [w_vec, ~] = solve_routing_optimization(in.q_sec, in.gamma_sec, n_redundancy);

                received_sum = 0;
                for k = 1:numel(w_vec)
                    if is_path_alive(in.paths{k}, adj_exec)
                        received_sum = received_sum + w_vec(k);
                    end
                end

                curr_last_paths{1} = in.paths;
                curr_last_w_vec{1} = w_vec;

                if received_sum >= 1.0 - 1e-12
                    success_counts(1) = success_counts(1) + 1;
                end
            catch
            end

            %% ===== 方法2：a = 0.8, RMPR 不相交 =====
            try
                curr_alpha = 0.7;
                curr_W = W_mats{2};
                curr_Tmax = T_maxs(2);
                curr_Rref = R_refs(2); %#ok<NASGU>

                in = rmpr_build_waterfill_inputs_multi_source( ...
                    t_idx, dist_matrix, curr_W, result3, Sources, ...
                    target_sat, L, M_bits, curr_alpha, curr_Tmax, false);

                [w_vec, ~] = solve_routing_optimization(in.q_sec, in.gamma_sec, n_redundancy);

                received_sum = 0;
                for k = 1:numel(w_vec)
                    if is_path_alive(in.paths{k}, adj_exec)
                        received_sum = received_sum + w_vec(k);
                    end
                end

                curr_last_paths{2} = in.paths;
                curr_last_w_vec{2} = w_vec;

                if received_sum >= 1.0 - 1e-12
                    success_counts(2) = success_counts(2) + 1;
                end
            catch
            end

            %% ===== 方法3：Single-path =====
            try
                [success_sp, selected_source_sp, selected_path_sp, selected_distance_sp, ...
                    all_candidate_paths_sp, all_candidate_distances_sp] = ...
                    run_single_path(dist_matrix, adj_exec, Sources, target_sat); %#ok<ASGLU>

                curr_last_paths{3} = {selected_path_sp};
                curr_last_w_vec{3} = 1;   % 单一路径默认全量1

                if success_sp == 1
                    success_counts(3) = success_counts(3) + 1;
                end
            catch
            end

            %% ===== 方法4：Overlap multi-path（按 a=0 的权重矩阵）=====
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

                received_sum_overlap = 0;
                for k = 1:numel(w_vec_overlap)
                    if is_path_alive(in_overlap.paths{k}, adj_exec)
                        received_sum_overlap = received_sum_overlap + w_vec_overlap(k);
                    end
                end

                curr_last_paths{4} = in_overlap.paths;
                curr_last_w_vec{4} = w_vec_overlap;

                if received_sum_overlap >= 1.0 - 1e-12
                    success_counts(4) = success_counts(4) + 1;
                end
            catch
            end

            if mod(i, 100) == 0
                fprintf('.');
            end
        end

        %% --- 3.4 存储该时刻、该 sigma 下 4 种算法的 PDR ---
        pdr_matrix(s_idx, :, t_idx) = success_counts / num_iterations;
        fprintf(' 完成');
        fprintf(' | PDR(a=0)=%.4f, PDR(a=0.8)=%.4f, PDR(single)=%.4f, PDR(overlap)=%.4f\n', ...
            pdr_matrix(s_idx, 1, t_idx), ...
            pdr_matrix(s_idx, 2, t_idx), ...
            pdr_matrix(s_idx, 3, t_idx), ...
            pdr_matrix(s_idx, 4, t_idx));

        %% --- 3.5 保存最后一次成功求解的路径与分配 ---
        for m_idx = 1:4
            last_paths{s_idx, m_idx, t_idx} = curr_last_paths{m_idx};
            last_w_vec{s_idx, m_idx, t_idx} = curr_last_w_vec{m_idx};
        end

        fprintf('\n----- 时刻 %d, sigma_s0=%.2f 的最终路径与分配结果 -----\n', t_idx, current_sigma_s0);

        method_names = {'a=0 RMPR', 'a=0.7 RMPR', 'Single-path', 'Overlap multi-path'};
        for m_idx = 1:4
            fprintf('算法 %s:\n', method_names{m_idx});

            if isempty(curr_last_paths{m_idx}) || isempty(curr_last_w_vec{m_idx})
                fprintf('  没有成功保存到路径/分配结果（可能该算法在该时刻下未成功求解）\n');
                continue;
            end

            if m_idx == 3
                % Single-path
                fprintf('  路径: %s\n', mat2str(curr_last_paths{m_idx}{1}));
                fprintf('  权重 = %.6f\n', curr_last_w_vec{m_idx});
                fprintf('  分配数据 = %.2f bits\n', curr_last_w_vec{m_idx} * M_bits);
            else
                for k = 1:numel(curr_last_w_vec{m_idx})
                    fprintf('  路径 %d: %s\n', k, mat2str(curr_last_paths{m_idx}{k}));
                    fprintf('          权重 = %.6f\n', curr_last_w_vec{m_idx}(k));
                    fprintf('          分配数据 = %.2f bits\n', curr_last_w_vec{m_idx}(k) * M_bits);
                end
            end
        end
    end
end
%% ===== 4. 绘图：所有结果画在一张图上，横轴为 0 ~ (time_diedai-1) =====
figure('Color', 'w', 'Position', [100, 100, 1200, 700]);
hold on;

time_axis = 0:(time_diedai-1);

% 固定四种算法的颜色（只改颜色，其他不变）
color_a0      = [0.0000, 0.4470, 0.7410];   % 蓝色
color_a08     = [0.8500, 0.3250, 0.0980];   % 橙色
color_single  = [0.4660, 0.6740, 0.1880];   % 绿色
color_overlap = [0.4940, 0.1840, 0.5560];   % 紫色

for s_idx = 1:num_s0

    % a = 0（RMPR）
    plot(time_axis, squeeze(pdr_matrix(s_idx, 1, :))', ...
        'LineStyle', '-', ...
        'Marker', 'o', ...
        'Color', color_a0, ...
        'LineWidth', 2, ...
        'MarkerSize', 4, ...
        'DisplayName', sprintf('\\sigma_{s0}=%.1f, a=0 (RMPR)', sigma_s0_list(s_idx)));

    % a = 0.8（RMPR）
    plot(time_axis, squeeze(pdr_matrix(s_idx, 2, :))', ...
        'LineStyle', '--', ...
        'Marker', 's', ...
        'Color', color_a08, ...
        'LineWidth', 2, ...
        'MarkerSize', 4, ...
        'DisplayName', sprintf('\\sigma_{s0}=%.1f, a=0.7 (RMPR)', sigma_s0_list(s_idx)));

    % Single-path
    plot(time_axis, squeeze(pdr_matrix(s_idx, 3, :))', ...
        'LineStyle', '-.', ...
        'Marker', '^', ...
        'Color', color_single, ...
        'LineWidth', 2, ...
        'MarkerSize', 4, ...
        'DisplayName', sprintf('\\sigma_{s0}=%.1f, Single-path', sigma_s0_list(s_idx)));

    % Overlap multi-path
    plot(time_axis, squeeze(pdr_matrix(s_idx, 4, :))', ...
        'LineStyle', ':', ...
        'Marker', 'd', ...
        'Color', color_overlap, ...
        'LineWidth', 2, ...
        'MarkerSize', 4, ...
        'DisplayName', sprintf('\\sigma_{s0}=%.1f, Overlap', sigma_s0_list(s_idx)));
end

grid on;
xlabel('时刻 t', 'FontSize', 12);
ylabel('数据包投递率 (PDR)', 'FontSize', 12);
title('不同抖动强度下，四种算法的 PDR 随时刻变化', 'FontSize', 14);
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

%% ===== 5. 如需查看某个时刻、某个sigma、某个方法的路径结果，可直接查看 =====
% method 索引：
% 1 -> a=0 RMPR
% 2 -> a=0.8 RMPR
% 3 -> Single-path
% 4 -> Overlap multi-path
%
% 示例：
% last_paths{1,1,10} -> sigma_s0_list(1), a=0 RMPR, t_idx=10
% last_paths{1,3,10} -> sigma_s0_list(1), single-path, t_idx=10
% last_paths{2,4,20} -> sigma_s0_list(2), overlap, t_idx=20

%% ===== 6. 辅助函数 =====
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
