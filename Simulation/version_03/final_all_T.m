%% ===== 0. 安全初始化与数据保护 =====
clc;


%% ===== 1. 参数设置 =====
% 只比较两个算法：a=0（原算法）和 a=0.7（改进算法）
alpha_list = [0,0.8];
num_alphas = numel(alpha_list);

% 扫描不同初始抖动强度 sigma_s0
sigma_s0_list = [2.0];
num_s0 = numel(sigma_s0_list);

% Monte Carlo 参数
num_iterations = 400;     % 每个时刻、每个sigma下的蒙特卡洛次数

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
% 如果工作区中没有 result3，就加载
if ~exist('result3', 'var')
    load('无人机-卫星速率数据_result3.mat', 'result3');
end

% result_static 必须已经在工作区里，且是按时刻存储的结构体数组
if ~exist('result_static', 'var')
    error('工作区中没有 result_static，请先加载 result_static。');
end

time_diedai = 1;

% 存储结果：维度 = [sigma_s0, alpha, t_idx]
pdr_matrix = zeros(num_s0, num_alphas, time_diedai);

% 用于保存“最后一次成功求解”的路径和分配结果
last_paths = cell(num_s0, num_alphas, time_diedai);
last_w_vec = cell(num_s0, num_alphas, time_diedai);

%% ===== 3. 主循环：遍历所有时刻 =====
for t_idx = 1:time_diedai
    fprintf('\n==================================================\n');
    fprintf('开始处理时刻 t_idx = %d / %d\n', t_idx, time_diedai);
    fprintf('==================================================\n');

    %% --- 3.1 提取当前时刻的静态图 dist_matrix ---
    % 当前时刻的静态结果
    result_static_temp = result_static(t_idx);

    % 为了兼容 create_diagram_dynamic(t_idx, result3, result_static_for_build, ...)
    % 构造一个长度 >= t_idx 的结构体数组，使其内部访问 result_static_for_build(t_idx) 时有效
    result_static_for_build = repmat(result_static_temp, t_idx, 1);

    [adj_matrices_static_temp, has_valid_links] = create_diagram_dynamic( ...
        t_idx, result3, result_static_for_build, ground_station_node);

    if isempty(adj_matrices_static_temp) || ~has_valid_links
        fprintf('时刻 %d: 未生成有效邻接矩阵，跳过当前时刻。\n', t_idx);
        continue;
    end

    % 当前时刻的静态图
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

        % 成功次数统计
        success_counts = zeros(1, num_alphas);

        % 临时记录“最后一次成功求解”的结果
        curr_last_paths = cell(1, num_alphas);
        curr_last_w_vec = cell(1, num_alphas);

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
            % 注意：同一次 Monte Carlo 中，a=0 和 a=0.7 共用同一张 adj_exec
            adj_exec = simulate_link_failure_oisl(dist_matrix, current_sigma_s0, min_dist_limit, false);

            % 分别测试 a=0 和 a=0.7
            for a_idx = 1:num_alphas
                curr_alpha = alpha_list(a_idx);
                curr_W = W_mats{a_idx};
                curr_Tmax = T_maxs(a_idx);
                curr_Rref = R_refs(a_idx); %#ok<NASGU>

                try
                    % 基于代价矩阵构建路径与注水输入
                    in = rmpr_build_waterfill_inputs_multi_source( ...
                        t_idx, dist_matrix, curr_W, result3, Sources, ...
                        target_sat, L, M_bits, curr_alpha, curr_Tmax, false);

                    % 注水分配
                    [w_vec, ~] = solve_routing_optimization(in.q_sec, in.gamma_sec, n_redundancy);

                    % 判定实际收到的权重是否达标
                    received_sum = 0;
                    for k = 1:numel(w_vec)
                        if is_path_alive(in.paths{k}, adj_exec)
                            received_sum = received_sum + w_vec(k);
                        end
                    end

                    % 保存最后一次成功求解到路径/分配结果
                    curr_last_paths{a_idx} = in.paths;
                    curr_last_w_vec{a_idx} = w_vec;

                    % PDR 成功判定
                    if received_sum >= 1.0 - 1e-12
                        success_counts(a_idx) = success_counts(a_idx) + 1;
                    end

                catch
                    % 寻路失败或优化失败，跳过本次
                end
            end

            % 每100次打印一个进度点
            if mod(i, 100) == 0
                fprintf('.');
            end
        end

        %% --- 3.4 存储该时刻、该 sigma 下两个算法的 PDR ---
        pdr_matrix(s_idx, :, t_idx) = success_counts / num_iterations;
        fprintf(' 完成');
        fprintf(' | PDR(a=0)=%.4f, PDR(a=0.7)=%.4f\n', ...
            pdr_matrix(s_idx, 1, t_idx), pdr_matrix(s_idx, 2, t_idx));

        %% --- 3.5 保存并打印最后一次成功求解的路径与分配 ---
        for a_idx = 1:num_alphas
            last_paths{s_idx, a_idx, t_idx} = curr_last_paths{a_idx};
            last_w_vec{s_idx, a_idx, t_idx} = curr_last_w_vec{a_idx};
        end

        fprintf('\n----- 时刻 %d, sigma_s0=%.2f 的最终路径与分配结果 -----\n', t_idx, current_sigma_s0);

        for a_idx = 1:num_alphas
            curr_alpha = alpha_list(a_idx);
            fprintf('算法 alpha = %.1f:\n', curr_alpha);

            if isempty(curr_last_paths{a_idx}) || isempty(curr_last_w_vec{a_idx})
                fprintf('  没有成功保存到路径/分配结果（可能该算法在该时刻下未成功求解）\n');
                continue;
            end

            for k = 1:numel(curr_last_w_vec{a_idx})
                fprintf('  路径 %d: %s\n', k, mat2str(curr_last_paths{a_idx}{k}));
                fprintf('          权重 = %.6f\n', curr_last_w_vec{a_idx}(k));
                fprintf('          分配数据 = %.2f bits\n', curr_last_w_vec{a_idx}(k) * M_bits);
            end
        end
    end
end

%% ===== 4. 绘图：所有结果画在一张图上，横轴改为 0 ~ (time_diedai-1) =====
figure('Color', 'w', 'Position', [100, 100, 1100, 650]);
hold on;

% 横轴：第1个时刻显示为0，第2个时刻显示为1，...，第361个时刻显示为360
time_axis = 0:(time_diedai-1);

% 每个 sigma 用一种颜色；a=0 用实线，a=0.7 用虚线
colors = {[0 0.4470 0.7410], [0.8500 0.3250 0.0980], [0.4660 0.6740 0.1880]};

for s_idx = 1:num_s0
    % a = 0（原算法）
    plot(time_axis, squeeze(pdr_matrix(s_idx, 1, :))', ...
        'LineStyle', '-', ...
        'Marker', 'o', ...
        'Color', colors{s_idx}, ...
        'LineWidth', 2, ...
        'MarkerSize', 4, ...
        'DisplayName', sprintf('\\sigma_{s0}=%.1f, a=0（原算法）', sigma_s0_list(s_idx)));

    % a = 0.7（改进算法）
    plot(time_axis, squeeze(pdr_matrix(s_idx, 2, :))', ...
        'LineStyle', '--', ...
        'Marker', 's', ...
        'Color', colors{s_idx}, ...
        'LineWidth', 2, ...
        'MarkerSize', 4, ...
        'DisplayName', sprintf('\\sigma_{s0}=%.1f, a=0.7（改进算法）', sigma_s0_list(s_idx)));
end

grid on;
xlabel('时刻 t', 'FontSize', 12);
ylabel('数据包投递率 (PDR)', 'FontSize', 12);
title('不同抖动强度下，两种算法的 PDR 随时刻变化', 'FontSize', 14);
legend('Location', 'best', 'FontSize', 10);
ylim([0 1.1]);
xlim([0, time_diedai-1]);

% 如果总时刻很多，自动稀疏显示刻度，避免太密
if time_diedai <= 20
    set(gca, 'XTick', 0:(time_diedai-1));
elseif time_diedai <= 60
    set(gca, 'XTick', 0:5:(time_diedai-1));
elseif time_diedai <= 120
    set(gca, 'XTick', 0:10:(time_diedai-1));
else
    set(gca, 'XTick', 0:20:(time_diedai-1));
end

%% ===== 5. 如需查看某个时刻、某个sigma、某个alpha的路径结果，可直接查看 =====
% 示例：
% last_paths{1,1,10}   -> sigma_s0_list(1), alpha=0, t_idx=10 的最后路径
% last_w_vec{1,1,10}   -> 对应权重
% last_paths{1,2,10}   -> sigma_s0_list(1), alpha=0.7, t_idx=10 的最后路径

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
