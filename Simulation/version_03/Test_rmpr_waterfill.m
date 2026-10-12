%% ===== 0. 安全初始化与数据保护 =====
clc;


%% ===== 1. 参数设置与 Sweep 设置 =====
% 1. 扫描权重因子 alpha (X 轴)
alpha_list = 0 : 0.5 : 1.0; 
num_alphas = numel(alpha_list);

% 2. 扫描不同初始抖动强度 sigma_s0 (多条线)
sigma_s0_list = [2.0,2.2]; 
num_s0 = numel(sigma_s0_list);

num_iterations = 50; % 蒙特卡洛迭代次数，建议 500 以上
t_idx = 1;
target_sat = 19;     % 目标卫星
L = 4;                % 协同接入卫星数
M_bits = 1e6;         % 传输数据量
n_redundancy = 2;     % 冗余度
min_dist_limit = 600e3; % 硬截断阈值

dist_matrix = adj_matrices_static_temp;
rng(1); % 固定随机种子

% 存储结果：行对应不同抖动，列对应不同 alpha
pdr_matrix = zeros(num_s0, num_alphas);

%% ===== 2. 仿真主循环 =====
for s_idx = 1:num_s0
    current_sigma_s0 = sigma_s0_list(s_idx);
    fprintf('\n>>> 正在测试抖动强度 sigma_s0 = %.2f m ', current_sigma_s0);

    % --- 0) 预生成当前抖动下不同 alpha 的代价矩阵，提高效率 ---
    W_mats = cell(num_alphas, 1);
    T_maxs = zeros(num_alphas, 1);
    R_refs = zeros(num_alphas, 1);
    for a_idx = 1:num_alphas
        % 调用 build_weighted_cost_matrix 获得矩阵和归一化基准
        [W_mats{a_idx}, T_maxs(a_idx), R_refs(a_idx)] = build_weighted_cost_matrix(dist_matrix, current_sigma_s0, alpha_list(a_idx));
    end

    success_counts = zeros(1, num_alphas);

    for i = 1:num_iterations
        % 获取当前时刻接入点
        drone_sat_ids = result3(t_idx).sat_ids(:);
        drone_sat_ids = drone_sat_ids(isfinite(drone_sat_ids));
        Sources = drone_sat_ids(1:min(end, L));
        if isempty(Sources), continue; end

        % 生成传输时刻真实的物理失效环境 (所有alpha方案在当前迭代共用此图)
        adj_exec = simulate_link_failure_oisl(dist_matrix, current_sigma_s0, min_dist_limit, false);

        % 对每一个权重 alpha 进行决策测试
        for a_idx = 1:num_alphas
            curr_alpha = alpha_list(a_idx);
            curr_W = W_mats{a_idx};
            curr_Tmax = T_maxs(a_idx);
            curr_Rref = R_refs(a_idx);
            
            try
                % 基于代价矩阵寻路并构建注水输入
                in = rmpr_build_waterfill_inputs_multi_source(t_idx, dist_matrix, curr_W, ...
                    result3, Sources, target_sat, L, M_bits, curr_alpha, curr_Tmax, false);
                
                % 注水分配
                [w_vec, ~] = solve_routing_optimization(in.q_sec, in.gamma_sec, n_redundancy);
                
                % 判定接收权重之和是否达标 (>= 1.0)
                received_sum = 0;
                for k = 1:numel(w_vec)
                    if is_path_alive(in.paths{k}, adj_exec)
                        received_sum = received_sum + w_vec(k);
                    end
                end
                
                if received_sum >= 1.0 - 1e-12
                    success_counts(a_idx) = success_counts(a_idx) + 1;
                end
            catch
                % 寻路失败或优化失败跳过
            end
        end
        
        % 每100次打印进度点
        if mod(i, 100) == 0, fprintf('.'); end
    end
    
    pdr_matrix(s_idx, :) = success_counts / num_iterations;
    fprintf(' 完成\n');
end

%% ===== 3. 绘图结果展现 =====
figure('Color', 'w', 'Position', [100, 100, 800, 600]);
colors = {[0 0.4470 0.7410], [0.8500 0.3250 0.0980], [0.4660 0.6740 0.1880]}; % 蓝、橙、绿
markers = {'o', 's', '^'};

for s_idx = 1:num_s0
    plot(alpha_list, pdr_matrix(s_idx, :), ...
        'LineStyle', '-', ...
        'Marker', markers{s_idx}, ...
        'Color', colors{s_idx}, ...
        'LineWidth', 2, ...
        'MarkerSize', 8, ...
        'MarkerFaceColor', colors{s_idx}, ...
        'DisplayName', sprintf('\\sigma_{s0} = %.1f m', sigma_s0_list(s_idx)));
    hold on;
end

grid on;
set(gca, 'XTick', 0:0.1:1);
xlabel('权重因子 \alpha (可靠性权重偏好 \rightarrow)', 'FontSize', 12);
ylabel('数据包投递率 (PDR)', 'FontSize', 12);
title('不同抖动强度下 PDR 随权重因子 \alpha 的演变趋势', 'FontSize', 14);
legend('Location', 'best', 'FontSize', 11);
ylim([0 1.1]);

%% ===== 4. 辅助函数 (脚本底部) =====
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
        if ~isfinite(A(p(k)+1, p(k+1)+1)), alive = false; return; end
    end
end