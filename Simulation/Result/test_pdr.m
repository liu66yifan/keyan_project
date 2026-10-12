%% ===== 0. 安全初始化与数据保护 =====
clc;

%% ===== 1. 参数设置与 Sweep 设置 =====
% 扫描抖动强度
sigma_s0_list = 0.8 : 0.05 : 0.9; 
num_s0 = numel(sigma_s0_list);

% 扫描不同权重 alpha (0 是 Baseline, 越大越看重可靠性)
alpha_list = [0,0.3,0.7]; 
num_alphas = numel(alpha_list);

num_iterations = 1; % 蒙特卡洛迭代次数
t_idx = 1;
target_sat = 402;     % 选择赤道附近的终点，对比更明显
L = 4;                % 协同接入卫星数
M_bits = 1e6;         % 8 Mb 数据量
n_redundancy = 2;     % 2倍冗余

dist_matrix = adj_matrices_static_temp;
rng(1); % 固定随机种子

% 存储结果：行对应抖动，列对应不同 alpha
pdr_matrix = zeros(num_s0, num_alphas);

%% ===== 2. 仿真主循环 =====
for s_idx = 1:num_s0
    current_sigma_s0 = sigma_s0_list(s_idx);
    fprintf('\n>>> 正在测试抖动 sigma_s0 = %.2f m\n', current_sigma_s0);

    % --- 0) 预处理：为当前抖动下的每一个 alpha 生成代价矩阵 ---
    % 存储不同 alpha 下的 W 矩阵和归一化基准
    W_mats = cell(num_alphas, 1);
    T_maxs = zeros(num_alphas, 1);
    
    for a_idx = 1:num_alphas
        [W_mats{a_idx}, T_maxs(a_idx)] = build_weighted_cost_matrix(dist_matrix, current_sigma_s0, alpha_list(a_idx));
    end

    success_counts = zeros(1, num_alphas);

    for i = 1:num_iterations
        % 获取当前时刻接入点
        drone_sat_ids = result3(t_idx).sat_ids(:);
        drone_sat_ids = drone_sat_ids(isfinite(drone_sat_ids));
        Sources = drone_sat_ids(1:min(end, L));
        if isempty(Sources), continue; end

        % 生成执行时刻真实的物理失效环境 (所有alpha方案共用此图)
        adj_exec = simulate_link_failure_oisl(dist_matrix, current_sigma_s0, false);

        % 对每一个权重 alpha 进行决策测试
        for a_idx = 1:num_alphas
            curr_alpha = alpha_list(a_idx);
            curr_W = W_mats{a_idx};
            curr_Tmax = T_maxs(a_idx);
            
            try
                % 构建注水输入并选路
                in = rmpr_build_waterfill_inputs_multi_source(t_idx, dist_matrix, curr_W, ...
                    result3, Sources, target_sat, L, M_bits, curr_alpha, curr_Tmax, false);
                
                % 注水分配权重
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
                % 寻路或优化失败视为该次传输失败
            end
        end
    end

    % 计算该抖动强度下各 alpha 的 PDR
    pdr_matrix(s_idx, :) = success_counts / num_iterations;
    
    % 实时打印结果
    for a_idx = 1:num_alphas
        fprintf('  Alpha=%.2f: PDR=%.4f |', alpha_list(a_idx), pdr_matrix(s_idx, a_idx));
    end
    fprintf('\n');
end

%% ===== 3. 绘图结果对比 =====
figure('Color', 'w', 'Position', [100, 100, 850, 600]);
markers = {'o', 's', '^', 'd', 'v'};
colors = lines(num_alphas);

for a_idx = 1:num_alphas
    plot_label = sprintf('\\alpha = %.2f', alpha_list(a_idx));
    if alpha_list(a_idx) == 0
        plot_label = [plot_label ' (Baseline)'];
        style = '--'; % 基准线用虚线
    else
        style = '-';  % 改进方案用实线
    end
    
    % 将 'Style' 改为 'LineStyle'
    plot(sigma_s0_list, pdr_matrix(:, a_idx), ...
        'LineStyle', style, ...
        'Marker', markers{mod(a_idx-1,length(markers))+1}, ...
        'Color', colors(a_idx,:), ...
        'LineWidth', 2, ...
        'MarkerSize', 8, ...
        'DisplayName', plot_label);
    hold on;
end

grid on;
xlabel('初始抖动强度 \sigma_{s0} (m)', 'FontSize', 12);
ylabel('数据包投递率 (PDR)', 'FontSize', 12);
title('不同权重因子 \alpha 下的系统可靠性对比', 'FontSize', 14);
legend('Location', 'southwest', 'FontSize', 10);
ylim([0 1.1]);

%% ===== 4. 辅助函数 (保持不变) =====
function A = simulate_link_failure_oisl(dist_mat, sigma_s0, verbose)
    A = dist_mat;
    [ii, jj] = find(triu(isfinite(A) & A > 0, 1));
    for k = 1:numel(ii)
        z_m = A(ii(k), jj(k));
        p_fail = oisl_failure_prob(z_m, sigma_s0);
        if rand() < p_fail
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