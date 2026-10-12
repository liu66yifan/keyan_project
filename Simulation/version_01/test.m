clc;
% 设定高危 卫星id 

%% ===== 参数设置 =====
total_duration = 90 * 60;   
time_interval = 15;        
time_points = total_duration / time_interval;
total_node_num = size(adj_matrices_static_temp, 1);

% --- 1. 观测图参数（算法以为的网络状态） ---
link_fail_probability_obs = 0.001;    % 观测时认为链路很稳
node_fail_probability_perm = 0.0005; 

% --- 2. 执行图参数（真实残酷的网络环境） ---
% 构造非均匀的高危环境 (陷阱)
% 基础失效概率（正常区域）
base_exec_fail_prob = 0.001; 
% 高危失效概率（干扰区域）
high_risk_prob = 0.2;         

% 初始化全网失效概率矩阵
P_fail_exec_matrix = ones(total_node_num, total_node_num) * base_exec_fail_prob;

% 【关键设置】定义高危节点集合 (假设是一片连续的ID，模拟经过南大西洋异常区或战区)
% 设定 ID 30 到 60 的卫星处于高危区
risky_nodes = 30:50; 

% 将连接高危节点的所有链路的失效概率设为 50%
P_fail_exec_matrix(risky_nodes, :) = high_risk_prob;
P_fail_exec_matrix(:, risky_nodes) = high_risk_prob;

% 节点突发失效概率
node_fail_probability_tx  = 0.0005;  

rng_seed = 1;
rng(rng_seed);

target_sat = 99; % 目标在 99
L = 4;
M_bits = 1000000;
t_idx = 1;
n_redundancy = 2; 

% ===== 状态初始化 =====
dead_nodes_mask = false(total_node_num, 1);
failed_node_history = [];
pdr_flags   = zeros(1, time_points);
pdr_running = zeros(1, time_points);
alive_node_counts = zeros(1, time_points);
success_cnt = 0;

%% ===== 主循环 =====
for i = 1:time_points
    fprintf('=== 时刻 %d / %d ===\n', i, time_points);

    %% === 1) 永久节点失效 ===
    alive_indices = find(~dead_nodes_mask);
    if ~isempty(alive_indices)
        r_node = rand(numel(alive_indices), 1);
        newly_failed_ids = alive_indices(r_node < node_fail_probability_perm);
        if ~isempty(newly_failed_ids)
            dead_nodes_mask(newly_failed_ids) = true;
            fprintf('[事件] 永久失效节点: %s\n', mat2str(newly_failed_ids'));
        end
    end
    alive_node_counts(i) = sum(~dead_nodes_mask);

    if dead_nodes_mask(target_sat)
        fprintf('[失败] 目标卫星 %d 已失效\n\n', target_sat);
        continue;
    end

    %% === 2) 构建观测图 (用于选路) ===
    % 这一步模拟“控制器”看到的世界。控制器不知道高危区的存在，
    % 或者认为所有链路的质量都是一样的（link_fail_probability_obs 很小）。
    adj_obs = adj_matrices_static_temp;
    adj_obs(dead_nodes_mask, :) = inf;
    adj_obs(:, dead_nodes_mask) = inf;
    % 观测阶段仅加入微小的随机噪声
    adj_obs = simulate_link_failure_uniform(adj_obs, link_fail_probability_obs);

    %% === 3) 决策阶段：RMPR 选路 (基于观测图) ===
    drone_sat_ids = result3(t_idx).sat_ids(:);
    drone_sat_ids = drone_sat_ids(isfinite(drone_sat_ids));
    valid_drone_sats = drone_sat_ids(~dead_nodes_mask(drone_sat_ids));
    
    if isempty(valid_drone_sats)
        fprintf('[失败] 无可用接入卫星\n\n');
        continue;
    end

    best_sat = NaN;
    in_best = [];
    w_best = [];
    best_z = inf;

    for sat_idx = 1:numel(valid_drone_sats)
        S = valid_drone_sats(sat_idx);
        try
            % 这里的 adj_obs 是“干净”的，不包含高危信息
            % 因此 RMPR 会倾向于选择跳数最少的最短路径
            in_tmp = rmpr_build_waterfill_inputs_verbose_temp( ...
                t_idx, adj_obs, result3, S, target_sat, L, M_bits, false);

            if ~isfield(in_tmp, 'valid_L') || in_tmp.valid_L < 1; continue; end

            [w, z] = solve_routing_optimization(in_tmp.q_sec, in_tmp.gamma_sec, n_redundancy);
            
            cur_z = min(z);
            if cur_z < best_z
                best_z = cur_z;
                best_sat = S;
                in_best = in_tmp;
                w_best = w;
            end
        catch; continue; end
    end

    if isnan(best_sat)
        fprintf('[失败] 无法计算路由\n\n');
        continue;
    end

    % --- 分析：检查选出的路径是否“踩雷”了 ---
    % 这一步是为了验证你的实验设置是否成功诱导了算法犯错
    hit_risk_zone = false;
    for k = 1:numel(in_best.paths)
        path_nodes = in_best.paths{k};
        % 检查路径中是否有节点属于 risky_nodes
        if any(ismember(path_nodes, risky_nodes))
            hit_risk_zone = true;
        end
    end
    
    fprintf('  选中接入: %d. ', best_sat);
    if hit_risk_zone
        fprintf('【踩雷】路径穿过了高危区域 (ID 30-60)！\n');
    else
        fprintf('路径未穿过高危区域。\n');
    end

    %% === 4) 执行阶段：真实环境模拟 (引入非均匀的高危失效) ===
    adj_exec = adj_obs; % 基础拓扑相同
    
    % 4.1 节点突发失效
    transient_down = false(total_node_num, 1);
    candidates = find(~dead_nodes_mask); 
    if ~isempty(candidates)
        r_tx = rand(numel(candidates), 1);
        t_ids = candidates(r_tx < node_fail_probability_tx);
        transient_down(t_ids) = true;
        adj_exec(t_ids, :) = inf;
        adj_exec(:, t_ids) = inf;
    end
    
    % 4.2 链路突发失效 (使用非均匀矩阵！)
    % 这里使用的是 P_fail_exec_matrix，高危区的链路有 50% 概率断开
    adj_exec = simulate_link_failure_matrix(adj_exec, P_fail_exec_matrix);

    %% === 5) 结果判定 ===
    received_amount = 0;
    
    if transient_down(best_sat)
        fprintf('  [FATAL] 接入卫星 %d 突发失效\n', best_sat);
        received_amount = 0;
    else
        survive_mask = false(1, in_best.valid_L);
        for k = 1:in_best.valid_L
            % 检查路径在“恶劣环境”下是否存活
            survive_mask(k) = is_path_alive(in_best.paths{k}, adj_exec);
            if survive_mask(k)
                received_amount = received_amount + w_best(k);
            else
                % 打印一下哪条路断了，用于观察
                % fprintf('    Path %d 断开 (w=%.2f)\n', k, w_best(k));
            end
        end
    end

    if received_amount >= 1 - 1e-12
        pdr_flags(i) = 1;
        success_cnt = success_cnt + 1;
        fprintf('  [结果] 成功 (Received=%.2f)\n', received_amount);
    else
        pdr_flags(i) = 0;
        fprintf('  [结果] 失败 (Received=%.2f)\n', received_amount);
    end
    
    pdr_running(i) = success_cnt / i;
    fprintf('\n');
end

%% ===== 画图 =====
figure; plot(pdr_running, 'LineWidth', 1.5); 
title('原RMPR算法在高危环境下的PDR'); ylim([0 1]); grid on;

%% ================== 辅助函数 ==================

% 1. 原来的均匀失效函数 (用于观测阶段)
function A = simulate_link_failure_uniform(Ain, p_fail)
    A = Ain;
    mask = triu(isfinite(A), 1);
    [ii, jj] = find(mask);
    if ~isempty(ii)
        r = rand(numel(ii), 1);
        fail = (r < p_fail);
        for k = 1:numel(ii)
            if fail(k)
                u = ii(k); v = jj(k);
                A(u, v) = inf; A(v, u) = inf;
            end
        end
    end
end

% 2. 【新】非均匀失效函数 (用于执行阶段)
% 输入的 P_mat 是一个 N*N 的矩阵，定义了每条链路的断开概率
function A = simulate_link_failure_matrix(Ain, P_mat)
    A = Ain;
    % 找到所有存在的链路 (只看上三角，避免重复)
    mask = triu(isfinite(A), 1);
    [ii, jj] = find(mask);
    
    if ~isempty(ii)
        for k = 1:numel(ii)
            u = ii(k); v = jj(k);
            % 从矩阵中获取该特定链路的失效概率
            prob_this_link = P_mat(u, v);
            
            % 掷骰子
            if rand() < prob_this_link
                A(u, v) = inf;
                A(v, u) = inf;
            end
        end
    end
end

function alive = is_path_alive(p, A)
    alive = true;
    if isempty(p) || numel(p) < 2
        alive = false; return;
    end
    for k = 1:numel(p)-1
        u = p(k); v = p(k+1);
        if ~isfinite(A(u, v)), alive = false; return; end
    end
end

function s = vec_to_str(x, prec)
    if isempty(x), s='[]'; return; end
    fmt = ['%.' num2str(prec) 'f'];
    parts = cell(1, numel(x));
    for k = 1:numel(x), parts{k} = sprintf(fmt, x(k)); end
    s = ['[' strjoin(parts, ', ') ']'];
end