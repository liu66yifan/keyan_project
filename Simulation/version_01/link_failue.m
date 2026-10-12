clc;

%% ===== 参数设置 =====
total_duration = 1 * 60;   
time_interval = 15;        
time_points = total_duration / time_interval;

% 第1次：观测图（时刻开始时的网络状态）
link_fail_probability_obs = 0.01;    
node_fail_probability_perm = 0; 

% 第2次：传输过程突发（执行阶段）
link_fail_probability_tx  = 0.01;    
node_fail_probability_tx  = 0;  

rng_seed = 1;
rng(rng_seed);

target_sat = 118;
L = 4;
M_bits = 1000000;
t_idx = 2;

n_redundancy = 2; % 冗余份数

total_node_num = size(adj_matrices_static_temp, 1);

% ===== 永久节点失效状态 =====
dead_nodes_mask = false(total_node_num, 1);
failed_node_history = [];

% ===== 记录 =====
pdr_flags   = zeros(1, time_points);
pdr_running = zeros(1, time_points);
alive_node_counts = zeros(1, time_points);
z_values = zeros(1, time_points);  % 用于存储每个时刻的时延

success_cnt = 0;

%% ===== 主循环 =====
for i = 1:time_points
    fprintf('=================== 时刻 %d / %d ===================\n', i, time_points);

    %% === 1) 永久节点失效 ===
    alive_indices = find(~dead_nodes_mask);
    if ~isempty(alive_indices)
        r_node = rand(numel(alive_indices), 1);
        newly_failed_ids = alive_indices(r_node < node_fail_probability_perm);
        if ~isempty(newly_failed_ids)
            dead_nodes_mask(newly_failed_ids) = true;
            failed_node_history = [failed_node_history; newly_failed_ids]; 
            fprintf('[事件] 新增永久失效节点: %s\n', mat2str(newly_failed_ids'));
        end
    end
    alive_node_counts(i) = sum(~dead_nodes_mask);

    if dead_nodes_mask(target_sat)
        fprintf('[失败] 目标卫星 %d 已永久失效，PDR=0\n', target_sat);
        pdr_flags(i) = 0;
        pdr_running(i) = success_cnt / i;
        continue;
    end

    %% === 2) 构建观测图 ===
    adj_obs = adj_matrices_static_temp;
    adj_obs(dead_nodes_mask, :) = inf;
    adj_obs(:, dead_nodes_mask) = inf;
    adj_obs = simulate_link_failure(adj_obs, link_fail_probability_obs);

    %% === 3) 决策阶段：选路 & 注水分配 ===
    drone_sat_ids = result3(t_idx).sat_ids(:);
    drone_sat_ids = drone_sat_ids(isfinite(drone_sat_ids));

    % 剔除永久失效的接入卫星
    valid_drone_sats = drone_sat_ids(~dead_nodes_mask(drone_sat_ids));
    if numel(valid_drone_sats) < numel(drone_sat_ids)
        dead_access = setdiff(drone_sat_ids, valid_drone_sats);
        fprintf('[警告] 接入卫星 %s 已永久失效，跳过。\n', mat2str(dead_access'));
    end
    drone_sat_ids = valid_drone_sats;

    if isempty(drone_sat_ids)
        fprintf('[失败] 无可用接入卫星，PDR=0\n');
        pdr_flags(i) = 0;
        pdr_running(i) = success_cnt / i;
        continue;
    end

    best_sat = NaN;
    in_best = [];
    w_best = [];
    best_z = inf;

    % --- 遍历候选接入卫星 ---
    for sat_idx = 1:numel(drone_sat_ids)
        S = drone_sat_ids(sat_idx);
        fprintf('  > 评估接入卫星 %d ... ', S);

        try
            in_tmp = rmpr_build_waterfill_inputs_verbose_temp( ...
                t_idx, adj_obs, result3, S, target_sat, L, M_bits, false);

            if ~isfield(in_tmp, 'valid_L') || in_tmp.valid_L < 1 || isempty(in_tmp.paths)
                fprintf('无有效路径\n');
                continue;
            end

            q = in_tmp.q_sec;
            gamma = in_tmp.gamma_sec;
            [w, z] = solve_routing_optimization(q, gamma, n_redundancy);

            % 打印该候选的分配详情
            fprintf('可行!\n');
            fprintf('    valid_L=%d, sum(w)=%.4f\n', in_tmp.valid_L, sum(w));
            fprintf('    w = %s\n', vec_to_str(w, 4));
            
            cur_z = min(z);
            if cur_z < best_z
                best_z = cur_z;
                best_sat = S;
                in_best = in_tmp;
                w_best = w;
            end
        catch ME
            fprintf('计算出错: %s\n', ME.message);
        end
    end

    if isnan(best_sat) || isempty(w_best)
        fprintf('[失败] 所有候选卫星均无法构建路由。\n');
        pdr_flags(i) = 0;
        pdr_running(i) = success_cnt / i;
        continue;
    end

    % --- 打印最终选路结果 ---
    fprintf('\n[决策结果] 选中接入卫星: %d\n', best_sat);
    fprintf('最优 w 分配: %s\n', vec_to_str(w_best, 5));
    fprintf('选定路径详情:\n');
    for k = 1:numel(in_best.paths)
        p_str = sprintf('%d->', in_best.paths{k});
        fprintf('    Path %d (w=%.4f): %s\n', k, w_best(k), p_str(1:end-2));
    end
    fprintf('\n');

    %% === 4) 执行阶段：突发失效 ===
    adj_exec = adj_obs;
    transient_down = false(total_node_num, 1);
    
    % 节点突发
    candidates = find(~dead_nodes_mask); 
    if ~isempty(candidates)
        r_tx_node = rand(numel(candidates), 1);
        transient_ids = candidates(r_tx_node < node_fail_probability_tx);
        if ~isempty(transient_ids)
            transient_down(transient_ids) = true;
            adj_exec(transient_ids, :) = inf;
            adj_exec(:, transient_ids) = inf;
            fprintf('[事件] 传输阶段节点突发失效: %s\n', mat2str(transient_ids'));
        end
    end
    % 链路突发
    adj_exec = simulate_link_failure(adj_exec, link_fail_probability_tx);

    %% === 5) 结果判定 ===
    fprintf('[传输判定] 正在检查路径存活...\n');
    
    received_amount = 0;
    
    if transient_down(best_sat)
        fprintf('    [FATAL] 接入卫星 %d 突发失效！\n', best_sat);
        received_amount = 0;
    else
        survive_mask = false(1, in_best.valid_L);
        for k = 1:in_best.valid_L
            path_nodes = in_best.paths{k};
            is_alive = is_path_alive(path_nodes, adj_exec);
            
            status_str = "存活";
            if ~is_alive
                status_str = "断开";
            else
                received_amount = received_amount + w_best(k);
            end
            
            % 打印每条路的最终状态
            fprintf('    Path %d [%s] w=%.4f\n', k, status_str, w_best(k));
        end
    end
    
    delivered = (received_amount >= 1 - 1e-12);
    
    if delivered
        pdr_flags(i) = 1;
        success_cnt = success_cnt + 1;
        fprintf('[结果] 本轮成功 (收到量=%.4f)\n', received_amount);
    else
        pdr_flags(i) = 0;
        fprintf('[结果] 本轮失败 (收到量=%.4f)\n', received_amount);
    end

    pdr_running(i) = success_cnt / i;
    z_values(i) = best_z;  % 记录时延
    fprintf('\n');
end

%% ===== 画图 =====
time = (1:time_points) * time_interval / 60;
figure('Position', [100, 100, 1100, 420]);
subplot(1, 2, 1);
plot(time, pdr_running, 'LineWidth', 1.5);
xlabel('时间 (min)'); ylabel('累计PDR');
title('PDR 曲线 (含路径详情打印)'); grid on; ylim([0 1]);

subplot(1, 2, 2);
plot(time, alive_node_counts, 'r-', 'LineWidth', 1.5);
xlabel('时间 (min)'); ylabel('存活节点数'); grid on;

% 画时延曲线
figure;
plot(time, z_values, 'LineWidth', 1.5);
xlabel('时间 (min)');
ylabel('时延 (z)');
title('时延曲线');
grid on;
%% ===== 辅助函数 =====
function A = simulate_link_failure(Ain, p_fail)
    A = Ain;
    mask_valid_links = triu(isfinite(A), 1);
    [ii, jj] = find(mask_valid_links);
    failed_count = 0;
    if ~isempty(ii)
        r = rand(numel(ii), 1);
        fail_indices = (r < p_fail);
        ii_f = ii(fail_indices);
        jj_f = jj(fail_indices);
        failed_count = numel(ii_f);
        for k = 1:failed_count
            u = ii_f(k); v = jj_f(k);
            A(u, v) = inf; A(v, u) = inf;
        end
    end
    if failed_count > 0
        fprintf('    (链路失效数: %d)\n', failed_count);
    end
end

function alive = is_path_alive(p, A)
    alive = true;
    if isempty(p) || numel(p) < 2
        alive = false; return;
    end
    for k = 1:numel(p)-1
        u = p(k); v = p(k+1);
        % 这里假设节点ID与矩阵索引一致（1-based）
        if ~isfinite(A(u, v))
            alive = false; return;
        end
    end
end

function s = vec_to_str(x, prec)
    if isempty(x), s='[]'; return; end
    fmt = ['%.' num2str(prec) 'f'];
    parts = cell(1, numel(x));
    for k = 1:numel(x), parts{k} = sprintf(fmt, x(k)); end
    s = ['[' strjoin(parts, ', ') ']'];
end