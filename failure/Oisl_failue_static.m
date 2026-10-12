clc;
% 固定概率数值抽奖！
%% ===== 概率 sweep 设置 =====
p_fail_list = [1e-4, 1e-3, 1e-2,1e-1];   % x 轴
num_p = numel(p_fail_list);

num_iterations = 1000;   % 每个概率 Monte Carlo 次数
t_idx = 1;               % 固定第一个时刻

target_sat = 109;
L = 4;
M_bits = 1e6;
n_redundancy = 2;

% 不考虑节点失效
node_fail_probability_perm = 0;
node_fail_probability_tx   = 0;

rng(1);  % 可复现

total_node_num = size(adj_matrices_static_temp, 1);

pdr_results = zeros(num_p, 1);   % y 轴

%% ===== 概率循环 =====
for p_idx = 1:num_p
    p_fail = p_fail_list(p_idx);
    fprintf('\n================= 链路失效概率 p = %.1e =================\n', p_fail);

    link_fail_probability_obs = p_fail;
    link_fail_probability_tx  = p_fail;

    success_cnt = 0;

    %% ===== Monte Carlo =====
    for i = 1:num_iterations

        verbose = (p_idx == 1 && i == 1);  % 只打印一次

        if verbose
            fprintf('---- 第一次实验（打印选路） ----\n');
        end

        %% --- 1) 初始化 ---
        dead_nodes_mask = false(total_node_num, 1);

        if dead_nodes_mask(target_sat)
            continue;
        end

        %% --- 2) 构建观测图 ---
        adj_obs = adj_matrices_static_temp;
        adj_obs(dead_nodes_mask,:) = inf;
        adj_obs(:,dead_nodes_mask) = inf;
        adj_obs = simulate_link_failure_silent(adj_obs, link_fail_probability_obs, verbose);

        %% --- 3) 决策阶段：选路 ---
        drone_sat_ids = result3(t_idx).sat_ids(:);
        drone_sat_ids = drone_sat_ids(isfinite(drone_sat_ids));
        valid_drone_sats = drone_sat_ids(~dead_nodes_mask(drone_sat_ids));

        if isempty(valid_drone_sats)
            continue;
        end

        best_sat = NaN;
        in_best = [];
        w_best = [];
        best_z = inf;

        for sat_idx = 1:numel(valid_drone_sats)
            S = valid_drone_sats(sat_idx);
            try
                in_tmp = rmpr_build_waterfill_inputs_verbose_temp( ...
                    t_idx, adj_obs, result3, S, target_sat, L, M_bits, false);

                if ~isfield(in_tmp,'valid_L') || in_tmp.valid_L < 1
                    continue;
                end

                [w, z] = solve_routing_optimization( ...
                    in_tmp.q_sec, in_tmp.gamma_sec, n_redundancy);

                cur_z = min(z);
                if cur_z < best_z
                    best_z = cur_z;
                    best_sat = S;
                    in_best = in_tmp;
                    w_best = w;
                end
            catch
                continue;
            end
        end

        if isnan(best_sat)
            continue;
        end

        %% --- 第一次打印选路 ---
        if verbose
            fprintf('[选路结果] 接入卫星 = %d\n', best_sat);
            for k = 1:numel(in_best.paths)
                p_str = sprintf('%d->', in_best.paths{k});
                fprintf('  Path %d (w=%.4f): %s\n', ...
                    k, w_best(k), p_str(1:end-2));
            end
        end

        %% --- 4) 执行阶段 ---
        adj_exec = adj_obs;
        adj_exec = simulate_link_failure_silent(adj_exec, link_fail_probability_tx, false);

        %% --- 5) 判定 ---
        received_amount = 0;
        for k = 1:in_best.valid_L
            if is_path_alive(in_best.paths{k}, adj_exec)
                received_amount = received_amount + w_best(k);
            end
        end

        if received_amount >= 1 - 1e-12
            success_cnt = success_cnt + 1;
        end
    end

    %% ===== 当前概率的 PDR =====
    pdr_results(p_idx) = success_cnt / num_iterations;
    fprintf('p = %.1e, PDR = %.4f\n', p_fail, pdr_results(p_idx));
end

%% ===== 画图：概率–PDR =====
figure;
semilogx(p_fail_list, pdr_results, '-o', 'LineWidth', 1.8);
xlabel('链路失效概率 p');
ylabel('PDR');
title('固定链路失效概率下的 PDR 基线曲线');
grid on;
ylim([0 1]);

%% ===== 辅助函数 =====
function A = simulate_link_failure_silent(Ain, p_fail, verbose)
    A = Ain;
    if p_fail <= 0, return; end
    mask = triu(isfinite(A),1);
    [ii,jj] = find(mask);
    if isempty(ii), return; end
    r = rand(numel(ii),1);
    idx = r < p_fail;
    ii = ii(idx); jj = jj(idx);
    for k = 1:numel(ii)
        A(ii(k),jj(k)) = inf;
        A(jj(k),ii(k)) = inf;
    end
    if verbose && ~isempty(ii)
        fprintf('  链路失效数: %d\n', numel(ii));
    end
end

function alive = is_path_alive(p, A)
    alive = true;
    for k = 1:numel(p)-1
        if ~isfinite(A(p(k),p(k+1)))
            alive = false; return;
        end
    end
end
