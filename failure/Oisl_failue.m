clc;
%这个代码在做蒙特卡洛仿真系统不同激光抖动强度下，（RMPR）可靠性表现
%目前算法是多个候选点找最优接入点，每个接入点找4条到达目标的路径
%% ===== 1. 参数设置与 Sweep 设置 =====
sigma_s0_list = [0.85,0.9,1.0];
num_s0 = numel(sigma_s0_list);

num_iterations = 100;
t_idx = 1;

target_sat = 109;   % 0-based 节点号
L = 4;
M_bits = 1e6;
n_redundancy = 2;

dist_matrix = adj_matrices_static_temp;
total_node_num = size(dist_matrix, 1);

rng(1);
pdr_results = zeros(num_s0, 1);

%% ===== 2. 抖动循环 =====
for s_idx = 1:num_s0
    current_sigma_s0 = sigma_s0_list(s_idx);
    fprintf('\n================= sigma_s0 = %.2f m =================\n', current_sigma_s0);

    success_cnt = 0;

    for i = 1:num_iterations
        verbose = (s_idx == 1 && i == 1);

        %% --- 1) 初始化 ---
        dead_nodes_mask = false(total_node_num, 1);
        if dead_nodes_mask(target_sat + 1)
            continue;
        end

        %% --- 2) 观测图 ---
        adj_obs = simulate_link_failure_oisl(dist_matrix, current_sigma_s0, verbose);
        adj_obs(dead_nodes_mask,:) = inf;
        adj_obs(:,dead_nodes_mask) = inf;

        %% --- 3) 决策选路 ---
        drone_sat_ids = result3(t_idx).sat_ids(:);
        drone_sat_ids = drone_sat_ids(isfinite(drone_sat_ids));
        valid_drone_sats = drone_sat_ids(~dead_nodes_mask(drone_sat_ids + 1));

        if isempty(valid_drone_sats)
            continue;
        end

        best_sat = NaN;
        in_best = [];
        w_best = [];
        best_z = inf;

        for sat_idx = 1:numel(valid_drone_sats)
            S = valid_drone_sats(sat_idx); % 0-based
            try
                in_tmp = rmpr_build_waterfill_inputs_verbose_temp( ...
                    t_idx, adj_obs, result3, S, target_sat, L, M_bits, false);

                if in_tmp.valid_L < 1
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

        %% --- 4) 执行图 ---
        adj_exec = simulate_link_failure_oisl(dist_matrix, current_sigma_s0, false);

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

        %% ===== 只打印：路径 + 距离索引 + p_fail + 注水结果 =====
        if verbose
            fprintf('\n=========== CHECK PRINT ===========\n');
            fprintf('best_sat(S)=%d, target(D)=%d, valid_L=%d\n', best_sat, target_sat, in_best.valid_L);

            for pp = 1:in_best.valid_L
                p = in_best.paths{pp};
                fprintf('\nPath %d: %s\n', pp, path_to_string_local(p));

                for ee = 1:numel(p)-1
                    u = p(ee);
                    v = p(ee+1);

                    ui = u + 1;
                    vi = v + 1;
                    z_m = dist_matrix(ui, vi);

                    p_fail = oisl_failure_prob(z_m, current_sigma_s0);

                    fprintf('  edge %d->%d | dist_matrix(%d,%d)=%.6f m | p_fail=%.8f\n', ...
                        u, v, ui, vi, z_m, p_fail);
                end
            end

            % ===== 新增：打印注水分配 w =====
            fprintf('\n[Water-filling allocation]\n');
            fprintf('sum(w) = %.12f\n', sum(w_best));
            for kk = 1:numel(w_best)
                fprintf('  w(%d) = %.12f\n', kk, w_best(kk));
            end

            fprintf('==================================\n\n');
        end
    end

    pdr_results(s_idx) = success_cnt / num_iterations;
    fprintf('sigma_s0 = %.2f, PDR = %.4f\n', current_sigma_s0, pdr_results(s_idx));
end

%% ===== 3. 绘图 =====
figure;
plot(sigma_s0_list, pdr_results, '-s', 'LineWidth', 2);
xlabel('\sigma_{s0} (m)');
ylabel('PDR');
grid on;
ylim([0 1.05]);

%% ===== 4. 辅助函数 =====
function A = simulate_link_failure_oisl(dist_mat, sigma_s0, verbose)
    A = dist_mat;
    [ii, jj] = find(triu(isfinite(A) & A > 0, 1));

    fail_count = 0;
    for k = 1:numel(ii)
        z_m = A(ii(k), jj(k));
        p_fail = oisl_failure_prob(z_m, sigma_s0);
        if rand() < p_fail
            A(ii(k), jj(k)) = inf;
            A(jj(k), ii(k)) = inf;
            fail_count = fail_count + 1;
        end
    end

    if verbose
        fprintf('  [OISL] total edges=%d, failed=%d\n', numel(ii), fail_count);
    end
end

function alive = is_path_alive(p, A)
    alive = true;
    N = size(A,1);
    for k = 1:numel(p)-1
        u = p(k) + 1;
        v = p(k+1) + 1;
        if u<1 || u>N || v<1 || v>N || ~isfinite(A(u,v))
            alive = false; return;
        end
    end
end

function s = path_to_string_local(p)
    s = sprintf('%d', p(1));
    for i = 2:numel(p)
        s = [s '->' num2str(p(i))]; %#ok<AGROW>
    end
end
