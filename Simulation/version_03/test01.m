clc;
% 多点协同接入（Multi-Source）下的 RMPR 可靠性仿真（带权重路径打印）
% 场景：在代价空间 (Latency + Reliability) 下进行寻路与分配

%% ===== 1. 参数设置与 Sweep 设置 =====
sigma_s0_list = [2]; % 测试不同抖动
num_s0 = numel(sigma_s0_list);

num_iterations = 100; % 蒙特卡洛次数
t_idx = 1;
target_sat = 401;   
L = 4;              
M_bits = 1e6;       
n_redundancy = 2;   
alpha_val = 0.7;   % 可靠性权重偏好

dist_matrix = adj_matrices_static_temp;
total_node_num = size(dist_matrix, 1);
c = 299792458;

rng(1);
pdr_results = zeros(num_s0, 1);

%% ===== 2. 抖动循环 =====
for s_idx = 1:num_s0
    current_sigma_s0 = sigma_s0_list(s_idx);
    fprintf('\n================= sigma_s0 = %.2f m =================\n', current_sigma_s0);

    % --- 0) 预处理：生成当前抖动下的代价矩阵和归一化基准 ---
    % 假设 build_weighted_cost_matrix 返回 [矩阵, Tmax, Rmax]
    [W_matrix, T_max_v, R_max_v] = build_weighted_cost_matrix(dist_matrix, current_sigma_s0, alpha_val);

    success_cnt = 0;

    for i = 1:num_iterations
        verbose_print = (i == 1); % 每种抖动只打印第一次结果

        %% --- 1) 初始化失效掩码 ---
         dead_nodes_mask = false(total_node_num, 1);
        if dead_nodes_mask(target_sat + 1), continue; end

        %% --- 2) 决策阶段 ---
        % 寻路算法看到的是经过加权惩罚的地图
        adj_obs_cost = W_matrix; 
        adj_obs_cost(dead_nodes_mask,:) = inf;
        adj_obs_cost(:,dead_nodes_mask) = inf;

        %% --- 3) 决策选路 (Multi-Source 逻辑) ---
        drone_sat_ids = result3(t_idx).sat_ids(:);
        drone_sat_ids = drone_sat_ids(isfinite(drone_sat_ids));
        Sources = drone_sat_ids(1:min(end, L)); 

        if isempty(Sources), continue; end

        try
            % 构建输入（传入物理矩阵和代价矩阵）
            in_multi = rmpr_build_waterfill_inputs_multi_source_weight( ...
                t_idx, dist_matrix, W_matrix, result3, Sources, target_sat, ...
                L, M_bits, alpha_val, T_max_v, false);

            if in_multi.valid_L < 1, continue; end

            % 联合注水优化
            [w_multi, z_multi] = solve_routing_optimization( ...
                in_multi.q_sec, in_multi.gamma_sec, n_redundancy);
        catch
            continue; 
        end

        %% --- 4) 执行阶段 ---
        adj_exec = simulate_link_failure_oisl(dist_matrix, current_sigma_s0, false);

        %% --- 5) 判定接收成功 ---
        received_amount = 0;
        for k = 1:in_multi.valid_L
            if is_path_alive(in_multi.paths{k}, adj_exec)
                received_amount = received_amount + w_multi(k);
            end
        end

        if received_amount >= 1 - 1e-12
            success_cnt = success_cnt + 1;
        end

        %% ===== 详细日志打印：包含距离、失效概率、综合代价 =====
        if verbose_print
            fprintf('\n=========== MULTI-SOURCE WEIGHTED CHECK PRINT ===========\n');
            fprintf('Target Satellite (D): %d | Redundancy n: %d | Alpha: %.2f\n', target_sat, n_redundancy, alpha_val);
            
            for pp = 1:in_multi.valid_L
                p = in_multi.paths{pp};
                fprintf('\nPath %d (from Sat %d) | Allocation w = %.6f:\n', ...
                    pp, in_multi.access_sats(pp), w_multi(pp));
                fprintf('  Route: %s\n', path_to_string_local(p));

                % 遍历该路径的每一跳
                for ee = 1:numel(p)-1
                    u = p(ee); v = p(ee+1);
                    z_m = dist_matrix(u+1, v+1);
                    
                    % 1. 物理失效概率
                    pf = oisl_failure_prob(z_m, current_sigma_s0);
                    
                    % 2. 计算综合代价 (还原 build_weighted_cost_matrix 的内部逻辑)
                    t_norm = (z_m / c) / T_max_v;
                    r_raw = -log(1 - min(pf, 0.9999));
                    r_norm = r_raw / R_max_v;
                    hop_cost = (1 - alpha_val) * t_norm + alpha_val * r_norm;

                    fprintf('    Edge %3d->%-3d | Dist: %8.2f km | p_fail: %.6f | Hop_Cost: %.4f\n', ...
                        u, v, z_m/1e3, pf, hop_cost);
                end
                fprintf('  Total Path Cost s_i (Floor): %.4f\n', in_multi.q_sec(pp));
            end
            fprintf('\n[Water-filling Summary]\n');
            fprintf('  Optimization Max Cost z: %.6f\n', min(z_multi));
            fprintf('========================================================\n\n');
        end
    end

    pdr_results(s_idx) = success_cnt / num_iterations;
    fprintf('sigma_s0 = %.2f, Multi-Source Weighted PDR = %.4f\n', current_sigma_s0, pdr_results(s_idx));
end

function A = simulate_link_failure_oisl(dist_mat, sigma_s0, verbose)
    % 基于 OISL 物理模型模拟链路随机失效
    A = dist_mat;
    % 只处理上三角矩阵，避免重复计算双向链路
    [ii, jj] = find(triu(isfinite(A) & A > 0, 1));
    
    fail_count = 0;
    for k = 1:numel(ii)
        z_m = A(ii(k), jj(k));
        % 调用物理模型计算失效概率
        p_fail = oisl_failure_prob(z_m, sigma_s0);
        
        % 蒙特卡洛抽样：如果随机数小于失效概率，则链路断开
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



%% --- 辅助函数 (simulate_link_failure_oisl 等) ---
% 此处粘贴你之前的辅助函数代码...
    function alive = is_path_alive(p, A)
        % 检查给定路径 p 在当前图 A 中是否依然连通
        alive = true;
        for k = 1:numel(p)-1
        u = p(k) + 1;
        v = p(k+1) + 1;
        if ~isfinite(A(u,v))
        alive = false;
        return;
        end
        end
    end
function s = path_to_string_local(p)
% 将路径数组转为字符串显示
s = num2str(p(1));
for i = 2:numel(p)
s = [s '->' num2str(p(i))];
end
end