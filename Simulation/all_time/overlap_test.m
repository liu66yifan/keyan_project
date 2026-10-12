%% ===== 测试：比较 disjoint 与 overlap 选出来的路径 =====

fprintf('\n============================================================\n');
fprintf('           Disjoint / Overlap 路径选择对比测试\n');
fprintf('============================================================\n');

% 你自己按需要改这几个
t_idx = 1;
target_sat = 402;          % 目标节点（0开始编号）
curr_alpha = 0;         % 测试 alpha，可改成 0 或 0.7
current_sigma_s0 = 2.2;   % 测试扰动强度
L_paths = L;              % 一般就是 4

% 当前时刻接入卫星
drone_sat_ids = result3(t_idx).sat_ids(:);
drone_sat_ids = drone_sat_ids(isfinite(drone_sat_ids));
Sources = drone_sat_ids(1:min(end, L_paths));

fprintf('时刻 t_idx = %d\n', t_idx);
fprintf('目标节点 target_sat = %d\n', target_sat);
fprintf('候选 Sources = %s\n', mat2str(Sources(:)'));
fprintf('alpha = %.2f\n', curr_alpha);
fprintf('sigma_s0 = %.2f\n', current_sigma_s0);
fprintf('L_paths = %d\n', L_paths);

% 生成当前 alpha 对应的代价矩阵
[W_test, T_max_test, R_ref_test] = build_weighted_cost_matrix( ...
    dist_matrix, current_sigma_s0, curr_alpha); %#ok<NASGU>

%% ==================== 1) 原方法：Disjoint ====================
fprintf('\n================ Disjoint 方法选路结果 ================\n');

try
    in_disjoint = rmpr_build_waterfill_inputs_multi_source( ...
        t_idx, dist_matrix, W_test, result3, Sources, target_sat, ...
        L_paths, M_bits, curr_alpha, T_max_test, false);

    fprintf('valid_L = %d\n', in_disjoint.valid_L);

    fprintf('\n--- Disjoint 选出的路径 ---\n');
    for k = 1:in_disjoint.valid_L
        fprintf('路径 %d: %s\n', k, mat2str(in_disjoint.paths{k}));
        fprintf('        接入星 = %d\n', in_disjoint.access_sats(k));
        fprintf('        q_sec  = %.6f\n', in_disjoint.q_sec(k));
        fprintf('        gamma  = %.6f\n', in_disjoint.gamma_sec(k));
    end

catch ME
    fprintf('Disjoint 方法报错：%s\n', ME.message);
end

fprintf('=======================================================\n');

%% ==================== 2) 新方法：Overlap ====================
fprintf('\n================ Overlap 方法选路结果 ================\n');

try
    in_overlap = rmpr_build_waterfill_inputs_multi_source_overlap( ...
        t_idx, dist_matrix, W_test, result3, Sources, target_sat, ...
        L_paths, M_bits, curr_alpha, T_max_test, false);

    fprintf('valid_L = %d\n', in_overlap.valid_L);

    fprintf('\n--- Overlap 选出的路径 ---\n');
    for k = 1:in_overlap.valid_L
        fprintf('路径 %d: %s\n', k, mat2str(in_overlap.paths{k}));
        fprintf('        接入星 = %d\n', in_overlap.access_sats(k));
        fprintf('        q_sec  = %.6f\n', in_overlap.q_sec(k));
        fprintf('        gamma  = %.6f\n', in_overlap.gamma_sec(k));
    end

catch ME
    fprintf('Overlap 方法报错：%s\n', ME.message);
end

fprintf('=======================================================\n');
