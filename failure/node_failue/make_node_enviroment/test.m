clc;
% 假设你已经有：
% t_idx, dist_matrix, result3, Sources, target_sat, L, M_bits, n_redundancy

alpha_val = 0.7;
sigma_s0 = 2.1;
dist_matrix = adj_matrix;
% 这里用旧函数，得到“没有节点失效权重”时的基线
[W0, T_max0, R_ref0] = ...
    build_weighted_cost_matrix(dist_matrix, sigma_s0, alpha_val);

[paths_base, weights_base, extra_base] = solve_rmpr_once_from_cost( ...
    t_idx, dist_matrix, W0, result3, Sources, target_sat, L, M_bits, alpha_val, T_max0, n_redundancy);

fprintf('\n===== 模块1测试：基线路径 =====\n');
for k = 1:numel(weights_base)
    fprintf('路径 %d: %s\n', k, mat2str(paths_base{k}));
    fprintf('权重 = %.6f\n', weights_base(k));
end

NumNodes = size(dist_matrix, 1);

% 背景节点：年轻
rng(10);
node_ages_special = 1.5 + (3.0 - 1.5) .* rand(1, NumNodes);

% 手动设置老化热点（注意：路径节点编号是0-based，MATLAB索引要+1）
hotspot_nodes_0based = [767 727 687 647 607 567 529 489 449 447];
hotspot_idx = hotspot_nodes_0based + 1;

% 热点节点：老化
node_ages_special(hotspot_idx) = 13.2 + (14.0 - 13.2) .* rand(1, numel(hotspot_idx));

fprintf('\n===== 模块2测试：老化热点设置 =====\n');
fprintf('热点节点(0-based): %s\n', mat2str(hotspot_nodes_0based));
fprintf('热点节点平均年龄 = %.4f\n', mean(node_ages_special(hotspot_idx)));

bg_idx = setdiff(1:NumNodes, hotspot_idx);
fprintf('背景节点平均年龄 = %.4f\n', mean(node_ages_special(bg_idx)));


omega_list = [0, 0.4,0.8];

for ii = 1:numel(omega_list)
    omega = omega_list(ii);

     [W_node, T_max_node, P_node_ref_node] = ...
     build_weighted_cost_matrix_node_addon(dist_matrix, sigma_s0, node_ages_special, alpha_val, omega);


    [paths_now, weights_now, extra_now] = solve_rmpr_once_from_cost( ...
        t_idx, dist_matrix, W_node, result3, Sources, target_sat, L, M_bits, alpha_val, T_max_node, n_redundancy);

    fprintf('\n=============================================\n');
    fprintf('omega = %.2f\n', omega);
    fprintf('=============================================\n');

    weighted_hot_hits = 0;

    for k = 1:numel(weights_now)
        p = paths_now{k};
        p_mid = p(2:end-1);
        hot_hits = sum(ismember(p_mid, hotspot_nodes_0based));

        fprintf('路径 %d: %s\n', k, mat2str(paths_now{k}));
        fprintf('权重 = %.6f\n', weights_now(k));
        fprintf('命中热点节点数 = %d\n', hot_hits);

        weighted_hot_hits = weighted_hot_hits + weights_now(k) * hot_hits;
    end

    fprintf('加权热点命中数 = %.6f\n', weighted_hot_hits);
end

