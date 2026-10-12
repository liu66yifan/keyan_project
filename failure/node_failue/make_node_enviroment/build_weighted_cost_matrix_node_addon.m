function [CostMatrix, T_max_val, P_node_ref] = ...
    build_weighted_cost_matrix_node_addon(dist_matrix, sigma_s0, node_ages, alpha, omega)

    % 1) 先拿旧基线
    [W_old, T_max_val, R_ref_old] = build_weighted_cost_matrix(dist_matrix, sigma_s0, alpha); %#ok<ASGLU>

    NumNodes = size(dist_matrix, 1);
    CostMatrix = W_old;
    slot_sec = 15;

    % q-Weibull 参数
    q_val = 1.1;
    beta_val = 2.5;
    eta_val = 15;
    accel_factor = 100000000;

    % 节点风险参考值
    P_node_ref = calc_qWeibull_node_fail_prob(12, slot_sec, q_val, beta_val, eta_val, accel_factor);

    % 所有节点的失效概率
    P_node_all = zeros(1, NumNodes);
    for n = 1:NumNodes
        P_node_all(n) = calc_qWeibull_node_fail_prob( ...
            node_ages(n), slot_sec, q_val, beta_val, eta_val, accel_factor);
    end

    % 2) 只叠加节点惩罚
    for i = 1:NumNodes
        for j = 1:NumNodes
            if i ~= j && isfinite(dist_matrix(i,j)) && isfinite(W_old(i,j))
                p_node_avg = (P_node_all(i) + P_node_all(j)) / 2;
                r_node_norm = p_node_avg / P_node_ref;

                % 这里只加节点项
                CostMatrix(i,j) = W_old(i,j) + alpha * omega * r_node_norm;
            end
        end
    end
end
