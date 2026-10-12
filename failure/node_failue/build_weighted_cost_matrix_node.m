function [CostMatrix, T_max_val, R_ref_val, P_node_ref] = ...
    build_weighted_cost_matrix_node(dist_matrix, sigma_s0, node_ages, alpha, omega)

    % 输入:
    %   dist_matrix : 原始距离矩阵 (m)
    %   sigma_s0    : 激光抖动标准差
    %   node_ages   : 节点服役年龄数组
    %   alpha       : 时延-可靠性总权重系数
    %   omega       : 在“可靠性项内部”分给节点风险的权重
    %
    % 输出:
    %   CostMatrix  : 综合代价矩阵
    %   T_max_val   : 时延归一化基准
    %   R_ref_val   : 链路风险归一化基准（完全沿用旧函数）
    %   P_node_ref  : 节点风险归一化基准

    NumNodes = size(dist_matrix, 1);
    CostMatrix = inf(NumNodes, NumNodes);
    c = 299792458;

    % --- 设定硬切断阈值 ---
    min_dist_threshold = 600e3; % 600 km

    % =========================
    % A. 完全沿用旧函数的链路逻辑
    % =========================
    valid_indices = find(dist_matrix >= min_dist_threshold & dist_matrix < inf);

    if isempty(valid_indices)
        error('在 600km 范围外没有发现有效链路！请检查星座拓扑。');
    end

    d_all = dist_matrix(valid_indices);
    T_raw_all = d_all / c;

    pf_all = arrayfun(@(d) oisl_failure_prob(d, sigma_s0), d_all);
    pf_all = min(pf_all, 0.9999);
    R_raw_all = -log(1 - pf_all); %#ok<NASGU>

    T_max_val = max(T_raw_all);

    % 这里严格沿用旧函数
    p_fail_redline = 0.05;
    R_ref_val = -log(1 - p_fail_redline);

    % =========================
    % B. 节点失效模型参数
    % =========================
    slot_sec = 15;

    q_val = 1.1;
    beta_val = 2.5;
    eta_val = 15;
    accel_factor = 1e8;

    % 节点风险红线：12年节点在单时隙内的失效概率
    P_node_ref = calc_qWeibull_node_fail_prob( ...
        13, slot_sec, q_val, beta_val, eta_val, accel_factor);

    % 预计算所有节点风险
    P_node_all = zeros(1, NumNodes);
    for n = 1:NumNodes
        P_node_all(n) = calc_qWeibull_node_fail_prob( ...
            node_ages(n), slot_sec, q_val, beta_val, eta_val, accel_factor);
    end

    fprintf('\n========== 节点增强代价矩阵生成统计 ==========\n');
    fprintf('距离过滤: 已剔除所有 d < %.0f km 的链路\n', min_dist_threshold/1e3);
    fprintf('有效距离分布: Min = %.2f km, Max = %.2f km\n', min(d_all)/1e3, max(d_all)/1e3);
    fprintf('有效链路失效概率: Min = %.6f, Max = %.6f\n', min(pf_all), max(pf_all));
    fprintf('当前 alpha = %.2f, omega = %.2f\n', alpha, omega);
    fprintf('节点风险参考值 P_node_ref = %.6e\n', P_node_ref);
    fprintf('------------------------------------------------\n');

    % =========================
    % 【核心修复：只对老旧节点启用风险，年轻节点=0】
    % =========================
    % 规则：
    % 年龄 > 12 岁 → 老旧热点 → 启用风险
    % 年龄 ≤12 岁 → 年轻正常 → 节点风险强制=0 → 和基线完全一样
    % =========================
    for n = 1:NumNodes
        if node_ages(n) <= 13.0  
            P_node_all(n) = 0;  % ? 年轻节点：风险清零！
        end
    end

    % =========================
    % C. 执行加权合成
    % =========================
    for i = 1:NumNodes
        for j = 1:NumNodes
            d = dist_matrix(i,j);

            if d >= min_dist_threshold && d < inf
                % --- 1) 时延项：完全沿用旧函数 ---
                t_val = d / c;
                t_norm = t_val / T_max_val;

                % --- 2) 链路风险项：完全沿用旧函数 ---
                pf = oisl_failure_prob(d, sigma_s0);
                pf = min(pf, 0.9999);
                r_link_val = -log(1 - pf);
                r_link_norm = r_link_val / R_ref_val;

                % --- 3) 节点风险项：只对 >12岁热点生效 ---
                p_node_avg = (P_node_all(i) + P_node_all(j)) / 2;
                r_node_norm = p_node_avg / P_node_ref;

                % --- 4) 综合代价 ---
                % 年轻节点：r_node_norm=0 → 等价于 omega 无效 → 完全=基线
                risk_norm = (1 - omega) * r_link_norm + omega * r_node_norm;

                CostMatrix(i,j) = (1 - alpha) * t_norm + alpha * risk_norm;
                CostMatrix(i,j) = max(CostMatrix(i,j), 1e-8);

            elseif i == j
                CostMatrix(i,j) = 0;
            else
                CostMatrix(i,j) = inf;
            end
        end
    end

    fprintf('节点增强代价矩阵生成完毕（仅对>12岁热点启用风险）。\n');
end