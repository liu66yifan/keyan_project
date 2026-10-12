function in = rmpr_build_waterfill_inputs_multi_source_overlap( ...
    t_idx, dist_matrix, cost_matrix, result3, Sources, D, L_paths, M_bits, alpha, T_max_val, verbose)
% 多源可相交版本：
% 每个起始卫星各自独立找一条到目标的最短路径，允许路径之间相交

    if nargin < 11
        verbose = true;
    end

    % --------- 1) 基于“代价矩阵”找多源可相交路径 ---------
    [paths, valid_L] = search_MultiSource_overlap_paths(cost_matrix, Sources, D, L_paths);

    if valid_L == 0
        error('时刻%d：所有提供的起点均无法到达终点', t_idx);
    end

    % --------- 2) 计算注水参数：底座 q_cost 和斜率 gamma_cost ---------
    q_cost = zeros(1, valid_L);
    gamma_cost = zeros(1, valid_L);
    access_sats = zeros(1, valid_L);

    sat_ids_in_res = result3(t_idx).sat_ids(:);
    rates_Mbps_in_res = result3(t_idx).rates_Mbps(:);

    for i = 1:valid_L
        p = paths{i};
        access_sats(i) = p(1);

        % A. 路径总代价（基于 cost_matrix）
        path_sum_cost = 0;
        for k = 1:numel(p)-1
            path_sum_cost = path_sum_cost + cost_matrix(p(k)+1, p(k+1)+1);
        end
        q_cost(i) = path_sum_cost;

        % B. 接入速率
        idx = find(sat_ids_in_res == access_sats(i), 1);
        if isempty(idx)
            error('时刻%d：接入星 %d 在 result3(t_idx).sat_ids 中找不到对应速率', ...
                t_idx, access_sats(i));
        end
        R_bps = rates_Mbps_in_res(idx) * 1e6;

        % C. 斜率项
        trans_time = M_bits / R_bps;
        gamma_cost(i) = (1 - alpha) * (trans_time / T_max_val);
    end

    % --------- 3) 封装输出 ---------
    in.paths = paths;
    in.valid_L = valid_L;
    in.q_sec = q_cost;
    in.gamma_sec = gamma_cost;
    in.access_sats = access_sats;
    in.M_bits = M_bits;
end
