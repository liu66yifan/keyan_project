function in = rmpr_build_waterfill_inputs_multi_source(t_idx, dist_matrix, cost_matrix, result3, Sources, D, L_paths, M_bits, alpha, T_max_val, verbose)
    % 输入增加了：
    %   dist_matrix : 原始物理距离矩阵 (用于算速率)
    %   cost_matrix : 加权后的代价矩阵 (用于寻路)
    %   alpha       : 权重系数
    %   T_max_val   : 之前归一化用到的 T_max (用于对齐斜率)

    if nargin < 11, verbose = true; end

    % --------- 1) 基于“代价矩阵”找多源不相交路径 ---------
    % 这样找出来的 paths 已经是避开风险的最优路径
    [paths, valid_L] = search_MultiSource_disjoint_paths(cost_matrix, Sources, D, L_paths);
    
    if valid_L == 0
        error('时刻%d：所有提供的起点均无法到达终点', t_idx);
    end

    % --------- 2) 计算注水参数：底座 (Floor) 和 斜率 (Slope) ---------
    c = 299792458;
    q_cost = zeros(1, valid_L);    % 对应文章1的 s_m (路径总代价)
    gamma_cost = zeros(1, valid_L); % 对应文章1的 e_m (传输产生的代价增量)
    access_sats = zeros(1, valid_L);

    sat_ids_in_res = result3(t_idx).sat_ids(:);
    rates_Mbps_in_res = result3(t_idx).rates_Mbps(:);

    for i = 1:valid_L
        p = paths{i};
        access_sats(i) = p(1);
        
        % A. 计算底座 q_cost (s_m): 直接累加路径上的“综合代价”
        % 这已经包含了归一化后的物理时延和可靠性惩罚
        path_sum_cost = 0;
        for k = 1:numel(p)-1
            path_sum_cost = path_sum_cost + cost_matrix(p(k)+1, p(k+1)+1);
        end
        q_cost(i) = path_sum_cost;

        % B. 获取接入速率 R_i
        idx = find(sat_ids_in_res == access_sats(i), 1);
        R_bps = rates_Mbps_in_res(idx) * 1e6;

        % C. 计算斜率 gamma_cost (e_m): 
        % 传输时延 = M_bits / R_bps。
        % 根据文章1，斜率也要映射到代价空间。
        % 因为我们的代价矩阵里时延项是 (1-alpha) * (T / T_max_val)
        % 所以传输产生的代价增量 = (1-alpha) * (传输时间 / T_max_val)
        trans_time = M_bits / R_bps;
        gamma_cost(i) = (1 - alpha) * (trans_time / T_max_val);
    end

    % --------- 3) 封装输出 ---------
    in.paths = paths;
    in.valid_L = valid_L;
    in.q_sec = q_cost;       % 此时 q 代表的是 Cost Floor
    in.gamma_sec = gamma_cost; % 此时 gamma 代表的是 Cost Slope
    in.access_sats = access_sats;
    in.M_bits = M_bits;
end