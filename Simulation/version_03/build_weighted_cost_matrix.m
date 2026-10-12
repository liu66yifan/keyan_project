% 生成权重矩阵（时延+可靠性）- 已加入 600km 硬截断逻辑
function [CostMatrix, T_max_val, R_ref_val] = build_weighted_cost_matrix(dist_matrix, sigma_s0, alpha)
    % 输入:
    %   dist_matrix : 原始距离矩阵 (m)
    %   sigma_s0    : 激光抖动标准差
    %   alpha       : 权重系数 (0-1)
    
    NumNodes = size(dist_matrix, 1);
    CostMatrix = inf(NumNodes, NumNodes);
    c = 299792458; 
    
    % --- 设定硬切断阈值 ---
    min_dist_threshold = 600e3; % 600 km

    % --- Step 1: 预扫描全图 (过滤掉过近的链路) ---
    % 修改逻辑：只有在 (600km, inf) 范围内的链路才被视为“有效”
    valid_indices = find(dist_matrix >= min_dist_threshold & dist_matrix < inf);
    
    if isempty(valid_indices)
        error('在 600km 范围外没有发现有效链路！请检查星座拓扑。');
    end
    
    d_all = dist_matrix(valid_indices);
    T_raw_all = d_all / c; 
    
    % 计算有效链路的失效概率
    pf_all = arrayfun(@(d) oisl_failure_prob(d, sigma_s0), d_all);
    R_raw_all = -log(1 - min(pf_all, 0.9999));

    % 提取基准
    T_max_val = max(T_raw_all); 
    p_fail_redline = 0.05; 
    R_ref_val = -log(1 - p_fail_redline); 
    
    fprintf('\n========== 物理层代价矩阵生成统计 (含 600km 截断) ==========\n');
    fprintf('距离过滤: 已剔除所有 d < %.0f km 的链路\n', min_dist_threshold/1e3);
    fprintf('有效距离分布: Min = %.2f km, Max = %.2f km\n', min(d_all)/1e3, max(d_all)/1e3);
    fprintf('有效链路失效概率: Min = %.6f, Max = %.6f\n', min(pf_all), max(pf_all));
    fprintf('当前权重 Alpha: %.2f\n', alpha);
    fprintf('----------------------------------------------------------\n');

    % --- Step 2: 执行加权合成 ---
    for i = 1:NumNodes
        for j = 1:NumNodes
            d = dist_matrix(i,j);
            
            % --- 核心修改点：加入距离下限判定 ---
            if d >= min_dist_threshold && d < inf
                % 正常的归一化与加权逻辑
                t_val = d / c;
                pf = oisl_failure_prob(d, sigma_s0);
                r_val = -log(1 - min(pf, 0.9999));
                
                t_norm = t_val / T_max_val;
                r_norm = r_val / R_ref_val;
                
                CostMatrix(i,j) = (1 - alpha) * t_norm + alpha * r_norm;
                CostMatrix(i,j) = max(CostMatrix(i,j), 1e-8);
                
            elseif i == j
                CostMatrix(i,j) = 0; 
            else
                % 距离小于 600km 或为 inf 的链路，代价均为 inf
                CostMatrix(i,j) = inf;
            end
        end
    end
    fprintf('代价矩阵生成完毕（近距离链路已断开）。\n');
end
