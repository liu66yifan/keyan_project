function in = rmpr_build_waterfill_inputs_verbose(t_idx, adj_matrix, result3, S, D, L_paths, M_bits, verbose)
% rmpr_build_waterfill_inputs_verbose
% Step 1：生成 solve_routing_optimization 的输入 q 和 gamma，并支持打印中间过程。
%
% 输入多一个:
%   verbose : true/false，是否打印过程（默认true）
%
% 输出 in 同 rmpr_build_waterfill_inputs

    if nargin < 8 || isempty(verbose)
        verbose = true;
    end
    if nargin < 7 || isempty(M_bits)
        %模拟1 Mb：M_bits = 1e6    1MB：M_bits = 8e6   10KB：M_bits = 10*1024*8 = 81920
        M_bits = 1e3;   % 1 Mb
    end

    % --------- 0) 基本校验 ---------
    if t_idx < 1 || t_idx ~= floor(t_idx)
        error('t_idx 必须是从1开始的整数。');
    end
    if isempty(adj_matrix) || any(size(adj_matrix) ~= [751 751])
        error('adj_matrix 必须是 751x751。');
    end
    if t_idx > numel(result3)
        error('t_idx 超出 result3 长度。');
    end
    if ~(isscalar(S) && isscalar(D) && isscalar(L_paths))
        error('S, D, L_paths 必须是标量。');
    end

    if verbose
        fprintf('\n================ RMPR Build Inputs (t=%d) ================\n', t_idx);
        fprintf('S=%d, D=%d, L_paths=%d, M_bits=%g\n', S, D, L_paths, M_bits);
    end

    % --------- 1) 找 L 条边不相交路径 ---------
    [paths, valid_L] = search_L_shortest_edge_disjoint_paths(adj_matrix, S, D, L_paths);
    if valid_L == 0
        error('时刻%d：未找到任何 S->D 路径', t_idx);
    end

    if verbose
        fprintf('\n[Step 1] Edge-disjoint shortest paths:\n');
        fprintf('目标L=%d，实际valid_L=%d\n', L_paths, valid_L);
    end

    % --------- 2) 计算每条路径距离与 q_i ---------
    c = 3e8; % m/s
    path_dist_m = zeros(1, valid_L);
    q_sec = zeros(1, valid_L);

    if verbose
        fprintf('\n[Step 2] Compute path distance and q_i (propagation delay):\n');
        fprintf('传播速度 c=%.3e m/s\n', c);
    end

    for i = 1:valid_L
        p = paths{i};
        dsum = 0;

        if verbose
            fprintf('\nPath %d: %s\n', i, path_to_string(p));
        end

        % 逐边距离
        if verbose
            fprintf('  Edge lengths: ');
        end

        for k = 1:numel(p)-1
            u = p(k);
            v = p(k+1);

            w = adj_matrix(u+1, v+1); % 节点0开始 -> 索引+1
            if ~isfinite(w)
                error('路径%d出现inf边：%d->%d', i, u, v);
            end
            dsum = dsum + w;

            if verbose
                fprintf('%d->%d=%.2f', u, v, w);
                if k < numel(p)-1
                    fprintf(', ');
                end
            end
        end

        if verbose
            fprintf('\n');
        end

        path_dist_m(i) = dsum;
        q_sec(i) = dsum / c;

        if verbose
            fprintf('  Total dist = %.2f m\n', path_dist_m(i));
            fprintf('  q_%d = dist/c = %.6e s\n', i, q_sec(i));
        end
    end

    % --------- 3) 从 result3 匹配 R_i（bps）并得到 gamma_i ---------
    sat_ids = result3(t_idx).sat_ids(:);
    rates_Mbps = result3(t_idx).rates_Mbps(:);

    if isempty(sat_ids) || isempty(rates_Mbps)
        error('时刻%d：result3中没有 sat_ids/rates_Mbps', t_idx);
    end

    if verbose
        fprintf('\n[Step 3] Match R_i from result3 (UAV->SAT rates):\n');
        fprintf('result3(t).sat_ids (Top list) = ');
        fprintf('%d ', sat_ids);
        fprintf('\n');
        fprintf('result3(t).rates_Mbps          = ');
        fprintf('%.3f ', rates_Mbps);
        fprintf('\n');
    end

    access_sat = nan(1, valid_L);
    R_bps = nan(1, valid_L);

    for i = 1:valid_L
        p = paths{i};
        if numel(p) < 2 || p(1) ~= 0
            error('路径%d不合法：必须从地面站0开始，且至少包含两个节点。', i);
        end

        access_sat(i) = p(2); % 0->接入卫星->...
        idx = find(sat_ids == access_sat(i), 1);

        if isempty(idx) || ~isfinite(rates_Mbps(idx))
            error(['路径%d 的接入卫星=%d 不在 result3(t_idx).sat_ids（Top-L UAV-SAT链路）里，' ...
                   '所以无法得到 R_i。\n建议：增大 result3 的Top-L，或先剪枝图只允许这些接入卫星。'], ...
                   i, access_sat(i));
        end

        R_bps(i) = rates_Mbps(idx) * 1e6;

        if verbose
            fprintf('\nPath %d access_sat=%d\n', i, access_sat(i));
            fprintf('  R_%d = %.3f Mbps = %.3e bps\n', i, R_bps(i)/1e6, R_bps(i));
        end
    end

    gamma_sec = M_bits ./ R_bps;

    if verbose
        fprintf('\n[Step 4] Compute gamma_i = M_bits / R_i:\n');
        for i = 1:valid_L
            fprintf('gamma_%d = %g / %.3e = %.6e s\n', i, M_bits, R_bps(i), gamma_sec(i));
        end
        fprintf('\n=============== Build Inputs DONE ===============\n');
    end

    % --------- 4) 输出结构体（注水输入） ---------
    in.t_idx = t_idx;
    in.S = S;
    in.D = D;
    in.L_paths = L_paths;

    in.paths = paths;
    in.valid_L = valid_L;

    in.path_dist_m = path_dist_m;
    in.q_sec = q_sec;

    in.access_sat = access_sat;
    in.R_bps = R_bps;

    in.M_bits = M_bits;
    in.c = c;
    in.gamma_sec = gamma_sec;
end

function s = path_to_string(p)
    if isempty(p)
        s = '';
        return;
    end
    s = sprintf('%d', p(1));
    for ii = 2:numel(p)
        s = [s, '---', sprintf('%d', p(ii))]; %#ok<AGROW>
    end
end
