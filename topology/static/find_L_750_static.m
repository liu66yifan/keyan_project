clc; 
%% 遍历12个时刻，调用create_diagram_dynamic生成邻接矩阵，找L条边不相交路径（无adj_matrices.mat依赖）
% 输出
%% ====================== 1. 参数配置 ======================
ground_station_node = 0;    % 地面站节点(0开始)
S = 460;                      % 起点：地面站
L = 4;                      % 需要的边不相交路径条数
total_time_steps = 1;       % 要遍历的总时刻数（1-12）

% 目标D选择策略：
% 1 = 使用第1个接入卫星作为D（默认）
% 2 = 手动指定D（你改成某个卫星ID，例如 D = 32）
D_mode = 2; % 2是手动
D_manual = 402; %目的卫星设成402

%% ====================== 2. 加载数据 ======================
try
    load('无人机-卫星速率数据_result3.mat', 'result3');
    load('ISL_AllSat_Static_Topology_Result.mat', 'result_static');
    fprintf('? 数据加载成功\n');
catch ME
    error('? 请确认数据文件在当前路径！原始错误：%s', ME.message);
end

% 校验result3的时刻数是否足够
if length(result3) < total_time_steps
    warning('?? result3的时刻数（%d）少于设定的总时刻数（%d），将只处理前%d个时刻', length(result3), total_time_steps, length(result3));
    total_time_steps = length(result3);
end

%% ====================== 3. 预分配每个时刻的路径结果（可选，便于后续分析） ======================
all_path_results_static = struct('time_idx', [], 'D', [], 'valid_L', [], 'paths', [], 'total_lens', []);
all_path_results_static(total_time_steps) = all_path_results_static(1);

%% ====================== 4. 外层遍历12个时刻 ======================
for time_idx = 1:total_time_steps
    fprintf('\n=====================================================\n');
    fprintf('? 正在处理第%d/%d个时刻\n', time_idx, total_time_steps);
    fprintf('=====================================================\n');

    %% 步骤1：调用create_diagram_dynamic生成当前时刻的751×751邻接矩阵
    % 直接调用构图函数，无需加载adj_matrices.mat
    [adj_matrix, has_valid_links] = create_diagram_dynamic(time_idx, result3, result_static, ground_station_node);

    % 若无有效链路，跳过该时刻
    if ~has_valid_links || isempty(adj_matrix)
        fprintf('??  时刻%d：无有效链路（地面站→接入卫星），跳过\n', time_idx);
        all_path_results_static(time_idx).time_idx = time_idx;
        all_path_results_static(time_idx).valid_L = 0;
        continue;
    end
    fprintf('? 时刻%d：邻接矩阵生成成功（751×751）\n', time_idx);

    %% 步骤2：选择目标节点D（保持原有逻辑）
    drone_sat_ids = result3(time_idx).sat_ids(:);
    drone_sat_ids = drone_sat_ids(isfinite(drone_sat_ids));

    if isempty(drone_sat_ids)
        fprintf('??  时刻%d：result3.sat_ids为空，跳过该时刻\n', time_idx);
        all_path_results_static(time_idx).time_idx = time_idx;
        all_path_results_static(time_idx).valid_L = 0;
        continue;
    end

    if D_mode == 1
        D = drone_sat_ids(1);
    else
        D = D_manual;
    end

    if ~(isfinite(D) && D == floor(D) && D >= 1 && D <= 750)
        fprintf('??  时刻%d：目标D=%g非法，跳过该时刻\n', time_idx, D);
        all_path_results_static(time_idx).time_idx = time_idx;
        all_path_results_static(time_idx).valid_L = 0;
        continue;
    end

    fprintf('路径搜索：S=%d (地面站), D=%d (卫星), 目标L=%d\n', S, D, L);

    %% 步骤3：搜索L条最短边不相交路径（保持原有逻辑）
    [paths, valid_L] = search_L_shortest_edge_disjoint_paths(adj_matrix, S, D, L);

    fprintf('\n=============================================\n');
    fprintf('边不相交最短路径搜索结果：目标L=%d，实际找到valid_L=%d\n', L, valid_L);

    if valid_L == 0
        fprintf('未找到任何可用路径（可能图不连通，或边被移除后无路）。\n');
        fprintf('=============================================\n');
        all_path_results_static(time_idx).time_idx = time_idx;
        all_path_results_static(time_idx).D = D;
        all_path_results_static(time_idx).valid_L = 0;
        all_path_results_static(time_idx).paths = [];
        all_path_results_static(time_idx).total_lens = [];
        continue;
    end

    %% 步骤4：打印每条路径（保持原有格式：路径字符串+总距离+逐边距离）
    total_lens = zeros(1, valid_L); % 记录每条路径总距离
    for i = 1:valid_L
        p = paths{i};  % 0开始节点序列

        % ---- 计算总距离 & 校验每条边 ----
        total_len = 0;
        edge_lens = zeros(1, max(0, numel(p)-1));
        edge_ok = true;

        for k = 1:numel(p)-1
            u = p(k);
            v = p(k+1);

            w = adj_matrix(u+1, v+1);   % 节点0开始→矩阵索引+1
            edge_lens(k) = w;

            if ~isfinite(w)
                edge_ok = false;
            else
                total_len = total_len + w;
            end
        end

        total_lens(i) = total_len; % 记录总距离

        % ---- 打印路径（保持原有格式）----
        fprintf('\nPath %d (len = %.2f m)%s: ', i, total_len, ternary(~edge_ok,' [WARNING: contains inf edge]',''));
        fprintf('%s\n', path_to_string(p));

        % ---- 打印逐边距离（保持原有格式）----
        fprintf('  Edge lengths: ');
        for k = 1:numel(p)-1
            fprintf('%d->%d=%.2f', p(k), p(k+1), edge_lens(k));
            if k < numel(p)-1
                fprintf(', ');
            end
        end
        fprintf('\n');
    end

    fprintf('\n=============================================\n');

    % 保存当前时刻结果（便于后续分析）
    all_path_results_static(time_idx).time_idx = time_idx;
    all_path_results_static(time_idx).D = D;
    all_path_results_static(time_idx).valid_L = valid_L;
    all_path_results_static(time_idx).paths = paths;
    all_path_results_static(time_idx).total_lens = total_lens;
end

%% ====================== 5. 12个时刻结果汇总（新增，便于快速查看） ======================
fprintf('\n=====================================================\n');
fprintf('? 12个时刻路径搜索汇总\n');
fprintf('=====================================================\n');
for time_idx = 1:total_time_steps
    res = all_path_results_static(time_idx);
    fprintf('时刻%d：D=%d | 有效路径数=%d | 路径总距离=', res.time_idx, res.D, res.valid_L);
    if res.valid_L > 0
        fprintf('%.2f', res.total_lens(1));
        for i = 2:res.valid_L
            fprintf(', %.2f', res.total_lens(i));
        end
    else
        fprintf('无');
    end
    fprintf(' (m)\n');
end
fprintf('=====================================================\n');

%% ====================== 原有辅助函数（完全保留，无修改） ======================
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

function out = ternary(cond, a, b)
    if cond
        out = a;
    else
        out = b;
    end
end