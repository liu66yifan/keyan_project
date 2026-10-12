function [adj_matrix, has_valid_links] = create_diagram_dynamic(t_idx, result3, result, ground_station_node)

% 这个代码需要卫星数量改的时候需要改



% CREATE_DIAGRAM_DYNAMIC 按时刻动态构建「地面站+无人机接入卫星+星间拓扑」的邻接矩阵
% 输入参数：
%   t_idx - 时刻索引（1~total_times）
%   result3 - 无人机-接入卫星数据结构体（含sat_ids/dists_m）
%   result - 星间拓扑数据结构体（含sat_data）
%   ground_station_node - 地面站节点编号（0开始，默认0）
% 输出参数：
%   adj_matrix - 751×751邻接矩阵（0=地面站，1-750=卫星，权重=距离m，inf=无链路）
%   has_valid_links - 逻辑值，是否有有效接入卫星链路

%% 1. 初始化参数
sat_num = 1000;
num_nodes = 1001;                                % 地面站0 + 卫星1~750
adj_matrix = inf(num_nodes, num_nodes);         % 初始全为inf（无链路）
adj_matrix(1:num_nodes+1:end) = 0;              % 正确方式：对角线置0
has_valid_links = false;                        % 默认无有效链路

%% 2. 校验输入
if nargin < 4 || isempty(ground_station_node)
    ground_station_node = 0;
end
if ~isscalar(t_idx) || t_idx < 1 || t_idx ~= floor(t_idx)
    error('t_idx 必须是从1开始的正整数索引。');
end
if t_idx > numel(result3) || t_idx > numel(result)
    fprintf('时刻%d超出数据范围，返回空矩阵\n', t_idx);
    adj_matrix = [];
    return;
end

%% 3. 填充「无人机→接入卫星」链路（这里你用的是地面站0连接到接入卫星）
drone_sat_ids   = result3(t_idx).sat_ids;    % 卫星ID（期望1-750）
drone_sat_dists = result3(t_idx).dists_m;    % 距离（米）

% 允许 sat_ids / dists_m 是行向量或列向量
drone_sat_ids   = drone_sat_ids(:);
drone_sat_dists = drone_sat_dists(:);

if isempty(drone_sat_ids)
    fprintf('时刻%d：无无人机接入卫星链路\n', t_idx);
    return;
end

filled_cnt = 0;
gs_idx = ground_station_node + 1;            % 0开始节点 -> 矩阵索引+1

for i = 1:numel(drone_sat_ids)
    sat_node = drone_sat_ids(i);
    dist_val = drone_sat_dists(i);

    % 严格校验：必须是有限数值、卫星编号必须在1~750
    if isfinite(sat_node) && sat_node == floor(sat_node) && sat_node >= 1 && sat_node <= sat_num ...
            && isfinite(dist_val) && dist_val >= 0

        sat_idx = sat_node + 1;
        adj_matrix(gs_idx, sat_idx) = dist_val;
        adj_matrix(sat_idx, gs_idx) = dist_val;
        has_valid_links = true;
        filled_cnt = filled_cnt + 1;
    end
end

fprintf('时刻%d：已填充地面站→接入卫星链路%d条（输入%d条）\n', t_idx, filled_cnt, numel(drone_sat_ids));

if ~has_valid_links
    fprintf('时刻%d：输入存在，但没有任何有效接入链路（可能sat_id越界或距离为NaN/Inf）\n', t_idx);
    return;
end

%% 4. 填充「卫星间链路」
sat_topology = result(t_idx).sat_data;       % 期望 750×9

if size(sat_topology, 1) < sat_num || size(sat_topology, 2) < 9
    error('result(t_idx).sat_data 尺寸不符合预期：需要至少 750×9。');
end

for sat_id = 1:sat_num
    sat_info = sat_topology(sat_id, :);

    link1_sat   = sat_info(2); link1_dist   = sat_info(3);
    link2_sat   = sat_info(4); link2_dist   = sat_info(5);
    orbit1_sat  = sat_info(6); orbit1_dist  = sat_info(7);
    orbit2_sat  = sat_info(8); orbit2_dist  = sat_info(9);

    % 内部小函数：统一校验并写入双向边
    write_edge(sat_id, link1_sat,  link1_dist);
    if ~(isequal(link2_sat, link1_sat))
        write_edge(sat_id, link2_sat, link2_dist);
    end
    write_edge(sat_id, orbit1_sat, orbit1_dist);
    if ~(isequal(orbit2_sat, orbit1_sat))
        write_edge(sat_id, orbit2_sat, orbit2_dist);
    end
end

fprintf('时刻%d：已填充750颗卫星的星间拓扑链路\n', t_idx);

    function write_edge(src_sat, dst_sat, dist_val)
        % dst_sat/dist_val 允许 Inf/NaN 表示无链路
        if isfinite(dst_sat) && dst_sat == floor(dst_sat) && dst_sat >= 1 && dst_sat <= sat_num ...
                && isfinite(dist_val) && dist_val >= 0
            src_idx = src_sat + 1;
            dst_idx = dst_sat + 1;
            adj_matrix(src_idx, dst_idx) = dist_val;
            adj_matrix(dst_idx, src_idx) = dist_val;
        end
    end
end
