function [success, selected_source, selected_path, selected_distance, all_candidate_paths, all_candidate_distances] = ...
    run_single_path(dist_matrix, adj_exec, Sources, target_sat)
% 单一路径基线：
% 只基于时延/距离图 dist_matrix（不是加权代价图）进行选路。
%
% 输入：
%   dist_matrix : 静态距离图 / 时延图（N×N）
%   adj_exec    : 当前 Monte Carlo 下的真实执行图（失效后的图）
%   Sources     : 当前时刻可接入卫星集合（0开始编号）
%   target_sat  : 目标节点（0开始编号）
%
% 输出：
%   success                  : 1=成功, 0=失败
%   selected_source          : 最终选中的源节点（0开始编号）
%   selected_path            : 最终选中的最短路径（0开始编号路径序列）
%   selected_distance        : 最终选中路径的距离/代价
%   all_candidate_paths      : 每个源对应求出的候选最短路径（cell）
%   all_candidate_distances  : 每个源对应的候选路径总代价

    success = 0;
    selected_source = [];
    selected_path = [];
    selected_distance = inf;

    if isempty(Sources)
        all_candidate_paths = {};
        all_candidate_distances = [];
        return;
    end

    num_sources = numel(Sources);
    all_candidate_paths = cell(num_sources, 1);
    all_candidate_distances = inf(num_sources, 1);

    best_distance = inf;
    best_path = [];
    best_source = [];

    % 对每个候选接入卫星都跑一次最短路径
    for s_idx = 1:num_sources
        src = Sources(s_idx);

        try
            [path_distance, path_nodes] = dijkstra(dist_matrix, src, target_sat);
        catch
            path_distance = inf;
            path_nodes = [];
        end

        all_candidate_paths{s_idx} = path_nodes;
        all_candidate_distances(s_idx) = path_distance;

        % 选取距离最小的那条路径作为单路径方案
        if ~isempty(path_nodes) && isfinite(path_distance)
            if path_distance < best_distance
                best_distance = path_distance;
                best_path = path_nodes;
                best_source = src;
            end
        end
    end

    % 如果没有任何可达路径
    if isempty(best_path)
        return;
    end

    selected_source = best_source;
    selected_path = best_path;
    selected_distance = best_distance;

    % 在真实执行图中检查路径是否存活
    if is_path_alive_local(selected_path, adj_exec)
        success = 1;
    end
end


function alive = is_path_alive_local(path_nodes, graph_exec)
% 判断一条路径在真实执行图中是否存活
% path_nodes 为 0 开始编号

    alive = true;

    if isempty(path_nodes) || numel(path_nodes) < 2
        alive = false;
        return;
    end

    for k = 1:numel(path_nodes)-1
        u = path_nodes(k);
        v = path_nodes(k+1);

        if ~isfinite(graph_exec(u+1, v+1))
            alive = false;
            return;
        end
    end
end
