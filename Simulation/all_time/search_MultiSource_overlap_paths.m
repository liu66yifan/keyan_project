function [paths, valid_L] = search_MultiSource_overlap_paths(cost_matrix, Sources, D, L_paths)
% 多源可相交路径搜索
% 对每个源节点 independently 找一条到目标的最短路径
% 不强制路径之间不相交
%
% 输入：
%   cost_matrix : 加权代价矩阵
%   Sources     : 多个源节点（0开始编号）
%   D           : 目标节点（0开始编号）
%   L_paths     : 最多保留的路径数
%
% 输出：
%   paths       : 找到的路径 cell
%   valid_L     : 实际有效路径数

    paths = {};
    path_costs = [];

    if isempty(Sources) || L_paths <= 0
        valid_L = 0;
        return;
    end

    % 对每个起点各找一条最短路
    for s_idx = 1:numel(Sources)
        src = Sources(s_idx);

        try
            [path_cost, path_nodes] = dijkstra(cost_matrix, src, D);
        catch
            path_cost = inf;
            path_nodes = [];
        end

        if ~isempty(path_nodes) && isfinite(path_cost)
            paths{end+1} = path_nodes; %#ok<AGROW>
            path_costs(end+1) = path_cost; %#ok<AGROW>
        end
    end

    % 如果一条都没有
    if isempty(paths)
        valid_L = 0;
        return;
    end

    % 如果有效路径超过 L_paths，就按代价从小到大排序后截取
    [~, sort_idx] = sort(path_costs, 'ascend');
    paths = paths(sort_idx);

    valid_L = min(numel(paths), L_paths);
    paths = paths(1:valid_L);
end
