%适配多起点逻辑，并计算各自的 

function [paths, valid_L] = search_MultiSource_disjoint_paths(original_graph, Sources, D, L)
    % 输入：
    %   original_graph: 距离矩阵
    %   Sources: 起点列表 (例如 [101, 102, 103, 104])
    %   D: 终点
    %   L: 期望路径总数
    
    temp_graph = original_graph;
    % 屏蔽地面站节点逻辑
    temp_graph(1, :) = inf;
    temp_graph(:, 1) = inf;
    temp_graph(1, 1) = 0;
    
    paths = cell(1, L);
    valid_L = 0;
    
    % 遍历起点列表，每个起点尝试找一条到达D的路径
    for i = 1:min(length(Sources), L)
        S = Sources(i);
        
        % 在被“烧过”的图中找当前起点的最短路径
        [dist, current_path] = dijkstra(temp_graph, S, D);
        
        if isempty(current_path) || dist == inf
            continue; % 当前起点无法到达D，跳过尝试下一个起点
        end
        
        % 保存路径
        valid_L = valid_L + 1;
        paths{valid_L} = current_path;
        
        % “烧边法”：移除该路径上所有的边，确保后续路径不重叠
        for k = 1:length(current_path)-1
            u = current_path(k);
            v = current_path(k+1);
            temp_graph(u+1, v+1) = inf;
            temp_graph(v+1, u+1) = inf;
        end
    end
    paths = paths(1:valid_L);
end