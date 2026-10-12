% 输入 图。S和D
% 输出 paths路径数组、valid_L找到的实际条数

function [paths, valid_L] = search_L_shortest_edge_disjoint_paths(original_graph, S, D, L)
    temp_graph = original_graph;
    paths = cell(1, L);
    valid_L = 0;
    
    for i = 1:L
        % 1. 找当前图的最短路径
        [dist, current_path] = dijkstra(temp_graph, S, D);
        
        % 2. 校验1：路径非空+距离有效
        if isempty(current_path) || dist == inf
            fprintf('仅找到%d条有效边不相交路径（后续无可用链路）\n', valid_L);
            break;
        end
        
        % 3. 校验2：路径的所有边必须存在于当前图（非inf）
        is_path_valid = true;
        for k = 1:length(current_path)-1
            u = current_path(k);
            v = current_path(k+1);
            % 检查当前图中u→v是否为有效边（非inf）
            if temp_graph(u+1, v+1) == inf
                is_path_valid = false;
                break;
            end
        end
        if ~is_path_valid
            fprintf('路径%d包含无效边，已跳过\n', i);
            continue;
        end
        
        % 4. 保存有效路径
        valid_L = valid_L + 1;
        paths{valid_L} = current_path;
        
        % 5. 彻底移除当前路径的所有边（双向设为inf）
        for k = 1:length(current_path)-1
            u = current_path(k);
            v = current_path(k+1);
            temp_graph(u+1, v+1) = inf;
            temp_graph(v+1, u+1) = inf;
        end
    end
    
    % 清理空路径（只保留有效路径）
    paths = paths(1:valid_L);
end