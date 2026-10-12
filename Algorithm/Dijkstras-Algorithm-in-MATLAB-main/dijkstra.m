function [target_distance, target_path] = dijkstra(graph, start_node, dest_node)
% graph: N×N，索引1对应节点0；inf=无边；对角线=0
% start_node / dest_node: 0开始

    num_nodes = size(graph, 1);

    % 基本校验
    if start_node < 0 || start_node >= num_nodes || dest_node < 0 || dest_node >= num_nodes
        error('start_node/dest_node 越界：应在[0, %d]内', num_nodes-1);
    end

    dist = inf(1, num_nodes);
    dist(start_node + 1) = 0;

    prev = -1 * ones(1, num_nodes);     % 用-1表示无前驱（关键修复点）
    visited = false(1, num_nodes);

    for iter = 1:num_nodes
        % 选取未访问且dist最小的节点
        unvisited_idx = find(~visited);
        [min_val, rel_idx] = min(dist(unvisited_idx));
        if isinf(min_val)
            break; % 剩余点不可达
        end
        u_idx = unvisited_idx(rel_idx);  % 1开始索引
        u = u_idx - 1;                   % 转回0开始节点

        visited(u_idx) = true;

        % 提前结束
        if u == dest_node
            break;
        end

        % 松弛所有邻居 v
        for v = 0:(num_nodes-1)
            if ~visited(v+1) && isfinite(graph(u+1, v+1))
                alt = dist(u+1) + graph(u+1, v+1);
                if alt < dist(v+1)
                    dist(v+1) = alt;
                    prev(v+1) = u;
                end
            end
        end
    end

    target_distance = dist(dest_node + 1);

    % 回溯路径
    if isinf(target_distance)
        target_path = [];
        return;
    end

    target_path = dest_node;
    cur = dest_node;
    while cur ~= start_node
        cur = prev(cur + 1);
        if cur == -1
            target_path = [];
            target_distance = inf;
            return;
        end
        target_path = [cur, target_path]; %#ok<AGROW>
    end
end
