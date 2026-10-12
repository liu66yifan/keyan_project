function [node_ages_special, hotspot_nodes] = build_hotspot_aging_from_paths( ...
    NumNodes, active_paths, active_weights, target_node)

    rng(7);

    % 1) 背景节点：年轻
    node_ages_special = 1.5 + (3.0 - 1.5) .* rand(1, NumNodes);

    % 2) 收集“无节点权重”时真正承担业务的活跃路径中的中间节点
    core_nodes = [];

    for k = 1:length(active_paths)
        if active_weights(k) < 0.05
            continue;
        end

        p = active_paths{k};
        p = p(:)';

        % 去掉起点和终点附近的节点，避免把不可避免的端点也老化
        if length(p) > 6
            p_mid = p(3:end-2);
        else
            p_mid = p(2:end-1);
        end

        % 只保留卫星节点编号
        p_mid = p_mid(p_mid >= 1 & p_mid <= NumNodes);

        % 去掉目标节点
        p_mid(p_mid == target_node) = [];

        core_nodes = [core_nodes, p_mid];
    end

    core_nodes = unique(core_nodes);

    % 3) 只老化其中一部分，制造“热点走廊”
    if isempty(core_nodes)
        warning('未找到可用于构造热点的活跃中继节点，返回纯背景年龄。');
        hotspot_nodes = [];
        return;
    end

    num_hot = max(4, round(0.45 * length(core_nodes)));
    idx = randperm(length(core_nodes), num_hot);
    hotspot_nodes = sort(core_nodes(idx));

    % 4) 老化热点节点：高年龄
    node_ages_special(hotspot_nodes) = 13.2 + (14.0 - 13.2) .* rand(1, length(hotspot_nodes));
end
