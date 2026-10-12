clc;
fprintf('\n================ Single-Path 测试结果 ================\n');
target_sat = 402;
fprintf('目标节点 target_sat = %d\n', target_sat);
fprintf('候选 Sources = %s\n', mat2str(Sources(:)'));

[success_sp, selected_source_sp, selected_path_sp, selected_distance_sp, ...
    all_candidate_paths_sp, all_candidate_distances_sp] = ...
    run_single_path(dist_matrix, adj_exec, Sources, target_sat);

fprintf('\n--- 每个候选接入卫星对应的最短路径（基于纯距离图）---\n');
for s_idx = 1:numel(Sources)
    fprintf('Source %d -> ', Sources(s_idx));

    if isempty(all_candidate_paths_sp{s_idx}) || ~isfinite(all_candidate_distances_sp(s_idx))
        fprintf('不可达\n');
    else
        fprintf('路径 = %s, 距离 = %.6f\n', ...
            mat2str(all_candidate_paths_sp{s_idx}), ...
            all_candidate_distances_sp(s_idx));
    end
end

fprintf('\n--- 最终 single-path 选择结果 ---\n');
if isempty(selected_path_sp)
    fprintf('没有找到任何可达的单路径。\n');
    fprintf('success = 0\n');
else
    fprintf('选中的源节点 = %d\n', selected_source_sp);
    fprintf('选中的路径   = %s\n', mat2str(selected_path_sp));
    fprintf('路径总距离   = %.6f\n', selected_distance_sp);
    fprintf('执行是否成功 = %d\n', success_sp);
end
fprintf('======================================================\n');
