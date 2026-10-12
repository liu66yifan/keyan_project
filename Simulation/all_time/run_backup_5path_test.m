%% ===== 测试：run_backup_5path =====
clc;
t_idx = 1;
target_sat = 402;          % 目标节点（0开始编号）
current_sigma_s0 = 2.4;    % 当前扰动强度
num_backup_paths = 4;      % backup 5-path

fprintf('\n================ Backup 5-path 测试结果 ================\n');
fprintf('时刻 t_idx = %d\n', t_idx);
fprintf('目标节点 target_sat = %d\n', target_sat);
fprintf('候选 Sources = %s\n', mat2str(Sources(:)'));
fprintf('sigma_s0 = %.2f\n', current_sigma_s0);
fprintf('num_backup_paths = %d\n', num_backup_paths);

[success_bk, selected_idx_bk, selected_source_bk, selected_path_bk, ...
    all_paths_bk, valid_L_bk, all_alive_flags_bk] = ...
    run_backup_5path(dist_matrix, adj_exec, Sources, target_sat, num_backup_paths);

fprintf('\n--- 找到的备份路径 ---\n');
if valid_L_bk == 0
    fprintf('没有找到任何有效备份路径。\n');
else
    for k = 1:valid_L_bk
        fprintf('路径 %d: %s\n', k, mat2str(all_paths_bk{k}));
        fprintf('        是否存活 = %d\n', all_alive_flags_bk(k));
    end
end

fprintf('\n--- 最终 backup 结果 ---\n');
if success_bk == 1
    fprintf('success = 1\n');
    fprintf('成功使用第 %d 条路径\n', selected_idx_bk);
    fprintf('选中的源节点 = %d\n', selected_source_bk);
    fprintf('选中的路径   = %s\n', mat2str(selected_path_bk));
else
    fprintf('success = 0\n');
    fprintf('所有备份路径均失败\n');
end

fprintf('========================================================\n');
