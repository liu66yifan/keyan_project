function [success, selected_idx, selected_source, selected_path, ...
          all_paths, valid_L, all_alive_flags] = ...
          run_backup_5path(route_matrix, adj_exec, Sources, target_sat, num_backup_paths)
% BACKUP 5-PATH ROUTING
% 先找多条最短边不相交路径，再按顺序切换：
% P1失败 -> P2 -> P3 -> ...
%
% 输入：
%   route_matrix      : 选路图（N×N）
%                       - 如果做论文里的 backup baseline，传 dist_matrix
%                       - 如果想测试加权版 backup，可传 curr_W
%   adj_exec          : 当前 Monte Carlo 下真实执行图（失效后的图）
%   Sources           : 当前时刻接入卫星集合（0开始编号）
%   target_sat        : 目标节点（0开始编号）
%   num_backup_paths  : 备份路径条数，默认 5
%
% 输出：
%   success           : 是否成功（1/0）
%   selected_idx      : 最终成功的是第几条路径；失败则为 0
%   selected_source   : 最终成功路径的源节点；失败则为空
%   selected_path     : 最终成功路径；失败则为空
%   all_paths         : 找到的所有备份路径（cell）
%   valid_L           : 实际找到的有效路径数
%   all_alive_flags   : 每条路径在 adj_exec 上是否存活（0/1）
%
% 依赖：
%   search_MultiSource_disjoint_paths(...)
%   is_path_alive(...) 或本文件内 local 版本
%
% 节点编号约定：
%   Sources / target_sat / path 都是 0 开始编号

    if nargin < 4 || isempty(num_backup_paths)
        num_backup_paths = 4;
    end

    success = 0;
    selected_idx = 0;
    selected_source = [];
    selected_path = [];
    all_paths = {};
    valid_L = 0;
    all_alive_flags = [];

    if isempty(Sources)
        return;
    end

    % ---------- 1) 找 num_backup_paths 条最短边不相交路径 ----------
    try
        [all_paths, valid_L] = search_MultiSource_disjoint_paths( ...
            route_matrix, Sources, target_sat, num_backup_paths);
    catch
        % 如果外部函数报错，直接返回失败
        all_paths = {};
        valid_L = 0;
        return;
    end

    if valid_L == 0 || isempty(all_paths)
        return;
    end

    % ---------- 2) 在真实执行图上按顺序尝试 ----------
    all_alive_flags = zeros(valid_L, 1);

    for k = 1:valid_L
        curr_path = all_paths{k};

        if isempty(curr_path)
            all_alive_flags(k) = 0;
            continue;
        end

        alive_flag = is_path_alive_local(curr_path, adj_exec);
        all_alive_flags(k) = alive_flag;

        if alive_flag
            success = 1;
            selected_idx = k;
            selected_path = curr_path;
            selected_source = curr_path(1);
            return;
        end
    end
end

function alive = is_path_alive_local(p, A)
% 本地路径验活函数
% p 为 0 开始编号路径
    alive = true;

    if isempty(p) || numel(p) < 2
        alive = false;
        return;
    end

    for k = 1:numel(p)-1
        if ~isfinite(A(p(k)+1, p(k+1)+1))
            alive = false;
            return;
        end
    end
end
