%% 111111得到第0个时刻的连接关系result_static_temp
% 这个代码是最原始的生成图！！！！！！！！！！！
result_static_temp = result_static(1);
%% 222222得到第0个时刻的图adj_matrices_static_temp
t_idx = 1;                 % 你要的时刻索引（改成 1~length(result3) 中你需要的那个）
ground_station_node = 0;   % 地面站节点（0开始）

%% ============ 2) 加载数据（你已有的）===========
load('无人机-卫星速率数据_result3.mat', 'result3');
% 你说你现在只有 result_static_temp（1x1 struct），这里假设它已在工作区
% 如果它在文件里，就用：load('result_static_temp.mat','result_static_temp');

%% ============ 3) 构造一个“伪 result_static”以兼容 create_diagram_dynamic ============
% 关键点：create_diagram_dynamic(t_idx, result3, result_static, ...) 会访问 result_static(t_idx)
% 但你只有 1 个时刻，所以我们把它复制成长度 >= t_idx 的结构体数组
result_static_for_build = repmat(result_static_temp, 1, max(1, t_idx));

%% ============ 4) 调用构图函数生成 1 个时刻邻接矩阵 ============
[adj_matrices_static_temp, has_valid_links] = create_diagram_dynamic( ...
    t_idx, result3, result_static_for_build, ground_station_node);

%% ============ 5) 基本校验与保存 ============
if isempty(adj_matrices_static_temp)
    error('adj_matrices_static_temp 为空：create_diagram_dynamic 未生成有效邻接矩阵。');
end
fprintf('生成成功：adj_matrices_static_temp 尺寸 = %d×%d, has_valid_links=%d\n', ...
    size(adj_matrices_static_temp,1), size(adj_matrices_static_temp,2), has_valid_links);



