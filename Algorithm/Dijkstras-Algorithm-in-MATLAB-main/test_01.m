% 前面算法部分生成的 final_valid_graphs图 然后逐个将邻接矩阵传入你的算法函数
% 无需修改写好的search_L_shortest_edge_disjoint_paths和dijkstra函数，本函数是“循环调用 + 结果存储” 的外层代码

% ====================== 步骤1：设置核心参数  ======================
S = 0;  % 起始节点：地面站（固定为0，和你的函数逻辑一致）
D = 22; % 目标节点：卫星25（0开始编号，你可改为1~25中的任意卫星）
L = 4;  % 期望查找的边不相交最短路径数量（你可自定义，如3、10等）
graph_count = length(final_valid_graphs);  % 有效图的总数（对应final_valid_graphs的长度）

fprintf('? 准备调用路径搜索算法，共处理%d个时刻的邻接矩阵\n', graph_count);
fprintf('? 起始节点（地面站）：%d，目标节点（卫星）：%d，期望路径数：%d\n', S, D, L);

% ====================== 步骤2：初始化结果存储（保存每个图的路径结果） ======================
% 每个图对应一组paths和valid_L，用cell数组存储
all_graph_paths = cell(graph_count, 1);    % 存储每个图的有效边不相交路径
all_graph_valid_L = zeros(graph_count, 1); % 存储每个图实际找到的有效路径数

% ====================== 步骤3：循环遍历每个图，调用你的算法函数 ======================
for i = 1:graph_count
    % 1. 取出当前时刻的邻接矩阵（对应你的original_graph输入）
    current_graph = final_valid_graphs{i};
    
    % 2. 调用你的路径搜索算法，传入参数
    [current_paths, current_valid_L] = search_L_shortest_edge_disjoint_paths(current_graph, S, D, L);
    
    % 3. 保存当前图的结果
    all_graph_paths{i} = current_paths;
    all_graph_valid_L(i) = current_valid_L;
    
    % 4. 打印进度与结果，方便你查看
    fprintf('\n第%d/%d个图处理完成：\n', i, graph_count);
    fprintf('   实际找到%d条有效边不相交最短路径（期望%d条）\n', current_valid_L, L);
end

fprintf('\n?所有图的路径搜索完成！\n');









