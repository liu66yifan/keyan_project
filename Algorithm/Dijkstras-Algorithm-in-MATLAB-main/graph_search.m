% dijkstra算法的测试案例

% 6节点邻接矩阵（0开始：0=地面站，1-5=卫星）
% graph(1,:)=节点0的链路，graph(2,:)=节点1的链路... graph(6,:)=节点5的链路
graph = [0   50  inf 40  25  10;   % 节点0（地面站）：连1(50)、3(40)、4(25)、5(10)
         50  0   15  20  inf 25;   % 节点1：连0(50)、2(15)、3(20)、5(25)
         inf 15  0   10  20  inf;  % 节点2：连1(15)、3(10)、4(20)
         40  20  10  0   10  25;   % 节点3：连0(40)、1(20)、2(10)、4(10)、5(25)
         25  inf 20  10  0   55;   % 节点4：连0(25)、2(20)、3(10)、5(55)
         10  25  inf 25  55  0];   % 节点5：连0(10)、1(25)、3(25)、4(55)

% 自定义参数（0开始）
start_node = 0;   % 起点：地面站0
dest_node = 2;    % 目标：卫星2

% 调用函数
[target_distance, target_path] = dijkstra(graph, start_node, dest_node);

% 输出结果
fprintf('起点%d（地面站）→ 目标点%d（卫星）的最短路径结果：\n', start_node, dest_node);
fprintf('最短距离：%g\n', target_distance);
if ~isempty(target_path)
    fprintf('最短路径：');
    for i = 1:length(target_path)
        if i == length(target_path)
            fprintf('%d\n', target_path(i));
        else
            fprintf('%d → ', target_path(i));
        end
    end
else
    fprintf('最短路径：无（起点不可达目标点）\n');
end