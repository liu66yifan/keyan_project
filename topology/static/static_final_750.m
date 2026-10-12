clc;
% 假设 adj_matrices 已经定义并且包含 12×751×751 的数据

% 设定参数
M_bits = 1000000;  % 比特数（根据您的需求，可能需要调整）
times = 25;
% 初始化 z 数组
z_values = zeros(1, times);  % 用于存储每个时刻的 z 值

% adj_matrices_static
for t_idx = 1:times
    % 获取当前时刻的邻接矩阵
    adj_matrix = squeeze(adj_matrices_static(t_idx, :, :));  % 去掉冗余的第一个维度

    % 调用 rmpr_build_waterfill_inputs_verbose 函数生成输入结构
    in = rmpr_build_waterfill_inputs_verbose(t_idx, adj_matrix, result3, S, D, L, M_bits, true);

    % 获取 q 和 gamma
    q = in.q_sec;
    gamma = in.gamma_sec;

    % 显示 q 和 gamma
    disp(q);
    disp(gamma);

    % 设置 n（根据需要进行调整）
    n = 1;

    % 调用 solve_routing_optimization 计算 w 和 z
    [w, z] = solve_routing_optimization(q, gamma, n);

    % 打印结果
    disp("------------------------------------");
    disp(w);
    disp(z);

    % 将每个时刻得到的 z 保存到 z_values 数组中
    z_values(t_idx) = z;  % 假设 z 是每个时刻的结果
end

% ====================== 统计图：z vs 时刻 =======================
figure;
plot(1:times, z_values, '-o', 'LineWidth', 2, 'MarkerSize', 6);
xlabel('时刻 (15秒间隔)');
ylabel('静态拓扑--RMPR算出的最大时延值z');
title('z 值 vs 时刻');
grid on;
