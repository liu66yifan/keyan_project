clc;
%% 先加上链路失效概率的情况
% ===== 参数 =====
p_fail = 0;      % 链路失效概率
rng_seed = 1;       % 可选：固定随机种子便于复现（不要就删掉这行和rng）
rng(rng_seed);

% ===== 只对“现存边”剪枝：仅取上三角 finite 边 =====
A = adj_matrices_static_temp;          % 751x751
n = size(A,1);

mask_upper_exist = triu(isfinite(A), 1);   % 上三角 & finite（自动跳过inf，且不含对角）
[ii, jj] = find(mask_upper_exist);         % 这些就是所有现存边(u,v)

m = numel(ii);
fprintf('现存边数量（上三角计数）= %d\n', m);

% ===== 对每条现存边做一次伯努利试验 =====
r = rand(m,1);                 % 每条边一个随机数
fail = (r < p_fail);           % fail(k)=true 表示第k条边失效

ii_f = ii(fail);
jj_f = jj(fail);

fprintf('本次失效边数量 = %d\n', numel(ii_f));

% ===== 打印失效边（节点编号=矩阵索引-1）并更新矩阵 =====
for k = 1:numel(ii_f)
    a = ii_f(k); b = jj_f(k);
    old_w = A(a,b);

    u = a - 1; v = b - 1;
    fprintf('[LINK FAIL] %d <-> %d | old_w = %.2f\n', u, v, old_w);

    A(a,b) = inf;
    A(b,a) = inf;
end

% ===== 写回更新后的邻接矩阵 =====
adj_matrices_static_temp = A;
%% 经过RMPR算法
time_idx = 1;  % 选择某时刻（如果遍历所有时刻，可以放进一个循环）
drone_sat_ids = result3(time_idx).sat_ids(:);
drone_sat_ids = drone_sat_ids(isfinite(drone_sat_ids));  % 只取有效的卫星

fprintf('当前时刻 %d 有效接入卫星：%s\n', time_idx, mat2str(drone_sat_ids'));

%% ===== 对每个接入卫星进行路径计算 =====
target = 402;  % 固定目标卫星为 402
L = 4;    % 需要找到的路径数量
M_bits = 1000000;
t_idx = 1;

% 初始化最小时延和最小时延对应的接入卫星
min_delay = inf;
best_sat = NaN;
best_paths = {};  % 用来存储最小延迟的所有路径

for sat_idx = 1:numel(drone_sat_ids)
    % 当前接入卫星作为起点
    S = drone_sat_ids(sat_idx);

    fprintf('\n=== 处理接入卫星 %d ===\n', S);

    % 调用搜索L条不相交路径函数
    in = rmpr_build_waterfill_inputs_verbose_temp(t_idx, adj_matrices_static_temp, result3, S, target, L, M_bits, true);
    disp("=====================================");
    q = in.q_sec;
    gamma = in.gamma_sec; 

    % 显示 q 和 gamma
    disp('q:');
    disp(q);
    disp('gamma:');
    disp(gamma);

    % 设置 n（根据需要进行调整）
    n = 2;

    % 调用 solve_routing_optimization 计算 w 和 z
    [w, z] = solve_routing_optimization(q, gamma, n);

    % 显示 w 和 z
    disp('w:');
    disp(w);
    disp('z:');
    disp(z);

    % 记录当前接入卫星的最小时延和对应的路径
    [min_z, idx_min] = min(z);
    
    % 如果当前接入卫星的最小时延小于之前记录的最小时延，则更新
    if min_z < min_delay
        min_delay = min_z;
        best_sat = S;
        best_paths = in.paths;  % 更新为当前接入卫星的所有L条路径
    % 如果当前时延等于最小时延，则追加到best_paths
    elseif min_z == min_delay
        best_paths = [best_paths; in.paths];  % 合并当前接入卫星的所有L条路径
    end
end

% 输出最终选择的最优接入卫星和路径
fprintf('\n最小时延的接入卫星是 %d\n', best_sat);
fprintf('最小时延为：%f\n', min_delay);
disp('对应的路径是：');
disp(best_paths);
