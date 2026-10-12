clc;
% 这个代码逻辑是先链路失效和节点失效，再进行选路分配，而不是先分配在失效==》我感觉体现不出来抗毁
% 但是这样适合画time–delay 看网络在随机失效下的“可达性/最短可达时延”怎么波动
%% ===== 参数设置 =====
total_duration = 90 * 60;   % 总时长 10 分钟 (单位: 秒)
time_interval = 15;         % 每次实验间隙 15 秒
time_points = total_duration / time_interval;  % 总轮数

link_fail_probability = 0.01;   % 瞬时链路失效概率 (例如干扰)
node_fail_probability = 0.0005; % 节点失效概率 (每个时间片内发生的概率)
                                % 注意：这是累积的，设置太大会导致后期全断

rng_seed = 1;  
rng(rng_seed);

% ===== 初始设置 =====
% 假设 adj_matrices_static_temp 是这一时刻的基准图
% 假设 result3 是卫星接入信息
% 假设 load 了相关数据... 
% (这里保留你原本的变量名，请确保 workspace 里有这些数据)

target_sat = 402;  % 固定目标卫星
L = 4;             % 路径数量
M_bits = 1000000;  % 数据量
t_idx = 1;         % 初始时刻索引

% 获取卫星总数 (根据矩阵维度)
total_sat_num = size(adj_matrices_static_temp, 1);

% ===== 【新增】节点状态初始化 =====
% 逻辑数组，false代表存活，true代表已失效
dead_nodes_mask = false(total_sat_num, 1); 
% 记录失效顺序和ID，方便后续分析
failed_node_history = []; 

% ===== 记录变量 =====
delays = zeros(1, time_points);
alive_node_counts = zeros(1, time_points); % 记录每轮存活节点数

%% ===== 循环进行实验 =====
for i = 1:time_points
    current_time = i * time_interval;
    
    %% === 1. 节点失效模拟 (永久性，累积) ===
    % 找出当前还活着的节点索引
    alive_indices = find(~dead_nodes_mask);
    
    % 对每个活着的节点做一次伯努利试验
    if ~isempty(alive_indices)
        r_node = rand(length(alive_indices), 1);
        % 判定新增失效
        newly_failed_mask = r_node < node_fail_probability;
        newly_failed_ids = alive_indices(newly_failed_mask);
        
        % 更新死亡名单
        if ~isempty(newly_failed_ids)
            dead_nodes_mask(newly_failed_ids) = true;
            failed_node_history = [failed_node_history; newly_failed_ids]; %#ok<AGROW>
            fprintf('时刻 %d 新增失效节点: %s\n', i, mat2str(newly_failed_ids'));
        end
    end
    
    % 记录当前存活数
    alive_node_counts(i) = sum(~dead_nodes_mask);

    %% === 2. 构建当前时刻的基础拓扑 ===
    % 取出原始静态拓扑
    current_adj_matrix = adj_matrices_static_temp;
    
    % 将失效节点的行和列全部置为 Inf (断开所有连接)
    % 这一步实现了"节点失效后对应的4条ISL断开"
    current_adj_matrix(dead_nodes_mask, :) = inf;
    current_adj_matrix(:, dead_nodes_mask) = inf;
    
    %% === 3. 链路失效模拟 (瞬时性) ===
    % 在已经剔除了死节点的矩阵基础上，再撒一次链路失效
    % 注意：simulate_link_failure 函数需要稍微调整，或者直接确保它只处理 finite 的边
    % 这里的 fprintf 改动在下面的函数定义里
    adj_matrix_final = simulate_link_failure(current_adj_matrix, link_fail_probability);

    %% === 4. 检查目标节点或接入节点是否挂了 ===
    % 如果目标卫星挂了，直接时延无限大，跳过计算
    if dead_nodes_mask(target_sat)
        fprintf('警告: 目标卫星 %d 已失效，通信中断。\n', target_sat);
        delays(i) = inf;
        continue;
    end

    %% === 5. 路由算法计算 (保持原逻辑) ===
    % 获取当前时刻潜在的接入卫星
    drone_sat_ids = result3(t_idx).sat_ids(:);
    drone_sat_ids = drone_sat_ids(isfinite(drone_sat_ids));
    
    % 剔除掉已经挂掉的接入卫星
    drone_sat_ids = drone_sat_ids(~dead_nodes_mask(drone_sat_ids));
    
    if isempty(drone_sat_ids)
        fprintf('时刻 %d 无可用接入卫星。\n', i);
        delays(i) = inf;
        continue;
    end

    % 初始化最小时延
    min_delay = inf;
    best_sat = NaN;
    
    % 对每个接入卫星进行路径计算
    for sat_idx = 1:numel(drone_sat_ids)
        S = drone_sat_ids(sat_idx);
        
        % 调用 RMPR 接口 (传入的是已经处理过节点和链路失效的矩阵 adj_matrix_final)
        try
            in = rmpr_build_waterfill_inputs_verbose_temp(t_idx, adj_matrix_final, result3, S, target_sat, L, M_bits, false); % verbose关掉减少刷屏
            
            if isempty(in.q_sec) % 如果找不到路径
                continue;
            end
            
            q = in.q_sec;
            gamma = in.gamma_sec; 
    
            % 计算注水
            [~, z] = solve_routing_optimization(q, gamma, 2);
    
            % 记录最优
            current_min_z = min(z);
            if current_min_z < min_delay
                min_delay = current_min_z;
                best_sat = S;
            end
        catch
            % 捕捉不可达的情况（例如图不连通）
        end
    end

    % 记录结果
    delays(i) = min_delay;
    
    if isfinite(min_delay)
        fprintf('时刻 %d/%d，存活节点 %d，最小时延 = %.6f s (接入: %d)\n', ...
            i, time_points, alive_node_counts(i), min_delay, best_sat);
    else
        fprintf('时刻 %d/%d，存活节点 %d，无法到达目标。\n', ...
            i, time_points, alive_node_counts(i) - 1);
    end
end

%% ===== 画图 =====
time = (1:time_points) * time_interval / 60; % 分钟

figure('Position', [100, 100, 1000, 400]);

subplot(1, 2, 1);
plot(time, delays, 'LineWidth', 1.5);
xlabel('时间 (分钟)');
ylabel('端到端时延 (秒)');
title(['时延变化 (LinkP=' num2str(link_fail_probability) ', NodeP=' num2str(node_fail_probability) ')']);
grid on;

subplot(1, 2, 2);
plot(time, alive_node_counts - 1, 'r-', 'LineWidth', 1.5);
xlabel('时间 (分钟)');
ylabel('存活卫星数量');
title('网络节点存活情况');
grid on;

%% ===== 链路失效模拟函数 (微调) =====
function adj_matrix_with_fail = simulate_link_failure(input_matrix, p_fail)
    % 输入矩阵 input_matrix 中，死掉的节点行/列已经是 Inf 了
    A = input_matrix; 
    
    % 只对存在的连接(finite)且在上三角进行随机失效判断
    mask_valid_links = triu(isfinite(A), 1); 
    [ii, jj] = find(mask_valid_links); 
    
    failed_count = 0; % 计数器
    
    if ~isempty(ii)
        m = numel(ii);
        % 生成随机数
        r = rand(m, 1);
        fail_indices = (r < p_fail);
        
        ii_f = ii(fail_indices);
        jj_f = jj(fail_indices);
        
        failed_count = numel(ii_f); % 记录失效数量
        
        % 断开选中的链路
        for k = 1:failed_count
            u = ii_f(k); v = jj_f(k);
            A(u, v) = inf;
            A(v, u) = inf;
        end
    end
    
    % 打印失效数量
    fprintf('  -> 本次随机链路失效数: %d\n', failed_count);
    
    adj_matrix_with_fail = A;
end