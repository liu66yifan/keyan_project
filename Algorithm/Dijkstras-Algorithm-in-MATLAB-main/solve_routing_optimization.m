function [w, z] = solve_routing_optimization(q, gamma, n)
    q = q(:)'; 
    gamma = gamma(:)';
    L = length(q);

    % 基本检查
    if length(gamma)~=L || n<=0 || n>L 
        error('输入不合法：0 < n <= L 且维数一致');
    end

    tol = 1e-10; % 容差
    w = zeros(1, L);
    
    % 初始水位 z 从最小传播时延开始
    z = min(q);
    total_w = 0;

    max_iter = 1000; % 防止死循环
    iter = 0;

    while total_w < n - tol
        iter = iter + 1;
        if iter > max_iter
            warning('达到最大迭代次数，可能未完全收敛');
            break;
        end

        % 1. 找出当前处于"活跃区间"的路径 (正在传输且未满)
        % 条件：路径开始了 (q <= z) 且 路径还没满 (z < q + gamma)
        active_idx = find(q <= z + tol & z < q + gamma - tol);
        
        % 2. 找出那些已经满了的路径，确保 w 置为 1
        full_idx = find(z >= q + gamma - tol);
        w(full_idx) = 1;
        
        % 3. 找出还没开始的路径
        future_idx = find(q > z + tol);
        
        % 计算当前的总流量
        % 对于 active 的路径，计算当前分配量
        w(active_idx) = (z - q(active_idx)) ./ gamma(active_idx);
        % 修正范围，防止浮点误差
        w = max(0, min(1, w));
        
        total_w = sum(w);
        
        if total_w >= n - tol
            break; % 完成分配
        end

        % === 核心修复逻辑 ===
        
        if isempty(active_idx)
            % 【情况A：断层】当前所有能用的路径都满了，但还没凑够 n
            % 必须“跳”到下一条路径的开始时刻
            if isempty(future_idx)
                % 没有未来路径了，说明 capacity 不够（通常 n > L 时发生，但前面检查过了）
                error('无法满足分配需求，所有路径已满但 total_w < n');
            else
                % 直接跳跃到下一条路径的起始时间
                z = min(q(future_idx));
            end
        else
            % 【情况B：正常注水】有路径正在传输，计算可以提升多少水位
            
            % B1. 下一条新路径开始的时刻
            if isempty(future_idx)
                next_q_start = inf;
            else
                next_q_start = min(q(future_idx));
            end
            
            % B2. 当前活跃路径填满的时刻
            current_paths_end = q(active_idx) + gamma(active_idx);
            next_path_full = min(current_paths_end);
            
            % B3. 确定下一个关键时间点 (next_target_z)
            next_event_z = min(next_q_start, next_path_full);
            
            % B4. 计算联合速率 (Joint Rate)
            joint_rate = sum(1 ./ gamma(active_idx));
            
            % B5. 计算如果注水到下一个事件点，会增加多少 w
            delta_z_max = next_event_z - z;
            if delta_z_max < 1e-12; delta_z_max = 1e-12; end % 防止停滞
            
            potential_w_increase = joint_rate * delta_z_max;
            
            % B6. 判断是直接完成，还是走到下一个事件点
            w_needed = n - total_w;
            
            if potential_w_increase >= w_needed
                % 可以直接在这个区间内完成
                delta_z = w_needed / joint_rate;
                z = z + delta_z;
                % 更新 w 并退出
                w(active_idx) = (z - q(active_idx)) ./ gamma(active_idx);
                w = max(0, min(1, w));
                break;
            else
                % 还没填满，走到下一个事件点
                z = next_event_z;
            end
        end
    end
    
    % 最终修正
    w = max(0, min(1, w));
    % 这里的 z 就是最终的端到端时延
end