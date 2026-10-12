clc;
%% 综合代价随距离连续变化的“代价墙”分析
sigma_s0 = 2;
c = 299792458;
min_dist_limit = 600e3;

% 1. 从真实的 dist_map 中提取归一化基准，保证和之前图表的尺度绝对一致
valid_dists = dist_matrix(dist_matrix >= min_dist_limit & dist_matrix < inf);
max_physical_dist = max(valid_dists);

T_max_val = max_physical_dist / c;
p_fail_redline = 0.07; 
R_ref_val = -log(1 - p_fail_redline);

% 2. 构造连续的距离序列 (X轴: 600km 到 2500km)
d_array = linspace(min_dist_limit, 2500e3, 500);

% 计算这批距离对应的纯物理时延和可靠性代价
T_raw_array = d_array / c;
% 注意：此处直接调用您的 oisl_failure_prob 函数
pf_array = arrayfun(@(d) oisl_failure_prob(d, sigma_s0), d_array);
R_raw_array = -log(1 - min(pf_array, 0.9999));

% 归一化
T_norm_array = T_raw_array / T_max_val;
R_norm_array = R_raw_array / R_ref_val;

% 3. 选择几个具有代表性的 alpha 进行对比
alpha_test_vals = [0, 0.2, 0.5, 0.8, 1.0];
colors = lines(length(alpha_test_vals));

figure('Color', 'w', 'Position', [150, 150, 850, 550]);
hold on; grid on;

% 绘制不同 alpha 下的 距离-代价 曲线
for i = 1:length(alpha_test_vals)
    a = alpha_test_vals(i);
    % 核心代价公式
    W_array = (1 - a) * T_norm_array + a * R_norm_array;
    
    plot(d_array / 1e3, W_array, 'LineWidth', 2, 'Color', colors(i,:), ...
        'DisplayName', sprintf('\\alpha = %.1f', a));
end

% 绘制参考线：您的网络的物理最大距离范围 (示意)
xline(max_physical_dist/1e3, '--k', 'Max Topology Dist', 'LineWidth', 1.5, 'HandleVisibility','off');

% 标注装饰
xlabel('Link Distance (km)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Composite Cost W', 'FontSize', 12, 'FontWeight', 'bold');
title(sprintf('Cost Wall: How Distance drives up Link Cost (\\sigma_{s0} = %.1f)', sigma_s0), 'FontSize', 13);
legend('Location', 'northwest', 'FontSize', 11);

% 限制 Y 轴范围，防止极高概率导致的无穷大破坏画面
ylim([0, 5]); 
xlim([min_dist_limit/1e3, 2200]); % 看到2200公里基本够了
hold off;
