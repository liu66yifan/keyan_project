clc; 

% 1. 定义失效概率范围 (从0到0.99，避免1处的对数无穷大导致图表无法显示)
p_fail = linspace(0, 0.99, 1000); 

% 2. 计算风险代价 W = -ln(1 - p_fail)
% 注意：1 - p_fail 就是成功率 P_success
cost = -log(1 - p_fail);

% 3. 绘图
figure('Color', 'w', 'Position', [200, 200, 800, 600]);
plot(p_fail, cost, 'LineWidth', 2.5, 'Color', [0.85 0.33 0.1]);
hold on;
grid on;

% 4. 标注关键逻辑点
% 点 (0, 0)
plot(0, 0, 'ko', 'MarkerFaceColor', 'g', 'MarkerSize', 8);
text(0.05, 0.3, ['\leftarrow p_{fail}=0 (100%成功率)', newline, '代价 W = 0 (最理想链路)'], ...
    'FontSize', 11, 'FontWeight', 'bold', 'Color', 'b');

% 标注趋向无穷大的趋势
xline(0.99, '--r', 'LineWidth', 1.5);

text(0.65, 4.5, ['p_{fail} \rightarrow 1 (完全失效) \rightarrow', newline, '代价 W \rightarrow \infty (彻底阻断)'], ...
    'FontSize', 11, 'FontWeight', 'bold', 'Color', 'r');

% 5. 图表修饰
xlabel('链路失效概率 p_{fail}', 'FontSize', 12);
ylabel('风险代价 W = -ln(1 - p_{fail})', 'FontSize', 12);

% 添加物理含义说明框
annotation('textbox', [0.15, 0.7, 0.3, 0.15], 'String', ...
    {['逻辑：'], ...
     ['1. 最大化pth(成功概率)=p1*p2*...pn'], ...
     ['2. 对数变换让乘法变加法'], ...
     ['3. 最大化路径成功率 <=> 最小化负对数风险']
}, ...
    'FontSize', 10, 'EdgeColor', 'k', 'BackgroundColor', [0.94 0.94 0.94]);

% 限制坐标轴范围
xlim([0, 1]);

ylim([-0.2, 5]);

fprintf('绘图完成。观察图中的单调递增、凸性增长特征。\n');
