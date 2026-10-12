clc; 
% 节点的图

%% =========================
% 1. 参数设置
%% =========================
beta = 2.5;
eta  = 15;

q_list = [0.8, 1.0, 1.2];   % 统一成 0.8 / 1.0 / 1.2
age = linspace(0.5, 18, 600);

% 场景分区
bg_range  = [1, 4];      % 背景年轻节点
hot_range = [11, 14];    % 老旧热点节点

%% =========================
% 2. 计算 hazard
%% =========================
h_all = zeros(numel(q_list), numel(age));
for i = 1:numel(q_list)
    h_all(i, :) = qweibull_hazard(age, q_list(i), beta, eta);
end

ymax = 1.12 * max(h_all(:), [], 'omitnan');

%% =========================
% 3. 画图
%% =========================
figure('Color','w','Position',[180,180,900,500]); 
hold on;

% 阴影区：背景节点
patch([bg_range(1) bg_range(2) bg_range(2) bg_range(1)], ...
      [0 0 ymax ymax], [0.88 0.93 1.00], ...
      'EdgeColor','none', 'FaceAlpha',0.55, ...
      'HandleVisibility','off');

% 阴影区：老旧热点节点
patch([hot_range(1) hot_range(2) hot_range(2) hot_range(1)], ...
      [0 0 ymax ymax], [1.00 0.90 0.90], ...
      'EdgeColor','none', 'FaceAlpha',0.55, ...
      'HandleVisibility','off');

% 三条曲线
plot(age, h_all(1,:), 'b-.', 'LineWidth', 2.0, ...
    'DisplayName', 'q-Weibull (q = 0.8)');
plot(age, h_all(2,:), 'k--', 'LineWidth', 2.0, ...
    'DisplayName', 'Weibull (q = 1.0)');
plot(age, h_all(3,:), '-', 'LineWidth', 2.4, ...
    'Color', [0.85 0.33 0.10], ...
    'DisplayName', 'q-Weibull (q = 1.2)');

% 在热点区标出差异
age_mark = 13;
h_08 = qweibull_hazard(age_mark, 0.8, beta, eta);
h_10 = qweibull_hazard(age_mark, 1.0, beta, eta);
h_12 = qweibull_hazard(age_mark, 1.2, beta, eta);

scatter(age_mark, h_08, 36, 'b', 'filled', 'HandleVisibility','off');
scatter(age_mark, h_10, 36, 'k', 'filled', 'HandleVisibility','off');
scatter(age_mark, h_12, 36, [0.85 0.33 0.10], 'filled', 'HandleVisibility','off');

plot([age_mark age_mark], [min([h_08,h_10,h_12]) max([h_08,h_10,h_12])], ...
    ':', 'Color', [0.35 0.35 0.35], 'LineWidth', 1.2, ...
    'HandleVisibility','off');

grid on;
box off;
set(gca,'FontSize',11,'LineWidth',1.0,'TickDir','out');

xlabel('Node age (years)', 'FontSize', 12);
ylabel('Hazard rate h(t)', 'FontSize', 12);
title('Hazard-rate comparison between Weibull and q-Weibull models', ...
    'FontSize', 13, 'FontWeight', 'normal');
legend('Location','northwest');

xlim([0 18]);
ylim([0 ymax]);

text(mean(bg_range), 0.93*ymax, 'Background nodes', ...
    'HorizontalAlignment','center', 'FontSize', 11);
text(mean(hot_range), 0.93*ymax, 'Aged hotspot nodes', ...
    'HorizontalAlignment','center', 'FontSize', 11);

%% =========================
% 4. 局部函数
%% =========================
function h_t = qweibull_hazard(t, q, beta, eta)
    numerator   = (beta ./ eta) .* (t ./ eta).^(beta - 1);
    denominator = 1 - (1 - q) .* (t ./ eta).^beta;
    h_t = numerator ./ denominator;
    h_t(denominator <= 0) = NaN;
end
