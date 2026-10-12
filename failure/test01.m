% 绘制OISL链路失效概率曲线 (只改线条，背景白色，其余不动)
clc;

% ===== 1) 距离范围 800 - 2000 km =====
z_km = linspace(800, 2000, 900); 
z_m  = z_km * 1e3;

% 离散距离点（m）
z_all_points = [1446904.80; 343758.53; 481038.35; 700364.98; ...
                922336.78; 1086058; 1287186.50; 1375989.48; ...
                1424983.79; 1589180.71; 1664558.92; 1752877.81; 1769863.24];

% ===== 2) 抖动参数 =====
sigma_s0_list = [1.6, 1.8, 2.0, 2.2, 2.4];

% ===== 白色背景图窗 =====
figure('Color','w','Position',[100,100,600,420]); 
ax = gca;
ax.Color = 'w';   % 坐标区背景白色

for j = 1:length(sigma_s0_list)
    s0 = sigma_s0_list(j);
    p_curve = arrayfun(@(x) oisl_failure_prob(x, s0), z_m);

    % 只改线条
    switch j
        case 1
            this_color = [0.00 0.4470 0.7410];   % 蓝
        case 2
            this_color = [0.8500 0.3250 0.0980]; % 橙
        case 3
            this_color = [0.9290 0.6940 0.1250]; % 黄
        case 4
            this_color = [0.4940 0.1840 0.5560]; % 紫
        case 5
            this_color = [0.4660 0.6740 0.1880]; % 绿
    end

    semilogy(z_km, p_curve + 1e-15, ...
        'Color', this_color, ...
        'LineWidth', 2.2, ...
        'DisplayName', sprintf('\\sigma_{s,0} = %.2f m', s0));
    hold on;
end


% ===== 下面全部保持你原来的设置 =====
grid on;
set(gca,'YMinorGrid','on','FontSize',10,'FontName','Times New Roman');
xlabel('Distance z (km)','FontSize',11,'FontName','Times New Roman');
ylabel('Link Failure Probability (p_{fail})','FontSize',11,'FontName','Times New Roman');

% 坐标轴范围
xlim([800 2000]);
ylim([1e-4 1]);

legend('Location','southeast','FontSize',9,'FontName','Times New Roman');
hold off;

% 导出高清图片
print('OISL_Failure_Probability','-dpng','-r300');
fprintf('已导出论文图片：OISL_Failure_Probability.png\n');
