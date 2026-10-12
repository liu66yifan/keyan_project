%% 无人机-卫星过渡的最终版（计算所有时刻+所有卫星速率，保存result3，新增L保留最优L条链路，并写入对应距离字段dists_m）
clc; 

%% ====================== 1. 核心参数 ======================
fc = 28e9;               % 载波频率：28 GHz
B = 10e6;                % 带宽：10 MHz
Pt_dBm = 20;             % 无人机发射功率：20 dBm
Gt_dBi = 30.5;           % 无人机天线增益：30.5 dBi
Gr_dBi = 58.5;           % 卫星接收天线增益：58.5 dBi
N0 = -174;               % dBm/Hz
Lsf = 5;                 % 损耗
NF_dB = 3;               % 噪声系数
N_total_dBm = N0 + 10*log10(B)+ NF_dB;      % 热噪声功率
L = 4;                   % 每个时刻保留速率最优的L条链路

%% ====================== 2. 加载数据 ======================
try
    load('全时刻可连接卫星数据.mat', 'results');
    load('无人机-卫星全时刻距离数据.mat', 'result2');
    fprintf('数据加载成功\n');
catch
    error('请确认「全时刻可连接卫星数据.mat」和「无人机-卫星全时刻距离数据.mat」在当前路径！');
end

%% ====================== 3. 预分配result3结构体（新增字段dists_m） ======================
total_times = length(results);

% 新增字段：dists_m（与sat_ids一一对应的距离，单位m）
result3 = struct('time', [], 'sat_ids', [], 'rates_Mbps', [], 'dists_m', []);
result3(total_times) = result3(1);

%% ====================== 4. 遍历所有时刻+卫星，计算并存储数据 ======================
fprintf('总时刻数：%d个，每时刻保留速率最优的%d条链路\n', total_times, L);
fprintf('=============================================\n');

for t_idx = 1:total_times
    current_time = results(t_idx).time;
    result3(t_idx).time = current_time;

    fprintf('\n正在计算第%d/%d个时刻：%s\n', t_idx, total_times, datestr(current_time));

    sat_list = results(t_idx).sat_list;
    if isempty(sat_list)
        fprintf('该时刻无有效卫星，跳过计算\n');
        result3(t_idx).sat_ids = [];
        result3(t_idx).rates_Mbps = [];
        result3(t_idx).dists_m = [];   % 新增：距离也为空
        fprintf('----------------------------------------\n');
        continue;
    end

    total_sat = length(sat_list);
    fprintf('该时刻可连接卫星总数：%d颗\n', total_sat);

    sat_ids = zeros(1, total_sat);
    rates_Mbps = nan(1, total_sat);     % 用NaN更稳
    dists_m_all = nan(1, total_sat);    % 新增：每颗候选卫星的距离（m）

    for sat_idx = 1:total_sat
        sat_id = sat_list(sat_idx).sat_id;
        sat_ele = sat_list(sat_idx).elevation;

        sat_ids(sat_idx) = sat_id;

        fprintf('\n----- 计算该时刻第%d颗卫星（ID=%d）-----\n', sat_idx, sat_id);
        fprintf('卫星ID：%d，仰角=%.2f°\n', sat_id, sat_ele);

        % 距离（m）
        sat_dist_m = result2(t_idx).sat_distances(sat_id);
        dists_m_all(sat_idx) = sat_dist_m;   % 新增：先把距离存下来（即使NaN也存）

        if isnan(sat_dist_m)
            fprintf('卫星ID=%d无距离数据，速率记为NaN\n', sat_id);
            rates_Mbps(sat_idx) = NaN;
            continue;
        end

        sat_dist_km = sat_dist_m / 1000;
        fprintf('无人机-卫星距离：%.1f 米 = %.1f 千米\n', sat_dist_m, sat_dist_km);

        %% FSPL
        fc_MHz = fc / 1e6;
        FSPL_dB = 20*log10(sat_dist_km) + 20*log10(fc_MHz) + 32.45 + Lsf;
        fprintf('FSPL = %.2f dB\n', FSPL_dB);

        %% Pr
        Pr_dBm = Pt_dBm + Gt_dBi + Gr_dBi - FSPL_dB;
        fprintf('Pr = %.2f dBm\n', Pr_dBm);

        %% SINR
        SINR_dB = Pr_dBm - N_total_dBm;
        SINR_linear = 10^(SINR_dB / 10);
        fprintf('SINR = %.2f dB\n', SINR_dB);

        %% Rate
        R_Mbps = (B / 1e6) * log2(1 + SINR_linear);
        rates_Mbps(sat_idx) = R_Mbps;
        fprintf('Rate = %.2f Mbps\n', R_Mbps);
    end

    %% ====================== 5. 筛选每个时刻速率最优的L条链路（同步筛选距离） ======================
    valid_idx = ~isnan(rates_Mbps);  % 有效速率索引
    valid_sat_ids = sat_ids(valid_idx);
    valid_rates = rates_Mbps(valid_idx);
    valid_dists = dists_m_all(valid_idx);   % 新增：同步距离
    valid_num = length(valid_sat_ids);

    if valid_num > 0
        [sorted_rates, sorted_idx] = sort(valid_rates, 'descend');
        sorted_sat_ids = valid_sat_ids(sorted_idx);
        sorted_dists = valid_dists(sorted_idx);   % 新增：按同样顺序排序距离

        keep_num = min(L, valid_num);
        final_sat_ids = sorted_sat_ids(1:keep_num);
        final_rates = sorted_rates(1:keep_num);
        final_dists_m = sorted_dists(1:keep_num); % 新增：最终距离（m）

        fprintf('\n----- 筛选该时刻速率最优的%d条链路 -----\n', L);
        fprintf('该时刻有效速率卫星数：%d颗 → 保留前%d颗\n', valid_num, keep_num);
        fprintf('保留的卫星ID：'); disp(final_sat_ids);
        fprintf('对应速率（Mbps）：'); disp(final_rates);
        fprintf('对应距离（m）：'); disp(final_dists_m);
    else
        final_sat_ids = [];
        final_rates = [];
        final_dists_m = [];
        fprintf('\n----- 筛选该时刻速率最优的%d条链路 -----\n', L);
        fprintf('该时刻无有效速率卫星，无链路保留\n');
    end

    % 写入result3（新增dists_m字段）
    result3(t_idx).sat_ids = final_sat_ids;
    result3(t_idx).rates_Mbps = final_rates;
    result3(t_idx).dists_m = final_dists_m;    % ? 新增字段写入
    fprintf('----------------------------------------\n');
end

%% ====================== 6. 保存result3到mat文件 ======================
save('无人机-卫星速率数据_result3.mat', 'result3');
fprintf('\n所有时刻速率数据已保存到result3！\n');
fprintf('result3字段：time、sat_ids、rates_Mbps、dists_m（与sat_ids对应的距离m）\n');
fprintf('筛选规则：按速率降序排序，保留每个时刻前%d条最优链路\n', L);

%% ====================== 7. 调用示例（展示新增字段dists_m） ======================
fprintf('\nresult3调用示例（含dists_m字段）:\n');
t_example = 1;
if t_example <= total_times
    fprintf('第%d个时刻：%s\n', t_example, datestr(result3(t_example).time)); 
    disp('sat_ids:'); disp(result3(t_example).sat_ids);
    disp('rates_Mbps:'); disp(result3(t_example).rates_Mbps);
    disp('dists_m:'); disp(result3(t_example).dists_m);
end
