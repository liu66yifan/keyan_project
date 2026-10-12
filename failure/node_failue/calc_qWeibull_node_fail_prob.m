function p_node = calc_qWeibull_node_fail_prob(age_year, slot_sec, q, beta, eta, accel)
% calc_qWeibull_node_fail_prob
% 根据 q-Weibull 模型，计算节点在单个时间片内的失效概率
%
% 输入:
%   age_year : 节点当前年龄（年），可为标量/向量
%   slot_sec : 一个判决时间片长度（秒）
%   q        : q-Weibull 参数
%   beta     : 形状参数
%   eta      : 尺度参数（年）
%   accel    : 风险加速因子
%              =1 表示原始概率
%              >1 表示放大节点失效风险，用于压力测试
%
% 输出:
%   p_node   : 单个时间片内节点失效概率，范围 [0,1]

    if nargin < 6 || isempty(accel)
        accel = 1;
    end

    if any(age_year < 0, 'all')
        error('age_year must be non-negative.');
    end
    if slot_sec <= 0
        error('slot_sec must be positive.');
    end
    if beta <= 0 || eta <= 0
        error('beta and eta must be positive.');
    end
    if accel <= 0
        error('accel must be positive.');
    end

    % 秒 -> 年
    dt_year = slot_sec / (365 * 24 * 3600);

    % q-Weibull hazard rate
    numerator   = (beta ./ eta) .* (age_year ./ eta).^(beta - 1);
    denominator = 1 - (1 - q) .* (age_year ./ eta).^beta;

    h_t = numerator ./ denominator;
    h_t(denominator <= 0) = inf;

    % 单时间片失效概率
    p_node = 1 - exp(-accel .* h_t .* dt_year);

    % 裁剪到 [0,1]
    p_node = min(max(p_node, 0), 1);
end
