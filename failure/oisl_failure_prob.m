%可调参数k0

function p_fail = oisl_failure_prob(z_m, sigma_s0)
% calculate_laser_fail_prob
% Strictly based on the paper "Channel Modeling and Rate Analysis of OISL"
% Input:
%   z_m      : link distance (m)
%   sigma_s0 : initial pointing jitter amplitude (m)
% Output:
%   p_fail   : failure probability in [0,1]

    %% ===== 0) check =====
    if z_m <= 0
        error('z_m must be > 0 (meters).');
    end

    %% ===== 1) Fixed parameters =====
    w0      = 0.042;          
    wd      = 0.1;          
    f_laser = 200e12;       
    h_PL    = 0.9;          
    eta     = 0.5;          
    P_T     = 0.5;          
    p_th    = 1e-6;         

    % 指数增长参数（保持原论文设置）
    k0       = 0.096;%这个是从0.12跳成0.1的到时候可以调！！！！！！！！！！！！！！！！！！！！！！！！！！！        
    d0_m     = 100e3;       

    %% ===== 2) physical constants and beam parameters =====
    c = 299792458;          
    lambda = c / f_laser;   
    theta = lambda / (pi * w0);
    w_z = z_m * tan(theta);

    %% ===== 3) channel threshold h_th =====
    h_th = p_th / (h_PL * eta * P_T);

    %% ===== 4) distance-dependent sigma_s =====
    sigma_s = sigma_s0 * exp(k0 * (z_m / d0_m));  

    %% ===== 5) feasibility via A0 =====
    A0 = 2 * wd^2 / (w_z^2);

    if h_th >= A0
        p_fail = 1.0;
        return;
    end

    %% ===== 6) Eq.(15) r_max =====
    inner_log = A0 / h_th; 
    r_max = sqrt((w_z^2 / 2) * log(inner_log));

    %% ===== 7) Eq.(18) failure probability =====
    p_fail = exp(-(r_max^2) / (2 * sigma_s^2));% 原文是1-exp(-(r_max^2) / (2 * sigma_s^2))是成功的概率，那exp就是失败的概率==r>rmax
    p_fail = max(0, min(1, p_fail));
end