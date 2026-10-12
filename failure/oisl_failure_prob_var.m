function p_fail = oisl_failure_prob_var(z_m, sigma_s2)
% OISL failure probability under fixed jitter variance
% Based on the paper "Channel Modeling and Rate Analysis of OISL"
%
% Input:
%   z_m      : link distance (m)
%   sigma_s2 : pointing jitter variance (m^2)
%
% Output:
%   p_fail   : failure probability in [0,1]

    %% ===== 0) check =====
    if z_m <= 0
        error('z_m must be > 0 (meters).');
    end

    if sigma_s2 <= 0
        error('sigma_s2 must be > 0 (m^2).');
    end

    %% ===== 1) Fixed parameters =====
    w0      = 0.1;
    wd      = 0.1;
    f_laser = 200e12;
    h_PL    = 0.9;
    eta     = 0.5;
    P_T     = 0.5;
    p_th    = 1e-6;

    %% ===== 2) physical constants and beam parameters =====
    c = 299792458;
    lambda = c / f_laser;
    theta  = lambda / (pi * w0);
    w_z    = z_m * tan(theta);

    %% ===== 3) channel threshold h_th =====
    h_th = p_th / (h_PL * eta * P_T);

    %% ===== 4) fixed jitter standard deviation from variance =====
    sigma_s = sqrt(sigma_s2);

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
    p_fail = exp(-(r_max^2) / (2 * sigma_s2));
    p_fail = max(0, min(1, p_fail));
end
