% Copyright Wechat: AerospaceEngineer_Z

function [AllEle0,AllRV0] = fun_GetWalker(Ele0,T,P,F)
% Ele0：基准卫星轨道根数
% 确定每个卫星的轨道根数

% 常数
miu = 398600.4418e9;
dtr = pi/180;
rtd = 180/pi;
% Ele0：单位m，角度
N = T/P;
a0 = Ele0(1,1);
e0 = Ele0(2,1);
i0 = Ele0(3,1);
Omega0 = Ele0(4,1);
w0 = Ele0(5,1);
f0 = Ele0(6,1);
for i = 1:P    % 轨道面循环
    for j = 1:N  % 面内卫星循环
        Omega_ij = mod(Omega0 + 360/P*(i-1) ,360);   % 升交点赤经更新
        f_ij = mod(f0 + 360/N*(j-1) + 360/T*F*(i-1),360);  % 相位更新
        AllEle0{i,j} = [a0,e0,i0, Omega_ij, w0, f_ij];  % 轨道根数
        [R,V] = fun_oe2rv(miu, a0,e0,i0*dtr, Omega_ij*dtr, w0*dtr, f_ij*dtr);   % 卫星位置速度
        AllRV0{i,j} = [R.',V.'];
    end
end