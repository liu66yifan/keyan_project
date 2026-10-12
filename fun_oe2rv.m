
% 函数的功能：将轨道根数(半长轴、偏心率、倾角等)转换为惯性坐标系下的卫星位置和速度。
% 函数输出：R:惯性坐标中的位置向量   V惯性坐标系中的速度向量
function [R,V] = fun_oe2rv(miu,a0,e0,i0,oumu0,w0,f0)
    % 将轨道根数转化为惯性位置速度程序
    p0 = a0*(1-e0^2);    % 半参数
    h0 = sqrt(miu*p0);   % 角动量
    % oumu0升交点赤经 w0近地点幅角 i0轨道倾角 就可以计算
    P0 = [cos(oumu0)*cos(w0)-sin(oumu0)*sin(w0)*cos(i0) 
          sin(oumu0)*cos(w0)+cos(oumu0)*sin(w0)*cos(i0) 
          sin(w0)*sin(i0)];                            
    Q0 = [-cos(oumu0)*sin(w0)-sin(oumu0)*cos(w0)*cos(i0)
          -sin(oumu0)*sin(w0)+cos(oumu0)*cos(w0)*cos(i0)
           cos(w0)*sin(i0)];
    R = p0/(1+e0*cos(f0))*cos(f0)*P0+p0/(1+e0*cos(f0))*sin(f0)*Q0;%惯性坐标系下主星近地点位置
    V = -h0/p0*sin(f0)*P0+h0/p0*(e0+cos(f0))*Q0;                  %惯性坐标系下主星近地点初始速度
end


