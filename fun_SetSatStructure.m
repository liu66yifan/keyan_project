%% function: set Constellation and Get Walker Structure
% Flag_DS: 1->Delta, 0->Star

% 函数功能是用一颗"基准卫星"的轨道根数,按Walker星座规则生成一整套星座(多轨道面、多卫星)，返回一个结构体
% 函数返回值：Structure_Walker结构体(卫星的轨道根数 T*6、所有卫星的位置速度、卫星的名称、卫星总数)

function Structure_Walker = fun_SetSatStructure(SeedSat, P, N, F, FirstName, Flag_DS)

    T = P * N;   % 卫星总数 = 轨道数 * 每个轨道上的卫星数
    if Flag_DS   
        [AllEle, AllRV] = fun_GetWalkerDelta(SeedSat, T, P, F);
    else
        [AllEle, AllRV] = fun_GetWalkerStar(SeedSat, T, P, F);
    end

    % 打包
    iter = 1;
    for i = 1:P
        for j = 1:N
            % Structure_Walker
            Structure_Walker.Ele(iter,:) = AllEle{i,j};
            Structure_Walker.RV(iter,:) = AllRV{i,j};
            %给这个轨道这个卫星命名
            Structure_Walker.Name(iter,:) = FirstName + "_P" + num2str(i) + "_N" + num2str(j) ;
            iter = iter+1;
        end
    end
    Structure_Walker.TNum = T;    % Total Sat
end

%% function: GetWalkerDelta elements
% 这个函数用于生成 Walker Delta 星座（RAAN 均匀分布 360/P）
% 函数输入：种子卫星、卫星总数、轨道数、每个轨道卫星数
% 函数输出：AllEle0 AllRV0
% 其中AllEle0返回的是一个 卫星总数 * 6（a, e, i, Ω, ω, f）     的矩阵
% 其中AllRV0 返回的是一个 卫星总数 * 6 (Rx, Ry, Rz, Vx, Vy, Vz) 的矩阵
% 假设两个轨道面，每个轨道面3个卫星
% AllEle0 = 2 * 3的数组，每个ij存放卫星的轨道根数 [a, e, i, Ω, ω, f]
% AllRV0  = 2 * 3的数组，每个 i,j 元素存放该卫星的 [位置, 速度] 向量 [R,V]
% Rx = R(1) Ry = R(2) Rz = R(3)

function [AllEle0,AllRV0] = fun_GetWalkerDelta(Ele0,T,P,F)

    % 
    miu = 398600.4418e9;% 引力参数
    dtr = pi/180;
    rtd = 180/pi;
    % Ele0：传入的种子卫星：是一个列向量
    N = T/P;
    a0 = Ele0(1,1);     % 半长轴
    e0 = Ele0(2,1);     % 偏心率
    i0 = Ele0(3,1);     % 倾角
    Omega0 = Ele0(4,1); % RAAN
    w0 = Ele0(5,1);     % 近地点幅角
    f0 = Ele0(6,1);     % 真近点角
    
    % 两层循环得到每个卫星
    for i = 1:P 
        for j = 1:N   
            % 轨道面RAAN分布
            Omega_ij = mod(Omega0 + 360/P*(i-1) ,360);
            % 面内卫星相位(真近点角)
            f_ij = mod(f0 + 360/N*(j-1) + 360/T*F*(i-1),360);
            % 每个卫星位置坐标 AllEle0
            AllEle0{i,j} = [a0,e0,i0, Omega_ij, w0, f_ij]; 
            % 将轨道根数转为惯性坐标系下的卫星位置和速度
            [R,V] = fun_oe2rv(miu, a0,e0,i0*dtr, Omega_ij*dtr, w0*dtr, f_ij*dtr); 
            AllRV0{i,j} = [R.',V.'];
        end
    end

end

%% function: GetWalkerStar elements
function [AllEle0, AllRV0] = fun_GetWalkerStar(Ele0,T,P,F)
    % Get Walker Star Elements

    % 甯告暟
    miu = 398600.4418e9;
    dtr = pi/180;
    rtd = 180/pi;
    % Ele0锛氬崟浣峬锛岃搴?
    N = T/P;
    a0 = Ele0(1,1);
    e0 = Ele0(2,1);
    i0 = Ele0(3,1);
    Omega0 = Ele0(4,1);
    w0 = Ele0(5,1);
    f0 = Ele0(6,1);
    for i = 1:P    % 杞ㄩ亾闈㈠惊鐜?
        for j = 1:N  % 闈㈠唴鍗槦寰幆
            Omega_ij = mod(Omega0 + 180/P*(i-1) ,360);   % 鍗囦氦鐐硅丹缁忔洿鏂?
            f_ij = mod(f0 + 360/N*(j-1) + 360/T*F*(i-1),360);  % 鐩镐綅鏇存柊
            AllEle0{i,j} = [a0,e0,i0, Omega_ij, w0, f_ij];  % 杞ㄩ亾鏍规暟
            [R,V] = fun_oe2rv(miu, a0,e0,i0*dtr, Omega_ij*dtr, w0*dtr, f_ij*dtr);   % 鍗槦浣嶇疆閫熷害
            AllRV0{i,j} = [R.',V.'];
        end
    end

end