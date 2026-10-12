% Input: STK root, Struct_Satellite
% Ouput: AllSatHandle
% J2摄动传播器：用于考虑地球的 J2 摄动效应

% 函数返回值：AllSatHandle == 返回所有卫星的句柄
function AllSatHandle = STK_Basefcn_CreateSatellite_J2(root, Struct_Satellite)

    for i = 1:Struct_Satellite.TNum
        satellite = root.CurrentScenario.Children.New('eSatellite', Struct_Satellite.Name(i));
        satellite.SetPropagatorType('ePropagatorJ2Perturbation');
        
        propagator = satellite.Propagator;%获取卫星对象的传播器（Propagator）对象
        % 设置卫星的初始状态
        % 使用 J2000 坐标系统
        % 提取每颗卫星的状态向量Rx Ry Rz Vx Vy Vz并赋给传播器的初始状态
        propagator.InitialState.Representation.AssignCartesian('eCoordinateSystemJ2000', ...
            Struct_Satellite.RV(i,1),Struct_Satellite.RV(i,2),Struct_Satellite.RV(i,3),...
            Struct_Satellite.RV(i,4),Struct_Satellite.RV(i,5),Struct_Satellite.RV(i,6));
        % 运行传播器
        propagator.Propagate();
        % 将每颗卫星的句柄（satellite 对象）存入 AllSatHandle 数组中,并返回所有卫星的句柄
        AllSatHandle(i,1) = satellite;
    end

end
