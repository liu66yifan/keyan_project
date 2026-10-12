% satellite = 
% Time_start = 
% Time_end = 
% time_step = 

% 这个函数作用：从STK获取卫星在J2000坐标系下的位置和速度 并存到J2000Data结构体中
function J2000Data = STK_Basefcn_GetJ2000(satellite, Time_start, Time_end, time_step)

    % DataProviders获取卫星在 J2000 坐标系下的位置 Exec执行计算
    satPosDP = satellite.DataProviders.Item('Cartesian Position').Group.Item('J2000').Exec(Time_start, Time_end, time_step);
    
    % 获取数据集中名为 Time 的数据集（卫星的位置数据对应的时间点）
    J2000Data.Time = datetime(cell2mat(satPosDP.DataSets.GetDataSetByName('Time').GetValues), 'InputFormat','dd MMM yyyy HH:mm:ss.SSSSSSSSS','Locale', 'en_US');
    
    % cell2mat 将数据从元胞数组转换为数值数组(坐标)
    J2000Data.X = cell2mat(satPosDP.DataSets.GetDataSetByName('x').GetValues);
    J2000Data.Y = cell2mat(satPosDP.DataSets.GetDataSetByName('y').GetValues);
    J2000Data.Z = cell2mat(satPosDP.DataSets.GetDataSetByName('z').GetValues);
    
    % cell2mat 将数据从元胞数组转换为数值数组(速度)
    satPosDP = satellite.DataProviders.Item('Cartesian Velocity').Group.Item('J2000').Exec(Time_start,Time_end,time_step);
    
    J2000Data.VX = cell2mat(satPosDP.DataSets.GetDataSetByName('x').GetValues);
    J2000Data.VY = cell2mat(satPosDP.DataSets.GetDataSetByName('y').GetValues);
    J2000Data.VZ = cell2mat(satPosDP.DataSets.GetDataSetByName('z').GetValues);

end