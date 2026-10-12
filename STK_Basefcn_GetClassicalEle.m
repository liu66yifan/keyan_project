
% 函数返回值：从STK中获取卫星的经典元素(半长轴、偏心率、升交点赤经、轨道倾角、近地点幅角和均匀近点角)
% 结果存储在ClassicalEleData结构体里面
function [ClassicalEleData] = STK_Basefcn_GetClassicalEle(satellite,Time_start,Time_end,time_step)

    % satellite = stk卫星中的handle句柄;
    % Time_start = 开始时间（数字，单位为秒）
    % Time_end = 结束时间（数字，单位为秒）
    % time_step = 时间间隔（数字，单位为秒）

    satAcc = satellite.DataProviders.Item('Classical Elements').Group.Item('TrueOfDate').Exec(Time_start,Time_end, time_step);
    ClassicalEleData.Axis = cell2mat(satAcc.DataSets.GetDataSetByName('Semi-major Axis').GetValues);        
    ClassicalEleData.Ecc = cell2mat(satAcc.DataSets.GetDataSetByName('Eccentricity').GetValues);
    ClassicalEleData.RAAN = cell2mat(satAcc.DataSets.GetDataSetByName('RAAN').GetValues);
    ClassicalEleData.Inc = cell2mat(satAcc.DataSets.GetDataSetByName('Inclination').GetValues);
    ClassicalEleData.w = cell2mat(satAcc.DataSets.GetDataSetByName('Arg of Perigee').GetValues);
    ClassicalEleData.M = cell2mat(satAcc.DataSets.GetDataSetByName('Mean Anomaly').GetValues);
    ClassicalEleData.Time = datetime(cell2mat(satAcc.DataSets.GetDataSetByName('Time').GetValues), 'InputFormat','dd MMM yyyy HH:mm:ss.SSSSSSSSS','Locale', 'en_US');

end

