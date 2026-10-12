function [FixedData] = STK_Basefcn_GetFixed(satellite,Time_start,Time_end,time_step)

% satellite = stk卫星中的handle句柄;
% Time_start = 开始时间（数字，单位为秒）
% Time_end = 结束时间（数字，单位为秒）
% time_step = 时间间隔（数字，单位为秒）

satPosDP = satellite.DataProviders.Item('Cartesian Position').Group.Item('Fixed').Exec(Time_start,Time_end,time_step);
FixedData.Time = datetime(cell2mat(satPosDP.DataSets.GetDataSetByName('Time').GetValues), 'InputFormat','dd MMM yyyy HH:mm:ss.SSSSSSSSS','Locale', 'en_US');
FixedData.X = cell2mat(satPosDP.DataSets.GetDataSetByName('x').GetValues);
FixedData.Y = cell2mat(satPosDP.DataSets.GetDataSetByName('y').GetValues);
FixedData.Z = cell2mat(satPosDP.DataSets.GetDataSetByName('z').GetValues);
satPosDP = satellite.DataProviders.Item('Cartesian Velocity').Group.Item('Fixed').Exec(Time_start,Time_end,time_step);
FixedData.VX = cell2mat(satPosDP.DataSets.GetDataSetByName('x').GetValues);
FixedData.VY = cell2mat(satPosDP.DataSets.GetDataSetByName('y').GetValues);
FixedData.VZ = cell2mat(satPosDP.DataSets.GetDataSetByName('z').GetValues);

end