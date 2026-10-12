% 固定坐标系下的坐标


% 函数返回值：FixedLLAData结构体(经度纬度高度)
function [FixedLLAData] = STK_Basefcn_GetFixedLLA(satellite,Time_start,Time_end,time_step)
    % satellite = 
    % Time_start = 
    % Time_end = 
    % time_step = 
    satPosDP = satellite.DataProviders.Item('LLR State').Group.Item('Fixed').Exec(Time_start,Time_end,time_step);
    FixedLLAData.Time = datetime(cell2mat(satPosDP.DataSets.GetDataSetByName('Time').GetValues), 'InputFormat','dd MMM yyyy HH:mm:ss.SSSSSSSSS','Locale', 'en_US');
    FixedLLAData.Lat = cell2mat(satPosDP.DataSets.GetDataSetByName('Lat').GetValues);
    FixedLLAData.Lon = cell2mat(satPosDP.DataSets.GetDataSetByName('Lon').GetValues);
    FixedLLAData.Rad = cell2mat(satPosDP.DataSets.GetDataSetByName('Rad').GetValues);

end