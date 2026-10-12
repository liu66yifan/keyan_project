%  Access from Sat to Facility

% 函数的返回值：所有卫星的access=StructAccess结构体
function StructAccess = STK_Basefcn_GetAccess(scenario, AllSatHandle, AllFacility, DataStep)
    % 创建一个datetime对象---用于后续计算访问的开始时间和结束时间的秒数差
    Begin_epoch = datetime(scenario.StartTime, 'InputFormat','dd MMM yyyy HH:mm:ss.SSSSSSSSS','Locale', 'en_US');

    Iter = 1;
    for n = 1:size(AllSatHandle,1)    
        % 获取当前卫星的索引
        satellite_handle = AllSatHandle(n,1);
        
        % 遍历所有地面站
        for i = 1:size(AllFacility,1)   % ?

            % KD_Station 是当前正在处理的地面站
            KD_Handle = AllFacility(i,1);

            % 地面站和卫星尝试连接access
            access = satellite_handle.GetAccessToObject(KD_Handle);
            access.ComputeAccess;%计算卫星与地面站之间的可访问时间段。考虑卫星的轨道、地面站的坐标以及地球的遮挡，计算出卫星何时可以看到地面站

            % DataProviders可以得到卫星和地面站之间可访问的数据 Exec可见性
            accessDP = access.DataProviders.Item('Access Data').Exec(scenario.StartTime, scenario.StopTime);

            % AccessOrNot检查是否存在可访问数据，
            AccessOrNot = accessDP.DataSets.RowCount;

            % 如果AccessOrNot大于0：表示卫星在该时间段内可以访问该地面站
            if AccessOrNot>0
                % 获取访问的开始时间、结束时间、持续时间
                accessStartTimes = cell2mat(accessDP.DataSets.GetDataSetByName('Start Time').GetValues);
                accessStopTimes = cell2mat(accessDP.DataSets.GetDataSetByName('Stop Time').GetValues);
                accessDuration = cell2mat(accessDP.DataSets.GetDataSetByName('Duration').GetValues);

                % 遍历每个访问时间段
                for p = 1:size(accessStartTimes,1)
                    
                    %StructAccess保存数据
                    StructAccess(Iter,1).Sat = n;          % 当前卫星索引
                    StructAccess(Iter,1).Facility = i;     % 当前地面站
                    
                    % StartTimeDate和StopTimeDate 转为datetime格式
                    StructAccess(Iter,1).StartTimeDate = datetime(accessStartTimes(p,:), 'InputFormat', 'dd MMM yyyy HH:mm:ss.SSSSSSSSS', 'Locale', 'en_US');
                    StructAccess(Iter,1).StopTimeDate = datetime(accessStopTimes(p,:), 'InputFormat', 'dd MMM yyyy HH:mm:ss.SSSSSSSSS', 'Locale', 'en_US');
                    
                    % StartTimeSec和StopTimeSec 转为datetime格式
                    StructAccess(Iter,1).StartTimeSec = seconds( datetime(accessStartTimes(p,:), 'InputFormat', 'dd MMM yyyy HH:mm:ss.SSSSSSSSS', 'Locale', 'en_US')-Begin_epoch );
                    StructAccess(Iter,1).StopTimeSec = seconds (datetime(accessStopTimes(p,:), 'InputFormat', 'dd MMM yyyy HH:mm:ss.SSSSSSSSS', 'Locale', 'en_US')-Begin_epoch );
                    StructAccess(Iter,1).Duration = accessDuration(p,1);

                    % 获取AER Data数据(Azimuth, Elevation, Range)
                    accessAER = access.DataProviders.Item('AER Data').Group.Item('Default').Exec(accessStartTimes(p,:), accessStopTimes(p,:), DataStep);
                    StructAccess(Iter,1).accessAER_Time = accessAER.DataSets.GetDataSetByName('Time').GetValues;
                    StructAccess(Iter,1).accessAER_Azimuth = accessAER.DataSets.GetDataSetByName('Azimuth').GetValues;
                    StructAccess(Iter,1).accessAER_Elevation = accessAER.DataSets.GetDataSetByName('Elevation').GetValues;
                    StructAccess(Iter,1).accessAER_Range = accessAER.DataSets.GetDataSetByName('Range').GetValues;

                    Iter = Iter+1;
                end
            end
            % 清除当前的访问数据，释放资源
            access.ClearAccess();
            access.RemoveAccess();

            % 清空临时存储的开始和结束时间
            accessStartTimes = [];
            accessStopTimes = [];
        end
    end
end