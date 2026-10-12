% function锛欼nitialize STK epoch
% Input: BeginEpoch and EndEpoch string
% Copyright Wechat: AerospaceEngineer_Z

function FLAG = STK_Basefcn_SetEpoch( root, BeginEpoch, EndEpoch )
    %开始时间和结束时间转为stk识别的字符串
    Begin_epoch = STK_Basefcn_Time2String(BeginEpoch);  %返回20 Dec 2025 0:0:0
    End_epoch = STK_Basefcn_Time2String(EndEpoch);      %返回21 Dec 2025 0:0:0
    
    %root.CurrentScenario 当前打开的场景
    %SetTimePeriod 设置开始时间和结束时间
    root.CurrentScenario.SetTimePeriod(Begin_epoch, End_epoch);
    
    %参考起点---仿真起点
    root.CurrentScenario.Epoch = Begin_epoch;
    
    %动画重置到时间轴起点
    FLAG =root.ExecuteCommand('Animate * Reset'); 
end

