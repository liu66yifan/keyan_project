% function锛欵poch to string
% Input: Epoch structure
% Ouput: String_epoch string
% 把自定义的时间结构体Epoch转换成STK能识别的时间字符串



%函数输入是 Epoch.year=2025,month=12, day=20, hour=0, minute=0, second=0
%函数输出是 20 Dec 2025 0:0:0
function String_epoch = STK_Basefcn_Time2String(Epoch)
    monthstr = Month2STK(Epoch.month);
    start.stk1 = num2str(Epoch.day);     %start是临时变量结构体，stk1是字段，这里是day，下面同理
    start.stk2 = monthstr;
    start.stk3 = num2str(Epoch.year);
    start.stk4 = num2str(Epoch.hour);
    start.stk5 = num2str(Epoch.minute);
    start.stk6 = num2str(Epoch.second);
    %最后把时间拼接起来
    String_epoch = strcat(start.stk1,32,start.stk2,32,start.stk3,32,start.stk4,':',start.stk5,':',start.stk6);
end

%把数字月份1-12转成时间字符串jan-Dec
function [ STK_Mounth ] = Month2STK( mounth )
    if(mounth==1) 
        STK_Mounth = 'Jan';
    elseif(mounth==2)
        STK_Mounth = 'Feb';
    elseif(mounth==3)
        STK_Mounth = 'Mar';
    elseif(mounth==4)
        STK_Mounth = 'Apr';
    elseif(mounth==5)
        STK_Mounth = 'May';    
    elseif(mounth==6)
        STK_Mounth = 'Jun';      
    elseif(mounth==7)
        STK_Mounth = 'Jul';       
    elseif(mounth==8)
        STK_Mounth = 'Aug';    
    elseif(mounth==9)
        STK_Mounth = 'Sep';      
     elseif(mounth==10)
        STK_Mounth = 'Oct';     
    elseif(mounth==11)
        STK_Mounth = 'Nov';  
    elseif(mounth==12)
        STK_Mounth = 'Dec';  
    else
        disp('鏈堜唤杈撳叆閿欒')
    end
end


