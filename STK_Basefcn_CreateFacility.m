% 函数功能是创建多个Facility，站点设置地理位置和高度

% 函数输入：Struct_Facility、MinElevation
% 函数输出：设置好参数约束的地面站矩阵

function AllFacility = STK_Basefcn_CreateFacility(root, Struct_Facility, MinElevation)
% IAgStkObjectRoot root: STK Object Model Root
    for i = 1:Struct_Facility.Num
        facility = root.CurrentScenario.Children.New('eFacility', Struct_Facility.Name(i));

        % Facility 设置 经度 纬度 高度(初始化0，后面用设置的高度覆盖这个数据)
        facility.Position.AssignGeodetic(Struct_Facility.Lat(i),Struct_Facility.Lon(i),0) % Latitude, Longitude, Altitude

        % 是否使用地形高度
        facility.UseTerrain = false;

        % 取我们设定的高度来覆盖
        facility.HeightAboveGround = Struct_Facility.High(i);   % km

        % 获取并添加访问约束对象?
        facConstraints = facility.AccessConstraints;

        % 对象中添加约束最小仰角对象
        k = facConstraints.AddConstraint('eCstrElevationAngle');

        % 启用最小仰角限制?
        k.EnableMin = 1;

        % 设定最小仰角阈值
        k.Min = MinElevation;

        % 把设置好的每个地面站一次放入结果矩阵
        AllFacility(i,1) = facility;
    end
end