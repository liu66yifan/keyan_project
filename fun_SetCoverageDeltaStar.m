function fun_SetCoverageDeltaStar(Coverage_Handle, P_D, N_D)

% Assign satellite to Region
% Add Delta
for i = 1:P_D
    for j = 1:N_D
        SatName =  ['Satellite/C_P', num2str(i), '_N', num2str(j)];
        Coverage_Handle.AssetList.Add(SatName);
    end
end


end