%单位偏好设置

function STK_Basefcn_SetDefaultUnit(root)
    root.UnitPreferences.SetCurrentUnit('Distance', 'm');       %距离单位
    root.UnitPreferences.SetCurrentUnit('DateFormat', 'UTCG');  %日期格式UTCG,通常是20 Dec 2025 00:00:00.000
end