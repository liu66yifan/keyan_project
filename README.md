# 星间链路与拓扑仿真项目说明

## 项目概况

本项目包含 STK 星座数据提取、无人机到卫星的接入与速率计算、星间链路拓扑构造、路径规划、链路/节点失效仿真和结果绘图。代码由多代实验脚本组成，目前没有统一的项目配置文件。许多 `.m` 文件是脚本，依赖 MATLAB 基础工作区中的变量和当前目录。

仓库中的目录名 `Intialization`、`failue`、`node_failue` 是实际拼写。根目录没有 `DataFromSTK.mat`；`main.m` 或 `main_2.m` 运行后会在当前目录生成该文件。仓库里另外有 `Algorithm/`、`create_diagram/` 下的同名 MAT 文件，不能仅凭文件名认定它们与当前参数匹配。

## 目录与文件地图

### 根目录

- 入口/基础函数：`main.m`、`main_2.m`、`fun_GetWalker.m`、`fun_oe2rv.m`、`fun_SetCoverageDeltaStar.m`、`fun_SetSatStructure.m`
- STK 函数：`STK_Basefcn_CreateFacility.m`、`STK_Basefcn_CreateSatellite_J2.m`、`STK_Basefcn_GetAccess.m`、`STK_Basefcn_GetClassicalEle.m`、`STK_Basefcn_GetFixed.m`、`STK_Basefcn_GetFixedLLA.m`、`STK_Basefcn_GetJ2000.m`、`STK_Basefcn_SetDefaultUnit.m`、`STK_Basefcn_SetEpoch.m`、`STK_Basefcn_Time2String.m`
- MAT 数据：`ISL_AllSat_Dynamic_Topology_Result.mat`、`ISL_AllSat_Static_Topology_Result.mat`、`ISL_AllSat_WindowGated_Result.mat`、`ISL_Distance_Matrix.mat`、`全时刻可连接卫星数据.mat`、`无人机-卫星全时刻距离数据.mat`、`无人机-卫星速率数据_result3.mat`、`无人机-卫星速率数据_result_test.mat`
- 其他：`OISL_Failure_Probability.png`

### `Algorithm/`

- `DataFromSTK.mat`
- `Dijkstras-Algorithm-in-MATLAB-main/`：`dijkstra.m`、`graph_search.m`、`rmpr_build_waterfill_inputs_verbose.m`、`search_L_shortest_edge_disjoint_paths.m`、`solve_routing_optimization.m`、`test_01.m`、`README.md`、`graph representation.pdf`

### `create_diagram/`

`create_diagram_dynamic.m`、`create_diagram02.m`、`extra_sta_uav_access.m`、`extra_sta_xyz.m`、`DataFromSTK.mat`、`说明.txt`

### `Intialization/`

`extra_sta_uav_access1.m`、`extra_sta_xyz2.m`、`cal_elevation3.m`、`cal_distance4.m`、`cal_rate5.m`、`static_topology6.m`、`create_static_750_7.m`、`version_01_8.m`、`save_project.m`、`说明.txt`

### `Simulation/`

- `all_time/`：`overlap_test.m`、`result.m`、`rmpr_build_waterfill_inputs_multi_source_overlap.m`、`run_backup_5path.m`、`run_backup_5path_test.m`、`run_single_path.m`、`search_MultiSource_overlap_paths.m`、`single_path_test.m`
- `paper result/`：`add_node_link.m`、`average_pdr_delay_jump.m`、`fig_exd_node.m`、`add_node_link.asv`
- `Result/`：`Copy_of_test_pdr.m`、`delay_pdr.m`、`test_pdr.m`
- `version_01/`：`failue_node_link.m`、`link_failue.m`、`link_node_fail.m`、`pdr_link_node_fai.m`、`rmpr_build_waterfill_inputs_verbose_temp.m`、`search_L_shortest_edge_disjoint_paths_from_sat.m`、`test.m`、`version_01.m`
- `version_03/`：`build_weighted_cost_matrix.m`、`final_all_T.m`、`rmpr_build_waterfill_inputs_multi_source.m`、`search_MultiSource_disjoint_paths.m`、`Test_rmpr_waterfill.m`、`Test_weighted_cost_matrix.m`、`test01.m`、`weight_pfail.m`
- `version6/`：`average_tasksize.m`、`final.m`、`q_weibuil.m`、`final.asv`

### `topology/static/`

`create_static_750.m`、`find_L_750_static.m`、`static_final_750.m`、`static_topology.m`

### `failure/`

- 根层：`Oisl_failue.m`、`Oisl_failue_static.m`、`oisl_failure_prob.m`、`oisl_failure_prob_var.m`、`test01.m`
- `node_failue/`：`build_weighted_cost_matrix_node.m`、`calc_qWeibull_node_fail_prob.m`
- `node_failue/make_node_enviroment/`：`build_hotspot_aging_from_paths.m`、`build_weighted_cost_matrix_node_addon.m`、`solve_rmpr_once_from_cost.m`、`test.m`、`untitled.m`

### `OutputData/`

36 个 Excel 文件：`ClassicalEleData_1_1.xlsx` 至 `ClassicalEleData_1_12.xlsx`、`J2000Data_1_1.xlsx` 至 `J2000Data_1_12.xlsx`、`LLAData_1_1.xlsx` 至 `LLAData_1_12.xlsx`。

未发现项目级 `.prj`、`.ini`、`.cfg`、`.json`、`.yaml` 或 `.xml` 配置文件。仓库清单中有 84 个 `.m`、10 个 `.mat`、2 个 `.txt`、2 个 `.asv`、36 个 `.xlsx`、1 个 `.md`、1 个 `.pdf`、1 个 `.png` 文件。

## 主流程与入口

### 1. STK 数据采集：`main_2.m`

`main_2.m:27-32` 连接 `STK11.Application` 并建立场景；`main_2.m:58-73` 配置 Walker 星座；`main_2.m:80-91` 计算卫星到设施的 Access，并提取卫星状态数据。

当前脚本参数：

- 仿真时间：2025-12-20 10:00 至 13:30
- 采样间隔：15 秒，共 841 个时刻
- 星座：25 个轨道面，每面 40 颗，共 1000 颗卫星
- 轨道高度约 550 km，倾角 53°
- 输出当前工作目录下的 `DataFromSTK.mat`，变量为 `J2000Data`、`ClassicalEleData`、`FixedLLAData`

`main.m` 也是 STK 入口，但配置为 3 个轨道面、每面 4 颗卫星、1.5 小时，适合小规模调试。它创建 UAV 航线，但当前代码没有提取该航线的 UAV 状态序列。

### 2. UAV 接入、距离和速率

下游脚本并未使用 `main_2.m` 中的 UAV 航线，而是用硬编码的固定无人机坐标计算接入与距离。

1. `Intialization/cal_elevation3.m`：读取 `DataFromSTK.mat` 的 `FixedLLAData`，按仰角阈值 10°筛选卫星，保存 `全时刻可连接卫星数据.mat` 中的 `results`。
2. `Intialization/cal_distance4.m`：计算固定 UAV 位置到每颗卫星的距离，保存 `无人机-卫星全时刻距离数据.mat` 中的 `result2`。
3. `Intialization/cal_rate5.m`：读取 `results` 与 `result2`，按链路预算计算速率，每时刻保留速率最高的最多 4 颗卫星，保存 `无人机-卫星速率数据_result3.mat` 中的 `result3`。

前两步都依赖 `DataFromSTK.mat`，彼此独立；第 3 步依赖前两步。

### 3. 星间拓扑

`Intialization/static_topology6.m` 读取 `J2000Data`，每颗卫星连接同轨道前后邻星，以及相邻轨道面相同位置的卫星。拓扑邻居规则固定，但链路距离逐时刻按坐标重算。输出 `ISL_AllSat_Static_Topology_Result.mat` 中的 `result_static`。

`create_diagram/create_diagram_dynamic.m` 将 `result3` 的地面站到接入卫星链路和 `result_static` 的星间链路组合成当前时刻的距离邻接矩阵。矩阵中不可用链路通常为 `Inf`，可用链路的权重为距离。

### 4. 仿真入口

- `Simulation/paper result/average_pdr_delay_jump.m`：比较单路径、Node-weight RMPR、RMPR、Overlap multi-path，统计平均时延、PDR 和跳数。默认 100 个时刻，每时刻 400 次 Monte Carlo，`n=2,3,4`。
- `Simulation/paper result/add_node_link.m`：比较 5 种方法，包含链路/热点节点失效实验。默认 100 个时刻，每时刻 400 次 Monte Carlo。
- `Simulation/paper result/fig_exd_node.m`：热点节点与路径避让实验及图形展示。默认 3 个时刻，每时刻 400 次 Monte Carlo。

以上脚本需要 `result_static` 已存在于工作区；若工作区没有 `result3`，脚本会尝试从速率 MAT 文件加载。它们逐时刻调用构图函数，不要求预先生成整个时间序列的邻接矩阵。绘图主要出现在 MATLAB 图窗中，没有看到这些论文脚本将图保存到磁盘的调用。

## 八个初始化脚本的职责

| 顺序 | 文件 | 输入与输出 | 备注 |
|---|---|---|---|
| 1 | `extra_sta_uav_access1.m` | 输入工作区 `StructAccess`，输出 `ground_sat_core_info`：卫星编号、Access 起止时间、AER 距离。 | 实际处理卫星到设施的 Access，不是下游速率计算所用 UAV 接入。 |
| 2 | `extra_sta_xyz2.m` | 输入 `ground_sat_core_info`、`J2000Data`，输出 `all_sat_interval_data`。 | 截取卫星 J2000 坐标，不保存文件；仓库其他脚本没有引用这个输出。 |
| 3 | `cal_elevation3.m` | 输入 `DataFromSTK.mat`，输出并保存 `results`。 | 设定 `all_times`，供当前第 4、6 步隐式使用。 |
| 4 | `cal_distance4.m` | 输入 `DataFromSTK.mat` 和工作区 `all_times`，输出并保存 `result2`。 | 单独运行时可能因 `all_times` 未定义而失败。 |
| 5 | `cal_rate5.m` | 输入 `results`、`result2`，输出并保存 `result3`。 | 速率公式及 `L=4` 写死在脚本中。 |
| 6 | `static_topology6.m` | 输入 `DataFromSTK.mat`、工作区 `all_times`，输出并保存 `result_static`。 | 拓扑计算本身可独立于第 3–5 步，但当前脚本依赖 `all_times`。 |
| 7 | `create_static_750_7.m` | 输入 `result3`、`result_static` MAT 文件，输出工作区 `adj_matrices_static`。 | 预分配约 6.7 GB 的 double 三维数组；不保存；论文脚本不需要它。 |
| 8 | `version_01_8.m` | 输入工作区 `result_static` 和磁盘上的 `result3`，输出单时刻 `adj_matrices_static_temp`。 | 只处理 `t_idx=1`，适合检查，不是全时序步骤。 |

## 推荐运行顺序

在 MATLAB 中先将当前目录设为项目根目录并加入代码路径：

```matlab
addpath(genpath(pwd));
```

重建完整输入数据时，当前有效顺序是：

```matlab
run('main_2.m')
run(fullfile('Intialization', 'cal_elevation3.m'))
run(fullfile('Intialization', 'cal_distance4.m'))
run(fullfile('Intialization', 'cal_rate5.m'))
run(fullfile('Intialization', 'static_topology6.m'))
```

第 3 步先运行是因为它建立 `all_times`，当前第 4、6 步会读取该工作区变量。步骤 1、2 是独立的 STK Access/坐标截取辅助流程，不是论文仿真输入的必要环节。步骤 7、8 是构图检查；论文脚本本身会逐时刻构图。

运行论文结果示例：

```matlab
load('ISL_AllSat_Static_Topology_Result.mat', 'result_static');
run(fullfile('Simulation', 'paper result', 'average_pdr_delay_jump.m'))
```

其他实验入口可替换为 `add_node_link.m` 或 `fig_exd_node.m`。这些脚本计算量较大；本说明整理期间没有运行它们。

## 核心数据和变量约定

- `J2000Data{sat_id}`：每颗卫星的位置/速度及 `Time` 时间序列，包含 `X/Y/Z`、`VX/VY/VZ`。
- `FixedLLAData{sat_id}`：每颗卫星的 `Time`、`Lat`、`Lon`、`Rad` 序列。
- `ClassicalEleData{sat_id}`：经典轨道根数时间序列。
- `results(t)`：每个时刻的可接入卫星列表，含 `time`、`connect_sat_count`、`sat_list`；卫星条目含 `sat_id`、`elevation`。
- `result2(t)`：距离记录，含 `time`、`sat_distances`，距离按卫星 ID 索引。
- `result3(t)`：仿真使用的接入候选，含 `time`、`sat_ids`、`rates_Mbps`、`dists_m`。
- `result_static(t).sat_data`：每颗卫星一行、9 列，记录卫星 ID 和两条跨轨道、两条同轨道链路的邻居 ID/距离。
- 节点标签中地面站为 0，卫星 ID 为 1–1000；邻接矩阵下标使用 `node_id + 1`。
- `t_idx` 是离散样本索引。`time_diedai=100` 表示只使用前 100 个样本，不自动代表 100 秒或完整 3.5 小时。
- `Sources` 是当前接入卫星；`target_sat` 默认 402；`L` 常为 4；`M_bits` 常为 `1e6` bit。
- `alpha`、`omega` 控制路径代价/节点权重；`sigma_s0` 是链路失效模型参数。

节点编号需谨慎：部分故障和热点变量注释称 `0based`，代码又会加 1 转成 MATLAB 矩阵下标。将这些 ID 与卫星编号、路径标签混用可能产生一位偏移。

## 环境要求

- `main_2.m` 使用 Windows COM 自动化接口 `actxserver('STK11.Application')`，需要可用的 STK 11 COM 注册/许可。当前环境是否已安装并配置 STK，尚未验证。
- 当前机器能找到 MATLAB 可执行文件，但具体版本尚未核实。脚本使用 `datetime`、脚本局部函数和 `tiledlayout` 等功能，建议 MATLAB R2019b 或更新版本。
- 代码未显示明确的专用工具箱依赖；坐标转换包含自定义函数。STK COM 和 MATLAB 基础函数是必要依赖。
- 相对路径加载 MAT 文件，因此应从项目根目录运行。
- 现有 MAT 文件来自不同日期；它们的时间长度、星座参数和时间戳是否一致，尚未检查。根目录当前缺少由 `main_2.m` 生成的 `DataFromSTK.mat`。
- 没有统一配置文件；星座参数、坐标、频率、带宽、失效参数和蒙特卡洛次数均写在脚本中。

## 明显问题与后续风险

1. `static_topology6.m` 中 750/30 颗旧注释与代码的 1000 颗/40 颗参数不符；卫星 1 的注释邻居仍为 31、721、30、2，按当前参数代码连接的是 41、961、40、2。
2. `create_static_750_7.m` 名称、注释和实际 1001×1001 图不一致，且默认 `1:841` 的注释错误地说成 12 个时刻。
3. `create_diagram_dynamic.m` 代码处理 1000 颗卫星，注释和输出文本仍写 750 颗、751×751。
4. `cal_elevation3.m` 和 `cal_distance4.m` 固定 UAV 位置；`main_2.m` 建立的 UAV 航线没有进入这条数据链。
5. `main_2.m` 的设施叫 `New_York`，坐标却约为东经 116°、北纬 40°。
6. UAV 高度用 `drone_Rad - 6378137` 计算；这是地心距减赤道长半轴，是否符合给定纬度的椭球高需要核实。改变该算法会改变结果，应和纯代码重构分开。
7. `cal_elevation3.m` 对每颗星捕获所有异常后直接跳过，容易掩盖数据错误；距离脚本也将异常转成 `NaN`。
8. `cal_elevation3.m`、`cal_distance4.m` 用 datetime 完全相等进行匹配，依赖各数据时间戳严格一致。
9. 论文脚本要求 `result_static` 在工作区中，但没有自动加载它；`result3` 则有缺失时加载文件的逻辑。
10. 多个实验版本包含重复的路径、失效和绘图逻辑，存在不同目录下的测试脚本同名问题。
11. `save_project.m` 保存为 `my_data.mat`，恢复示例却加载 `all_my_data.mat`。
12. `main_2.m` 设施坐标与名称不符；`main.m` 是小规模调试配置，不应误认为主实验参数。

## 关于合并初始化脚本

可以新增一个初始化总控脚本，按顺序 `run` 现有脚本；无需合并计算代码。建议保留 `main_2.m → 初始化总控 → 仿真` 的流程，并在总控中按顺序执行仰角、距离、速率、拓扑计算。`extra_sta_uav_access1.m` 和 `extra_sta_xyz2.m` 可作为可选辅助步骤；全时序邻接矩阵和单时刻图构造也可作为可选检查。

若要求输出数值不变，首先只整理调度和依赖，不改轨道、坐标换算、链路预算、拓扑规则、失效概率、随机种子或循环顺序。`create_static_750_7.m` 的大矩阵不是论文仿真必需输入，可以不在常规流程中运行；仿真入口已经按时刻即时构图。若后续需要重构计算实现，应对比旧新 MAT 变量、维度和数值误差后再替换。
