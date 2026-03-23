# GNSS/INS 目标一代码使用指南（ins1-onlyguide）

## 1. 适用范围
本指南对应当前分支 `ins1-onlyguide`，用于复现论文第一章目标：

- 基于 GNSS/INS 紧组合的 INS 辅助多普勒一致性检测；
- 检测到欺骗后执行导航层抑制（INS 接管）并输出导航结果与图像。

默认推荐使用 `ins1` 配置进行论文图和结果复现。

## 2. 环境要求
- MATLAB（建议 R2021a 或以上）
- Windows 路径环境（脚本中含绝对路径默认值）
- 仓库根目录下可访问以下数据文件：
  - `cleandynamic_400s.mat`
  - `ds5_400s.mat`
  - `ds6_400s.mat`

INS 轨迹默认路径在代码中为：
- `E:\fgi_result\ins_simulation\ins1_trj.mat`
- `E:\fgi_result\ins_simulation\ins2_trj.mat`
- `E:\fgi_result\ins_simulation\ins3_trj.mat`

如果你的本机路径不同，请修改 `deepIntegration/run_goal1_unified_400s_insonly.m` 中 `trjFile` 对应路径。

## 3. 快速开始（ins1）
在 MATLAB 当前目录切到仓库根目录后执行：

```matlab
addpath('deepIntegration');
```

### 3.1 单场景运行
```matlab
% ds5 + ins1（离线重放）
s_ds5 = run_goal1_unified_400s_insonly('ds5','ins1', struct('useOfflineReplay',1,'msToProcess',399000));

% ds6 + ins1（离线重放）
s_ds6 = run_goal1_unified_400s_insonly('ds6','ins1', struct('useOfflineReplay',1,'msToProcess',399000));
```

### 3.2 三场景总览图
```matlab
R = run_goal1_three_scenarios_400s('ins1', struct('useOfflineReplay',1,'msToProcess',399000));
```

## 4. 绘图入口
所有图默认输出到 `deepIntegration/figures/`。

### 4.1 三轨迹对比图（clean + raw场景 + 紧组合结果）
```matlab
% ds5 三轨迹
plot_clean_spoof_tc_compare('rt_tight_goal1_ds5_400s_unified_ins1.mat');

% ds6 三轨迹
plot_clean_spoof_tc_compare('rt_tight_goal1_ds6_400s_unified_ins1.mat');
```

### 4.2 结果总图（轨迹/时间线/残差/多普勒等）
```matlab
plot_goal1_results('rt_tight_goal1_ds5_400s_unified_ins1.mat');
plot_goal1_results('rt_tight_goal1_ds6_400s_unified_ins1.mat');
```

### 4.3 多普勒图（GNSS vs INS；INS vs Combined）
```matlab
plot_doppler_gps_ins_combined('rt_tight_goal1_ds5_400s_unified_ins1.mat', [22 15 9 21]);
plot_doppler_gps_ins_combined('rt_tight_goal1_ds6_400s_unified_ins1.mat', [22 15 9 21]);
```

### 4.4 论文风格多普勒图（含 400s 全图 + 5s 放大窗）
> 当前版本语义已对齐“原始逻辑”：
> - 上图：GNSS measured vs INS generated  
> - 下图：INS generated vs Combined navigation

```matlab
% PRN22 示例（会输出 full + zoom_onset_5s + zoom_alarm_5s 三张）
plot_doppler_paper_style('rt_tight_goal1_ds5_400s_unified_ins1.mat', [22]);
plot_doppler_paper_style('rt_tight_goal1_ds6_400s_unified_ins1.mat', [22]);
```

## 5. 输出文件说明
每次 `run_goal1_unified_400s_insonly(...)` 会在仓库根目录输出一个结果 MAT，例如：
- `rt_tight_goal1_ds5_400s_unified_ins1.mat`

结果内主要字段：
- `navResults`：逐历元导航与检测结果
- `summary`：关键摘要（报警时刻、接管历元数、误差统计）

常用摘要字段：
- `firstAlarmSec`
- `insOnlyEpochs`
- `takeoverPrrAssistEpochs`
- `preRmseH`, `postRmseH`
- `endErrH`, `endErr3D`

## 6. 常见问题
### 6.1 `Missing dataset: xxx.mat`
检查 MAT 文件是否在仓库根目录，或文件名是否一致。

### 6.2 `Raw IF file is required ...`
你使用了 `useOfflineReplay = 0` 但没有可用原始 `.bin`。  
若只做离线复现，请用 `useOfflineReplay = 1`。

### 6.3 运行后历元数小于 800
属于 `trackResults` 可用长度限制，日志会提示 `GNSS trackResults exhausted`，属于可预期现象。

### 6.4 图像未生成
先确认对应结果 MAT 已存在，再执行绘图函数；图像统一在 `deepIntegration/figures/`。

## 7. 推荐复现顺序（论文写作）
1. 跑 `ds5 + ins1`，导出轨迹与多普勒图；
2. 跑 `ds6 + ins1`，导出轨迹与多普勒图；
3. 跑 `run_goal1_three_scenarios_400s('ins1',...)` 导出总览图；
4. 使用 `summary` 数值填表，图像用于章节展示。
