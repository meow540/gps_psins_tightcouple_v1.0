# INS2 版本使用文档

本说明对应分支 `ins2-onlyguide`，用于复现第一章目标在 INS2 器件下的 `clean / ds5 / ds6` 结果。

## 1. 运行前准备

- MATLAB 可用（支持 `-batch`）
- 工程根目录下存在：
  - `cleandynamic_400s.mat`
  - `ds5_400s.mat`
  - `ds6_400s.mat`
- INS 轨迹文件可访问：
  - `E:\fgi_result\ins_simulation\ins2_trj.mat`

## 2. 快速复现（INS2）

在 MATLAB 命令行执行：

```matlab
addpath('deepIntegration');

s_clean = run_goal1_unified_400s_insonly('clean','ins2',struct('useOfflineReplay',1));
s_ds5   = run_goal1_unified_400s_insonly('ds5','ins2',struct('useOfflineReplay',1));
s_ds6   = run_goal1_unified_400s_insonly('ds6','ins2',struct('useOfflineReplay',1));

plot_goal1_results('rt_tight_goal1_ds5_400s_unified_ins2.mat');
plot_goal1_results('rt_tight_goal1_ds6_400s_unified_ins2.mat');
```

## 3. 主要输出文件

- 结果 MAT：
  - `rt_tight_goal1_clean_400s_unified_ins2.mat`
  - `rt_tight_goal1_ds5_400s_unified_ins2.mat`
  - `rt_tight_goal1_ds6_400s_unified_ins2.mat`
- 图像目录：`deepIntegration/figures/`
  - `*_traj_vs_cleandynamic_enu_en.png`
  - `*_spoof_metric.png`
  - `*_timeline.png`
  - `*_error_vs_cleandynamic.png`
  - `*_doppler_compare.png`
  - `*_doppler_compare_prnXX.png`

## 4. 当前 INS2 配置要点

- ds5：采用 `clock_hold` 路径（报警后进入时钟欺骗抑制模式）
- ds6：采用 INS 接管 + PRR 辅助（含平滑衰减）
- ds6 参数（已固化在 runner）：
  - `insTakeoverPrrResidualClipHz = 10.0`
  - `insTakeoverPrrNoiseStdMps = 0.12`
  - `insTakeoverResidualIirAlpha = 0.25`
  - `insTakeoverPrrAssistFadeStartSec = 220.0`
  - `insTakeoverPrrAssistFadeDurationSec = 120.0`

## 5. 常见问题

- 如果运行慢：优先使用 `useOfflineReplay = 1`（本文默认）。
- 如果 `postProcessing` 阶段星历不足：先确认 `ds*_400s.mat` 是同一套参数重生成的数据。
- 若图像未刷新：重新执行 `plot_goal1_results(...)` 覆盖输出。
