# DS5 NCO True-Tracking Recovery Validation

本文档说明 `ds5_nco_version1` 分支的用途和运行方法。

## 目标

本分支用于验证 DS5 欺骗场景下的真实信号恢复链：

1. 利用 INS 递推状态和卫星星历预测真实信号 Doppler/code。
2. 估计并去除 DS5 公共 time / Doppler drag。
3. 将 `f_ref_true` 和 `code_ref_true` 注入 shadow tracking loop，修改载波/码 NCO。
4. 将本地复制信号从欺骗峰牵引到真实信号峰。
5. 用恢复观测生成 recovered trajectory。
6. 通过一致性门限后切换 final output，并稳定输出真实轨迹。

当前 400s hybrid 验证已达到工程恢复目标：180s 后 final output 全程使用 recovered/recoveredHold，没有回到 baseline。

## 获取分支

如果是已有仓库：

```powershell
cd E:\ZCJ_GNSSINS_DeepIntegration-main\ZCJ_GNSSINS_DeepIntegration-main
git fetch origin
git switch ds5_nco_version1
```

如果本地没有该分支：

```powershell
git fetch origin
git switch -c ds5_nco_version1 origin/ds5_nco_version1
```

确认当前分支：

```powershell
git branch --show-current
git log --oneline --decorate -1
```

## 输入文件要求

脚本默认从仓库根目录运行，并依赖以下文件：

```text
ds5_400s.mat
rt_deep_goal2_rawtrack_100s_testAC_ds5only_batch_summary.mat
```

raw IF 文件优先使用：

```text
E:\ds5.bin
```

如果 `ds5_400s.mat` 内的 `settings.fileName` 指向存在的 raw 文件，批处理也会尝试使用它。

## 推荐运行方式

不要直接调用 `DeepCouple_perINStime`，因为项目根目录可能存在同名 `.p` 文件。请使用下面的 run 脚本，它们会显式执行：

```matlab
run('deepIntegration/DeepCouple_perINStime.m')
```

### 120s smoke

用于快速检查代码路径和参数透传：

```powershell
matlab -batch "cd('E:\ZCJ_GNSSINS_DeepIntegration-main\ZCJ_GNSSINS_DeepIntegration-main'); run('run_ds5_120s_hybrid_true_tracking_goal.m')"
```

### 240s validation

用于检查 180s 后 recovered output 是否稳定：

```powershell
matlab -batch "cd('E:\ZCJ_GNSSINS_DeepIntegration-main\ZCJ_GNSSINS_DeepIntegration-main'); run('run_ds5_240s_hybrid_true_tracking_goal.m')"
```

### 400s validation

用于长窗验证：

```powershell
matlab -batch "cd('E:\ZCJ_GNSSINS_DeepIntegration-main\ZCJ_GNSSINS_DeepIntegration-main'); run('run_ds5_400s_hybrid_true_tracking_goal.m')"
```

400s hybrid 当前通常约 4 到 6 分钟。输出文件名通常为：

```text
rt_deep_goal2_ds5_hybrid_true_tracking_goal_400s_ds5_399s.mat
rt_deep_goal2_ds5_hybrid_true_tracking_goal_400s_run.log
```

`399s` 是因为当前数据可用 epoch 对应约 398.5s。

## 通过标准

运行结束后脚本会自动调用：

```matlab
printDs5ObsContractSummary(resultFile, 180)
```

重点查看：

```text
recoveredFilterAuthority
finalOutputRecoveryGatePass
finalOutputContinuityPass
finalOutputUseRecovered
finalOutput source counts
```

400s 已验证通过的一组典型结果为：

```text
recoveredFilterAuthority: 438/438
finalOutputRecoveryGatePass: 427/438
finalOutputContinuityPass: 438/438
finalOutputUseRecovered: 438/438
finalOutput source counts: baseline=0 recovered=427 recoveredHold=11 trusted=0 baselineHold=0 zero=0
```

含义：

- `baseline=0`：final output 没有回到欺骗基线。
- `recovered=427`：大多数 epoch 使用新鲜 recovered output。
- `recoveredHold=11`：少数短时 gate 缺口由 recovered hold 跨过。
- `finalOutputUseRecovered=438/438`：180s 后 final output 全程输出 recovered/recoveredHold。

## Hybrid 限流说明

400s 脚本使用 hybrid raw shadow tracking：

```matlab
opts.deepShadowDs5RefObsTrackRawStrideEpochs = 4;
opts.deepShadowDs5RefObsTrackRawBurstMs = 20;
opts.deepShadowDs5RefObsTrackRawWarmupEpochs = 3;
```

含义：

- 每 2s 抽样运行一次 raw shadow tracking。
- 每次每星最多运行 20ms raw IF tracking。
- 非抽样 epoch 使用 trueRef Doppler/code 对 shadow tracking 状态 coasting。

这样做是为了避免 full raw tracking 的长时间运行和 native heap corruption 风险。该验证能够证明工程目标链，即 trueRef 注入、recovered trajectory 和 final output 稳定切换；但它不是“每 1ms 全速 raw IF 连续跟踪”的严格证明。

## 当前限制

当前结论：

```text
DS5 欺骗后真实轨迹恢复与 final output 切换已通过 400s hybrid 验证。
```

仍需注意：

- `obsContractPass` 不一定全程通过，因为 raw observation contract 仍受 `baseSeedPass` 严格判据影响。
- `finalObsContractBaseSeedHold` 用于接住 base-seed-only drop。
- 因此当前证明的是“final output 稳定恢复真实轨迹”，不是“每个 epoch 都形成严格独立 raw true pseudorange”。

如果后续需要论文级观测层证明，需要继续修 raw/base-seed 独立性判据或运行更高密度 raw tracking 验证。

## 相关文件

核心实现：

```text
deepIntegration/DeepCouple_perINStime.m
deepIntegration/perChannelTrackOnce_DeepIn.m
deepIntegration/postNavTight.m
deepIntegration/run_goal2_deep_400s_batch.m
```

配置和运行脚本：

```text
make_goal2_frontfix_120s_opts.m
make_goal2_recoveryboost_120s_opts.m
run_ds5_120s_hybrid_true_tracking_goal.m
run_ds5_240s_hybrid_true_tracking_goal.m
run_ds5_400s_hybrid_true_tracking_goal.m
```

摘要工具：

```text
printDs5ObsContractSummary.m
```

## 常见问题

### MATLAB 退出时报 Heap corruption

早期 full raw 或长窗运行可能在 MATLAB 退出/清理阶段报：

```text
0xc0000374, Heap corruption
```

当前 400s 脚本会在打印摘要后显式 `exit`，并且主体结果会先保存到 MAT 文件。如果仍看到退出阶段错误，先检查 MAT 和 log 是否已经生成。

### GitHub 上看不到分支

确认远端分支：

```powershell
git fetch origin
git branch -r | findstr ds5_nco_version1
```

切换分支：

```powershell
git switch -c ds5_nco_version1 origin/ds5_nco_version1
```
