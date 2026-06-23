# DS5 NCO true-tracking recovery validation

This branch is `ds5_nco_version1`.

It validates the DS5 spoofing-recovery chain:

1. Predict true-signal Doppler/code from INS propagation and satellite ephemeris.
2. Estimate and remove the DS5 common time/Doppler drag.
3. Inject `f_ref_true` and `code_ref_true` into the receiver shadow tracking loop.
4. Pull the local carrier/code NCO from the spoofing peak toward the true-signal peak.
5. Build independent recovered pseudorange observations and a recovered trajectory.
6. Switch final output to the recovered trajectory after consistency gates pass.

## Current status

Two validation modes are kept on purpose:

- `400s hybrid`: engineering end-to-end validation for stable recovered final output.
- `186s strict raw`: short-window proof that the DS5 observation layer can pass a strict raw independent pseudorange contract without base-seed fallback or coasting.

The latest strict raw 186 s run passed the full chain from 180 s:

```text
obsContractPass: 12/12
strictRawIndependentPass: 12/12
refObsTrackRawSampled: 72/72
refObsTrackCoasted: 0/72
refObsRecoveryPass: 12/12
deltaConsistencyPass: 12/12
refObsPositionUsed: 12/12
recoveredFilterMeasPass: 12/12
recoveredFilterAuthority: 12/12
finalOutputUseRecovered: 12/12
finalOutput source counts: baseline=0 recovered=12 recoveredHold=0 trusted=0 baselineHold=0 zero=0
```

This means the short strict window is not relying on base-seed-only hold:

```text
obsContract effectiveBaseSeedFrac: med/p95/max = 0.000 / 0.000 / 0.000
strictRawIndependent sat: med/p95/max = 6.000 / 6.000 / 6.000
strictRaw rawNonCoastedUsed sat: med/p95/max = 6.000 / 6.000 / 6.000
strictRaw noBaseSeedUsed sat: med/p95/max = 6.000 / 6.000 / 6.000
strictRaw qualityIndependent sat: med/p95/max = 6.000 / 6.000 / 6.000
```

## Get the branch

```powershell
cd E:\ZCJ_GNSSINS_DeepIntegration-main\ZCJ_GNSSINS_DeepIntegration-main
git fetch origin
git switch ds5_nco_version1
```

If the local branch does not exist:

```powershell
git fetch origin
git switch -c ds5_nco_version1 origin/ds5_nco_version1
```

## Required inputs

Run scripts from the repository root. The scripts expect the DS5 dataset and batch summaries used by the project, including:

```text
ds5_400s.mat
rt_deep_goal2_rawtrack_100s_testAC_ds5only_batch_summary.mat
```

The raw IF file is normally:

```text
E:\ds5.bin
```

## Strict raw short-window validation

Use this first when checking the observation layer. It disables generic shadow raw reacquisition/search, then activates the DS5 true-reference raw observation path only near the strict window.

Default end time is 200 s:

```powershell
matlab -batch "cd('E:\ZCJ_GNSSINS_DeepIntegration-main\ZCJ_GNSSINS_DeepIntegration-main'); run('run_ds5_200s_strict_raw_true_tracking_goal.m')"
```

Recommended smoke window is 186 s:

```powershell
cd E:\ZCJ_GNSSINS_DeepIntegration-main\ZCJ_GNSSINS_DeepIntegration-main
$env:DS5_STRICT_RAW_END_SEC="186"
matlab -batch "cd('E:\ZCJ_GNSSINS_DeepIntegration-main\ZCJ_GNSSINS_DeepIntegration-main'); run('run_ds5_200s_strict_raw_true_tracking_goal.m')"
Remove-Item Env:\DS5_STRICT_RAW_END_SEC -ErrorAction SilentlyContinue
```

Typical runtime for the 186 s strict window is about 10 to 15 minutes on the current machine.

Key pass fields:

```text
strictRawIndependentPass
refObsTrackRawSampled
refObsTrackCoasted
obsContractBaseSeedPass
refObsPositionUsed
recoveredFilterAuthority
finalOutputUseRecovered
```

Expected result for the 186 s smoke:

```text
strictRawIndependentPass: 12/12
refObsTrackCoasted: 0/72
refObsPositionUsed: 12/12
recoveredFilterAuthority: 12/12
finalOutputUseRecovered: 12/12
```

## 400 s hybrid validation

Use this for the end-to-end final output recovery check:

```powershell
matlab -batch "cd('E:\ZCJ_GNSSINS_DeepIntegration-main\ZCJ_GNSSINS_DeepIntegration-main'); run('run_ds5_400s_hybrid_true_tracking_goal.m')"
```

The hybrid script limits raw shadow tracking density to keep runtime and MATLAB native heap risk manageable. It validates stable recovered final output, but it is not the strict proof that every epoch is a fresh raw independent pseudorange observation.

Typical 400 s hybrid pass shape:

```text
finalOutputUseRecovered: 438/438
finalOutput source counts: baseline=0 recovered=427 recoveredHold=11 trusted=0 baselineHold=0 zero=0
```

## Summary command

To reprint a saved result:

```powershell
matlab -batch "cd('E:\ZCJ_GNSSINS_DeepIntegration-main\ZCJ_GNSSINS_DeepIntegration-main'); printDs5ObsContractSummary('rt_deep_goal2_ds5_strict_raw_true_tracking_goal_186s_ds5_186s.mat',180)"
```

## MATLAB heap corruption note

Some runs still print this when MATLAB exits:

```text
ERROR: MATLAB error Exit Status: 0xc0000374, Heap corruption
```

If the summary printed completely and the `*_summary_done.txt` marker exists, treat this as a MATLAB/native cleanup issue after result generation, not as an algorithm failure. The strict raw script writes the MAT file, prints the summary, writes the done marker, closes files, then calls `quit force`.

## Main files

Core implementation:

```text
deepIntegration/DeepCouple_perINStime.m
deepIntegration/run_goal2_deep_400s_batch.m
```

Run scripts:

```text
run_ds5_200s_strict_raw_true_tracking_goal.m
run_ds5_400s_hybrid_true_tracking_goal.m
run_ds5_240s_hybrid_true_tracking_goal.m
run_ds5_120s_hybrid_true_tracking_goal.m
```

Summary helper:

```text
printDs5ObsContractSummary.m
```

## Remaining work

The strict raw 186 s window now proves that the DS5 tracking/observation/final-output chain can pass independently after 180 s. The next validation step is to extend this strict raw window gradually, for example 200 s, 220 s, then 240 s, while watching runtime and MATLAB heap behavior.

The 400 s hybrid run remains the practical long-window final-output validation. A full 400 s strict raw run may be very slow and more likely to trigger native MATLAB heap cleanup failures.
