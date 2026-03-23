# Chapter 1 Implementation Checklist

This file is the coding checklist for Chapter 1 only.

Target:

1. `deepIntegration/realtime_tightCouple.m` is the only Chapter 1 mainline.
2. Detection is navigation-layer only.
3. Mitigation is `GNSS update off + INS takeover + EKF virtual motion constraints`.

## 1. Global Rule

Do not build Chapter 1 on the old mixed logic.

Chapter 1 must not depend on:

1. `robust_tc`
2. pseudorange metric as the main detector
3. loose-vs-tight position difference as the main detector
4. stage-2 pause/recovery logic
5. GNSS weight recovery after alarm
6. signal-layer correction

The shortest correct path is:

1. keep the existing tight-coupled propagation framework
2. replace the detector
3. simplify mitigation to one latched `INS takeover` path
4. move drift suppression into EKF virtual constraints

## 2. `deepIntegration/realtime_tightCouple.m`

### 2.1 Keep

Keep these parts as the Chapter 1 backbone:

1. dataset loading and initial `trackDeepIn` construction
2. INS mechanization and EKF propagation
3. `postNavTight(...)` measurement building
4. satellite geometry and range-rate computation
5. receiver-domain Doppler prediction structure that already includes:
   - geometric Doppler
   - satellite clock drift
   - receiver clock drift
6. navigation result output structure

Keep the existing state definition:

1. continue using `psinstypedef('test_SINS_GPS_tightly_def')`
2. do not redesign the state vector for Chapter 1
3. reuse the existing clock bias / clock drift states already used in the file

### 2.2 Delete Or Disable

Disable the old detector and mitigation logic from the Chapter 1 mainline:

1. `spoofDetUseJointDecision`
2. `spoofDetPrMetric*`
3. `spoofDetUsePosFeature`
4. `spoofDetJointScoreThreshold`
5. `spoofDetSevere*`
6. `spoofDetAllowRelease`
7. `spoofDetQualityGateEnable`
8. `spoofMitigationMode = 'robust_tc'`
9. `spoofMitTwoStageEnable`
10. stage-2 refresh / pause logic
11. adaptive GNSS downweighting path
12. per-satellite `R` inflation rejection path
13. innovation clipping as the main anti-spoofing method
14. odometer aiding in Chapter 1 mainline

These items can remain in history, but they must not participate in the
Chapter 1 execution path.

### 2.3 Replace The Detector Block

Replace the current detector block with this exact logic:

1. At each navigation epoch, compute raw residual for each valid satellite:
   - `r_i = f_gnss,i - f_ins_geom,i - f_satclk,i - f_rcvclk`
2. Apply the clean-data per-satellite bias correction:
   - `r_i_c = r_i - b_i`
3. Keep only satellites that satisfy:
   - valid ephemeris
   - finite Doppler
   - elevation above `minElevDeg`
4. Compute:
   - `M_cm = abs(median(r_i_c))`
   - `M_df = median(abs(r_i_c - median(r_i_c)))`
5. Count hit satellites:
   - hit if `abs(r_i_c) > T_sat`
6. Alarm confirmation rule:
   - detector armed after calibration time
   - `numHit >= N_hit`
   - and one of:
     - `M_cm > T_cm`
     - `M_df > T_df`
   - confirm for `N_confirm` consecutive epochs
7. Latch `spoofFlag = true` after first confirmed alarm
8. Do not release the alarm during the Chapter 1 run

### 2.4 Add Required Detector Outputs

Add or rename `navResults` fields so Chapter 1 outputs are explicit:

1. `dopplerResidualRawHz`
2. `dopplerResidualBiasCorrectedHz`
3. `detectorMetricCommonHz`
4. `detectorMetricDiffHz`
5. `detectorHitSatNum`
6. `detectorValidSatNum`
7. `detectorThresholdCommonHz`
8. `detectorThresholdDiffHz`
9. `detectorThresholdSatHz`
10. `modeInsTakeover`
11. `modeVirtualNHC`
12. `modeVirtualZUPT`
13. `gnssUpdateUsed`
14. `spoofAlarm`
15. `spoofFlag`

### 2.5 Replace The Mitigation Branch

The mitigation branch must become binary.

Normal mode:

1. run standard GNSS/INS tight-coupled EKF update

Takeover mode:

1. do not call GNSS measurement update with pseudorange innovation
2. keep INS mechanization and state propagation
3. apply only motion-constraint virtual measurements through EKF
4. feedback corrected state to INS

### 2.6 Implement NHC And ZUPT As Virtual Measurements

Do not directly overwrite INS velocity as the Chapter 1 final method.

Required implementation rule:

1. build virtual measurement residuals
2. build corresponding `H` and `R`
3. call EKF update
4. feed back the updated state to INS

Required virtual measurements:

1. NHC:
   - lateral velocity near zero
   - vertical velocity near zero
2. ZUPT:
   - all three velocity components near zero when stop condition holds

Stop condition can be based on:

1. horizontal speed threshold
2. or cleanly defined low-dynamics condition already available from INS

### 2.7 Lever Arm Handling

Chapter 1 must make the lever-arm assumption explicit in code.

Implementation rule:

1. add a settings field such as `settings.leverArm_b`
2. default to `[0;0;0]`
3. if nonzero lever arm is later provided, compute antenna-center velocity
   before Doppler prediction

### 2.8 Remove Wrong Residual Processing

Do not use the old "always subtract current-epoch common bias before
thresholding" as the only detector path.

Reason:

1. it weakens `ds5` detection because `ds5` contains common-mode spoofing
   structure

The Chapter 1 detector must preserve:

1. common-mode residual feature for `ds5`
2. differential residual feature for `ds6`

## 3. `deepIntegration/build_goal1_clean_baseline.m`

### 3.1 Keep

Keep:

1. clean dataset loading
2. same runner path through `realtime_tightCouple.m`
3. same IMU profile option
4. same calibration window concept

### 3.2 Delete Or Replace

Replace the old baseline outputs based on a single aggregated metric.

Do not use as final Chapter 1 calibration outputs:

1. old `spoofMetricHz` only
2. old common-bias-removed-only metric threshold

### 3.3 Add

This file must output a baseline struct containing at least:

1. `prnList`
2. `satBiasHz`
3. `satSigmaHz`
4. `commonMedianHz`
5. `commonSigmaHz`
6. `diffMedianHz`
7. `diffSigmaHz`
8. `T_cm`
9. `T_df`
10. `T_sat`
11. `N_hit`
12. `N_confirm`
13. `calibStartSec`
14. `calibEndSec`

Threshold formulas must be explicit:

1. `mu = median(...)`
2. `sigma = 1.4826 * MAD`
3. `T = mu + k * sigma`

### 3.4 Important Constraint

Baseline generation must use the same:

1. time alignment rule
2. clock-drift compensation rule
3. lever-arm assumption
4. tracking settings
5. IMU profile

as the spoofed run.

## 4. `deepIntegration/run_goal1_unified_400s_insonly.m`

### 4.1 Keep

Keep this file as the Chapter 1 batch runner.

It should still:

1. select dataset
2. select IMU profile
3. build or load clean baseline
4. call `realtime_tightCouple`
5. save summary MAT

### 4.2 Replace Settings Passed Into Mainline

This runner should pass only Chapter 1 settings:

1. baseline thresholds `T_cm`, `T_df`, `T_sat`
2. `N_hit`
3. `N_confirm`
4. `spoofDetArmTimeSec`
5. `minElevDeg`
6. `leverArm_b`
7. `virtualNHCEnable`
8. `virtualZUPTEnable`
9. `gnssTakeoverMode = 'ins_only_latched'`

Do not pass old mixed settings as active Chapter 1 logic:

1. `spoofDetUseJointDecision`
2. `spoofDetUsePosFeature`
3. `spoofMitTwoStageEnable`
4. `spoofMitPerSatEnable`
5. `spoofMitAdaptiveWeighting`
6. `odoEnable`

### 4.3 Scene Compatibility Rule

This runner may allow small ds5/ds6 threshold differences, but only in:

1. `T_cm`
2. `T_df`
3. `N_confirm`

It must not change:

1. detector structure
2. takeover structure
3. motion-constraint structure

## 5. `deepIntegration/plot_goal1_results.m`

### 5.1 Keep

Keep:

1. ENU trajectory plot
2. alarm vertical line
3. timeline output
4. Doppler comparison output

### 5.2 Replace

Replace old Chapter 1 plotting focus with the new metrics:

1. plot `detectorMetricCommonHz` with `T_cm`
2. plot `detectorMetricDiffHz` with `T_df`
3. plot hit-satellite count with `N_hit`
4. plot takeover mode timeline
5. plot four PRNs of:
   - `f_gnss`
   - INS-predicted Doppler
   - bias-corrected residual

Do not keep old Chapter 1 detector plots as the main result if they are based
on:

1. joint score
2. pseudorange metric
3. position-difference detector

## 6. Files To Ignore For Chapter 1 Mainline

Do not use these as the Chapter 1 final route:

1. `deepIntegration/DeepCouple_perINStime.m`
2. `deepIntegration/run_goal1_standard_*.m`
3. `deepIntegration/run_goal1_compare_modes.m`
4. `deepIntegration/run_goal1_param_search.m`
5. temporary `tmp_*.m`

They may remain for history, but not as the thesis Chapter 1 implementation.

## 7. Acceptance Checklist Before Coding Is Considered Done

All of these must be true:

1. `clean` has no alarm
2. `ds5` alarms
3. `ds6` alarms
4. after alarm, `gnssUpdateUsed = false`
5. after alarm, `modeInsTakeover = true`
6. after alarm, virtual NHC/ZUPT can still update EKF
7. trajectory with mitigation is better than no-mitigation
8. plots use Chapter 1 metrics, not legacy mixed metrics

## 8. Recommended Coding Order

Use this order and do not skip:

1. refactor baseline output in `build_goal1_clean_baseline.m`
2. replace detector in `realtime_tightCouple.m`
3. replace mitigation branch in `realtime_tightCouple.m`
4. add virtual NHC/ZUPT update path
5. simplify runner settings in `run_goal1_unified_400s_insonly.m`
6. update plots in `plot_goal1_results.m`
7. validate on `clean`
8. validate on `ds5`
9. validate on `ds6`
