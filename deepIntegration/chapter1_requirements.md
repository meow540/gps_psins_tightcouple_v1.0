# Chapter 1 Requirements

## 1. Goal

Chapter 1 implements a navigation-layer GNSS/INS anti-spoofing method in
`deepIntegration/realtime_tightCouple.m`.

The target is:

1. Use the residual between GNSS measured Doppler and INS predicted Doppler
   to detect spoofing.
2. Once spoofing is declared, raise an alarm.
3. After the alarm, stop GNSS navigation updates.
4. Let INS take over navigation output.
5. Use navigation-layer motion constraints to suppress INS drift during
   takeover.

This chapter is only about navigation-layer detection and mitigation.

## 2. Scope

In scope:

1. Tight-coupled navigation framework in `deepIntegration/realtime_tightCouple.m`
2. Doppler residual based spoofing detection
3. INS-only takeover after alarm
4. Navigation-layer motion constraints during takeover
5. Evaluation on `clean`, `ds5`, and `ds6`

Out of scope:

1. Tracking-loop NCO correction
2. Pseudorange correction from Doppler smoothing
3. Signal-layer spoofing mitigation
4. Deep integration mainline for thesis Chapter 2
5. Multi-correlator signal parameter estimation

## 3. Datasets

Required datasets:

1. `cleandynamic_400s.mat`
2. `ds5_400s.mat`
3. `ds6_400s.mat`

Role of each dataset:

1. `clean` is used for baseline calibration and false-alarm check.
2. `ds5` is used to validate detection and mitigation in dynamic,
   overpowered, frequency-unlocked time-push spoofing.
3. `ds6` is used to validate detection and mitigation in dynamic,
   matched-power, frequency-locked position-push spoofing.

All datasets must be generated with the same raw processing settings before
being used in Chapter 1 comparisons.

## 4. High-Level Logic

For each navigation epoch:

1. Build the GNSS measurements from the tracked channels.
2. Predict per-satellite Doppler using INS state and broadcast ephemeris.
   The predicted Doppler must be the receiver-domain Doppler, not only the
   pure geometric Doppler.
3. Compute per-satellite Doppler residual:

   `r_i = f_gnss,i - f_ins,i`

4. Remove only the clean-data calibrated per-satellite static bias:

   `r_i_c = r_i - b_i`

5. Build navigation-layer spoofing metrics from `r_i_c`.
6. Apply detector decision logic.
7. If no spoofing is declared, keep normal tight GNSS/INS fusion.
8. If spoofing is declared, stop GNSS measurement updates and switch to INS
   takeover mode with motion constraints.

## 4.1 Time Alignment and Clock Drift

Chapter 1 must explicitly align the GNSS Doppler measurement epoch and the
INS predicted Doppler epoch.

Required rule:

1. `f_gnss,i` and `f_ins,i` must refer to the same receiver time tag.

The predicted Doppler must include:

1. satellite-receiver geometric range rate
2. satellite clock drift contribution
3. receiver clock drift contribution estimated by the EKF

Required residual definition:

`r_i = f_gnss,i - f_ins_geom,i - f_satclk,i - f_rcvclk`

Not allowed:

1. Ignoring receiver clock drift in the residual
2. Using star single-difference as the main Chapter 1 detector input

Reason:

1. Ignoring receiver clock drift causes a large time-varying common bias and
   invalidates the common-mode detector.
2. Single-difference would also remove the common-mode spoofing feature that
   is required for `ds5`.

## 4.2 Lever Arm Assumption

For the Chapter 1 benchmark, the IMU navigation point and the GNSS antenna
phase center must be treated explicitly.

Allowed implementation choices:

1. If no calibrated lever-arm vector is available, declare the Chapter 1
   benchmark assumption:
   `leverArm_b = [0;0;0]`
2. If a calibrated lever-arm vector is available, INS predicted Doppler must
   be computed at the antenna phase center, not at the IMU origin.

If lever-arm compensation is enabled, the antenna velocity must include the
rotational contribution:

`v_ant = v_ins + C_b2n * (omega_ib_b x leverArm_b)`

Then Doppler prediction must use `v_ant`.

## 5. Detection Features

Chapter 1 must use Doppler residual features only.

The detector must include two residual modes:

1. Common-mode metric
   - Purpose: capture the common Doppler offset typical in `ds5`
   - Definition:
     `M_cm = |median_i(r_i_c)|`

2. Differential-mode metric
   - Purpose: capture satellite-dependent residual inconsistency typical in
     `ds6`
   - Definition:
     `M_df = median_i |r_i_c - median_j(r_j_c)|`

The detector must also include a multi-satellite support condition:

1. At least `N_min` valid satellites participate in the metric.
2. At least `N_hit` satellites exceed the per-satellite residual threshold
   before alarm confirmation.

## 6. Calibration Requirements

Calibration must come from the clean dataset only.

Required clean calibration outputs:

1. Per-satellite residual bias `b_i`
2. Common-mode baseline statistics
3. Differential-mode baseline statistics
4. Per-satellite residual dispersion statistics

Calibration rules:

1. Use the same tracking settings and IMU profile as the spoofed run.
2. Use a clean time window before any spoofing, for example 20 s to 90 s.
3. Do not use spoofed data to fit the baseline.
4. Use the same time-alignment, clock-drift compensation, and lever-arm
   assumptions as the spoofed run.

## 6.1 Threshold Definitions

Thresholds must be written as explicit formulas.

For any calibrated metric series `M[k]` in the clean window:

1. `mu_M = median(M[k])`
2. `sigma_M = 1.4826 * median(|M[k] - mu_M|)`
3. `T_M = mu_M + k_M * sigma_M`

Required thresholds:

1. `T_cm` for common-mode metric `M_cm`
2. `T_df` for differential-mode metric `M_df`
3. `T_sat` for per-satellite residual magnitude support counting

Recommended default:

1. use robust median/MAD thresholds, not mean/std thresholds

Reason:

1. the Chapter 1 detector itself is based on median-type robust statistics
2. clean and spoofed residuals may contain outliers and short transients

## 7. Decision Logic

The Chapter 1 detector must be simple and explicit.

Required logic:

1. Arm the detector only after the clean calibration window.
2. Use short confirmation logic across consecutive navigation epochs.
3. Raise spoofing alarm if either condition is met:
   - common-mode metric exceeds its threshold and satellite support is enough
   - differential-mode metric exceeds its threshold and satellite support is enough
4. After alarm, keep the spoof flag latched for the rest of the Chapter 1 run.

Required support counting rule:

1. A satellite is counted as hit if `|r_i_c| > T_sat`
2. Alarm confirmation is valid only when the number of hit satellites is not
   less than `N_hit`

Not allowed in Chapter 1 mainline:

1. Joint decision with pseudorange feature
2. Joint decision with loose-vs-tight position difference
3. Two-stage robust weighting logic as the main mitigation path
4. Alarm release and GNSS weight recovery logic

Reason:

Chapter 1 must remain a clean navigation-layer INS takeover implementation.

## 8. Mitigation Logic

Once spoofing is declared:

1. Output alarm flag
2. Disable GNSS measurement update in the EKF
3. Continue INS mechanization and error-state propagation
4. Apply navigation-layer motion constraints to reduce drift

Required motion constraints:

1. NHC for land vehicle side and vertical velocity suppression
2. ZUPT when vehicle speed is near zero
3. Optional vertical soft damping

Required EKF behavior during takeover:

1. NHC and ZUPT must be implemented as virtual measurements in the EKF
2. Virtual-measurement update must be followed by state feedback to INS
3. The virtual-measurement path must remain active after GNSS updates are
   disabled

Not allowed in Chapter 1 mainline:

1. Hard-setting INS velocity to zero as the main ZUPT implementation
2. Hard-damping body-frame velocity as the main NHC implementation
3. Replacing EKF virtual updates with direct state overwrite

Also not allowed in Chapter 1 mainline:

1. Robust TC as the final mitigation method
2. Pseudorange innovation clipping as the core solution
3. Signal tracking correction
4. NCO correction
5. Pseudorange correction

## 9. DS5 and DS6 Compatibility

The same Chapter 1 framework must work on both `ds5` and `ds6`.

Allowed differences:

1. Threshold values may differ slightly between `ds5` and `ds6`
2. Alarm timing may differ between `ds5` and `ds6`
3. Final error level may differ between `ds5` and `ds6`

Not allowed differences:

1. Different core detector logic
2. Different mitigation architecture
3. Replacing Doppler detection with unrelated features in one scene only

## 10. Expected Outputs

Chapter 1 run outputs must include:

1. Navigation result MAT file
2. Alarm time and alarm epoch
3. ENU trajectory plot
4. Alarm timeline plot
5. Four-PRN Doppler comparison plot
6. Residual metric versus threshold plot

The MAT result should at least contain:

1. Position and velocity solution
2. Spoof alarm flag
3. Spoof start epoch
4. GNSS update used flag
5. INS takeover mode flag
6. Motion constraint mode flags
7. Per-satellite Doppler residuals
8. Common-mode and differential-mode metrics

## 11. Acceptance Criteria

Chapter 1 is considered complete only if all of the following are true:

1. `clean` produces no spoofing alarm
2. `ds5` produces a spoofing alarm
3. `ds6` produces a spoofing alarm
4. After alarm, GNSS updates are disabled
5. After alarm, navigation output is produced by INS takeover
6. Motion constraints are active during takeover
7. Spoofed solution drift is reduced compared with the no-mitigation case

## 12. File Responsibilities

Main file:

1. `deepIntegration/realtime_tightCouple.m`
   - keep as Chapter 1 mainline
   - keep tight-coupled navigation propagation and update framework
   - replace old spoof detector with Chapter 1 detector
   - replace old mitigation mainline with INS takeover plus motion constraints

Calibration helper:

1. `deepIntegration/build_goal1_clean_baseline.m`
   - keep as clean baseline builder
   - change output from old aggregated metric thresholding to
     per-satellite bias and common/differential metric statistics

Run script:

1. `deepIntegration/run_goal1_unified_400s_insonly.m`
   - keep as Chapter 1 batch runner
   - configure clean calibration, ds5/ds6 runs, and output summaries

Plot script:

1. `deepIntegration/plot_goal1_results.m`
   - update to show Chapter 1 metrics and takeover status only

## 13. Non-Goals for Chapter 1

The following belong to Chapter 2, not Chapter 1:

1. `realtime_deepCouple.m`
2. Tracking-level spoofing suppression
3. INS-aided NCO correction
4. INS-aided pseudorange correction
5. Correlator-level recovery of authentic tracking

## 14. Thesis Wording Constraint

For Chapter 1, the phrase "recover correct navigation solution" must be
understood as:

1. stop further corruption from spoofed GNSS measurements
2. keep continuous navigation output after alarm
3. suppress drift using navigation-layer constraints

It must not be interpreted as guaranteeing long-duration absolute positioning
accuracy using pure INS alone.
