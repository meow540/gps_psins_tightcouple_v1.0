
function opts = make_goal2_frontfix_120s_opts(outPrefix, datasets)
%MAKE_GOAL2_FRONTFIX_120S_OPTS Build the 120s DS5/DS6 front-end recovery test options.
% This keeps the experiment configuration in one place and avoids PRN- or
% dataset-specific tuning in the core algorithm.
if nargin < 1 || isempty(outPrefix)
    outPrefix = 'rt_deep_goal2_frontfix_ds5ds6_120s';
end
if nargin < 2 || isempty(datasets)
    datasets = {'ds5', 'ds6'};
end

opts = struct();
opts.outPrefix = outPrefix;
opts.roundTime = 240;
opts.datasets = datasets;

opts.reuseCalib = true;
opts.calibFile = 'rt_deep_goal2_rawtrack_100s_testAC_ds5only_batch_summary.mat';
opts.deepFastMode = 0;
opts.deepPerfLightNormalMode = 1;
opts.deepPerfDiagPrint = 1;
opts.deepPerfDiagInterval = 20;

opts.deepUseGnssKfUpdateNormal = 1;
opts.deepUseGnssKfUpdateSuspect = 0;
opts.deepUseGnssKfUpdateSpoof = 0;
opts.deepUseKfClockUpdateNormal = 0;
opts.deepUseKfClockUpdateSuspect = 0;
opts.deepUseKfClockUpdateSpoof = 0;
opts.deepConfirmEpochs = 3;

% Shadow observations are optional; tracking-only tests disable this feedback.
opts.deepShadowUseNavQualifiedForKf = 1;
opts.deepShadowKfMinSat = 4;
opts.deepShadowKfRScale = 20.0;
opts.deepShadowKfMedianDetrend = 1;
opts.deepShadowKfDetrendGateM = 5000.0;

opts.deepShadowAcqEnable = 0;
opts.deepCodeWideReacqEnable = 0;
opts.deepCodeReacqEnable = 0;

opts.deepShadowRawReacqEnable = 1;
opts.deepShadowRawReacqIntervalEpochs = 8;
opts.deepShadowRawReacqHalfChips = 20.0;
opts.deepShadowRawReacqStepChips = 0.50;
opts.deepShadowRawReacqFreqHalfHz = 1000;
opts.deepShadowRawReacqFreqStepHz = 250;
opts.deepShadowRawReacqNoncohMs = 8;
opts.deepShadowRawReacqCoherentMs = 1;
opts.deepShadowRawReacqTopK = 3;
opts.deepShadowRawReacqNmsChipGuard = 2.0;
opts.deepShadowRawReacqNmsFreqGuardHz = 350;
opts.deepShadowRawReacqFineHalfChips = 2.0;
opts.deepShadowRawReacqFineStepChips = 0.50;
opts.deepShadowRawReacqFineFreqHalfHz = 250;
opts.deepShadowRawReacqFineFreqStepHz = 125;
opts.deepShadowRawReacqExcludeChips = 0.75;

opts.deepShadowRawClusterAssocTolChips = 1.0;
opts.deepShadowRawClusterAssocTolFreqHz = 250;
opts.deepShadowRawClusterScoreDecay = 0.85;
opts.deepShadowRawClusterHitGain = 1.0;
opts.deepShadowRawClusterMissDecay = 0.75;
opts.deepShadowRawClusterReadyScore = 2.0;
opts.deepShadowRawClusterReadyHits = 2;
opts.deepShadowRawClusterReadyMargin = 0.5;
opts.deepShadowRawClusterValidateTolChips = 1.5;
opts.deepShadowRawClusterValidateTolFreqHz = 300;
opts.deepShadowRawClusterPassScore = 1.4;
opts.deepShadowRawClusterPassHits = 3;
opts.deepShadowRawClusterPassMargin = 0.6;
opts.deepShadowRawClusterPassImproveRelaxM = 0.0;
opts.deepShadowRawClusterInitScore = 1.2;
opts.deepShadowRawClusterInitHits = 3;
opts.deepShadowRawClusterInitMargin = 0.15;
opts.deepShadowRawClusterDirectInitScore = 1.6;
opts.deepShadowRawClusterDirectInitHits = 4;
opts.deepShadowRawClusterDirectInitMargin = 0.20;
opts.deepShadowRawClusterInitBlendAlpha = 0.65;

opts.deepShadowRawExplorePeakRatioMin = 1.00;
opts.deepShadowRawExploreZeroRatioMin = 0.90;
opts.deepShadowRawWeakVotePeakRatioMin = 1.00;
opts.deepShadowRawWeakVoteZeroRatioMin = 0.12;
opts.deepShadowRawWeakVoteMetricScale = 0.35;
opts.deepShadowRawExploreClusterScore = 1.0;
opts.deepShadowRawExploreClusterHits = 2;
opts.deepShadowRawExploreClusterMargin = 0.10;

opts.deepShadowRawTrackEnable = 1;
opts.deepShadowRawTrackPeakRatioMin = 1.03;
opts.deepShadowRawTrackZeroRatioMin = 1.05;
opts.deepShadowRawTrackProtectEpochs = 4;
opts.deepShadowRawTrackValidateEpochs = 2;
opts.deepShadowRawTrackValidateImproveMinM = 100.0;
% Not a DS/PRN heuristic: this accepts cluster-backed shadow tracks whose
% absolute residual is already plausible even if main-track improvement is weak.
opts.deepShadowRawTrackValidateAbsDeltaEnable = 1;
opts.deepShadowRawTrackValidateAbsDeltaMaxM = 3500.0;
opts.deepShadowRawTrackValidateDuringHoldEnable = 1;
opts.deepShadowRawTrackValidateMaxEpochs = 8;
opts.deepShadowRawTrackValidateWindowEpochs = 4;
opts.deepShadowRawTrackValidateWindowPassHits = 2;
opts.deepShadowRawTrackScoreGoodImproveM = 200.0;
opts.deepShadowRawTrackScoreBadImproveM = -300.0;
opts.deepShadowRawTrackScoreRiseM = 150.0;
opts.deepShadowRawTrackScorePassMin = 2.0;
opts.deepShadowRawTrackScoreRejectMin = -2.0;
opts.deepShadowRawTrackEarlyRejectEpochs = 2;
opts.deepShadowRawTrackEarlyRejectImproveMaxM = -200.0;
opts.deepShadowRawTrackClusterWindowImproveMinM = 0.0;
opts.deepShadowRawTrackClusterInstantFloorM = -150.0;
opts.deepShadowRawTrackClusterRejectFloorM = -600.0;
opts.deepShadowRawTrackClusterSoftRejectEpochs = 3;
opts.deepShadowRawTrackClusterTrendMeanMinM = -50.0;
opts.deepShadowRawTrackClusterTrendMinM = -300.0;
opts.deepShadowRawTrackClusterTrendPassHits = 3;
opts.deepShadowRawTrackClusterRecheckBudget = 2;
opts.deepShadowRawTrackClusterRecheckHoldEpochs = 2;
opts.deepShadowRawTrackClusterExtraMaxEpochs = 4;
opts.deepShadowRawRelayCooldownEpochs = 2;
opts.deepShadowRawCandReuseMaxEpochs = 6;
opts.deepShadowRawCandTopKUse = 3;
opts.deepShadowRawFailExcludeTolChips = 1.5;
opts.deepShadowRawFailExcludeTolFreqHz = 300;

% Quality maintenance: keep pass-ready from accumulating inconsistent tracks,
% and allow >4 nav-qualified sats only when they agree with the best4 core.
opts.deepShadowRawPassReadyAgingEnable = 1;
opts.deepShadowRawPassReadyBadEpochs = 2;
opts.deepShadowRawPassReadyMinSet = 4;
opts.deepShadowRawPassReadyDevThrM = 5000.0;
opts.deepShadowRawPassReadyAbsThrM = 20000.0;
opts.deepShadowRawNavExpandEnable = 1;
opts.deepShadowRawNavExpandDevThrM = 5000.0;

opts.deepShadowMainSwitchEnable = 1;
opts.deepShadowMainSwitchImproveMinM = 100.0;
opts.deepShadowMainSwitchConfirmEpochs = 3;
opts.deepShadowMainSwitchHoldEpochs = 6;
opts.deepShadowMainSwitchWaitProtectDone = 1;

opts.deepBypassPllInSpoof = 1;
opts.deepBypassDllInSpoof = 0;
end
