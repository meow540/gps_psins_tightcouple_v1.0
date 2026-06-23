function results = run_goal2_deep_400s_batch(opts)
% Run Chapter-2 deep-coupling pipeline on clean/ds5/ds6 with one parameter set.
%
% Usage:
%   results = run_goal2_deep_400s_batch();
%   results = run_goal2_deep_400s_batch(struct('roundTime', 120));

if nargin < 1 || isempty(opts)
    opts = struct();
end

opts = fillDefault(opts, 'roundTime', 800);  % 400s at 0.5s nav period
opts = fillDefault(opts, 'trjFile', 'E:\fgi_result\ins_simulation\ins1_trj.mat');
opts = fillDefault(opts, 'datasets', {'clean', 'ds5', 'ds6'});
opts = fillDefault(opts, 'skipMissingRaw', true);
opts = fillDefault(opts, 'outPrefix', 'rt_deep_goal2');
opts = fillDefault(opts, 'kCm', 4.0);
opts = fillDefault(opts, 'kDf', 4.0);
opts = fillDefault(opts, 'kSat', 4.0);
opts = fillDefault(opts, 'kZ', 4.0);
opts = fillDefault(opts, 'sigmaMeasHz', 1.5);
opts = fillDefault(opts, 'deepConfirmEpochs', 5);
opts = fillDefault(opts, 'deepSpoofConfirmMinSec', -inf);
opts = fillDefault(opts, 'deepSpoofForceConfirmSec', nan);
opts = fillDefault(opts, 'calibStartEpoch', 10);
opts = fillDefault(opts, 'calibEndEpoch', min(opts.roundTime, 180));
opts = fillDefault(opts, 'aidWeightNormal', 0.20);
opts = fillDefault(opts, 'aidWeightSuspect', 0.70);
opts = fillDefault(opts, 'aidWeightSpoof', 1.10);
opts = fillDefault(opts, 'clkWeightNormal', 1.00);
opts = fillDefault(opts, 'clkWeightSuspect', 1.00);
opts = fillDefault(opts, 'clkWeightSpoof', 1.00);
opts = fillDefault(opts, 'rhoBlendNormal', 0.00);
opts = fillDefault(opts, 'rhoBlendSuspect', 0.05);
opts = fillDefault(opts, 'rhoBlendSpoof', 0.30);
opts = fillDefault(opts, 'deepUseGnssKfUpdate', 1);
opts = fillDefault(opts, 'deepUseKfClockUpdate', 0);
opts = fillDefault(opts, 'deepUseGnssKfUpdateNormal', opts.deepUseGnssKfUpdate);
opts = fillDefault(opts, 'deepUseGnssKfUpdateSuspect', opts.deepUseGnssKfUpdate);
opts = fillDefault(opts, 'deepUseGnssKfUpdateSpoof', 0);
opts = fillDefault(opts, 'deepUseKfClockUpdateNormal', opts.deepUseKfClockUpdate);
opts = fillDefault(opts, 'deepUseKfClockUpdateSuspect', opts.deepUseKfClockUpdate);
opts = fillDefault(opts, 'deepUseKfClockUpdateSpoof', opts.deepUseKfClockUpdate);
opts = fillDefault(opts, 'deepRScaleNormal', 1.0);
opts = fillDefault(opts, 'deepRScaleSuspect', 4.0);
opts = fillDefault(opts, 'deepRScaleSpoof', 10.0);
opts = fillDefault(opts, 'deepShadowReacqEnable', 1);
opts = fillDefault(opts, 'deepShadowResidualClipHz', 20.0);
opts = fillDefault(opts, 'deepShadowLockGateHz', 25.0);
opts = fillDefault(opts, 'deepShadowStableEpochs', 2);
opts = fillDefault(opts, 'deepShadowMinRecoveredSat', 3);
opts = fillDefault(opts, 'deepShadowReleaseConfirmEpochs', 4);
opts = fillDefault(opts, 'deepShadowAllowReturnGnss', 1);
opts = fillDefault(opts, 'deepShadowRecoveryRampEpochs', 10);
opts = fillDefault(opts, 'deepShadowMaxPullHzPerStep', 2.0);
opts = fillDefault(opts, 'deepShadowWeightSuspect', 0.50);
opts = fillDefault(opts, 'deepShadowWeightSpoof', 0.90);
opts = fillDefault(opts, 'deepShadowTargetPureInsInSpoof', 1);
opts = fillDefault(opts, 'deepShadowUseTrackResidualGate', 1);
opts = fillDefault(opts, 'deepShadowUseCarrErrorGate', 1);
opts = fillDefault(opts, 'deepShadowCarrErrGateCycles', 0.20);
opts = fillDefault(opts, 'deepShadowReleaseUseTrackMetric', 1);
opts = fillDefault(opts, 'deepShadowReleaseNeedDetectorMetric', 0);
opts = fillDefault(opts, 'deepShadowReleaseTcmScale', 1.30);
opts = fillDefault(opts, 'deepShadowReleaseTdfScale', 1.30);
opts = fillDefault(opts, 'deepShadowReleaseTzScale', 1.20);
opts = fillDefault(opts, 'deepShadowReleaseTcmMin', 0.1);
opts = fillDefault(opts, 'deepShadowReleaseTdfMin', 0.1);
opts = fillDefault(opts, 'deepShadowReleaseTzMin', 0.1);
opts = fillDefault(opts, 'deepShadowReleaseTrackCmScale', 0.70);
opts = fillDefault(opts, 'deepShadowReleaseTrackDfScale', 0.70);
opts = fillDefault(opts, 'deepShadowReleaseTrackCmMin', 0.5);
opts = fillDefault(opts, 'deepShadowReleaseTrackDfMin', 0.5);
opts = fillDefault(opts, 'deepShadowReleaseUseNavResidual', 0);
opts = fillDefault(opts, 'deepShadowReleaseNavMedM', 80.0);
opts = fillDefault(opts, 'deepShadowReleaseNavP95M', 300.0);
opts = fillDefault(opts, 'deepShadowReleaseNavMaxM', 800.0);
opts = fillDefault(opts, 'deepForceShadowNcoInSuspect', 0);
opts = fillDefault(opts, 'deepForceShadowNcoInSpoof', 1);
opts = fillDefault(opts, 'deepBypassPllInSuspect', 0);
opts = fillDefault(opts, 'deepBypassPllInSpoof', 1);
opts = fillDefault(opts, 'deepBypassDllInSuspect', 0);
opts = fillDefault(opts, 'deepBypassDllInSpoof', 0);
opts = fillDefault(opts, 'deepCodeErrorScaleSuspect', 1.0);
opts = fillDefault(opts, 'deepCodeErrorScaleSpoof', 1.0);
opts = fillDefault(opts, 'deepFreezeCodeErrorInSpoof', 0);
opts = fillDefault(opts, 'deepCodeNcoStepLimitHz', inf);
opts = fillDefault(opts, 'deepCodeReacqEnable', 0);
opts = fillDefault(opts, 'deepCodeReacqMinMode', 1);
opts = fillDefault(opts, 'deepCodeReacqHalfChips', 0.50);
opts = fillDefault(opts, 'deepCodeReacqNarrowHalfChips', 0.16);
opts = fillDefault(opts, 'deepCodeReacqStepChips', 0.04);
opts = fillDefault(opts, 'deepCodeReacqNarrowAfter', 80);
opts = fillDefault(opts, 'deepCodeReacqPeakRatio', 1.05);
opts = fillDefault(opts, 'deepCodeReacqZeroRatio', 1.00);
opts = fillDefault(opts, 'deepCodeReacqMaxShiftChips', 0.08);
opts = fillDefault(opts, 'deepCodeReacqApplyGain', 0.50);
opts = fillDefault(opts, 'deepCodeReacqMinPromptMag', 0);
opts = fillDefault(opts, 'deepCodeReacqNavCorrEnable', 0);
opts = fillDefault(opts, 'deepCodeReacqNavCorrGain', 0.20);
opts = fillDefault(opts, 'deepCodeReacqNavCorrStepChips', 0.02);
opts = fillDefault(opts, 'deepCodeReacqNavCorrLimitChips', 0.50);
opts = fillDefault(opts, 'deepCodeReacqNavCorrDecay', 0.995);
opts = fillDefault(opts, 'deepCodeWideReacqEnable', 0);
opts = fillDefault(opts, 'deepCodeWideReacqMinMode', 2);
opts = fillDefault(opts, 'deepCodeWideReacqHalfChips', 5.0);
opts = fillDefault(opts, 'deepCodeWideReacqStepChips', 0.25);
opts = fillDefault(opts, 'deepCodeWideReacqIntervalMs', 50);
opts = fillDefault(opts, 'deepCodeWideReacqPeakRatio', 1.08);
opts = fillDefault(opts, 'deepCodeWideReacqZeroRatio', 1.03);
opts = fillDefault(opts, 'deepCodeWideReacqStableEpochs', 3);
opts = fillDefault(opts, 'deepCodeWideReacqCandidateTolChips', 0.50);
opts = fillDefault(opts, 'deepCodeWideReacqAccumMs', 10);
opts = fillDefault(opts, 'deepCodeWideReacqExcludeChips', 0.50);
opts = fillDefault(opts, 'deepCodeWideReacqUseSecondPeak', 1);
opts = fillDefault(opts, 'deepCodeWideReacqAccumRatio', 1.02);
opts = fillDefault(opts, 'deepCodeWideReacqCorrGain', 0.35);
opts = fillDefault(opts, 'deepCodeWideReacqCorrStepChips', 0.25);
opts = fillDefault(opts, 'deepCodeWideReacqCorrLimitChips', 5.0);
opts = fillDefault(opts, 'deepShadowAcqEnable', 0);
opts = fillDefault(opts, 'deepShadowAcqIntervalEpochs', 2);
opts = fillDefault(opts, 'deepShadowAcqHalfChips', 20.0);
opts = fillDefault(opts, 'deepShadowAcqStepChips', 0.50);
opts = fillDefault(opts, 'deepShadowAcqNoncohMs', 10);
opts = fillDefault(opts, 'deepShadowAcqExcludeChips', 0.75);
opts = fillDefault(opts, 'deepShadowAcqStableEpochs', 4);
opts = fillDefault(opts, 'deepShadowAcqOffsetTolChips', 1.0);
opts = fillDefault(opts, 'deepShadowAcqMinPeakRatio', 1.08);
opts = fillDefault(opts, 'deepShadowAcqMinZeroRatio', 1.05);
opts = fillDefault(opts, 'deepShadowAcqBoundaryGuardChips', 1.0);
opts = fillDefault(opts, 'deepShadowRawReacqEnable', 0);
opts = fillDefault(opts, 'deepShadowRawReacqIntervalEpochs', 4);
opts = fillDefault(opts, 'deepShadowRawReacqHalfChips', 20.0);
opts = fillDefault(opts, 'deepShadowRawReacqStepChips', 0.50);
opts = fillDefault(opts, 'deepShadowRawReacqNoncohMs', 20);
opts = fillDefault(opts, 'deepShadowRawReacqCoherentMs', 2);
opts = fillDefault(opts, 'deepShadowRawReacqFreqHalfHz', 1000);
opts = fillDefault(opts, 'deepShadowRawReacqFreqStepHz', 250);
opts = fillDefault(opts, 'deepShadowRawReacqExcludeChips', 0.75);
opts = fillDefault(opts, 'deepShadowRawReacqNmsChipGuard', 2.0);
opts = fillDefault(opts, 'deepShadowRawReacqNmsFreqGuardHz', 350);
opts = fillDefault(opts, 'deepShadowRawReacqTopK', 3);
opts = fillDefault(opts, 'deepShadowRawReacqFineHalfChips', 3.0);
opts = fillDefault(opts, 'deepShadowRawReacqFineStepChips', 0.25);
opts = fillDefault(opts, 'deepShadowRawReacqFineFreqHalfHz', 250);
opts = fillDefault(opts, 'deepShadowRawReacqFineFreqStepHz', 125);
opts = fillDefault(opts, 'deepShadowRawTrackEnable', 0);
opts = fillDefault(opts, 'deepShadowContinuousTrackEnable', 0);
opts = fillDefault(opts, 'deepShadowRawTrackPeakRatioMin', 1.03);
opts = fillDefault(opts, 'deepShadowRawTrackZeroRatioMin', 1.05);
opts = fillDefault(opts, 'deepShadowRawTrackProtectEpochs', 4);
opts = fillDefault(opts, 'deepShadowRawTrackValidateEpochs', 2);
opts = fillDefault(opts, 'deepShadowRawTrackValidateImproveMinM', 100.0);

opts = fillDefault(opts, 'deepShadowRawTrackValidateAbsDeltaEnable', 0);

opts = fillDefault(opts, 'deepShadowRawTrackValidateAbsDeltaMaxM', 3500.0);

opts = fillDefault(opts, 'deepShadowRawTrackValidateDuringHoldEnable', 0);
opts = fillDefault(opts, 'deepShadowRawTrackValidateMaxEpochs', 8);
opts = fillDefault(opts, 'deepShadowRawTrackValidateWindowEpochs', 4);
opts = fillDefault(opts, 'deepShadowRawTrackValidateWindowPassHits', 2);
opts = fillDefault(opts, 'deepShadowRawTrackScoreGoodImproveM', 200.0);
opts = fillDefault(opts, 'deepShadowRawTrackScoreBadImproveM', -300.0);
opts = fillDefault(opts, 'deepShadowRawTrackScoreRiseM', 150.0);
opts = fillDefault(opts, 'deepShadowRawTrackScorePassMin', 2.0);
opts = fillDefault(opts, 'deepShadowRawCandReuseMaxEpochs', 6);
opts = fillDefault(opts, 'deepShadowRawCandTopKUse', 3);
opts = fillDefault(opts, 'deepShadowRawFailExcludeTolChips', 1.5);
opts = fillDefault(opts, 'deepShadowRawFailExcludeTolFreqHz', 300);
opts = fillDefault(opts, 'deepShadowRawClusterAssocTolChips', 1.0);
opts = fillDefault(opts, 'deepShadowRawClusterAssocTolFreqHz', 250);
opts = fillDefault(opts, 'deepShadowRawClusterScoreDecay', 0.85);
opts = fillDefault(opts, 'deepShadowRawClusterHitGain', 1.0);
opts = fillDefault(opts, 'deepShadowRawClusterMissDecay', 0.75);
opts = fillDefault(opts, 'deepShadowRawClusterReadyScore', 2.0);
opts = fillDefault(opts, 'deepShadowRawClusterReadyHits', 2);
opts = fillDefault(opts, 'deepShadowRawClusterReadyMargin', 0.5);
opts = fillDefault(opts, 'deepShadowRawClusterValidateTolChips', 1.5);
opts = fillDefault(opts, 'deepShadowRawClusterValidateTolFreqHz', 300);
opts = fillDefault(opts, 'deepShadowRawClusterPassScore', 1.4);
opts = fillDefault(opts, 'deepShadowRawClusterPassHits', 3);
opts = fillDefault(opts, 'deepShadowRawClusterPassMargin', 0.6);
opts = fillDefault(opts, 'deepShadowRawClusterPassImproveRelaxM', 0.0);
opts = fillDefault(opts, 'deepShadowRawClusterInitBlendAlpha', 0.65);
opts = fillDefault(opts, 'deepShadowRawExplorePeakRatioMin', 1.00);
opts = fillDefault(opts, 'deepShadowRawExploreZeroRatioMin', 0.90);
opts = fillDefault(opts, 'deepShadowRawWeakVotePeakRatioMin', 1.00);
opts = fillDefault(opts, 'deepShadowRawWeakVoteZeroRatioMin', 0.12);
opts = fillDefault(opts, 'deepShadowRawWeakVoteMetricScale', 0.35);
opts = fillDefault(opts, 'deepShadowRawExploreClusterScore', 1.0);
opts = fillDefault(opts, 'deepShadowRawExploreClusterHits', 2);
opts = fillDefault(opts, 'deepShadowRawExploreClusterMargin', 0.10);
opts = fillDefault(opts, 'deepShadowRawTrackScoreRejectMin', -2.0);
opts = fillDefault(opts, 'deepShadowRawTrackEarlyRejectEpochs', 2);
opts = fillDefault(opts, 'deepShadowRawTrackEarlyRejectImproveMaxM', -200.0);
opts = fillDefault(opts, 'deepShadowRawClusterInitScore', 1.2);
opts = fillDefault(opts, 'deepShadowRawClusterInitHits', 3);
opts = fillDefault(opts, 'deepShadowRawClusterInitMargin', 0.15);
opts = fillDefault(opts, 'deepShadowRawClusterDirectInitScore', 1.6);
opts = fillDefault(opts, 'deepShadowRawClusterDirectInitHits', 4);
opts = fillDefault(opts, 'deepShadowRawClusterDirectInitMargin', 0.20);
opts = fillDefault(opts, 'deepShadowRawTrackClusterRejectFloorM', -600.0);
opts = fillDefault(opts, 'deepShadowRawTrackClusterSoftRejectEpochs', 3);
opts = fillDefault(opts, 'deepShadowRawTrackClusterRecheckBudget', 2);
opts = fillDefault(opts, 'deepShadowRawTrackClusterRecheckHoldEpochs', 2);
opts = fillDefault(opts, 'deepShadowRawTrackClusterExtraMaxEpochs', 4);
opts = fillDefault(opts, 'deepShadowRawTrackClusterWindowImproveMinM', 0.0);
opts = fillDefault(opts, 'deepShadowRawTrackClusterInstantFloorM', -150.0);
opts = fillDefault(opts, 'deepShadowRawTrackClusterTrendMeanMinM', -50.0);
opts = fillDefault(opts, 'deepShadowRawTrackClusterTrendMinM', -300.0);
opts = fillDefault(opts, 'deepShadowRawTrackClusterTrendPassHits', 3);
opts = fillDefault(opts, 'deepShadowRawRelayCooldownEpochs', 2);
opts = fillDefault(opts, 'deepShadowRawPassReadyAgingEnable', 0);
opts = fillDefault(opts, 'deepShadowRawPassReadyRevokeEnable', 1);
opts = fillDefault(opts, 'deepShadowRawPassUseActivePool', 0);
opts = fillDefault(opts, 'deepShadowRawPassReadyBadEpochs', 3);
opts = fillDefault(opts, 'deepShadowRawPassReadyMinSet', 4);
opts = fillDefault(opts, 'deepShadowRawPassReadyDevThrM', 6000.0);
opts = fillDefault(opts, 'deepShadowRawPassReadyAbsThrM', 20000.0);
opts = fillDefault(opts, 'deepShadowRawNavExpandEnable', 0);
opts = fillDefault(opts, 'deepShadowRawNavCoreDevThrM', inf);
opts = fillDefault(opts, 'deepShadowRawNavExpandDevThrM', 5000.0);
opts = fillDefault(opts, 'deepShadowRawAmbiguityEnable', 0);
opts = fillDefault(opts, 'deepShadowRawAmbiguityMaxChips', 80.0);
opts = fillDefault(opts, 'deepShadowRawAmbiguityStepMaxChips', 8.0);
opts = fillDefault(opts, 'deepShadowRawAmbiguityMinSat', 4);
opts = fillDefault(opts, 'deepShadowBranchSearchEnable', 0);
opts = fillDefault(opts, 'deepShadowBranchUseNavQ', 1);
opts = fillDefault(opts, 'deepShadowBranchMinSat', 5);
opts = fillDefault(opts, 'deepShadowBranchMaxChips', 120.0);
opts = fillDefault(opts, 'deepShadowBranchAmbigContinuityEnable', 1);
opts = fillDefault(opts, 'deepShadowBranchAmbigContinuityMaxAgeEpochs', 80);
opts = fillDefault(opts, 'deepShadowBranchAmbigContinuityStepMaxChips', 2.0);
opts = fillDefault(opts, 'deepShadowBranchAmbigContinuityTolChips', 0.75);
opts = fillDefault(opts, 'deepShadowBranchAmbigContinuityPenaltyM', 120.0);
opts = fillDefault(opts, 'deepShadowBranchAmbigContinuityJumpPenaltyM', 250.0);
opts = fillDefault(opts, 'deepShadowBranchPriorRmsMaxM', 500.0);
opts = fillDefault(opts, 'deepShadowBranchPostfitRmsMaxM', 300.0);
opts = fillDefault(opts, 'deepShadowBranchPdopMax', 20.0);
opts = fillDefault(opts, 'deepShadowBranchPosJumpMaxM', 3000.0);
opts = fillDefault(opts, 'deepShadowDs5BaselineTrustedRefEnable', 0);
opts = fillDefault(opts, 'deepShadowDs5BaselineTrustedRefUseVelocity', 1);
opts = fillDefault(opts, 'deepShadowDs5BaselineTrustedRefMaxSec', inf);
opts = fillDefault(opts, 'deepShadowDs5BranchAbsoluteConsistencyEnable', 0);
opts = fillDefault(opts, 'deepShadowDs5BranchAbsoluteConsistencyMaxDiffM', 1200.0);
opts = fillDefault(opts, 'deepShadowDs5BranchLocalCorrectionMaxM', 1200.0);
opts = fillDefault(opts, 'deepShadowDs5BranchLocalStepMaxM', 250.0);
opts = fillDefault(opts, 'deepShadowDs5BranchLocalIterMax', 3);
opts = fillDefault(opts, 'deepShadowDs5BranchLocalCorrPenaltyWeight', 0.10);
opts = fillDefault(opts, 'deepShadowDs5BranchObsGateEnable', 0);
opts = fillDefault(opts, 'deepShadowDs5BranchObsGateMinSat', 4);
opts = fillDefault(opts, 'deepShadowDs5BranchObsGateFloorM', 120.0);
opts = fillDefault(opts, 'deepShadowDs5BranchObsGateMadScale', 3.0);
opts = fillDefault(opts, 'deepShadowDs5BranchObsGateCeilM', 1500.0);
opts = fillDefault(opts, 'deepShadowTrustedAnchorEnable', 0);
opts = fillDefault(opts, 'deepShadowTrustedAnchorUseTruthTrj', 0);
opts = fillDefault(opts, 'deepShadowTrustedAnchorTimeSec', 90.0);
opts = fillDefault(opts, 'deepShadowTrustedAnchorUseVelocity', 1);
opts = fillDefault(opts, 'deepShadowTrustedAnchorAllowCurrentFallback', 1);
opts = fillDefault(opts, 'deepShadowBranchTakeoverConfirmEpochs', 3);
opts = fillDefault(opts, 'deepShadowBranchTakeoverPostfitRmsMaxM', 100.0);
opts = fillDefault(opts, 'deepShadowBranchTakeoverPdopMax', 10.0);
opts = fillDefault(opts, 'deepShadowBranchTakeoverMinSat', 5);
opts = fillDefault(opts, 'deepShadowBranchTakeoverRequireTrustedPrior', 1);
opts = fillDefault(opts, 'deepShadowBranchResetEnable', 0);
opts = fillDefault(opts, 'deepShadowBranchResetOnce', 1);
opts = fillDefault(opts, 'deepShadowBranchResetMaxJumpM', 50000.0);
opts = fillDefault(opts, 'deepShadowBranchResetVelBlend', 0.0);
opts = fillDefault(opts, 'deepShadowBranchResetClock', 1);
opts = fillDefault(opts, 'deepShadowBranchInjectEnable', 0);
opts = fillDefault(opts, 'deepShadowBranchInjectRScale', 25.0);
opts = fillDefault(opts, 'deepShadowBranchInjectMinSat', 5);
opts = fillDefault(opts, 'deepShadowBranchInjectResidualGateM', 300.0);
opts = fillDefault(opts, 'deepShadowRecoveryModeEnable', 0);
opts = fillDefault(opts, 'deepShadowRecoveryHoldEpochs', 80);
opts = fillDefault(opts, 'deepShadowRecoverySuppressMainGnss', 1);
opts = fillDefault(opts, 'deepShadowRecoveryContinueMinSat', opts.deepShadowBranchInjectMinSat);
opts = fillDefault(opts, 'deepShadowRecoveryContinuePostfitRmsMaxM', 120.0);
opts = fillDefault(opts, 'deepShadowRecoveryContinuePdopMax', 12.0);
opts = fillDefault(opts, 'deepShadowRecoveryRequireTrustedPrior', 1);
opts = fillDefault(opts, 'deepShadowDs5BranchTakeoverAllow4Sat', 1);
opts = fillDefault(opts, 'deepShadowDs5BranchTakeoverDetrendedP95MaxM', 180.0);
opts = fillDefault(opts, 'deepShadowDs5BranchContinueAllow4Sat', 1);
opts = fillDefault(opts, 'deepShadowDs5BranchContinueDetrendedP95MaxM', 220.0);
opts = fillDefault(opts, 'deepShadowRecoveryReanchorEnable', 1);
opts = fillDefault(opts, 'deepShadowRecoveryReanchorMinJumpM', 500.0);
opts = fillDefault(opts, 'deepShadowRecoveryReanchorCooldownEpochs', 3);
opts = fillDefault(opts, 'deepShadowRecoveryReanchorMaxCount', 20);
opts = fillDefault(opts, 'deepShadowClosedLoopUseBranchOutput', 1);
opts = fillDefault(opts, 'deepShadowDs5SoftRecoveryEnable', 0);
opts = fillDefault(opts, 'deepShadowDs5SoftRecoveryStartSec', 92.0);
opts = fillDefault(opts, 'deepShadowDs5SoftRecoveryRequireBranchSane', 1);
opts = fillDefault(opts, 'deepShadowDs5ClosedLiftPreferEnable', 0);
opts = fillDefault(opts, 'deepShadowDs5ClosedLiftPreferStartSec', 118.0);
opts = fillDefault(opts, 'deepShadowDs5RawLiftRejectDevM', 12000.0);
opts = fillDefault(opts, 'deepShadowDs5CommonDoppEnable', 0);
opts = fillDefault(opts, 'deepShadowDs5CommonDoppStartSec', 92.0);
opts = fillDefault(opts, 'deepShadowDs5CommonDoppFocusPrns', [27 3]);
opts = fillDefault(opts, 'deepShadowDs5CommonDoppMinBaseSat', 3);
opts = fillDefault(opts, 'deepShadowDs5CommonDoppGateFloorHz', 10.0);
opts = fillDefault(opts, 'deepShadowDs5CommonDoppMadScale', 4.0);
opts = fillDefault(opts, 'deepShadowDs5CommonDoppMaxInnovHz', 80.0);
opts = fillDefault(opts, 'deepShadowDs5CommonDoppMaxRateHzps', 120.0);
opts = fillDefault(opts, 'deepShadowDs5CommonDoppApplyToAidEnable', 1);
opts = fillDefault(opts, 'deepShadowDs5CommonDoppAlphaGateEnable', 0);
opts = fillDefault(opts, 'deepShadowDs5CommonDoppAlphaProxyMin', 0.0);
opts = fillDefault(opts, 'deepShadowDs5CommonDoppAidApplyMaxHz', 200.0);
opts = fillDefault(opts, 'deepShadowDs5TruePeakEnable', 0);
opts = fillDefault(opts, 'deepShadowDs5TruePeakStartSec', 92.0);
opts = fillDefault(opts, 'deepShadowDs5TruePeakFreqWindowHz', 125.0);
opts = fillDefault(opts, 'deepShadowDs5TruePeakCodeWindowChips', 0.75);
opts = fillDefault(opts, 'deepShadowDs5TruePeakCodeWindowMaxChips', 1.50);
opts = fillDefault(opts, 'deepShadowDs5TruePeakUseCodeRef', 1);
opts = fillDefault(opts, 'deepShadowDs5RefObsModelEnable', 0);
opts = fillDefault(opts, 'deepShadowDs5RefObsModelStartSec', 92.0);
opts = fillDefault(opts, 'deepShadowDs5RefObsAutoActivate', 1);
opts = fillDefault(opts, 'deepShadowDs5RefObsMinBaseSat', 3);
opts = fillDefault(opts, 'deepShadowDs5RefObsLocalCorrMaxChips', 1.50);
opts = fillDefault(opts, 'deepShadowDs5RefObsUseCodeCorr', 1);
opts = fillDefault(opts, 'deepShadowDs5RefObsRequireActiveTrack', 0);
opts = fillDefault(opts, 'deepShadowDs5CodeRefHalfChips', 0.50);
opts = fillDefault(opts, 'deepShadowDs5CodeRefStepChips', 0.10);
opts = fillDefault(opts, 'deepShadowDs5CodeRefPeakRatioMin', 1.01);
opts = fillDefault(opts, 'deepShadowDs5CodeRefZeroRatioMin', 1.00);
opts = fillDefault(opts, 'deepShadowDs5CodeRefApplyGain', 0.50);
opts = fillDefault(opts, 'deepShadowDs5CodeRefStepMaxChips', 0.08);
opts = fillDefault(opts, 'deepShadowDs5CodeRefTotalMaxChips', 1.50);
opts = fillDefault(opts, 'deepShadowDs5CodeRefMinPromptMag', 0);
opts = fillDefault(opts, 'deepShadowDs5CodeRefNcoPullEnable', 1);
opts = fillDefault(opts, 'deepShadowDs5CodeRefNcoGain', 1.0);
opts = fillDefault(opts, 'deepShadowDs5CodeRefNcoMaxHz', 120.0);
opts = fillDefault(opts, 'deepShadowDs5RefObsRecoveryEnable', 1);
opts = fillDefault(opts, 'deepShadowDs5RefObsRecoveryStartSec', 92.0);
opts = fillDefault(opts, 'deepShadowDs5RefObsRecoveryMinSat', 4);
opts = fillDefault(opts, 'deepShadowDs5RefObsRecoveryMinUsedSat', 4);
opts = fillDefault(opts, 'deepShadowDs5RefObsRecoveryCodeMedMaxChips', 0.50);
opts = fillDefault(opts, 'deepShadowDs5RefObsRecoveryCodeP95MaxChips', 1.20);
opts = fillDefault(opts, 'deepShadowDs5RefObsRecoveryFreqP95MaxHz', 300.0);
opts = fillDefault(opts, 'deepShadowDs5RefObsRecoveryRobustEnable', 0);
opts = fillDefault(opts, 'deepShadowDs5RefObsRecoveryRobustMaxDropSat', 2);
opts = fillDefault(opts, 'deepShadowDs5RefObsPositionUseRecoveryKeep', 1);
opts = fillDefault(opts, 'deepShadowDs5RefObsRecoveryRequireBranch', 0);
opts = fillDefault(opts, 'deepShadowDs5RefObsRecoveryBranchPostfitMaxM', 250.0);
opts = fillDefault(opts, 'deepShadowDs5RefObsRecoveryBaselineDiffMaxM', 2500.0);
opts = fillDefault(opts, 'deepShadowDs5ObsContractEnable', 1);
opts = fillDefault(opts, 'deepShadowDs5ObsContractMinSat', 4);
opts = fillDefault(opts, 'deepShadowDs5ObsContractSpreadP95MaxM', 800.0);
opts = fillDefault(opts, 'deepShadowDs5ObsContractRequireCommonClock', 1);
opts = fillDefault(opts, 'deepShadowDs5ObsContractRequireTrackingPass', 1);
opts = fillDefault(opts, 'deepShadowDs5ObsContractMaxBaseSeedFrac', 0.75);
opts = fillDefault(opts, 'deepShadowDs5ObsContractNoBaselineSeedForRecovery', 0);
opts = fillDefault(opts, 'deepShadowDs5ObsContractUseRecoveryKeep', 0);
opts = fillDefault(opts, 'deepShadowDs5ObsContractRobustTrackingEnable', 0);
opts = fillDefault(opts, 'deepShadowDs5ObsContractRobustTrackingMaxDropSat', 2);
opts = fillDefault(opts, 'deepShadowDs5ObsContractIndependentCodeErrMaxChips', 1.60);
opts = fillDefault(opts, 'deepShadowDs5ObsContractIndependentFreqErrMaxHz', 350.0);
opts = fillDefault(opts, 'deepShadowDs5RefObsPositionCommonClockRebaseEnable', 1);
opts = fillDefault(opts, 'deepShadowDs5RefObsPositionCommonClockRebasePeriodM', 299792.458);
opts = fillDefault(opts, 'deepShadowDs5RefObsPositionRawAnchorLiftEnable', 1);
opts = fillDefault(opts, 'deepShadowDs5RefObsPositionRawAnchorLiftMaxSpreadP95M', 800.0);
opts = fillDefault(opts, 'deepShadowDs5RefObsPositionRawAnchorLiftUseCodeCorr', 1);
opts = fillDefault(opts, 'deepShadowDs5FinalForceRefObsRecovered', 0);
opts = fillDefault(opts, 'deepShadowDs5FinalQualitySelectEnable', 1);
opts = fillDefault(opts, 'deepShadowDs5FinalQualityImproveMinM', -10.0);
opts = fillDefault(opts, 'deepShadowDs5FinalQualityBaselineDiffMaxM', 180.0);
opts = fillDefault(opts, 'deepShadowDs5FinalRecoveredAuthorityRequireBaselineGate', 1);
opts = fillDefault(opts, 'deepShadowDs5FinalRecoveredAuthorityMaxBaselineDiffM', opts.deepShadowDs5FinalQualityBaselineDiffMaxM);
opts = fillDefault(opts, 'deepShadowDs5FinalAllowNoBaselineRecovered', 0);
opts = fillDefault(opts, 'deepShadowDs5FinalRecoveredConfirmEpochs', 3);
opts = fillDefault(opts, 'deepShadowDs5FinalContinuityGateEnable', 1);
opts = fillDefault(opts, 'deepShadowDs5FinalContinuityMaxPredDiffM', 60.0);
opts = fillDefault(opts, 'deepShadowDs5FinalContinuitySlewEnable', 1);
opts = fillDefault(opts, 'deepShadowDs5FinalContinuityMaxAgeEpochs', 6);
opts = fillDefault(opts, 'deepShadowDs5FinalHoldBaselineCompeteEnable', 1);
opts = fillDefault(opts, 'deepShadowDs5FinalHoldBaselineMinAgeEpochs', 2);
opts = fillDefault(opts, 'deepShadowDs5FinalHoldBaselineProxyMarginM', 20.0);
opts = fillDefault(opts, 'deepShadowDs5FinalHoldBaselineDiffMaxM', 220.0);
opts = fillDefault(opts, 'deepShadowDs5FinalHoldBaselineForceDiffM', 220.0);
opts = fillDefault(opts, 'deepShadowDs5FinalHoldBaselineMaxAgeEpochs', 6);
opts = fillDefault(opts, 'deepShadowDs5FinalRecoveredHoldAllowObsGap', 0);
opts = fillDefault(opts, 'deepShadowDs5FinalRecoveredHoldAllowAuthorityGap', 0);
opts = fillDefault(opts, 'deepShadowDs5FinalRecoveredHoldSuppressBaselineCompete', 0);
opts = fillDefault(opts, 'deepShadowDs5FinalRecoveredHoldBaselineDiffMaxM', opts.deepShadowDs5RecoveredFilterAuthorityBaselineDiffMaxM);
opts = fillDefault(opts, 'deepShadowDs5FinalRecoveredAuthorityEnable', 1);
opts = fillDefault(opts, 'deepShadowDs5FinalBaselineOffAblation', 0);
opts = fillDefault(opts, 'deepShadowDs5FinalRequireObsContractForRecovered', 1);
opts = fillDefault(opts, 'deepShadowDs5FinalRequireObsContractForHold', 1);
opts = fillDefault(opts, 'deepShadowDs5FinalAllowObsContractTrackingHold', 0);
opts = fillDefault(opts, 'deepShadowDs5FinalAllowObsContractBaseSeedHold', 0);
opts = fillDefault(opts, 'deepShadowDs5RecoveredFilterEnable', 1);
opts = fillDefault(opts, 'deepShadowDs5RecoveredFilterConfirmEpochs', 3);
opts = fillDefault(opts, 'deepShadowDs5RecoveredFilterMaxBadEpochs', 24);
opts = fillDefault(opts, 'deepShadowDs5RecoveredFilterMaxCoastEpochs', 80);
opts = fillDefault(opts, 'deepShadowDs5RecoveredFilterUpdateAlpha', 0.85);
opts = fillDefault(opts, 'deepShadowDs5RecoveredFilterVelMeasBlend', 0.35);
opts = fillDefault(opts, 'deepShadowDs5RecoveredFilterVelInsBlend', 0.10);
opts = fillDefault(opts, 'deepShadowDs5RecoveredFilterMeasMaxPredDiffM', 350.0);
opts = fillDefault(opts, 'deepShadowDs5RecoveredFilterPropagateWithInsVel', 1);
opts = fillDefault(opts, 'deepShadowDs5RecoveredFilterClockEnable', 1);
opts = fillDefault(opts, 'deepShadowDs5RecoveredFilterClockUpdateAlpha', 0.85);
opts = fillDefault(opts, 'deepShadowDs5RecoveredFilterClockRateBlend', 0.35);
opts = fillDefault(opts, 'deepShadowDs5RecoveredFilterClockMeasMaxPredDiffM', 350.0);
opts = fillDefault(opts, 'deepShadowDs5RecoveredFilterClockHardGateEnable', 0);
opts = fillDefault(opts, 'deepShadowDs5RecoveredFilterClockResetDiffMaxM', 1500.0);
opts = fillDefault(opts, 'deepShadowDs5RecoveredFilterResetOnContractMeas', 1);
opts = fillDefault(opts, 'deepShadowDs5RecoveredFilterResetMeasPredDiffM', 1500.0);
opts = fillDefault(opts, 'deepShadowDs5RecoveredFilterAbsAnchorGateRequire', 1);
opts = fillDefault(opts, 'deepShadowDs5RecoveredFilterAuthorityRequireAbsAnchor', 1);
opts = fillDefault(opts, 'deepShadowDs5RecoveredFilterPropCapEnable', 1);
opts = fillDefault(opts, 'deepShadowDs5RecoveredFilterPropMaxStepM', 120.0);
opts = fillDefault(opts, 'deepShadowDs5RecoveredFilterPropMaxTotalM', 600.0);
opts = fillDefault(opts, 'deepShadowDs5RecoveredFilterPredMaxClockStepM', 600.0);
opts = fillDefault(opts, 'deepShadowDs5RecoveredFilterAuthorityMaxCoastEpochs', 12);
opts = fillDefault(opts, 'deepShadowDs5RecoveredFilterAuthorityBaselineDiffMaxM', 700.0);
opts = fillDefault(opts, 'deepShadowDs5RecoveredFilterAuthorityUseBaselineDiffHardCap', 0);
opts = fillDefault(opts, 'deepShadowDs5RecoveredFilterAuthorityMeasPredDiffMaxM', 500.0);
opts = fillDefault(opts, 'deepShadowDs5RecoveredFilterAuthorityAbsAnchorMaxM', 1800.0);
opts = fillDefault(opts, 'deepShadowDs5RecoveredFilterAbsAnchorEnable', 1);
opts = fillDefault(opts, 'deepShadowDs5RecoveredFilterAbsAnchorSoftMaxDiffM', 900.0);
opts = fillDefault(opts, 'deepShadowDs5RecoveredFilterAbsAnchorMaxDiffM', 1800.0);
opts = fillDefault(opts, 'deepShadowDs5RecoveredFilterAbsAnchorBlend', 0.02);
opts = fillDefault(opts, 'deepShadowDs5RecoveredFilterAbsAnchorBlendHigh', 0.12);
opts = fillDefault(opts, 'deepShadowDs5RecoveredFilterAbsAnchorAlphaMinScale', 0.15);
opts = fillDefault(opts, 'deepShadowDs5RecoveredFilterDopplerVelEnable', 1);
opts = fillDefault(opts, 'deepShadowDs5RecoveredFilterDopplerVelMinSat', 4);
opts = fillDefault(opts, 'deepShadowDs5RecoveredFilterDopplerVelMaxResidMps', 85.0);
opts = fillDefault(opts, 'deepShadowDs5RecoveredFilterDopplerVelBlend', 0.35);
opts = fillDefault(opts, 'deepShadowDs5RecoveredFilterDopplerVelRobustMaxDropSat', 2);
opts = fillDefault(opts, 'deepShadowDs5RefObsPositionRobustSubsetEnable', 1);
opts = fillDefault(opts, 'deepShadowDs5RefObsPositionRobustMaxDropSat', 2);
opts = fillDefault(opts, 'deepShadowDs5RefObsPositionRobustDropPenaltyM', 20.0);
opts = fillDefault(opts, 'deepShadowDs5TruePeakMetricWeight', 0.20);
opts = fillDefault(opts, 'deepShadowDs5TruePeakFreqWeight', 0.40);
opts = fillDefault(opts, 'deepShadowDs5TruePeakCodeWeight', 0.20);
opts = fillDefault(opts, 'deepShadowDs5TruePeakResidualWeight', 0.20);
opts = fillDefault(opts, 'deepShadowDs5TruePeakCurrentMetricBias', 0.35);
opts = fillDefault(opts, 'deepShadowDs5TruePeakCandidateMargin', 0.10);
opts = fillDefault(opts, 'deepShadowDs5TruePeakProbationEpochs', 4);
opts = fillDefault(opts, 'deepShadowDs5TruePeakProbationWindowEpochs', 6);
opts = fillDefault(opts, 'deepShadowDs5TruePeakProbationPassHits', 4);
opts = fillDefault(opts, 'deepShadowDs5TruePeakTakeoverConfirmEpochs', 3);
opts = fillDefault(opts, 'deepShadowDs5TruePeakRevokeBadEpochs', 2);
opts = fillDefault(opts, 'deepShadowDs5TruePeakRevokeFreqErrHz', 180.0);
opts = fillDefault(opts, 'deepShadowDs5TruePeakRevokeCodeErrChips', 1.25);
opts = fillDefault(opts, 'deepShadowDs5TruePeakRevokeImproveMaxM', -150.0);
opts = fillDefault(opts, 'deepShadowDs5SoftClampEnable', 0);
opts = fillDefault(opts, 'deepShadowDs5SoftClampStartSec', 150.0);
opts = fillDefault(opts, 'deepShadowDs5SoftClampAlpha', 0.35);
opts = fillDefault(opts, 'deepShadowDs5SoftClampVelBlend', 0.25);
opts = fillDefault(opts, 'deepShadowDs5FinalOutputEnable', 0);
opts = fillDefault(opts, 'deepShadowDs5FinalOutputHoldEpochs', 120);
opts = fillDefault(opts, 'deepShadowDs5FinalRecoveryBaselineGateEnable', 1);
opts = fillDefault(opts, 'deepShadowDs5FinalRecoveryBaselineMaxDiffM', 350.0);
opts = fillDefault(opts, 'deepShadowDs5FinalRecoveryImproveGateEnable', 1);
opts = fillDefault(opts, 'deepShadowDs5FinalRecoveryImproveMinM', 120.0);
opts = fillDefault(opts, 'deepShadowDs5RefObsClosedLoopUseBaselineHold', 1);
opts = fillDefault(opts, 'deepShadowDs5RefObsClosedLoopPreferBaselineHold', 1);
opts = fillDefault(opts, 'deepShadowDs5RefObsPositionEnable', 1);
opts = fillDefault(opts, 'deepShadowDs5RefObsPositionMinSat', 4);
opts = fillDefault(opts, 'deepShadowDs5RefObsPositionPostfitMaxM', 220.0);
opts = fillDefault(opts, 'deepShadowDs5RefObsPositionAdaptivePostfitMaxM', opts.deepShadowDs5RefObsPositionPostfitMaxM);
opts = fillDefault(opts, 'deepShadowDs5RefObsPositionPdopMax', 20.0);
opts = fillDefault(opts, 'deepShadowDs5RefObsPositionCorrMaxM', 260.0);
opts = fillDefault(opts, 'deepShadowDs5RefObsPositionAnchorDiffMaxM', 260.0);
opts = fillDefault(opts, 'deepShadowDs5RefObsPositionJumpMaxM', 220.0);
opts = fillDefault(opts, 'deepShadowDs5RefObsPositionAdaptiveJumpMaxM', 450.0);
opts = fillDefault(opts, 'deepShadowDs5RefObsPositionJumpMaxAgeEpochs', 4);
opts = fillDefault(opts, 'deepShadowDs5RefObsPositionSpeedGateEnable', 1);
opts = fillDefault(opts, 'deepShadowDs5RefObsPositionSpeedMaxMps', 140.0);
opts = fillDefault(opts, 'deepShadowDs5RefObsPositionAdaptiveSpeedMaxMps', 450.0);
opts = fillDefault(opts, 'deepShadowDs5RefObsPositionAccelMaxMps2', 180.0);
opts = fillDefault(opts, 'deepShadowDs5RefObsPositionAdaptiveAccelMaxMps2', 700.0);
opts = fillDefault(opts, 'deepShadowDs5RefObsPositionDynGateMaxAgeEpochs', 4);
opts = fillDefault(opts, 'deepShadowClosedLoopCoastEpochs', 20);
opts = fillDefault(opts, 'deepShadowClosedLoopUseVelocityCoast', 1);
opts = fillDefault(opts, 'deepShadowClosedLoopAllowNavFallbackInRecovery', 1);
opts = fillDefault(opts, 'deepShadowClosedLoopUseTrustedFallback', 1);
opts = fillDefault(opts, 'deepShadowClosedLoopTrustedBlendEnable', 0);
opts = fillDefault(opts, 'deepShadowClosedLoopTrustedBlendBranchAlpha', 0.0);
opts = fillDefault(opts, 'deepShadowClosedLoopTrustedBlendCoastAlpha', 0.0);
opts = fillDefault(opts, 'deepShadowClosedLoopSafeBranchGateEnable', 1);
opts = fillDefault(opts, 'deepShadowClosedLoopSafeBranchMaxDiffM', 450.0);
opts = fillDefault(opts, 'deepShadowClosedLoopSafeBranchPostfitMaxM', 80.0);
opts = fillDefault(opts, 'deepShadowClosedLoopSafeBranchPdopMax', 8.0);
opts = fillDefault(opts, 'deepShadowClosedLoopSafeBranchMinSat', 5);
opts = fillDefault(opts, 'deepShadowClosedLoopSafeBranchAlpha', 0.25);
opts = fillDefault(opts, 'deepShadowCoreServoEnable', 0);
opts = fillDefault(opts, 'deepShadowCoreServoGain', 0.05);
opts = fillDefault(opts, 'deepShadowCoreServoStepMaxChips', 0.02);
opts = fillDefault(opts, 'deepShadowCoreServoTotalMaxChips', 1.0);
opts = fillDefault(opts, 'deepShadowCoreServoMinSat', 4);
opts = fillDefault(opts, 'deepShadowCoreServoCoreDevMaxM', 10000.0);
opts = fillDefault(opts, 'deepShadowCoreServoMinElevationDeg', 0.0);
opts = fillDefault(opts, 'deepShadowMainSwitchEnable', 0);
opts = fillDefault(opts, 'deepShadowMainSwitchImproveMinM', 100.0);
opts = fillDefault(opts, 'deepShadowMainSwitchConfirmEpochs', 3);
opts = fillDefault(opts, 'deepShadowMainSwitchHoldEpochs', 6);
opts = fillDefault(opts, 'deepShadowMainSwitchWaitProtectDone', 1);
opts = fillDefault(opts, 'deepPrInnovationGateEnable', 0);
opts = fillDefault(opts, 'deepPrInnovationGateK', 4.0);
opts = fillDefault(opts, 'deepPrInnovationGateMinM', 20.0);
opts = fillDefault(opts, 'deepPrInnovationGateMaxM', 300.0);
opts = fillDefault(opts, 'deepPrInnovationGateMinSat', 4);
opts = fillDefault(opts, 'deepPrInnovationGateUseMedianDetrend', 1);
opts = fillDefault(opts, 'deepRecoveryHoldEpochs', 16);
opts = fillDefault(opts, 'deepReentryStrictEpochs', 30);
opts = fillDefault(opts, 'deepReentryTcmScale', 1.50);
opts = fillDefault(opts, 'deepReentryTdfScale', 1.50);
opts = fillDefault(opts, 'deepReentryTsatScale', 1.50);
opts = fillDefault(opts, 'deepReentryTzScale', 1.30);
opts = fillDefault(opts, 'deepReentryConfirmEpochs', max(opts.deepConfirmEpochs + 2, 5));
opts = fillDefault(opts, 'deepUseIndependentDetectTrack', 0);
opts = fillDefault(opts, 'deepDetectUseTrackResults', 1);
opts = fillDefault(opts, 'deepFastMode', 0);
opts = fillDefault(opts, 'deepShadowDs5FastRefObsTrackBypass', 1);
opts = fillDefault(opts, 'deepShadowDs5RefObsTrackRawStrideEpochs', 1);
opts = fillDefault(opts, 'deepShadowDs5RefObsTrackRawBurstMs', inf);
opts = fillDefault(opts, 'deepShadowDs5RefObsTrackRawWarmupEpochs', 0);
opts = fillDefault(opts, 'deepPerfLightNormalMode', 1);
opts = fillDefault(opts, 'deepPerfDiagPrint', 1);
opts = fillDefault(opts, 'deepPerfDiagInterval', 10);
opts = fillDefault(opts, 'deepKeepTrackHistory', 0);
opts = fillDefault(opts, 'deepUseReferenceResidual', 1);
opts = fillDefault(opts, 'deepShadowUseNavQualifiedForKf', 0);
opts = fillDefault(opts, 'deepShadowKfMinSat', 4);
opts = fillDefault(opts, 'deepShadowKfRScale', opts.deepRScaleSpoof);
opts = fillDefault(opts, 'deepShadowKfMedianDetrend', 1);

opts = fillDefault(opts, 'deepShadowKfDetrendGateM', 5000.0);
opts = fillDefault(opts, 'reuseCalib', false);
opts = fillDefault(opts, 'calibFile', '');
opts = fillDefault(opts, 'saveCalibFile', '');
opts = fillDefault(opts, 'minTcm', 0.5);
opts = fillDefault(opts, 'minTdf', 0.3);
opts = fillDefault(opts, 'minTsat', 1.0);
opts = fillDefault(opts, 'minTz', 1.0);
if opts.deepFastMode && (opts.deepUseGnssKfUpdateNormal || opts.deepUseGnssKfUpdateSuspect || ...
        opts.deepUseGnssKfUpdateSpoof || opts.deepUseKfClockUpdateNormal || ...
        opts.deepUseKfClockUpdateSuspect || opts.deepUseKfClockUpdateSpoof)
    warning('[GOAL2] deepFastMode=1 is inconsistent with KF measurement update. Force deepFastMode=0.');
    opts.deepFastMode = 0;
end
if ~isfield(opts, 'deepDetrendWindowEpoch')
    if opts.deepUseReferenceResidual
        opts.deepDetrendWindowEpoch = 1;
    else
        opts.deepDetrendWindowEpoch = 80;
    end
end
if ~isfield(opts, 'hitMin')
    if opts.deepUseReferenceResidual
        opts.hitMin = 0;
    else
        opts.hitMin = 3;
    end
end
if ~isfield(opts, 'deepReentryHitMin')
    opts.deepReentryHitMin = max(opts.hitMin + 1, 1);
end

matMap = struct( ...
    'clean', 'cleandynamic_400s.mat', ...
    'ds5', 'ds5_400s.mat', ...
    'ds6', 'ds6_400s.mat');
rawMap = struct( ...
    'clean', 'E:\cleanDynamic.bin', ...
    'ds5', 'E:\ds5.bin', ...
    'ds6', 'F:\dataset\TEXBAT\ds6.bin');

% Base settings used in all runs.
baseOverride = struct();
baseOverride.trjFile = opts.trjFile;
baseOverride.deepRoundTime = opts.roundTime;
baseOverride.deepSpoofDetectEnable = 1;
baseOverride.deepSpoofLatch = 1;
baseOverride.deepHitMin = opts.hitMin;
baseOverride.deepConfirmEpochs = opts.deepConfirmEpochs;
baseOverride.deepSpoofConfirmMinSec = opts.deepSpoofConfirmMinSec;
baseOverride.deepSpoofForceConfirmSec = opts.deepSpoofForceConfirmSec;
baseOverride.deepSigmaMeasHz = opts.sigmaMeasHz;
baseOverride.deepKappaZ = 3.5;
baseOverride.deepTz = 3.5;
baseOverride.deepAidWeightNormal = opts.aidWeightNormal;
baseOverride.deepAidWeightSuspect = opts.aidWeightSuspect;
baseOverride.deepAidWeightSpoof = opts.aidWeightSpoof;
baseOverride.deepClkWeightNormal = opts.clkWeightNormal;
baseOverride.deepClkWeightSuspect = opts.clkWeightSuspect;
baseOverride.deepClkWeightSpoof = opts.clkWeightSpoof;
baseOverride.deepRhoBlendNormal = opts.rhoBlendNormal;
baseOverride.deepRhoBlendSuspect = opts.rhoBlendSuspect;
baseOverride.deepRhoBlendSpoof = opts.rhoBlendSpoof;
baseOverride.deepUseGnssKfUpdate = opts.deepUseGnssKfUpdate;
baseOverride.deepUseKfClockUpdate = opts.deepUseKfClockUpdate;
baseOverride.deepUseGnssKfUpdateNormal = opts.deepUseGnssKfUpdateNormal;
baseOverride.deepUseGnssKfUpdateSuspect = opts.deepUseGnssKfUpdateSuspect;
baseOverride.deepUseGnssKfUpdateSpoof = opts.deepUseGnssKfUpdateSpoof;
baseOverride.deepUseKfClockUpdateNormal = opts.deepUseKfClockUpdateNormal;
baseOverride.deepUseKfClockUpdateSuspect = opts.deepUseKfClockUpdateSuspect;
baseOverride.deepUseKfClockUpdateSpoof = opts.deepUseKfClockUpdateSpoof;
baseOverride.deepRScaleNormal = opts.deepRScaleNormal;
baseOverride.deepRScaleSuspect = opts.deepRScaleSuspect;
baseOverride.deepRScaleSpoof = opts.deepRScaleSpoof;
baseOverride.deepShadowReacqEnable = opts.deepShadowReacqEnable;
baseOverride.deepShadowResidualClipHz = opts.deepShadowResidualClipHz;
baseOverride.deepShadowLockGateHz = opts.deepShadowLockGateHz;
baseOverride.deepShadowStableEpochs = opts.deepShadowStableEpochs;
baseOverride.deepShadowMinRecoveredSat = opts.deepShadowMinRecoveredSat;
baseOverride.deepShadowReleaseConfirmEpochs = opts.deepShadowReleaseConfirmEpochs;
baseOverride.deepShadowAllowReturnGnss = opts.deepShadowAllowReturnGnss;
baseOverride.deepShadowRecoveryRampEpochs = opts.deepShadowRecoveryRampEpochs;
baseOverride.deepShadowMaxPullHzPerStep = opts.deepShadowMaxPullHzPerStep;
baseOverride.deepShadowWeightSuspect = opts.deepShadowWeightSuspect;
baseOverride.deepShadowWeightSpoof = opts.deepShadowWeightSpoof;
baseOverride.deepShadowTargetPureInsInSpoof = opts.deepShadowTargetPureInsInSpoof;
baseOverride.deepShadowUseTrackResidualGate = opts.deepShadowUseTrackResidualGate;
baseOverride.deepShadowUseCarrErrorGate = opts.deepShadowUseCarrErrorGate;
baseOverride.deepShadowCarrErrGateCycles = opts.deepShadowCarrErrGateCycles;
baseOverride.deepShadowRawReacqEnable = opts.deepShadowRawReacqEnable;
baseOverride.deepShadowRawReacqIntervalEpochs = opts.deepShadowRawReacqIntervalEpochs;
baseOverride.deepShadowRawReacqHalfChips = opts.deepShadowRawReacqHalfChips;
baseOverride.deepShadowRawReacqStepChips = opts.deepShadowRawReacqStepChips;
baseOverride.deepShadowRawReacqNoncohMs = opts.deepShadowRawReacqNoncohMs;
baseOverride.deepShadowRawReacqCoherentMs = opts.deepShadowRawReacqCoherentMs;
baseOverride.deepShadowRawReacqFreqHalfHz = opts.deepShadowRawReacqFreqHalfHz;
baseOverride.deepShadowRawReacqFreqStepHz = opts.deepShadowRawReacqFreqStepHz;
baseOverride.deepShadowRawReacqExcludeChips = opts.deepShadowRawReacqExcludeChips;
baseOverride.deepShadowRawReacqNmsChipGuard = opts.deepShadowRawReacqNmsChipGuard;
baseOverride.deepShadowRawReacqNmsFreqGuardHz = opts.deepShadowRawReacqNmsFreqGuardHz;
baseOverride.deepShadowRawReacqTopK = opts.deepShadowRawReacqTopK;
baseOverride.deepShadowRawReacqFineHalfChips = opts.deepShadowRawReacqFineHalfChips;
baseOverride.deepShadowRawReacqFineStepChips = opts.deepShadowRawReacqFineStepChips;
baseOverride.deepShadowRawReacqFineFreqHalfHz = opts.deepShadowRawReacqFineFreqHalfHz;
baseOverride.deepShadowRawReacqFineFreqStepHz = opts.deepShadowRawReacqFineFreqStepHz;
baseOverride.deepShadowRawTrackEnable = opts.deepShadowRawTrackEnable;
baseOverride.deepShadowContinuousTrackEnable = opts.deepShadowContinuousTrackEnable;
baseOverride.deepShadowRawTrackPeakRatioMin = opts.deepShadowRawTrackPeakRatioMin;
baseOverride.deepShadowRawTrackZeroRatioMin = opts.deepShadowRawTrackZeroRatioMin;
baseOverride.deepShadowRawTrackProtectEpochs = opts.deepShadowRawTrackProtectEpochs;
baseOverride.deepShadowRawTrackValidateEpochs = opts.deepShadowRawTrackValidateEpochs;
baseOverride.deepShadowRawTrackValidateImproveMinM = opts.deepShadowRawTrackValidateImproveMinM;

baseOverride.deepShadowRawTrackValidateAbsDeltaEnable = opts.deepShadowRawTrackValidateAbsDeltaEnable;

baseOverride.deepShadowRawTrackValidateAbsDeltaMaxM = opts.deepShadowRawTrackValidateAbsDeltaMaxM;

baseOverride.deepShadowRawTrackValidateDuringHoldEnable = opts.deepShadowRawTrackValidateDuringHoldEnable;
baseOverride.deepShadowRawTrackValidateMaxEpochs = opts.deepShadowRawTrackValidateMaxEpochs;
baseOverride.deepShadowRawTrackValidateWindowEpochs = opts.deepShadowRawTrackValidateWindowEpochs;
baseOverride.deepShadowRawTrackValidateWindowPassHits = opts.deepShadowRawTrackValidateWindowPassHits;
baseOverride.deepShadowRawTrackScoreGoodImproveM = opts.deepShadowRawTrackScoreGoodImproveM;
baseOverride.deepShadowRawTrackScoreBadImproveM = opts.deepShadowRawTrackScoreBadImproveM;
baseOverride.deepShadowRawTrackScoreRiseM = opts.deepShadowRawTrackScoreRiseM;
baseOverride.deepShadowRawTrackScorePassMin = opts.deepShadowRawTrackScorePassMin;
baseOverride.deepShadowRawCandReuseMaxEpochs = opts.deepShadowRawCandReuseMaxEpochs;
baseOverride.deepShadowRawCandTopKUse = opts.deepShadowRawCandTopKUse;
baseOverride.deepShadowRawFailExcludeTolChips = opts.deepShadowRawFailExcludeTolChips;
baseOverride.deepShadowRawFailExcludeTolFreqHz = opts.deepShadowRawFailExcludeTolFreqHz;
baseOverride.deepShadowRawClusterAssocTolChips = opts.deepShadowRawClusterAssocTolChips;
baseOverride.deepShadowRawClusterAssocTolFreqHz = opts.deepShadowRawClusterAssocTolFreqHz;
baseOverride.deepShadowRawClusterScoreDecay = opts.deepShadowRawClusterScoreDecay;
baseOverride.deepShadowRawClusterHitGain = opts.deepShadowRawClusterHitGain;
baseOverride.deepShadowRawClusterMissDecay = opts.deepShadowRawClusterMissDecay;
baseOverride.deepShadowRawClusterReadyScore = opts.deepShadowRawClusterReadyScore;
baseOverride.deepShadowRawClusterReadyHits = opts.deepShadowRawClusterReadyHits;
baseOverride.deepShadowRawClusterReadyMargin = opts.deepShadowRawClusterReadyMargin;
baseOverride.deepShadowRawClusterValidateTolChips = opts.deepShadowRawClusterValidateTolChips;
baseOverride.deepShadowRawClusterValidateTolFreqHz = opts.deepShadowRawClusterValidateTolFreqHz;
baseOverride.deepShadowRawClusterPassScore = opts.deepShadowRawClusterPassScore;
baseOverride.deepShadowRawClusterPassHits = opts.deepShadowRawClusterPassHits;
baseOverride.deepShadowRawClusterPassMargin = opts.deepShadowRawClusterPassMargin;
baseOverride.deepShadowRawClusterPassImproveRelaxM = opts.deepShadowRawClusterPassImproveRelaxM;
baseOverride.deepShadowRawClusterInitBlendAlpha = opts.deepShadowRawClusterInitBlendAlpha;
baseOverride.deepShadowRawExplorePeakRatioMin = opts.deepShadowRawExplorePeakRatioMin;
baseOverride.deepShadowRawExploreZeroRatioMin = opts.deepShadowRawExploreZeroRatioMin;
baseOverride.deepShadowRawWeakVotePeakRatioMin = opts.deepShadowRawWeakVotePeakRatioMin;
baseOverride.deepShadowRawWeakVoteZeroRatioMin = opts.deepShadowRawWeakVoteZeroRatioMin;
baseOverride.deepShadowRawWeakVoteMetricScale = opts.deepShadowRawWeakVoteMetricScale;
baseOverride.deepShadowRawExploreClusterScore = opts.deepShadowRawExploreClusterScore;
baseOverride.deepShadowRawExploreClusterHits = opts.deepShadowRawExploreClusterHits;
baseOverride.deepShadowRawExploreClusterMargin = opts.deepShadowRawExploreClusterMargin;
baseOverride.deepShadowRawTrackScoreRejectMin = opts.deepShadowRawTrackScoreRejectMin;
baseOverride.deepShadowRawTrackEarlyRejectEpochs = opts.deepShadowRawTrackEarlyRejectEpochs;
baseOverride.deepShadowRawTrackEarlyRejectImproveMaxM = opts.deepShadowRawTrackEarlyRejectImproveMaxM;
baseOverride.deepShadowRawClusterInitScore = opts.deepShadowRawClusterInitScore;
baseOverride.deepShadowRawClusterInitHits = opts.deepShadowRawClusterInitHits;
baseOverride.deepShadowRawClusterInitMargin = opts.deepShadowRawClusterInitMargin;
baseOverride.deepShadowRawClusterDirectInitScore = opts.deepShadowRawClusterDirectInitScore;
baseOverride.deepShadowRawClusterDirectInitHits = opts.deepShadowRawClusterDirectInitHits;
baseOverride.deepShadowRawClusterDirectInitMargin = opts.deepShadowRawClusterDirectInitMargin;
baseOverride.deepShadowRawTrackClusterRejectFloorM = opts.deepShadowRawTrackClusterRejectFloorM;
baseOverride.deepShadowRawTrackClusterSoftRejectEpochs = opts.deepShadowRawTrackClusterSoftRejectEpochs;
baseOverride.deepShadowRawTrackClusterRecheckBudget = opts.deepShadowRawTrackClusterRecheckBudget;
baseOverride.deepShadowRawTrackClusterRecheckHoldEpochs = opts.deepShadowRawTrackClusterRecheckHoldEpochs;
baseOverride.deepShadowRawTrackClusterExtraMaxEpochs = opts.deepShadowRawTrackClusterExtraMaxEpochs;
baseOverride.deepShadowRawTrackClusterWindowImproveMinM = opts.deepShadowRawTrackClusterWindowImproveMinM;
baseOverride.deepShadowRawTrackClusterInstantFloorM = opts.deepShadowRawTrackClusterInstantFloorM;
baseOverride.deepShadowRawTrackClusterTrendMeanMinM = opts.deepShadowRawTrackClusterTrendMeanMinM;
baseOverride.deepShadowRawTrackClusterTrendMinM = opts.deepShadowRawTrackClusterTrendMinM;
baseOverride.deepShadowRawTrackClusterTrendPassHits = opts.deepShadowRawTrackClusterTrendPassHits;
baseOverride.deepShadowRawRelayCooldownEpochs = opts.deepShadowRawRelayCooldownEpochs;
baseOverride.deepShadowRawPassReadyAgingEnable = opts.deepShadowRawPassReadyAgingEnable;
baseOverride.deepShadowRawPassReadyRevokeEnable = opts.deepShadowRawPassReadyRevokeEnable;
baseOverride.deepShadowRawPassUseActivePool = opts.deepShadowRawPassUseActivePool;
baseOverride.deepShadowRawPassReadyBadEpochs = opts.deepShadowRawPassReadyBadEpochs;
baseOverride.deepShadowRawPassReadyMinSet = opts.deepShadowRawPassReadyMinSet;
baseOverride.deepShadowRawPassReadyDevThrM = opts.deepShadowRawPassReadyDevThrM;
baseOverride.deepShadowRawPassReadyAbsThrM = opts.deepShadowRawPassReadyAbsThrM;
baseOverride.deepShadowRawNavExpandEnable = opts.deepShadowRawNavExpandEnable;
baseOverride.deepShadowRawNavCoreDevThrM = opts.deepShadowRawNavCoreDevThrM;
baseOverride.deepShadowRawNavExpandDevThrM = opts.deepShadowRawNavExpandDevThrM;
baseOverride.deepShadowScenarioMode = opts.deepShadowScenarioMode;
baseOverride.deepShadowDs5TailNavRelaxEnable = opts.deepShadowDs5TailNavRelaxEnable;
baseOverride.deepShadowDs5TailNavRelaxStartSec = opts.deepShadowDs5TailNavRelaxStartSec;
baseOverride.deepShadowDs5TailNavRelaxPrns = opts.deepShadowDs5TailNavRelaxPrns;
baseOverride.deepShadowDs5TailNavRelaxMinBaseSat = opts.deepShadowDs5TailNavRelaxMinBaseSat;
baseOverride.deepShadowDs5TailNavRelaxMinThrM = opts.deepShadowDs5TailNavRelaxMinThrM;
baseOverride.deepShadowDs5TailNavRelaxMaxThrM = opts.deepShadowDs5TailNavRelaxMaxThrM;
baseOverride.deepShadowDs5TailNavRelaxMadScale = opts.deepShadowDs5TailNavRelaxMadScale;
baseOverride.deepShadowDs5TailNavRelaxCoreScale = opts.deepShadowDs5TailNavRelaxCoreScale;
baseOverride.deepShadowDs5TailNavRelaxPairThrM = opts.deepShadowDs5TailNavRelaxPairThrM;
baseOverride.deepShadowDs5TailSourceFallbackEnable = opts.deepShadowDs5TailSourceFallbackEnable;
baseOverride.deepShadowDs5TailSourceFallbackStartSec = opts.deepShadowDs5TailSourceFallbackStartSec;
baseOverride.deepShadowDs5TailSourceFallbackPrns = opts.deepShadowDs5TailSourceFallbackPrns;
baseOverride.deepShadowDs5TailSourceFallbackUseClosedLift = opts.deepShadowDs5TailSourceFallbackUseClosedLift;
baseOverride.deepShadowDs5TailSourceFallbackUseHold = opts.deepShadowDs5TailSourceFallbackUseHold;
baseOverride.deepShadowDs5TailSourceFallbackDevThrM = opts.deepShadowDs5TailSourceFallbackDevThrM;
baseOverride.deepShadowDs5TailSourceFallbackJumpThrM = opts.deepShadowDs5TailSourceFallbackJumpThrM;
baseOverride.deepShadowDs5TailSourceFallbackHoldMaxAgeSec = opts.deepShadowDs5TailSourceFallbackHoldMaxAgeSec;
baseOverride.deepShadowDs5CommonDragEnable = opts.deepShadowDs5CommonDragEnable;
baseOverride.deepShadowDs5CommonDragStartSec = opts.deepShadowDs5CommonDragStartSec;
baseOverride.deepShadowDs5CommonDragFocusPrns = opts.deepShadowDs5CommonDragFocusPrns;
baseOverride.deepShadowDs5CommonDragMinBaseSat = opts.deepShadowDs5CommonDragMinBaseSat;
baseOverride.deepShadowDs5CommonDragGateFloorM = opts.deepShadowDs5CommonDragGateFloorM;
baseOverride.deepShadowDs5CommonDragMadScale = opts.deepShadowDs5CommonDragMadScale;
baseOverride.deepShadowDs5CommonDragMaxInnovM = opts.deepShadowDs5CommonDragMaxInnovM;
baseOverride.deepShadowDs5CommonDragMaxRateMps = opts.deepShadowDs5CommonDragMaxRateMps;
baseOverride.deepShadowDs5CommonDoppEnable = opts.deepShadowDs5CommonDoppEnable;
baseOverride.deepShadowDs5CommonDoppStartSec = opts.deepShadowDs5CommonDoppStartSec;
baseOverride.deepShadowDs5CommonDoppFocusPrns = opts.deepShadowDs5CommonDoppFocusPrns;
baseOverride.deepShadowDs5CommonDoppMinBaseSat = opts.deepShadowDs5CommonDoppMinBaseSat;
baseOverride.deepShadowDs5CommonDoppGateFloorHz = opts.deepShadowDs5CommonDoppGateFloorHz;
baseOverride.deepShadowDs5CommonDoppMadScale = opts.deepShadowDs5CommonDoppMadScale;
baseOverride.deepShadowDs5CommonDoppMaxInnovHz = opts.deepShadowDs5CommonDoppMaxInnovHz;
baseOverride.deepShadowDs5CommonDoppMaxRateHzps = opts.deepShadowDs5CommonDoppMaxRateHzps;
baseOverride.deepShadowDs5CommonDoppApplyToAidEnable = opts.deepShadowDs5CommonDoppApplyToAidEnable;
baseOverride.deepShadowDs5CommonDoppAlphaGateEnable = opts.deepShadowDs5CommonDoppAlphaGateEnable;
baseOverride.deepShadowDs5CommonDoppAlphaProxyMin = opts.deepShadowDs5CommonDoppAlphaProxyMin;
baseOverride.deepShadowDs5CommonDoppAidApplyMaxHz = opts.deepShadowDs5CommonDoppAidApplyMaxHz;
baseOverride.deepShadowDs5TruePeakEnable = opts.deepShadowDs5TruePeakEnable;
baseOverride.deepShadowDs5TruePeakStartSec = opts.deepShadowDs5TruePeakStartSec;
baseOverride.deepShadowDs5TruePeakFreqWindowHz = opts.deepShadowDs5TruePeakFreqWindowHz;
baseOverride.deepShadowDs5TruePeakCodeWindowChips = opts.deepShadowDs5TruePeakCodeWindowChips;
baseOverride.deepShadowDs5TruePeakCodeWindowMaxChips = opts.deepShadowDs5TruePeakCodeWindowMaxChips;
baseOverride.deepShadowDs5TruePeakUseCodeRef = opts.deepShadowDs5TruePeakUseCodeRef;
baseOverride.deepShadowDs5RefObsModelEnable = opts.deepShadowDs5RefObsModelEnable;
baseOverride.deepShadowDs5RefObsModelStartSec = opts.deepShadowDs5RefObsModelStartSec;
baseOverride.deepShadowDs5RefObsAutoActivate = opts.deepShadowDs5RefObsAutoActivate;
baseOverride.deepShadowDs5RefObsMinBaseSat = opts.deepShadowDs5RefObsMinBaseSat;
baseOverride.deepShadowDs5RefObsLocalCorrMaxChips = opts.deepShadowDs5RefObsLocalCorrMaxChips;
baseOverride.deepShadowDs5RefObsUseCodeCorr = opts.deepShadowDs5RefObsUseCodeCorr;
baseOverride.deepShadowDs5RefObsRequireActiveTrack = opts.deepShadowDs5RefObsRequireActiveTrack;
baseOverride.deepShadowDs5CodeRefHalfChips = opts.deepShadowDs5CodeRefHalfChips;
baseOverride.deepShadowDs5CodeRefStepChips = opts.deepShadowDs5CodeRefStepChips;
baseOverride.deepShadowDs5CodeRefPeakRatioMin = opts.deepShadowDs5CodeRefPeakRatioMin;
baseOverride.deepShadowDs5CodeRefZeroRatioMin = opts.deepShadowDs5CodeRefZeroRatioMin;
baseOverride.deepShadowDs5CodeRefApplyGain = opts.deepShadowDs5CodeRefApplyGain;
baseOverride.deepShadowDs5CodeRefStepMaxChips = opts.deepShadowDs5CodeRefStepMaxChips;
baseOverride.deepShadowDs5CodeRefTotalMaxChips = opts.deepShadowDs5CodeRefTotalMaxChips;
baseOverride.deepShadowDs5CodeRefMinPromptMag = opts.deepShadowDs5CodeRefMinPromptMag;
baseOverride.deepShadowDs5CodeRefNcoPullEnable = opts.deepShadowDs5CodeRefNcoPullEnable;
baseOverride.deepShadowDs5CodeRefNcoGain = opts.deepShadowDs5CodeRefNcoGain;
baseOverride.deepShadowDs5CodeRefNcoMaxHz = opts.deepShadowDs5CodeRefNcoMaxHz;
baseOverride.deepShadowDs5RefObsRecoveryEnable = opts.deepShadowDs5RefObsRecoveryEnable;
baseOverride.deepShadowDs5RefObsRecoveryStartSec = opts.deepShadowDs5RefObsRecoveryStartSec;
baseOverride.deepShadowDs5RefObsRecoveryMinSat = opts.deepShadowDs5RefObsRecoveryMinSat;
baseOverride.deepShadowDs5RefObsRecoveryMinUsedSat = opts.deepShadowDs5RefObsRecoveryMinUsedSat;
baseOverride.deepShadowDs5RefObsRecoveryCodeMedMaxChips = opts.deepShadowDs5RefObsRecoveryCodeMedMaxChips;
baseOverride.deepShadowDs5RefObsRecoveryCodeP95MaxChips = opts.deepShadowDs5RefObsRecoveryCodeP95MaxChips;
baseOverride.deepShadowDs5RefObsRecoveryFreqP95MaxHz = opts.deepShadowDs5RefObsRecoveryFreqP95MaxHz;
baseOverride.deepShadowDs5RefObsRecoveryRobustEnable = opts.deepShadowDs5RefObsRecoveryRobustEnable;
baseOverride.deepShadowDs5RefObsRecoveryRobustMaxDropSat = opts.deepShadowDs5RefObsRecoveryRobustMaxDropSat;
baseOverride.deepShadowDs5RefObsPositionUseRecoveryKeep = opts.deepShadowDs5RefObsPositionUseRecoveryKeep;
baseOverride.deepShadowDs5RefObsRecoveryRequireBranch = opts.deepShadowDs5RefObsRecoveryRequireBranch;
baseOverride.deepShadowDs5RefObsRecoveryBranchPostfitMaxM = opts.deepShadowDs5RefObsRecoveryBranchPostfitMaxM;
baseOverride.deepShadowDs5RefObsRecoveryBaselineDiffMaxM = opts.deepShadowDs5RefObsRecoveryBaselineDiffMaxM;
baseOverride.deepShadowDs5ObsContractEnable = opts.deepShadowDs5ObsContractEnable;
baseOverride.deepShadowDs5ObsContractMinSat = opts.deepShadowDs5ObsContractMinSat;
baseOverride.deepShadowDs5ObsContractSpreadP95MaxM = opts.deepShadowDs5ObsContractSpreadP95MaxM;
baseOverride.deepShadowDs5ObsContractRequireCommonClock = opts.deepShadowDs5ObsContractRequireCommonClock;
baseOverride.deepShadowDs5ObsContractRequireTrackingPass = opts.deepShadowDs5ObsContractRequireTrackingPass;
baseOverride.deepShadowDs5ObsContractMaxBaseSeedFrac = opts.deepShadowDs5ObsContractMaxBaseSeedFrac;
baseOverride.deepShadowDs5ObsContractNoBaselineSeedForRecovery = opts.deepShadowDs5ObsContractNoBaselineSeedForRecovery;
baseOverride.deepShadowDs5ObsContractUseRecoveryKeep = opts.deepShadowDs5ObsContractUseRecoveryKeep;
baseOverride.deepShadowDs5ObsContractRobustTrackingEnable = opts.deepShadowDs5ObsContractRobustTrackingEnable;
baseOverride.deepShadowDs5ObsContractRobustTrackingMaxDropSat = opts.deepShadowDs5ObsContractRobustTrackingMaxDropSat;
baseOverride.deepShadowDs5ObsContractIndependentCodeErrMaxChips = opts.deepShadowDs5ObsContractIndependentCodeErrMaxChips;
baseOverride.deepShadowDs5ObsContractIndependentFreqErrMaxHz = opts.deepShadowDs5ObsContractIndependentFreqErrMaxHz;
baseOverride.deepShadowDs5RefObsPositionCommonClockRebaseEnable = opts.deepShadowDs5RefObsPositionCommonClockRebaseEnable;
baseOverride.deepShadowDs5RefObsPositionCommonClockRebasePeriodM = opts.deepShadowDs5RefObsPositionCommonClockRebasePeriodM;
baseOverride.deepShadowDs5RefObsPositionRawAnchorLiftEnable = opts.deepShadowDs5RefObsPositionRawAnchorLiftEnable;
baseOverride.deepShadowDs5RefObsPositionRawAnchorLiftMaxSpreadP95M = opts.deepShadowDs5RefObsPositionRawAnchorLiftMaxSpreadP95M;
baseOverride.deepShadowDs5RefObsPositionRawAnchorLiftUseCodeCorr = opts.deepShadowDs5RefObsPositionRawAnchorLiftUseCodeCorr;
baseOverride.deepShadowDs5FinalForceRefObsRecovered = opts.deepShadowDs5FinalForceRefObsRecovered;
baseOverride.deepShadowDs5FinalQualitySelectEnable = opts.deepShadowDs5FinalQualitySelectEnable;
baseOverride.deepShadowDs5FinalQualityImproveMinM = opts.deepShadowDs5FinalQualityImproveMinM;
baseOverride.deepShadowDs5FinalQualityBaselineDiffMaxM = opts.deepShadowDs5FinalQualityBaselineDiffMaxM;
baseOverride.deepShadowDs5FinalRecoveredAuthorityRequireBaselineGate = opts.deepShadowDs5FinalRecoveredAuthorityRequireBaselineGate;
baseOverride.deepShadowDs5FinalRecoveredAuthorityMaxBaselineDiffM = opts.deepShadowDs5FinalRecoveredAuthorityMaxBaselineDiffM;
baseOverride.deepShadowDs5FinalAllowNoBaselineRecovered = opts.deepShadowDs5FinalAllowNoBaselineRecovered;
baseOverride.deepShadowDs5FinalRecoveredConfirmEpochs = opts.deepShadowDs5FinalRecoveredConfirmEpochs;
baseOverride.deepShadowDs5FinalContinuityGateEnable = opts.deepShadowDs5FinalContinuityGateEnable;
baseOverride.deepShadowDs5FinalContinuityMaxPredDiffM = opts.deepShadowDs5FinalContinuityMaxPredDiffM;
baseOverride.deepShadowDs5FinalContinuitySlewEnable = opts.deepShadowDs5FinalContinuitySlewEnable;
baseOverride.deepShadowDs5FinalContinuityMaxAgeEpochs = opts.deepShadowDs5FinalContinuityMaxAgeEpochs;
baseOverride.deepShadowDs5FinalHoldBaselineCompeteEnable = opts.deepShadowDs5FinalHoldBaselineCompeteEnable;
baseOverride.deepShadowDs5FinalHoldBaselineMinAgeEpochs = opts.deepShadowDs5FinalHoldBaselineMinAgeEpochs;
baseOverride.deepShadowDs5FinalHoldBaselineProxyMarginM = opts.deepShadowDs5FinalHoldBaselineProxyMarginM;
baseOverride.deepShadowDs5FinalHoldBaselineDiffMaxM = opts.deepShadowDs5FinalHoldBaselineDiffMaxM;
baseOverride.deepShadowDs5FinalHoldBaselineForceDiffM = opts.deepShadowDs5FinalHoldBaselineForceDiffM;
baseOverride.deepShadowDs5FinalHoldBaselineMaxAgeEpochs = opts.deepShadowDs5FinalHoldBaselineMaxAgeEpochs;
baseOverride.deepShadowDs5FinalRecoveredHoldAllowObsGap = opts.deepShadowDs5FinalRecoveredHoldAllowObsGap;
baseOverride.deepShadowDs5FinalRecoveredHoldAllowAuthorityGap = opts.deepShadowDs5FinalRecoveredHoldAllowAuthorityGap;
baseOverride.deepShadowDs5FinalRecoveredHoldSuppressBaselineCompete = opts.deepShadowDs5FinalRecoveredHoldSuppressBaselineCompete;
baseOverride.deepShadowDs5FinalRecoveredHoldBaselineDiffMaxM = opts.deepShadowDs5FinalRecoveredHoldBaselineDiffMaxM;
baseOverride.deepShadowDs5FinalRecoveredAuthorityEnable = opts.deepShadowDs5FinalRecoveredAuthorityEnable;
baseOverride.deepShadowDs5FinalBaselineOffAblation = opts.deepShadowDs5FinalBaselineOffAblation;
baseOverride.deepShadowDs5FinalRequireObsContractForRecovered = opts.deepShadowDs5FinalRequireObsContractForRecovered;
baseOverride.deepShadowDs5FinalRequireObsContractForHold = opts.deepShadowDs5FinalRequireObsContractForHold;
baseOverride.deepShadowDs5FinalAllowObsContractTrackingHold = opts.deepShadowDs5FinalAllowObsContractTrackingHold;
baseOverride.deepShadowDs5FinalAllowObsContractBaseSeedHold = opts.deepShadowDs5FinalAllowObsContractBaseSeedHold;
baseOverride.deepShadowDs5RecoveredFilterEnable = opts.deepShadowDs5RecoveredFilterEnable;
baseOverride.deepShadowDs5RecoveredFilterConfirmEpochs = opts.deepShadowDs5RecoveredFilterConfirmEpochs;
baseOverride.deepShadowDs5RecoveredFilterMaxBadEpochs = opts.deepShadowDs5RecoveredFilterMaxBadEpochs;
baseOverride.deepShadowDs5RecoveredFilterMaxCoastEpochs = opts.deepShadowDs5RecoveredFilterMaxCoastEpochs;
baseOverride.deepShadowDs5RecoveredFilterUpdateAlpha = opts.deepShadowDs5RecoveredFilterUpdateAlpha;
baseOverride.deepShadowDs5RecoveredFilterVelMeasBlend = opts.deepShadowDs5RecoveredFilterVelMeasBlend;
baseOverride.deepShadowDs5RecoveredFilterVelInsBlend = opts.deepShadowDs5RecoveredFilterVelInsBlend;
baseOverride.deepShadowDs5RecoveredFilterMeasMaxPredDiffM = opts.deepShadowDs5RecoveredFilterMeasMaxPredDiffM;
baseOverride.deepShadowDs5RecoveredFilterPropagateWithInsVel = opts.deepShadowDs5RecoveredFilterPropagateWithInsVel;
baseOverride.deepShadowDs5RecoveredFilterClockEnable = opts.deepShadowDs5RecoveredFilterClockEnable;
baseOverride.deepShadowDs5RecoveredFilterClockUpdateAlpha = opts.deepShadowDs5RecoveredFilterClockUpdateAlpha;
baseOverride.deepShadowDs5RecoveredFilterClockRateBlend = opts.deepShadowDs5RecoveredFilterClockRateBlend;
baseOverride.deepShadowDs5RecoveredFilterClockMeasMaxPredDiffM = opts.deepShadowDs5RecoveredFilterClockMeasMaxPredDiffM;
baseOverride.deepShadowDs5RecoveredFilterClockHardGateEnable = opts.deepShadowDs5RecoveredFilterClockHardGateEnable;
baseOverride.deepShadowDs5RecoveredFilterClockResetDiffMaxM = opts.deepShadowDs5RecoveredFilterClockResetDiffMaxM;
baseOverride.deepShadowDs5RecoveredFilterResetOnContractMeas = opts.deepShadowDs5RecoveredFilterResetOnContractMeas;
baseOverride.deepShadowDs5RecoveredFilterResetMeasPredDiffM = opts.deepShadowDs5RecoveredFilterResetMeasPredDiffM;
baseOverride.deepShadowDs5RecoveredFilterAbsAnchorGateRequire = opts.deepShadowDs5RecoveredFilterAbsAnchorGateRequire;
baseOverride.deepShadowDs5RecoveredFilterAuthorityRequireAbsAnchor = opts.deepShadowDs5RecoveredFilterAuthorityRequireAbsAnchor;
baseOverride.deepShadowDs5RecoveredFilterPropCapEnable = opts.deepShadowDs5RecoveredFilterPropCapEnable;
baseOverride.deepShadowDs5RecoveredFilterPropMaxStepM = opts.deepShadowDs5RecoveredFilterPropMaxStepM;
baseOverride.deepShadowDs5RecoveredFilterPropMaxTotalM = opts.deepShadowDs5RecoveredFilterPropMaxTotalM;
baseOverride.deepShadowDs5RecoveredFilterPredMaxClockStepM = opts.deepShadowDs5RecoveredFilterPredMaxClockStepM;
baseOverride.deepShadowDs5RecoveredFilterAuthorityMaxCoastEpochs = opts.deepShadowDs5RecoveredFilterAuthorityMaxCoastEpochs;
baseOverride.deepShadowDs5RecoveredFilterAuthorityBaselineDiffMaxM = opts.deepShadowDs5RecoveredFilterAuthorityBaselineDiffMaxM;
baseOverride.deepShadowDs5RecoveredFilterAuthorityUseBaselineDiffHardCap = opts.deepShadowDs5RecoveredFilterAuthorityUseBaselineDiffHardCap;
baseOverride.deepShadowDs5RecoveredFilterAuthorityMeasPredDiffMaxM = opts.deepShadowDs5RecoveredFilterAuthorityMeasPredDiffMaxM;
baseOverride.deepShadowDs5RecoveredFilterAuthorityAbsAnchorMaxM = opts.deepShadowDs5RecoveredFilterAuthorityAbsAnchorMaxM;
baseOverride.deepShadowDs5RecoveredFilterAbsAnchorEnable = opts.deepShadowDs5RecoveredFilterAbsAnchorEnable;
baseOverride.deepShadowDs5RecoveredFilterAbsAnchorSoftMaxDiffM = opts.deepShadowDs5RecoveredFilterAbsAnchorSoftMaxDiffM;
baseOverride.deepShadowDs5RecoveredFilterAbsAnchorMaxDiffM = opts.deepShadowDs5RecoveredFilterAbsAnchorMaxDiffM;
baseOverride.deepShadowDs5RecoveredFilterAbsAnchorBlend = opts.deepShadowDs5RecoveredFilterAbsAnchorBlend;
baseOverride.deepShadowDs5RecoveredFilterAbsAnchorBlendHigh = opts.deepShadowDs5RecoveredFilterAbsAnchorBlendHigh;
baseOverride.deepShadowDs5RecoveredFilterAbsAnchorAlphaMinScale = opts.deepShadowDs5RecoveredFilterAbsAnchorAlphaMinScale;
baseOverride.deepShadowDs5RecoveredFilterDopplerVelEnable = opts.deepShadowDs5RecoveredFilterDopplerVelEnable;
baseOverride.deepShadowDs5RecoveredFilterDopplerVelMinSat = opts.deepShadowDs5RecoveredFilterDopplerVelMinSat;
baseOverride.deepShadowDs5RecoveredFilterDopplerVelMaxResidMps = opts.deepShadowDs5RecoveredFilterDopplerVelMaxResidMps;
baseOverride.deepShadowDs5RecoveredFilterDopplerVelBlend = opts.deepShadowDs5RecoveredFilterDopplerVelBlend;
baseOverride.deepShadowDs5RecoveredFilterDopplerVelRobustMaxDropSat = opts.deepShadowDs5RecoveredFilterDopplerVelRobustMaxDropSat;
baseOverride.deepShadowDs5RefObsClosedLoopUseBaselineHold = opts.deepShadowDs5RefObsClosedLoopUseBaselineHold;
baseOverride.deepShadowDs5RefObsClosedLoopPreferBaselineHold = opts.deepShadowDs5RefObsClosedLoopPreferBaselineHold;
baseOverride.deepShadowDs5RefObsPositionEnable = opts.deepShadowDs5RefObsPositionEnable;
baseOverride.deepShadowDs5RefObsPositionMinSat = opts.deepShadowDs5RefObsPositionMinSat;
baseOverride.deepShadowDs5RefObsPositionPostfitMaxM = opts.deepShadowDs5RefObsPositionPostfitMaxM;
baseOverride.deepShadowDs5RefObsPositionAdaptivePostfitMaxM = opts.deepShadowDs5RefObsPositionAdaptivePostfitMaxM;
baseOverride.deepShadowDs5RefObsPositionPdopMax = opts.deepShadowDs5RefObsPositionPdopMax;
baseOverride.deepShadowDs5RefObsPositionCorrMaxM = opts.deepShadowDs5RefObsPositionCorrMaxM;
baseOverride.deepShadowDs5RefObsPositionAnchorDiffMaxM = opts.deepShadowDs5RefObsPositionAnchorDiffMaxM;
baseOverride.deepShadowDs5RefObsPositionJumpMaxM = opts.deepShadowDs5RefObsPositionJumpMaxM;
baseOverride.deepShadowDs5RefObsPositionAdaptiveJumpMaxM = opts.deepShadowDs5RefObsPositionAdaptiveJumpMaxM;
baseOverride.deepShadowDs5RefObsPositionJumpMaxAgeEpochs = opts.deepShadowDs5RefObsPositionJumpMaxAgeEpochs;
baseOverride.deepShadowDs5RefObsPositionSpeedGateEnable = opts.deepShadowDs5RefObsPositionSpeedGateEnable;
baseOverride.deepShadowDs5RefObsPositionSpeedMaxMps = opts.deepShadowDs5RefObsPositionSpeedMaxMps;
baseOverride.deepShadowDs5RefObsPositionAdaptiveSpeedMaxMps = opts.deepShadowDs5RefObsPositionAdaptiveSpeedMaxMps;
baseOverride.deepShadowDs5RefObsPositionAccelMaxMps2 = opts.deepShadowDs5RefObsPositionAccelMaxMps2;
baseOverride.deepShadowDs5RefObsPositionAdaptiveAccelMaxMps2 = opts.deepShadowDs5RefObsPositionAdaptiveAccelMaxMps2;
baseOverride.deepShadowDs5RefObsPositionDynGateMaxAgeEpochs = opts.deepShadowDs5RefObsPositionDynGateMaxAgeEpochs;
baseOverride.deepShadowDs5RefObsPositionRobustSubsetEnable = opts.deepShadowDs5RefObsPositionRobustSubsetEnable;
baseOverride.deepShadowDs5RefObsPositionRobustMaxDropSat = opts.deepShadowDs5RefObsPositionRobustMaxDropSat;
baseOverride.deepShadowDs5RefObsPositionRobustDropPenaltyM = opts.deepShadowDs5RefObsPositionRobustDropPenaltyM;
baseOverride.deepShadowDs5TruePeakMetricWeight = opts.deepShadowDs5TruePeakMetricWeight;
baseOverride.deepShadowDs5TruePeakFreqWeight = opts.deepShadowDs5TruePeakFreqWeight;
baseOverride.deepShadowDs5TruePeakCodeWeight = opts.deepShadowDs5TruePeakCodeWeight;
baseOverride.deepShadowDs5TruePeakResidualWeight = opts.deepShadowDs5TruePeakResidualWeight;
baseOverride.deepShadowDs5TruePeakCurrentMetricBias = opts.deepShadowDs5TruePeakCurrentMetricBias;
baseOverride.deepShadowDs5TruePeakCandidateMargin = opts.deepShadowDs5TruePeakCandidateMargin;
baseOverride.deepShadowDs5TruePeakProbationEpochs = opts.deepShadowDs5TruePeakProbationEpochs;
baseOverride.deepShadowDs5TruePeakProbationWindowEpochs = opts.deepShadowDs5TruePeakProbationWindowEpochs;
baseOverride.deepShadowDs5TruePeakProbationPassHits = opts.deepShadowDs5TruePeakProbationPassHits;
baseOverride.deepShadowDs5TruePeakTakeoverConfirmEpochs = opts.deepShadowDs5TruePeakTakeoverConfirmEpochs;
baseOverride.deepShadowDs5TruePeakRevokeBadEpochs = opts.deepShadowDs5TruePeakRevokeBadEpochs;
baseOverride.deepShadowDs5TruePeakRevokeFreqErrHz = opts.deepShadowDs5TruePeakRevokeFreqErrHz;
baseOverride.deepShadowDs5TruePeakRevokeCodeErrChips = opts.deepShadowDs5TruePeakRevokeCodeErrChips;
baseOverride.deepShadowDs5TruePeakRevokeImproveMaxM = opts.deepShadowDs5TruePeakRevokeImproveMaxM;
baseOverride.deepShadowDs5DetrendedGateEnable = opts.deepShadowDs5DetrendedGateEnable;
baseOverride.deepShadowDs5DetrendedGateStartSec = opts.deepShadowDs5DetrendedGateStartSec;
baseOverride.deepShadowDs5DetrendedGateMinSat = opts.deepShadowDs5DetrendedGateMinSat;
baseOverride.deepShadowDs5DetrendedGateFloorM = opts.deepShadowDs5DetrendedGateFloorM;
baseOverride.deepShadowDs5DetrendedGateMadScale = opts.deepShadowDs5DetrendedGateMadScale;
baseOverride.deepShadowDs5DetrendedGateCeilM = opts.deepShadowDs5DetrendedGateCeilM;
baseOverride.deepShadowDs5DetrendedGateProtectPrns = opts.deepShadowDs5DetrendedGateProtectPrns;
baseOverride.deepShadowBranchDs5DetrendedScoreWeight = opts.deepShadowBranchDs5DetrendedScoreWeight;
baseOverride.deepShadowBranchDs5DetrendedP95Weight = opts.deepShadowBranchDs5DetrendedP95Weight;
baseOverride.deepShadowBranchDs5DetrendedReadyP95MaxM = opts.deepShadowBranchDs5DetrendedReadyP95MaxM;
baseOverride.deepShadowDs5BranchTakeoverAllow4Sat = opts.deepShadowDs5BranchTakeoverAllow4Sat;
baseOverride.deepShadowDs5BranchTakeoverDetrendedP95MaxM = opts.deepShadowDs5BranchTakeoverDetrendedP95MaxM;
baseOverride.deepShadowDs5BranchContinueAllow4Sat = opts.deepShadowDs5BranchContinueAllow4Sat;
baseOverride.deepShadowDs5BranchContinueDetrendedP95MaxM = opts.deepShadowDs5BranchContinueDetrendedP95MaxM;
baseOverride.deepShadowRawAmbiguityEnable = opts.deepShadowRawAmbiguityEnable;
baseOverride.deepShadowRawAmbiguityMaxChips = opts.deepShadowRawAmbiguityMaxChips;
baseOverride.deepShadowRawAmbiguityStepMaxChips = opts.deepShadowRawAmbiguityStepMaxChips;
baseOverride.deepShadowRawAmbiguityMinSat = opts.deepShadowRawAmbiguityMinSat;
baseOverride.deepShadowBranchSearchEnable = opts.deepShadowBranchSearchEnable;
baseOverride.deepShadowBranchUseNavQ = opts.deepShadowBranchUseNavQ;
baseOverride.deepShadowBranchMinSat = opts.deepShadowBranchMinSat;
baseOverride.deepShadowBranchMaxChips = opts.deepShadowBranchMaxChips;
baseOverride.deepShadowBranchAmbigContinuityEnable = opts.deepShadowBranchAmbigContinuityEnable;
baseOverride.deepShadowBranchAmbigContinuityMaxAgeEpochs = opts.deepShadowBranchAmbigContinuityMaxAgeEpochs;
baseOverride.deepShadowBranchAmbigContinuityStepMaxChips = opts.deepShadowBranchAmbigContinuityStepMaxChips;
baseOverride.deepShadowBranchAmbigContinuityTolChips = opts.deepShadowBranchAmbigContinuityTolChips;
baseOverride.deepShadowBranchAmbigContinuityPenaltyM = opts.deepShadowBranchAmbigContinuityPenaltyM;
baseOverride.deepShadowBranchAmbigContinuityJumpPenaltyM = opts.deepShadowBranchAmbigContinuityJumpPenaltyM;
baseOverride.deepShadowBranchPriorRmsMaxM = opts.deepShadowBranchPriorRmsMaxM;
baseOverride.deepShadowBranchPostfitRmsMaxM = opts.deepShadowBranchPostfitRmsMaxM;
baseOverride.deepShadowBranchPdopMax = opts.deepShadowBranchPdopMax;
baseOverride.deepShadowBranchPosJumpMaxM = opts.deepShadowBranchPosJumpMaxM;
baseOverride.deepShadowDs5BaselineTrustedRefEnable = opts.deepShadowDs5BaselineTrustedRefEnable;
baseOverride.deepShadowDs5BaselineTrustedRefUseVelocity = opts.deepShadowDs5BaselineTrustedRefUseVelocity;
baseOverride.deepShadowDs5BaselineTrustedRefMaxSec = opts.deepShadowDs5BaselineTrustedRefMaxSec;
baseOverride.deepShadowDs5BranchAbsoluteConsistencyEnable = opts.deepShadowDs5BranchAbsoluteConsistencyEnable;
baseOverride.deepShadowDs5BranchAbsoluteConsistencyMaxDiffM = opts.deepShadowDs5BranchAbsoluteConsistencyMaxDiffM;
baseOverride.deepShadowDs5BranchLocalCorrectionMaxM = opts.deepShadowDs5BranchLocalCorrectionMaxM;
baseOverride.deepShadowDs5BranchLocalStepMaxM = opts.deepShadowDs5BranchLocalStepMaxM;
baseOverride.deepShadowDs5BranchLocalIterMax = opts.deepShadowDs5BranchLocalIterMax;
baseOverride.deepShadowDs5BranchLocalCorrPenaltyWeight = opts.deepShadowDs5BranchLocalCorrPenaltyWeight;
baseOverride.deepShadowDs5BranchObsGateEnable = opts.deepShadowDs5BranchObsGateEnable;
baseOverride.deepShadowDs5BranchObsGateMinSat = opts.deepShadowDs5BranchObsGateMinSat;
baseOverride.deepShadowDs5BranchObsGateFloorM = opts.deepShadowDs5BranchObsGateFloorM;
baseOverride.deepShadowDs5BranchObsGateMadScale = opts.deepShadowDs5BranchObsGateMadScale;
baseOverride.deepShadowDs5BranchObsGateCeilM = opts.deepShadowDs5BranchObsGateCeilM;
baseOverride.deepShadowTrustedAnchorEnable = opts.deepShadowTrustedAnchorEnable;
baseOverride.deepShadowTrustedAnchorUseTruthTrj = opts.deepShadowTrustedAnchorUseTruthTrj;
baseOverride.deepShadowTrustedAnchorTimeSec = opts.deepShadowTrustedAnchorTimeSec;
baseOverride.deepShadowTrustedAnchorUseVelocity = opts.deepShadowTrustedAnchorUseVelocity;
baseOverride.deepShadowTrustedAnchorAllowCurrentFallback = opts.deepShadowTrustedAnchorAllowCurrentFallback;
baseOverride.deepShadowBranchTakeoverConfirmEpochs = opts.deepShadowBranchTakeoverConfirmEpochs;
baseOverride.deepShadowBranchTakeoverPostfitRmsMaxM = opts.deepShadowBranchTakeoverPostfitRmsMaxM;
baseOverride.deepShadowBranchTakeoverPdopMax = opts.deepShadowBranchTakeoverPdopMax;
baseOverride.deepShadowBranchTakeoverMinSat = opts.deepShadowBranchTakeoverMinSat;
baseOverride.deepShadowBranchTakeoverRequireTrustedPrior = opts.deepShadowBranchTakeoverRequireTrustedPrior;
baseOverride.deepShadowBranchResetEnable = opts.deepShadowBranchResetEnable;
baseOverride.deepShadowBranchResetOnce = opts.deepShadowBranchResetOnce;
baseOverride.deepShadowBranchResetMaxJumpM = opts.deepShadowBranchResetMaxJumpM;
baseOverride.deepShadowBranchResetVelBlend = opts.deepShadowBranchResetVelBlend;
baseOverride.deepShadowBranchResetClock = opts.deepShadowBranchResetClock;
baseOverride.deepShadowBranchInjectEnable = opts.deepShadowBranchInjectEnable;
baseOverride.deepShadowBranchInjectRScale = opts.deepShadowBranchInjectRScale;
baseOverride.deepShadowBranchInjectMinSat = opts.deepShadowBranchInjectMinSat;
baseOverride.deepShadowBranchInjectResidualGateM = opts.deepShadowBranchInjectResidualGateM;
baseOverride.deepShadowRecoveryModeEnable = opts.deepShadowRecoveryModeEnable;
baseOverride.deepShadowRecoveryHoldEpochs = opts.deepShadowRecoveryHoldEpochs;
baseOverride.deepShadowRecoverySuppressMainGnss = opts.deepShadowRecoverySuppressMainGnss;
baseOverride.deepShadowRecoveryContinueMinSat = opts.deepShadowRecoveryContinueMinSat;
baseOverride.deepShadowRecoveryContinuePostfitRmsMaxM = opts.deepShadowRecoveryContinuePostfitRmsMaxM;
baseOverride.deepShadowRecoveryContinuePdopMax = opts.deepShadowRecoveryContinuePdopMax;
baseOverride.deepShadowRecoveryRequireTrustedPrior = opts.deepShadowRecoveryRequireTrustedPrior;
baseOverride.deepShadowRecoveryReanchorEnable = opts.deepShadowRecoveryReanchorEnable;
baseOverride.deepShadowRecoveryReanchorMinJumpM = opts.deepShadowRecoveryReanchorMinJumpM;
baseOverride.deepShadowRecoveryReanchorCooldownEpochs = opts.deepShadowRecoveryReanchorCooldownEpochs;
baseOverride.deepShadowRecoveryReanchorMaxCount = opts.deepShadowRecoveryReanchorMaxCount;
baseOverride.deepShadowClosedLoopUseBranchOutput = opts.deepShadowClosedLoopUseBranchOutput;
baseOverride.deepShadowDs5SoftRecoveryEnable = opts.deepShadowDs5SoftRecoveryEnable;
baseOverride.deepShadowDs5SoftRecoveryStartSec = opts.deepShadowDs5SoftRecoveryStartSec;
baseOverride.deepShadowDs5SoftRecoveryRequireBranchSane = opts.deepShadowDs5SoftRecoveryRequireBranchSane;
baseOverride.deepShadowDs5ClosedLiftPreferEnable = opts.deepShadowDs5ClosedLiftPreferEnable;
baseOverride.deepShadowDs5ClosedLiftPreferStartSec = opts.deepShadowDs5ClosedLiftPreferStartSec;
baseOverride.deepShadowDs5RawLiftRejectDevM = opts.deepShadowDs5RawLiftRejectDevM;
baseOverride.deepShadowDs5SoftClampEnable = opts.deepShadowDs5SoftClampEnable;
baseOverride.deepShadowDs5SoftClampStartSec = opts.deepShadowDs5SoftClampStartSec;
baseOverride.deepShadowDs5SoftClampAlpha = opts.deepShadowDs5SoftClampAlpha;
baseOverride.deepShadowDs5SoftClampVelBlend = opts.deepShadowDs5SoftClampVelBlend;
baseOverride.deepShadowClosedLoopCoastEpochs = opts.deepShadowClosedLoopCoastEpochs;
baseOverride.deepShadowClosedLoopUseVelocityCoast = opts.deepShadowClosedLoopUseVelocityCoast;
baseOverride.deepShadowClosedLoopAllowNavFallbackInRecovery = opts.deepShadowClosedLoopAllowNavFallbackInRecovery;
baseOverride.deepShadowClosedLoopUseTrustedFallback = opts.deepShadowClosedLoopUseTrustedFallback;
baseOverride.deepShadowClosedLoopTrustedBlendEnable = opts.deepShadowClosedLoopTrustedBlendEnable;
baseOverride.deepShadowClosedLoopTrustedBlendBranchAlpha = opts.deepShadowClosedLoopTrustedBlendBranchAlpha;
baseOverride.deepShadowClosedLoopTrustedBlendCoastAlpha = opts.deepShadowClosedLoopTrustedBlendCoastAlpha;
baseOverride.deepShadowClosedLoopSafeBranchGateEnable = opts.deepShadowClosedLoopSafeBranchGateEnable;
baseOverride.deepShadowClosedLoopSafeBranchMaxDiffM = opts.deepShadowClosedLoopSafeBranchMaxDiffM;
baseOverride.deepShadowClosedLoopSafeBranchPostfitMaxM = opts.deepShadowClosedLoopSafeBranchPostfitMaxM;
baseOverride.deepShadowClosedLoopSafeBranchPdopMax = opts.deepShadowClosedLoopSafeBranchPdopMax;
baseOverride.deepShadowClosedLoopSafeBranchMinSat = opts.deepShadowClosedLoopSafeBranchMinSat;
baseOverride.deepShadowClosedLoopSafeBranchAlpha = opts.deepShadowClosedLoopSafeBranchAlpha;
baseOverride.deepShadowCoreServoEnable = opts.deepShadowCoreServoEnable;
baseOverride.deepShadowCoreServoGain = opts.deepShadowCoreServoGain;
baseOverride.deepShadowCoreServoStepMaxChips = opts.deepShadowCoreServoStepMaxChips;
baseOverride.deepShadowCoreServoTotalMaxChips = opts.deepShadowCoreServoTotalMaxChips;
baseOverride.deepShadowCoreServoMinSat = opts.deepShadowCoreServoMinSat;
baseOverride.deepShadowCoreServoCoreDevMaxM = opts.deepShadowCoreServoCoreDevMaxM;
baseOverride.deepShadowCoreServoMinElevationDeg = opts.deepShadowCoreServoMinElevationDeg;
baseOverride.deepShadowMainSwitchEnable = opts.deepShadowMainSwitchEnable;
baseOverride.deepShadowMainSwitchImproveMinM = opts.deepShadowMainSwitchImproveMinM;
baseOverride.deepShadowMainSwitchConfirmEpochs = opts.deepShadowMainSwitchConfirmEpochs;
baseOverride.deepShadowMainSwitchHoldEpochs = opts.deepShadowMainSwitchHoldEpochs;
baseOverride.deepShadowMainSwitchWaitProtectDone = opts.deepShadowMainSwitchWaitProtectDone;
baseOverride.deepShadowReleaseUseTrackMetric = opts.deepShadowReleaseUseTrackMetric;
baseOverride.deepShadowReleaseNeedDetectorMetric = opts.deepShadowReleaseNeedDetectorMetric;
baseOverride.deepShadowReleaseUseNavResidual = opts.deepShadowReleaseUseNavResidual;
baseOverride.deepShadowReleaseNavMedM = opts.deepShadowReleaseNavMedM;
baseOverride.deepShadowReleaseNavP95M = opts.deepShadowReleaseNavP95M;
baseOverride.deepShadowReleaseNavMaxM = opts.deepShadowReleaseNavMaxM;
baseOverride.deepForceShadowNcoInSuspect = opts.deepForceShadowNcoInSuspect;
baseOverride.deepForceShadowNcoInSpoof = opts.deepForceShadowNcoInSpoof;
baseOverride.deepBypassPllInSuspect = opts.deepBypassPllInSuspect;
baseOverride.deepBypassPllInSpoof = opts.deepBypassPllInSpoof;
baseOverride.deepBypassDllInSuspect = opts.deepBypassDllInSuspect;
baseOverride.deepBypassDllInSpoof = opts.deepBypassDllInSpoof;
baseOverride.deepCodeErrorScaleSuspect = opts.deepCodeErrorScaleSuspect;
baseOverride.deepCodeErrorScaleSpoof = opts.deepCodeErrorScaleSpoof;
baseOverride.deepFreezeCodeErrorInSpoof = opts.deepFreezeCodeErrorInSpoof;
baseOverride.deepCodeNcoStepLimitHz = opts.deepCodeNcoStepLimitHz;
baseOverride.deepCodeReacqEnable = opts.deepCodeReacqEnable;
baseOverride.deepCodeReacqMinMode = opts.deepCodeReacqMinMode;
baseOverride.deepCodeReacqHalfChips = opts.deepCodeReacqHalfChips;
baseOverride.deepCodeReacqNarrowHalfChips = opts.deepCodeReacqNarrowHalfChips;
baseOverride.deepCodeReacqStepChips = opts.deepCodeReacqStepChips;
baseOverride.deepCodeReacqNarrowAfter = opts.deepCodeReacqNarrowAfter;
baseOverride.deepCodeReacqPeakRatio = opts.deepCodeReacqPeakRatio;
baseOverride.deepCodeReacqZeroRatio = opts.deepCodeReacqZeroRatio;
baseOverride.deepCodeReacqMaxShiftChips = opts.deepCodeReacqMaxShiftChips;
baseOverride.deepCodeReacqApplyGain = opts.deepCodeReacqApplyGain;
baseOverride.deepCodeReacqMinPromptMag = opts.deepCodeReacqMinPromptMag;
baseOverride.deepCodeReacqNavCorrEnable = opts.deepCodeReacqNavCorrEnable;
baseOverride.deepCodeReacqNavCorrGain = opts.deepCodeReacqNavCorrGain;
baseOverride.deepCodeReacqNavCorrStepChips = opts.deepCodeReacqNavCorrStepChips;
baseOverride.deepCodeReacqNavCorrLimitChips = opts.deepCodeReacqNavCorrLimitChips;
baseOverride.deepCodeReacqNavCorrDecay = opts.deepCodeReacqNavCorrDecay;
baseOverride.deepCodeWideReacqEnable = opts.deepCodeWideReacqEnable;
baseOverride.deepCodeWideReacqMinMode = opts.deepCodeWideReacqMinMode;
baseOverride.deepCodeWideReacqHalfChips = opts.deepCodeWideReacqHalfChips;
baseOverride.deepCodeWideReacqStepChips = opts.deepCodeWideReacqStepChips;
baseOverride.deepCodeWideReacqIntervalMs = opts.deepCodeWideReacqIntervalMs;
baseOverride.deepCodeWideReacqPeakRatio = opts.deepCodeWideReacqPeakRatio;
baseOverride.deepCodeWideReacqZeroRatio = opts.deepCodeWideReacqZeroRatio;
baseOverride.deepCodeWideReacqStableEpochs = opts.deepCodeWideReacqStableEpochs;
baseOverride.deepCodeWideReacqCandidateTolChips = opts.deepCodeWideReacqCandidateTolChips;
baseOverride.deepCodeWideReacqAccumMs = opts.deepCodeWideReacqAccumMs;
baseOverride.deepCodeWideReacqExcludeChips = opts.deepCodeWideReacqExcludeChips;
baseOverride.deepCodeWideReacqUseSecondPeak = opts.deepCodeWideReacqUseSecondPeak;
baseOverride.deepCodeWideReacqAccumRatio = opts.deepCodeWideReacqAccumRatio;
baseOverride.deepCodeWideReacqCorrGain = opts.deepCodeWideReacqCorrGain;
baseOverride.deepCodeWideReacqCorrStepChips = opts.deepCodeWideReacqCorrStepChips;
baseOverride.deepCodeWideReacqCorrLimitChips = opts.deepCodeWideReacqCorrLimitChips;
baseOverride.deepShadowAcqEnable = opts.deepShadowAcqEnable;
baseOverride.deepShadowAcqIntervalEpochs = opts.deepShadowAcqIntervalEpochs;
baseOverride.deepShadowAcqHalfChips = opts.deepShadowAcqHalfChips;
baseOverride.deepShadowAcqStepChips = opts.deepShadowAcqStepChips;
baseOverride.deepShadowAcqNoncohMs = opts.deepShadowAcqNoncohMs;
baseOverride.deepShadowAcqExcludeChips = opts.deepShadowAcqExcludeChips;
baseOverride.deepShadowAcqStableEpochs = opts.deepShadowAcqStableEpochs;
baseOverride.deepShadowAcqOffsetTolChips = opts.deepShadowAcqOffsetTolChips;
baseOverride.deepShadowAcqMinPeakRatio = opts.deepShadowAcqMinPeakRatio;
baseOverride.deepShadowAcqMinZeroRatio = opts.deepShadowAcqMinZeroRatio;
baseOverride.deepShadowAcqBoundaryGuardChips = opts.deepShadowAcqBoundaryGuardChips;
baseOverride.deepPrInnovationGateEnable = opts.deepPrInnovationGateEnable;
baseOverride.deepPrInnovationGateK = opts.deepPrInnovationGateK;
baseOverride.deepPrInnovationGateMinM = opts.deepPrInnovationGateMinM;
baseOverride.deepPrInnovationGateMaxM = opts.deepPrInnovationGateMaxM;
baseOverride.deepPrInnovationGateMinSat = opts.deepPrInnovationGateMinSat;
baseOverride.deepPrInnovationGateUseMedianDetrend = opts.deepPrInnovationGateUseMedianDetrend;
baseOverride.deepRecoveryHoldEpochs = opts.deepRecoveryHoldEpochs;
baseOverride.deepReentryStrictEpochs = opts.deepReentryStrictEpochs;
baseOverride.deepReentryTcmScale = opts.deepReentryTcmScale;
baseOverride.deepReentryTdfScale = opts.deepReentryTdfScale;
baseOverride.deepReentryTsatScale = opts.deepReentryTsatScale;
baseOverride.deepReentryTzScale = opts.deepReentryTzScale;
baseOverride.deepReentryConfirmEpochs = opts.deepReentryConfirmEpochs;
baseOverride.deepReentryHitMin = opts.deepReentryHitMin;
baseOverride.deepUseIndependentDetectTrack = opts.deepUseIndependentDetectTrack;
baseOverride.deepDetectUseTrackResults = opts.deepDetectUseTrackResults;
baseOverride.deepFastMode = opts.deepFastMode;
baseOverride.deepShadowDs5FastRefObsTrackBypass = opts.deepShadowDs5FastRefObsTrackBypass;
baseOverride.deepShadowDs5RefObsTrackRawStrideEpochs = opts.deepShadowDs5RefObsTrackRawStrideEpochs;
baseOverride.deepShadowDs5RefObsTrackRawBurstMs = opts.deepShadowDs5RefObsTrackRawBurstMs;
baseOverride.deepShadowDs5RefObsTrackRawWarmupEpochs = opts.deepShadowDs5RefObsTrackRawWarmupEpochs;
baseOverride.deepPerfLightNormalMode = opts.deepPerfLightNormalMode;
baseOverride.deepPerfDiagPrint = opts.deepPerfDiagPrint;
baseOverride.deepPerfDiagInterval = opts.deepPerfDiagInterval;
baseOverride.deepKeepTrackHistory = opts.deepKeepTrackHistory;
baseOverride.deepDetrendWindowEpoch = opts.deepDetrendWindowEpoch;
% Shadow recovery / KF overrides used by DeepCouple_perINStime.
baseOverride.deepShadowUseNavQualifiedForKf = opts.deepShadowUseNavQualifiedForKf;
baseOverride.deepShadowKfMinSat = opts.deepShadowKfMinSat;
baseOverride.deepShadowKfRScale = opts.deepShadowKfRScale;
baseOverride.deepShadowKfMedianDetrend = opts.deepShadowKfMedianDetrend;
baseOverride.deepShadowKfDetrendGateM = opts.deepShadowKfDetrendGateM;
% Branch-search robust subset tuning.
baseOverride.deepShadowBranchRobustSubsetEnable = opts.deepShadowBranchRobustSubsetEnable;
baseOverride.deepShadowBranchRobustSubsetMinSat = opts.deepShadowBranchRobustSubsetMinSat;
baseOverride.deepShadowBranchRobustSubsetMaxSat = opts.deepShadowBranchRobustSubsetMaxSat;
baseOverride.deepShadowBranchRobustSubsetMaxCand = opts.deepShadowBranchRobustSubsetMaxCand;
baseOverride.deepShadowBranchRobustSubsetDropPenaltyM = opts.deepShadowBranchRobustSubsetDropPenaltyM;
baseOverride.deepShadowBranchRobustSubsetFourSatPenaltyM = opts.deepShadowBranchRobustSubsetFourSatPenaltyM;
baseOverride.deepShadowBranchRobustDopplerWeight = opts.deepShadowBranchRobustDopplerWeight;
% DS5 authority / source-gate controls.
baseOverride.deepShadowDs5KfSourceGateEnable = opts.deepShadowDs5KfSourceGateEnable;
baseOverride.deepShadowDs5KfSourceGateStartSec = opts.deepShadowDs5KfSourceGateStartSec;
baseOverride.deepShadowDs5KfRequireCommonDrag = opts.deepShadowDs5KfRequireCommonDrag;
baseOverride.deepShadowDs5KfTrustedMinSat = opts.deepShadowDs5KfTrustedMinSat;
baseOverride.deepShadowDs5KfMode2MinSat = opts.deepShadowDs5KfMode2MinSat;
baseOverride.deepShadowDs5KfAllowMode3Prns = opts.deepShadowDs5KfAllowMode3Prns;
baseOverride.deepShadowDs5KfAllowMode3MaxDetM = opts.deepShadowDs5KfAllowMode3MaxDetM;
baseOverride.deepShadowDs5KfAllowMode3OnlyWhenBranchSane = opts.deepShadowDs5KfAllowMode3OnlyWhenBranchSane;
baseOverride.deepShadowDs5AuthorityEnable = opts.deepShadowDs5AuthorityEnable;
baseOverride.deepShadowDs5AuthorityStartSec = opts.deepShadowDs5AuthorityStartSec;
baseOverride.deepShadowDs5AuthorityConfirmEpochs = opts.deepShadowDs5AuthorityConfirmEpochs;
baseOverride.deepShadowDs5AuthorityHoldEpochs = opts.deepShadowDs5AuthorityHoldEpochs;
baseOverride.deepShadowDs5AuthorityRequireBranchSane = opts.deepShadowDs5AuthorityRequireBranchSane;
baseOverride.deepShadowDs5AuthorityRequireClosedLoopRef = opts.deepShadowDs5AuthorityRequireClosedLoopRef;
baseOverride.deepShadowDs5AuthorityRefPosMaxDiffM = opts.deepShadowDs5AuthorityRefPosMaxDiffM;
baseOverride.deepShadowDs5AuthorityHardRevokeEnable = opts.deepShadowDs5AuthorityHardRevokeEnable;
baseOverride.deepShadowDs5AuthorityHardRevokeRefDiffM = opts.deepShadowDs5AuthorityHardRevokeRefDiffM;
baseOverride.deepShadowDs5AuthorityFastDeauthBadEpochs = opts.deepShadowDs5AuthorityFastDeauthBadEpochs;
baseOverride.deepShadowDs5AuthorityTrustedMinSat = opts.deepShadowDs5AuthorityTrustedMinSat;
baseOverride.deepShadowDs5AuthorityMode2MinSat = opts.deepShadowDs5AuthorityMode2MinSat;
baseOverride.deepShadowDs5AuthorityMode23MinSat = opts.deepShadowDs5AuthorityMode23MinSat;
baseOverride.deepShadowDs5ObserveOnlyFollowEnable = opts.deepShadowDs5ObserveOnlyFollowEnable;
baseOverride.deepShadowDs5ObsContractEnable = opts.deepShadowDs5ObsContractEnable;
baseOverride.deepShadowDs5ObsContractMinSat = opts.deepShadowDs5ObsContractMinSat;
baseOverride.deepShadowDs5ObsContractSpreadP95MaxM = opts.deepShadowDs5ObsContractSpreadP95MaxM;
baseOverride.deepShadowDs5ObsContractRequireCommonClock = opts.deepShadowDs5ObsContractRequireCommonClock;
baseOverride.deepShadowDs5ObsContractRequireTrackingPass = opts.deepShadowDs5ObsContractRequireTrackingPass;
baseOverride.deepShadowDs5ObsContractMaxBaseSeedFrac = opts.deepShadowDs5ObsContractMaxBaseSeedFrac;
baseOverride.deepShadowDs5ObsContractNoBaselineSeedForRecovery = opts.deepShadowDs5ObsContractNoBaselineSeedForRecovery;
baseOverride.deepShadowDs5ObsContractUseRecoveryKeep = opts.deepShadowDs5ObsContractUseRecoveryKeep;
baseOverride.deepShadowDs5ObsContractRobustTrackingEnable = opts.deepShadowDs5ObsContractRobustTrackingEnable;
baseOverride.deepShadowDs5ObsContractRobustTrackingMaxDropSat = opts.deepShadowDs5ObsContractRobustTrackingMaxDropSat;
baseOverride.deepShadowDs5ObsContractIndependentCodeErrMaxChips = opts.deepShadowDs5ObsContractIndependentCodeErrMaxChips;
baseOverride.deepShadowDs5ObsContractIndependentFreqErrMaxHz = opts.deepShadowDs5ObsContractIndependentFreqErrMaxHz;
baseOverride.deepShadowDs5RefObsPositionCommonClockRebaseEnable = opts.deepShadowDs5RefObsPositionCommonClockRebaseEnable;
baseOverride.deepShadowDs5RefObsPositionCommonClockRebasePeriodM = opts.deepShadowDs5RefObsPositionCommonClockRebasePeriodM;
baseOverride.deepShadowDs5RefObsPositionRawAnchorLiftEnable = opts.deepShadowDs5RefObsPositionRawAnchorLiftEnable;
baseOverride.deepShadowDs5RefObsPositionRawAnchorLiftMaxSpreadP95M = opts.deepShadowDs5RefObsPositionRawAnchorLiftMaxSpreadP95M;
baseOverride.deepShadowDs5RefObsPositionRawAnchorLiftUseCodeCorr = opts.deepShadowDs5RefObsPositionRawAnchorLiftUseCodeCorr;
baseOverride.deepShadowDs5FinalOutputEnable = opts.deepShadowDs5FinalOutputEnable;
baseOverride.deepShadowDs5FinalOutputHoldEpochs = opts.deepShadowDs5FinalOutputHoldEpochs;
baseOverride.deepShadowDs5FinalForceRefObsRecovered = opts.deepShadowDs5FinalForceRefObsRecovered;
baseOverride.deepShadowDs5FinalQualitySelectEnable = opts.deepShadowDs5FinalQualitySelectEnable;
baseOverride.deepShadowDs5FinalQualityImproveMinM = opts.deepShadowDs5FinalQualityImproveMinM;
baseOverride.deepShadowDs5FinalQualityBaselineDiffMaxM = opts.deepShadowDs5FinalQualityBaselineDiffMaxM;
baseOverride.deepShadowDs5FinalRecoveredAuthorityRequireBaselineGate = opts.deepShadowDs5FinalRecoveredAuthorityRequireBaselineGate;
baseOverride.deepShadowDs5FinalRecoveredAuthorityMaxBaselineDiffM = opts.deepShadowDs5FinalRecoveredAuthorityMaxBaselineDiffM;
baseOverride.deepShadowDs5FinalAllowNoBaselineRecovered = opts.deepShadowDs5FinalAllowNoBaselineRecovered;
baseOverride.deepShadowDs5FinalRecoveredConfirmEpochs = opts.deepShadowDs5FinalRecoveredConfirmEpochs;
baseOverride.deepShadowDs5FinalContinuityGateEnable = opts.deepShadowDs5FinalContinuityGateEnable;
baseOverride.deepShadowDs5FinalContinuityMaxPredDiffM = opts.deepShadowDs5FinalContinuityMaxPredDiffM;
baseOverride.deepShadowDs5FinalContinuitySlewEnable = opts.deepShadowDs5FinalContinuitySlewEnable;
baseOverride.deepShadowDs5FinalContinuityMaxAgeEpochs = opts.deepShadowDs5FinalContinuityMaxAgeEpochs;
baseOverride.deepShadowDs5FinalHoldBaselineCompeteEnable = opts.deepShadowDs5FinalHoldBaselineCompeteEnable;
baseOverride.deepShadowDs5FinalHoldBaselineMinAgeEpochs = opts.deepShadowDs5FinalHoldBaselineMinAgeEpochs;
baseOverride.deepShadowDs5FinalHoldBaselineProxyMarginM = opts.deepShadowDs5FinalHoldBaselineProxyMarginM;
baseOverride.deepShadowDs5FinalHoldBaselineDiffMaxM = opts.deepShadowDs5FinalHoldBaselineDiffMaxM;
baseOverride.deepShadowDs5FinalHoldBaselineForceDiffM = opts.deepShadowDs5FinalHoldBaselineForceDiffM;
baseOverride.deepShadowDs5FinalHoldBaselineMaxAgeEpochs = opts.deepShadowDs5FinalHoldBaselineMaxAgeEpochs;
baseOverride.deepShadowDs5FinalRecoveredHoldAllowObsGap = opts.deepShadowDs5FinalRecoveredHoldAllowObsGap;
baseOverride.deepShadowDs5FinalRecoveredHoldAllowAuthorityGap = opts.deepShadowDs5FinalRecoveredHoldAllowAuthorityGap;
baseOverride.deepShadowDs5FinalRecoveredHoldSuppressBaselineCompete = opts.deepShadowDs5FinalRecoveredHoldSuppressBaselineCompete;
baseOverride.deepShadowDs5FinalRecoveredHoldBaselineDiffMaxM = opts.deepShadowDs5FinalRecoveredHoldBaselineDiffMaxM;
baseOverride.deepShadowDs5FinalRecoveredAuthorityEnable = opts.deepShadowDs5FinalRecoveredAuthorityEnable;
baseOverride.deepShadowDs5FinalBaselineOffAblation = opts.deepShadowDs5FinalBaselineOffAblation;
baseOverride.deepShadowDs5FinalRequireObsContractForRecovered = opts.deepShadowDs5FinalRequireObsContractForRecovered;
baseOverride.deepShadowDs5FinalRequireObsContractForHold = opts.deepShadowDs5FinalRequireObsContractForHold;
baseOverride.deepShadowDs5FinalAllowObsContractTrackingHold = opts.deepShadowDs5FinalAllowObsContractTrackingHold;
baseOverride.deepShadowDs5FinalAllowObsContractBaseSeedHold = opts.deepShadowDs5FinalAllowObsContractBaseSeedHold;
baseOverride.deepShadowDs5RecoveredFilterEnable = opts.deepShadowDs5RecoveredFilterEnable;
baseOverride.deepShadowDs5RecoveredFilterConfirmEpochs = opts.deepShadowDs5RecoveredFilterConfirmEpochs;
baseOverride.deepShadowDs5RecoveredFilterMaxBadEpochs = opts.deepShadowDs5RecoveredFilterMaxBadEpochs;
baseOverride.deepShadowDs5RecoveredFilterMaxCoastEpochs = opts.deepShadowDs5RecoveredFilterMaxCoastEpochs;
baseOverride.deepShadowDs5RecoveredFilterUpdateAlpha = opts.deepShadowDs5RecoveredFilterUpdateAlpha;
baseOverride.deepShadowDs5RecoveredFilterVelMeasBlend = opts.deepShadowDs5RecoveredFilterVelMeasBlend;
baseOverride.deepShadowDs5RecoveredFilterVelInsBlend = opts.deepShadowDs5RecoveredFilterVelInsBlend;
baseOverride.deepShadowDs5RecoveredFilterMeasMaxPredDiffM = opts.deepShadowDs5RecoveredFilterMeasMaxPredDiffM;
baseOverride.deepShadowDs5RecoveredFilterPropagateWithInsVel = opts.deepShadowDs5RecoveredFilterPropagateWithInsVel;
baseOverride.deepShadowDs5RecoveredFilterClockEnable = opts.deepShadowDs5RecoveredFilterClockEnable;
baseOverride.deepShadowDs5RecoveredFilterClockUpdateAlpha = opts.deepShadowDs5RecoveredFilterClockUpdateAlpha;
baseOverride.deepShadowDs5RecoveredFilterClockRateBlend = opts.deepShadowDs5RecoveredFilterClockRateBlend;
baseOverride.deepShadowDs5RecoveredFilterClockMeasMaxPredDiffM = opts.deepShadowDs5RecoveredFilterClockMeasMaxPredDiffM;
baseOverride.deepShadowDs5RecoveredFilterClockHardGateEnable = opts.deepShadowDs5RecoveredFilterClockHardGateEnable;
baseOverride.deepShadowDs5RecoveredFilterClockResetDiffMaxM = opts.deepShadowDs5RecoveredFilterClockResetDiffMaxM;
baseOverride.deepShadowDs5RecoveredFilterResetOnContractMeas = opts.deepShadowDs5RecoveredFilterResetOnContractMeas;
baseOverride.deepShadowDs5RecoveredFilterResetMeasPredDiffM = opts.deepShadowDs5RecoveredFilterResetMeasPredDiffM;
baseOverride.deepShadowDs5RecoveredFilterAbsAnchorGateRequire = opts.deepShadowDs5RecoveredFilterAbsAnchorGateRequire;
baseOverride.deepShadowDs5RecoveredFilterAuthorityRequireAbsAnchor = opts.deepShadowDs5RecoveredFilterAuthorityRequireAbsAnchor;
baseOverride.deepShadowDs5RecoveredFilterPropCapEnable = opts.deepShadowDs5RecoveredFilterPropCapEnable;
baseOverride.deepShadowDs5RecoveredFilterPropMaxStepM = opts.deepShadowDs5RecoveredFilterPropMaxStepM;
baseOverride.deepShadowDs5RecoveredFilterPropMaxTotalM = opts.deepShadowDs5RecoveredFilterPropMaxTotalM;
baseOverride.deepShadowDs5RecoveredFilterPredMaxClockStepM = opts.deepShadowDs5RecoveredFilterPredMaxClockStepM;
baseOverride.deepShadowDs5RecoveredFilterAuthorityMaxCoastEpochs = opts.deepShadowDs5RecoveredFilterAuthorityMaxCoastEpochs;
baseOverride.deepShadowDs5RecoveredFilterAuthorityBaselineDiffMaxM = opts.deepShadowDs5RecoveredFilterAuthorityBaselineDiffMaxM;
baseOverride.deepShadowDs5RecoveredFilterAuthorityUseBaselineDiffHardCap = opts.deepShadowDs5RecoveredFilterAuthorityUseBaselineDiffHardCap;
baseOverride.deepShadowDs5RecoveredFilterAuthorityMeasPredDiffMaxM = opts.deepShadowDs5RecoveredFilterAuthorityMeasPredDiffMaxM;
baseOverride.deepShadowDs5RecoveredFilterAuthorityAbsAnchorMaxM = opts.deepShadowDs5RecoveredFilterAuthorityAbsAnchorMaxM;
baseOverride.deepShadowDs5RecoveredFilterAbsAnchorEnable = opts.deepShadowDs5RecoveredFilterAbsAnchorEnable;
baseOverride.deepShadowDs5RecoveredFilterAbsAnchorSoftMaxDiffM = opts.deepShadowDs5RecoveredFilterAbsAnchorSoftMaxDiffM;
baseOverride.deepShadowDs5RecoveredFilterAbsAnchorMaxDiffM = opts.deepShadowDs5RecoveredFilterAbsAnchorMaxDiffM;
baseOverride.deepShadowDs5RecoveredFilterAbsAnchorBlend = opts.deepShadowDs5RecoveredFilterAbsAnchorBlend;
baseOverride.deepShadowDs5RecoveredFilterAbsAnchorBlendHigh = opts.deepShadowDs5RecoveredFilterAbsAnchorBlendHigh;
baseOverride.deepShadowDs5RecoveredFilterAbsAnchorAlphaMinScale = opts.deepShadowDs5RecoveredFilterAbsAnchorAlphaMinScale;
baseOverride.deepShadowDs5RecoveredFilterDopplerVelEnable = opts.deepShadowDs5RecoveredFilterDopplerVelEnable;
baseOverride.deepShadowDs5RecoveredFilterDopplerVelMinSat = opts.deepShadowDs5RecoveredFilterDopplerVelMinSat;
baseOverride.deepShadowDs5RecoveredFilterDopplerVelMaxResidMps = opts.deepShadowDs5RecoveredFilterDopplerVelMaxResidMps;
baseOverride.deepShadowDs5RecoveredFilterDopplerVelBlend = opts.deepShadowDs5RecoveredFilterDopplerVelBlend;
baseOverride.deepShadowDs5RecoveredFilterDopplerVelRobustMaxDropSat = opts.deepShadowDs5RecoveredFilterDopplerVelRobustMaxDropSat;
baseOverride.deepShadowDs5FinalRecoveryBaselineGateEnable = opts.deepShadowDs5FinalRecoveryBaselineGateEnable;
baseOverride.deepShadowDs5FinalRecoveryBaselineMaxDiffM = opts.deepShadowDs5FinalRecoveryBaselineMaxDiffM;
baseOverride.deepShadowDs5FinalRecoveryImproveGateEnable = opts.deepShadowDs5FinalRecoveryImproveGateEnable;
baseOverride.deepShadowDs5FinalRecoveryImproveMinM = opts.deepShadowDs5FinalRecoveryImproveMinM;
baseOverride.deepShadowDs5RefObsClosedLoopUseBaselineHold = opts.deepShadowDs5RefObsClosedLoopUseBaselineHold;
baseOverride.deepShadowDs5RefObsClosedLoopPreferBaselineHold = opts.deepShadowDs5RefObsClosedLoopPreferBaselineHold;
baseOverride.deepShadowDs5RefObsPositionEnable = opts.deepShadowDs5RefObsPositionEnable;
baseOverride.deepShadowDs5RefObsPositionMinSat = opts.deepShadowDs5RefObsPositionMinSat;
baseOverride.deepShadowDs5RefObsPositionPostfitMaxM = opts.deepShadowDs5RefObsPositionPostfitMaxM;
baseOverride.deepShadowDs5RefObsPositionAdaptivePostfitMaxM = opts.deepShadowDs5RefObsPositionAdaptivePostfitMaxM;
baseOverride.deepShadowDs5RefObsPositionPdopMax = opts.deepShadowDs5RefObsPositionPdopMax;
baseOverride.deepShadowDs5RefObsPositionCorrMaxM = opts.deepShadowDs5RefObsPositionCorrMaxM;
baseOverride.deepShadowDs5RefObsPositionAnchorDiffMaxM = opts.deepShadowDs5RefObsPositionAnchorDiffMaxM;
baseOverride.deepShadowDs5RefObsPositionJumpMaxM = opts.deepShadowDs5RefObsPositionJumpMaxM;
baseOverride.deepShadowDs5RefObsPositionAdaptiveJumpMaxM = opts.deepShadowDs5RefObsPositionAdaptiveJumpMaxM;
baseOverride.deepShadowDs5RefObsPositionJumpMaxAgeEpochs = opts.deepShadowDs5RefObsPositionJumpMaxAgeEpochs;
baseOverride.deepShadowDs5RefObsPositionSpeedGateEnable = opts.deepShadowDs5RefObsPositionSpeedGateEnable;
baseOverride.deepShadowDs5RefObsPositionSpeedMaxMps = opts.deepShadowDs5RefObsPositionSpeedMaxMps;
baseOverride.deepShadowDs5RefObsPositionAdaptiveSpeedMaxMps = opts.deepShadowDs5RefObsPositionAdaptiveSpeedMaxMps;
baseOverride.deepShadowDs5RefObsPositionAccelMaxMps2 = opts.deepShadowDs5RefObsPositionAccelMaxMps2;
baseOverride.deepShadowDs5RefObsPositionAdaptiveAccelMaxMps2 = opts.deepShadowDs5RefObsPositionAdaptiveAccelMaxMps2;
baseOverride.deepShadowDs5RefObsPositionDynGateMaxAgeEpochs = opts.deepShadowDs5RefObsPositionDynGateMaxAgeEpochs;
baseOverride.deepShadowDs5RefObsPositionRobustSubsetEnable = opts.deepShadowDs5RefObsPositionRobustSubsetEnable;
baseOverride.deepShadowDs5RefObsPositionRobustMaxDropSat = opts.deepShadowDs5RefObsPositionRobustMaxDropSat;
baseOverride.deepShadowDs5RefObsPositionRobustDropPenaltyM = opts.deepShadowDs5RefObsPositionRobustDropPenaltyM;
baseOverride.deepUseReferenceResidual = 0;
baseOverride.deepRefResidualHz = [];
baseOverride.deepPllNoiseBandwidthNormal = 12;
baseOverride.deepPllNoiseBandwidthSuspect = 6;
baseOverride.deepPllNoiseBandwidthSpoof = 3;
baseOverride.deepCarrNcoStepLimitHz = 120;
baseOverride.deepClkDriftHzLimit = 120;
baseOverride.deepCarrErrorScaleSuspect = 0.5;
baseOverride.deepCarrErrorScaleSpoof = 0.15;
baseOverride.deepAlignCarrNcoOnSpoof = 1;
baseOverride.deepResetCarrErrorOnSpoof = 1;
baseOverride.deepFreezeCarrErrorInSpoof = 0;
baseOverride.deepBiasByPrnHz = zeros(1, 64);

% 1) Calibration: run clean pass OR reuse existing calibration file.
biasByPrn = zeros(1, 64);
Tcm = nan; Tdf = nan; Tsat = nan; Tz = nan;
refResidualHz = [];
if opts.reuseCalib
    [biasByPrn, Tcm, Tdf, Tsat, Tz, refResidualHz] = ...
        loadCalibrationFromFile(opts.calibFile, opts.deepUseReferenceResidual);
    fprintf('\n[GOAL2] Reuse calibration from: %s\n', opts.calibFile);
else
    calibOverride = baseOverride;
    calibOverride.deepSpoofDetectEnable = 0;
    fprintf('\n[GOAL2] Calibration run: CLEAN\n');
    cleanCal = runOneDataset('clean', matMap, rawMap, calibOverride, opts.outPrefix, true);

    [biasByPrn, Tcm, Tdf, Tsat, Tz] = buildCalibration(cleanCal.navResults, opts);
    if opts.deepUseReferenceResidual
        refResidualHz = cleanCal.navResults.residualHz;
        calibRefOverride = baseOverride;
        calibRefOverride.deepSpoofDetectEnable = 0;
        calibRefOverride.deepBiasByPrnHz = biasByPrn;
        calibRefOverride.deepUseReferenceResidual = 1;
        calibRefOverride.deepRefResidualHz = refResidualHz;
        fprintf('\n[GOAL2] Calibration run: CLEAN (reference-subtracted)\n');
        cleanCalRef = runOneDataset('clean', matMap, rawMap, calibRefOverride, ...
            sprintf('%s_ref', opts.outPrefix), true);
        [~, Tcm, Tdf, Tsat, Tz] = buildCalibration(cleanCalRef.navResults, opts);
    end

    if ~isempty(opts.saveCalibFile)
        saveCalibrationToFile(opts.saveCalibFile, biasByPrn, Tcm, Tdf, Tsat, Tz, ...
            refResidualHz, opts, opts.outPrefix);
        fprintf('[GOAL2] Calibration package saved: %s\n', opts.saveCalibFile);
    end
end
fprintf('[GOAL2] Calibrated thresholds: Tcm=%.3f Hz, Tdf=%.3f Hz, Tsat=%.3f Hz, Tz=%.3f\n', ...
    Tcm, Tdf, Tsat, Tz);
fprintf('[GOAL2] Calibration window: epoch %d -> %d\n', opts.calibStartEpoch, opts.calibEndEpoch);

% 2) Final runs with calibrated thresholds.
results = struct([]);
for ii = 1:numel(opts.datasets)
    tag = lower(string(opts.datasets{ii}));
    ov = baseOverride;
    ov.deepBiasByPrnHz = biasByPrn;
    ov.deepTcmHz = Tcm;
    ov.deepTdfHz = Tdf;
    ov.deepTsatHz = Tsat;
    ov.deepTz = Tz;
    ov.deepSpoofDetectEnable = 1;
    ov.deepDetectArmEpoch = max(1, opts.calibEndEpoch + 1);
    ov.deepShadowReleaseTcmHz = max(opts.deepShadowReleaseTcmMin, opts.deepShadowReleaseTcmScale * Tcm);
    ov.deepShadowReleaseTdfHz = max(opts.deepShadowReleaseTdfMin, opts.deepShadowReleaseTdfScale * Tdf);
    ov.deepShadowReleaseTz = max(opts.deepShadowReleaseTzMin, opts.deepShadowReleaseTzScale * Tz);
    ov.deepShadowReleaseTrackCmHz = max(opts.deepShadowReleaseTrackCmMin, opts.deepShadowReleaseTrackCmScale * Tcm);
    ov.deepShadowReleaseTrackDfHz = max(opts.deepShadowReleaseTrackDfMin, opts.deepShadowReleaseTrackDfScale * Tdf);
    ov.deepUseReferenceResidual = opts.deepUseReferenceResidual;
    if opts.deepUseReferenceResidual
        ov.deepRefResidualHz = refResidualHz;
    else
        ov.deepRefResidualHz = [];
    end

    fprintf('\n[GOAL2] Final run: %s\n', upper(char(tag)));
    try
        runRes = runOneDataset(char(tag), matMap, rawMap, ov, opts.outPrefix, false);
        results = [results; rmfield(runRes, {'navResults', 'settingsOverride'})]; %#ok<AGROW>
    catch ME
        missingInput = contains(ME.message, 'Missing raw IF file') || ...
            contains(ME.message, 'Missing MAT dataset') || ...
            contains(ME.message, 'Unsupported dataset tag');
        if opts.skipMissingRaw && missingInput
            warning('[GOAL2] Skip %s: %s', upper(char(tag)), ME.message);
        else
            fprintf(2, '[GOAL2] %s failed: %s\n', upper(char(tag)), ME.message);
            for ss = 1:numel(ME.stack)
                fprintf(2, '  at %s:%d\n', ME.stack(ss).name, ME.stack(ss).line);
            end
            rethrow(ME);
        end
    end
end

% Save batch summary.
save(sprintf('%s_batch_summary.mat', opts.outPrefix), 'results', 'opts', ...
    'biasByPrn', 'Tcm', 'Tdf', 'Tsat', 'Tz', 'refResidualHz', '-v7.3');
fprintf('\n[GOAL2] Batch summary saved: %s_batch_summary.mat\n', opts.outPrefix);
end

function out = runOneDataset(tag, matMap, rawMap, settingsOverride, outPrefix, isCalib)
tag = lower(string(tag));
if ~isfield(matMap, tag)
    error('Unsupported dataset tag: %s', tag);
end

matFile = matMap.(tag);
rawFile = rawMap.(tag);
if exist(matFile, 'file') ~= 2
    error('Missing MAT dataset: %s', matFile);
end

S = load(matFile, 'trackResults', 'channel', 'TOW', 'eph', 'subFrameStart', 'settings', 'navSolutions');
trackResults = S.trackResults; %#ok<NASGU>
channel = S.channel; %#ok<NASGU>
TOW = S.TOW; %#ok<NASGU>
eph = S.eph; %#ok<NASGU>
subFrameStart = S.subFrameStart; %#ok<NASGU>
navSolutions = S.navSolutions; %#ok<NASGU>

rawCandidates = {};
if isfield(rawMap, tag)
    rawCandidates{end+1} = rawMap.(tag); %#ok<AGROW>
end
if isfield(S, 'settings') && isstruct(S.settings) && isfield(S.settings, 'fileName')
    rawCandidates{end+1} = S.settings.fileName; %#ok<AGROW>
end
if strcmp(tag, 'ds6')
    rawCandidates{end+1} = 'E:\ds6.bin'; %#ok<AGROW>
    userProf = getenv('USERPROFILE');
    if ~isempty(userProf)
        rawCandidates{end+1} = fullfile(userProf, 'Downloads', 'ds6.bin'); %#ok<AGROW>
    end
end
rawFile = pickFirstExisting(rawCandidates);
if isempty(rawFile)
    error('Missing raw IF file for %s. Tried: %s', tag, strjoin(rawCandidates, ', '));
end
settingsOverride.fileName = rawFile;
run('deepIntegration/DeepCouple_perINStime.m');  %#ok<RUN>

navSolPeriodSec = 0.5;
if isfield(settingsOverride, 'navSolPeriod')
    navSolPeriodSec = settingsOverride.navSolPeriod / 1000;
end

firstSuspect = find(navResults.spoofState >= 1, 1, 'first');
firstSpoof = find(navResults.spoofState == 2, 1, 'first');

out = struct();
out.dataset = char(tag);
out.epochs = numel(navResults.X);
out.stateMax = max(navResults.spoofState);
out.firstSuspectEpoch = firstSuspect;
out.firstSpoofEpoch = firstSpoof;
out.firstSuspectSec = epochToSec(firstSuspect, navSolPeriodSec);
out.firstSpoofSec = epochToSec(firstSpoof, navSolPeriodSec);
out.metricCmMedian = median(navResults.metricCmHz(isfinite(navResults.metricCmHz)));
out.metricDfMedian = median(navResults.metricDfHz(isfinite(navResults.metricDfHz)));
out.metricZMedian = median(navResults.metricZ(isfinite(navResults.metricZ)));
out.hitCountMax = max(navResults.hitCount);
out.rhoCorrRmsMean = mean(navResults.rhoCorrRmsM(isfinite(navResults.rhoCorrRmsM)));
out.prrCorrRmsMean = mean(navResults.prrCorrRmsMps(isfinite(navResults.prrCorrRmsMps)));
out.navResults = navResults;
out.settingsOverride = settingsOverride;

if isCalib
    outFile = sprintf('%s_%s_calib.mat', outPrefix, char(tag));
else
    outFile = sprintf('%s_%s_%ds.mat', outPrefix, char(tag), round(out.epochs*navSolPeriodSec));
end
save(outFile, 'navResults', 'settingsOverride', 'tag', '-v7.3');

fprintf('[GOAL2] %s -> %s\n', upper(char(tag)), outFile);
fprintf('        epochs=%d, maxState=%d, firstSpoof=%s\n', ...
    out.epochs, out.stateMax, num2str(out.firstSpoofSec, '%.2f'));
end

function [biasByPrn, Tcm, Tdf, Tsat, Tz] = buildCalibration(navResults, opts)
prnList = navResults.prnList(:);
resAll = navResults.residualHz;
ep1 = max(1, min(size(resAll, 2), opts.calibStartEpoch));
ep2 = max(ep1, min(size(resAll, 2), opts.calibEndEpoch));
res = resAll(:, ep1:ep2);

biasByPrn = zeros(1, 64);
for ii = 1:numel(prnList)
    prn = prnList(ii);
    rr = res(ii, :);
    rr = rr(isfinite(rr));
    if ~isempty(rr) && prn <= numel(biasByPrn)
        biasByPrn(prn) = median(rr);
    end
end

resNoBias = res;
for ii = 1:numel(prnList)
    prn = prnList(ii);
    if prn <= numel(biasByPrn)
        resNoBias(ii, :) = resNoBias(ii, :) - biasByPrn(prn);
    end
end

if isfield(navResults, 'residualDetHz')
    resDet = navResults.residualDetHz(:, ep1:ep2);
else
    resDet = resNoBias;
end

cm = navResults.metricCmHz(ep1:ep2); cm = cm(isfinite(cm));
df = navResults.metricDfHz(ep1:ep2); df = df(isfinite(df));
sat = abs(resDet(:)); sat = sat(isfinite(sat));
zz = navResults.metricZ(ep1:ep2); zz = zz(isfinite(zz));

Tcm = robustThr(cm, opts.kCm);
Tdf = robustThr(df, opts.kDf);
Tsat = robustThr(sat, opts.kSat);
Tz = robustThr(zz, opts.kZ);
Tcm = max(Tcm, opts.minTcm);
Tdf = max(Tdf, opts.minTdf);
Tsat = max(Tsat, opts.minTsat);
Tz = max(Tz, opts.minTz);
end

function [biasByPrn, Tcm, Tdf, Tsat, Tz, refResidualHz] = loadCalibrationFromFile(calibFile, useRefResidual)
if isempty(calibFile)
    error('opts.reuseCalib=true requires opts.calibFile.');
end
if exist(calibFile, 'file') ~= 2
    error('Calibration file not found: %s', calibFile);
end

S = load(calibFile);
if isfield(S, 'calib') && isstruct(S.calib)
    src = S.calib;
else
    src = S;
end

required = {'biasByPrn', 'Tcm', 'Tdf', 'Tsat', 'Tz'};
for ii = 1:numel(required)
    key = required{ii};
    if ~isfield(src, key)
        error('Calibration file missing field "%s": %s', key, calibFile);
    end
end

biasByPrn = double(src.biasByPrn(:).');
if numel(biasByPrn) < 64
    biasByPrn(64) = 0;
elseif numel(biasByPrn) > 64
    biasByPrn = biasByPrn(1:64);
end

Tcm = double(src.Tcm);
Tdf = double(src.Tdf);
Tsat = double(src.Tsat);
Tz = double(src.Tz);

refResidualHz = [];
if useRefResidual
    if isfield(src, 'refResidualHz')
        refResidualHz = src.refResidualHz;
    elseif isfield(S, 'refResidualHz')
        refResidualHz = S.refResidualHz;
    else
        warning(['[GOAL2] Calibration file has no refResidualHz. ', ...
            'deepUseReferenceResidual will run without reference subtraction.']);
    end
end
end

function saveCalibrationToFile(calibFile, biasByPrn, Tcm, Tdf, Tsat, Tz, refResidualHz, opts, outPrefix)
[calibDir, ~, ~] = fileparts(calibFile);
if ~isempty(calibDir) && exist(calibDir, 'dir') ~= 7
    mkdir(calibDir);
end

calib = struct();
calib.biasByPrn = biasByPrn;
calib.Tcm = Tcm;
calib.Tdf = Tdf;
calib.Tsat = Tsat;
calib.Tz = Tz;
calib.refResidualHz = refResidualHz;
calib.generatedBy = mfilename;
calib.generatedOn = datestr(now, 31);
calib.sourceOutPrefix = outPrefix;
calib.deepUseReferenceResidual = opts.deepUseReferenceResidual;
save(calibFile, '-struct', 'calib', '-v7.3');
end

function th = robustThr(x, k)
if isempty(x)
    th = inf;
    return;
end
mu = median(x);
sig = 1.4826 * median(abs(x - mu));
if sig <= 0
    sig = std(x);
end
th = mu + k * sig;
end

function s = epochToSec(ep, dt)
if isempty(ep)
    s = nan;
else
    s = (ep - 1) * dt;
end
end

function S = fillDefault(S, name, val)
if ~isfield(S, name)
    S.(name) = val;
end
end

function p = pickFirstExisting(cands)
p = '';
for ii = 1:numel(cands)
    c = cands{ii};
    if isstring(c), c = char(c); end
    if isempty(c), continue; end
    if exist(c, 'file') == 2
        p = c;
        return;
    end
end
end
