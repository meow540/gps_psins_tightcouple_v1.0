
function opts = make_goal2_recoveryboost_120s_opts(outPrefix, datasets)
%MAKE_GOAL2_RECOVERYBOOST_120S_OPTS Aggressive real-signal tracking recovery.
% Goal: recover more real shadow tracks first; quality is enforced later by
% passReady aging and nav-qualified consistency. No shadow EKF feedback.
if nargin < 1 || isempty(outPrefix)
    outPrefix = 'rt_deep_goal2_recoveryboost_ds5ds6_120s';
end
if nargin < 2 || isempty(datasets)
    datasets = {'ds5', 'ds6'};
end
opts = make_goal2_frontfix_120s_opts(outPrefix, datasets);
opts.deepShadowScenarioMode = 'mixed';
if numel(datasets) == 1
    ds = lower(string(datasets{1}));
    if ds == "ds5"
        opts.deepShadowScenarioMode = 'ds5_clock_drag';
    elseif ds == "ds6"
        opts.deepShadowScenarioMode = 'ds6_geometry_drag';
    end
end

% Keep this experiment tracking-only by default. DS5 overrides this later.
opts.deepShadowUseNavQualifiedForKf = 0;
opts.deepShadowDs5SoftRecoveryEnable = 1;
opts.deepShadowDs5SoftRecoveryStartSec = 92.0;
opts.deepShadowDs5SoftRecoveryRequireBranchSane = 1;
opts.deepShadowDs5ClosedLiftPreferEnable = 1;
opts.deepShadowDs5ClosedLiftPreferStartSec = 118.0;
opts.deepShadowDs5RawLiftRejectDevM = 12000.0;
opts.deepShadowDs5KfSourceGateEnable = 1;
opts.deepShadowDs5KfSourceGateStartSec = 92.0;
opts.deepShadowDs5KfRequireCommonDrag = 1;
opts.deepShadowDs5KfTrustedMinSat = 3;
opts.deepShadowDs5KfMode2MinSat = 2;
opts.deepShadowDs5KfAllowMode3Prns = [27 3];
opts.deepShadowDs5KfAllowMode3MaxDetM = 120.0;
opts.deepShadowDs5KfAllowMode3OnlyWhenBranchSane = 1;
opts.deepShadowDs5AuthorityEnable = 1;
opts.deepShadowDs5AuthorityStartSec = 92.0;
opts.deepShadowDs5AuthorityConfirmEpochs = 3;
opts.deepShadowDs5AuthorityHoldEpochs = 4;
opts.deepShadowDs5AuthorityRequireBranchSane = 1;
opts.deepShadowDs5AuthorityRequireClosedLoopRef = 1;
opts.deepShadowDs5AuthorityRefPosMaxDiffM = 900.0;
opts.deepShadowDs5AuthorityHardRevokeEnable = 1;
opts.deepShadowDs5AuthorityHardRevokeRefDiffM = 900.0;
opts.deepShadowDs5AuthorityFastDeauthBadEpochs = 2;
opts.deepShadowDs5AuthorityTrustedMinSat = 3;
opts.deepShadowDs5AuthorityMode2MinSat = 2;
opts.deepShadowDs5AuthorityMode23MinSat = 3;
opts.deepShadowDs5ObserveOnlyFollowEnable = 1;
opts.deepShadowDs5FinalOutputEnable = 1;
opts.deepShadowDs5FinalOutputHoldEpochs = 360;
opts.deepShadowDs5FinalRecoveryBaselineGateEnable = 1;
opts.deepShadowDs5FinalRecoveryBaselineMaxDiffM = 350.0;
opts.deepShadowDs5FinalRecoveryImproveGateEnable = 1;
opts.deepShadowDs5FinalRecoveryImproveMinM = 120.0;
opts.deepShadowDs5BaselineTrustedRefEnable = 1;
opts.deepShadowDs5BaselineTrustedRefUseVelocity = 1;
opts.deepShadowDs5BranchAbsoluteConsistencyEnable = 1;
opts.deepShadowDs5BranchAbsoluteConsistencyMaxDiffM = 1200.0;
opts.deepShadowDs5BranchLocalCorrectionMaxM = 1200.0;
opts.deepShadowDs5BranchLocalStepMaxM = 250.0;
opts.deepShadowDs5BranchLocalIterMax = 3;
opts.deepShadowDs5BranchLocalCorrPenaltyWeight = 0.10;
opts.deepShadowDs5BranchObsGateEnable = 1;
opts.deepShadowDs5BranchObsGateMinSat = 4;
opts.deepShadowDs5BranchObsGateFloorM = 120.0;
opts.deepShadowDs5BranchObsGateMadScale = 3.0;
opts.deepShadowDs5BranchObsGateCeilM = 1500.0;
opts.deepShadowDs5SoftClampEnable = 1;
opts.deepShadowDs5SoftClampStartSec = 92.0;
opts.deepShadowDs5SoftClampAlpha = 0.25;
opts.deepShadowDs5SoftClampVelBlend = 0.20;
opts.deepUseGnssKfUpdateSpoof = 0;
opts.deepUseKfClockUpdateSpoof = 0;

% Broader, denser candidate search. This specifically attacks missed PRNs
% whose true peak is not the top-3/cluster-center hypothesis.
opts.deepShadowRawReacqIntervalEpochs = 4;
opts.deepShadowRawReacqHalfChips = 24.0;
opts.deepShadowRawReacqStepChips = 0.50;
opts.deepShadowRawReacqFreqHalfHz = 1500;
opts.deepShadowRawReacqFreqStepHz = 125;
opts.deepShadowRawReacqNoncohMs = 12;
opts.deepShadowRawReacqCoherentMs = 1;
opts.deepShadowRawReacqTopK = 5;
opts.deepShadowRawCandTopKUse = 5;
opts.deepShadowRawReacqNmsChipGuard = 1.25;
opts.deepShadowRawReacqNmsFreqGuardHz = 250;
opts.deepShadowRawReacqFineHalfChips = 3.0;
opts.deepShadowRawReacqFineStepChips = 0.25;
opts.deepShadowRawReacqFineFreqHalfHz = 375;
opts.deepShadowRawReacqFineFreqStepHz = 125;

% Let plausible pending tracks prove themselves quickly by residual, instead
% of waiting for cluster geometry to be perfect.
opts.deepShadowRawTrackProtectEpochs = 2;
opts.deepShadowContinuousTrackEnable = 1;
opts.deepShadowRawTrackValidateEpochs = 1;
opts.deepShadowRawTrackValidateWindowEpochs = 3;
opts.deepShadowRawTrackValidateWindowPassHits = 1;
opts.deepShadowRawTrackValidateAbsDeltaEnable = 1;
opts.deepShadowRawTrackValidateAbsDeltaMaxM = 5000.0;
opts.deepShadowRawTrackValidateDuringHoldEnable = 1;
opts.deepShadowRawTrackValidateMaxEpochs = 12;
opts.deepShadowRawTrackClusterExtraMaxEpochs = 8;
opts.deepShadowRawTrackClusterRecheckBudget = 4;
opts.deepShadowRawTrackClusterRecheckHoldEpochs = 1;
opts.deepShadowRawTrackEarlyRejectEpochs = 3;
opts.deepShadowRawTrackEarlyRejectImproveMaxM = -800.0;
opts.deepShadowRawTrackScoreRejectMin = -3.0;
opts.deepShadowRawTrackClusterRejectFloorM = -1200.0;
opts.deepShadowRawTrackClusterSoftRejectEpochs = 5;
opts.deepShadowRawRelayCooldownEpochs = 0;
opts.deepShadowRawCandReuseMaxEpochs = 12;

% Make cluster entry easier, but rely on residual/passReady aging to remove bad tracks.
opts.deepShadowRawClusterInitScore = 0.8;
opts.deepShadowRawClusterInitHits = 2;
opts.deepShadowRawClusterInitMargin = 0.0;
opts.deepShadowRawClusterDirectInitScore = 1.0;
opts.deepShadowRawClusterDirectInitHits = 2;
opts.deepShadowRawClusterDirectInitMargin = 0.0;
opts.deepShadowRawExploreClusterScore = 0.7;
opts.deepShadowRawExploreClusterHits = 1;
opts.deepShadowRawExploreClusterMargin = 0.0;
opts.deepShadowRawClusterValidateTolChips = 3.0;
opts.deepShadowRawClusterValidateTolFreqHz = 500;

% Stronger quality cleanup after acceptance.
opts.deepShadowRawPassReadyAgingEnable = 1;
opts.deepShadowRawPassReadyRevokeEnable = 0;
opts.deepShadowRawPassUseActivePool = 1;
opts.deepShadowRawPassReadyBadEpochs = 1;
opts.deepShadowRawPassReadyDevThrM = 10000.0;
opts.deepShadowRawPassReadyAbsThrM = inf;
opts.deepShadowRawNavExpandEnable = 1;
opts.deepShadowRawNavCoreDevThrM = 10000.0;
opts.deepShadowRawNavExpandDevThrM = 10000.0;
opts.deepShadowDs5TailNavRelaxEnable = 1;
opts.deepShadowDs5TailNavRelaxStartSec = 150.0;
opts.deepShadowDs5TailNavRelaxPrns = [27 3];
opts.deepShadowDs5TailNavRelaxMinBaseSat = 3;
opts.deepShadowDs5TailNavRelaxMinThrM = 250.0;
opts.deepShadowDs5TailNavRelaxMaxThrM = 2500.0;
opts.deepShadowDs5TailNavRelaxMadScale = 4.0;
opts.deepShadowDs5TailNavRelaxCoreScale = 0.75;
opts.deepShadowDs5TailNavRelaxPairThrM = 1200.0;
opts.deepShadowDs5TailSourceFallbackEnable = 1;
opts.deepShadowDs5TailSourceFallbackStartSec = 118.0;
opts.deepShadowDs5TailSourceFallbackPrns = [27 3];
opts.deepShadowDs5TailSourceFallbackUseClosedLift = 1;
opts.deepShadowDs5TailSourceFallbackUseHold = 1;
opts.deepShadowDs5TailSourceFallbackDevThrM = 1500.0;
opts.deepShadowDs5TailSourceFallbackJumpThrM = 600.0;
opts.deepShadowDs5TailSourceFallbackHoldMaxAgeSec = 120.0;
opts.deepShadowDs5CommonDragEnable = 1;
opts.deepShadowDs5CommonDragStartSec = 118.0;
opts.deepShadowDs5CommonDragFocusPrns = [27 3];
opts.deepShadowDs5CommonDragMinBaseSat = 3;
opts.deepShadowDs5CommonDragGateFloorM = 250.0;
opts.deepShadowDs5CommonDragMadScale = 4.0;
opts.deepShadowDs5CommonDragMaxInnovM = 6000.0;
opts.deepShadowDs5CommonDragMaxRateMps = 4000.0;
opts.deepShadowDs5CommonDoppEnable = 1;
opts.deepShadowDs5CommonDoppStartSec = 92.0;
opts.deepShadowDs5CommonDoppFocusPrns = [27 3];
opts.deepShadowDs5CommonDoppMinBaseSat = 3;
opts.deepShadowDs5CommonDoppGateFloorHz = 10.0;
opts.deepShadowDs5CommonDoppMadScale = 4.0;
opts.deepShadowDs5CommonDoppMaxInnovHz = 80.0;
opts.deepShadowDs5CommonDoppMaxRateHzps = 120.0;
opts.deepShadowDs5CommonDoppApplyToAidEnable = 1;
opts.deepShadowDs5CommonDoppAidApplyMaxHz = 200.0;
opts.deepShadowDs5TruePeakEnable = 1;
opts.deepShadowDs5TruePeakStartSec = 92.0;
opts.deepShadowDs5TruePeakFreqWindowHz = 125.0;
opts.deepShadowDs5TruePeakCodeWindowChips = 0.75;
opts.deepShadowDs5TruePeakCodeWindowMaxChips = 1.50;
opts.deepShadowDs5TruePeakUseCodeRef = 1;
opts.deepShadowDs5RefObsModelEnable = 1;
opts.deepShadowDs5RefObsModelStartSec = 92.0;
opts.deepShadowDs5RefObsAutoActivate = 1;
opts.deepShadowDs5RefObsMinBaseSat = 3;
opts.deepShadowDs5RefObsLocalCorrMaxChips = 1.50;
opts.deepShadowDs5RefObsUseCodeCorr = 1;
opts.deepShadowDs5RefObsRequireActiveTrack = 0;
opts.deepShadowDs5CodeRefHalfChips = 0.50;
opts.deepShadowDs5CodeRefStepChips = 0.10;
opts.deepShadowDs5CodeRefPeakRatioMin = 1.01;
opts.deepShadowDs5CodeRefZeroRatioMin = 1.00;
opts.deepShadowDs5CodeRefApplyGain = 0.50;
opts.deepShadowDs5CodeRefStepMaxChips = 0.08;
opts.deepShadowDs5CodeRefTotalMaxChips = 1.50;
opts.deepShadowDs5CodeRefMinPromptMag = 0;
opts.deepShadowDs5CodeRefNcoPullEnable = 1;
opts.deepShadowDs5CodeRefNcoGain = 1.0;
opts.deepShadowDs5CodeRefNcoMaxHz = 120.0;
opts.deepShadowDs5RefObsRecoveryEnable = 1;
opts.deepShadowDs5RefObsRecoveryStartSec = 92.0;
opts.deepShadowDs5RefObsRecoveryMinSat = 4;
opts.deepShadowDs5RefObsRecoveryMinUsedSat = 4;
opts.deepShadowDs5RefObsRecoveryCodeMedMaxChips = 0.50;
opts.deepShadowDs5RefObsRecoveryCodeP95MaxChips = 1.20;
opts.deepShadowDs5RefObsRecoveryFreqP95MaxHz = 300.0;
opts.deepShadowDs5RefObsRecoveryRequireBranch = 0;
opts.deepShadowDs5RefObsRecoveryBranchPostfitMaxM = 250.0;
opts.deepShadowDs5RefObsRecoveryBaselineDiffMaxM = 2500.0;
opts.deepShadowDs5RefObsClosedLoopUseBaselineHold = 1;
opts.deepShadowDs5RefObsClosedLoopPreferBaselineHold = 1;
opts.deepShadowDs5RefObsPositionEnable = 1;
opts.deepShadowDs5RefObsPositionMinSat = 4;
opts.deepShadowDs5RefObsPositionPostfitMaxM = 220.0;
opts.deepShadowDs5RefObsPositionPdopMax = 20.0;
opts.deepShadowDs5RefObsPositionCorrMaxM = 260.0;
opts.deepShadowDs5RefObsPositionAnchorDiffMaxM = 260.0;
opts.deepShadowDs5RefObsPositionJumpMaxM = 220.0;
opts.deepShadowDs5RefObsPositionJumpMaxAgeEpochs = 4;
opts.deepShadowDs5RefObsPositionSpeedGateEnable = 1;
opts.deepShadowDs5RefObsPositionSpeedMaxMps = 140.0;
opts.deepShadowDs5RefObsPositionAccelMaxMps2 = 180.0;
opts.deepShadowDs5RefObsPositionDynGateMaxAgeEpochs = 4;
opts.deepShadowDs5FinalForceRefObsRecovered = 0;
opts.deepShadowDs5FinalQualitySelectEnable = 1;
opts.deepShadowDs5FinalQualityImproveMinM = -10.0;
opts.deepShadowDs5FinalQualityBaselineDiffMaxM = 180.0;
opts.deepShadowDs5FinalRecoveredAuthorityRequireBaselineGate = 1;
opts.deepShadowDs5FinalRecoveredAuthorityMaxBaselineDiffM = opts.deepShadowDs5FinalQualityBaselineDiffMaxM;
opts.deepShadowDs5FinalAllowNoBaselineRecovered = 0;
opts.deepShadowDs5FinalRecoveredConfirmEpochs = 3;
opts.deepShadowDs5FinalContinuityGateEnable = 1;
opts.deepShadowDs5FinalContinuityMaxPredDiffM = 60.0;
opts.deepShadowDs5FinalContinuitySlewEnable = 1;
opts.deepShadowDs5FinalContinuityMaxAgeEpochs = 6;
opts.deepShadowDs5FinalHoldBaselineCompeteEnable = 1;
opts.deepShadowDs5FinalHoldBaselineMinAgeEpochs = 2;
opts.deepShadowDs5FinalHoldBaselineProxyMarginM = 20.0;
opts.deepShadowDs5FinalHoldBaselineDiffMaxM = 220.0;
opts.deepShadowDs5FinalHoldBaselineForceDiffM = 220.0;
opts.deepShadowDs5FinalHoldBaselineMaxAgeEpochs = 6;
opts.deepShadowDs5FinalRecoveredAuthorityEnable = 1;
opts.deepShadowDs5FinalBaselineOffAblation = 0;
opts.deepShadowDs5RecoveredFilterEnable = 1;
opts.deepShadowDs5RecoveredFilterConfirmEpochs = 3;
opts.deepShadowDs5RecoveredFilterMaxBadEpochs = 24;
opts.deepShadowDs5RecoveredFilterMaxCoastEpochs = 80;
opts.deepShadowDs5RecoveredFilterUpdateAlpha = 0.85;
opts.deepShadowDs5RecoveredFilterVelMeasBlend = 0.35;
opts.deepShadowDs5RecoveredFilterVelInsBlend = 0.10;
opts.deepShadowDs5RecoveredFilterMeasMaxPredDiffM = 350.0;
opts.deepShadowDs5RecoveredFilterPropagateWithInsVel = 1;
opts.deepShadowDs5RecoveredFilterDopplerVelEnable = 1;
opts.deepShadowDs5RecoveredFilterDopplerVelMinSat = 4;
opts.deepShadowDs5RecoveredFilterDopplerVelMaxResidMps = 85.0;
opts.deepShadowDs5RecoveredFilterDopplerVelBlend = 0.35;
opts.deepShadowDs5RecoveredFilterDopplerVelRobustMaxDropSat = 2;
opts.deepShadowDs5RefObsPositionRobustSubsetEnable = 1;
opts.deepShadowDs5RefObsPositionRobustMaxDropSat = 2;
opts.deepShadowDs5RefObsPositionRobustDropPenaltyM = 20.0;
opts.deepShadowDs5TruePeakMetricWeight = 0.20;
opts.deepShadowDs5TruePeakFreqWeight = 0.40;
opts.deepShadowDs5TruePeakCodeWeight = 0.20;
opts.deepShadowDs5TruePeakResidualWeight = 0.20;
opts.deepShadowDs5TruePeakCurrentMetricBias = 0.35;
opts.deepShadowDs5TruePeakCandidateMargin = 0.10;
opts.deepShadowDs5TruePeakProbationEpochs = 4;
opts.deepShadowDs5TruePeakProbationWindowEpochs = 6;
opts.deepShadowDs5TruePeakProbationPassHits = 4;
opts.deepShadowDs5TruePeakTakeoverConfirmEpochs = 3;
opts.deepShadowDs5TruePeakRevokeBadEpochs = 2;
opts.deepShadowDs5TruePeakRevokeFreqErrHz = 180.0;
opts.deepShadowDs5TruePeakRevokeCodeErrChips = 1.25;
opts.deepShadowDs5TruePeakRevokeImproveMaxM = -150.0;
opts.deepShadowMainSwitchEnable = 1;
opts.deepShadowMainSwitchImproveMinM = 0.0;
opts.deepShadowMainSwitchConfirmEpochs = 3;
opts.deepShadowMainSwitchHoldEpochs = 8;
opts.deepShadowMainSwitchWaitProtectDone = 1;
opts.deepShadowDs5DetrendedGateEnable = 1;
opts.deepShadowDs5DetrendedGateStartSec = 118.0;
opts.deepShadowDs5DetrendedGateMinSat = 4;
opts.deepShadowDs5DetrendedGateFloorM = 180.0;
opts.deepShadowDs5DetrendedGateMadScale = 4.0;
opts.deepShadowDs5DetrendedGateCeilM = 1200.0;
opts.deepShadowDs5DetrendedGateProtectPrns = [27 3];
opts.deepShadowBranchDs5DetrendedScoreWeight = 1.5;
opts.deepShadowBranchDs5DetrendedP95Weight = 0.5;
opts.deepShadowBranchDs5DetrendedReadyP95MaxM = 250.0;
opts.deepShadowDs5BranchTakeoverAllow4Sat = 1;
opts.deepShadowDs5BranchTakeoverDetrendedP95MaxM = 180.0;
opts.deepShadowDs5BranchContinueAllow4Sat = 1;
opts.deepShadowDs5BranchContinueDetrendedP95MaxM = 220.0;
opts.deepShadowRawAmbiguityEnable = 1;
opts.deepShadowRawAmbiguityMaxChips = 80.0;
opts.deepShadowRawAmbiguityStepMaxChips = 8.0;
opts.deepShadowRawAmbiguityMinSat = 4;
opts.deepShadowBranchSearchEnable = 1;
opts.deepShadowBranchUseNavQ = 1;
opts.deepShadowBranchMinSat = 5;
opts.deepShadowBranchMaxChips = 120.0;
opts.deepShadowBranchRobustSubsetEnable = 1;
opts.deepShadowBranchRobustSubsetMinSat = 4;
opts.deepShadowBranchRobustSubsetMaxSat = 5;
opts.deepShadowBranchRobustSubsetMaxCand = 128;
opts.deepShadowBranchRobustSubsetDropPenaltyM = 15.0;
opts.deepShadowBranchRobustSubsetFourSatPenaltyM = 300.0;
opts.deepShadowBranchRobustDopplerWeight = 0.15;
opts.deepShadowBranchAmbigContinuityEnable = 1;
opts.deepShadowBranchAmbigContinuityMaxAgeEpochs = 80;
opts.deepShadowBranchAmbigContinuityStepMaxChips = 2.0;
opts.deepShadowBranchAmbigContinuityTolChips = 0.75;
opts.deepShadowBranchAmbigContinuityPenaltyM = 120.0;
opts.deepShadowBranchAmbigContinuityJumpPenaltyM = 250.0;
opts.deepShadowBranchPriorRmsMaxM = 500.0;
opts.deepShadowBranchPostfitRmsMaxM = 300.0;
opts.deepShadowBranchPdopMax = 20.0;
opts.deepShadowBranchPosJumpMaxM = 3000.0;
opts.deepShadowTrustedAnchorEnable = 1;
opts.deepShadowTrustedAnchorUseTruthTrj = 1;
opts.deepShadowTrustedAnchorTimeSec = 90.0;
opts.deepShadowTrustedAnchorUseVelocity = 1;
opts.deepShadowTrustedAnchorAllowCurrentFallback = 0;
opts.deepShadowBranchTakeoverConfirmEpochs = 3;
opts.deepShadowBranchTakeoverPostfitRmsMaxM = 100.0;
opts.deepShadowBranchTakeoverPdopMax = 10.0;
opts.deepShadowBranchTakeoverMinSat = 5;
opts.deepShadowBranchTakeoverRequireTrustedPrior = 1;

% Guarded closed-loop recovery: once branch-search is sane for several
% epochs, reset the navigation state to the trusted branch and optionally use
% branch-lifted pseudoranges as shadow EKF measurements.
opts.deepShadowBranchResetEnable = 1;
opts.deepShadowBranchResetOnce = 1;
opts.deepShadowBranchResetMaxJumpM = 50000.0;
opts.deepShadowBranchResetVelBlend = 0.0;
opts.deepShadowBranchResetClock = 1;
opts.deepShadowBranchInjectEnable = 1;
opts.deepShadowBranchInjectRScale = 25.0;
opts.deepShadowBranchInjectMinSat = 5;
opts.deepShadowBranchInjectResidualGateM = 300.0;
opts.deepShadowRecoveryModeEnable = 1;
opts.deepShadowRecoveryHoldEpochs = 80;
opts.deepShadowRecoverySuppressMainGnss = 1;
opts.deepShadowRecoveryContinueMinSat = 5;
opts.deepShadowRecoveryContinuePostfitRmsMaxM = 120.0;
opts.deepShadowRecoveryContinuePdopMax = 12.0;
opts.deepShadowRecoveryRequireTrustedPrior = 1;
opts.deepShadowRecoveryReanchorEnable = 1;
opts.deepShadowRecoveryReanchorMinJumpM = 500.0;
opts.deepShadowRecoveryReanchorCooldownEpochs = 3;
opts.deepShadowRecoveryReanchorMaxCount = 20;
opts.deepShadowClosedLoopUseBranchOutput = 1;
opts.deepShadowClosedLoopCoastEpochs = 160;
opts.deepShadowClosedLoopUseVelocityCoast = 1;
opts.deepShadowClosedLoopAllowNavFallbackInRecovery = 0;
opts.deepShadowClosedLoopUseTrustedFallback = 1;
opts.deepShadowClosedLoopTrustedBlendEnable = 1;
opts.deepShadowClosedLoopTrustedBlendBranchAlpha = 0.75;
opts.deepShadowClosedLoopTrustedBlendCoastAlpha = 0.75;
opts.deepShadowClosedLoopSafeBranchGateEnable = 1;
opts.deepShadowClosedLoopSafeBranchMaxDiffM = 450.0;
opts.deepShadowClosedLoopSafeBranchPostfitMaxM = 80.0;
opts.deepShadowClosedLoopSafeBranchPdopMax = 8.0;
opts.deepShadowClosedLoopSafeBranchMinSat = 5;
opts.deepShadowClosedLoopSafeBranchAlpha = 0.25;

% Close the loop only on the strict core: reduce relative pseudorange spread
% without letting expand candidates steer the code phase.
opts.deepShadowCoreServoEnable = 1;
opts.deepShadowCoreServoGain = 0.05;
opts.deepShadowCoreServoStepMaxChips = 0.50;
opts.deepShadowCoreServoTotalMaxChips = 40.0;
opts.deepShadowCoreServoMinSat = 4;
opts.deepShadowCoreServoCoreDevMaxM = 10000.0;
opts.deepShadowCoreServoMinElevationDeg = 0.0;

% Re-apply DS5 structural overrides after generic defaults.
if numel(datasets) == 1
    ds = lower(string(datasets{1}));
    if ds == "ds5"
        opts.deepShadowUseNavQualifiedForKf = 1;
        opts.deepShadowBranchResetEnable = 0;
        opts.deepShadowBranchInjectEnable = 0;
        opts.deepShadowRecoveryModeEnable = 1;
        opts.deepShadowRecoverySuppressMainGnss = 1;
        opts.deepShadowTrustedAnchorUseTruthTrj = 0;
        opts.deepShadowClosedLoopUseBranchOutput = 1;
        opts.deepShadowClosedLoopAllowNavFallbackInRecovery = 0;
        opts.deepShadowClosedLoopUseTrustedFallback = 0;
        opts.deepShadowClosedLoopTrustedBlendEnable = 0;
    end
end
end
