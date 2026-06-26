clear;
clc;

projectRoot = fileparts(mfilename('fullpath'));
cd(projectRoot);
deepCoupleScript = prepareDeepCoupleRuntime(projectRoot);
localPrepareMatlabHeapSafeRuntime();

summaryStartSec = 180.0;
trueRefStartSec = 176.0;
runEndSec = 400.0;
anchorMode = lower(strtrim(getenv('DS5_HEAP_SAFE_ANCHOR_MODE')));
if isempty(anchorMode)
    anchorMode = 'proof';
end
if ~ismember(anchorMode, {'proof', 'online', 'online_loose', 'online_ins', 'online_prop', 'online_base'})
    error('GOAL2:BadAnchorMode', ...
        'DS5_HEAP_SAFE_ANCHOR_MODE must be proof, online, online_loose, online_ins, online_prop, or online_base, got: %s', anchorMode);
end

endSecEnv = str2double(getenv('DS5_HEAP_SAFE_END_SEC'));
if isfinite(endSecEnv) && endSecEnv > summaryStartSec
    runEndSec = endSecEnv;
end

startSecEnv = str2double(getenv('DS5_HEAP_SAFE_START_SEC'));
if isfinite(startSecEnv) && startSecEnv > 0 && startSecEnv < summaryStartSec
    trueRefStartSec = startSecEnv;
end

rawStrideEpochs = 1;       % 0.5 s nav period -> raw sample every epoch
rawBurstMs = 20.0;         % bounded raw snapshot per satellite and epoch
rawWarmupEpochs = 0;

strideEnv = str2double(getenv('DS5_HEAP_SAFE_RAW_STRIDE_EPOCHS'));
if isfinite(strideEnv) && strideEnv >= 1
    rawStrideEpochs = round(strideEnv);
end

burstEnv = str2double(getenv('DS5_HEAP_SAFE_RAW_BURST_MS'));
if isfinite(burstEnv) && burstEnv > 0
    rawBurstMs = burstEnv;
end

warmupEnv = str2double(getenv('DS5_HEAP_SAFE_RAW_WARMUP_EPOCHS'));
if isfinite(warmupEnv) && warmupEnv >= 0
    rawWarmupEpochs = round(warmupEnv);
end

outPrefix = sprintf('rt_deep_goal2_ds5_heap_safe_true_tracking_goal_%03.0fs', round(runEndSec));
if ~strcmp(anchorMode, 'proof')
    outPrefix = sprintf('%s_%s', outPrefix, anchorMode);
end
resultFile = sprintf('%s_ds5_%03.0fs.mat', outPrefix, round(runEndSec));
logFile = [outPrefix '_run.log'];
doneFile = [outPrefix '_summary_done.txt'];

if exist(logFile, 'file') == 2
    delete(logFile);
end
if exist(doneFile, 'file') == 2
    delete(doneFile);
end
diary(logFile);
batchCleanup = onCleanup(@() localCleanupMatlabHeapSafeRuntime()); %#ok<NASGU>

opts = make_goal2_recoveryboost_120s_opts(outPrefix, {'ds5'});
opts.datasets = {'ds5'};
opts.roundTime = round(runEndSec / 0.5);  % 0.5 s nav period
opts.calibEndEpoch = 180;
opts.deepPerfDiagInterval = 20;
opts.deepShadowDs5ValidationProofMode = double(strcmp(anchorMode, 'proof'));
opts.deepShadowDs5ValidationAnchorModeName = anchorMode;

% Heap-safe long-window mode: use precomputed main tracking, keep only the
% DS5 true-reference refObs raw path, and avoid generic shadow raw search.
opts.deepFastMode = 1;
opts.deepShadowDs5FastRefObsTrackBypass = 0;
opts.deepShadowDs5RefObsTrackRawStrideEpochs = rawStrideEpochs;
opts.deepShadowDs5RefObsTrackRawBurstMs = rawBurstMs;
opts.deepShadowDs5RefObsTrackRawWarmupEpochs = rawWarmupEpochs;
opts.deepShadowDs5RefObsTrackSnapshotAcceptEnable = 1;
opts.deepShadowRawAmbiguityEnable = 1;
opts.deepShadowRawAmbiguityMaxChips = 2000.0;
opts.deepShadowRawAmbiguityStepMaxChips = 2000.0;
opts.deepShadowRawAmbiguityMinSat = 4;

opts.deepShadowAcqEnable = 0;
opts.deepShadowRawReacqEnable = 0;
opts.deepShadowRawTrackEnable = 0;
opts.deepShadowMainSwitchEnable = 0;
opts.deepShadowRawPassReadyAgingEnable = 0;
opts.deepShadowRawNavExpandEnable = 0;
opts.deepShadowBranchSearchEnable = 0;
opts.deepShadowBranchResetEnable = 0;
opts.deepShadowBranchInjectEnable = 0;
opts.deepShadowRecoveryModeEnable = 1;
opts.deepShadowCoreServoEnable = 0;

opts.deepUseGnssKfUpdate = 0;
opts.deepUseGnssKfUpdateNormal = 0;
opts.deepUseGnssKfUpdateSuspect = 0;
opts.deepUseGnssKfUpdateSpoof = 0;
opts.deepUseKfClockUpdate = 0;
opts.deepUseKfClockUpdateNormal = 0;
opts.deepUseKfClockUpdateSuspect = 0;
opts.deepUseKfClockUpdateSpoof = 0;

opts.deepShadowDs5RefObsModelEnable = 1;
opts.deepShadowDs5RefObsModelStartSec = trueRefStartSec;
opts.deepShadowDs5RefObsRecoveryStartSec = summaryStartSec;
opts.deepShadowDs5TruePeakEnable = 1;
opts.deepShadowDs5TruePeakStartSec = trueRefStartSec;
opts.deepShadowDs5TruePeakUseCodeRef = 1;
opts.deepShadowDs5CommonDragStartSec = trueRefStartSec;
opts.deepShadowDs5CommonDoppStartSec = trueRefStartSec;
opts.deepShadowDs5SoftRecoveryStartSec = trueRefStartSec;
opts.deepShadowDs5AuthorityStartSec = trueRefStartSec;
opts.deepShadowDs5KfSourceGateStartSec = trueRefStartSec;
opts.deepShadowDs5ClosedLiftPreferStartSec = trueRefStartSec;
opts.deepShadowDs5TailSourceFallbackStartSec = trueRefStartSec;
opts.deepShadowDs5TailNavRelaxStartSec = trueRefStartSec;
opts.deepShadowDs5SoftClampStartSec = trueRefStartSec;
opts.deepShadowDs5DetrendedGateStartSec = trueRefStartSec;
opts.deepShadowTrustedAnchorEnable = 1;
opts.deepShadowTrustedAnchorUseInsPropagated = 1;
opts.deepShadowTrustedAnchorAllowBaselineFedAnchor = 0;
opts.deepShadowTrustedAnchorAllowHigherPriorityReplace = 1;
opts.deepShadowTrustedAnchorRefreshSameSource = 0;
opts.deepShadowTrustedAnchorRefreshOnlyS5 = 0;
opts.deepShadowTrustedAnchorTimeSec = summaryStartSec;
opts.deepShadowTrustedAnchorUseVelocity = 0;
if strcmp(anchorMode, 'proof')
    opts.deepShadowTrustedAnchorUseTruthTrj = 1;
    opts.deepShadowTrustedAnchorPreferTruthTrj = 1;
    opts.deepShadowTrustedAnchorTruthUseCurrentEpoch = 1;
    opts.deepShadowTrustedAnchorInsPropagatedUseCurrent = 1;
    opts.deepShadowTrustedAnchorUseBaselineInsAlign = 1;
    opts.deepShadowTrustedAnchorRefreshSameSource = 1;
    opts.deepShadowTrustedAnchorRefreshOnlyS5 = 0;
else
    opts.deepShadowTrustedAnchorUseTruthTrj = 0;
    opts.deepShadowTrustedAnchorPreferTruthTrj = 0;
    opts.deepShadowTrustedAnchorTruthUseCurrentEpoch = 0;
    opts.deepShadowTrustedAnchorInsPropagatedUseCurrent = 1;
    opts.deepShadowTrustedAnchorUseBaselineInsAlign = 1;
    opts.deepShadowTrustedAnchorRefreshSameSource = 0;
    opts.deepShadowTrustedAnchorRefreshOnlyS5 = 0;
end
opts.deepShadowDs5BaselineTrustedRefMaxSec = trueRefStartSec - 0.5;
opts.deepShadowDs5BaselineTrustedRefUseVelocity = 0;

opts.deepShadowDs5RefObsAutoActivate = 1;
opts.deepShadowDs5RefObsRequireActiveTrack = 1;
opts.deepShadowDs5RefObsSeedUseBaseFallback = 0;
opts.deepShadowDs5RefObsSeedMaxDevFromBaseM = 1500.0;
opts.deepShadowDs5RefObsLocalCorrMaxChips = 1.50;
opts.deepShadowDs5RefObsUseCodeCorr = 1;

opts.deepShadowDs5CodeRefNcoPullEnable = 1;
opts.deepShadowDs5CodeRefNcoGain = 1.0;
opts.deepShadowDs5CodeRefNcoMaxHz = 120.0;
opts.deepTrackGuardEnable = 1;
opts.deepTrackGuardCodeFreqMinHz = 0.90 * 1.023e6;
opts.deepTrackGuardCodeFreqMaxHz = 1.10 * 1.023e6;
opts.deepTrackGuardCarrOffsetMaxHz = 20000.0;
opts.deepTrackGuardMinBlockSize = 1;
opts.deepTrackGuardMaxBlockSize = 12000;

opts.deepShadowDs5ObsContractEnable = 1;
opts.deepShadowDs5ObsContractMinSat = 4;
opts.deepShadowDs5ObsContractSpreadP95MaxM = 800.0;
opts.deepShadowDs5ObsContractRequireCommonClock = 1;
opts.deepShadowDs5ObsContractRequireTrackingPass = 1;
opts.deepShadowDs5ObsContractMaxBaseSeedFrac = 0.0;
opts.deepShadowDs5ObsContractNoBaselineSeedForRecovery = 1;
opts.deepShadowDs5ObsContractUseRecoveryKeep = 0;
opts.deepShadowDs5ObsContractRobustTrackingEnable = 0;
opts.deepShadowDs5ObsContractIndependentCodeErrMaxChips = 1.60;
opts.deepShadowDs5ObsContractIndependentFreqErrMaxHz = 350.0;

opts.deepShadowDs5RefObsPositionCommonClockRebaseEnable = 1;
opts.deepShadowDs5RefObsPositionCommonClockRebaseEachDelta = 1;
opts.deepShadowDs5RefObsPositionCommonClockRebasePeriodM = 299792.458;
opts.deepShadowDs5RefObsPositionCommonClockHalfPeriodRebaseEnable = 1;
opts.deepShadowDs5RefObsPositionCommonClockSubsetSoftMaxM = 1200.0;
opts.deepShadowDs5RefObsPositionCommonClockSubsetHardMaxM = 3500.0;
opts.deepShadowDs5RefObsPositionCommonClockSubsetScoreWeightM = 140.0;
opts.deepShadowDs5RefObsPositionCommonClockHalfPeriodGuardM = 2500.0;
opts.deepShadowDs5RefObsPositionCommonClockHalfPeriodPenaltyM = 260.0;
opts.deepShadowDs5RefObsPositionRawAnchorLiftEnable = 1;
opts.deepShadowDs5RefObsPositionRawAnchorLiftMaxSpreadP95M = 800.0;
opts.deepShadowDs5RefObsPositionRawAnchorLiftUseCodeCorr = 1;
opts.deepShadowDs5RefObsPositionPreferAbsAnchor = 1;
opts.deepShadowDs5RefObsPositionAbsAnchorUseHardGate = 1;
opts.deepShadowDs5RefObsPositionAbsAnchorSoftMaxDiffM = 300.0;
opts.deepShadowDs5RefObsPositionAbsAnchorMaxDiffM = 550.0;
opts.deepShadowDs5RefObsPositionAbsAnchorScoreWeightM = 140.0;
opts.deepShadowDs5RefObsPositionAbsAnchorPriorEnable = 1;
opts.deepShadowDs5RefObsPositionAbsAnchorPriorSigmaM = 900.0;
opts.deepShadowDs5RefObsPositionAbsAnchorPriorSoftSigmaM = 1800.0;
opts.deepShadowDs5RefObsPositionBootstrapEpochs = 40;
opts.deepShadowDs5RefObsPositionBootstrapCorrMaxM = 700.0;
opts.deepShadowDs5RefObsPositionBootstrapAnchorDiffMaxM = 700.0;
opts.deepShadowDs5RefObsPositionBootstrapJumpMaxM = 900.0;
opts.deepShadowDs5RefObsPositionResetHistoryAtRecoveryStart = 1;
opts.deepShadowDs5RefObsPositionCorrMaxM = 450.0;
opts.deepShadowDs5RefObsPositionAnchorDiffMaxM = 450.0;
opts.deepShadowDs5RefObsPositionAdaptiveCorrMaxM = 450.0;
opts.deepShadowDs5RefObsPositionAdaptiveAnchorDiffMaxM = 450.0;
opts.deepShadowDs5RefObsPositionJumpMaxM = 450.0;
opts.deepShadowDs5RefObsPositionAdaptiveJumpMaxM = 600.0;
opts.deepShadowDs5RefObsPositionSpeedGateEnable = 0;
if strcmp(anchorMode, 'online') || strcmp(anchorMode, 'online_loose')
    opts.deepShadowDs5RefObsPositionAbsAnchorScoreWeightM = 30.0;
    opts.deepShadowDs5RefObsPositionAbsAnchorPriorSigmaM = 2500.0;
    opts.deepShadowDs5RefObsPositionAbsAnchorPriorSoftSigmaM = 4500.0;
end
if strcmp(anchorMode, 'online_loose')
    opts.deepShadowDs5RefObsPositionPreferAbsAnchor = 0;
    opts.deepShadowDs5RefObsPositionAbsAnchorUseHardGate = 0;
    opts.deepShadowDs5RefObsPositionAbsAnchorPriorEnable = 0;
    opts.deepShadowDs5RefObsPositionRobustHardDropEnable = 0;
    opts.deepShadowDs5RefObsPositionUseRecentFilterHold = 1;
    opts.deepShadowDs5RefObsPositionRecentFilterHoldMaxAgeEpochs = 24;
    opts.deepShadowDs5RefObsPositionRecentFilterHoldTrustedMaxDiffM = 220.0;
    opts.deepShadowDs5RefObsPositionAcceptedHoldTrustedMaxDiffM = 220.0;
    opts.deepShadowDs5RefObsPositionHoldPredDiffMaxM = 280.0;
    opts.deepShadowDs5RefObsPositionBootstrapEpochs = 100;
    opts.deepShadowDs5RefObsPositionBootstrapCorrMaxM = 800.0;
    opts.deepShadowDs5RefObsPositionBootstrapAnchorDiffMaxM = 800.0;
    opts.deepShadowDs5RefObsPositionBootstrapJumpMaxM = 1000.0;
    opts.deepShadowDs5RefObsPositionTailStartSec = 320.0;
    opts.deepShadowDs5RefObsPositionTailCorrMaxM = 650.0;
    opts.deepShadowDs5RefObsPositionTailAnchorDiffMaxM = 650.0;
    opts.deepShadowDs5RefObsPositionTailJumpMaxM = 800.0;
    opts.deepShadowDs5RefObsPositionTailPredVelDiffMaxMps = 260.0;
    opts.deepShadowDs5RefObsPositionEscapeEnable = 1;
    opts.deepShadowDs5RefObsPositionEscapeStaleAgeEpochs = 8;
    opts.deepShadowDs5RefObsPositionEscapeMinSat = 5;
    opts.deepShadowDs5RefObsPositionEscapePostfitMaxM = 40.0;
    opts.deepShadowDs5RefObsPositionEscapeDeltaSpreadP95MaxM = 250.0;
    opts.deepShadowDs5RefObsPositionEscapeCorrMaxM = 900.0;
    opts.deepShadowDs5RefObsPositionEscapeAnchorDiffMaxM = 900.0;
    opts.deepShadowDs5RefObsPositionEscapeJumpMaxM = 1200.0;
    opts.deepShadowDs5RefObsPositionEscapePredVelDiffMaxMps = 250.0;
    opts.deepShadowTrustedAnchorRefreshSameSource = 1;
    opts.deepShadowTrustedAnchorRefreshOnlyS5 = 1;
    opts.deepShadowTrustedAnchorBaselineInsAlignUseBaselineCurrent = 1;
    opts.deepShadowTrustedAnchorBaselineInsAlignUseCurrent = 1;
    opts.deepShadowTrustedAnchorBaselineInsAlignRefreshAfterStart = 0;
    opts.deepShadowDs5RefObsPositionPreferTrustedAnchorStartSec = 180.0;
    opts.deepShadowDs5RefObsPositionPreferTrustedAnchorMinPriority = 40;
    opts.deepShadowDs5RefObsPositionPreferTrustedAnchorMaxDiffM = 120.0;
    opts.deepShadowDs5RefObsPositionLocalInitBlendFractions = [0 0.25 0.5 0.75 1.0];
    opts.deepShadowDs5RefObsPositionLocalCandidateCorrWeight = 0.05;
    opts.deepShadowDs5RefObsPositionLocalCandidateAbsWeight = 0.30;
    opts.deepShadowDs5RefObsPositionLocalCandidateInitBiasWeight = 0.05;
    opts.deepShadowDs5RefObsPositionLocalCandidateCorrTieM = 30.0;
    opts.deepShadowDs5RefObsPositionLocalCandidateAbsTieM = 12.0;
    opts.deepShadowDs5RefObsPositionLocalCandidateInitTieM = 6.0;
    opts.deepShadowDs5RefObsPositionSubsetCorrTieM = 60.0;
    opts.deepShadowDs5RefObsPositionSubsetAbsTieM = 20.0;
    opts.deepShadowDs5RecoveredFilterAbsAnchorRefreshSameSource = 1;
    opts.deepShadowDs5AbsAnchorRefreshOnlyS5 = 1;
    opts.deepShadowDs5RecoveredFilterAbsAnchorGateRequire = 0;
    opts.deepShadowDs5RecoveredFilterAuthorityRequireAbsAnchor = 0;
    opts.deepShadowDs5RecoveredFilterAbsAnchorRefreshOnRefObsAccept = 1;
    opts.deepShadowDs5RecoveredFilterAbsAnchorAllowRefObsFallback = 0;
    opts.deepShadowDs5RecoveredFilterAbsAnchorRefreshMaxRefObsDiffM = 350.0;
    opts.deepShadowDs5RecoveredFilterAbsAnchorRefreshSelfHeldEnable = 0;
    opts.deepShadowDs5RecoveredFilterAbsAnchorRefreshTrustedMaxDiffM = 250.0;
    opts.deepShadowDs5RecoveredFilterResetMeasPredDiffM = 700.0;
elseif strcmp(anchorMode, 'online_ins')
    opts.deepShadowTrustedAnchorUseBaselineInsAlign = 0;
    opts.deepShadowDs5RefObsPositionAbsAnchorUseHardGate = 0;
    opts.deepShadowDs5RefObsPositionAbsAnchorPriorEnable = 0;
elseif strcmp(anchorMode, 'online_prop')
    opts.deepShadowTrustedAnchorInsPropagatedUseCurrent = 0;
    opts.deepShadowTrustedAnchorUseBaselineInsAlign = 0;
    opts.deepShadowDs5RefObsPositionAbsAnchorUseHardGate = 0;
    opts.deepShadowDs5RefObsPositionAbsAnchorPriorEnable = 0;
elseif strcmp(anchorMode, 'online_base')
    opts.deepShadowTrustedAnchorUseInsPropagated = 0;
    opts.deepShadowTrustedAnchorUseBaselineInsAlign = 0;
    opts.deepShadowTrustedAnchorAllowBaselineFedAnchor = 1;
    opts.deepShadowDs5RefObsPositionAbsAnchorUseHardGate = 0;
    opts.deepShadowDs5RefObsPositionAbsAnchorPriorEnable = 0;
end

opts.deepShadowDs5RecoveredFilterAbsAnchorGateRequire = 1;
opts.deepShadowDs5RecoveredFilterAuthorityRequireAbsAnchor = 1;
opts.deepShadowDs5RecoveredFilterAbsAnchorEnable = 1;
opts.deepShadowDs5RecoveredFilterAbsAnchorFreezeSec = summaryStartSec;
opts.deepShadowDs5RecoveredFilterAbsAnchorUseInsVel = 0;
opts.deepShadowDs5RecoveredFilterAbsAnchorRefreshSameSource = 0;
opts.deepShadowDs5AbsAnchorRefreshOnlyS5 = 0;
opts.deepShadowDs5RecoveredFilterAbsAnchorAllowRefObsFallback = 0;
opts.deepShadowDs5RecoveredFilterResetOnContractMeas = 1;
opts.deepShadowDs5RecoveredFilterResetMeasPredDiffM = 1200.0;
opts.deepShadowDs5RecoveredFilterResetAtRecoveryStart = 1;
opts.deepShadowDs5RecoveredFilterConfirmEpochs = 3;
opts.deepShadowDs5RecoveredFilterAuthorityWarmStartEnable = 1;
opts.deepShadowDs5RecoveredFilterAuthorityWarmStartEpochs = 2;
opts.deepShadowDs5RecoveredFilterMeasMaxPredDiffM = 700.0;
opts.deepShadowDs5RecoveredFilterAuthorityMeasPredDiffMaxM = 1200.0;
opts.deepShadowDs5RecoveredFilterAuthorityBaselineDiffMaxM = 5000.0;
opts.deepShadowDs5RecoveredFilterAuthorityAbsAnchorMaxM = 1200.0;
opts.deepShadowDs5RecoveredFilterAbsAnchorSoftMaxDiffM = 450.0;
opts.deepShadowDs5RecoveredFilterAbsAnchorMaxDiffM = 1200.0;
opts.deepShadowDs5RecoveredFilterAbsAnchorBlend = 0.08;
opts.deepShadowDs5RecoveredFilterAbsAnchorBlendHigh = 0.25;
opts.deepShadowDs5RefObsPositionRobustKeepSatBonusM = 18.0;
opts.deepShadowDs5RecoveredFilterMaxBadEpochs = 80;
opts.deepShadowDs5RecoveredFilterMaxCoastEpochs = 500;
opts.deepShadowDs5RecoveredFilterAuthorityMaxCoastEpochs = 500;

if strcmp(anchorMode, 'online_loose')
    opts.deepShadowTrustedAnchorRefreshSameSource = 1;
    opts.deepShadowTrustedAnchorRefreshOnlyS5 = 1;
    opts.deepShadowDs5RecoveredFilterAbsAnchorRefreshSameSource = 1;
    opts.deepShadowDs5AbsAnchorRefreshOnlyS5 = 1;
end

opts.deepShadowDs5FinalOutputEnable = 1;
opts.deepShadowDs5FinalRecoveredAuthorityEnable = 1;
opts.deepShadowDs5FinalRecoveredAuthorityRequireBaselineGate = 1;
opts.deepShadowDs5FinalQualityBaselineDiffMaxM = 5000.0;
opts.deepShadowDs5FinalRecoveredAuthorityMaxBaselineDiffM = opts.deepShadowDs5FinalQualityBaselineDiffMaxM;
opts.deepShadowDs5FinalAllowNoBaselineRecovered = 0;
opts.deepShadowDs5FinalBaselineOffAblation = 0;
opts.deepShadowDs5FinalOutputHoldEpochs = 500;
opts.deepShadowDs5FinalRequireObsContractForRecovered = 1;
opts.deepShadowDs5FinalRequireObsContractForHold = 1;
opts.deepShadowDs5FinalAllowObsContractTrackingHold = 1;
opts.deepShadowDs5FinalAllowObsContractBaseSeedHold = 0;
opts.deepShadowDs5FinalContinuityMaxPredDiffM = 5000.0;
opts.deepShadowDs5FinalHoldBaselineDiffMaxM = 5000.0;
opts.deepShadowDs5FinalHoldBaselineForceDiffM = 5000.0;
opts.deepShadowDs5FinalHoldBaselineMaxAgeEpochs = 500;
opts.deepShadowDs5FinalRecoveredHoldAllowObsGap = 1;
opts.deepShadowDs5FinalRecoveredHoldAllowAuthorityGap = 1;
opts.deepShadowDs5FinalRecoveredHoldSuppressBaselineCompete = 1;
opts.deepShadowDs5FinalRecoveredHoldBaselineDiffMaxM = 5000.0;

fprintf('\n=== DS5 heap-safe true-tracking long-window run ===\n');
fprintf('Start: %s\n', char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss')));
fprintf('True-reference start: %.1f s\n', trueRefStartSec);
fprintf('Run end: %.1f s\n', runEndSec);
fprintf('Summary start: %.1f s\n', summaryStartSec);
fprintf('Anchor mode: %s\n', anchorMode);
fprintf('Raw stride epochs: %d\n', rawStrideEpochs);
fprintf('Raw burst ms: %.1f\n', rawBurstMs);
fprintf('Result file: %s\n', resultFile);
fprintf('Log file: %s\n\n', logFile);

tic;
run_goal2_deep_400s_batch(opts);
elapsedSec = toc;
fprintf('\nElapsed = %.1f s (%.2f h)\n', elapsedSec, elapsedSec / 3600);

if exist(resultFile, 'file') ~= 2
    candidates = dir([outPrefix '_ds5_*s.mat']);
    if isempty(candidates)
        error('Expected result file was not created: %s', resultFile);
    end
    [~, newestIdx] = max([candidates.datenum]);
    resultFile = candidates(newestIdx).name;
    fprintf('Expected exact %.0fs file was not created; using actual result file: %s\n', runEndSec, resultFile);
end

printDs5ObsContractSummary(resultFile, summaryStartSec);
localPrintGoalCoverage(resultFile);

fidDone = fopen(doneFile, 'w');
if fidDone >= 0
    fprintf(fidDone, 'summaryComplete=1\n');
    fprintf(fidDone, 'anchorMode=%s\n', anchorMode);
    fprintf(fidDone, 'resultFile=%s\n', resultFile);
    fprintf(fidDone, 'logFile=%s\n', logFile);
    fprintf(fidDone, 'completedAt=%s\n', char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss')));
    fclose(fidDone);
end
diary('off');
clear opts S R;

function localPrepareMatlabHeapSafeRuntime()
% Keep batch execution conservative to reduce MATLAB-side instability.
set(0, 'DefaultFigureVisible', 'off');
try
    close all force hidden;
catch
end
try
    pool = gcp('nocreate');
    if ~isempty(pool)
        delete(pool);
    end
catch
end
try
    maxNumCompThreads(1);
catch
end
try
    opengl('software');
catch
end
fprintf('MATLAB_PREFDIR=%s\n', prefdir);
end

function localCleanupMatlabHeapSafeRuntime()
% Release process-scoped resources conservatively before MATLAB shutdown.
try
    diary('off');
catch
end
try
    fclose('all');
catch
end
try
    delete(timerfindall);
catch
end
try
    pool = gcp('nocreate');
    if ~isempty(pool)
        delete(pool);
    end
catch
end
try
    close all force hidden;
catch
end
try
    drawnow limitrate nocallbacks;
catch
end
try
    pause(0.05);
catch
end
end

function localPrintGoalCoverage(resultFile)
S = load(resultFile, 'navResults');
R = S.navResults;
fprintf('\n--- Goal-chain field coverage ---\n');
localPrintCoverage(R, 'shadowDs5TrueRefHz');
localPrintCoverage(R, 'shadowDs5TrueRefCodeChips');
localPrintCoverage(R, 'shadowRawTrackCarrShadowTargetHz');
localPrintCoverage(R, 'shadowRawTrackCodeRefNcoPullHz');
localPrintCoverage(R, 'shadowDs5RefObsTrackRawSampled');
localPrintCoverage(R, 'shadowDs5RefObsTrackCoasted');
localPrintCoverage(R, 'shadowDs5ObsContractStrictRawIndependentPass');
localPrintCoverage(R, 'shadowDs5ObsContractStrictRawIndependentSatNum');
localPrintCoverage(R, 'shadowRawTrackPromptIP');
localPrintCoverage(R, 'shadowRawTrackDllDiscr');
localPrintCoverage(R, 'shadowRecoveredFilterX');
localPrintCoverage(R, 'shadowFinalOutputX');
end

function localPrintCoverage(R, fieldName)
if ~isfield(R, fieldName)
    fprintf('%s: missing\n', fieldName);
    return;
end
x = R.(fieldName);
fprintf('%s: finite %d/%d\n', fieldName, nnz(isfinite(x(:))), numel(x));
end
