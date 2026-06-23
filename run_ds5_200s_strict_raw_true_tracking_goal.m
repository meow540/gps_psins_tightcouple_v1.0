clear;
clc;

projectRoot = fileparts(mfilename('fullpath'));
cd(projectRoot);
addpath('deepIntegration');

strictRawStartSec = 176.0;
summaryStartSec = 180.0;
strictRawEndSec = 200.0;
endSecEnv = str2double(getenv('DS5_STRICT_RAW_END_SEC'));
if isfinite(endSecEnv) && endSecEnv > summaryStartSec
    strictRawEndSec = endSecEnv;
end

outPrefix = sprintf('rt_deep_goal2_ds5_strict_raw_true_tracking_goal_%03.0fs', round(strictRawEndSec));
resultFile = sprintf('%s_ds5_%03.0fs.mat', outPrefix, round(strictRawEndSec));
logFile = [outPrefix '_run.log'];
doneFile = [outPrefix '_summary_done.txt'];

if exist(logFile, 'file') == 2
    delete(logFile);
end
if exist(doneFile, 'file') == 2
    delete(doneFile);
end
diary(logFile);
cleanupObj = onCleanup(@() diary('off'));

opts = make_goal2_recoveryboost_120s_opts(outPrefix, {'ds5'});
opts.datasets = {'ds5'};
opts.roundTime = round(strictRawEndSec / 0.5);  % 0.5 s nav period
opts.calibEndEpoch = 180;
opts.deepPerfDiagInterval = 10;

% Strict short-window runtime: use precomputed main tracking, but run
% true-reference shadow tracking continuously on raw IF only near 180 s.
opts.deepFastMode = 1;
opts.deepShadowDs5FastRefObsTrackBypass = 0;
opts.deepShadowDs5RefObsTrackRawStrideEpochs = 1;
opts.deepShadowDs5RefObsTrackRawBurstMs = inf;
opts.deepShadowDs5RefObsTrackRawWarmupEpochs = 0;
opts.deepShadowRawAmbiguityEnable = 1;
opts.deepShadowRawAmbiguityMaxChips = 120000.0;
opts.deepShadowRawAmbiguityStepMaxChips = 120000.0;
opts.deepShadowRawAmbiguityMinSat = 4;

% Keep the smoke test focused on the DS5 true-reference path. The recovery
% preset enables generic shadow raw reacquisition/tracking from spoof onset
% (~91 s), which is expensive and has triggered MATLAB native heap crashes
% before the strict window. The DS5 refObs path below can self-activate from
% trackDeepIn at strictRawStartSec, so generic shadow raw machinery is off.
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

% Delay DS5 true-reference/raw-observation machinery so full raw work is
% limited to the short validation window.
opts.deepShadowDs5RefObsModelEnable = 1;
opts.deepShadowDs5RefObsModelStartSec = strictRawStartSec;
opts.deepShadowDs5RefObsRecoveryStartSec = summaryStartSec;
opts.deepShadowDs5TruePeakEnable = 1;
opts.deepShadowDs5TruePeakStartSec = strictRawStartSec;
opts.deepShadowDs5TruePeakUseCodeRef = 1;
opts.deepShadowDs5CommonDragStartSec = strictRawStartSec;
opts.deepShadowDs5CommonDoppStartSec = strictRawStartSec;
opts.deepShadowDs5SoftRecoveryStartSec = strictRawStartSec;
opts.deepShadowDs5AuthorityStartSec = strictRawStartSec;
opts.deepShadowDs5KfSourceGateStartSec = strictRawStartSec;
opts.deepShadowDs5ClosedLiftPreferStartSec = strictRawStartSec;
opts.deepShadowDs5TailSourceFallbackStartSec = strictRawStartSec;
opts.deepShadowDs5TailNavRelaxStartSec = strictRawStartSec;
opts.deepShadowDs5SoftClampStartSec = strictRawStartSec;
opts.deepShadowDs5DetrendedGateStartSec = strictRawStartSec;
opts.deepShadowDs5BaselineTrustedRefMaxSec = strictRawStartSec - 0.5;

opts.deepShadowDs5RefObsAutoActivate = 1;
opts.deepShadowDs5RefObsRequireActiveTrack = 1;
opts.deepShadowDs5RefObsSeedUseBaseFallback = 0;
opts.deepShadowDs5RefObsSeedMaxDevFromBaseM = 1500.0;
opts.deepShadowDs5RefObsLocalCorrMaxChips = 1.50;
opts.deepShadowDs5RefObsUseCodeCorr = 1;

opts.deepShadowDs5CodeRefNcoPullEnable = 1;
opts.deepShadowDs5CodeRefNcoGain = 1.0;
opts.deepShadowDs5CodeRefNcoMaxHz = 120.0;

% Strict raw-independent contract: no base-seed fallback is accepted.
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
opts.deepShadowDs5RefObsPositionCommonClockRebasePeriodM = 299792.458;
opts.deepShadowDs5RefObsPositionRawAnchorLiftEnable = 1;
opts.deepShadowDs5RefObsPositionRawAnchorLiftMaxSpreadP95M = 800.0;
opts.deepShadowDs5RefObsPositionRawAnchorLiftUseCodeCorr = 1;
opts.deepShadowDs5RefObsPositionCorrMaxM = 900.0;
opts.deepShadowDs5RefObsPositionAnchorDiffMaxM = 900.0;
opts.deepShadowDs5RefObsPositionAdaptiveCorrMaxM = 900.0;
opts.deepShadowDs5RefObsPositionAdaptiveAnchorDiffMaxM = 900.0;
opts.deepShadowDs5RefObsPositionJumpMaxM = 900.0;
opts.deepShadowDs5RefObsPositionAdaptiveJumpMaxM = 900.0;
opts.deepShadowDs5RefObsPositionSpeedGateEnable = 0;

opts.deepShadowDs5RecoveredFilterAbsAnchorGateRequire = 1;
opts.deepShadowDs5RecoveredFilterAuthorityRequireAbsAnchor = 1;
opts.deepShadowDs5RecoveredFilterAbsAnchorEnable = 1;
opts.deepShadowDs5RecoveredFilterResetOnContractMeas = 1;
opts.deepShadowDs5RecoveredFilterResetMeasPredDiffM = 1500.0;
opts.deepShadowDs5RecoveredFilterConfirmEpochs = 3;
opts.deepShadowDs5RecoveredFilterMeasMaxPredDiffM = 1200.0;
opts.deepShadowDs5RecoveredFilterAuthorityMeasPredDiffMaxM = 1800.0;
opts.deepShadowDs5RecoveredFilterAuthorityBaselineDiffMaxM = 5000.0;
opts.deepShadowDs5RecoveredFilterAuthorityAbsAnchorMaxM = 3500.0;
opts.deepShadowDs5RecoveredFilterAbsAnchorMaxDiffM = 3500.0;
opts.deepShadowDs5RecoveredFilterMaxBadEpochs = 20;
opts.deepShadowDs5RecoveredFilterMaxCoastEpochs = 20;
opts.deepShadowDs5RecoveredFilterAuthorityMaxCoastEpochs = 6;

opts.deepShadowDs5FinalOutputEnable = 1;
opts.deepShadowDs5FinalRecoveredAuthorityEnable = 1;
opts.deepShadowDs5FinalRecoveredAuthorityRequireBaselineGate = 1;
opts.deepShadowDs5FinalQualityBaselineDiffMaxM = 5000.0;
opts.deepShadowDs5FinalRecoveredAuthorityMaxBaselineDiffM = opts.deepShadowDs5FinalQualityBaselineDiffMaxM;
opts.deepShadowDs5FinalAllowNoBaselineRecovered = 0;
opts.deepShadowDs5FinalBaselineOffAblation = 0;
opts.deepShadowDs5FinalOutputHoldEpochs = 12;
opts.deepShadowDs5FinalRequireObsContractForRecovered = 1;
opts.deepShadowDs5FinalRequireObsContractForHold = 1;
opts.deepShadowDs5FinalAllowObsContractTrackingHold = 0;
opts.deepShadowDs5FinalAllowObsContractBaseSeedHold = 0;
opts.deepShadowDs5FinalContinuityMaxPredDiffM = 900.0;
opts.deepShadowDs5FinalHoldBaselineDiffMaxM = 1800.0;
opts.deepShadowDs5FinalHoldBaselineForceDiffM = 1800.0;
opts.deepShadowDs5FinalHoldBaselineMaxAgeEpochs = 12;
opts.deepShadowDs5FinalRecoveredHoldAllowObsGap = 0;
opts.deepShadowDs5FinalRecoveredHoldAllowAuthorityGap = 0;
opts.deepShadowDs5FinalRecoveredHoldSuppressBaselineCompete = 1;
opts.deepShadowDs5FinalRecoveredHoldBaselineDiffMaxM = 2200.0;

fprintf('\n=== DS5 200s strict raw true-tracking short-window run ===\n');
fprintf('Start: %s\n', char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss')));
fprintf('Strict raw start: %.1f s\n', strictRawStartSec);
fprintf('Run end: %.1f s\n', strictRawEndSec);
fprintf('Summary start: %.1f s\n', summaryStartSec);
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
    fprintf('Expected exact %.0fs file was not created; using actual result file: %s\n', strictRawEndSec, resultFile);
end

printDs5ObsContractSummary(resultFile, summaryStartSec);
localPrintGoalCoverage(resultFile);
diary('off');
fidDone = fopen(doneFile, 'w');
if fidDone >= 0
    fprintf(fidDone, 'summaryComplete=1\n');
    fprintf(fidDone, 'resultFile=%s\n', resultFile);
    fprintf(fidDone, 'logFile=%s\n', logFile);
    fprintf(fidDone, 'completedAt=%s\n', char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss')));
    fclose(fidDone);
end
fclose('all');
quit force;

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
