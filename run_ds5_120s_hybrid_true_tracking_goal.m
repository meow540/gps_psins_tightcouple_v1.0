clear;
clc;

projectRoot = fileparts(mfilename('fullpath'));
cd(projectRoot);
addpath('deepIntegration');

outPrefix = 'rt_deep_goal2_ds5_hybrid_true_tracking_goal_120s';
resultFile = [outPrefix '_ds5_120s.mat'];
logFile = [outPrefix '_run.log'];

if exist(logFile, 'file') == 2
    delete(logFile);
end
diary(logFile);
cleanupObj = onCleanup(@() diary('off')); %#ok<NASGU>

opts = make_goal2_recoveryboost_120s_opts(outPrefix, {'ds5'});
opts.datasets = {'ds5'};
opts.roundTime = 240;          % 120 s at 0.5 s nav period
opts.calibEndEpoch = 180;
opts.deepPerfDiagInterval = 20;

% Hybrid runtime: use precomputed main tracking before/around spoof, but
% keep true-reference shadow tracking on the actual raw IF samples.
opts.deepFastMode = 1;
opts.deepShadowDs5FastRefObsTrackBypass = 0;
opts.deepShadowDs5RefObsTrackRawStrideEpochs = 4;   % 0.5 s nav period -> raw sample every 2 s
opts.deepShadowDs5RefObsTrackRawBurstMs = 20;       % per-satellite raw IF tracking budget per sampled epoch
opts.deepShadowDs5RefObsTrackRawWarmupEpochs = 3;

opts.deepUseGnssKfUpdate = 0;
opts.deepUseGnssKfUpdateNormal = 0;
opts.deepUseGnssKfUpdateSuspect = 0;
opts.deepUseGnssKfUpdateSpoof = 0;
opts.deepUseKfClockUpdate = 0;
opts.deepUseKfClockUpdateNormal = 0;
opts.deepUseKfClockUpdateSuspect = 0;
opts.deepUseKfClockUpdateSpoof = 0;

opts.deepShadowDs5ObsContractEnable = 1;
opts.deepShadowDs5ObsContractMinSat = 4;
opts.deepShadowDs5ObsContractSpreadP95MaxM = 800.0;
opts.deepShadowDs5ObsContractRequireCommonClock = 1;
opts.deepShadowDs5ObsContractRequireTrackingPass = 1;
opts.deepShadowDs5ObsContractMaxBaseSeedFrac = 0.75;
opts.deepShadowDs5ObsContractNoBaselineSeedForRecovery = 0;
opts.deepShadowDs5ObsContractUseRecoveryKeep = 0;
opts.deepShadowDs5ObsContractRobustTrackingEnable = 0;
opts.deepShadowDs5ObsContractRobustTrackingMaxDropSat = 2;
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

opts.deepShadowDs5RefObsModelEnable = 1;
opts.deepShadowDs5RefObsModelStartSec = 92.0;
opts.deepShadowDs5RefObsAutoActivate = 1;
opts.deepShadowDs5TruePeakEnable = 1;
opts.deepShadowDs5TruePeakUseCodeRef = 1;
opts.deepShadowDs5CodeRefNcoPullEnable = 1;
opts.deepShadowDs5CodeRefNcoGain = 1.0;
opts.deepShadowDs5CodeRefNcoMaxHz = 120.0;

opts.deepShadowDs5RecoveredFilterAbsAnchorGateRequire = 1;
opts.deepShadowDs5RecoveredFilterAuthorityRequireAbsAnchor = 1;
opts.deepShadowDs5RecoveredFilterAbsAnchorEnable = 1;
opts.deepShadowDs5RecoveredFilterResetOnContractMeas = 1;
opts.deepShadowDs5RecoveredFilterResetMeasPredDiffM = 1500.0;
opts.deepShadowDs5RecoveredFilterConfirmEpochs = 3;
opts.deepShadowDs5RecoveredFilterMeasMaxPredDiffM = 1200.0;
opts.deepShadowDs5RecoveredFilterAuthorityMeasPredDiffMaxM = 1800.0;
opts.deepShadowDs5RecoveredFilterAuthorityBaselineDiffMaxM = 1800.0;
opts.deepShadowDs5RecoveredFilterMaxBadEpochs = 12;

opts.deepShadowDs5FinalOutputEnable = 1;
opts.deepShadowDs5FinalRecoveredAuthorityEnable = 1;
opts.deepShadowDs5FinalRecoveredAuthorityRequireBaselineGate = 1;
opts.deepShadowDs5FinalQualityBaselineDiffMaxM = 1800.0;
opts.deepShadowDs5FinalRecoveredAuthorityMaxBaselineDiffM = opts.deepShadowDs5FinalQualityBaselineDiffMaxM;
opts.deepShadowDs5FinalAllowNoBaselineRecovered = 0;
opts.deepShadowDs5FinalBaselineOffAblation = 0;
opts.deepShadowDs5FinalRequireObsContractForRecovered = 1;
opts.deepShadowDs5FinalRequireObsContractForHold = 1;
opts.deepShadowDs5FinalAllowObsContractTrackingHold = 1;
opts.deepShadowDs5FinalAllowObsContractBaseSeedHold = 1;
opts.deepShadowDs5FinalContinuityMaxPredDiffM = 900.0;
opts.deepShadowDs5FinalHoldBaselineDiffMaxM = 1800.0;
opts.deepShadowDs5FinalHoldBaselineForceDiffM = 1800.0;
opts.deepShadowDs5FinalHoldBaselineMaxAgeEpochs = 40;
opts.deepShadowDs5FinalRecoveredHoldAllowObsGap = 1;
opts.deepShadowDs5FinalRecoveredHoldAllowAuthorityGap = 1;
opts.deepShadowDs5FinalRecoveredHoldSuppressBaselineCompete = 1;
opts.deepShadowDs5FinalRecoveredHoldBaselineDiffMaxM = 2200.0;

fprintf('\n=== DS5 120s hybrid true-tracking smoke run ===\n');
fprintf('Start: %s\n', char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss')));
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
    fprintf('Expected exact 120s file was not created; using actual result file: %s\n', resultFile);
end

printDs5ObsContractSummary(resultFile, 92);
localPrintGoalCoverage(resultFile);

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
