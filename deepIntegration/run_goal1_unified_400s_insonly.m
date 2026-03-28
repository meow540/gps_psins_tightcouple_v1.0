function summary = run_goal1_unified_400s_insonly(datasetTag, profile, opts)
% Goal-1 unified spoof decision parameters for ds5/ds6 (400 s, weak-assisted INS takeover).
%
% Usage:
%   run_goal1_unified_400s_insonly()                 % ds5 + ins1
%   run_goal1_unified_400s_insonly('ds5','ins2')
%   run_goal1_unified_400s_insonly('ds6','ins1')
%   run_goal1_unified_400s_insonly('ds6','ins1', struct('useOfflineReplay',1))
%
if nargin < 1 || isempty(datasetTag)
    datasetTag = 'ds5';
end
if nargin < 2 || isempty(profile)
    profile = 'ins1';
end
if nargin < 3 || isempty(opts)
    opts = struct();
end
datasetTag = lower(string(datasetTag));
profile = lower(string(profile));

if ~isfield(opts, 'useOfflineReplay'), opts.useOfflineReplay = 0; end
if ~isfield(opts, 'msToProcess'), opts.msToProcess = 399 * 1000; end
if ~isfield(opts, 'rawFileName'), opts.rawFileName = ""; end
if ~isfield(opts, 'extraSettings'), opts.extraSettings = struct(); end
if ~isfield(opts, 'baselineOpts'), opts.baselineOpts = struct(); end
if ~isfield(opts, 'cleanRefAutoAlignEnable'), opts.cleanRefAutoAlignEnable = 1; end
if ~isfield(opts, 'cleanRefAutoAlignMaxShiftSec'), opts.cleanRefAutoAlignMaxShiftSec = 10; end

if datasetTag ~= "ds5" && datasetTag ~= "ds6" && datasetTag ~= "clean"
    error('datasetTag must be ''ds5'', ''ds6'', or ''clean''.');
end
if profile ~= "ins1" && profile ~= "ins2" && profile ~= "ins3"
    error('profile must be ''ins1'', ''ins2'', or ''ins3''.');
end

thisDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(thisDir);
addpath(fullfile(projectRoot, 'include'))
addpath(fullfile(projectRoot, 'geoFunctions'))
addpath(fullfile(projectRoot, 'acquire_zcj'))
addpath(fullfile(projectRoot, 'track_zcj'))
addpath(fullfile(projectRoot, 'PVT'))
addpath(thisDir)

if datasetTag == "clean"
    dataFile = fullfile(projectRoot, 'cleandynamic_400s.mat');
else
    dataFile = fullfile(projectRoot, sprintf('%s_400s.mat', datasetTag));
end
if ~exist(dataFile, 'file')
    error('Missing dataset: %s', dataFile);
end
outFile = localBuildOutputFile(projectRoot, datasetTag, profile, opts.msToProcess);

S = load(dataFile, 'trackResults', 'channel', 'TOW', 'eph', 'subFrameStart', 'settings', 'navSolutions');
trackResults = S.trackResults; %#ok<NASGU>
channel = S.channel; %#ok<NASGU>
TOW = S.TOW; %#ok<NASGU>
eph = S.eph; %#ok<NASGU>
subFrameStart = S.subFrameStart; %#ok<NASGU>
navSolPeriodSec = initSettings().navSolPeriod / 1000;
datasetSettings = [];
if isfield(S, 'settings')
    datasetSettings = S.settings;
end
trjStartOffsetSec = localChooseTrajectoryOffsetSec(datasetTag, opts, trackResults, subFrameStart, datasetSettings);
referenceEpochOffset = 0;

rawFileName = string(opts.rawFileName);
if strlength(rawFileName) == 0
    tmp = load(dataFile, 'settings');
    if isfield(tmp, 'settings') && isfield(tmp.settings, 'fileName')
        rawFileName = string(tmp.settings.fileName);
    end
end
if ~opts.useOfflineReplay && (strlength(rawFileName) == 0 || exist(char(rawFileName), 'file') ~= 2)
    error('Raw IF file is required for non-offline run but not found: %s', char(rawFileName));
end

if datasetTag == "ds6"
    eph = rebuildEphFromTrackResults(trackResults, channel, subFrameStart, eph, dataFile); %#ok<NASGU>
end

switch profile
    case "ins1"
        trjFile = "E:\\fgi_result\\ins_simulation\\ins1_trj.mat";
        imuProfile = 'ins1';
    case "ins2"
        trjFile = "E:\\fgi_result\\ins_simulation\\ins2_trj.mat";
        imuProfile = 'ins2';
    case "ins3"
        trjFile = "E:\\fgi_result\\ins_simulation\\ins3_trj.mat";
        imuProfile = 'ins3';
end

baselineOpts = struct( ...
    'useOfflineReplay', 1, ...
    'calibStartSec', 20, ...
    'calibEndSec', localChooseCalibEndSec(datasetTag), ...
    'commonSigmaScale', 4, ...
    'diffSigmaScale', 3, ...
    'satSigmaScale', 1.5, ...
    'pllNoiseBandwidth', 15, ...
    'minElevDeg', 20, ...
    'minSat', 4, ...
    'minHitSat', 4, ...
    'confirmEpochs', 2, ...
    'templateSmoothWin', 5, ...
    'armTimeSec', localChooseArmTimeSec(datasetTag), ...
    'msToProcess', opts.msToProcess);
if isstruct(opts.baselineOpts)
    fnBase = fieldnames(opts.baselineOpts);
    for iBase = 1:numel(fnBase)
        baselineOpts.(fnBase{iBase}) = opts.baselineOpts.(fnBase{iBase});
    end
end
baseline = build_goal1_clean_baseline(profile, baselineOpts);

settingsOverride = struct(); %#ok<NASGU>
if opts.useOfflineReplay
    settingsOverride.fileName = "";
else
    settingsOverride.fileName = rawFileName;
end
settingsOverride.offlineReplayFromTrackResults = opts.useOfflineReplay;
settingsOverride.msToProcess = opts.msToProcess;
settingsOverride.plotTracking = 0;
settingsOverride.skipQuickPlot = 1;
settingsOverride.verboseEpochPrint = 0;
settingsOverride.trjFile = trjFile;
settingsOverride.imuerrProfile = imuProfile;
settingsOverride.pllNoiseBandwidth = 15;
settingsOverride.trjStartOffsetSec = trjStartOffsetSec;
settingsOverride.spoofDetTemplateEpochOffset = 0;
% Chapter-1 requirement: the tight-coupled state must start from a single
% self-consistent AVP source. Do not splice dataset GNSS position/velocity
% onto the IMU trajectory attitude, otherwise the filter starts from a
% physically inconsistent state before spoof detection even begins.
settingsOverride.initAvpUseDatasetGnss = 0;

% Goal-1 mainline: clean-baseline Doppler detector + weak-assisted INS takeover.
settingsOverride.spoofDetArmTimeSec = baseline.recommendedArmTimeSec;
settingsOverride.spoofDetPostArmGraceSec = 0;
settingsOverride.spoofDetConfirmEpochs = baseline.N_confirm;
settingsOverride.spoofDetCommonThresholdHz = baseline.T_cm;
settingsOverride.spoofDetDiffThresholdHz = baseline.T_df;
settingsOverride.spoofDetSatThresholdHz = baseline.T_sat;
settingsOverride.spoofDetMinSat = baseline.minSat;
settingsOverride.spoofDetMinHitSat = baseline.N_hit;
settingsOverride.spoofDetMinElevDeg = baseline.minElevDeg;
settingsOverride.spoofDetMetricSmoothWin = baseline.metricSmoothWin;
settingsOverride.spoofDetBaselinePrnList = baseline.prnList;
settingsOverride.spoofDetSatBiasHz = baseline.satBiasHz;
settingsOverride.spoofDetTemplatePrnList = baseline.prnList;
settingsOverride.spoofDetTemplateResidualHz = baseline.templateResidualHz;
settingsOverride.leverArm_b = baseline.leverArm_b;

settingsOverride.spoofMitigationMode = 'ins_only';
settingsOverride.gnssTakeoverMode = 'ins_only_latched';
settingsOverride.virtualNHCEnable = 0;
settingsOverride.virtualZUPTEnable = 1;
settingsOverride.virtualZUPTSpeedTh = 0.2;
settingsOverride.virtualZUPTNoiseStdMps = [0.03; 0.03; 0.03];
settingsOverride.virtualSpeedHoldEnable = 0;
settingsOverride.virtualHeightHoldEnable = 1;
settingsOverride.virtualHeightHoldNoiseStdM = 12;
settingsOverride.virtualPitchRollHoldEnable = 0;
settingsOverride.virtualPitchRollHoldNoiseStdRad = [0.5; 0.5] * pi / 180;
settingsOverride.virtualVertVelHoldEnable = 0;
settingsOverride.virtualVertVelHoldNoiseStdMps = 0.10;
settingsOverride.tcUseRangeRateUpdate = 1;
settingsOverride.tcRangeNoiseStdM = 2.0;
settingsOverride.tcRangeRateNoiseStdMps = 0.10;
settingsOverride.insTakeoverPrrAssistEnable = 1;
settingsOverride.insTakeoverPrrAssistPolicy = 'always';
settingsOverride.insTakeoverPrrAssistMcmOverDiffMarginHz = inf;
settingsOverride.insTakeoverPrrAssistMinDiffHz = 0.0;
settingsOverride.insTakeoverPrrResidualClipHz = 10.0;
% In Chapter-1 weak takeover, suppressed Doppler is the only post-alarm
% dynamic aid, so PRR weighting must stay tight enough to constrain drift.
settingsOverride.insTakeoverPrrNoiseStdMps = localChooseTakeoverPrrNoiseStd(datasetTag, profile);
settingsOverride.insTakeoverPrrOutlierRejectEnable = 1;
settingsOverride.insTakeoverPrrOutlierMadScale = 4.0;
settingsOverride.insTakeoverPrrOutlierAbsMps = inf;
settingsOverride.insTakeoverPrrOutlierMinMadMps = 0.03;
settingsOverride.insTakeoverPrrOutlierMaxIter = 1;
if datasetTag == "ds5"
    % DS5 is clock-drag dominant. Keep Chapter-1 takeover as pure INS to
    % avoid spoofed Doppler residue re-pulling the filter.
    settingsOverride.spoofAlarmAction = 'ins_takeover';
    settingsOverride.insTakeoverPrrAssistEnable = 0;
    settingsOverride.insTakeoverPrrAssistPolicy = 'off';
    % Push DS5 alarm closer to the known spoof onset neighborhood
    % (~100 s) while avoiding early pre-arm false trigger.
    settingsOverride.spoofDetCommonThresholdHz = 4.2;
    settingsOverride.spoofDetDiffThresholdHz = 4.6;
    settingsOverride.spoofDetSatThresholdHz = 5.2;
    settingsOverride.spoofDetMinHitSat = 3;
    if profile == "ins2"
        settingsOverride.spoofAlarmAction = 'clock_hold';
        % DS5 mid-track mismatch is mainly driven by PRR common-mode pull
        % during clock spoof. Keep clock_hold, but downweight updates less
        % aggressively and explicitly remove PRR common component.
        settingsOverride.clockSpoofPrInflate = 1.0;
        settingsOverride.clockSpoofPrrInflate = 1.0;
        settingsOverride.clockSpoofRemoveCommonPrr = 1;
        settingsOverride.clockSpoofPrInnovationClipM = inf;
        % During clock-hold, lightly constrain vertical channel to avoid
        % transient altitude blow-up while still allowing smooth motion.
        settingsOverride.virtualVertVelHoldEnable = 1;
        settingsOverride.virtualVertVelHoldNoiseStdMps = 0.12;
        settingsOverride.clockHoldPlanarClampEnable = 1;
        settingsOverride.clockHoldPlanarClampCommonHz = 14.0;
        settingsOverride.clockHoldPlanarClampDiffHz = 22.0;
        % Keep detector arming at clean-baseline recommendation instead of
        % forcing a delayed arm, to avoid long pre-alarm contamination in ds5.
    end
elseif datasetTag == "ds6"
    % DS6 keeps PRR assist only when differential residual is sufficiently
    % strong; common-mode dominant epochs are blocked.
    settingsOverride.spoofAlarmAction = 'ins_takeover';
    settingsOverride.insTakeoverPrrAssistEnable = 1;
    settingsOverride.insTakeoverPrrAssistPolicy = 'conditional';
    settingsOverride.insTakeoverPrrAssistMcmOverDiffMarginHz = 2.0;
    settingsOverride.insTakeoverPrrAssistMinDiffHz = 1.8;
    settingsOverride.insTakeoverPrrResidualClipHz = 4.0;
    settingsOverride.insTakeoverPrrNoiseStdMps = 0.25;
    if profile == "ins2"
        % INS2 tuning: keep takeover logic unchanged but tighten PRR assist
        % to reduce long-tail drift in DS6.
        settingsOverride.spoofDetArmTimeSec = 88.0;
        settingsOverride.spoofDetCommonThresholdHz = 4.2;
        settingsOverride.spoofDetDiffThresholdHz = 4.6;
        settingsOverride.spoofDetSatThresholdHz = 6.2;
        settingsOverride.insTakeoverPrrAssistPolicy = 'always';
        settingsOverride.insTakeoverPrrResidualClipHz = 10.0;
        settingsOverride.insTakeoverPrrNoiseStdMps = 0.12;
        settingsOverride.insTakeoverPrrOutlierMadScale = 3.2;
        settingsOverride.insTakeoverPrrOutlierAbsMps = 0.90;
        settingsOverride.insTakeoverPrrOutlierMinMadMps = 0.05;
        settingsOverride.insTakeoverPrrOutlierMaxIter = 2;
        % Smooth per-satellite differential residual during takeover.
        % This keeps DS6 mid-segment swings smaller while preserving detection.
        settingsOverride.insTakeoverResidualIirAlpha = 0.25;
        % Gradually fade PRR assist in late takeover (no hard switch),
        % so DS6 keeps a smaller mid-segment error but shows INS-like
        % growth trend toward the end.
        settingsOverride.insTakeoverPrrAssistFadeStartSec = 220.0;
        settingsOverride.insTakeoverPrrAssistFadeDurationSec = 120.0;
        settingsOverride.virtualZUPTSpeedTh = 0.30;
    end
end
settingsOverride.gnssRecoveryEnable = 1;
settingsOverride.spoofReleaseCommonScale = 0.35;
settingsOverride.spoofReleaseDiffScale = 0.35;
settingsOverride.spoofReleaseSatScale = 0.35;
settingsOverride.spoofReleaseMinHitSat = 0;
settingsOverride.spoofReleaseConfirmEpochs = 20;
settingsOverride.spoofRecoveryMinLockSec = 20;
settingsOverride.gnssRampDurationSec = 10;
settingsOverride.gnssRampPrScaleStart = 10;
settingsOverride.gnssRampPrrScaleStart = 5;
settingsOverride.odoEnable = 0;

if isstruct(opts.extraSettings)
    fns = fieldnames(opts.extraSettings);
    for iField = 1:numel(fns)
        settingsOverride.(fns{iField}) = opts.extraSettings.(fns{iField});
    end
end

realtime_tightCouple;

summary = struct();
summary.datasetTag = char(datasetTag);
summary.profile = char(profile);
summary.useOfflineReplay = logical(opts.useOfflineReplay);
summary.rawFileName = char(rawFileName);
summary.dataset = dataFile;
summary.output = outFile;
summary.trjStartOffsetSec = trjStartOffsetSec;
summary.referenceTimeOffsetSec = trjStartOffsetSec;
summary.referenceEpochOffset = referenceEpochOffset;
summary.baselineThresholdCommonHz = baseline.T_cm;
summary.baselineThresholdDiffHz = baseline.T_df;
summary.baselineThresholdSatHz = baseline.T_sat;
summary.baselineMedianCommonHz = baseline.commonMedianHz;
summary.baselineMedianDiffHz = baseline.diffMedianHz;
summary.baselineSigmaCommonHz = baseline.commonSigmaHz;
summary.baselineSigmaDiffHz = baseline.diffSigmaHz;
summary.armTimeSec = baseline.recommendedArmTimeSec;
summary.epochs = numel(navResults.X);
summary.finiteXYZ = sum(isfinite(navResults.X) & isfinite(navResults.Y) & isfinite(navResults.Z));
summary.alarms = sum(navResults.spoofAlarm);
alarmIdx = find(navResults.spoofAlarm, 1, 'first');
if isempty(alarmIdx)
    summary.firstAlarmEpoch = 0;
    summary.firstAlarmSec = NaN;
else
    summary.firstAlarmEpoch = alarmIdx;
    summary.firstAlarmSec = (alarmIdx - 1) * settings.navSolPeriod / 1000;
end
summary.insOnlyEpochs = sum(navResults.modeInsOnly);
summary.gnssRampEpochs = sum(navResults.modeGnssRamp);
if isfield(navResults, 'modeClockHold')
    summary.clockHoldEpochs = sum(navResults.modeClockHold);
else
    summary.clockHoldEpochs = 0;
end
if isfield(navResults, 'modeClockHold')
    summary.insTakeoverEpochs = sum(navResults.modeInsOnly | navResults.modeGnssRamp | navResults.modeClockHold);
else
    summary.insTakeoverEpochs = sum(navResults.modeInsOnly | navResults.modeGnssRamp);
end
summary.virtualNHCEpochs = sum(navResults.modeVirtualNHC);
summary.virtualZUPTEpochs = sum(navResults.modeVirtualZUPT);
summary.takeoverPrrAssistEpochs = sum(navResults.modeTakeoverPrrAssist);
summary.qualityFallbackEpochs = sum(navResults.qualityFallback);
summary.gnssUpdateUsedEpochs = sum(navResults.gnssUpdateUsed);
summary.truthReferenceType = 'trajectory_avp';
summary.truthReferenceFile = trjFile;
summary.truthStartOffsetSec = trjStartOffsetSec;
[truthErr3d, truthErrH] = calcErrorAgainstTrajectoryTruth(navResults, trjFile, trjStartOffsetSec, settings.navSolPeriod);
navResults.errorVsTruth3D = truthErr3d(:)';
navResults.errorVsTruthH = truthErrH(:)';

if datasetTag == "clean"
    summary.referenceType = 'clean_realtime_self';
    summary.referenceFile = outFile;
    cleanErr3d = zeros(numel(navResults.X), 1);
    cleanErrH = zeros(numel(navResults.X), 1);
else
    cleanRef = loadCleanReferenceNavResults(projectRoot, profile, opts);
    nominalRefOffset = round((trjStartOffsetSec - cleanRef.referenceTimeOffsetSec) / navSolPeriodSec);
    referenceEpochOffset = nominalRefOffset;
    alignInfo = struct('used', false, 'searchEpoch', 0, 'searchSec', 0, ...
                       'fitEpochs', 0, 'bestRmse3D', NaN);
    if opts.cleanRefAutoAlignEnable
        [referenceEpochOffset, alignInfo] = localEstimateBestCleanOffset( ...
            navResults, cleanRef.navResults, nominalRefOffset, settings.navSolPeriod, opts.cleanRefAutoAlignMaxShiftSec);
    end
    summary.referenceEpochOffset = referenceEpochOffset;
    summary.referenceEpochOffsetNominal = nominalRefOffset;
    summary.referenceEpochOffsetSec = referenceEpochOffset * navSolPeriodSec;
    summary.referenceAutoAlignUsed = alignInfo.used;
    summary.referenceAutoAlignSearchEpoch = alignInfo.searchEpoch;
    summary.referenceAutoAlignSearchSec = alignInfo.searchSec;
    summary.referenceAutoAlignPreFitEpochs = alignInfo.fitEpochs;
    summary.referenceAutoAlignPreFitRmse3D = alignInfo.bestRmse3D;
    summary.referenceType = 'clean_realtime_baseline';
    summary.referenceFile = cleanRef.referenceFile;
    [cleanErr3d, cleanErrH] = calcErrorAgainstCleanReference(navResults, cleanRef.navResults, referenceEpochOffset);
end
navResults.errorVsClean3D = cleanErr3d(:)';
navResults.errorVsCleanH = cleanErrH(:)';
summary.cleanPreRmse3D = sqrt(mean(cleanErr3d.^2));
summary.cleanPreRmseH = sqrt(mean(cleanErrH.^2));
summary.cleanPostRmse3D = NaN;
summary.cleanPostRmseH = NaN;
summary.cleanEndErr3D = cleanErr3d(end);
summary.cleanEndErrH = cleanErrH(end);
if summary.firstAlarmEpoch > 0
    preIdxTruth = 1 : min(max(1, summary.firstAlarmEpoch - 1), numel(truthErr3d));
    postIdxTruth = min(summary.firstAlarmEpoch, numel(truthErr3d)) : numel(truthErr3d);
    preIdxClean = 1 : min(max(1, summary.firstAlarmEpoch - 1), numel(cleanErr3d));
    postIdxClean = min(summary.firstAlarmEpoch, numel(cleanErr3d)) : numel(cleanErr3d);
    summary.preRmse3D = sqrt(mean(truthErr3d(preIdxTruth).^2));
    summary.preRmseH = sqrt(mean(truthErrH(preIdxTruth).^2));
    summary.postRmse3D = sqrt(mean(truthErr3d(postIdxTruth).^2));
    summary.postRmseH = sqrt(mean(truthErrH(postIdxTruth).^2));
    summary.cleanPreRmse3D = sqrt(mean(cleanErr3d(preIdxClean).^2));
    summary.cleanPreRmseH = sqrt(mean(cleanErrH(preIdxClean).^2));
    summary.cleanPostRmse3D = sqrt(mean(cleanErr3d(postIdxClean).^2));
    summary.cleanPostRmseH = sqrt(mean(cleanErrH(postIdxClean).^2));
else
    summary.preRmse3D = sqrt(mean(truthErr3d.^2));
    summary.preRmseH = sqrt(mean(truthErrH.^2));
    summary.postRmse3D = NaN;
    summary.postRmseH = NaN;
end
summary.endErr3D = truthErr3d(end);
summary.endErrH = truthErrH(end);

save(outFile, 'navResults', 'settings', 'summary', '-v7.3');

fprintf('\n==== Goal-1 Unified 400s (%s, %s) ====\n', upper(char(datasetTag)), upper(char(profile)));
fprintf('Dataset: %s\n', summary.dataset);
fprintf('Output : %s\n', summary.output);
fprintf('Baseline thresholds: Tcm=%.3f Hz, Tdf=%.3f Hz, Tsat=%.3f Hz\n', ...
    summary.baselineThresholdCommonHz, summary.baselineThresholdDiffHz, summary.baselineThresholdSatHz);
fprintf('Epochs: %d, finiteXYZ: %d\n', summary.epochs, summary.finiteXYZ);
fprintf('Alarms: %d, first alarm: %.2f s (epoch %d)\n', ...
    summary.alarms, summary.firstAlarmSec, summary.firstAlarmEpoch);
fprintf('Mode epochs: ins_only=%d, gnss_ramp=%d, clock_hold=%d, mitigation_total=%d\n', ...
    summary.insOnlyEpochs, summary.gnssRampEpochs, summary.clockHoldEpochs, summary.insTakeoverEpochs);
fprintf('Virtual constraints: NHC=%d, ZUPT=%d, takeover_prr=%d\n', ...
    summary.virtualNHCEpochs, summary.virtualZUPTEpochs, summary.takeoverPrrAssistEpochs);
fprintf('Quality fallback epochs: %d\n', summary.qualityFallbackEpochs);
fprintf('GNSS update used epochs: %d\n', summary.gnssUpdateUsedEpochs);
fprintf('Truth reference: %s (offset %.2f s)\n', summary.truthReferenceFile, summary.truthStartOffsetSec);
fprintf('Clean reference: %s\n', summary.referenceFile);
fprintf('Relative-to-truth RMSE pre-alarm: 3D=%.2f m, H=%.2f m\n', summary.preRmse3D, summary.preRmseH);
if summary.firstAlarmEpoch > 0
    fprintf('Relative-to-truth RMSE post-alarm: 3D=%.2f m, H=%.2f m\n', summary.postRmse3D, summary.postRmseH);
else
    fprintf('Relative-to-truth RMSE post-alarm: N/A (no alarm)\n');
end
fprintf('Relative-to-truth end error: 3D=%.2f m, H=%.2f m\n', summary.endErr3D, summary.endErrH);
if datasetTag ~= "clean"
    fprintf('Relative-to-clean end error: 3D=%.2f m, H=%.2f m\n', summary.cleanEndErr3D, summary.cleanEndErrH);
    if isfield(summary, 'referenceAutoAlignUsed') && summary.referenceAutoAlignUsed
        fprintf('Clean-reference auto-align: nominal=%d, used=%d epochs (%.2f s), pre-fit RMSE=%.2f m\n', ...
            summary.referenceEpochOffsetNominal, summary.referenceEpochOffset, ...
            summary.referenceEpochOffsetSec, summary.referenceAutoAlignPreFitRmse3D);
    end
end
fprintf('========================================\n\n');
end

function cleanRef = loadCleanReferenceNavResults(projectRoot, profile, opts)
cleanRef = struct();
cleanRef.referenceFile = fullfile(projectRoot, sprintf('rt_tight_goal1_clean_400s_unified_%s.mat', profile));
needRebuild = (exist(cleanRef.referenceFile, 'file') ~= 2);
if ~needRebuild
    Rcheck = load(cleanRef.referenceFile, 'navResults', 'summary');
    if ~isfield(Rcheck, 'navResults') || numel(Rcheck.navResults.X) < 700
        needRebuild = true;
    elseif isfield(Rcheck, 'summary') && isstruct(Rcheck.summary) && ...
            isfield(Rcheck.summary, 'alarms') && Rcheck.summary.alarms > 0
        needRebuild = true;
    elseif isfield(Rcheck, 'summary') && isstruct(Rcheck.summary) && ...
            isfield(Rcheck.summary, 'referenceTimeOffsetSec') && ...
            isfinite(Rcheck.summary.referenceTimeOffsetSec) && ...
            abs(Rcheck.summary.referenceTimeOffsetSec) > 1e-6
        needRebuild = true;
    end
end
if needRebuild
    cleanOpts = opts;
    cleanOpts.msToProcess = 399 * 1000;
    cleanExtra = struct();
    if isfield(opts, 'extraSettings') && isstruct(opts.extraSettings)
        cleanExtra = opts.extraSettings;
    end
    cleanExtra.spoofDetArmTimeSec = 1e9;
    cleanExtra.spoofDetCommonThresholdHz = 1e9;
    cleanExtra.spoofDetDiffThresholdHz = 1e9;
    cleanExtra.spoofDetSatThresholdHz = 1e9;
    cleanExtra.gnssRecoveryEnable = 0;
    cleanOpts.extraSettings = cleanExtra;
    run_goal1_unified_400s_insonly('clean', profile, cleanOpts);
end
R = load(cleanRef.referenceFile, 'navResults', 'summary');
cleanRef.navResults = R.navResults;
cleanRef.referenceTimeOffsetSec = localEstimateDatasetOffsetSec(projectRoot, 'clean');
if isfield(R, 'summary') && isstruct(R.summary) && isfield(R.summary, 'referenceTimeOffsetSec') && ...
        ~isempty(R.summary.referenceTimeOffsetSec) && isfinite(R.summary.referenceTimeOffsetSec)
    cleanRef.referenceTimeOffsetSec = double(R.summary.referenceTimeOffsetSec);
end
end

function [err3d, errH] = calcErrorAgainstCleanReference(navResults, refNavResults, refEpochOffset)
if nargin < 3
    refEpochOffset = 0;
end
[navIdx, refIdx] = localAlignedIndexPair(numel(navResults.X), numel(refNavResults.X), refEpochOffset);
n = numel(navIdx);
dx = navResults.X(navIdx)' - refNavResults.X(refIdx)';
dy = navResults.Y(navIdx)' - refNavResults.Y(refIdx)';
dz = navResults.Z(navIdx)' - refNavResults.Z(refIdx)';
err3d = sqrt(dx.^2 + dy.^2 + dz.^2);
errH = nan(n, 1);
for i = 1 : n
    [latDeg, lonDeg, ~] = cart2geo(refNavResults.X(refIdx(i)), refNavResults.Y(refIdx(i)), refNavResults.Z(refIdx(i)), 5);
    lat = latDeg * pi / 180;
    lon = lonDeg * pi / 180;
    eErr = -sin(lon) * dx(i) + cos(lon) * dy(i);
    nErr = -sin(lat) * cos(lon) * dx(i) - sin(lat) * sin(lon) * dy(i) + cos(lat) * dz(i);
    errH(i) = hypot(eErr, nErr);
end
end

function [navIdx, refIdx] = localAlignedIndexPair(navLen, refLen, refEpochOffset)
navStart = 1 + max(-refEpochOffset, 0);
refStart = 1 + max(refEpochOffset, 0);
pairLen = min(navLen - navStart + 1, refLen - refStart + 1);
if pairLen <= 0
    error('No overlapping epochs after applying reference offset %d.', refEpochOffset);
end
navIdx = navStart : (navStart + pairLen - 1);
refIdx = refStart : (refStart + pairLen - 1);
end

function [bestOffset, info] = localEstimateBestCleanOffset(navResults, refNavResults, initOffset, navSolPeriodMs, maxShiftSec)
bestOffset = initOffset;
info = struct('used', false, 'searchEpoch', 0, 'searchSec', 0, ...
              'fitEpochs', 0, 'bestRmse3D', NaN);
if nargin < 5 || isempty(maxShiftSec) || ~isfinite(maxShiftSec) || (maxShiftSec <= 0)
    return;
end

maxShiftEpoch = max(0, round(maxShiftSec * 1000 / navSolPeriodMs));
if maxShiftEpoch == 0
    return;
end

preEnd = numel(navResults.X);
if isfield(navResults, 'spoofAlarm')
    alarmIdx = find(navResults.spoofAlarm, 1, 'first');
    if ~isempty(alarmIdx) && (alarmIdx > 1)
        preEnd = alarmIdx - 1;
    end
end
preEnd = min(preEnd, numel(navResults.X));
if preEnd < 60
    return;
end

bestRmse = inf;
searchMin = initOffset - maxShiftEpoch;
searchMax = initOffset + maxShiftEpoch;
for off = searchMin : searchMax
    try
        [navIdxAll, refIdxAll] = localAlignedIndexPair(numel(navResults.X), numel(refNavResults.X), off);
    catch
        continue;
    end
    useMask = navIdxAll <= preEnd;
    if nnz(useMask) < 40
        continue;
    end
    navIdx = navIdxAll(useMask);
    refIdx = refIdxAll(useMask);

    dx = navResults.X(navIdx)' - refNavResults.X(refIdx)';
    dy = navResults.Y(navIdx)' - refNavResults.Y(refIdx)';
    dz = navResults.Z(navIdx)' - refNavResults.Z(refIdx)';
    err3d = sqrt(dx.^2 + dy.^2 + dz.^2);
    rmse = sqrt(mean(err3d.^2));
    if rmse < bestRmse
        bestRmse = rmse;
        bestOffset = off;
        info.fitEpochs = numel(navIdx);
    end
end

if isfinite(bestRmse)
    info.used = true;
    info.searchEpoch = maxShiftEpoch;
    info.searchSec = maxShiftEpoch * navSolPeriodMs / 1000;
    info.bestRmse3D = bestRmse;
end
end

function trjStartOffsetSec = localChooseTrajectoryOffsetSec(datasetTag, opts, trackResults, subFrameStart, datasetSettings)
if nargin >= 2 && isstruct(opts)
    if isfield(opts, 'trjStartOffsetSec') && ~isempty(opts.trjStartOffsetSec) && isfinite(opts.trjStartOffsetSec)
        trjStartOffsetSec = double(opts.trjStartOffsetSec);
        return;
    end
    if isfield(opts, 'extraSettings') && isstruct(opts.extraSettings) && ...
            isfield(opts.extraSettings, 'trjStartOffsetSec') && ...
            ~isempty(opts.extraSettings.trjStartOffsetSec) && isfinite(opts.extraSettings.trjStartOffsetSec)
        trjStartOffsetSec = double(opts.extraSettings.trjStartOffsetSec);
        return;
    end
end

% Chapter-1 uses the prepared IMU trajectory and GNSS dataset as the same
% scenario timeline. Estimating an extra offset from subframe timing pushes
% the filter to start from the wrong IMU epoch and breaks the pre-alarm
% tight-coupled trajectory shape.
trjStartOffsetSec = 0.0;
end

function trjStartOffsetSec = localEstimateDatasetOffsetSec(projectRoot, datasetTag)
% Kept for backward compatibility with old summary files; Chapter-1 assumes
% the prepared dataset and IMU trajectory already share the same time zero.
trjStartOffsetSec = 0.0;
end

function trjStartOffsetSec = localEstimateTrajectoryOffsetSec(trackResults, subFrameStart, datasetSettings)
trjStartOffsetSec = nan;
if isempty(trackResults) || isempty(subFrameStart)
    return;
end
baseSettings = initSettings();
samplingFreq = baseSettings.samplingFreq;
startOffsetSec = baseSettings.startOffset / 1000;
if nargin >= 3 && isstruct(datasetSettings)
    if isfield(datasetSettings, 'samplingFreq') && ~isempty(datasetSettings.samplingFreq)
        samplingFreq = double(datasetSettings.samplingFreq);
    end
    if isfield(datasetSettings, 'startOffset') && ~isempty(datasetSettings.startOffset)
        startOffsetSec = double(datasetSettings.startOffset) / 1000;
    end
end
valid = find(subFrameStart(:)' > 1);
if isempty(valid)
    return;
end
samplePos = nan(size(valid));
for ii = 1 : numel(valid)
    idx = valid(ii);
    if isfield(trackResults(idx), 'absoluteSample') && numel(trackResults(idx).absoluteSample) >= subFrameStart(idx) - 1
        samplePos(ii) = double(trackResults(idx).absoluteSample(subFrameStart(idx) - 1));
    end
end
samplePos = samplePos(isfinite(samplePos));
if isempty(samplePos)
    return;
end
trjStartOffsetSec = max(0, max(samplePos) / samplingFreq - startOffsetSec);
end

function outFile = localBuildOutputFile(projectRoot, datasetTag, profile, msToProcess)
defaultMsToProcess = 399 * 1000;
if nargin < 4 || isempty(msToProcess)
    msToProcess = defaultMsToProcess;
end
if abs(double(msToProcess) - defaultMsToProcess) < 1
    outName = sprintf('rt_tight_goal1_%s_400s_unified_%s.mat', char(datasetTag), char(profile));
else
    outName = sprintf('rt_tight_goal1_%s_%ds_unified_%s.mat', ...
        char(datasetTag), round(double(msToProcess) / 1000), char(profile));
end
outFile = fullfile(projectRoot, outName);
end

function [err3d, errH] = calcErrorAgainstTrajectoryTruth(navResults, trjFile, trjStartOffsetSec, navSolPeriodMs)
trjData = load(char(trjFile), 'trj');
trj = trjData.trj;

navPeriodSec = navSolPeriodMs / 1000;
tQuery = trjStartOffsetSec + (1 : numel(navResults.X)) * navPeriodSec;
tAvp = trj.avp(:, end);
lat = trj.avp(:, 7);
lon = trj.avp(:, 8);
h = trj.avp(:, 9);
[Xr, Yr, Zr] = localBlhToXyzWgs84(lat, lon, h);

Xq = interp1(tAvp, Xr, tQuery, 'linear', 'extrap');
Yq = interp1(tAvp, Yr, tQuery, 'linear', 'extrap');
Zq = interp1(tAvp, Zr, tQuery, 'linear', 'extrap');
latq = interp1(tAvp, lat, tQuery, 'linear', 'extrap');
lonq = interp1(tAvp, lon, tQuery, 'linear', 'extrap');

dx = navResults.X(:) - Xq(:);
dy = navResults.Y(:) - Yq(:);
dz = navResults.Z(:) - Zq(:);
err3d = sqrt(dx.^2 + dy.^2 + dz.^2);
eErr = -sin(lonq(:)).*dx + cos(lonq(:)).*dy;
nErr = -sin(latq(:)).*cos(lonq(:)).*dx - sin(latq(:)).*sin(lonq(:)).*dy + cos(latq(:)).*dz;
errH = sqrt(eErr.^2 + nErr.^2);
end

function [X, Y, Z] = localBlhToXyzWgs84(lat, lon, h)
a = 6378137;
f = 1 / 298.257223563;
ex2 = (2 - f) * f / ((1 - f)^2);
c = a * sqrt(1 + ex2);
N = c ./ sqrt(1 + ex2 * cos(lat).^2);

X = (N + h) .* cos(lat) .* cos(lon);
Y = (N + h) .* cos(lat) .* sin(lon);
Z = ((1 - f)^2 * N + h) .* sin(lat);
end

function armTimeSec = localChooseArmTimeSec(datasetTag)
switch lower(char(datasetTag))
    case 'ds5'
        armTimeSec = 95.0;
    case 'ds6'
        armTimeSec = 90.0;
    otherwise
        armTimeSec = 94.5;
end
end

function calibEndSec = localChooseCalibEndSec(datasetTag)
switch lower(char(datasetTag))
    case 'ds5'
        calibEndSec = 80.0;
    otherwise
        calibEndSec = 90.0;
end
end

function prrNoiseStd = localChooseTakeoverPrrNoiseStd(datasetTag, profile)
if profile == "ins1"
    prrNoiseStd = 0.10;
    return;
end

switch lower(char(datasetTag))
    case 'ds5'
        if profile == "ins2"
            prrNoiseStd = 0.20;
        else
            prrNoiseStd = 0.30;
        end
    case 'ds6'
        if profile == "ins2"
            prrNoiseStd = 0.18;
        else
            prrNoiseStd = 0.28;
        end
    otherwise
        if profile == "ins2"
            prrNoiseStd = 0.18;
        else
            prrNoiseStd = 0.28;
        end
end
end

function eph = rebuildEphFromTrackResults(trackResults, channel, subFrameStart, ephIn, dataFile)
ephTemplate = initEphLocal();
if isempty(ephIn) || numel(ephIn) < 32
    eph = repmat(ephTemplate, 1, 32);
else
    eph = ephIn;
end

activeCh = find([channel.status] ~= '-');
for idx = 1:numel(activeCh)
    ch = activeCh(idx);
    prn = trackResults(ch).PRN;
    if isValidEphRecord(eph(prn))
        continue;
    end
    sfs = subFrameStart(ch);
    if sfs <= 1
        continue;
    end
    startIdx = sfs - 20;
    endIdx = sfs + (1500 * 20) - 1;
    if startIdx < 1 || endIdx > numel(trackResults(ch).I_P)
        continue;
    end

    navBitsSamples = trackResults(ch).I_P(startIdx:endIdx)';
    navBitsSamples = reshape(navBitsSamples, 20, []);
    navBits = sum(navBitsSamples) > 0;
    navBitsBin = dec2bin(navBits);
    [ephDecoded, ~] = ephemeris(navBitsBin(2:1501)', navBitsBin(1));

    ephTmp = ephTemplate;
    fn = fieldnames(ephDecoded);
    for fi = 1:numel(fn)
        ephTmp.(fn{fi}) = ephDecoded.(fn{fi});
    end
    if isValidEphRecord(ephTmp)
        eph(prn) = ephTmp;
    end
end

validCnt = 0;
for prn = 1:32
    validCnt = validCnt + double(isValidEphRecord(eph(prn)));
end
if validCnt < 4
    fprintf('[WARN] Ephemeris count after rebuild is low: %d (dataset: %s)\n', validCnt, dataFile);
end
end

function ok = isValidEphRecord(e)
ok = isstruct(e) && isfield(e, 'IODC') && isfield(e, 'IODE_sf2') && ...
     isfield(e, 'IODE_sf3') && isfield(e, 'a_f0') && ...
     isfield(e, 'a_f1') && isfield(e, 'a_f2') && isfield(e, 't_oc') && ...
     ~isempty(e.IODC) && ~isempty(e.IODE_sf2) && ~isempty(e.IODE_sf3) && ...
     isscalar(e.a_f0) && isscalar(e.a_f1) && isscalar(e.a_f2) && isscalar(e.t_oc);
end

function eph = initEphLocal()
eph = struct( ...
    'weekNumber', [], ...
    'accuracy', [], ...
    'health', [], ...
    'T_GD', [], ...
    'IODC', [], ...
    't_oc', [], ...
    'a_f2', [], ...
    'a_f1', [], ...
    'a_f0', [], ...
    'IODE_sf2', [], ...
    'C_rs', [], ...
    'deltan', [], ...
    'M_0', [], ...
    'C_uc', [], ...
    'e', [], ...
    'C_us', [], ...
    'sqrtA', [], ...
    't_oe', [], ...
    'C_ic', [], ...
    'omega_0', [], ...
    'C_is', [], ...
    'i_0', [], ...
    'C_rc', [], ...
    'omega', [], ...
    'omegaDot', [], ...
    'IODE_sf3', [], ...
    'iDot', [] ...
    );
end
