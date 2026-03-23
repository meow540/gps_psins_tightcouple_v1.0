function report = run_goal1_compare_modes(dataFile, msToProcess)
% Compare Goal-1 mitigation modes on the same dataset and settings.
%
% Usage:
%   run_goal1_compare_modes
%   run_goal1_compare_modes('ds5_rollback_200s.mat', 120*1000)

if nargin < 1 || isempty(dataFile)
    dataFile = 'ds5_rollback_200s.mat';
end
if nargin < 2 || isempty(msToProcess)
    msToProcess = 150 * 1000;
end

thisDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(thisDir);
addpath(fullfile(projectRoot, 'include'))
addpath(fullfile(projectRoot, 'geoFunctions'))
addpath(fullfile(projectRoot, 'acquire_zcj'))
addpath(fullfile(projectRoot, 'track_zcj'))
addpath(fullfile(projectRoot, 'PVT'))
addpath(thisDir)

if ~exist(dataFile, 'file')
    altPath = fullfile(projectRoot, dataFile);
    if exist(altPath, 'file')
        dataFile = altPath;
    else
        error('Missing dataset: %s', dataFile);
    end
end

S = load(dataFile, 'trackResults', 'channel', 'TOW', 'eph', 'subFrameStart');
modes = {'robust_tc', 'ins_only'};
report = struct([]);

for ii = 1 : numel(modes)
    modeName = modes{ii};

    trackResults = S.trackResults; %#ok<NASGU>
    channel = S.channel; %#ok<NASGU>
    TOW = S.TOW; %#ok<NASGU>
    eph = S.eph; %#ok<NASGU>
    subFrameStart = S.subFrameStart; %#ok<NASGU>

    settingsOverride = struct(); %#ok<NASGU>
    settingsOverride.msToProcess = msToProcess;
    settingsOverride.plotTracking = 0;
    settingsOverride.skipQuickPlot = 1;
    settingsOverride.verboseEpochPrint = 0;
    settingsOverride.trjFile = "E:\\fgi_result\\ins_simulation\\ins2_trj.mat";
    settingsOverride.imuerrProfile = 'ins2';
    settingsOverride.pllNoiseBandwidth = 15;
    settingsOverride.spoofDetArmTimeSec = 80;
    settingsOverride.spoofDetConfirmEpochs = 2;
    settingsOverride.spoofDetUseProfileThreshold = 1;
    settingsOverride.spoofDetUseJointDecision = 1;
    settingsOverride.spoofDetMinElevDeg = 20;
    settingsOverride.spoofDetMetricSmoothWin = 3;
    settingsOverride.spoofMitigationMode = modeName;
    settingsOverride.spoofMitPerSatEnable = 1;
    settingsOverride.spoofMitPerSatScale = 8;
    settingsOverride.spoofMitPerSatRejectHz = 6;
    settingsOverride.spoofMitTwoStageEnable = 0;
    settingsOverride.spoofMitStage2PauseWarmupEpochs = 20;
    settingsOverride.spoofMitStage2PauseEveryEpochs = 6;
    settingsOverride.spoofMitStage2RefreshRInflation = 6;
    settingsOverride.spoofMitStage2RefreshClipMeters = 10;
    settingsOverride.spoofMitStage2RefreshClockGain = 0.02;
    settingsOverride.spoofMitStage2RefreshMaxMetricHz = 20;
    settingsOverride.kinConstraintEnable = 0;
    settingsOverride.kinConstraintApplyWhenSpoof = 1;
    settingsOverride.kinConstraintNhcEnable = 1;
    settingsOverride.kinConstraintNhcGain = 0.35;
    settingsOverride.kinConstraintNhcMinHorSpeed = 1.0;
    settingsOverride.kinConstraintAxisLearnEnable = 1;
    settingsOverride.kinConstraintAxisLearnMinSamples = 40;
    settingsOverride.kinConstraintAxisLearnSpeedTh = 1.0;
    settingsOverride.kinConstraintZuptEnable = 1;
    settingsOverride.kinConstraintZuptSpeedTh = 0.35;
    settingsOverride.kinConstraintZuptGain = 0.8;
    settingsOverride.odoEnable = 1;
    settingsOverride.odoUseWhenSpoof = 1;
    settingsOverride.odoMode = 'virtual_from_trj';
    settingsOverride.odoSeed = 20260307;
    settingsOverride.odoScale = 1.00;
    settingsOverride.odoNoiseStdMps = 0.10;
    settingsOverride.odoBiasMps = 0.00;
    settingsOverride.odoGain = 0.45;
    settingsOverride.odoMaxScaleStep = 0.30;
    settingsOverride.odoZeroSpeedTh = 0.35;
    settingsOverride.odoZeroGain = 0.80;
    settingsOverride.odoVerticalDampGain = 0.20;

    realtime_tightCouple;
    if ~exist('settings', 'var') || ~isstruct(settings)
        error('realtime_tightCouple did not return settings struct.');
    end
    settingsRun = settings;

    summary = summarize_mode(navResults, settingsRun, modeName, dataFile);
    outFile = fullfile(projectRoot, sprintf('rt_tight_goal1_%s_%ds.mat', modeName, round(msToProcess/1000)));
    settings = settingsRun; %#ok<NASGU>
    save(outFile, 'navResults', 'settings', 'summary', '-v7.3');
    summary.output = outFile;
    report = [report; summary]; %#ok<AGROW>
end

fprintf('\n==== Goal-1 Mode Comparison ====\n');
for ii = 1 : numel(report)
    fprintf('mode=%s | epochs=%d | alarms=%d | firstAlarm=%.2fs | pre80ResMed=%.2fHz | gnssUsed=%d\n', ...
        report(ii).mode, report(ii).epochs, report(ii).alarms, report(ii).firstAlarmSec, ...
        report(ii).pre80ResidualMedHz, report(ii).gnssUpdateUsedEpochs);
end
fprintf('================================\n\n');
end

function summary = summarize_mode(navResults, settings, modeName, dataFile)
t = (0 : numel(navResults.X)-1) * settings.navSolPeriod / 1000;
preIdx = t < min(80, t(end));

summary = struct();
summary.mode = modeName;
summary.dataset = dataFile;
summary.epochs = numel(navResults.X);
summary.alarms = sum(navResults.spoofAlarm);
idxAlarm = find(navResults.spoofAlarm, 1, 'first');
if isempty(idxAlarm)
    summary.firstAlarmSec = NaN;
else
    summary.firstAlarmSec = t(idxAlarm);
end
summary.robustTcEpochs = sum(navResults.modeRobustTc);
summary.insOnlyEpochs = sum(navResults.modeInsOnly);
summary.stage2PauseEpochs = sum(navResults.modeStage2PausePr);
if isfield(navResults, 'modeOdoConstraint')
    summary.odoConstraintEpochs = sum(navResults.modeOdoConstraint);
    summary.odoValidEpochs = sum(navResults.odoDataValid);
else
    summary.odoConstraintEpochs = NaN;
    summary.odoValidEpochs = NaN;
end
if isfield(navResults, 'modeKinConstraint')
    summary.kinConstraintEpochs = sum(navResults.modeKinConstraint);
    summary.nhcEpochs = sum(navResults.modeNhc);
    summary.zuptEpochs = sum(navResults.modeZupt);
else
    summary.kinConstraintEpochs = NaN;
    summary.nhcEpochs = NaN;
    summary.zuptEpochs = NaN;
end
summary.gnssUpdateUsedEpochs = sum(navResults.gnssUpdateUsed);
summary.qualityFallbackEpochs = sum(navResults.qualityFallback);
summary.validSatMin = min(navResults.spoofValidSatNum);
summary.validSatMedian = median(navResults.spoofValidSatNum);
summary.pre80ResidualMedHz = median(abs(navResults.dopplerResidualHz(:, preIdx)), 'all', 'omitnan');
summary.pre80MetricMedHz = median(navResults.spoofMetricHz(preIdx), 'omitnan');
end
