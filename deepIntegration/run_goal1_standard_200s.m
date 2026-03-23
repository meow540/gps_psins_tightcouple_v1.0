function summary = run_goal1_standard_200s()
% Reproduce Goal-1 tight-coupling result on standard 200s dataset.
% Dataset: ds5_rollback_200s.mat (first 200s of ds5.bin, baseline pipeline)

thisDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(thisDir);
addpath(fullfile(projectRoot, 'include'))
addpath(fullfile(projectRoot, 'geoFunctions'))
addpath(fullfile(projectRoot, 'acquire_zcj'))
addpath(fullfile(projectRoot, 'track_zcj'))
addpath(fullfile(projectRoot, 'PVT'))
addpath(thisDir)

dataFile = fullfile(projectRoot, 'ds5_rollback_200s.mat');
outFile = fullfile(projectRoot, 'rt_tight_goal1_standard_150s.mat');
if ~exist(dataFile, 'file')
    error('Missing dataset: %s', dataFile);
end

S = load(dataFile, 'trackResults', 'channel', 'TOW', 'eph', 'subFrameStart');
trackResults = S.trackResults; %#ok<NASGU>
channel = S.channel; %#ok<NASGU>
TOW = S.TOW; %#ok<NASGU>
eph = S.eph; %#ok<NASGU>
subFrameStart = S.subFrameStart; %#ok<NASGU>

% Standard Goal-1 run setup.
settingsOverride = struct(); %#ok<NASGU>
settingsOverride.msToProcess = 150 * 1000;
settingsOverride.plotTracking = 0;
settingsOverride.skipQuickPlot = 1;
settingsOverride.verboseEpochPrint = 0;
settingsOverride.trjFile = "E:\\fgi_result\\ins_simulation\\ins2_trj.mat";
settingsOverride.imuerrProfile = 'ins2';
settingsOverride.spoofDetArmTimeSec = 80;
settingsOverride.spoofDetConfirmEpochs = 2;
settingsOverride.spoofDetUseProfileThreshold = 1;
settingsOverride.spoofDetUseJointDecision = 1;
settingsOverride.spoofDetMinElevDeg = 20;
settingsOverride.spoofDetMetricSmoothWin = 3;
settingsOverride.pllNoiseBandwidth = 15;
settingsOverride.spoofMitigationMode = 'robust_tc';
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

summary = struct();
summary.dataset = dataFile;
summary.output = outFile;
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
summary.robustTcEpochs = sum(navResults.modeRobustTc);
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
if isfield(navResults, 'qualityFallback')
    summary.qualityFallbackEpochs = sum(navResults.qualityFallback);
else
    summary.qualityFallbackEpochs = NaN;
end
if isfield(navResults, 'gnssUpdateUsed')
    summary.gnssUpdateUsedEpochs = sum(navResults.gnssUpdateUsed);
else
    summary.gnssUpdateUsedEpochs = NaN;
end

save(outFile, 'navResults', 'settings', 'summary', '-v7.3');

fprintf('\n==== Goal-1 Standard 200s Reproduction ====\n');
fprintf('Dataset: %s\n', summary.dataset);
fprintf('Output : %s\n', summary.output);
fprintf('Epochs: %d, finiteXYZ: %d\n', summary.epochs, summary.finiteXYZ);
fprintf('Alarms: %d, first alarm: %.2f s (epoch %d)\n', ...
    summary.alarms, summary.firstAlarmSec, summary.firstAlarmEpoch);
fprintf('Mode epochs: ins_only=%d, robust_tc=%d\n', ...
    summary.insOnlyEpochs, summary.robustTcEpochs);
fprintf('Odometer epochs: constrained=%d, valid=%d\n', ...
    summary.odoConstraintEpochs, summary.odoValidEpochs);
fprintf('Kinematic epochs: total=%d, nhc=%d, zupt=%d\n', ...
    summary.kinConstraintEpochs, summary.nhcEpochs, summary.zuptEpochs);
fprintf('Quality fallback epochs: %d\n', summary.qualityFallbackEpochs);
fprintf('GNSS update used epochs: %d\n', summary.gnssUpdateUsedEpochs);
fprintf('===========================================\n\n');
end
