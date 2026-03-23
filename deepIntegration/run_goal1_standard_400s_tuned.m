function summary = run_goal1_standard_400s_tuned()
% Tuned 400s Goal-1 run targeting lower long-window end error.

thisDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(thisDir);
addpath(fullfile(projectRoot, 'include'))
addpath(fullfile(projectRoot, 'geoFunctions'))
addpath(fullfile(projectRoot, 'acquire_zcj'))
addpath(fullfile(projectRoot, 'track_zcj'))
addpath(fullfile(projectRoot, 'PVT'))
addpath(thisDir)

dataFile = fullfile(projectRoot, 'ds5_400s.mat');
outFile = fullfile(projectRoot, 'rt_tight_goal1_standard_400s_tuned.mat');
if ~exist(dataFile, 'file')
    error('Missing dataset: %s', dataFile);
end

S = load(dataFile, 'trackResults', 'channel', 'TOW', 'eph', 'subFrameStart');
trackResults = S.trackResults; %#ok<NASGU>
channel = S.channel; %#ok<NASGU>
TOW = S.TOW; %#ok<NASGU>
eph = S.eph; %#ok<NASGU>
subFrameStart = S.subFrameStart; %#ok<NASGU>

settingsOverride = struct(); %#ok<NASGU>
settingsOverride.msToProcess = 399 * 1000;
settingsOverride.plotTracking = 0;
settingsOverride.skipQuickPlot = 1;
settingsOverride.verboseEpochPrint = 0;
settingsOverride.trjFile = "E:\\fgi_result\\ins_simulation\\ins2_trj.mat";
settingsOverride.imuerrProfile = 'ins2';
settingsOverride.pllNoiseBandwidth = 15;

% Detector
settingsOverride.spoofDetArmTimeSec = 80;
settingsOverride.spoofDetConfirmEpochs = 2;
settingsOverride.spoofDetUseProfileThreshold = 1;
settingsOverride.spoofDetUseJointDecision = 1;
settingsOverride.spoofDetMinElevDeg = 20;
settingsOverride.spoofDetMetricSmoothWin = 3;

% Mitigation: stronger GNSS downweighting during spoof.
settingsOverride.spoofMitigationMode = 'robust_tc';
settingsOverride.spoofMitRInflation = 8;
settingsOverride.spoofMitPerSatEnable = 1;
settingsOverride.spoofMitPerSatScale = 10;
settingsOverride.spoofMitPerSatRejectHz = 5;
settingsOverride.spoofMitTwoStageEnable = 0;

% Disable NHC (historically harmful on this data); keep optional ZUPT only.
settingsOverride.kinConstraintEnable = 1;
settingsOverride.kinConstraintApplyWhenSpoof = 1;
settingsOverride.kinConstraintNhcEnable = 0;
settingsOverride.kinConstraintZuptEnable = 1;
settingsOverride.kinConstraintZuptSpeedTh = 0.55;
settingsOverride.kinConstraintZuptGain = 0.65;

% Virtual odometer: stronger but bounded correction.
settingsOverride.odoEnable = 1;
settingsOverride.odoUseWhenSpoof = 1;
settingsOverride.odoMode = 'virtual_from_trj';
settingsOverride.odoSeed = 20260307;
settingsOverride.odoScale = 1.00;
settingsOverride.odoNoiseStdMps = 0.05;
settingsOverride.odoBiasMps = 0.00;
settingsOverride.odoGain = 0.65;
settingsOverride.odoMaxScaleStep = 0.20;
settingsOverride.odoZeroSpeedTh = 0.35;
settingsOverride.odoZeroGain = 0.85;
settingsOverride.odoVerticalDampGain = 0.25;

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
summary.odoConstraintEpochs = sum(navResults.modeOdoConstraint);
summary.odoValidEpochs = sum(navResults.odoDataValid);
summary.kinConstraintEpochs = sum(navResults.modeKinConstraint);
summary.nhcEpochs = sum(navResults.modeNhc);
summary.zuptEpochs = sum(navResults.modeZupt);
summary.qualityFallbackEpochs = sum(navResults.qualityFallback);
summary.gnssUpdateUsedEpochs = sum(navResults.gnssUpdateUsed);

save(outFile, 'navResults', 'settings', 'summary', '-v7.3');

fprintf('\n==== Goal-1 Tuned 400s Run ====\n');
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
fprintf('================================\n\n');
end

