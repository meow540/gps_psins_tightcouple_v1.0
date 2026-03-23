function summary = run_goal1_ins2_odometer_sim_130s()
% INS2 + virtual odometer simulation (130 s).
% This script is for odometer-aiding behavior study without real odometer file.

thisDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(thisDir);
addpath(fullfile(projectRoot, 'include'))
addpath(fullfile(projectRoot, 'geoFunctions'))
addpath(fullfile(projectRoot, 'acquire_zcj'))
addpath(fullfile(projectRoot, 'track_zcj'))
addpath(fullfile(projectRoot, 'PVT'))
addpath(thisDir)

dataFile = fullfile(projectRoot, 'ds5_rollback_200s.mat');
outFile = fullfile(projectRoot, 'rt_tight_goal1_ins2_odo_sim_130s.mat');
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
settingsOverride.msToProcess = 130 * 1000;
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
settingsOverride.spoofMitTwoStageEnable = 0;
settingsOverride.spoofMitPerSatEnable = 1;
settingsOverride.spoofMitPerSatScale = 8;
settingsOverride.spoofMitPerSatRejectHz = 6;

% Virtual odometer (no external file required)
settingsOverride.odoEnable = 1;
settingsOverride.odoUseWhenSpoof = 0;  % simulate odometer aiding for all epochs
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
idxAlarm = find(navResults.spoofAlarm, 1, 'first');
if isempty(idxAlarm)
    summary.firstAlarmSec = NaN;
    summary.firstAlarmEpoch = 0;
else
    summary.firstAlarmSec = (idxAlarm - 1) * settings.navSolPeriod / 1000;
    summary.firstAlarmEpoch = idxAlarm;
end
summary.robustTcEpochs = sum(navResults.modeRobustTc);
summary.insOnlyEpochs = sum(navResults.modeInsOnly);
summary.odoConstraintEpochs = sum(navResults.modeOdoConstraint);
summary.odoValidEpochs = sum(navResults.odoDataValid);
summary.gnssUpdateUsedEpochs = sum(navResults.gnssUpdateUsed);

save(outFile, 'navResults', 'settings', 'summary', '-v7.3');

fprintf('\n==== INS2 Odometer Simulation (130s) ====\n');
fprintf('Dataset: %s\n', summary.dataset);
fprintf('Output : %s\n', summary.output);
fprintf('Epochs: %d, finiteXYZ: %d\n', summary.epochs, summary.finiteXYZ);
fprintf('Alarms: %d, first alarm: %.2f s (epoch %d)\n', ...
    summary.alarms, summary.firstAlarmSec, summary.firstAlarmEpoch);
fprintf('Mode epochs: robust_tc=%d, ins_only=%d\n', ...
    summary.robustTcEpochs, summary.insOnlyEpochs);
fprintf('Odometer epochs: constrained=%d, valid=%d\n', ...
    summary.odoConstraintEpochs, summary.odoValidEpochs);
fprintf('GNSS update used epochs: %d\n', summary.gnssUpdateUsedEpochs);
fprintf('========================================\n\n');
end
