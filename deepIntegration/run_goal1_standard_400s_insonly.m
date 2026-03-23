function summary = run_goal1_standard_400s_insonly()
% 400s Goal-1 run with INS-only takeover after spoof alarm.

thisDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(thisDir);
addpath(fullfile(projectRoot, 'include'))
addpath(fullfile(projectRoot, 'geoFunctions'))
addpath(fullfile(projectRoot, 'acquire_zcj'))
addpath(fullfile(projectRoot, 'track_zcj'))
addpath(fullfile(projectRoot, 'PVT'))
addpath(thisDir)

dataFile = fullfile(projectRoot, 'ds5_400s.mat');
outFile = fullfile(projectRoot, 'rt_tight_goal1_standard_400s_insonly.mat');
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
settingsOverride.fileName = "E:\\ds5.bin";
settingsOverride.msToProcess = 399 * 1000;
settingsOverride.plotTracking = 0;
settingsOverride.skipQuickPlot = 1;
settingsOverride.verboseEpochPrint = 0;
settingsOverride.trjFile = "E:\\fgi_result\\ins_simulation\\ins2_trj.mat";
settingsOverride.imuerrProfile = 'ins2';
settingsOverride.pllNoiseBandwidth = 15;

settingsOverride.spoofDetArmTimeSec = 95;
settingsOverride.spoofDetConfirmEpochs = 1;
settingsOverride.spoofDetUseProfileThreshold = 1;
settingsOverride.spoofDetUseJointDecision = 1;
settingsOverride.spoofDetPrMetricThresholdM = 2.3;
settingsOverride.spoofDetMinElevDeg = 20;
settingsOverride.spoofDetMetricSmoothWin = 3;

settingsOverride.spoofMitigationMode = 'ins_only';
settingsOverride.spoofMitTwoStageEnable = 0;

settingsOverride.kinConstraintEnable = 0;
settingsOverride.odoEnable = 1;
settingsOverride.odoUseWhenSpoof = 1;
settingsOverride.odoMode = 'virtual_from_trj';
settingsOverride.odoSeed = 20260307;
settingsOverride.odoScale = 1.00;
settingsOverride.odoNoiseStdMps = 0.08;
settingsOverride.odoBiasMps = 0.00;
settingsOverride.odoGain = 0.60;
settingsOverride.odoMaxScaleStep = 0.25;
settingsOverride.odoZeroSpeedTh = 0.35;
settingsOverride.odoZeroGain = 0.85;
settingsOverride.odoVerticalDampGain = 0.35;

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
summary.qualityFallbackEpochs = sum(navResults.qualityFallback);
summary.gnssUpdateUsedEpochs = sum(navResults.gnssUpdateUsed);

% Error metrics w.r.t. ins2 trajectory reference.
trjData = load('E:\\fgi_result\\ins_simulation\\ins2_trj.mat', 'trj');
trj = trjData.trj;
dt = settings.navSolPeriod / 1000;
tNav = TOW + (1 : numel(navResults.X)) * dt;
tImu = trj.imu(1,end) + (tNav - TOW);
lat = trj.avp(:,7); lon = trj.avp(:,8); h = trj.avp(:,9); tAvp = trj.avp(:,end);
a = 6378137; f = 1/298.257223563; ex2 = (2-f)*f / ((1-f)^2); c = a*sqrt(1+ex2);
Nphi = c ./ sqrt(1 + ex2*cos(lat).^2);
Xr = (Nphi + h).*cos(lat).*cos(lon);
Yr = (Nphi + h).*cos(lat).*sin(lon);
Zr = ((1-f)^2*Nphi + h).*sin(lat);
Xq = interp1(tAvp, Xr, tImu, 'linear', 'extrap');
Yq = interp1(tAvp, Yr, tImu, 'linear', 'extrap');
Zq = interp1(tAvp, Zr, tImu, 'linear', 'extrap');
latq = interp1(tAvp, lat, tImu, 'linear', 'extrap');
lonq = interp1(tAvp, lon, tImu, 'linear', 'extrap');
dx = navResults.X(:) - Xq(:);
dy = navResults.Y(:) - Yq(:);
dz = navResults.Z(:) - Zq(:);
err3d = sqrt(dx.^2 + dy.^2 + dz.^2);
eErr = -sin(lonq(:)).*dx + cos(lonq(:)).*dy;
nErr = -sin(latq(:)).*cos(lonq(:)).*dx - sin(latq(:)).*sin(lonq(:)).*dy + cos(latq(:)).*dz;
errH = sqrt(eErr.^2 + nErr.^2);
preIdx = 1 : max(1, summary.firstAlarmEpoch - 1);
postIdx = max(1, summary.firstAlarmEpoch) : numel(err3d);
summary.preRmse3D = sqrt(mean(err3d(preIdx).^2));
summary.preRmseH = sqrt(mean(errH(preIdx).^2));
summary.postRmse3D = sqrt(mean(err3d(postIdx).^2));
summary.postRmseH = sqrt(mean(errH(postIdx).^2));
summary.endErr3D = err3d(end);
summary.endErrH = errH(end);

save(outFile, 'navResults', 'settings', 'summary', '-v7.3');

fprintf('\n==== Goal-1 400s INS-only Run ====\n');
fprintf('Dataset: %s\n', summary.dataset);
fprintf('Output : %s\n', summary.output);
fprintf('Epochs: %d, finiteXYZ: %d\n', summary.epochs, summary.finiteXYZ);
fprintf('Alarms: %d, first alarm: %.2f s (epoch %d)\n', ...
    summary.alarms, summary.firstAlarmSec, summary.firstAlarmEpoch);
fprintf('Mode epochs: ins_only=%d, robust_tc=%d\n', ...
    summary.insOnlyEpochs, summary.robustTcEpochs);
fprintf('Odometer epochs: constrained=%d, valid=%d\n', ...
    summary.odoConstraintEpochs, summary.odoValidEpochs);
fprintf('Quality fallback epochs: %d\n', summary.qualityFallbackEpochs);
fprintf('GNSS update used epochs: %d\n', summary.gnssUpdateUsedEpochs);
fprintf('RMSE pre-alarm: 3D=%.2f m, H=%.2f m\n', summary.preRmse3D, summary.preRmseH);
fprintf('RMSE post-alarm: 3D=%.2f m, H=%.2f m\n', summary.postRmse3D, summary.postRmseH);
fprintf('End error: 3D=%.2f m, H=%.2f m\n', summary.endErr3D, summary.endErrH);
fprintf('==================================\n\n');
end
