function summary = run_goal1_standard_400s_insonly_ds6(profile, rawBinPath)
% Run Goal-1 on ds6_400s with INS-only takeover.
%
% Usage:
%   run_goal1_standard_400s_insonly_ds6            % default ins2
%   run_goal1_standard_400s_insonly_ds6('ins3')
%   run_goal1_standard_400s_insonly_ds6('ins2', "D:\path\ds6.bin")

if nargin < 1 || isempty(profile)
    profile = 'ins2';
end
if nargin < 2
    rawBinPath = "";
end
profile = lower(string(profile));

thisDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(thisDir);
addpath(fullfile(projectRoot, 'include'))
addpath(fullfile(projectRoot, 'geoFunctions'))
addpath(fullfile(projectRoot, 'acquire_zcj'))
addpath(fullfile(projectRoot, 'track_zcj'))
addpath(fullfile(projectRoot, 'PVT'))
addpath(thisDir)

dataFile = fullfile(projectRoot, 'ds6_400s.mat');
if ~exist(dataFile, 'file')
    error('Missing dataset: %s', dataFile);
end

switch profile
    case "ins3"
        trjFile = "E:\\fgi_result\\ins_simulation\\ins3_trj.mat";
        imuProfile = 'ins3';
        prThresholdM = 5.0;
        odoNoiseStd = 0.06;
        odoGain = 0.65;
        odoVerticalDamp = 0.55;
        outFile = fullfile(projectRoot, 'rt_tight_goal1_ds6_400s_insonly_ins3.mat');
    otherwise
        trjFile = "E:\\fgi_result\\ins_simulation\\ins2_trj.mat";
        imuProfile = 'ins2';
        prThresholdM = 2.3;
        odoNoiseStd = 0.08;
        odoGain = 0.60;
        odoVerticalDamp = 0.35;
        outFile = fullfile(projectRoot, 'rt_tight_goal1_ds6_400s_insonly_ins2.mat');
end

S = load(dataFile, 'trackResults', 'channel', 'TOW', 'eph', 'subFrameStart', 'settings');
trackResults = S.trackResults; %#ok<NASGU>
channel = S.channel; %#ok<NASGU>
TOW = S.TOW; %#ok<NASGU>
eph = S.eph; %#ok<NASGU>
subFrameStart = S.subFrameStart; %#ok<NASGU>
eph = rebuildEphFromTrackResults(trackResults, channel, subFrameStart, eph, dataFile); %#ok<NASGU>
if ~isfinite(TOW) || isempty(TOW)
    TOW = inferTowFromEph(eph); %#ok<NASGU>
end
if ~isfinite(TOW) || isempty(TOW)
    error('Cannot infer valid TOW from %s', dataFile);
end

activeCh = find([channel.status] ~= '-');
activeCh = activeCh(subFrameStart(activeCh) > 1);
validEphCnt = 0;
for ii = 1:numel(activeCh)
    prn = trackResults(activeCh(ii)).PRN;
    validEphCnt = validEphCnt + double(isValidEphRecord(eph(prn)));
end
if validEphCnt < 2
    error('Too few valid satellites for ds6 replay after eph rebuild: %d', validEphCnt);
end
tcMinSat = min(3, validEphCnt);

% Prefer replaying directly from tracked-channel arrays in ds6_400s.mat.
resolvedBin = "";
if strlength(string(rawBinPath)) > 0
    resolvedBin = string(rawBinPath);
elseif isfield(S, 'settings') && isfield(S.settings, 'fileName')
    resolvedBin = string(S.settings.fileName);
end

settingsOverride = struct(); %#ok<NASGU>
settingsOverride.fileName = resolvedBin;
settingsOverride.offlineReplayFromTrackResults = 1;
settingsOverride.tcMinSatForInit = tcMinSat;
settingsOverride.msToProcess = 399 * 1000;
settingsOverride.plotTracking = 0;
settingsOverride.skipQuickPlot = 1;
settingsOverride.verboseEpochPrint = 0;
settingsOverride.trjFile = trjFile;
settingsOverride.imuerrProfile = imuProfile;
settingsOverride.pllNoiseBandwidth = 15;

settingsOverride.spoofDetArmTimeSec = 95;
settingsOverride.spoofDetConfirmEpochs = 1;
settingsOverride.spoofDetUseProfileThreshold = 1;
settingsOverride.spoofDetUseJointDecision = 1;
settingsOverride.spoofDetPrMetricThresholdM = prThresholdM;
settingsOverride.spoofDetMinSat = tcMinSat;
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
settingsOverride.odoNoiseStdMps = odoNoiseStd;
settingsOverride.odoBiasMps = 0.00;
settingsOverride.odoGain = odoGain;
settingsOverride.odoMaxScaleStep = 0.25;
settingsOverride.odoZeroSpeedTh = 0.35;
settingsOverride.odoZeroGain = 0.85;
settingsOverride.odoVerticalDampGain = odoVerticalDamp;

realtime_tightCouple;

summary = struct();
summary.profile = char(profile);
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

% Error metrics w.r.t selected INS trajectory reference.
trjData = load(char(trjFile), 'trj');
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

fprintf('\n==== ds6_400s INS-only Run (%s) ====\n', upper(char(profile)));
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
fprintf('====================================\n\n');
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

function tow = inferTowFromEph(eph)
tow = NaN;
if isempty(eph) || ~isstruct(eph)
    return;
end
for prn = 1:numel(eph)
    if isValidEphRecord(eph(prn)) && isfield(eph(prn), 't_oc') && ~isempty(eph(prn).t_oc)
        tow = floor(eph(prn).t_oc / 6) * 6;
        return;
    end
end
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
