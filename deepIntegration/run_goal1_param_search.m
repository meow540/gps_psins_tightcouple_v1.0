function resultsTbl = run_goal1_param_search()
% Coarse parameter search for Goal-1 spoofing mitigation in realtime_tightCouple.
% It evaluates several robust-TC parameter sets on spoof data and checks
% false alarms on clean data for the best candidate.

projectRoot = fileparts(fileparts(mfilename('fullpath')));
origDir = pwd;
cleanupObj = onCleanup(@() cd(origDir)); %#ok<NASGU>
cd(projectRoot);

spoofDataFile = 'ds5_400s.mat';
cleanDataFile = 'myworkspace_400s.mat';
trjFile = 'E:\fgi_result\ins_simulation\ins3_trj.mat';

if ~exist(spoofDataFile, 'file')
    error('Missing spoof data file: %s', spoofDataFile);
end
if ~exist(cleanDataFile, 'file')
    error('Missing clean data file: %s', cleanDataFile);
end
if ~exist(trjFile, 'file')
    error('Missing trajectory file: %s', trjFile);
end

spoofData = load(spoofDataFile, 'trackResults', 'channel', 'TOW', 'eph', 'subFrameStart', 'settings');
cleanData = load(cleanDataFile, 'trackResults', 'channel', 'TOW', 'eph', 'subFrameStart', 'settings');
trjData = load(trjFile, 'trj');
trj = trjData.trj;

outDir = fullfile(projectRoot, 'deepIntegration', 'search_runs');
if ~exist(outDir, 'dir')
    mkdir(outDir);
end

% Keep runtime manageable for search; best candidate can be re-checked at 150s.
baseOverride = struct();
baseOverride.msToProcess = 130 * 1000;
baseOverride.plotTracking = 0;
baseOverride.skipQuickPlot = 1;
baseOverride.verboseEpochPrint = 0;
baseOverride.spoofDetMetricThresholdHz = 6;
baseOverride.spoofDetConfirmEpochs = 2;
baseOverride.spoofDetArmTimeSec = 80;
baseOverride.spoofMitigationMode = 'robust_tc';
baseOverride.spoofMitUseDopplerGate = 1;

combos = struct( ...
    'name', {'c00_current', 'c01_moderate', 'c02_light'}, ...
    'RInflation', {50, 10, 3}, ...
    'DopplerGateHz', {4.0, 3.5, 3.0}, ...
    'OutlierScale', {4, 8, 12}, ...
    'ClockGain', {0.0, 0.1, 0.2} ...
);

results = repmat(struct( ...
    'name', '', ...
    'elapsedSec', nan, ...
    'alarmEpoch', 0, ...
    'alarmTimeSec', nan, ...
    'insOnlyEpochs', 0, ...
    'robustTcEpochs', 0, ...
    'postAlarm3dRmse', inf, ...
    'postAlarmHrRmse', inf, ...
    'end3dErr', inf, ...
    'endHrErr', inf, ...
    'RInflation', nan, ...
    'DopplerGateHz', nan, ...
    'OutlierScale', nan, ...
    'ClockGain', nan), 1, numel(combos));

for i = 1 : numel(combos)
    fprintf('\n[SEARCH] Running %s (%d/%d)\n', combos(i).name, i, numel(combos));

    settingsOverride = baseOverride;
    settingsOverride.spoofMitRInflation = combos(i).RInflation;
    settingsOverride.spoofMitDopplerGateHz = combos(i).DopplerGateHz;
    settingsOverride.spoofMitOutlierScale = combos(i).OutlierScale;
    settingsOverride.spoofMitClockGain = combos(i).ClockGain;

    trackResults = spoofData.trackResults; %#ok<NASGU>
    channel = spoofData.channel; %#ok<NASGU>
    TOW = spoofData.TOW; %#ok<NASGU>
    eph = spoofData.eph; %#ok<NASGU>
    subFrameStart = spoofData.subFrameStart; %#ok<NASGU>

    tStart = tic;
    realtime_tightCouple; %#ok<NASGU>
    elapsed = toc(tStart);

    metrics = evaluate_nav_errors(navResults, TOW, spoofData.settings.navSolPeriod, trj);

    results(i).name = combos(i).name;
    results(i).elapsedSec = elapsed;
    results(i).alarmEpoch = metrics.alarmEpoch;
    results(i).alarmTimeSec = metrics.alarmTimeSec;
    results(i).insOnlyEpochs = sum(navResults.modeInsOnly);
    results(i).robustTcEpochs = sum(navResults.modeRobustTc);
    results(i).postAlarm3dRmse = metrics.postAlarm3dRmse;
    results(i).postAlarmHrRmse = metrics.postAlarmHrRmse;
    results(i).end3dErr = metrics.end3dErr;
    results(i).endHrErr = metrics.endHrErr;
    results(i).RInflation = combos(i).RInflation;
    results(i).DopplerGateHz = combos(i).DopplerGateHz;
    results(i).OutlierScale = combos(i).OutlierScale;
    results(i).ClockGain = combos(i).ClockGain;

    save(fullfile(outDir, sprintf('%s_spoof.mat', combos(i).name)), ...
        'navResults', 'metrics', 'settingsOverride', 'elapsed');
end

resultsTbl = struct2table(results);
resultsTbl = sortrows(resultsTbl, {'postAlarmHrRmse', 'postAlarm3dRmse'});
disp(resultsTbl);

best = resultsTbl(1, :);
fprintf('\n[SEARCH] Best candidate on spoof data: %s\n', best.name{1});

% Clean-data sanity check for the best candidate (short window).
settingsOverride = baseOverride;
settingsOverride.spoofMitRInflation = best.RInflation;
settingsOverride.spoofMitDopplerGateHz = best.DopplerGateHz;
settingsOverride.spoofMitOutlierScale = best.OutlierScale;
settingsOverride.spoofMitClockGain = best.ClockGain;
settingsOverride.msToProcess = 20 * 1000;

trackResults = cleanData.trackResults; %#ok<NASGU>
channel = cleanData.channel; %#ok<NASGU>
TOW = cleanData.TOW; %#ok<NASGU>
eph = cleanData.eph; %#ok<NASGU>
subFrameStart = cleanData.subFrameStart; %#ok<NASGU>

tStart = tic;
realtime_tightCouple; %#ok<NASGU>
cleanElapsed = toc(tStart);
cleanSummary = struct();
cleanSummary.alarms = sum(navResults.spoofAlarm);
cleanSummary.insOnlyEpochs = sum(navResults.modeInsOnly);
cleanSummary.robustTcEpochs = sum(navResults.modeRobustTc);
cleanSummary.elapsedSec = cleanElapsed;

fprintf('[SEARCH] Clean sanity: alarms=%d, insOnly=%d, robustTc=%d\n', ...
    cleanSummary.alarms, cleanSummary.insOnlyEpochs, cleanSummary.robustTcEpochs);

writetable(resultsTbl, fullfile(outDir, 'goal1_param_search_summary.csv'));
save(fullfile(outDir, 'goal1_param_search_summary.mat'), ...
    'resultsTbl', 'cleanSummary', 'baseOverride', 'combos');
end


function metrics = evaluate_nav_errors(navResults, TOW, navSolPeriodMs, trj)
% Evaluate post-alarm navigation errors against trajectory truth.

navPeriodSec = navSolPeriodMs / 1000;
tNav = TOW + (1 : numel(navResults.X)) * navPeriodSec;
tImuQuery = trj.imu(1, end) + (tNav - TOW);

lat = trj.avp(:,7); lon = trj.avp(:,8); h = trj.avp(:,9); tAvp = trj.avp(:,end);
[Xr, Yr, Zr] = blh_to_xyz_wgs84(lat, lon, h);

Xq = interp1(tAvp, Xr, tImuQuery, 'linear', 'extrap');
Yq = interp1(tAvp, Yr, tImuQuery, 'linear', 'extrap');
Zq = interp1(tAvp, Zr, tImuQuery, 'linear', 'extrap');
latq = interp1(tAvp, lat, tImuQuery, 'linear', 'extrap');
lonq = interp1(tAvp, lon, tImuQuery, 'linear', 'extrap');

dx = navResults.X(:) - Xq(:);
dy = navResults.Y(:) - Yq(:);
dz = navResults.Z(:) - Zq(:);

err3d = sqrt(dx.^2 + dy.^2 + dz.^2);
eErr = -sin(lonq(:)).*dx + cos(lonq(:)).*dy;
nErr = -sin(latq(:)).*cos(lonq(:)).*dx - sin(latq(:)).*sin(lonq(:)).*dy + cos(latq(:)).*dz;
errH = sqrt(eErr.^2 + nErr.^2);

idxAlarm = find(navResults.spoofAlarm, 1, 'first');
if isempty(idxAlarm)
    metrics.alarmEpoch = 0;
    metrics.alarmTimeSec = nan;
    metrics.postAlarm3dRmse = inf;
    metrics.postAlarmHrRmse = inf;
else
    idx = idxAlarm : numel(err3d);
    metrics.alarmEpoch = idxAlarm;
    metrics.alarmTimeSec = (idxAlarm - 1) * navPeriodSec;
    metrics.postAlarm3dRmse = sqrt(mean(err3d(idx).^2));
    metrics.postAlarmHrRmse = sqrt(mean(errH(idx).^2));
end

metrics.end3dErr = err3d(end);
metrics.endHrErr = errH(end);
end


function [X, Y, Z] = blh_to_xyz_wgs84(lat, lon, h)
% lat/lon in radians, h in meters.
a = 6378137;
f = 1/298.257223563;
ex2 = (2-f)*f / ((1-f)^2);
c = a * sqrt(1 + ex2);
N = c ./ sqrt(1 + ex2 * cos(lat).^2);

X = (N + h) .* cos(lat) .* cos(lon);
Y = (N + h) .* cos(lat) .* sin(lon);
Z = ((1-f)^2 * N + h) .* sin(lat);
end
