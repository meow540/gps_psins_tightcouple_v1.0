function figPaths = plot_goal1_results(resultFile)
% Plot Chapter-1 navigation-layer spoof detection and INS-takeover results.

if nargin < 1 || isempty(resultFile)
    resultFile = 'rt_tight_goal1_ds5_400s_unified_ins2.mat';
end

thisDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(thisDir);
if exist(resultFile, 'file') ~= 2
    altFile = fullfile(projectRoot, resultFile);
    if exist(altFile, 'file') ~= 2
        error('Result file not found: %s', resultFile);
    end
    resultFile = altFile;
end

S = load(resultFile);
if ~isfield(S, 'navResults') || ~isfield(S, 'settings')
    error('Result MAT must contain navResults and settings.');
end

navResults = S.navResults;
settings = S.settings;
summary = struct();
if isfield(S, 'summary')
    summary = S.summary;
end
dt = settings.navSolPeriod / 1000;
t = (0 : numel(navResults.X) - 1) * dt;
alarmIdx = find(navResults.spoofAlarm, 1, 'first');
alarmTime = [];
if ~isempty(alarmIdx)
    alarmTime = t(alarmIdx);
end

[~, baseName, ~] = fileparts(resultFile);
outDir = fullfile(projectRoot, 'deepIntegration', 'figures');
if exist(outDir, 'dir') ~= 7
    mkdir(outDir);
end

scenarioLabel = localBuildScenarioLabel(summary, resultFile);
prnList = localPickPrnList(navResults);
[mainRefNavResults, mainRefLabel, navIdx, refIdx, mainRefKind] = ...
    localLoadMainReferenceNavResults(summary, navResults, settings, resultFile);
compareLen = numel(navIdx);
tCompare = t(navIdx);
[E, N, U, Er, Nr, Ur] = localComputeEnuPair(navResults, mainRefNavResults, navIdx, refIdx);
[residualMat, residualLabel] = localSelectResidualMatrix(navResults);
[err3dRef, errHRef, errLabel, errFileSuffix] = ...
    localComputeReferenceErrors(navResults, mainRefNavResults, navIdx, refIdx, mainRefKind);

figPaths = {};

% 1) ENU horizontal trajectory
f1 = figure('Visible', 'off');
plot(Er, Nr, 'k--', 'LineWidth', 1.1);
hold on;
plot(E, N, 'b-', 'LineWidth', 1.2);
grid on;
axis equal;
xlabel('East (m)');
ylabel('North (m)');
title(sprintf('%s Trajectory vs %s in ENU', scenarioLabel, mainRefLabel), 'Interpreter', 'none');
legend({mainRefLabel, scenarioLabel}, 'Location', 'best', 'Interpreter', 'none');
fp = fullfile(outDir, [baseName, '_traj_vs_cleandynamic_enu_en.png']);
exportgraphics(f1, fp, 'Resolution', 180);
close(f1);
figPaths{end + 1} = fp; %#ok<AGROW>

% 2) ENU 3D trajectory
f2 = figure('Visible', 'off');
plot3(Er, Nr, Ur, 'k--', 'LineWidth', 1.0);
hold on;
plot3(E, N, U, 'b-', 'LineWidth', 1.0);
grid on;
axis equal;
xlabel('East (m)');
ylabel('North (m)');
zlabel('Up (m)');
title(sprintf('%s Trajectory vs %s in ENU (3D)', scenarioLabel, mainRefLabel), 'Interpreter', 'none');
legend({mainRefLabel, scenarioLabel}, 'Location', 'best', 'Interpreter', 'none');
view(45, 30);
fp = fullfile(outDir, [baseName, '_traj_vs_cleandynamic_enu_3d.png']);
exportgraphics(f2, fp, 'Resolution', 180);
close(f2);
figPaths{end + 1} = fp; %#ok<AGROW>

% 2b) Post-alarm relative horizontal ENU trajectory
if compareLen > 0
    [Erel, Nrel, ErelRef, NrelRef, relMask] = localComputeAlarmRelativeEnu(E, N, Er, Nr, navIdx, alarmIdx);
    if any(relMask)
        f2b = figure('Visible', 'off');
        plot(ErelRef(relMask), NrelRef(relMask), 'k--', 'LineWidth', 1.1);
        hold on;
        plot(Erel(relMask), Nrel(relMask), 'b-', 'LineWidth', 1.2);
        grid on;
        axis equal;
        xlabel('East (m)');
        ylabel('North (m)');
        title(sprintf('%s Post-Alarm Relative ENU', scenarioLabel), 'Interpreter', 'none');
        legend({sprintf('%s (relative)', mainRefLabel), sprintf('%s (relative)', scenarioLabel)}, ...
            'Location', 'best', 'Interpreter', 'none');
        fp = fullfile(outDir, [baseName, '_traj_postalarm_relative_to_cleandynamic.png']);
        exportgraphics(f2b, fp, 'Resolution', 180);
        close(f2b);
        figPaths{end + 1} = fp; %#ok<AGROW>
    end
end

% 3) Detector metrics
f3 = figure('Visible', 'off');
tl3 = tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
plot(t, navResults.detectorMetricCommonHz, 'b-', 'LineWidth', 1.1);
hold on;
grid on;
if isfield(navResults, 'detectorThresholdCommonHz')
    plot(t, navResults.detectorThresholdCommonHz, 'r--', 'LineWidth', 1.0);
end
localAddAlarmLine(alarmTime);
ylabel('Hz');
title('Common-Mode Metric');
legend({'M_{cm}', 'T_{cm}'}, 'Location', 'best');

nexttile;
plot(t, navResults.detectorMetricDiffHz, 'Color', [0 0.5 0], 'LineWidth', 1.1);
hold on;
grid on;
if isfield(navResults, 'detectorThresholdDiffHz')
    plot(t, navResults.detectorThresholdDiffHz, 'r--', 'LineWidth', 1.0);
end
localAddAlarmLine(alarmTime);
ylabel('Hz');
title('Differential-Mode Metric');
legend({'M_{df}', 'T_{df}'}, 'Location', 'best');

nexttile;
stairs(t, navResults.detectorHitSatNum, 'm-', 'LineWidth', 1.1);
hold on;
grid on;
stairs(t, navResults.detectorValidSatNum, 'k--', 'LineWidth', 1.0);
if isfield(settings, 'spoofDetMinHitSat')
    yline(settings.spoofDetMinHitSat, 'r--', 'LineWidth', 1.0);
end
localAddAlarmLine(alarmTime);
xlabel('Time (s)');
ylabel('Satellite Count');
title('Detector Satellite Support');
legend({'Hit satellites', 'Valid satellites', 'N_{hit}'}, 'Location', 'best');

title(tl3, sprintf('%s Spoof Detector Metrics', scenarioLabel), 'Interpreter', 'none');
fp = fullfile(outDir, [baseName, '_spoof_metric.png']);
exportgraphics(f3, fp, 'Resolution', 180);
close(f3);
figPaths{end + 1} = fp; %#ok<AGROW>

% 4) Timeline
stateOrder = {'detectorArmed', 'spoofFlag', 'modeInsTakeover', 'modeVirtualNHC', 'modeVirtualZUPT', 'gnssUpdateUsed'};
stateLabel = {'detectorArmed', 'spoofFlag', 'insTakeover', 'virtualNHC', 'virtualZUPT', 'gnssUpdate'};
f4 = figure('Visible', 'off');
tl4 = tiledlayout(numel(stateOrder), 1, 'TileSpacing', 'compact', 'Padding', 'compact');
for ii = 1 : numel(stateOrder)
    nexttile;
    stairs(t, double(navResults.(stateOrder{ii})), 'LineWidth', 1.1);
    grid on;
    ylim([-0.1 1.1]);
    xlim([t(1), t(end)]);
    ylabel(stateLabel{ii}, 'Interpreter', 'none');
    localAddAlarmLine(alarmTime);
    if ii < numel(stateOrder)
        set(gca, 'XTickLabel', []);
    end
end
xlabel(tl4, 'Time (s)');
title(tl4, sprintf('%s Detection and INS-Takeover Timeline', scenarioLabel), 'Interpreter', 'none');
fp = fullfile(outDir, [baseName, '_timeline.png']);
exportgraphics(f4, fp, 'Resolution', 180);
close(f4);
figPaths{end + 1} = fp; %#ok<AGROW>

% 5) Residual heatmap
f5 = figure('Visible', 'off');
imagesc(t, 1 : size(residualMat, 1), residualMat);
axis xy;
grid on;
colorbar;
xlabel('Time (s)');
ylabel('PRN');
title(sprintf('%s - %s', scenarioLabel, residualLabel), 'Interpreter', 'none');
if numel(prnList) == size(residualMat, 1)
    set(gca, 'YTick', 1 : numel(prnList), ...
             'YTickLabel', compose('PRN %d', prnList(:)));
end
fp = fullfile(outDir, [baseName, '_doppler_residual.png']);
exportgraphics(f5, fp, 'Resolution', 180);
close(f5);
figPaths{end + 1} = fp; %#ok<AGROW>

% 6) Error relative to selected reference
f6 = figure('Visible', 'off');
tl6 = tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
plot(tCompare, err3dRef, 'b-', 'LineWidth', 1.1);
grid on;
localAddAlarmLine(alarmTime);
ylabel('3D Error (m)');
title(sprintf('Relative-to-%s 3D Position Error', errLabel), 'Interpreter', 'none');

nexttile;
plot(tCompare, errHRef, 'Color', [0 0.5 0], 'LineWidth', 1.1);
grid on;
localAddAlarmLine(alarmTime);
xlabel('Time (s)');
ylabel('Horizontal Error (m)');
title(sprintf('Relative-to-%s Horizontal Error', errLabel), 'Interpreter', 'none');

title(tl6, sprintf('%s Error Relative to %s', scenarioLabel, mainRefLabel), 'Interpreter', 'none');
fp = fullfile(outDir, sprintf('%s_error_vs_%s.png', baseName, errFileSuffix));
exportgraphics(f6, fp, 'Resolution', 180);
close(f6);
figPaths{end + 1} = fp; %#ok<AGROW>

% 7) XYZ comparison against clean reference
f7 = figure('Visible', 'off');
tl7 = tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
stateNames = {'X', 'Y', 'Z'};
stateYLabels = {'X (m)', 'Y (m)', 'Z (m)'};
for ii = 1 : 3
    nexttile;
    plot(tCompare, mainRefNavResults.(stateNames{ii})(refIdx), 'k--', 'LineWidth', 1.0);
    hold on;
    grid on;
    plot(tCompare, navResults.(stateNames{ii})(navIdx), 'b-', 'LineWidth', 1.1);
    localAddAlarmLine(alarmTime);
    ylabel(stateYLabels{ii});
    title(sprintf('%s State Comparison', stateNames{ii}));
    if ii == 1
        legend({mainRefLabel, scenarioLabel}, 'Location', 'best', 'Interpreter', 'none');
    end
end
xlabel(tl7, 'Time (s)');
title(tl7, sprintf('%s vs %s: X / Y / Z', scenarioLabel, mainRefLabel), 'Interpreter', 'none');
fp = fullfile(outDir, [baseName, '_xyz_vs_cleandynamic.png']);
exportgraphics(f7, fp, 'Resolution', 180);
close(f7);
figPaths{end + 1} = fp; %#ok<AGROW>

% 8) Clock bias and drift comparison
f8 = figure('Visible', 'off');
tl8 = tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
hold on;
grid on;
if isfield(mainRefNavResults, 'dt') && any(isfinite(mainRefNavResults.dt(refIdx)))
    plot(tCompare, mainRefNavResults.dt(refIdx), 'k--', 'LineWidth', 1.0);
end
plot(tCompare, navResults.dt(navIdx), 'b-', 'LineWidth', 1.1);
localAddAlarmLine(alarmTime);
ylabel('Clock Bias (m)');
title('Receiver Clock Bias Comparison');
if isfield(mainRefNavResults, 'dt') && any(isfinite(mainRefNavResults.dt(refIdx)))
    legend({mainRefLabel, scenarioLabel}, 'Location', 'best', 'Interpreter', 'none');
else
    legend({scenarioLabel}, 'Location', 'best', 'Interpreter', 'none');
end

nexttile;
hold on;
grid on;
if isfield(mainRefNavResults, 'df') && any(isfinite(mainRefNavResults.df(refIdx)))
    plot(tCompare, mainRefNavResults.df(refIdx), 'k--', 'LineWidth', 1.0);
end
plot(tCompare, navResults.df(navIdx), 'b-', 'LineWidth', 1.1);
localAddAlarmLine(alarmTime);
xlabel('Time (s)');
ylabel('Clock Drift (m/s)');
title('Receiver Clock Drift Comparison');

title(tl8, sprintf('%s vs %s: Clock Bias / Drift', scenarioLabel, mainRefLabel), 'Interpreter', 'none');
fp = fullfile(outDir, [baseName, '_clock_vs_cleandynamic.png']);
exportgraphics(f8, fp, 'Resolution', 180);
close(f8);
figPaths{end + 1} = fp; %#ok<AGROW>

% 9) Doppler comparison
if isfield(navResults, 'dopplerGpsMeasHz') && isfield(navResults, 'dopplerInsPredHz')
    dopPlot = compute_doppler_plot_series(navResults, settings);
    gpsMed = median(dopPlot.gpsRawHz, 1, 'omitnan');
    insMed = median(dopPlot.insAlignedHz, 1, 'omitnan');
    supMed = median(dopPlot.supAlignedHz, 1, 'omitnan');

    f9 = figure('Visible', 'off');
    tl9 = tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    nexttile;
    plot(t, gpsMed, 'k-', 'LineWidth', 1.1);
    hold on;
    grid on;
    plot(t, insMed, 'b-', 'LineWidth', 1.1);
    localAddAlarmLine(alarmTime);
    ylabel('Doppler (Hz)');
    title('GPS vs INS Doppler');
    legend({sprintf('%s GPS', scenarioLabel), 'INS Prediction'}, 'Location', 'best', 'Interpreter', 'none');

    nexttile;
    plot(t, gpsMed, 'k-', 'LineWidth', 1.1);
    hold on;
    grid on;
    plot(t, supMed, 'r-', 'LineWidth', 1.1);
    localAddAlarmLine(alarmTime);
    xlabel('Time (s)');
    ylabel('Doppler (Hz)');
    title('GPS vs Suppressed Doppler');
    legend({sprintf('%s GPS', scenarioLabel), 'Suppressed'}, 'Location', 'best', 'Interpreter', 'none');

    title(tl9, sprintf('%s Median Doppler Comparison', scenarioLabel), 'Interpreter', 'none');
    fp = fullfile(outDir, [baseName, '_doppler_compare.png']);
    exportgraphics(f9, fp, 'Resolution', 180);
    close(f9);
    figPaths{end + 1} = fp; %#ok<AGROW>

    satNum = size(navResults.dopplerGpsMeasHz, 1);
    finiteCount = sum(isfinite(navResults.dopplerGpsMeasHz), 2);
    [~, ord] = sort(finiteCount, 'descend');
    selIdx = ord(1 : min(4, satNum));
    for kk = 1 : numel(selIdx)
        idxSat = selIdx(kk);
        prn = prnList(min(idxSat, numel(prnList)));
        f6p = figure('Visible', 'off');
        tl6p = tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

        nexttile;
        plot(t, dopPlot.gpsRawHz(idxSat, :), 'k-', 'LineWidth', 1.1);
        hold on;
        grid on;
        plot(t, dopPlot.insAlignedHz(idxSat, :), 'b-', 'LineWidth', 1.1);
        localAddAlarmLine(alarmTime);
        ylabel('Doppler (Hz)');
        title(sprintf('%s PRN %d vs INS', scenarioLabel, prn), 'Interpreter', 'none');
        legend({sprintf('%s PRN %d', scenarioLabel, prn), 'INS'}, 'Location', 'best', 'Interpreter', 'none');

        nexttile;
        plot(t, dopPlot.gpsRawHz(idxSat, :), 'k-', 'LineWidth', 1.1);
        hold on;
        grid on;
        plot(t, dopPlot.supAlignedHz(idxSat, :), 'r-', 'LineWidth', 1.1);
        localAddAlarmLine(alarmTime);
        xlabel('Time (s)');
        ylabel('Doppler (Hz)');
        title(sprintf('%s PRN %d vs Suppressed', scenarioLabel, prn), 'Interpreter', 'none');
        legend({sprintf('%s PRN %d', scenarioLabel, prn), 'Suppressed'}, 'Location', 'best', 'Interpreter', 'none');

        title(tl6p, sprintf('%s Doppler Comparison - PRN %d', scenarioLabel, prn), 'Interpreter', 'none');
        fp = fullfile(outDir, sprintf('%s_doppler_compare_prn%02d.png', baseName, prn));
        exportgraphics(f6p, fp, 'Resolution', 180);
        close(f6p);
        figPaths{end + 1} = fp; %#ok<AGROW>
    end
end

fprintf('\nSaved figures:\n');
for ii = 1 : numel(figPaths)
    fprintf('  %s\n', figPaths{ii});
end
fprintf('\n');
end

function scenarioLabel = localBuildScenarioLabel(summary, resultFile)
scenarioLabel = 'Result';
if isstruct(summary) && isfield(summary, 'datasetTag') && isfield(summary, 'profile')
    scenarioLabel = sprintf('%s / %s', upper(char(summary.datasetTag)), upper(char(summary.profile)));
elseif isstruct(summary) && isfield(summary, 'datasetTag')
    scenarioLabel = upper(char(summary.datasetTag));
else
    [~, scenarioLabel, ~] = fileparts(resultFile);
end
end

function [refNavResults, refLabel, navIdx, refIdx, refKind] = localLoadMainReferenceNavResults(summary, navResults, settings, resultFile)
refNavResults = navResults;
refLabel = 'Actual Trajectory (cleandynamic)';
refKind = 'cleandynamic_raw';
navIdx = 1 : numel(navResults.X);
refIdx = 1 : numel(refNavResults.X);

projectRoot = fileparts(fileparts(mfilename('fullpath')));
[rawCleanNavResults, okRawClean] = localLoadRawCleanNavResults(projectRoot);
if okRawClean
    refNavResults = rawCleanNavResults;
    [scenarioTime, okScenarioTime] = localLoadScenarioTimeVector(summary, numel(navResults.X));
    if okScenarioTime && isfield(refNavResults, 'receiverTime') && ~isempty(refNavResults.receiverTime)
        [navIdxTime, refIdxTime, okTimeAlign] = localAlignedIndexPairByTime( ...
            scenarioTime, refNavResults.receiverTime(:)', settings.navSolPeriod / 1000);
        if okTimeAlign
            navIdx = navIdxTime;
            refIdx = refIdxTime;
            return;
        end
    end
else
    refKind = 'clean_realtime_baseline';
    refLabel = 'Clean Baseline';
    if isstruct(summary) && isfield(summary, 'referenceFile') && ~isempty(summary.referenceFile)
        refFile = char(summary.referenceFile);
        if exist(refFile, 'file') == 2
            R = load(refFile, 'navResults');
            if isfield(R, 'navResults')
                refNavResults = R.navResults;
            end
        end
    end
end

if isstruct(summary) && isfield(summary, 'datasetTag') && strcmpi(char(summary.datasetTag), 'clean')
    refLabel = 'Actual Trajectory (cleandynamic)';
end

refEpochOffset = 0;
if isstruct(summary) && isfield(summary, 'referenceEpochOffset') && ~isempty(summary.referenceEpochOffset)
    refEpochOffset = summary.referenceEpochOffset;
end
[navIdx, refIdx] = localAlignedIndexPair(numel(navResults.X), numel(refNavResults.X), refEpochOffset);
end

function [refNavResults, ok] = localLoadRawCleanNavResults(projectRoot)
refNavResults = struct();
ok = false;

rawCleanFile = fullfile(projectRoot, 'cleandynamic_400s.mat');
if exist(rawCleanFile, 'file') ~= 2
    return;
end

S = load(rawCleanFile, 'navSolutions');
if ~isfield(S, 'navSolutions')
    return;
end

refNavResults = S.navSolutions;
ok = all(isfield(refNavResults, {'X', 'Y', 'Z'}));
end

function [scenarioTime, ok] = localLoadScenarioTimeVector(summary, targetLen)
scenarioTime = [];
ok = false;
if ~(isstruct(summary) && isfield(summary, 'dataset') && ~isempty(summary.dataset))
    return;
end

scenarioFile = char(summary.dataset);
if exist(scenarioFile, 'file') ~= 2
    return;
end

S = load(scenarioFile, 'navSolutions');
if ~isfield(S, 'navSolutions') || ~isfield(S.navSolutions, 'receiverTime') || isempty(S.navSolutions.receiverTime)
    return;
end

rawTime = S.navSolutions.receiverTime(:)';
[scenarioTime, ok] = localResizeTimeVector(rawTime, targetLen);
end

function [timeVec, ok] = localResizeTimeVector(rawTime, targetLen)
timeVec = [];
ok = false;
rawLen = numel(rawTime);
if rawLen == targetLen
    timeVec = rawTime;
    ok = true;
elseif rawLen == targetLen + 1
    % Tight-coupled results usually start one navigation epoch after the
    % first standalone PVT epoch stored in navSolutions.
    timeVec = rawTime(2:end);
    ok = true;
elseif rawLen > targetLen
    timeVec = rawTime((rawLen - targetLen + 1):end);
    ok = true;
end
end

function [navIdx, refIdx, ok] = localAlignedIndexPairByTime(navTime, refTime, tolSec)
navIdx = [];
refIdx = [];
ok = false;
if nargin < 3 || isempty(tolSec)
    tolSec = 0.30;
end
if isempty(navTime) || isempty(refTime)
    return;
end

refQuery = interp1(refTime, 1:numel(refTime), navTime, 'nearest', nan);
refQuery = round(refQuery);
valid = isfinite(refQuery) & refQuery >= 1 & refQuery <= numel(refTime);
if any(valid)
    deltaTime = nan(size(refQuery));
    deltaTime(valid) = abs(refTime(refQuery(valid)) - navTime(valid));
    valid = valid & deltaTime <= tolSec;
end
if ~any(valid)
    return;
end

navIdx = find(valid);
refIdx = refQuery(valid);
ok = true;
end

function [refNavResults, refLabel, navIdx, refIdx, ok] = localLoadTruthReferenceNavResults(summary, navResults, settings)
refNavResults = navResults;
refLabel = 'Trajectory Truth';
navIdx = 1 : numel(navResults.X);
refIdx = 1 : numel(navResults.X);
ok = false;
if ~(isstruct(summary) && isfield(summary, 'truthReferenceFile') && ~isempty(summary.truthReferenceFile) && ...
        isfield(summary, 'truthStartOffsetSec'))
    return;
end
[refNavResults, ok] = localBuildTruthReference(summary, navResults, settings);
if ok
    navIdx = 1 : numel(navResults.X);
    refIdx = 1 : numel(refNavResults.X);
end
end

function [refNavResults, ok] = localBuildTruthReference(summary, navResults, settings)
ok = false;
refNavResults = navResults;

trjFile = char(summary.truthReferenceFile);
if exist(trjFile, 'file') ~= 2
    return;
end

trjData = load(trjFile, 'trj');
if ~isfield(trjData, 'trj')
    return;
end
trj = trjData.trj;
if ~isfield(summary, 'truthStartOffsetSec')
    return;
end

navPeriodSec = settings.navSolPeriod / 1000;
tQuery = double(summary.truthStartOffsetSec) + (1 : numel(navResults.X)) * navPeriodSec;
tAvp = trj.avp(:, end);

[Xr, Yr, Zr] = localBlhToXyzWgs84(trj.avp(:, 7), trj.avp(:, 8), trj.avp(:, 9));
refNavResults = struct();
refNavResults.X = interp1(tAvp, Xr, tQuery, 'linear', 'extrap');
refNavResults.Y = interp1(tAvp, Yr, tQuery, 'linear', 'extrap');
refNavResults.Z = interp1(tAvp, Zr, tQuery, 'linear', 'extrap');
refNavResults.dt = nan(size(refNavResults.X));
refNavResults.df = nan(size(refNavResults.X));
ok = true;
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

function [err3d, errH, errLabel, errFileSuffix] = ...
    localComputeReferenceErrors(navResults, refNavResults, navIdx, refIdx, refKind)
compareLen = numel(navIdx);
if compareLen <= 0
    err3d = nan(1, 0);
    errH = nan(1, 0);
else
    dx = navResults.X(navIdx) - refNavResults.X(refIdx);
    dy = navResults.Y(navIdx) - refNavResults.Y(refIdx);
    dz = navResults.Z(navIdx) - refNavResults.Z(refIdx);
    err3d = sqrt(dx.^2 + dy.^2 + dz.^2);
    errH = nan(1, compareLen);
    for ii = 1 : compareLen
        [latDeg, lonDeg, ~] = cart2geo(refNavResults.X(refIdx(ii)), ...
            refNavResults.Y(refIdx(ii)), refNavResults.Z(refIdx(ii)), 5);
        lat = deg2rad(latDeg);
        lon = deg2rad(lonDeg);
        eErr = -sin(lon) * dx(ii) + cos(lon) * dy(ii);
        nErr = -sin(lat) * cos(lon) * dx(ii) - sin(lat) * sin(lon) * dy(ii) + cos(lat) * dz(ii);
        errH(ii) = hypot(eErr, nErr);
    end
end

switch lower(refKind)
    case 'cleandynamic_raw'
        errLabel = 'Actual Trajectory (cleandynamic)';
        errFileSuffix = 'cleandynamic';
    otherwise
        errLabel = 'Clean Baseline';
        errFileSuffix = 'cleanbaseline';
end
end

function [E, N, U, Er, Nr, Ur] = localComputeEnuPair(navResults, refNavResults, navIdx, refIdx)
compareLen = numel(navIdx);
if compareLen <= 0
    E = nan(size(navResults.X));
    N = nan(size(navResults.Y));
    U = nan(size(navResults.Z));
    Er = E;
    Nr = N;
    Ur = U;
    return;
end

validXYZ = isfinite(refNavResults.X(refIdx)) & isfinite(refNavResults.Y(refIdx)) & isfinite(refNavResults.Z(refIdx));
if ~any(validXYZ)
    E = nan(1, compareLen);
    N = nan(1, compareLen);
    U = nan(1, compareLen);
    Er = E;
    Nr = N;
    Ur = U;
    return;
end

idx0 = find(validXYZ, 1, 'first');
ref0 = refIdx(idx0);
X0 = refNavResults.X(ref0);
Y0 = refNavResults.Y(ref0);
Z0 = refNavResults.Z(ref0);
[lat0Deg, lon0Deg, ~] = cart2geo(X0, Y0, Z0, 5);
lat0 = deg2rad(lat0Deg);
lon0 = deg2rad(lon0Deg);
R = [-sin(lon0),                cos(lon0),               0; ...
     -sin(lat0) * cos(lon0), -sin(lat0) * sin(lon0),  cos(lat0); ...
      cos(lat0) * cos(lon0),  cos(lat0) * sin(lon0),  sin(lat0)];

dX = navResults.X(navIdx) - X0;
dY = navResults.Y(navIdx) - Y0;
dZ = navResults.Z(navIdx) - Z0;
enu = R * [dX; dY; dZ];
E = enu(1, :);
N = enu(2, :);
U = enu(3, :);

dXr = refNavResults.X(refIdx) - X0;
dYr = refNavResults.Y(refIdx) - Y0;
dZr = refNavResults.Z(refIdx) - Z0;
enur = R * [dXr; dYr; dZr];
Er = enur(1, :);
Nr = enur(2, :);
Ur = enur(3, :);
end

function [Erel, Nrel, ErelRef, NrelRef, relMask] = localComputeAlarmRelativeEnu(E, N, Er, Nr, navIdx, alarmIdx)
Erel = E;
Nrel = N;
ErelRef = Er;
NrelRef = Nr;
relMask = true(size(E));
if isempty(E) || isempty(navIdx)
    relMask = false(size(E));
    return;
end

if isempty(alarmIdx)
    localIdx = 1;
else
    localIdx = find(navIdx >= alarmIdx, 1, 'first');
    if isempty(localIdx)
        localIdx = 1;
    end
end

Erel = E - E(localIdx);
Nrel = N - N(localIdx);
ErelRef = Er - Er(localIdx);
NrelRef = Nr - Nr(localIdx);
relMask = false(size(E));
relMask(localIdx:end) = true;
end

function [residualMat, residualLabel] = localSelectResidualMatrix(navResults)
if isfield(navResults, 'dopplerResidualTemplateDevHz') && any(isfinite(navResults.dopplerResidualTemplateDevHz(:)))
    residualMat = navResults.dopplerResidualTemplateDevHz;
    residualLabel = 'Doppler Residual Deviation from Clean Template (Hz)';
elseif isfield(navResults, 'dopplerResidualBiasCorrectedHz') && any(isfinite(navResults.dopplerResidualBiasCorrectedHz(:)))
    residualMat = navResults.dopplerResidualBiasCorrectedHz;
    residualLabel = 'Bias-Corrected Doppler Residual (Hz)';
elseif isfield(navResults, 'dopplerResidualRawHz')
    residualMat = navResults.dopplerResidualRawHz;
    residualLabel = 'Raw Doppler Residual (Hz)';
else
    residualMat = nan(1, numel(navResults.X));
    residualLabel = 'Doppler Residual';
end
end

function prnList = localPickPrnList(navResults)
if isfield(navResults, 'prnList') && ~isempty(navResults.prnList)
    prnList = navResults.prnList(:)';
else
    prnList = 1 : size(navResults.dopplerGpsMeasHz, 1);
end
end

function localAddAlarmLine(alarmTime)
if ~isempty(alarmTime)
    xline(alarmTime, 'm--', 'LineWidth', 1.0);
end
end

function err = localGetErrorField(navResults, fieldName, compareLen)
if isfield(navResults, fieldName) && ~isempty(navResults.(fieldName))
    err = navResults.(fieldName);
    err = err(1:min(compareLen, numel(err)));
else
    err = nan(1, compareLen);
end
err = err(:)';
end

function [navIdx, refIdx] = localAlignedIndexPair(navLen, refLen, refEpochOffset)
navStart = 1 + max(-refEpochOffset, 0);
refStart = 1 + max(refEpochOffset, 0);
pairLen = min(navLen - navStart + 1, refLen - refStart + 1);
if pairLen <= 0
    error('No overlapping epochs remain after applying reference offset %d.', refEpochOffset);
end
navIdx = navStart : (navStart + pairLen - 1);
refIdx = refStart : (refStart + pairLen - 1);
end
