function outPath = plot_clean_spoof_tc_compare(resultFile)
% Plot raw clean trajectory, raw spoofed trajectory, and tight-coupled result together.

if nargin < 1 || isempty(resultFile)
    resultFile = 'rt_tight_goal1_ds5_400s_unified_ins1.mat';
end

thisDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(thisDir);
addpath(fullfile(projectRoot, 'geoFunctions'));

if exist(resultFile, 'file') ~= 2
    altFile = fullfile(projectRoot, resultFile);
    if exist(altFile, 'file') ~= 2
        error('Result file not found: %s', resultFile);
    end
    resultFile = altFile;
end

R = load(resultFile, 'navResults', 'summary');
if ~isfield(R, 'navResults') || ~isfield(R, 'summary')
    error('Result MAT must contain navResults and summary.');
end

if ~isfield(R.summary, 'dataset') || isempty(R.summary.dataset)
    error('Result summary does not contain the raw scenario dataset path.');
end

rawScenarioFile = char(R.summary.dataset);
if exist(rawScenarioFile, 'file') ~= 2
    error('Raw scenario MAT not found: %s', rawScenarioFile);
end

cleanFile = fullfile(projectRoot, 'cleandynamic_400s.mat');
if exist(cleanFile, 'file') ~= 2
    error('Missing clean reference MAT: %s', cleanFile);
end

rawScenario = load(rawScenarioFile, 'navSolutions');
rawClean = load(cleanFile, 'navSolutions');
if ~isfield(rawScenario, 'navSolutions') || ~isfield(rawClean, 'navSolutions')
    error('Both scenario MAT and cleandynamic MAT must contain navSolutions.');
end

scenarioNav = rawScenario.navSolutions;
cleanNav = rawClean.navSolutions;
tcNav = R.navResults;

scenarioTime = localGetTimeVectorForNavResult(scenarioNav, numel(tcNav.X));
rawTime = scenarioNav.receiverTime(:)';
cleanTime = cleanNav.receiverTime(:)';

[rawNavIdx, rawCleanIdx, rawOk] = localAlignedIndexPairByTime(rawTime, cleanTime, 0.30);
[tcNavIdx, tcCleanIdx, tcOk] = localAlignedIndexPairByTime(scenarioTime, cleanTime, 0.30);
if ~rawOk || ~tcOk
    error('Failed to align scenario/result time with cleandynamic receiverTime.');
end

originRefIdx = min([rawCleanIdx(1), tcCleanIdx(1)]);
[Ec, Nc] = localComputeEnu(cleanNav.X, cleanNav.Y, cleanNav.Z, cleanNav.X(originRefIdx), cleanNav.Y(originRefIdx), cleanNav.Z(originRefIdx));
[Er, Nr] = localComputeEnu(scenarioNav.X(rawNavIdx), scenarioNav.Y(rawNavIdx), scenarioNav.Z(rawNavIdx), ...
    cleanNav.X(originRefIdx), cleanNav.Y(originRefIdx), cleanNav.Z(originRefIdx));
[Et, Nt] = localComputeEnu(tcNav.X(tcNavNavIdx(tcNavIdx)), tcNav.Y(tcNavNavIdx(tcNavIdx)), tcNav.Z(tcNavNavIdx(tcNavIdx)), ...
    cleanNav.X(originRefIdx), cleanNav.Y(originRefIdx), cleanNav.Z(originRefIdx));

cleanPlotIdx = originRefIdx : max([rawCleanIdx(end), tcCleanIdx(end)]);
cleanE = Ec(cleanPlotIdx);
cleanN = Nc(cleanPlotIdx);

[~, baseName, ~] = fileparts(resultFile);
outDir = fullfile(projectRoot, 'deepIntegration', 'figures');
if exist(outDir, 'dir') ~= 7
    mkdir(outDir);
end

scenarioTag = localGetScenarioTag(R.summary, rawScenarioFile);
profileTag = localGetProfileTag(R.summary);
tcLabel = sprintf('%s / %s', upper(scenarioTag), upper(profileTag));
rawLabel = sprintf('Pure %s', upper(scenarioTag));

f = figure('Visible', 'off');
plot(cleanE, cleanN, 'k--', 'LineWidth', 1.1);
hold on;
plot(Er, Nr, '-', 'Color', [0.85 0.33 0.10], 'LineWidth', 1.2);
plot(Et, Nt, 'b-', 'LineWidth', 1.2);
grid on;
axis equal;
xlabel('East (m)');
ylabel('North (m)');
title(sprintf('%s Trajectory Comparison in ENU', upper(scenarioTag)), 'Interpreter', 'none');
legend({'Actual Trajectory (cleandynamic)', rawLabel, tcLabel}, ...
    'Location', 'best', 'Interpreter', 'none');

alarmIdx = find(tcNav.spoofAlarm, 1, 'first');
if ~isempty(alarmIdx) && alarmIdx <= numel(Et)
    plot(Et(alarmIdx), Nt(alarmIdx), 'mo', 'MarkerSize', 7, 'LineWidth', 1.1);
end

outPath = fullfile(outDir, [baseName, '_traj_threeway_clean_raw_tc.png']);
exportgraphics(f, outPath, 'Resolution', 180);
close(f);

fprintf('Saved figure:\n  %s\n', outPath);
end

function scenarioTime = localGetTimeVectorForNavResult(scenarioNav, targetLen)
rawTime = scenarioNav.receiverTime(:)';
if numel(rawTime) == targetLen
    scenarioTime = rawTime;
elseif numel(rawTime) == targetLen + 1
    scenarioTime = rawTime(2:end);
elseif numel(rawTime) > targetLen
    scenarioTime = rawTime((numel(rawTime) - targetLen + 1):end);
else
    error('Raw scenario receiverTime length (%d) is shorter than navResults length (%d).', numel(rawTime), targetLen);
end
end

function [navIdx, refIdx, ok] = localAlignedIndexPairByTime(navTime, refTime, tolSec)
navIdx = [];
refIdx = [];
ok = false;
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

function [E, N] = localComputeEnu(X, Y, Z, X0, Y0, Z0)
[lat0Deg, lon0Deg, ~] = cart2geo(X0, Y0, Z0, 5);
lat0 = deg2rad(lat0Deg);
lon0 = deg2rad(lon0Deg);
R = [-sin(lon0),                cos(lon0),               0; ...
     -sin(lat0) * cos(lon0), -sin(lat0) * sin(lon0),  cos(lat0); ...
      cos(lat0) * cos(lon0),  cos(lat0) * sin(lon0),  sin(lat0)];

dX = X(:)' - X0;
dY = Y(:)' - Y0;
dZ = Z(:)' - Z0;
enu = R * [dX; dY; dZ];
E = enu(1, :);
N = enu(2, :);
end

function scenarioTag = localGetScenarioTag(summary, rawScenarioFile)
scenarioTag = '';
if isstruct(summary) && isfield(summary, 'datasetTag') && ~isempty(summary.datasetTag)
    scenarioTag = char(summary.datasetTag);
else
    [~, scenarioTag, ~] = fileparts(rawScenarioFile);
    scenarioTag = regexprep(scenarioTag, '_400s$', '');
end
end

function profileTag = localGetProfileTag(summary)
profileTag = 'ins';
if isstruct(summary) && isfield(summary, 'profile') && ~isempty(summary.profile)
    profileTag = char(summary.profile);
end
end

function idx = tcNavNavIdx(navIdx)
idx = navIdx;
end
