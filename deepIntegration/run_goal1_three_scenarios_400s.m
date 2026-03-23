function result = run_goal1_three_scenarios_400s(profile, opts)
% Run clean/ds5/ds6 full 400 s with unified Chapter-1 settings and
% generate one ENU trajectory comparison plot.

if nargin < 1 || isempty(profile)
    profile = 'ins1';
end
if nargin < 2 || isempty(opts)
    opts = struct();
end
if ~isfield(opts, 'useOfflineReplay'), opts.useOfflineReplay = 1; end
if ~isfield(opts, 'msToProcess'), opts.msToProcess = 399 * 1000; end
if ~isfield(opts, 'extraSettings'), opts.extraSettings = struct(); end
if ~isfield(opts, 'baselineOpts'), opts.baselineOpts = struct(); end

thisDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(thisDir);
addpath(fullfile(projectRoot, 'geoFunctions'));

runOpts = struct( ...
    'useOfflineReplay', opts.useOfflineReplay, ...
    'msToProcess', opts.msToProcess, ...
    'extraSettings', opts.extraSettings, ...
    'baselineOpts', opts.baselineOpts);

summaryClean = run_goal1_unified_400s_insonly('clean', profile, runOpts);
summaryDs5 = run_goal1_unified_400s_insonly('ds5', profile, runOpts);
summaryDs6 = run_goal1_unified_400s_insonly('ds6', profile, runOpts);

Rclean = load(summaryClean.output, 'navResults');
Rds5 = load(summaryDs5.output, 'navResults');
Rds6 = load(summaryDs6.output, 'navResults');
clean = Rclean.navResults;
ds5 = Rds5.navResults;
ds6 = Rds6.navResults;

[x0, y0, z0] = localPickOrigin(clean.X, clean.Y, clean.Z);
[Eclean, Nclean] = localComputeEnu(clean.X, clean.Y, clean.Z, x0, y0, z0);
[Eds5, Nds5] = localComputeEnu(ds5.X, ds5.Y, ds5.Z, x0, y0, z0);
[Eds6, Nds6] = localComputeEnu(ds6.X, ds6.Y, ds6.Z, x0, y0, z0);

outDir = fullfile(projectRoot, 'deepIntegration', 'figures');
if exist(outDir, 'dir') ~= 7
    mkdir(outDir);
end
outFig = fullfile(outDir, sprintf('goal1_three_scenarios_400s_%s_enu.png', lower(profile)));

f = figure('Visible', 'off');
plot(Eclean, Nclean, 'k-', 'LineWidth', 1.3); hold on;
plot(Eds5, Nds5, 'b-', 'LineWidth', 1.2);
plot(Eds6, Nds6, 'r-', 'LineWidth', 1.2);
grid on; axis equal;
xlabel('East (m)');
ylabel('North (m)');
title(sprintf('Goal-1 Trajectory Comparison (400 s, %s)', upper(profile)), 'Interpreter', 'none');
legend({'clean', 'ds5 / INS tight-coupled', 'ds6 / INS tight-coupled'}, ...
    'Location', 'best', 'Interpreter', 'none');
exportgraphics(f, outFig, 'Resolution', 180);
close(f);

result = struct();
result.profile = char(profile);
result.cleanOutput = summaryClean.output;
result.ds5Output = summaryDs5.output;
result.ds6Output = summaryDs6.output;
result.figure = outFig;
result.cleanEpochs = numel(clean.X);
result.ds5Epochs = numel(ds5.X);
result.ds6Epochs = numel(ds6.X);

fprintf('\n==== Three-Scenario 400s Plot ====\n');
fprintf('Profile: %s\n', upper(char(profile)));
fprintf('Clean output: %s\n', result.cleanOutput);
fprintf('DS5 output  : %s\n', result.ds5Output);
fprintf('DS6 output  : %s\n', result.ds6Output);
fprintf('Figure      : %s\n', result.figure);
fprintf('Epochs      : clean=%d, ds5=%d, ds6=%d\n', ...
    result.cleanEpochs, result.ds5Epochs, result.ds6Epochs);
fprintf('==================================\n\n');
end

function [x0, y0, z0] = localPickOrigin(X, Y, Z)
idx = find(isfinite(X) & isfinite(Y) & isfinite(Z), 1, 'first');
if isempty(idx)
    error('Clean trajectory has no finite XYZ samples.');
end
x0 = X(idx);
y0 = Y(idx);
z0 = Z(idx);
end

function [E, N] = localComputeEnu(X, Y, Z, x0, y0, z0)
valid = isfinite(X) & isfinite(Y) & isfinite(Z);
X = X(valid);
Y = Y(valid);
Z = Z(valid);
[lat0Deg, lon0Deg, ~] = cart2geo(x0, y0, z0, 5);
lat0 = deg2rad(lat0Deg);
lon0 = deg2rad(lon0Deg);
R = [-sin(lon0),                cos(lon0),               0; ...
     -sin(lat0) * cos(lon0), -sin(lat0) * sin(lon0),  cos(lat0); ...
      cos(lat0) * cos(lon0),  cos(lat0) * sin(lon0),  sin(lat0)];
dX = X(:)' - x0;
dY = Y(:)' - y0;
dZ = Z(:)' - z0;
enu = R * [dX; dY; dZ];
E = enu(1, :);
N = enu(2, :);
end
