function outPaths = plot_clean_vs_ds_deep_pairs(resultFiles, cleanFile)
% Plot clean trajectory vs deep-coupling navigation trajectory (ENU).
%
% Defaults:
%   resultFiles = {'rt_deep_goal2_trackres_final2all_default_ds5_400s.mat', ...
%                  'rt_deep_goal2_trackres_final2all_default_ds6_400s.mat'};
%   cleanFile   = 'cleandynamic_400s.mat';

if nargin < 1 || isempty(resultFiles)
    resultFiles = { ...
        'rt_deep_goal2_trackres_final2all_default_ds5_400s.mat', ...
        'rt_deep_goal2_trackres_final2all_default_ds6_400s.mat'};
end
if ischar(resultFiles) || isstring(resultFiles)
    resultFiles = {char(resultFiles)};
end

if nargin < 2 || isempty(cleanFile)
    cleanFile = 'cleandynamic_400s.mat';
end

thisDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(thisDir);
addpath(fullfile(projectRoot, 'geoFunctions'));

if exist(cleanFile, 'file') ~= 2
    cleanFile = fullfile(projectRoot, cleanFile);
end
Sclean = load(cleanFile, 'navSolutions');
cleanNav = Sclean.navSolutions;
cleanTime = cleanNav.receiverTime(:)';

outDir = fullfile(projectRoot, 'deepIntegration', 'figures');
if exist(outDir, 'dir') ~= 7
    mkdir(outDir);
end

outPaths = cell(1, numel(resultFiles));
for kk = 1:numel(resultFiles)
    rf = resultFiles{kk};
    if exist(rf, 'file') ~= 2
        rf = fullfile(projectRoot, rf);
    end
    R = load(rf, 'navResults', 'tag');
    nav = R.navResults;
    [~, baseName, ~] = fileparts(rf);

    % Determine scenario tag and corresponding scenario MAT for timing.
    if isfield(R, 'tag') && ~isempty(R.tag)
        tag = lower(char(R.tag));
    elseif contains(lower(baseName), 'ds6')
        tag = 'ds6';
    elseif contains(lower(baseName), 'ds5')
        tag = 'ds5';
    else
        error('Cannot infer scenario tag from %s', rf);
    end
    sceneMat = fullfile(projectRoot, [tag '_400s.mat']);
    Sscene = load(sceneMat, 'navSolutions', 'TOW');
    sceneTime = Sscene.navSolutions.receiverTime(:)';
    TOW = Sscene.TOW;

    X = nav.X(:)'; Y = nav.Y(:)'; Z = nav.Z(:)';
    validXYZ = isfinite(X) & isfinite(Y) & isfinite(Z) & ~(X == 0 & Y == 0 & Z == 0);
    X = X(validXYZ); Y = Y(validXYZ); Z = Z(validXYZ);
    n = numel(X);
    if n < 10
        error('Too few valid points in %s', rf);
    end

    % Build deep result time vector.
    if numel(sceneTime) == n
        t = sceneTime;
    elseif numel(sceneTime) == n + 1
        t = sceneTime(2:end);
    elseif numel(sceneTime) > n
        t = sceneTime((numel(sceneTime) - n + 1):end);
    else
        t = TOW + (1:n) * 0.5;
    end

    cleanIdx = round(interp1(cleanTime, 1:numel(cleanTime), t, 'nearest', nan));
    validT = isfinite(cleanIdx) & cleanIdx >= 1 & cleanIdx <= numel(cleanTime);
    dt = nan(size(cleanIdx));
    dt(validT) = abs(cleanTime(cleanIdx(validT)) - t(validT));
    validT = validT & dt <= 1.0;
    cleanIdx = cleanIdx(validT);
    X = X(validT); Y = Y(validT); Z = Z(validT);
    if isempty(cleanIdx)
        error('Time alignment failed for %s', rf);
    end

    ori = cleanIdx(1);
    [Ec, Nc] = localComputeEnu(cleanNav.X, cleanNav.Y, cleanNav.Z, ...
        cleanNav.X(ori), cleanNav.Y(ori), cleanNav.Z(ori));
    [Ed, Nd] = localComputeEnu(X, Y, Z, cleanNav.X(ori), cleanNav.Y(ori), cleanNav.Z(ori));

    cleanPlotIdx = cleanIdx(1):cleanIdx(end);
    figure('Visible', 'off');
    plot(Ec(cleanPlotIdx), Nc(cleanPlotIdx), 'k--', 'LineWidth', 1.2);
    hold on;
    plot(Ed, Nd, 'b-', 'LineWidth', 1.25);
    grid on;
    axis equal;
    xlabel('East (m)');
    ylabel('North (m)');
    title(sprintf('Clean vs %s/INS Deep-Coupling Trajectory (ENU)', upper(tag)), 'Interpreter', 'none');
    legend({'Actual trajectory (clean)', sprintf('%s/INS deep-coupling', upper(tag))}, ...
        'Location', 'best', 'Interpreter', 'none');

    outPath = fullfile(outDir, [baseName '_traj_clean_vs_deep_enu.png']);
    exportgraphics(gcf, outPath, 'Resolution', 180);
    close(gcf);
    outPaths{kk} = outPath;
    fprintf('Saved figure:\n  %s\n', outPath);
end
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
