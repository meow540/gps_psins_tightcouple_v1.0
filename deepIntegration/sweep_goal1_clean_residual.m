function results = sweep_goal1_clean_residual(dataFile)
% Quick tuning sweep on the clean head segment.
% Focus: residual size vs available satellite count.
%
% Usage:
%   sweep_goal1_clean_residual
%   sweep_goal1_clean_residual('ds5_rollback_200s.mat')

if nargin < 1 || isempty(dataFile)
    dataFile = 'ds5_rollback_200s.mat';
end

thisDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(thisDir);
addpath(fullfile(projectRoot, 'include'))
addpath(fullfile(projectRoot, 'geoFunctions'))
addpath(fullfile(projectRoot, 'acquire_zcj'))
addpath(fullfile(projectRoot, 'track_zcj'))
addpath(fullfile(projectRoot, 'PVT'))
addpath(thisDir)

if ~exist(dataFile, 'file')
    altPath = fullfile(projectRoot, dataFile);
    if exist(altPath, 'file')
        dataFile = altPath;
    else
        error('Missing dataset: %s', dataFile);
    end
end

S = load(dataFile, 'trackResults', 'channel', 'TOW', 'eph', 'subFrameStart');

cfg = struct( ...
    'name', {'base_15_25_1', 'tuned_20_15_3', 'tuned_22_15_3', 'tuned_25_15_3', 'aggr_20_13_5'}, ...
    'minElevDeg', {15, 20, 22, 25, 20}, ...
    'pllBwHz', {25, 15, 15, 15, 13}, ...
    'smoothWin', {1, 3, 3, 3, 5});

results = struct([]);
for ii = 1 : numel(cfg)
    trackResults = S.trackResults; %#ok<NASGU>
    channel = S.channel; %#ok<NASGU>
    TOW = S.TOW; %#ok<NASGU>
    eph = S.eph; %#ok<NASGU>
    subFrameStart = S.subFrameStart; %#ok<NASGU>

    settingsOverride = struct(); %#ok<NASGU>
    settingsOverride.msToProcess = 20 * 1000;
    settingsOverride.plotTracking = 0;
    settingsOverride.skipQuickPlot = 1;
    settingsOverride.verboseEpochPrint = 0;
    settingsOverride.spoofDetArmTimeSec = 80;
    settingsOverride.spoofDetConfirmEpochs = 2;
    settingsOverride.spoofMitigationMode = 'robust_tc';
    settingsOverride.spoofDetMinElevDeg = cfg(ii).minElevDeg;
    settingsOverride.pllNoiseBandwidth = cfg(ii).pllBwHz;
    settingsOverride.spoofDetMetricSmoothWin = cfg(ii).smoothWin;

    realtime_tightCouple;

    gpsMed = median(navResults.dopplerGpsMeasHz, 1, 'omitnan');
    insMed = median(navResults.dopplerInsPredHz, 1, 'omitnan');
    row = struct();
    row.name = cfg(ii).name;
    row.minElevDeg = cfg(ii).minElevDeg;
    row.pllBwHz = cfg(ii).pllBwHz;
    row.smoothWin = cfg(ii).smoothWin;
    row.epochs = numel(navResults.X);
    row.rawGapMedianHz = median(abs(gpsMed - insMed), 'omitnan');
    row.residualMedianHz = median(abs(navResults.dopplerResidualHz), 'all', 'omitnan');
    row.metricMedianHz = median(navResults.spoofMetricHz, 'omitnan');
    row.validSatMin = min(navResults.spoofValidSatNum);
    row.validSatMedian = median(navResults.spoofValidSatNum);
    row.gnssUsedEpochs = sum(navResults.gnssUpdateUsed);
    results = [results; row]; %#ok<AGROW>
end

T = struct2table(results);
disp(T);
end
