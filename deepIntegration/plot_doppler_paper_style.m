function figPaths = plot_doppler_paper_style(resultFile, arg2, arg3)
% Paper-style Doppler plots with original Chapter-1 semantics:
%   Top    : GNSS measured Doppler vs INS-generated Doppler
%   Bottom : INS-generated Doppler vs combined-navigation Doppler
%
% For each PRN this function outputs:
%   1) full 400 s figure
%   2) 5 s zoom around spoof onset
%   3) 5 s zoom around mitigation start (alarm)
%
% Backward compatibility:
%   Older calls might pass (spoofResultFile, cleanResultFile, prnList).
%   cleanResultFile is ignored in this version.

if nargin < 1 || isempty(resultFile)
    resultFile = 'rt_tight_goal1_ds5_400s_unified_ins1.mat';
end

prnList = [];
if nargin >= 2 && ~isempty(arg2)
    if isnumeric(arg2)
        prnList = arg2(:)';
    elseif nargin >= 3 && isnumeric(arg3)
        prnList = arg3(:)';
    end
end

thisDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(thisDir);
resultFile = localResolveResultFile(resultFile, projectRoot);

S = load(resultFile);
if ~isfield(S, 'navResults') || ~isfield(S, 'settings')
    error('Result MAT must contain navResults and settings.');
end
navResults = S.navResults;
settings = S.settings;

if ~isfield(navResults, 'dopplerGpsMeasHz') || ~isfield(navResults, 'dopplerInsPredHz')
    error('Result MAT is missing Doppler fields.');
end

dop = compute_doppler_plot_series(navResults, settings);
t = dop.tSec(:)';
gpsHz = dop.gpsRawHz;
insHz = dop.insAlignedHz;
cmbHz = dop.supAlignedHz;

prnPool = localPrnList(navResults);
if isempty(prnList)
    finiteCount = sum(isfinite(gpsHz), 2);
    [~, ord] = sort(finiteCount, 'descend');
    keepN = min(4, numel(ord));
    idxKeep = sort(ord(1:keepN));
    prnList = prnPool(idxKeep);
end

alarmIdx = localFirstAlarmIndex(navResults);
onsetIdx = localFirstOnsetIndex(navResults, settings, alarmIdx);

alarmTime = localIndexToTime(t, alarmIdx);
onsetTime = localIndexToTime(t, onsetIdx);
zoomSec = 5.0;

outDir = fullfile(projectRoot, 'deepIntegration', 'figures');
if exist(outDir, 'dir') ~= 7
    mkdir(outDir);
end

[~, baseName, ~] = fileparts(resultFile);
scenarioLabel = localScenarioLabel(S, baseName);
figPaths = {};

for i = 1 : numel(prnList)
    prn = prnList(i);
    idx = find(prnPool == prn, 1, 'first');
    if isempty(idx)
        warning('PRN %d not available in this result, skipped.', prn);
        continue;
    end

    g = gpsHz(idx, :);
    ins = insHz(idx, :);
    cmb = cmbHz(idx, :);

    outFull = fullfile(outDir, sprintf('%s_doppler_paper_style_prn%02d_full.png', baseName, prn));
    localPlotOneFigure(t, g, ins, cmb, scenarioLabel, prn, ...
        onsetTime, alarmTime, [], outFull, 'Full 400 s');
    figPaths{end + 1} = outFull; %#ok<AGROW>

    if isfinite(onsetTime)
        xlimOnset = [onsetTime - zoomSec / 2, onsetTime + zoomSec / 2];
        xlimOnset = localClampWindow(xlimOnset, t);
        outOnset = fullfile(outDir, sprintf('%s_doppler_paper_style_prn%02d_zoom_onset_5s.png', baseName, prn));
        localPlotOneFigure(t, g, ins, cmb, scenarioLabel, prn, ...
            onsetTime, alarmTime, xlimOnset, outOnset, 'Zoom 5 s Around Spoof Onset');
        figPaths{end + 1} = outOnset; %#ok<AGROW>
    end

    if isfinite(alarmTime)
        xlimAlarm = [alarmTime - zoomSec / 2, alarmTime + zoomSec / 2];
        xlimAlarm = localClampWindow(xlimAlarm, t);
        outAlarm = fullfile(outDir, sprintf('%s_doppler_paper_style_prn%02d_zoom_alarm_5s.png', baseName, prn));
        localPlotOneFigure(t, g, ins, cmb, scenarioLabel, prn, ...
            onsetTime, alarmTime, xlimAlarm, outAlarm, 'Zoom 5 s Around Mitigation Start');
        figPaths{end + 1} = outAlarm; %#ok<AGROW>
    end
end

fprintf('\nSaved paper-style Doppler figures:\n');
for i = 1 : numel(figPaths)
    fprintf('  %s\n', figPaths{i});
end
fprintf('\n');
end

function localPlotOneFigure(t, gpsHz, insHz, cmbHz, scenarioLabel, prn, onsetTime, alarmTime, xlimRange, outPng, subtitleText)
f = figure('Visible', 'off');
tl = tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
plot(t, gpsHz, 'b-', 'LineWidth', 1.1);
hold on;
plot(t, insHz, 'r-', 'LineWidth', 1.1);
grid on;
ylabel('Doppler (Hz)');
title(sprintf('%s PRN %d: GNSS vs INS', scenarioLabel, prn), 'Interpreter', 'none');
legend({'GNSS measured', 'INS generated'}, 'Location', 'best', 'Interpreter', 'none');
localPlotEventLines(onsetTime, alarmTime, xlimRange);
if ~isempty(xlimRange)
    xlim(xlimRange);
end

nexttile;
plot(t, insHz, 'r-', 'LineWidth', 1.1);
hold on;
plot(t, cmbHz, 'k-', 'LineWidth', 1.1);
grid on;
ylabel('Doppler (Hz)');
xlabel('Time (s)');
title(sprintf('%s PRN %d: INS vs Combined', scenarioLabel, prn), 'Interpreter', 'none');
legend({'INS generated', 'Combined navigation'}, 'Location', 'best', 'Interpreter', 'none');
localPlotEventLines(onsetTime, alarmTime, xlimRange);
if ~isempty(xlimRange)
    xlim(xlimRange);
end

title(tl, sprintf('%s Doppler Comparison PRN %d (%s)', scenarioLabel, prn, subtitleText), 'Interpreter', 'none');
exportgraphics(f, outPng, 'Resolution', 200);
close(f);
end

function localPlotEventLines(onsetTime, alarmTime, xlimRange)
if isfinite(onsetTime)
    if isempty(xlimRange) || (onsetTime >= xlimRange(1) && onsetTime <= xlimRange(2))
        xline(onsetTime, 'c--', 'Onset', 'LineWidth', 1.0, 'LabelVerticalAlignment', 'middle');
    end
end
if isfinite(alarmTime)
    if isempty(xlimRange) || (alarmTime >= xlimRange(1) && alarmTime <= xlimRange(2))
        xline(alarmTime, 'm--', 'Alarm', 'LineWidth', 1.0, 'LabelVerticalAlignment', 'middle');
    end
end
end

function idx = localFirstAlarmIndex(navResults)
idx = [];
if isfield(navResults, 'spoofAlarm')
    idx = find(navResults.spoofAlarm, 1, 'first');
end
end

function idx = localFirstOnsetIndex(navResults, settings, alarmIdx)
idx = [];
needFields = {'detectorArmed', 'detectorValidSatNum', 'detectorHitSatNum', ...
              'detectorMetricCommonHz', 'detectorMetricDiffHz'};
hasAll = true;
for k = 1 : numel(needFields)
    if ~isfield(navResults, needFields{k})
        hasAll = false;
        break;
    end
end
if hasAll
    armed = navResults.detectorArmed(:)' > 0;
    validSat = navResults.detectorValidSatNum(:)' >= settings.spoofDetMinSat;
    hitSat = navResults.detectorHitSatNum(:)' >= settings.spoofDetMinHitSat;
    hitCommon = navResults.detectorMetricCommonHz(:)' > settings.spoofDetCommonThresholdHz;
    hitDiff = navResults.detectorMetricDiffHz(:)' > settings.spoofDetDiffThresholdHz;
    onsetMask = armed & validSat & hitSat & (hitCommon | hitDiff);
    idx = find(onsetMask, 1, 'first');
end

if isempty(idx) && ~isempty(alarmIdx)
    idx = max(1, alarmIdx - 1);
end
end

function tEvent = localIndexToTime(t, idx)
tEvent = nan;
if isempty(idx)
    return;
end
if idx < 1 || idx > numel(t)
    return;
end
tEvent = t(idx);
end

function xlimOut = localClampWindow(xlimIn, t)
xlimOut = xlimIn;
tMin = t(1);
tMax = t(end);
if xlimOut(1) < tMin
    shift = tMin - xlimOut(1);
    xlimOut = xlimOut + shift;
end
if xlimOut(2) > tMax
    shift = xlimOut(2) - tMax;
    xlimOut = xlimOut - shift;
end
xlimOut(1) = max(tMin, xlimOut(1));
xlimOut(2) = min(tMax, xlimOut(2));
end

function resultFile = localResolveResultFile(resultFile, projectRoot)
if exist(resultFile, 'file') == 2
    return;
end
alt = fullfile(projectRoot, resultFile);
if exist(alt, 'file') ~= 2
    error('Result file not found: %s', resultFile);
end
resultFile = alt;
end

function prn = localPrnList(navResults)
if isfield(navResults, 'prnList') && ~isempty(navResults.prnList)
    prn = navResults.prnList(:)';
else
    prn = 1 : size(navResults.dopplerGpsMeasHz, 1);
end
end

function label = localScenarioLabel(S, fallback)
label = fallback;
if isfield(S, 'summary') && isstruct(S.summary) && isfield(S.summary, 'datasetTag')
    tag = char(S.summary.datasetTag);
    if isfield(S.summary, 'profile')
        label = sprintf('%s / %s', upper(tag), upper(char(S.summary.profile)));
    else
        label = upper(tag);
    end
end
end
