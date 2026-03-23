function figPaths = plot_doppler_clean_vs_spoof_prn4(cleanResultFile, spoofResultFile)
% Plot clean-vs-spoof Doppler comparison for up to 4 common PRNs.
%
% Top subplot   : clean GPS Doppler vs spoof GPS Doppler
% Bottom subplot: clean aligned INS/suppressed Doppler vs spoof aligned
%                 INS/suppressed Doppler

if nargin < 1 || isempty(cleanResultFile)
    cleanResultFile = 'rt_tight_goal1_clean_400s_unified_ins2.mat';
end
if nargin < 2 || isempty(spoofResultFile)
    spoofResultFile = 'rt_tight_goal1_ds5_400s_unified_ins2.mat';
end

thisDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(thisDir);

if ~exist(cleanResultFile, 'file')
    alt = fullfile(projectRoot, cleanResultFile);
    if exist(alt, 'file')
        cleanResultFile = alt;
    else
        error('Clean result file not found: %s', cleanResultFile);
    end
end
if ~exist(spoofResultFile, 'file')
    alt = fullfile(projectRoot, spoofResultFile);
    if exist(alt, 'file')
        spoofResultFile = alt;
    else
        error('Spoof result file not found: %s', spoofResultFile);
    end
end

C = load(cleanResultFile);
S = load(spoofResultFile);
if ~isfield(C, 'navResults') || ~isfield(S, 'navResults') || ...
   ~isfield(C, 'settings') || ~isfield(S, 'settings')
    error('Result file missing navResults/settings.');
end

clean = C.navResults;
spoof = S.navResults;
dtC = C.settings.navSolPeriod / 1000;
dtS = S.settings.navSolPeriod / 1000;
tC = (0 : size(clean.dopplerGpsMeasHz, 2) - 1) * dtC;
tS = (0 : size(spoof.dopplerGpsMeasHz, 2) - 1) * dtS;

dopClean = compute_doppler_plot_series(clean, C.settings);
dopSpoof = compute_doppler_plot_series(spoof, S.settings);

prnC = inferPrnList(clean, C, cleanResultFile, projectRoot);
prnS = inferPrnList(spoof, S, spoofResultFile, projectRoot);
if isempty(prnC)
    prnC = 1 : size(clean.dopplerGpsMeasHz, 1);
end
if isempty(prnS)
    prnS = 1 : size(spoof.dopplerGpsMeasHz, 1);
end

[commonPrn, ia, ib] = intersect(prnC(:)', prnS(:)', 'stable');
if isempty(commonPrn)
    error('No common PRNs between clean and spoof result files.');
end

finiteC = sum(isfinite(clean.dopplerGpsMeasHz(ia, :)), 2);
finiteS = sum(isfinite(spoof.dopplerGpsMeasHz(ib, :)), 2);
[~, ord] = sort(min(finiteC, finiteS), 'descend');
sel = ord(1 : min(4, numel(commonPrn)));
commonPrn = commonPrn(sel);
ia = ia(sel);
ib = ib(sel);

outDir = fullfile(projectRoot, 'deepIntegration', 'figures');
if ~exist(outDir, 'dir')
    mkdir(outDir);
end

[~, cleanBase, ~] = fileparts(cleanResultFile);
[~, spoofBase, ~] = fileparts(spoofResultFile);
baseName = sprintf('%s_vs_%s', cleanBase, spoofBase);
alarmIdxSpoof = find(spoof.spoofAlarm, 1, 'first');

figPaths = {};
for k = 1 : numel(commonPrn)
    prn = commonPrn(k);
    idxC = ia(k);
    idxS = ib(k);

    gpsC = dopClean.gpsRawHz(idxC, :);
    gpsS = dopSpoof.gpsRawHz(idxS, :);
    supC = dopClean.supAlignedHz(idxC, :);
    supS = dopSpoof.supAlignedHz(idxS, :);

    f = figure('Visible', 'off');
    tl = tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    nexttile;
    plot(tC, gpsC, 'b-', 'LineWidth', 1.1);
    hold on;
    grid on;
    plot(tS, gpsS, 'r-', 'LineWidth', 1.1);
    if ~isempty(alarmIdxSpoof)
        xline(tS(alarmIdxSpoof), 'm--', 'Spoof Alarm');
    end
    ylabel('Doppler (Hz)');
    title(sprintf('PRN %d clean GPS vs spoof GPS', prn));
    legend({'Clean GPS', 'Spoof GPS'}, 'Location', 'best');

    nexttile;
    plot(tC, supC, 'b-', 'LineWidth', 1.1);
    hold on;
    grid on;
    plot(tS, supS, 'r-', 'LineWidth', 1.1);
    if ~isempty(alarmIdxSpoof)
        xline(tS(alarmIdxSpoof), 'm--', 'Spoof Alarm');
    end
    ylabel('Doppler (Hz)');
    xlabel('Time (s)');
    title(sprintf('PRN %d clean aligned vs spoof suppressed', prn));
    legend({'Clean aligned', 'Spoof suppressed'}, 'Location', 'best');

    title(tl, sprintf('Clean vs Spoof Doppler - PRN %d', prn));
    outPng = fullfile(outDir, sprintf('%s_clean_vs_spoof_doppler_prn%02d.png', baseName, prn));
    exportgraphics(f, outPng, 'Resolution', 180);
    close(f);
    figPaths{end + 1} = outPng; %#ok<AGROW>
end

fprintf('\nSaved clean-vs-spoof Doppler figures:\n');
for i = 1 : numel(figPaths)
    fprintf('  %s\n', figPaths{i});
end
fprintf('\n');
end

function prnList = inferPrnList(navResults, loadedStruct, resultFile, projectRoot)
prnList = [];
if isfield(navResults, 'prnList') && ~isempty(navResults.prnList)
    prnList = navResults.prnList(:)';
    return;
end

datasetFile = '';
if isfield(loadedStruct, 'summary') && isfield(loadedStruct.summary, 'dataset')
    datasetFile = loadedStruct.summary.dataset;
end
if isempty(datasetFile) || ~exist(datasetFile, 'file')
    [~, bn, ~] = fileparts(resultFile);
    if contains(lower(bn), 'clean')
        datasetFile = fullfile(projectRoot, 'cleandynamic_400s.mat');
    elseif contains(lower(bn), 'ds6')
        datasetFile = fullfile(projectRoot, 'ds6_400s.mat');
    else
        datasetFile = fullfile(projectRoot, 'ds5_400s.mat');
    end
end
if ~exist(datasetFile, 'file')
    return;
end

try
    D = load(datasetFile, 'trackResults', 'subFrameStart', 'eph');
    trackResults = D.trackResults;
    subFrameStart = D.subFrameStart;
    eph = D.eph;
    active = find([trackResults.status] ~= '-');
    active = active(subFrameStart(active) > 1);
    p = [];
    for ii = 1 : numel(active)
        ch = active(ii);
        prn = trackResults(ch).PRN;
        e = eph(prn);
        ok = ~isempty(e.IODC) && ~isempty(e.IODE_sf2) && ~isempty(e.IODE_sf3) && ...
             isscalar(e.a_f0) && isscalar(e.a_f1) && isscalar(e.a_f2) && isscalar(e.t_oc);
        if ok
            p(end + 1) = prn; %#ok<AGROW>
        end
    end
    prnList = p;
catch
    prnList = [];
end
end
