function baseline = build_goal1_clean_baseline(profile, opts)
% Build Chapter-1 clean Doppler baseline from the clean dynamic dataset.
%
% The baseline is derived from the same tight-coupled Chapter-1 pipeline
% used in spoofed runs. It produces:
%   1) per-PRN residual bias
%   2) per-PRN residual sigma
%   3) common-mode metric threshold
%   4) differential-mode metric threshold
%   5) per-satellite support threshold

if nargin < 1 || isempty(profile)
    profile = 'ins1';
end
if nargin < 2 || isempty(opts)
    opts = struct();
end

profile = lower(string(profile));
if profile ~= "ins1" && profile ~= "ins2" && profile ~= "ins3"
    error('profile must be ''ins1'', ''ins2'', or ''ins3''.');
end

if ~isfield(opts, 'useOfflineReplay'), opts.useOfflineReplay = 1; end
if ~isfield(opts, 'rawFileName'), opts.rawFileName = ""; end
if ~isfield(opts, 'calibStartSec'), opts.calibStartSec = 20; end
if ~isfield(opts, 'calibEndSec'), opts.calibEndSec = 90; end
if ~isfield(opts, 'commonSigmaScale'), opts.commonSigmaScale = 4; end
if ~isfield(opts, 'diffSigmaScale'), opts.diffSigmaScale = 3; end
if ~isfield(opts, 'satSigmaScale'), opts.satSigmaScale = 1.5; end
if ~isfield(opts, 'pllNoiseBandwidth'), opts.pllNoiseBandwidth = 15; end
if ~isfield(opts, 'minElevDeg'), opts.minElevDeg = 20; end
if ~isfield(opts, 'minSat'), opts.minSat = 4; end
if ~isfield(opts, 'minHitSat'), opts.minHitSat = opts.minSat; end
if ~isfield(opts, 'confirmEpochs'), opts.confirmEpochs = 2; end
if ~isfield(opts, 'metricSmoothWin'), opts.metricSmoothWin = 3; end
if ~isfield(opts, 'templateSmoothWin'), opts.templateSmoothWin = 5; end
if ~isfield(opts, 'armTimeSec'), opts.armTimeSec = 94.5; end
if ~isfield(opts, 'msToProcess'), opts.msToProcess = 399 * 1000; end
if ~isfield(opts, 'leverArm_b'), opts.leverArm_b = [0;0;0]; end

if opts.calibEndSec <= opts.calibStartSec
    error('calibEndSec must be greater than calibStartSec.');
end

thisDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(thisDir);
addpath(fullfile(projectRoot, 'include'))
addpath(fullfile(projectRoot, 'geoFunctions'))
addpath(fullfile(projectRoot, 'acquire_zcj'))
addpath(fullfile(projectRoot, 'track_zcj'))
addpath(fullfile(projectRoot, 'PVT'))
addpath(thisDir)

dataFile = fullfile(projectRoot, 'cleandynamic_400s.mat');
if ~exist(dataFile, 'file')
    error('Missing clean baseline dataset: %s', dataFile);
end

S = load(dataFile, 'trackResults', 'channel', 'TOW', 'eph', 'subFrameStart', 'settings');
trackResults = S.trackResults; %#ok<NASGU>
channel = S.channel; %#ok<NASGU>
TOW = S.TOW; %#ok<NASGU>
eph = S.eph; %#ok<NASGU>
subFrameStart = S.subFrameStart; %#ok<NASGU>

trjFile = chooseTrjFile(profile);
rawFileName = string(opts.rawFileName);
if strlength(rawFileName) == 0 && isfield(S, 'settings') && isfield(S.settings, 'fileName')
    rawFileName = string(S.settings.fileName);
end

settingsOverride = struct(); %#ok<NASGU>
if opts.useOfflineReplay
    settingsOverride.fileName = "";
else
    settingsOverride.fileName = rawFileName;
end
settingsOverride.offlineReplayFromTrackResults = opts.useOfflineReplay;
settingsOverride.msToProcess = max(ceil(opts.calibEndSec * 1000), opts.msToProcess);
settingsOverride.plotTracking = 0;
settingsOverride.skipQuickPlot = 1;
settingsOverride.verboseEpochPrint = 0;
settingsOverride.trjFile = trjFile;
settingsOverride.imuerrProfile = char(profile);
settingsOverride.pllNoiseBandwidth = opts.pllNoiseBandwidth;
settingsOverride.spoofDetArmTimeSec = 1e9;
settingsOverride.spoofDetPostArmGraceSec = 0;
settingsOverride.spoofDetConfirmEpochs = opts.confirmEpochs;
settingsOverride.spoofDetMinSat = opts.minSat;
settingsOverride.spoofDetMinHitSat = opts.minHitSat;
settingsOverride.spoofDetMinElevDeg = opts.minElevDeg;
settingsOverride.spoofDetMetricSmoothWin = opts.metricSmoothWin;
settingsOverride.spoofDetCommonThresholdHz = 1e9;
settingsOverride.spoofDetDiffThresholdHz = 1e9;
settingsOverride.spoofDetSatThresholdHz = 1e9;
settingsOverride.virtualNHCEnable = 0;
settingsOverride.virtualZUPTEnable = 0;
settingsOverride.leverArm_b = opts.leverArm_b(:);
settingsOverride.spoofMitigationMode = 'ins_only';

realtime_tightCouple;

tSec = (0 : size(navResults.dopplerResidualRawHz, 2) - 1) * settings.navSolPeriod / 1000;
calibMask = (tSec >= opts.calibStartSec) & (tSec < opts.calibEndSec);
if nnz(calibMask) < 10
    error('Too few clean epochs in calibration window.');
end

prnList = navResults.prnList(:);
rawResidual = navResults.dopplerResidualRawHz(:, calibMask);
validResidual = isfinite(rawResidual);
satNum = size(rawResidual, 1);

satBiasHz = nan(satNum, 1);
satSigmaHz = nan(satNum, 1);
for ii = 1 : satNum
    satVals = rawResidual(ii, validResidual(ii, :));
    if numel(satVals) < 10
        continue;
    end
    satBiasHz(ii) = median(satVals);
    satSigmaHz(ii) = localRobustSigma(satVals - satBiasHz(ii), 0.05);
end

if sum(isfinite(satBiasHz)) < opts.minSat
    error('Too few satellites with usable clean residual samples.');
end

residualBcFull = navResults.dopplerResidualRawHz - satBiasHz;
templateResidualHz = movmedian(residualBcFull, opts.templateSmoothWin, 2, 'omitnan');
templateResidualHz(~isfinite(templateResidualHz)) = residualBcFull(~isfinite(templateResidualHz));
templateDevFull = residualBcFull - templateResidualHz;
templateDevCalib = templateDevFull(:, calibMask);

metricCommonSeries = nan(1, size(templateDevCalib, 2));
metricDiffSeries = nan(1, size(templateDevCalib, 2));
for kk = 1 : size(templateDevCalib, 2)
    vals = templateDevCalib(:, kk);
    vals = vals(isfinite(vals));
    if numel(vals) < opts.minSat
        continue;
    end
    center = median(vals);
    metricCommonSeries(kk) = abs(center);
    metricDiffSeries(kk) = median(abs(vals - center));
end

metricCommonVals = metricCommonSeries(isfinite(metricCommonSeries));
metricDiffVals = metricDiffSeries(isfinite(metricDiffSeries));
if numel(metricCommonVals) < 10 || numel(metricDiffVals) < 10
    error('Too few clean common/differential metric samples.');
end

absResidualVals = abs(templateDevCalib(isfinite(templateDevCalib)));
if numel(absResidualVals) < 50
    error('Too few clean bias-corrected residual samples.');
end

[commonMedianHz, commonSigmaHz, T_cm] = localRobustThreshold(metricCommonVals, opts.commonSigmaScale, 0.05);
[diffMedianHz, diffSigmaHz, T_df] = localRobustThreshold(metricDiffVals, opts.diffSigmaScale, 0.05);
[satMedianHz, satSigmaPooledHz, T_sat] = localRobustThreshold(absResidualVals, opts.satSigmaScale, 0.05);

baseline = struct();
baseline.profile = char(profile);
baseline.dataset = dataFile;
baseline.trjFile = trjFile;
baseline.useOfflineReplay = logical(opts.useOfflineReplay);
baseline.rawFileName = char(rawFileName);
baseline.calibStartSec = opts.calibStartSec;
baseline.calibEndSec = opts.calibEndSec;
baseline.prnList = prnList;
baseline.satBiasHz = satBiasHz;
baseline.satSigmaHz = satSigmaHz;
baseline.commonMedianHz = commonMedianHz;
baseline.commonSigmaHz = commonSigmaHz;
baseline.diffMedianHz = diffMedianHz;
baseline.diffSigmaHz = diffSigmaHz;
baseline.satMedianHz = satMedianHz;
baseline.satSigmaPooledHz = satSigmaPooledHz;
baseline.T_cm = T_cm;
baseline.T_df = T_df;
baseline.T_sat = T_sat;
baseline.N_hit = opts.minHitSat;
baseline.N_confirm = opts.confirmEpochs;
baseline.minSat = opts.minSat;
baseline.minElevDeg = opts.minElevDeg;
baseline.metricSmoothWin = opts.metricSmoothWin;
baseline.templateSmoothWin = opts.templateSmoothWin;
baseline.pllNoiseBandwidth = opts.pllNoiseBandwidth;
baseline.leverArm_b = opts.leverArm_b(:);
baseline.armTimeSec = opts.armTimeSec;
baseline.recommendedArmTimeSec = max(opts.armTimeSec, opts.calibEndSec);
baseline.templateTsec = tSec(:)';
baseline.templateResidualHz = templateResidualHz;
baseline.sampleCountEpoch = nnz(calibMask);
baseline.sampleCountCommon = numel(metricCommonVals);
baseline.sampleCountDiff = numel(metricDiffVals);
baseline.sampleCountSat = numel(absResidualVals);

fprintf('[BASELINE] %s clean common metric: median=%.3f Hz, sigma=%.3f Hz, Tcm=%.3f Hz\n', ...
    upper(char(profile)), baseline.commonMedianHz, baseline.commonSigmaHz, baseline.T_cm);
fprintf('[BASELINE] %s clean diff metric  : median=%.3f Hz, sigma=%.3f Hz, Tdf=%.3f Hz\n', ...
    upper(char(profile)), baseline.diffMedianHz, baseline.diffSigmaHz, baseline.T_df);
fprintf('[BASELINE] %s clean |residual|   : median=%.3f Hz, sigma=%.3f Hz, Tsat=%.3f Hz\n', ...
    upper(char(profile)), baseline.satMedianHz, baseline.satSigmaPooledHz, baseline.T_sat);
fprintf('[BASELINE] template smooth win = %d epochs, arm time = %.1f s\n', ...
    baseline.templateSmoothWin, baseline.recommendedArmTimeSec);
fprintf('[BASELINE] calibration window: [%.1f, %.1f] s, PRNs=%s\n', ...
    baseline.calibStartSec, baseline.calibEndSec, mat2str(prnList(:)'));
end

function trjFile = chooseTrjFile(profile)
switch lower(char(profile))
    case 'ins1'
        trjFile = "E:\\fgi_result\\ins_simulation\\ins1_trj.mat";
    case 'ins2'
        trjFile = "E:\\fgi_result\\ins_simulation\\ins2_trj.mat";
    case 'ins3'
        trjFile = "E:\\fgi_result\\ins_simulation\\ins3_trj.mat";
end
end

function sigma = localRobustSigma(vals, sigmaFloor)
vals = vals(:);
vals = vals(isfinite(vals));
if isempty(vals)
    sigma = sigmaFloor;
    return;
end
center = median(vals);
sigma = max(sigmaFloor, 1.4826 * median(abs(vals - center)));
end

function [mu, sigma, T] = localRobustThreshold(vals, sigmaScale, sigmaFloor)
vals = vals(:);
vals = vals(isfinite(vals));
if isempty(vals)
    error('Cannot build robust threshold from empty input.');
end
mu = median(vals);
sigma = max(sigmaFloor, 1.4826 * median(abs(vals - mu)));
T = mu + sigmaScale * sigma;
end
