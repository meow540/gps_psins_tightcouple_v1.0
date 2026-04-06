function results = run_goal2_deep_400s_batch(opts)
% Run Chapter-2 deep-coupling pipeline on clean/ds5/ds6 with one parameter set.
%
% Usage:
%   results = run_goal2_deep_400s_batch();
%   results = run_goal2_deep_400s_batch(struct('roundTime', 120));

if nargin < 1 || isempty(opts)
    opts = struct();
end

opts = fillDefault(opts, 'roundTime', 800);  % 400s at 0.5s nav period
opts = fillDefault(opts, 'trjFile', 'E:\fgi_result\ins_simulation\ins1_trj.mat');
opts = fillDefault(opts, 'datasets', {'clean', 'ds5', 'ds6'});
opts = fillDefault(opts, 'skipMissingRaw', true);
opts = fillDefault(opts, 'outPrefix', 'rt_deep_goal2');
opts = fillDefault(opts, 'kCm', 4.0);
opts = fillDefault(opts, 'kDf', 4.0);
opts = fillDefault(opts, 'kSat', 4.0);
opts = fillDefault(opts, 'kZ', 4.0);
opts = fillDefault(opts, 'sigmaMeasHz', 1.5);
opts = fillDefault(opts, 'deepConfirmEpochs', 5);
opts = fillDefault(opts, 'calibStartEpoch', 10);
opts = fillDefault(opts, 'calibEndEpoch', min(opts.roundTime, 180));
opts = fillDefault(opts, 'aidWeightNormal', 0.20);
opts = fillDefault(opts, 'aidWeightSuspect', 0.70);
opts = fillDefault(opts, 'aidWeightSpoof', 1.10);
opts = fillDefault(opts, 'clkWeightNormal', 1.00);
opts = fillDefault(opts, 'clkWeightSuspect', 1.00);
opts = fillDefault(opts, 'clkWeightSpoof', 1.00);
opts = fillDefault(opts, 'rhoBlendNormal', 0.00);
opts = fillDefault(opts, 'rhoBlendSuspect', 0.05);
opts = fillDefault(opts, 'rhoBlendSpoof', 0.30);
opts = fillDefault(opts, 'deepUseGnssKfUpdate', 1);
opts = fillDefault(opts, 'deepUseKfClockUpdate', 0);
opts = fillDefault(opts, 'deepUseGnssKfUpdateNormal', opts.deepUseGnssKfUpdate);
opts = fillDefault(opts, 'deepUseGnssKfUpdateSuspect', opts.deepUseGnssKfUpdate);
opts = fillDefault(opts, 'deepUseGnssKfUpdateSpoof', 0);
opts = fillDefault(opts, 'deepUseKfClockUpdateNormal', opts.deepUseKfClockUpdate);
opts = fillDefault(opts, 'deepUseKfClockUpdateSuspect', opts.deepUseKfClockUpdate);
opts = fillDefault(opts, 'deepUseKfClockUpdateSpoof', opts.deepUseKfClockUpdate);
opts = fillDefault(opts, 'deepRScaleNormal', 1.0);
opts = fillDefault(opts, 'deepRScaleSuspect', 4.0);
opts = fillDefault(opts, 'deepRScaleSpoof', 10.0);
opts = fillDefault(opts, 'deepUseIndependentDetectTrack', 0);
opts = fillDefault(opts, 'deepDetectUseTrackResults', 1);
opts = fillDefault(opts, 'deepFastMode', 0);
opts = fillDefault(opts, 'deepUseReferenceResidual', 1);
% Guard floors for detector thresholds:
% calibration window may have near-zero MAD, causing thresholds ~= median.
% Keep absolute floors to avoid clean-scene over-trigger.
opts = fillDefault(opts, 'minTcm', 350.0);
opts = fillDefault(opts, 'minTdf', 150.0);
opts = fillDefault(opts, 'minTsat', 100.0);
opts = fillDefault(opts, 'minTz', 220.0);
if opts.deepFastMode && (opts.deepUseGnssKfUpdateNormal || opts.deepUseGnssKfUpdateSuspect || ...
        opts.deepUseGnssKfUpdateSpoof || opts.deepUseKfClockUpdateNormal || ...
        opts.deepUseKfClockUpdateSuspect || opts.deepUseKfClockUpdateSpoof)
    warning('[GOAL2] deepFastMode=1 is inconsistent with KF measurement update. Force deepFastMode=0.');
    opts.deepFastMode = 0;
end
if ~isfield(opts, 'deepDetrendWindowEpoch')
    if opts.deepUseReferenceResidual
        opts.deepDetrendWindowEpoch = 1;
    else
        opts.deepDetrendWindowEpoch = 80;
    end
end
if ~isfield(opts, 'hitMin')
    if opts.deepUseReferenceResidual
        opts.hitMin = 0;
    else
        opts.hitMin = 3;
    end
end

matMap = struct( ...
    'clean', 'cleandynamic_400s.mat', ...
    'ds5', 'ds5_400s.mat', ...
    'ds6', 'ds6_400s.mat');
rawMap = struct( ...
    'clean', 'E:\cleanDynamic.bin', ...
    'ds5', 'E:\ds5.bin', ...
    'ds6', 'F:\dataset\TEXBAT\ds6.bin');

% Base settings used in all runs.
baseOverride = struct();
baseOverride.trjFile = opts.trjFile;
baseOverride.deepRoundTime = opts.roundTime;
baseOverride.deepSpoofDetectEnable = 1;
baseOverride.deepSpoofLatch = 1;
baseOverride.deepHitMin = opts.hitMin;
baseOverride.deepConfirmEpochs = opts.deepConfirmEpochs;
baseOverride.deepSigmaMeasHz = opts.sigmaMeasHz;
baseOverride.deepKappaZ = 3.5;
baseOverride.deepTz = 3.5;
baseOverride.deepAidWeightNormal = opts.aidWeightNormal;
baseOverride.deepAidWeightSuspect = opts.aidWeightSuspect;
baseOverride.deepAidWeightSpoof = opts.aidWeightSpoof;
baseOverride.deepClkWeightNormal = opts.clkWeightNormal;
baseOverride.deepClkWeightSuspect = opts.clkWeightSuspect;
baseOverride.deepClkWeightSpoof = opts.clkWeightSpoof;
baseOverride.deepRhoBlendNormal = opts.rhoBlendNormal;
baseOverride.deepRhoBlendSuspect = opts.rhoBlendSuspect;
baseOverride.deepRhoBlendSpoof = opts.rhoBlendSpoof;
baseOverride.deepUseGnssKfUpdate = opts.deepUseGnssKfUpdate;
baseOverride.deepUseKfClockUpdate = opts.deepUseKfClockUpdate;
baseOverride.deepUseGnssKfUpdateNormal = opts.deepUseGnssKfUpdateNormal;
baseOverride.deepUseGnssKfUpdateSuspect = opts.deepUseGnssKfUpdateSuspect;
baseOverride.deepUseGnssKfUpdateSpoof = opts.deepUseGnssKfUpdateSpoof;
baseOverride.deepUseKfClockUpdateNormal = opts.deepUseKfClockUpdateNormal;
baseOverride.deepUseKfClockUpdateSuspect = opts.deepUseKfClockUpdateSuspect;
baseOverride.deepUseKfClockUpdateSpoof = opts.deepUseKfClockUpdateSpoof;
baseOverride.deepRScaleNormal = opts.deepRScaleNormal;
baseOverride.deepRScaleSuspect = opts.deepRScaleSuspect;
baseOverride.deepRScaleSpoof = opts.deepRScaleSpoof;
baseOverride.deepUseIndependentDetectTrack = opts.deepUseIndependentDetectTrack;
baseOverride.deepDetectUseTrackResults = opts.deepDetectUseTrackResults;
baseOverride.deepFastMode = opts.deepFastMode;
baseOverride.deepDetrendWindowEpoch = opts.deepDetrendWindowEpoch;
baseOverride.deepUseReferenceResidual = 0;
baseOverride.deepRefResidualHz = [];
baseOverride.deepPllNoiseBandwidthNormal = 12;
baseOverride.deepPllNoiseBandwidthSuspect = 6;
baseOverride.deepPllNoiseBandwidthSpoof = 3;
baseOverride.deepCarrNcoStepLimitHz = 120;
baseOverride.deepClkDriftHzLimit = 120;
baseOverride.deepCarrErrorScaleSuspect = 0.5;
baseOverride.deepCarrErrorScaleSpoof = 0.15;
baseOverride.deepAlignCarrNcoOnSpoof = 1;
baseOverride.deepResetCarrErrorOnSpoof = 1;
baseOverride.deepFreezeCarrErrorInSpoof = 0;
baseOverride.deepBiasByPrnHz = zeros(1, 64);

% 1) Clean calibration pass (detector disabled).
calibOverride = baseOverride;
calibOverride.deepSpoofDetectEnable = 0;
fprintf('\n[GOAL2] Calibration run: CLEAN\n');
cleanCal = runOneDataset('clean', matMap, rawMap, calibOverride, opts.outPrefix, true);

[biasByPrn, Tcm, Tdf, Tsat, Tz] = buildCalibration(cleanCal.navResults, opts);
refResidualHz = [];
if opts.deepUseReferenceResidual
    refResidualHz = cleanCal.navResults.residualHz;
    calibRefOverride = baseOverride;
    calibRefOverride.deepSpoofDetectEnable = 0;
    calibRefOverride.deepBiasByPrnHz = biasByPrn;
    calibRefOverride.deepUseReferenceResidual = 1;
    calibRefOverride.deepRefResidualHz = refResidualHz;
    fprintf('\n[GOAL2] Calibration run: CLEAN (reference-subtracted)\n');
    cleanCalRef = runOneDataset('clean', matMap, rawMap, calibRefOverride, ...
        sprintf('%s_ref', opts.outPrefix), true);
    [~, Tcm, Tdf, Tsat, Tz] = buildCalibration(cleanCalRef.navResults, opts);
end
fprintf('[GOAL2] Calibrated thresholds: Tcm=%.3f Hz, Tdf=%.3f Hz, Tsat=%.3f Hz, Tz=%.3f\n', ...
    Tcm, Tdf, Tsat, Tz);
fprintf('[GOAL2] Calibration window: epoch %d -> %d\n', opts.calibStartEpoch, opts.calibEndEpoch);

% 2) Final runs with calibrated thresholds.
results = struct([]);
for ii = 1:numel(opts.datasets)
    tag = lower(string(opts.datasets{ii}));
    ov = baseOverride;
    ov.deepBiasByPrnHz = biasByPrn;
    ov.deepTcmHz = Tcm;
    ov.deepTdfHz = Tdf;
    ov.deepTsatHz = Tsat;
    ov.deepTz = Tz;
    ov.deepSpoofDetectEnable = 1;
    ov.deepDetectArmEpoch = max(1, opts.calibEndEpoch + 1);
    ov.deepUseReferenceResidual = opts.deepUseReferenceResidual;
    if opts.deepUseReferenceResidual
        ov.deepRefResidualHz = refResidualHz;
    else
        ov.deepRefResidualHz = [];
    end

    fprintf('\n[GOAL2] Final run: %s\n', upper(char(tag)));
    try
        runRes = runOneDataset(char(tag), matMap, rawMap, ov, opts.outPrefix, false);
        results = [results; rmfield(runRes, {'navResults', 'settingsOverride'})]; %#ok<AGROW>
    catch ME
        if opts.skipMissingRaw
            warning('[GOAL2] Skip %s: %s', upper(char(tag)), ME.message);
        else
            rethrow(ME);
        end
    end
end

% Save batch summary.
save(sprintf('%s_batch_summary.mat', opts.outPrefix), 'results', 'opts', ...
    'biasByPrn', 'Tcm', 'Tdf', 'Tsat', 'Tz', 'refResidualHz', '-v7.3');
fprintf('\n[GOAL2] Batch summary saved: %s_batch_summary.mat\n', opts.outPrefix);
end

function out = runOneDataset(tag, matMap, rawMap, settingsOverride, outPrefix, isCalib)
tag = lower(string(tag));
if ~isfield(matMap, tag)
    error('Unsupported dataset tag: %s', tag);
end

matFile = matMap.(tag);
rawFile = rawMap.(tag);
if exist(matFile, 'file') ~= 2
    error('Missing MAT dataset: %s', matFile);
end

S = load(matFile, 'trackResults', 'channel', 'TOW', 'eph', 'subFrameStart', 'settings');
trackResults = S.trackResults; %#ok<NASGU>
channel = S.channel; %#ok<NASGU>
TOW = S.TOW; %#ok<NASGU>
eph = S.eph; %#ok<NASGU>
subFrameStart = S.subFrameStart; %#ok<NASGU>

rawCandidates = {};
if isfield(rawMap, tag)
    rawCandidates{end+1} = rawMap.(tag); %#ok<AGROW>
end
if isfield(S, 'settings') && isstruct(S.settings) && isfield(S.settings, 'fileName')
    rawCandidates{end+1} = S.settings.fileName; %#ok<AGROW>
end
if strcmp(tag, 'ds6')
    rawCandidates{end+1} = 'E:\ds6.bin'; %#ok<AGROW>
    userProf = getenv('USERPROFILE');
    if ~isempty(userProf)
        rawCandidates{end+1} = fullfile(userProf, 'Downloads', 'ds6.bin'); %#ok<AGROW>
    end
end
rawFile = pickFirstExisting(rawCandidates);
if isempty(rawFile)
    error('Missing raw IF file for %s. Tried: %s', tag, strjoin(rawCandidates, ', '));
end
settingsOverride.fileName = rawFile;
run('deepIntegration/DeepCouple_perINStime.m');  %#ok<RUN>

navSolPeriodSec = 0.5;
if isfield(settingsOverride, 'navSolPeriod')
    navSolPeriodSec = settingsOverride.navSolPeriod / 1000;
end

firstSuspect = find(navResults.spoofState >= 1, 1, 'first');
firstSpoof = find(navResults.spoofState == 2, 1, 'first');

out = struct();
out.dataset = char(tag);
out.epochs = numel(navResults.X);
out.stateMax = max(navResults.spoofState);
out.firstSuspectEpoch = firstSuspect;
out.firstSpoofEpoch = firstSpoof;
out.firstSuspectSec = epochToSec(firstSuspect, navSolPeriodSec);
out.firstSpoofSec = epochToSec(firstSpoof, navSolPeriodSec);
out.metricCmMedian = median(navResults.metricCmHz(isfinite(navResults.metricCmHz)));
out.metricDfMedian = median(navResults.metricDfHz(isfinite(navResults.metricDfHz)));
out.metricZMedian = median(navResults.metricZ(isfinite(navResults.metricZ)));
out.hitCountMax = max(navResults.hitCount);
out.rhoCorrRmsMean = mean(navResults.rhoCorrRmsM(isfinite(navResults.rhoCorrRmsM)));
out.prrCorrRmsMean = mean(navResults.prrCorrRmsMps(isfinite(navResults.prrCorrRmsMps)));
out.navResults = navResults;
out.settingsOverride = settingsOverride;

if isCalib
    outFile = sprintf('%s_%s_calib.mat', outPrefix, char(tag));
else
    outFile = sprintf('%s_%s_%ds.mat', outPrefix, char(tag), round(out.epochs*navSolPeriodSec));
end
save(outFile, 'navResults', 'settingsOverride', 'tag', '-v7.3');

fprintf('[GOAL2] %s -> %s\n', upper(char(tag)), outFile);
fprintf('        epochs=%d, maxState=%d, firstSpoof=%s\n', ...
    out.epochs, out.stateMax, num2str(out.firstSpoofSec, '%.2f'));
end

function [biasByPrn, Tcm, Tdf, Tsat, Tz] = buildCalibration(navResults, opts)
prnList = navResults.prnList(:);
resAll = navResults.residualHz;
ep1 = max(1, min(size(resAll, 2), opts.calibStartEpoch));
ep2 = max(ep1, min(size(resAll, 2), opts.calibEndEpoch));
res = resAll(:, ep1:ep2);

biasByPrn = zeros(1, 64);
for ii = 1:numel(prnList)
    prn = prnList(ii);
    rr = res(ii, :);
    rr = rr(isfinite(rr));
    if ~isempty(rr) && prn <= numel(biasByPrn)
        biasByPrn(prn) = median(rr);
    end
end

resNoBias = res;
for ii = 1:numel(prnList)
    prn = prnList(ii);
    if prn <= numel(biasByPrn)
        resNoBias(ii, :) = resNoBias(ii, :) - biasByPrn(prn);
    end
end

if isfield(navResults, 'residualDetHz')
    resDet = navResults.residualDetHz(:, ep1:ep2);
else
    resDet = resNoBias;
end

cm = navResults.metricCmHz(ep1:ep2); cm = cm(isfinite(cm));
df = navResults.metricDfHz(ep1:ep2); df = df(isfinite(df));
sat = abs(resDet(:)); sat = sat(isfinite(sat));
zz = navResults.metricZ(ep1:ep2); zz = zz(isfinite(zz));

Tcm = robustThr(cm, opts.kCm);
Tdf = robustThr(df, opts.kDf);
Tsat = robustThr(sat, opts.kSat);
Tz = robustThr(zz, opts.kZ);
Tcm = max(Tcm, opts.minTcm);
Tdf = max(Tdf, opts.minTdf);
Tsat = max(Tsat, opts.minTsat);
Tz = max(Tz, opts.minTz);
end

function th = robustThr(x, k)
if isempty(x)
    th = inf;
    return;
end
mu = median(x);
sig = 1.4826 * median(abs(x - mu));
if sig <= 0
    sig = std(x);
end
th = mu + k * sig;
end

function s = epochToSec(ep, dt)
if isempty(ep)
    s = nan;
else
    s = (ep - 1) * dt;
end
end

function S = fillDefault(S, name, val)
if ~isfield(S, name)
    S.(name) = val;
end
end

function p = pickFirstExisting(cands)
p = '';
for ii = 1:numel(cands)
    c = cands{ii};
    if isstring(c), c = char(c); end
    if isempty(c), continue; end
    if exist(c, 'file') == 2
        p = c;
        return;
    end
end
end
