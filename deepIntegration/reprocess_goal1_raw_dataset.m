function summary = reprocess_goal1_raw_dataset(datasetTag, opts)
% Rebuild one Goal-1 input MAT from raw IF data with a single unified parameter set.
%
% Usage:
%   reprocess_goal1_raw_dataset('clean')
%   reprocess_goal1_raw_dataset('ds5')
%   reprocess_goal1_raw_dataset('ds6')

if nargin < 1 || isempty(datasetTag)
    error('datasetTag is required: ''clean'', ''ds5'', or ''ds6''.');
end
if nargin < 2 || isempty(opts)
    opts = struct();
end

datasetTag = lower(string(datasetTag));
if datasetTag ~= "clean" && datasetTag ~= "ds5" && datasetTag ~= "ds6"
    error('datasetTag must be ''clean'', ''ds5'', or ''ds6''.');
end

if ~isfield(opts, 'rawFileName'), opts.rawFileName = ""; end
if ~isfield(opts, 'msToProcess'), opts.msToProcess = 400 * 1000; end
if ~isfield(opts, 'numberOfChannels'), opts.numberOfChannels = 12; end
if ~isfield(opts, 'acqThreshold'), opts.acqThreshold = 2; end
if ~isfield(opts, 'pllNoiseBandwidth'), opts.pllNoiseBandwidth = 12; end
if ~isfield(opts, 'dllNoiseBandwidth'), opts.dllNoiseBandwidth = 1.5; end
if ~isfield(opts, 'trkCoIntime'), opts.trkCoIntime = 2; end
if ~isfield(opts, 'trknonCoIntime'), opts.trknonCoIntime = 2; end
if ~isfield(opts, 'plotTracking'), opts.plotTracking = 0; end

thisDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(thisDir);
addpath(projectRoot)
addpath(fullfile(projectRoot, 'include'))
addpath(fullfile(projectRoot, 'geoFunctions'))
addpath(fullfile(projectRoot, 'acquire_zcj'))
addpath(fullfile(projectRoot, 'track_zcj'))
addpath(fullfile(projectRoot, 'PVT'))
addpath(thisDir)

[defaultRawFile, outFileName] = mapDatasetPaths(datasetTag);
rawFileName = string(opts.rawFileName);
if strlength(rawFileName) == 0
    rawFileName = defaultRawFile;
end
if exist(char(rawFileName), 'file') ~= 2
    error('Raw IF file not found: %s', char(rawFileName));
end

settings = initSettings();
settings.fileName = rawFileName;
settings.msToProcess = opts.msToProcess;
settings.numberOfChannels = opts.numberOfChannels;
settings.acqThreshold = opts.acqThreshold;
settings.pllNoiseBandwidth = opts.pllNoiseBandwidth;
settings.dllNoiseBandwidth = opts.dllNoiseBandwidth;
settings.trkCoIntime = opts.trkCoIntime;
settings.trknonCoIntime = opts.trknonCoIntime;
settings.plotTracking = opts.plotTracking;
settings.goal1UnifiedRawVersion = 'goal1_unified_raw_20260318_v1';
settings.goal1UnifiedRawDataset = char(datasetTag);

fprintf('\n[RAW] Reprocessing %s\n', upper(char(datasetTag)));
fprintf('[RAW] Input : %s\n', char(rawFileName));
fprintf('[RAW] Output: %s\n', fullfile(projectRoot, outFileName));
fprintf('[RAW] Params: %d s, %d ch, acqTh=%.2f, PLL=%.1f Hz, DLL=%.1f Hz\n', ...
    round(settings.msToProcess / 1000), settings.numberOfChannels, ...
    settings.acqThreshold, settings.pllNoiseBandwidth, settings.dllNoiseBandwidth);

fid = fopen(settings.fileName, 'rb');
if fid < 0
    error('Unable to open raw file: %s', settings.fileName);
end
cleanupObj = onCleanup(@() fclose(fid)); %#ok<NASGU>

fseek(fid, settings.skipNumberOfSamples * settings.dataFormat * settings.fileType, 'bof');
samplesPerCode = round(settings.samplingFreq / (settings.codeFreqBasis / settings.codeLength));
data = fread(fid, 20 * samplesPerCode * settings.fileType, settings.dataType)';
if settings.fileType == 2
    dataI = data(1:2:end);
    dataQ = data(2:2:end);
    data = dataI + 1j * dataQ;
end

acqResults = acquisition_L1CA1(data, settings);
channel = preRun(acqResults, settings);

trackedCh = find([channel.status] ~= '-');
if isempty(trackedCh)
    error('No satellites acquired for dataset %s.', char(datasetTag));
end

isFLL = 0;
[trackResults, channel] = trackfll1stpll2nd(fid, channel, settings, isFLL);
[navSolutions, eph, subFrameStart, TOW] = postNavigation_zcj(trackResults, settings);

outFile = fullfile(projectRoot, outFileName);
save(outFile, ...
    'trackResults', 'channel', 'TOW', 'eph', 'subFrameStart', ...
    'settings', 'navSolutions', 'acqResults', '-v7.3');

summary = buildSummary(datasetTag, rawFileName, outFile, trackResults, channel, eph, subFrameStart, navSolutions);

fprintf('[RAW] tracked=%d, frameSync=%d, ephValid=%d, navEpochs=%d\n', ...
    summary.trackedChannels, summary.frameSyncChannels, ...
    summary.validEphSatellites, summary.navEpochs);
fprintf('[RAW] Saved: %s\n\n', outFile);
end


function [rawFileName, outFileName] = mapDatasetPaths(datasetTag)
switch lower(char(datasetTag))
    case 'clean'
        rawFileName = "E:\cleanDynamic.bin";
        outFileName = 'cleandynamic_400s.mat';
    case 'ds5'
        rawFileName = "E:\ds5.bin";
        outFileName = 'ds5_400s.mat';
    case 'ds6'
        rawFileName = "F:\dataset\TEXBAT\ds6.bin";
        outFileName = 'ds6_400s.mat';
    otherwise
        error('Unsupported datasetTag: %s', char(datasetTag));
end
end


function summary = buildSummary(datasetTag, rawFileName, outFile, trackResults, channel, eph, subFrameStart, navSolutions)
summary = struct();
summary.datasetTag = char(datasetTag);
summary.rawFileName = char(rawFileName);
summary.output = outFile;
summary.trackedChannels = sum([channel.status] ~= '-');

if isempty(subFrameStart)
    frameSyncCh = [];
else
    trackedCh = find([trackResults.status] ~= '-');
    frameSyncCh = trackedCh(subFrameStart(trackedCh) > 1);
end
summary.frameSyncChannels = numel(frameSyncCh);

validEphCount = 0;
for ii = 1:numel(frameSyncCh)
    prn = trackResults(frameSyncCh(ii)).PRN;
    if isValidEphRecord(eph, prn)
        validEphCount = validEphCount + 1;
    end
end
summary.validEphSatellites = validEphCount;

if isempty(navSolutions) || ~isfield(navSolutions, 'X')
    summary.navEpochs = 0;
else
    summary.navEpochs = numel(navSolutions.X);
end
end


function tf = isValidEphRecord(eph, prn)
tf = false;
if isempty(eph) || numel(eph) < prn || isempty(eph(prn))
    return;
end

needed = {'IODC', 'IODE_sf2', 'IODE_sf3', 'a_f0', 'a_f1', 'a_f2', 't_oc'};
for ii = 1:numel(needed)
    if ~isfield(eph(prn), needed{ii})
        return;
    end
end

tf = ~isempty(eph(prn).IODC) && ~isempty(eph(prn).IODE_sf2) && ...
     ~isempty(eph(prn).IODE_sf3) && isscalar(eph(prn).a_f0) && ...
     isscalar(eph(prn).a_f1) && isscalar(eph(prn).a_f2) && ...
     isscalar(eph(prn).t_oc);
end
