function outFile = reprocess_ds5_200s_baseline()
% Reprocess first 200s of ds5.bin with baseline settings close to ds5_400s.mat.

thisDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(thisDir);
addpath(fullfile(projectRoot, 'include'))
addpath(fullfile(projectRoot, 'geoFunctions'))
addpath(fullfile(projectRoot, 'acquire_zcj'))
addpath(fullfile(projectRoot, 'track_zcj'))
addpath(fullfile(projectRoot, 'PVT'))
addpath(thisDir)

settings = initSettings();
settings.msToProcess = 200 * 1000;
settings.numberOfChannels = 6;
settings.acqThreshold = 2.5;
settings.plotTracking = 0;
settings.fileName = "E:\ds5.bin";

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
    data  = dataI + 1j * dataQ;
end

acqResults = acquisition_L1CA1(data, settings);
channel = preRun(acqResults, settings);

isFLL = 0;
[trackResults, channel] = trackfll1stpll2nd(fid, channel, settings, isFLL);
[navSolutions, eph, subFrameStart, TOW] = postNavigation_zcj(trackResults, settings);

outFile = fullfile(projectRoot, 'ds5_rollback_200s.mat');
save(outFile, 'trackResults', 'channel', 'TOW', 'eph', 'subFrameStart', 'settings', 'navSolutions', '-v7.3');
fprintf('Saved: %s\n', outFile);
end
