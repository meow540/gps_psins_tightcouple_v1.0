%% GNSS INS 娣辩粍鍚堬紝鍦ㄧ揣缁勫悎鐨勫熀纭€涓婂璺熻釜鐜矾杩涜杈呭姪
% trackResults, channel, TOW, eph, subFrameStart 闇€瑕佹彁鍓嶅噯澶囧ソ
close all;

% Ensure project functions are ahead of toolbox functions with the same names.
thisDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(thisDir);
addpath(projectRoot)
addpath(fullfile(projectRoot, 'include'))
addpath(fullfile(projectRoot, 'geoFunctions'))
addpath(fullfile(projectRoot, 'acquire_zcj'))
addpath(fullfile(projectRoot, 'track_zcj'))
addpath(fullfile(projectRoot, 'PVT'))
addpath(thisDir)

settings = initSettings();
if exist('settingsOverride', 'var') && isstruct(settingsOverride)
    fn = fieldnames(settingsOverride);
    for fi = 1:numel(fn)
        settings.(fn{fi}) = settingsOverride.(fn{fi});
    end
end


[fid, ~] = fopen(settings.fileName, 'rb');
if fid < 0
    error('Cannot open raw IF file: %s', settings.fileName);
end
cleanupObj = onCleanup(@() fclose(fid)); %#ok<NASGU>
if ~isfield(settings, 'deepUseIndependentDetectTrack')
    settings.deepUseIndependentDetectTrack = 1;
end
fidDet = [];
if settings.deepUseIndependentDetectTrack
    [fidDet, ~] = fopen(settings.fileName, 'rb');
    if fidDet < 0
        error('Cannot open raw IF file for detector tracking: %s', settings.fileName);
    end
    cleanupObjDet = onCleanup(@() fclose(fidDet)); %#ok<NASGU>
end

positioningTime = TOW + settings.navSolPeriod / 1000;     

%% INS妯″潡鍒濆鍖?
glvs
ggpsvars
psinstypedef('test_SINS_GPS_tightly_def');
trjFile = 'trj_sc522.mat';
if isfield(settings, 'trjFile')
    trjFile = settings.trjFile;
end
trj = trjfile(trjFile);
[nn, ts, nts] = nnts(2, diff(trj.imu(1:2,end)));    
avp0 = trj.avp0;  
t0_imu = trj.imu(1, end);
t0_gps = TOW;

davp = avperrset([10;10;60], 0.5, [2; 2; 6]);     

% ins = insinit(avpadderr(trj.avp0, davp), ts);
ins = insinit(trj.avp0, ts);

if ~isfield(settings, 'deepInjectImuError'), settings.deepInjectImuError = 0; end
if settings.deepInjectImuError
    imuerr = imuerrset(5,1000,0.05,30);
    trj.imu = imuadderr(trj.imu, imuerr);
else
    imuerr = imuerrset(0,0,0,0);
end

kf = kfinit(ins, davp, imuerr);

%% INS杩愯鑷抽娆＄粍鍚堟椂鍒?
k = 1;
k1 = 1;                 
t = trj.imu(k1, end);   
while (t0_gps + (t - t0_imu)) < positioningTime
    k1 = k+nn-1;
    if k1 > size(trj.imu, 1)
        error('IMU data exhausted before first coupling epoch.');
    end
    wvm = trj.imu(k:k1,1:6);  t = trj.imu(k1,end);
    ins = insupdate(ins, wvm);
    kf.Phikk_1 = kffk(ins);
    kf = kfupdate(kf);

    k = k + nn;      
end


%% GNSS璺熻釜缁撴瀯浣撳垵濮嬪寲
activeChnList = find([trackResults.status] ~= '-');  
BitSyncTime = subFrameStart - 1;
activeChnList = activeChnList(BitSyncTime(activeChnList) > 0);
numActChnList = length(activeChnList);
if numActChnList < 4
    error('Too few valid channels after preamble sync filtering: %d', numActChnList);
end
ii = 1;
trackRawChIdx = zeros(1, numActChnList);
trackRawStartIdx = zeros(1, numActChnList);
for ch = activeChnList      
    trackDeepIn(ii).PRN = trackResults(ch).PRN;  
    
    trackDeepIn(ii).status = trackResults(ch).status;

    trackDeepIn(ii).SamplePos = trackResults(ch).absoluteSample(BitSyncTime(ch));
    

    trackDeepIn(ii).codeFreq = trackResults(ch).codeFreq(BitSyncTime(ch));

    trackDeepIn(ii).remCodePhase = trackResults(ch).remCodePhase(BitSyncTime(ch));

    trackDeepIn(ii).carrFreq = trackResults(ch).carrFreq(BitSyncTime(ch));
    
    trackDeepIn(ii).remCarrPhase = trackResults(ch).remCarrPhase(BitSyncTime(ch));


    trackDeepIn(ii).codeError = trackResults(ch).dllDiscr(BitSyncTime(ch));
    trackDeepIn(ii).codeNco = trackResults(ch).dllDiscrFilt(BitSyncTime(ch));
    

    trackDeepIn(ii).carrError = trackResults(ch).pllDiscr(BitSyncTime(ch));
    trackDeepIn(ii).carrNco = trackResults(ch).pllDiscrFilt(BitSyncTime(ch));
    

    trackDeepIn(ii).carrFreqBasis = channel(ch).acquiredFreq;

    trackDeepIn(ii).carrFreqBasis = trackResults(ch).carrFreq(BitSyncTime(ch));
    trackDeepIn(ii).carrNco = 0;
    trackDeepIn(ii).deepPrevMode = 0;
    trackDeepIn(ii).deepShadowFreqHz = trackDeepIn(ii).carrFreq - settings.IF;
    trackDeepIn(ii).deepShadowErrHz = 0;
    trackDeepIn(ii).deepAccumCarrierCycles = 0;
    trackDeepIn(ii).deepAccumCodeChips = 0;
    trackDeepIn(ii).deepCarrFreqStartHz = trackDeepIn(ii).carrFreq - settings.IF;
    trackDeepIn(ii).deepCarrFreqEndHz = trackDeepIn(ii).carrFreq - settings.IF;
    trackDeepIn(ii).deepCarrCmdHz = trackDeepIn(ii).carrFreq - settings.IF;
    trackDeepIn(ii).deepCarrBaseCmdHz = trackDeepIn(ii).carrFreq - settings.IF;
    trackDeepIn(ii).deepCarrShadowCmdHz = trackDeepIn(ii).carrFreq - settings.IF;
    trackDeepIn(ii).deepCarrAidFreqHz = 0;
    trackDeepIn(ii).deepCarrShadowTargetHz = nan;
    trackDeepIn(ii).deepCarrShadowPullHz = 0;
    trackDeepIn(ii).deepCarrShadowWeight = 0;
    trackDeepIn(ii).deepCarrForceShadow = false;
    trackDeepIn(ii).deepCarrPllBypass = false;
    trackDeepIn(ii).deepCarrOldNcoHz = 0;
    trackDeepIn(ii).deepCarrNcoHz = 0;
    trackDeepIn(ii).deepCarrNcoStepHz = 0;
    trackDeepIn(ii).deepCarrOldErrorCycles = trackDeepIn(ii).carrError;
    trackDeepIn(ii).deepCarrErrorRawCycles = trackDeepIn(ii).carrError;
    trackDeepIn(ii).deepCarrErrorScaledCycles = trackDeepIn(ii).carrError;
    trackDeepIn(ii).deepPromptI = nan;
    trackDeepIn(ii).deepPromptQ = nan;
    trackDeepIn(ii).deepEarlyI = nan;
    trackDeepIn(ii).deepEarlyQ = nan;
    trackDeepIn(ii).deepLateI = nan;
    trackDeepIn(ii).deepLateQ = nan;
    trackDeepIn(ii).deepDllDiscrRaw = nan;
    trackDeepIn(ii).deepDllDiscr = nan;
    trackDeepIn(ii).deepCodeReacqStableCnt = 0;
    trackDeepIn(ii).deepCodeReacqUsed = false;
    trackDeepIn(ii).deepCodeReacqOffsetChips = 0;
    trackDeepIn(ii).deepCodeReacqAppliedChips = 0;
    trackDeepIn(ii).deepCodeReacqMetric = nan;
    trackDeepIn(ii).deepCodeReacqPeakRatio = nan;
    trackDeepIn(ii).deepCodePhaseCorrChips = 0;
    trackDeepIn(ii).deepCodeWideCounter = 0;
    trackDeepIn(ii).deepCodeWideCandidateChips = nan;
    trackDeepIn(ii).deepCodeWideStableCnt = 0;
    trackDeepIn(ii).deepCodeWideUsed = false;
    trackDeepIn(ii).deepCodeWideOffsetChips = nan;
    trackDeepIn(ii).deepCodeWidePeakRatio = nan;
    trackDeepIn(ii).deepCodeWideOffsets = [];
    trackDeepIn(ii).deepCodeWideMetricAccum = [];
    trackDeepIn(ii).deepCodeWideMetricCount = 0;
    trackDeepIn(ii).deepCodeWideAccumCount = 0;
    trackDeepIn(ii).deepShadowCodeRefUsed = false;
    trackDeepIn(ii).deepShadowCodeRefOffsetChips = nan;
    trackDeepIn(ii).deepShadowCodeRefMetric = nan;
    trackDeepIn(ii).deepShadowCodeRefPeakRatio = nan;
    trackDeepIn(ii).deepShadowCodeRefAppliedChips = 0;
    trackDeepIn(ii).deepShadowCodeRefNcoPullHz = 0;
    trackDeepIn(ii).deepShadowCodeRefNcoStepHz = 0;
    trackDeepIn(ii).deepShadowInitHold = 0;
    trackDeepIn(ii).deepShadowCandOffsetChips = nan;
    trackDeepIn(ii).deepShadowCandFreqAbsHz = nan;
    trackDeepIn(ii).deepShadowInitFromClusterCenter = false;
    trackDeepIn(ii).deepShadowInitClusterScore = 0;
    trackDeepIn(ii).deepShadowInitClusterHits = 0;
    trackDeepIn(ii).deepShadowInitClusterMargin = 0;
    
    trackDeepIn(ii).numOfCoInt = 0;

    trackProcess(ii).codeErrorList = [];
    trackProcess(ii).carrErrorList = [];
    trackProcess(ii).codeFreqList = [];
    trackProcess(ii).carrFreqList = [];   
    trackProcess(ii).PLI = [];
    trackRawChIdx(ii) = ch;
    trackRawStartIdx(ii) = BitSyncTime(ch);
    
    ii = ii + 1;
end

% Filter satellites without complete ephemeris.
validEph = false(1, numActChnList);
for ii = 1 : numActChnList
    prn = trackDeepIn(ii).PRN;
    validEph(ii) = ~isempty(eph(prn).IODC) && ~isempty(eph(prn).IODE_sf2) && ...
                   ~isempty(eph(prn).IODE_sf3) && isscalar(eph(prn).a_f0) && ...
                   isscalar(eph(prn).a_f1) && isscalar(eph(prn).a_f2) && ...
                   isscalar(eph(prn).t_oc);
end
trackDeepIn = trackDeepIn(validEph);
trackProcess = trackProcess(validEph);
trackRawChIdx = trackRawChIdx(validEph);
trackRawStartIdx = trackRawStartIdx(validEph);
numActChnList = numel(trackDeepIn);
if numActChnList < 4
    error('Too few satellites with valid ephemeris for deep coupling: %d', numActChnList);
end

SamplePosatFirstFrame = zeros(1, numActChnList);
for ch = 1 : numActChnList
    SamplePosatFirstFrame(ch) = trackDeepIn(ch).SamplePos;
end

settings.recvTime = TOW + (settings.startOffset)/1000;  


recvTimeforFirstFrameperChannel = getTimeforFirstFrameEachChannel(settings, SamplePosatFirstFrame);
for ii = 1 : numActChnList
    trackDeepIn(ii).recvTime = recvTimeforFirstFrameperChannel(ii);  
end

%% GNSS杩愯鑷充簬棣栨缁勫悎鏃跺埢
I_P_1_list = [];
Q_P_1_list = [];


for ii = 1 : numActChnList
    trackans = trackDeepIn(ii);
    while trackans.recvTime < positioningTime    
        trackDeepIn(ii) = trackans;
        [trackans, I_P, Q_P] = perChannelTrackOnce(trackans, settings, fid);
        
            if settings.deepKeepTrackHistory && ii == 1
                I_P_1_list = [I_P_1_list, I_P];
                Q_P_1_list = [Q_P_1_list, Q_P];
            end

            if settings.deepKeepTrackHistory
                trackProcess(ii).codeErrorList = [trackProcess(ii).codeErrorList, trackans.codeError];
                trackProcess(ii).carrErrorList = [trackProcess(ii).carrErrorList, trackans.carrError];
                trackProcess(ii).codeFreqList = [trackProcess(ii).codeFreqList, trackans.codeFreq];
                trackProcess(ii).carrFreqList = [trackProcess(ii).carrFreqList, trackans.carrFreq];
                trackProcess(ii).PLI = [trackProcess(ii).PLI, (I_P^2-Q_P^2)/(I_P^2+Q_P^2)];
            end
    end
    trackDeepIn(ii) = trackans;
end
trackDeepDet = trackDeepIn;


%% 娣辩粍鍚?
roundTime = 40;
if isfield(settings, 'deepRoundTime') && settings.deepRoundTime > 0
    roundTime = floor(settings.deepRoundTime);
end
navPeriodSec = settings.navSolPeriod / 1000;
imuSpanSec = trj.imu(end, end) - trj.imu(1, end);
% Current deep-coupling loop requires roughly (roundTime + 1) navigation periods
% of IMU span (one pre-run step + one step after each epoch).
maxRoundTimeByImu = floor(imuSpanSec / navPeriodSec) - 1;
if maxRoundTimeByImu < 1
    error('IMU span is too short for deep coupling: span=%.3fs, navPeriod=%.3fs.', ...
        imuSpanSec, navPeriodSec);
end
if roundTime > maxRoundTimeByImu
    warning(['deepRoundTime=%d exceeds available IMU span (%.3fs). ' ...
             'Clamp deepRoundTime to %d.'], ...
            roundTime, imuSpanSec, maxRoundTimeByImu);
    roundTime = maxRoundTimeByImu;
end
navResults = [];
navResults.X = nan(1, roundTime); navResults.Y = nan(1, roundTime); navResults.Z = nan(1, roundTime); navResults.dt = nan(1, roundTime);
navResults.VX = nan(1, roundTime); navResults.VY = nan(1, roundTime); navResults.VZ = nan(1, roundTime);
navResults.spoofState = zeros(1, roundTime);  % 0 normal, 1 spoof
navResults.metricCmHz = nan(1, roundTime);
navResults.metricDfHz = nan(1, roundTime);
navResults.metricCmRawHz = nan(1, roundTime);
navResults.metricDfRawHz = nan(1, roundTime);
navResults.deepClkDriftHz = nan(1, roundTime);
navResults.prnList = [trackDeepIn.PRN];
navResults.residualHz = nan(numActChnList, roundTime);
navResults.residualBaseHz = nan(numActChnList, roundTime);
navResults.residualDetHz = nan(numActChnList, roundTime);
navResults.dopplerMeasHz = nan(numActChnList, roundTime);
navResults.dopplerPredHz = nan(numActChnList, roundTime);

settings.pllNoiseBandwidth = 3;    % 瀵逛簬test_522杩欑粍鏁版嵁锛孭LL鐨勫櫔澹板甫瀹戒负3鏃跺垰鍒氬け閿?
settings.dllNoiseBandwidth = 2;

oldAidFreq = zeros(numActChnList, 1);
closedLiftPrevAmbigChips = nan(numActChnList, 1);
closedLiftLastDelta = nan(numActChnList, 1);
closedLiftLastGoodEpoch = nan(numActChnList, 1);
closedLiftLastOffsetToBase = nan(numActChnList, 1);
closedLiftSegmentId = zeros(numActChnList, 1);
ds5CommonDragM = nan;
ds5CommonDragRateMps = 0.0;
ds5CommonDragLastEpoch = nan;
ds5CommonDoppHz = nan;
ds5CommonDoppRateHzps = 0.0;
ds5CommonDoppLastEpoch = nan;
confirmCnt = 0;
if ~isfield(settings, 'deepSpoofDetectEnable'), settings.deepSpoofDetectEnable = 0; end
if ~isfield(settings, 'deepTcmHz'), settings.deepTcmHz = 5.0; end
if ~isfield(settings, 'deepTdfHz'), settings.deepTdfHz = 3.0; end
if ~isfield(settings, 'deepTsatHz'), settings.deepTsatHz = 8.0; end
if ~isfield(settings, 'deepHitMin'), settings.deepHitMin = 3; end
if ~isfield(settings, 'deepConfirmEpochs'), settings.deepConfirmEpochs = 3; end
if ~isfield(settings, 'deepDetectArmEpoch'), settings.deepDetectArmEpoch = 1; end
if ~isfield(settings, 'deepSpoofConfirmMinSec'), settings.deepSpoofConfirmMinSec = -inf; end
if ~isfield(settings, 'deepSpoofForceConfirmSec'), settings.deepSpoofForceConfirmSec = nan; end
if ~isfield(settings, 'deepSpoofLatch'), settings.deepSpoofLatch = 1; end
if ~isfield(settings, 'deepAidWeightNormal'), settings.deepAidWeightNormal = 1.0; end
if ~isfield(settings, 'deepAidWeightSuspect'), settings.deepAidWeightSuspect = settings.deepAidWeightNormal; end
if ~isfield(settings, 'deepAidWeightSpoof'), settings.deepAidWeightSpoof = 1.0; end
if ~isfield(settings, 'deepClkWeightNormal'), settings.deepClkWeightNormal = 1.0; end
if ~isfield(settings, 'deepClkWeightSuspect'), settings.deepClkWeightSuspect = settings.deepClkWeightNormal; end
if ~isfield(settings, 'deepClkWeightSpoof'), settings.deepClkWeightSpoof = 1.0; end
if ~isfield(settings, 'deepInterpDiv'), settings.deepInterpDiv = 20; end
if ~isfield(settings, 'deepPllNoiseBandwidthNormal'), settings.deepPllNoiseBandwidthNormal = settings.pllNoiseBandwidth; end
if ~isfield(settings, 'deepPllNoiseBandwidthSuspect'), settings.deepPllNoiseBandwidthSuspect = settings.pllNoiseBandwidth; end
if ~isfield(settings, 'deepPllNoiseBandwidthSpoof'), settings.deepPllNoiseBandwidthSpoof = settings.pllNoiseBandwidth; end
settingsDet = settings;
settingsDet.pllNoiseBandwidth = settings.deepPllNoiseBandwidthNormal;
if ~isfield(settings, 'deepSigmaMeasHz'), settings.deepSigmaMeasHz = 1.5; end
if ~isfield(settings, 'deepKappaZ'), settings.deepKappaZ = 3.5; end
if ~isfield(settings, 'deepTz'), settings.deepTz = settings.deepKappaZ; end
if ~isfield(settings, 'deepClkDriftHzLimit'), settings.deepClkDriftHzLimit = 200; end
if ~isfield(settings, 'deepVelStateIdx'), settings.deepVelStateIdx = [4 5 6]; end
if ~isfield(settings, 'deepClkStateIdx'), settings.deepClkStateIdx = numel(kf.xk); end
if ~isfield(settings, 'deepBiasByPrnHz'), settings.deepBiasByPrnHz = zeros(1, 64); end
if ~isfield(settings, 'deepRhoBlendNormal'), settings.deepRhoBlendNormal = 0.0; end
if ~isfield(settings, 'deepRhoBlendSuspect'), settings.deepRhoBlendSuspect = 0.35; end
if ~isfield(settings, 'deepRhoBlendSpoof'), settings.deepRhoBlendSpoof = 1.0; end
if ~isfield(settings, 'deepUseGnssKfUpdate'), settings.deepUseGnssKfUpdate = 0; end
if ~isfield(settings, 'deepUseKfClockUpdate'), settings.deepUseKfClockUpdate = 1; end
if ~isfield(settings, 'deepUseGnssKfUpdateNormal'), settings.deepUseGnssKfUpdateNormal = settings.deepUseGnssKfUpdate; end
if ~isfield(settings, 'deepUseGnssKfUpdateSuspect'), settings.deepUseGnssKfUpdateSuspect = settings.deepUseGnssKfUpdateNormal; end
if ~isfield(settings, 'deepUseGnssKfUpdateSpoof'), settings.deepUseGnssKfUpdateSpoof = 0; end
if ~isfield(settings, 'deepUseKfClockUpdateNormal'), settings.deepUseKfClockUpdateNormal = settings.deepUseKfClockUpdate; end
if ~isfield(settings, 'deepUseKfClockUpdateSuspect'), settings.deepUseKfClockUpdateSuspect = settings.deepUseKfClockUpdateNormal; end
if ~isfield(settings, 'deepUseKfClockUpdateSpoof'), settings.deepUseKfClockUpdateSpoof = settings.deepUseKfClockUpdate; end
if ~isfield(settings, 'deepShadowUseNavQualifiedForKf'), settings.deepShadowUseNavQualifiedForKf = 0; end
if ~isfield(settings, 'deepShadowKfMinSat'), settings.deepShadowKfMinSat = 4; end
if ~isfield(settings, 'deepShadowKfRScale'), settings.deepShadowKfRScale = settings.deepRScaleSpoof; end
if ~isfield(settings, 'deepShadowKfMedianDetrend'), settings.deepShadowKfMedianDetrend = 1; end
if ~isfield(settings, 'deepShadowKfDetrendGateM'), settings.deepShadowKfDetrendGateM = 5000.0; end
if ~isfield(settings, 'deepRScaleNormal'), settings.deepRScaleNormal = 1.0; end
if ~isfield(settings, 'deepRScaleSuspect'), settings.deepRScaleSuspect = 4.0; end
if ~isfield(settings, 'deepRScaleSpoof'), settings.deepRScaleSpoof = 10.0; end
if ~isfield(settings, 'deepUseReferenceResidual'), settings.deepUseReferenceResidual = 0; end
if ~isfield(settings, 'deepRefResidualHz'), settings.deepRefResidualHz = []; end
if ~isfield(settings, 'deepDetectUseTrackResults'), settings.deepDetectUseTrackResults = 1; end
if ~isfield(settings, 'deepFastMode'), settings.deepFastMode = 1; end
if ~isfield(settings, 'deepKeepTrackHistory'), settings.deepKeepTrackHistory = 0; end
if ~isfield(settings, 'deepDetrendWindowEpoch'), settings.deepDetrendWindowEpoch = 80; end
if ~isfield(settings, 'deepShadowReacqEnable'), settings.deepShadowReacqEnable = 1; end
if ~isfield(settings, 'deepShadowResidualClipHz'), settings.deepShadowResidualClipHz = 3.0; end
if ~isfield(settings, 'deepShadowLockGateHz'), settings.deepShadowLockGateHz = 2.0; end
if ~isfield(settings, 'deepShadowStableEpochs'), settings.deepShadowStableEpochs = 4; end
if ~isfield(settings, 'deepShadowMinRecoveredSat'), settings.deepShadowMinRecoveredSat = 4; end
if ~isfield(settings, 'deepShadowReleaseTcmHz'), settings.deepShadowReleaseTcmHz = max(settings.deepTcmHz * 0.9, 0.1); end
if ~isfield(settings, 'deepShadowReleaseTdfHz'), settings.deepShadowReleaseTdfHz = max(settings.deepTdfHz * 0.9, 0.1); end
if ~isfield(settings, 'deepShadowReleaseTz'), settings.deepShadowReleaseTz = max(settings.deepTz * 0.9, 0.1); end
if ~isfield(settings, 'deepShadowReleaseConfirmEpochs'), settings.deepShadowReleaseConfirmEpochs = 8; end
if ~isfield(settings, 'deepShadowAllowReturnGnss'), settings.deepShadowAllowReturnGnss = 1; end
if ~isfield(settings, 'deepShadowRecoveryRampEpochs'), settings.deepShadowRecoveryRampEpochs = 20; end
if ~isfield(settings, 'deepShadowMaxPullHzPerStep'), settings.deepShadowMaxPullHzPerStep = 2.0; end
if ~isfield(settings, 'deepShadowWeightSuspect'), settings.deepShadowWeightSuspect = 0.50; end
if ~isfield(settings, 'deepShadowWeightSpoof'), settings.deepShadowWeightSpoof = 0.90; end
if ~isfield(settings, 'deepShadowTargetPureInsInSpoof'), settings.deepShadowTargetPureInsInSpoof = 1; end
if ~isfield(settings, 'deepShadowUseTrackResidualGate'), settings.deepShadowUseTrackResidualGate = 1; end
if ~isfield(settings, 'deepShadowUseCarrErrorGate'), settings.deepShadowUseCarrErrorGate = 1; end
if ~isfield(settings, 'deepShadowCarrErrGateCycles'), settings.deepShadowCarrErrGateCycles = 0.20; end
if ~isfield(settings, 'deepShadowReleaseUseTrackMetric'), settings.deepShadowReleaseUseTrackMetric = 1; end
if ~isfield(settings, 'deepShadowReleaseNeedDetectorMetric'), settings.deepShadowReleaseNeedDetectorMetric = 0; end
if ~isfield(settings, 'deepShadowReleaseTrackCmHz'), settings.deepShadowReleaseTrackCmHz = max(0.1, settings.deepShadowReleaseTcmHz); end
if ~isfield(settings, 'deepShadowReleaseTrackDfHz'), settings.deepShadowReleaseTrackDfHz = max(0.1, settings.deepShadowReleaseTdfHz); end
if ~isfield(settings, 'deepShadowReleaseUseNavResidual'), settings.deepShadowReleaseUseNavResidual = 0; end
if ~isfield(settings, 'deepShadowReleaseNavMedM'), settings.deepShadowReleaseNavMedM = 80.0; end
if ~isfield(settings, 'deepShadowReleaseNavP95M'), settings.deepShadowReleaseNavP95M = 300.0; end
if ~isfield(settings, 'deepShadowReleaseNavMaxM'), settings.deepShadowReleaseNavMaxM = 800.0; end
if ~isfield(settings, 'deepForceShadowNcoInSuspect'), settings.deepForceShadowNcoInSuspect = 0; end
if ~isfield(settings, 'deepForceShadowNcoInSpoof'), settings.deepForceShadowNcoInSpoof = 1; end
if ~isfield(settings, 'deepBypassPllInSuspect'), settings.deepBypassPllInSuspect = 0; end
if ~isfield(settings, 'deepBypassPllInSpoof'), settings.deepBypassPllInSpoof = 1; end
if ~isfield(settings, 'deepBypassDllInSuspect'), settings.deepBypassDllInSuspect = 0; end
if ~isfield(settings, 'deepBypassDllInSpoof'), settings.deepBypassDllInSpoof = 0; end
if ~isfield(settings, 'deepCodeErrorScaleSuspect'), settings.deepCodeErrorScaleSuspect = 1.0; end
if ~isfield(settings, 'deepCodeErrorScaleSpoof'), settings.deepCodeErrorScaleSpoof = 1.0; end
if ~isfield(settings, 'deepFreezeCodeErrorInSpoof'), settings.deepFreezeCodeErrorInSpoof = 0; end
if ~isfield(settings, 'deepCodeNcoStepLimitHz'), settings.deepCodeNcoStepLimitHz = inf; end
if ~isfield(settings, 'deepCodeReacqEnable'), settings.deepCodeReacqEnable = 0; end
if ~isfield(settings, 'deepCodeReacqMinMode'), settings.deepCodeReacqMinMode = 1; end
if ~isfield(settings, 'deepCodeReacqHalfChips'), settings.deepCodeReacqHalfChips = 0.50; end
if ~isfield(settings, 'deepCodeReacqNarrowHalfChips'), settings.deepCodeReacqNarrowHalfChips = 0.16; end
if ~isfield(settings, 'deepCodeReacqStepChips'), settings.deepCodeReacqStepChips = 0.04; end
if ~isfield(settings, 'deepCodeReacqNarrowAfter'), settings.deepCodeReacqNarrowAfter = 80; end
if ~isfield(settings, 'deepCodeReacqPeakRatio'), settings.deepCodeReacqPeakRatio = 1.05; end
if ~isfield(settings, 'deepCodeReacqZeroRatio'), settings.deepCodeReacqZeroRatio = 1.00; end
if ~isfield(settings, 'deepCodeReacqMaxShiftChips'), settings.deepCodeReacqMaxShiftChips = 0.08; end
if ~isfield(settings, 'deepCodeReacqApplyGain'), settings.deepCodeReacqApplyGain = 0.50; end
if ~isfield(settings, 'deepCodeReacqMinPromptMag'), settings.deepCodeReacqMinPromptMag = 0; end
if ~isfield(settings, 'deepCodeReacqNavCorrEnable'), settings.deepCodeReacqNavCorrEnable = 0; end
if ~isfield(settings, 'deepCodeReacqNavCorrGain'), settings.deepCodeReacqNavCorrGain = 0.20; end
if ~isfield(settings, 'deepCodeReacqNavCorrStepChips'), settings.deepCodeReacqNavCorrStepChips = 0.02; end
if ~isfield(settings, 'deepCodeReacqNavCorrLimitChips'), settings.deepCodeReacqNavCorrLimitChips = 0.50; end
if ~isfield(settings, 'deepCodeReacqNavCorrDecay'), settings.deepCodeReacqNavCorrDecay = 0.995; end
if ~isfield(settings, 'deepCodeWideReacqEnable'), settings.deepCodeWideReacqEnable = 0; end
if ~isfield(settings, 'deepCodeWideReacqMinMode'), settings.deepCodeWideReacqMinMode = 2; end
if ~isfield(settings, 'deepCodeWideReacqHalfChips'), settings.deepCodeWideReacqHalfChips = 5.0; end
if ~isfield(settings, 'deepCodeWideReacqStepChips'), settings.deepCodeWideReacqStepChips = 0.25; end
if ~isfield(settings, 'deepCodeWideReacqIntervalMs'), settings.deepCodeWideReacqIntervalMs = 50; end
if ~isfield(settings, 'deepCodeWideReacqPeakRatio'), settings.deepCodeWideReacqPeakRatio = 1.08; end
if ~isfield(settings, 'deepCodeWideReacqZeroRatio'), settings.deepCodeWideReacqZeroRatio = 1.03; end
if ~isfield(settings, 'deepCodeWideReacqStableEpochs'), settings.deepCodeWideReacqStableEpochs = 3; end
if ~isfield(settings, 'deepCodeWideReacqCandidateTolChips'), settings.deepCodeWideReacqCandidateTolChips = 0.50; end
if ~isfield(settings, 'deepCodeWideReacqAccumMs'), settings.deepCodeWideReacqAccumMs = 10; end
if ~isfield(settings, 'deepCodeWideReacqExcludeChips'), settings.deepCodeWideReacqExcludeChips = 0.50; end
if ~isfield(settings, 'deepCodeWideReacqUseSecondPeak'), settings.deepCodeWideReacqUseSecondPeak = 1; end
if ~isfield(settings, 'deepCodeWideReacqAccumRatio'), settings.deepCodeWideReacqAccumRatio = 1.02; end
if ~isfield(settings, 'deepCodeWideReacqCorrGain'), settings.deepCodeWideReacqCorrGain = 0.35; end
if ~isfield(settings, 'deepCodeWideReacqCorrStepChips'), settings.deepCodeWideReacqCorrStepChips = 0.25; end
if ~isfield(settings, 'deepCodeWideReacqCorrLimitChips'), settings.deepCodeWideReacqCorrLimitChips = 5.0; end
if ~isfield(settings, 'deepShadowAcqEnable'), settings.deepShadowAcqEnable = 0; end
if ~isfield(settings, 'deepShadowAcqIntervalEpochs'), settings.deepShadowAcqIntervalEpochs = 2; end
if ~isfield(settings, 'deepShadowAcqHalfChips'), settings.deepShadowAcqHalfChips = 20.0; end
if ~isfield(settings, 'deepShadowAcqStepChips'), settings.deepShadowAcqStepChips = 0.50; end
if ~isfield(settings, 'deepShadowAcqNoncohMs'), settings.deepShadowAcqNoncohMs = 10; end
if ~isfield(settings, 'deepShadowAcqExcludeChips'), settings.deepShadowAcqExcludeChips = 0.75; end
if ~isfield(settings, 'deepShadowAcqStableEpochs'), settings.deepShadowAcqStableEpochs = 4; end
if ~isfield(settings, 'deepShadowAcqOffsetTolChips'), settings.deepShadowAcqOffsetTolChips = 1.0; end
if ~isfield(settings, 'deepShadowAcqMinPeakRatio'), settings.deepShadowAcqMinPeakRatio = 1.08; end
if ~isfield(settings, 'deepShadowAcqMinZeroRatio'), settings.deepShadowAcqMinZeroRatio = 1.05; end
if ~isfield(settings, 'deepShadowAcqBoundaryGuardChips'), settings.deepShadowAcqBoundaryGuardChips = 1.0; end
if ~isfield(settings, 'deepShadowAcqMinSharpRatio'), settings.deepShadowAcqMinSharpRatio = 1.02; end
if ~isfield(settings, 'deepShadowAcqMinCenterPrior'), settings.deepShadowAcqMinCenterPrior = 0.02; end
if ~isfield(settings, 'deepShadowAcqMinSymmetryRatio'), settings.deepShadowAcqMinSymmetryRatio = 0.20; end
if ~isfield(settings, 'deepShadowAcqVoteBinChips'), settings.deepShadowAcqVoteBinChips = 1.0; end
if ~isfield(settings, 'deepShadowAcqVoteDecay'), settings.deepShadowAcqVoteDecay = 0.85; end
if ~isfield(settings, 'deepShadowAcqVoteHit'), settings.deepShadowAcqVoteHit = 1.0; end
if ~isfield(settings, 'deepShadowAcqVoteReady'), settings.deepShadowAcqVoteReady = 2.5; end
if ~isfield(settings, 'deepShadowAcqVoteMargin'), settings.deepShadowAcqVoteMargin = 0.75; end
if ~isfield(settings, 'deepShadowAcqVoteNeighborWeight'), settings.deepShadowAcqVoteNeighborWeight = 0.35; end
if ~isfield(settings, 'deepShadowAcqVoteHoldDecay'), settings.deepShadowAcqVoteHoldDecay = 0.93; end
if ~isfield(settings, 'deepShadowAcqVoteReadyHold'), settings.deepShadowAcqVoteReadyHold = 2; end
if ~isfield(settings, 'deepShadowTrackNum'), settings.deepShadowTrackNum = 3; end
if ~isfield(settings, 'deepShadowTrackAssocTolChips'), settings.deepShadowTrackAssocTolChips = 1.5; end
if ~isfield(settings, 'deepShadowTrackScoreDecay'), settings.deepShadowTrackScoreDecay = 0.90; end
if ~isfield(settings, 'deepShadowTrackMissDecay'), settings.deepShadowTrackMissDecay = 0.82; end
if ~isfield(settings, 'deepShadowTrackHitGain'), settings.deepShadowTrackHitGain = 1.0; end
if ~isfield(settings, 'deepShadowTrackReadyScore'), settings.deepShadowTrackReadyScore = 2.6; end
if ~isfield(settings, 'deepShadowTrackReadyHits'), settings.deepShadowTrackReadyHits = 3; end
if ~isfield(settings, 'deepShadowTrackReadyMargin'), settings.deepShadowTrackReadyMargin = 0.50; end
if ~isfield(settings, 'deepShadowConsensusBinChips'), settings.deepShadowConsensusBinChips = 1.0; end
if ~isfield(settings, 'deepShadowConsensusMinSat'), settings.deepShadowConsensusMinSat = 4; end
if ~isfield(settings, 'deepShadowConsensusStableEpochs'), settings.deepShadowConsensusStableEpochs = 3; end
if ~isfield(settings, 'deepShadowConsensusOffsetTolChips'), settings.deepShadowConsensusOffsetTolChips = 1.0; end
if ~isfield(settings, 'deepShadowUseCodeReadyForRecovery'), settings.deepShadowUseCodeReadyForRecovery = 1; end
if ~isfield(settings, 'deepShadowRawReacqEnable'), settings.deepShadowRawReacqEnable = 0; end
if ~isfield(settings, 'deepShadowRawReacqIntervalEpochs'), settings.deepShadowRawReacqIntervalEpochs = 4; end
if ~isfield(settings, 'deepShadowRawTrackEnable'), settings.deepShadowRawTrackEnable = 0; end
if ~isfield(settings, 'deepShadowContinuousTrackEnable'), settings.deepShadowContinuousTrackEnable = 0; end
if ~isfield(settings, 'deepShadowRawTrackPeakRatioMin'), settings.deepShadowRawTrackPeakRatioMin = 1.03; end
if ~isfield(settings, 'deepShadowRawTrackZeroRatioMin'), settings.deepShadowRawTrackZeroRatioMin = 1.05; end
if ~isfield(settings, 'deepShadowRawExplorePeakRatioMin'), settings.deepShadowRawExplorePeakRatioMin = 1.00; end
if ~isfield(settings, 'deepShadowRawExploreZeroRatioMin'), settings.deepShadowRawExploreZeroRatioMin = 0.90; end
if ~isfield(settings, 'deepShadowRawWeakVotePeakRatioMin'), settings.deepShadowRawWeakVotePeakRatioMin = 1.00; end
if ~isfield(settings, 'deepShadowRawWeakVoteZeroRatioMin'), settings.deepShadowRawWeakVoteZeroRatioMin = 0.12; end
if ~isfield(settings, 'deepShadowRawWeakVoteMetricScale'), settings.deepShadowRawWeakVoteMetricScale = 0.35; end
if ~isfield(settings, 'deepShadowRawExploreClusterScore'), settings.deepShadowRawExploreClusterScore = 1.0; end
if ~isfield(settings, 'deepShadowRawExploreClusterHits'), settings.deepShadowRawExploreClusterHits = 2; end
if ~isfield(settings, 'deepShadowRawExploreClusterMargin'), settings.deepShadowRawExploreClusterMargin = 0.10; end
if ~isfield(settings, 'deepShadowRawTrackProtectEpochs'), settings.deepShadowRawTrackProtectEpochs = 4; end
if ~isfield(settings, 'deepShadowRawTrackValidateEpochs'), settings.deepShadowRawTrackValidateEpochs = 2; end
if ~isfield(settings, 'deepShadowRawTrackValidateImproveMinM'), settings.deepShadowRawTrackValidateImproveMinM = 100.0; end
if ~isfield(settings, 'deepShadowRawTrackValidateAbsDeltaEnable'), settings.deepShadowRawTrackValidateAbsDeltaEnable = 0; end
if ~isfield(settings, 'deepShadowRawTrackValidateAbsDeltaMaxM'), settings.deepShadowRawTrackValidateAbsDeltaMaxM = 3500.0; end
if ~isfield(settings, 'deepShadowRawTrackValidateDuringHoldEnable'), settings.deepShadowRawTrackValidateDuringHoldEnable = 0; end
if ~isfield(settings, 'deepShadowRawTrackValidateMaxEpochs'), settings.deepShadowRawTrackValidateMaxEpochs = 8; end
if ~isfield(settings, 'deepShadowRawTrackValidateWindowEpochs'), settings.deepShadowRawTrackValidateWindowEpochs = 4; end
if ~isfield(settings, 'deepShadowRawTrackValidateWindowPassHits'), settings.deepShadowRawTrackValidateWindowPassHits = 2; end
if ~isfield(settings, 'deepShadowRawTrackScoreGoodImproveM'), settings.deepShadowRawTrackScoreGoodImproveM = 200.0; end
if ~isfield(settings, 'deepShadowRawTrackScoreBadImproveM'), settings.deepShadowRawTrackScoreBadImproveM = -300.0; end
if ~isfield(settings, 'deepShadowRawTrackScoreRiseM'), settings.deepShadowRawTrackScoreRiseM = 150.0; end
if ~isfield(settings, 'deepShadowRawTrackScorePassMin'), settings.deepShadowRawTrackScorePassMin = 2.0; end
if ~isfield(settings, 'deepShadowRawTrackScoreRejectMin'), settings.deepShadowRawTrackScoreRejectMin = -2.0; end
if ~isfield(settings, 'deepShadowRawTrackEarlyRejectEpochs'), settings.deepShadowRawTrackEarlyRejectEpochs = 2; end
if ~isfield(settings, 'deepShadowRawTrackEarlyRejectImproveMaxM'), settings.deepShadowRawTrackEarlyRejectImproveMaxM = -200.0; end
if ~isfield(settings, 'deepShadowRawCandReuseMaxEpochs'), settings.deepShadowRawCandReuseMaxEpochs = 6; end
if ~isfield(settings, 'deepShadowRawCandTopKUse'), settings.deepShadowRawCandTopKUse = 3; end
if ~isfield(settings, 'deepShadowRawClusterAssocTolChips'), settings.deepShadowRawClusterAssocTolChips = 1.0; end
if ~isfield(settings, 'deepShadowRawClusterAssocTolFreqHz'), settings.deepShadowRawClusterAssocTolFreqHz = 250; end
if ~isfield(settings, 'deepShadowRawClusterScoreDecay'), settings.deepShadowRawClusterScoreDecay = 0.85; end
if ~isfield(settings, 'deepShadowRawClusterDecayPerReacq'), settings.deepShadowRawClusterDecayPerReacq = 1; end
if ~isfield(settings, 'deepShadowRawClusterHitGain'), settings.deepShadowRawClusterHitGain = 1.0; end
if ~isfield(settings, 'deepShadowRawClusterMissDecay'), settings.deepShadowRawClusterMissDecay = 0.75; end
if ~isfield(settings, 'deepShadowRawClusterReadyScore'), settings.deepShadowRawClusterReadyScore = 2.0; end
if ~isfield(settings, 'deepShadowRawClusterReadyHits'), settings.deepShadowRawClusterReadyHits = 2; end
if ~isfield(settings, 'deepShadowRawClusterReadyMargin'), settings.deepShadowRawClusterReadyMargin = 0.5; end
if ~isfield(settings, 'deepShadowRawClusterValidateTolChips'), settings.deepShadowRawClusterValidateTolChips = 1.5; end
if ~isfield(settings, 'deepShadowRawClusterValidateTolFreqHz'), settings.deepShadowRawClusterValidateTolFreqHz = 300; end
if ~isfield(settings, 'deepShadowRawClusterPassScore'), settings.deepShadowRawClusterPassScore = 1.4; end
if ~isfield(settings, 'deepShadowRawClusterPassHits'), settings.deepShadowRawClusterPassHits = 3; end
if ~isfield(settings, 'deepShadowRawClusterPassMargin'), settings.deepShadowRawClusterPassMargin = 0.6; end
if ~isfield(settings, 'deepShadowRawClusterPassImproveRelaxM'), settings.deepShadowRawClusterPassImproveRelaxM = 0.0; end
if ~isfield(settings, 'deepShadowRawClusterInitScore'), settings.deepShadowRawClusterInitScore = 1.2; end
if ~isfield(settings, 'deepShadowRawClusterInitHits'), settings.deepShadowRawClusterInitHits = 3; end
if ~isfield(settings, 'deepShadowRawClusterInitMargin'), settings.deepShadowRawClusterInitMargin = 0.15; end
if ~isfield(settings, 'deepShadowRawClusterDirectInitScore'), settings.deepShadowRawClusterDirectInitScore = 1.6; end
if ~isfield(settings, 'deepShadowRawClusterDirectInitHits'), settings.deepShadowRawClusterDirectInitHits = 4; end
if ~isfield(settings, 'deepShadowRawClusterDirectInitMargin'), settings.deepShadowRawClusterDirectInitMargin = 0.20; end
if ~isfield(settings, 'deepShadowRawClusterInitBlendAlpha'), settings.deepShadowRawClusterInitBlendAlpha = 0.65; end
if ~isfield(settings, 'deepShadowRawCandMetricWeight'), settings.deepShadowRawCandMetricWeight = 0.60; end
if ~isfield(settings, 'deepShadowRawCandClusterWeight'), settings.deepShadowRawCandClusterWeight = 0.85; end
if ~isfield(settings, 'deepShadowRawCandDominantWeight'), settings.deepShadowRawCandDominantWeight = 0.35; end
if ~isfield(settings, 'deepShadowRawCandMetricGapSoft'), settings.deepShadowRawCandMetricGapSoft = 0.12; end
if ~isfield(settings, 'deepShadowRawCandClusterOnlyGapSoft'), settings.deepShadowRawCandClusterOnlyGapSoft = 0.06; end
if ~isfield(settings, 'deepShadowRawCandClusterOnlyMinSupport'), settings.deepShadowRawCandClusterOnlyMinSupport = 0.85; end
if ~isfield(settings, 'deepShadowRawFailFallbackMinClusterSupport'), settings.deepShadowRawFailFallbackMinClusterSupport = 0.35; end
if ~isfield(settings, 'deepShadowRawClusterInitUseCenterScore'), settings.deepShadowRawClusterInitUseCenterScore = 2.6; end
if ~isfield(settings, 'deepShadowRawClusterInitUseCenterHits'), settings.deepShadowRawClusterInitUseCenterHits = 4; end
if ~isfield(settings, 'deepShadowRawClusterInitUseCenterMargin'), settings.deepShadowRawClusterInitUseCenterMargin = 0.55; end
if ~isfield(settings, 'deepShadowRawClusterInitCenterBlendAlpha'), settings.deepShadowRawClusterInitCenterBlendAlpha = 0.20; end
if ~isfield(settings, 'deepShadowRawFailExcludeTolChips'), settings.deepShadowRawFailExcludeTolChips = 1.5; end
if ~isfield(settings, 'deepShadowRawFailExcludeTolFreqHz'), settings.deepShadowRawFailExcludeTolFreqHz = 300; end
if ~isfield(settings, 'deepShadowRawFailExcludeMaxAgeEpochs'), settings.deepShadowRawFailExcludeMaxAgeEpochs = 24; end
if ~isfield(settings, 'deepShadowRawTrackClusterWindowImproveMinM'), settings.deepShadowRawTrackClusterWindowImproveMinM = 0.0; end
if ~isfield(settings, 'deepShadowRawTrackClusterInstantFloorM'), settings.deepShadowRawTrackClusterInstantFloorM = -150.0; end
if ~isfield(settings, 'deepShadowRawTrackClusterRejectFloorM'), settings.deepShadowRawTrackClusterRejectFloorM = -600.0; end
if ~isfield(settings, 'deepShadowRawTrackClusterSoftRejectEpochs'), settings.deepShadowRawTrackClusterSoftRejectEpochs = 3; end
if ~isfield(settings, 'deepShadowRawTrackClusterTrendMeanMinM'), settings.deepShadowRawTrackClusterTrendMeanMinM = -50.0; end
if ~isfield(settings, 'deepShadowRawTrackClusterTrendMinM'), settings.deepShadowRawTrackClusterTrendMinM = -300.0; end
if ~isfield(settings, 'deepShadowRawTrackClusterTrendPassHits'), settings.deepShadowRawTrackClusterTrendPassHits = 3; end
if ~isfield(settings, 'deepShadowRawTrackClusterRecheckBudget'), settings.deepShadowRawTrackClusterRecheckBudget = 2; end
if ~isfield(settings, 'deepShadowRawTrackClusterRecheckHoldEpochs'), settings.deepShadowRawTrackClusterRecheckHoldEpochs = 2; end
if ~isfield(settings, 'deepShadowRawTrackClusterExtraMaxEpochs'), settings.deepShadowRawTrackClusterExtraMaxEpochs = 4; end
if ~isfield(settings, 'deepShadowRawRelayCooldownEpochs'), settings.deepShadowRawRelayCooldownEpochs = 2; end
if ~isfield(settings, 'deepShadowRawPassReadyAgingEnable'), settings.deepShadowRawPassReadyAgingEnable = 0; end
if ~isfield(settings, 'deepShadowRawPassReadyRevokeEnable'), settings.deepShadowRawPassReadyRevokeEnable = 1; end
if ~isfield(settings, 'deepShadowRawPassUseActivePool'), settings.deepShadowRawPassUseActivePool = 0; end
if ~isfield(settings, 'deepShadowRawPassReadyBadEpochs'), settings.deepShadowRawPassReadyBadEpochs = 3; end
if ~isfield(settings, 'deepShadowRawPassReadyMinSet'), settings.deepShadowRawPassReadyMinSet = 4; end
if ~isfield(settings, 'deepShadowRawPassReadyDevThrM'), settings.deepShadowRawPassReadyDevThrM = 6000.0; end
if ~isfield(settings, 'deepShadowRawPassReadyAbsThrM'), settings.deepShadowRawPassReadyAbsThrM = 20000.0; end
if ~isfield(settings, 'deepShadowRawNavExpandEnable'), settings.deepShadowRawNavExpandEnable = 0; end
if ~isfield(settings, 'deepShadowRawNavCoreDevThrM'), settings.deepShadowRawNavCoreDevThrM = inf; end
if ~isfield(settings, 'deepShadowRawNavExpandDevThrM'), settings.deepShadowRawNavExpandDevThrM = 5000.0; end
if ~isfield(settings, 'deepShadowDs5CommonDragEnable'), settings.deepShadowDs5CommonDragEnable = 0; end
if ~isfield(settings, 'deepShadowDs5CommonDragStartSec'), settings.deepShadowDs5CommonDragStartSec = 118.0; end
if ~isfield(settings, 'deepShadowDs5CommonDragFocusPrns'), settings.deepShadowDs5CommonDragFocusPrns = [27 3]; end
if ~isfield(settings, 'deepShadowDs5CommonDragMinBaseSat'), settings.deepShadowDs5CommonDragMinBaseSat = 3; end
if ~isfield(settings, 'deepShadowDs5CommonDragGateFloorM'), settings.deepShadowDs5CommonDragGateFloorM = 250.0; end
if ~isfield(settings, 'deepShadowDs5CommonDragMadScale'), settings.deepShadowDs5CommonDragMadScale = 4.0; end
if ~isfield(settings, 'deepShadowDs5CommonDragMaxInnovM'), settings.deepShadowDs5CommonDragMaxInnovM = 6000.0; end
if ~isfield(settings, 'deepShadowDs5CommonDragMaxRateMps'), settings.deepShadowDs5CommonDragMaxRateMps = 4000.0; end
if ~isfield(settings, 'deepShadowDs5CommonDoppEnable'), settings.deepShadowDs5CommonDoppEnable = 0; end
if ~isfield(settings, 'deepShadowDs5CommonDoppStartSec'), settings.deepShadowDs5CommonDoppStartSec = 92.0; end
if ~isfield(settings, 'deepShadowDs5CommonDoppFocusPrns'), settings.deepShadowDs5CommonDoppFocusPrns = [27 3]; end
if ~isfield(settings, 'deepShadowDs5CommonDoppMinBaseSat'), settings.deepShadowDs5CommonDoppMinBaseSat = 3; end
if ~isfield(settings, 'deepShadowDs5CommonDoppGateFloorHz'), settings.deepShadowDs5CommonDoppGateFloorHz = 10.0; end
if ~isfield(settings, 'deepShadowDs5CommonDoppMadScale'), settings.deepShadowDs5CommonDoppMadScale = 4.0; end
if ~isfield(settings, 'deepShadowDs5CommonDoppMaxInnovHz'), settings.deepShadowDs5CommonDoppMaxInnovHz = 80.0; end
if ~isfield(settings, 'deepShadowDs5CommonDoppMaxRateHzps'), settings.deepShadowDs5CommonDoppMaxRateHzps = 120.0; end
if ~isfield(settings, 'deepShadowDs5CommonDoppApplyToAidEnable'), settings.deepShadowDs5CommonDoppApplyToAidEnable = 1; end
if ~isfield(settings, 'deepShadowDs5CommonDoppAlphaGateEnable'), settings.deepShadowDs5CommonDoppAlphaGateEnable = 0; end
if ~isfield(settings, 'deepShadowDs5CommonDoppAlphaProxyMin'), settings.deepShadowDs5CommonDoppAlphaProxyMin = 0.0; end
if ~isfield(settings, 'deepShadowDs5CommonDoppAidApplyMaxHz'), settings.deepShadowDs5CommonDoppAidApplyMaxHz = 200.0; end
if ~isfield(settings, 'deepShadowDs5RefObsModelEnable'), settings.deepShadowDs5RefObsModelEnable = 0; end
if ~isfield(settings, 'deepShadowDs5RefObsModelStartSec'), settings.deepShadowDs5RefObsModelStartSec = 92.0; end
if ~isfield(settings, 'deepShadowDs5RefObsAutoActivate'), settings.deepShadowDs5RefObsAutoActivate = 1; end
if ~isfield(settings, 'deepShadowDs5RefObsMinBaseSat'), settings.deepShadowDs5RefObsMinBaseSat = 3; end
if ~isfield(settings, 'deepShadowDs5RefObsLocalCorrMaxChips'), settings.deepShadowDs5RefObsLocalCorrMaxChips = 1.50; end
if ~isfield(settings, 'deepShadowDs5RefObsUseCodeCorr'), settings.deepShadowDs5RefObsUseCodeCorr = 1; end
if ~isfield(settings, 'deepShadowDs5RefObsRequireActiveTrack'), settings.deepShadowDs5RefObsRequireActiveTrack = 0; end
if ~isfield(settings, 'deepShadowDs5RefObsSeedUseBaseFallback'), settings.deepShadowDs5RefObsSeedUseBaseFallback = 1; end
if ~isfield(settings, 'deepShadowDs5RefObsSeedMaxDevFromBaseM'), settings.deepShadowDs5RefObsSeedMaxDevFromBaseM = 1500.0; end
if ~isfield(settings, 'deepShadowDs5RefObsPositionCommonClockExcludePrns'), settings.deepShadowDs5RefObsPositionCommonClockExcludePrns = [21 27 9]; end
if ~isfield(settings, 'deepShadowDs5RefObsPositionCommonClockFallbackPrns'), settings.deepShadowDs5RefObsPositionCommonClockFallbackPrns = [9 21 27]; end
if ~isfield(settings, 'deepShadowDs5CodeRefHalfChips'), settings.deepShadowDs5CodeRefHalfChips = 0.50; end
if ~isfield(settings, 'deepShadowDs5CodeRefStepChips'), settings.deepShadowDs5CodeRefStepChips = 0.10; end
if ~isfield(settings, 'deepShadowDs5CodeRefPeakRatioMin'), settings.deepShadowDs5CodeRefPeakRatioMin = 1.01; end
if ~isfield(settings, 'deepShadowDs5CodeRefZeroRatioMin'), settings.deepShadowDs5CodeRefZeroRatioMin = 1.00; end
if ~isfield(settings, 'deepShadowDs5CodeRefApplyGain'), settings.deepShadowDs5CodeRefApplyGain = 0.50; end
if ~isfield(settings, 'deepShadowDs5CodeRefStepMaxChips'), settings.deepShadowDs5CodeRefStepMaxChips = 0.08; end
if ~isfield(settings, 'deepShadowDs5CodeRefTotalMaxChips'), settings.deepShadowDs5CodeRefTotalMaxChips = 1.50; end
if ~isfield(settings, 'deepShadowDs5CodeRefMinPromptMag'), settings.deepShadowDs5CodeRefMinPromptMag = 0; end
if ~isfield(settings, 'deepShadowDs5CodeRefNcoPullEnable'), settings.deepShadowDs5CodeRefNcoPullEnable = 1; end
if ~isfield(settings, 'deepShadowDs5CodeRefNcoGain'), settings.deepShadowDs5CodeRefNcoGain = 1.0; end
if ~isfield(settings, 'deepShadowDs5CodeRefNcoMaxHz'), settings.deepShadowDs5CodeRefNcoMaxHz = 120.0; end
if ~isfield(settings, 'deepShadowDs5RefObsRecoveryEnable'), settings.deepShadowDs5RefObsRecoveryEnable = 1; end
if ~isfield(settings, 'deepShadowDs5RefObsRecoveryStartSec'), settings.deepShadowDs5RefObsRecoveryStartSec = 92.0; end
if ~isfield(settings, 'deepShadowDs5RefObsRecoveryMinSat'), settings.deepShadowDs5RefObsRecoveryMinSat = 4; end
if ~isfield(settings, 'deepShadowDs5RefObsRecoveryMinUsedSat'), settings.deepShadowDs5RefObsRecoveryMinUsedSat = 4; end
if ~isfield(settings, 'deepShadowDs5RefObsRecoveryCodeMedMaxChips'), settings.deepShadowDs5RefObsRecoveryCodeMedMaxChips = 0.50; end
if ~isfield(settings, 'deepShadowDs5RefObsRecoveryCodeP95MaxChips'), settings.deepShadowDs5RefObsRecoveryCodeP95MaxChips = 1.20; end
if ~isfield(settings, 'deepShadowDs5RefObsRecoveryFreqP95MaxHz'), settings.deepShadowDs5RefObsRecoveryFreqP95MaxHz = 300.0; end
if ~isfield(settings, 'deepShadowDs5RefObsRecoveryRobustEnable'), settings.deepShadowDs5RefObsRecoveryRobustEnable = 0; end
if ~isfield(settings, 'deepShadowDs5RefObsRecoveryRobustMaxDropSat'), settings.deepShadowDs5RefObsRecoveryRobustMaxDropSat = 2; end
if ~isfield(settings, 'deepShadowDs5RefObsPositionUseRecoveryKeep'), settings.deepShadowDs5RefObsPositionUseRecoveryKeep = 1; end
if ~isfield(settings, 'deepShadowDs5RefObsTrackingEnable'), settings.deepShadowDs5RefObsTrackingEnable = 1; end
if ~isfield(settings, 'deepShadowDs5FastRefObsTrackBypass'), settings.deepShadowDs5FastRefObsTrackBypass = 1; end
if ~isfield(settings, 'deepShadowDs5RefObsTrackRawStrideEpochs'), settings.deepShadowDs5RefObsTrackRawStrideEpochs = 1; end
if ~isfield(settings, 'deepShadowDs5RefObsTrackRawBurstMs'), settings.deepShadowDs5RefObsTrackRawBurstMs = inf; end
if ~isfield(settings, 'deepShadowDs5RefObsTrackRawWarmupEpochs'), settings.deepShadowDs5RefObsTrackRawWarmupEpochs = 0; end
if ~isfield(settings, 'deepShadowDs5RefObsTrackingCodeMedMaxChips'), settings.deepShadowDs5RefObsTrackingCodeMedMaxChips = 0.75; end
if ~isfield(settings, 'deepShadowDs5RefObsTrackingCodeP95MaxChips'), settings.deepShadowDs5RefObsTrackingCodeP95MaxChips = 1.60; end
if ~isfield(settings, 'deepShadowDs5RefObsTrackingFreqP95MaxHz'), settings.deepShadowDs5RefObsTrackingFreqP95MaxHz = 350.0; end
if ~isfield(settings, 'deepShadowDs5RefObsTrackingHoldMaxEpochs'), settings.deepShadowDs5RefObsTrackingHoldMaxEpochs = 8; end
if ~isfield(settings, 'deepShadowDs5ObsContractEnable'), settings.deepShadowDs5ObsContractEnable = 1; end
if ~isfield(settings, 'deepShadowDs5ObsContractMinSat'), settings.deepShadowDs5ObsContractMinSat = 4; end
if ~isfield(settings, 'deepShadowDs5ObsContractSpreadP95MaxM'), settings.deepShadowDs5ObsContractSpreadP95MaxM = 800.0; end
if ~isfield(settings, 'deepShadowDs5ObsContractRequireCommonClock'), settings.deepShadowDs5ObsContractRequireCommonClock = 1; end
if ~isfield(settings, 'deepShadowDs5ObsContractRequireTrackingPass'), settings.deepShadowDs5ObsContractRequireTrackingPass = 1; end
if ~isfield(settings, 'deepShadowDs5ObsContractMaxBaseSeedFrac'), settings.deepShadowDs5ObsContractMaxBaseSeedFrac = 0.75; end
if ~isfield(settings, 'deepShadowDs5ObsContractNoBaselineSeedForRecovery'), settings.deepShadowDs5ObsContractNoBaselineSeedForRecovery = 0; end
if ~isfield(settings, 'deepShadowDs5ObsContractUseRecoveryKeep'), settings.deepShadowDs5ObsContractUseRecoveryKeep = 0; end
if ~isfield(settings, 'deepShadowDs5ObsContractRobustTrackingEnable'), settings.deepShadowDs5ObsContractRobustTrackingEnable = 0; end
if ~isfield(settings, 'deepShadowDs5ObsContractRobustTrackingMaxDropSat'), settings.deepShadowDs5ObsContractRobustTrackingMaxDropSat = 2; end
if ~isfield(settings, 'deepShadowDs5RefObsRecoveryRequireBranch'), settings.deepShadowDs5RefObsRecoveryRequireBranch = 0; end
if ~isfield(settings, 'deepShadowDs5RefObsRecoveryBranchPostfitMaxM'), settings.deepShadowDs5RefObsRecoveryBranchPostfitMaxM = 250.0; end
if ~isfield(settings, 'deepShadowDs5RefObsRecoveryBaselineDiffMaxM'), settings.deepShadowDs5RefObsRecoveryBaselineDiffMaxM = 2500.0; end
if ~isfield(settings, 'deepShadowDs5RefObsPositionAdaptiveDynGateEnable'), settings.deepShadowDs5RefObsPositionAdaptiveDynGateEnable = 1; end
if ~isfield(settings, 'deepShadowDs5RefObsPositionAdaptiveDeltaSpreadP95MaxM'), settings.deepShadowDs5RefObsPositionAdaptiveDeltaSpreadP95MaxM = 400.0; end
if ~isfield(settings, 'deepShadowDs5RefObsPositionAdaptivePostfitMaxM'), settings.deepShadowDs5RefObsPositionAdaptivePostfitMaxM = 220.0; end
if ~isfield(settings, 'deepShadowDs5RefObsPositionAdaptiveCorrMaxM'), settings.deepShadowDs5RefObsPositionAdaptiveCorrMaxM = 260.0; end
if ~isfield(settings, 'deepShadowDs5RefObsPositionAdaptiveAnchorDiffMaxM'), settings.deepShadowDs5RefObsPositionAdaptiveAnchorDiffMaxM = 260.0; end
if ~isfield(settings, 'deepShadowDs5RefObsPositionAdaptivePredVelDiffMaxMps'), settings.deepShadowDs5RefObsPositionAdaptivePredVelDiffMaxMps = 220.0; end
if ~isfield(settings, 'deepShadowDs5RefObsPositionAdaptiveJumpMaxM'), settings.deepShadowDs5RefObsPositionAdaptiveJumpMaxM = 450.0; end
if ~isfield(settings, 'deepShadowDs5RefObsPositionAdaptiveSpeedMaxMps'), settings.deepShadowDs5RefObsPositionAdaptiveSpeedMaxMps = 450.0; end
if ~isfield(settings, 'deepShadowDs5RefObsPositionAdaptiveAccelMaxMps2'), settings.deepShadowDs5RefObsPositionAdaptiveAccelMaxMps2 = 700.0; end
if ~isfield(settings, 'deepShadowDs5FinalForceRefObsRecovered'), settings.deepShadowDs5FinalForceRefObsRecovered = 0; end
if ~isfield(settings, 'deepShadowDs5FinalQualitySelectEnable'), settings.deepShadowDs5FinalQualitySelectEnable = 1; end
if ~isfield(settings, 'deepShadowDs5FinalQualityImproveMinM'), settings.deepShadowDs5FinalQualityImproveMinM = -10.0; end
if ~isfield(settings, 'deepShadowDs5FinalQualityBaselineDiffMaxM'), settings.deepShadowDs5FinalQualityBaselineDiffMaxM = 180.0; end
if ~isfield(settings, 'deepShadowDs5FinalRecoveredAuthorityRequireBaselineGate'), settings.deepShadowDs5FinalRecoveredAuthorityRequireBaselineGate = 1; end
if ~isfield(settings, 'deepShadowDs5FinalRecoveredAuthorityMaxBaselineDiffM'), settings.deepShadowDs5FinalRecoveredAuthorityMaxBaselineDiffM = settings.deepShadowDs5FinalQualityBaselineDiffMaxM; end
if ~isfield(settings, 'deepShadowDs5FinalAllowNoBaselineRecovered'), settings.deepShadowDs5FinalAllowNoBaselineRecovered = 0; end
if ~isfield(settings, 'deepShadowDs5FinalRecoveredConfirmEpochs'), settings.deepShadowDs5FinalRecoveredConfirmEpochs = 3; end
if ~isfield(settings, 'deepShadowDs5FinalContinuityGateEnable'), settings.deepShadowDs5FinalContinuityGateEnable = 1; end
if ~isfield(settings, 'deepShadowDs5FinalContinuityMaxPredDiffM'), settings.deepShadowDs5FinalContinuityMaxPredDiffM = 60.0; end
if ~isfield(settings, 'deepShadowDs5FinalContinuitySlewEnable'), settings.deepShadowDs5FinalContinuitySlewEnable = 1; end
if ~isfield(settings, 'deepShadowDs5FinalContinuityMaxAgeEpochs'), settings.deepShadowDs5FinalContinuityMaxAgeEpochs = 6; end
if ~isfield(settings, 'deepShadowDs5FinalHoldBaselineCompeteEnable'), settings.deepShadowDs5FinalHoldBaselineCompeteEnable = 1; end
if ~isfield(settings, 'deepShadowDs5FinalHoldBaselineMinAgeEpochs'), settings.deepShadowDs5FinalHoldBaselineMinAgeEpochs = 2; end
if ~isfield(settings, 'deepShadowDs5FinalHoldBaselineProxyMarginM'), settings.deepShadowDs5FinalHoldBaselineProxyMarginM = 20.0; end
if ~isfield(settings, 'deepShadowDs5FinalHoldBaselineDiffMaxM'), settings.deepShadowDs5FinalHoldBaselineDiffMaxM = 220.0; end
if ~isfield(settings, 'deepShadowDs5FinalHoldBaselineForceDiffM'), settings.deepShadowDs5FinalHoldBaselineForceDiffM = 220.0; end
if ~isfield(settings, 'deepShadowDs5FinalHoldBaselineMaxAgeEpochs'), settings.deepShadowDs5FinalHoldBaselineMaxAgeEpochs = 6; end
if ~isfield(settings, 'deepShadowDs5FinalRecoveredHoldAllowObsGap'), settings.deepShadowDs5FinalRecoveredHoldAllowObsGap = 0; end
if ~isfield(settings, 'deepShadowDs5FinalRecoveredHoldAllowAuthorityGap'), settings.deepShadowDs5FinalRecoveredHoldAllowAuthorityGap = 0; end
if ~isfield(settings, 'deepShadowDs5FinalRecoveredHoldSuppressBaselineCompete'), settings.deepShadowDs5FinalRecoveredHoldSuppressBaselineCompete = 0; end
if ~isfield(settings, 'deepShadowDs5FinalRecoveredHoldBaselineDiffMaxM')
    if isfield(settings, 'deepShadowDs5RecoveredFilterAuthorityBaselineDiffMaxM')
        settings.deepShadowDs5FinalRecoveredHoldBaselineDiffMaxM = settings.deepShadowDs5RecoveredFilterAuthorityBaselineDiffMaxM;
    else
        settings.deepShadowDs5FinalRecoveredHoldBaselineDiffMaxM = settings.deepShadowDs5FinalQualityBaselineDiffMaxM;
    end
end
if ~isfield(settings, 'deepShadowDs5FinalRecoveredAuthorityEnable'), settings.deepShadowDs5FinalRecoveredAuthorityEnable = 1; end
if ~isfield(settings, 'deepShadowDs5FinalBaselineOffAblation'), settings.deepShadowDs5FinalBaselineOffAblation = 0; end
if ~isfield(settings, 'deepShadowDs5FinalRequireObsContractForRecovered'), settings.deepShadowDs5FinalRequireObsContractForRecovered = 1; end
if ~isfield(settings, 'deepShadowDs5FinalRequireObsContractForHold'), settings.deepShadowDs5FinalRequireObsContractForHold = 1; end
if ~isfield(settings, 'deepShadowDs5FinalAllowObsContractTrackingHold'), settings.deepShadowDs5FinalAllowObsContractTrackingHold = 0; end
if ~isfield(settings, 'deepShadowDs5FinalAllowObsContractBaseSeedHold'), settings.deepShadowDs5FinalAllowObsContractBaseSeedHold = 0; end
if ~isfield(settings, 'deepShadowDs5RecoveredFilterEnable'), settings.deepShadowDs5RecoveredFilterEnable = 1; end
if ~isfield(settings, 'deepShadowDs5RecoveredFilterConfirmEpochs'), settings.deepShadowDs5RecoveredFilterConfirmEpochs = 3; end
if ~isfield(settings, 'deepShadowDs5RecoveredFilterMaxBadEpochs'), settings.deepShadowDs5RecoveredFilterMaxBadEpochs = 24; end
if ~isfield(settings, 'deepShadowDs5RecoveredFilterMaxCoastEpochs'), settings.deepShadowDs5RecoveredFilterMaxCoastEpochs = 80; end
if ~isfield(settings, 'deepShadowDs5RecoveredFilterUpdateAlpha'), settings.deepShadowDs5RecoveredFilterUpdateAlpha = 0.85; end
if ~isfield(settings, 'deepShadowDs5RecoveredFilterVelMeasBlend'), settings.deepShadowDs5RecoveredFilterVelMeasBlend = 0.35; end
if ~isfield(settings, 'deepShadowDs5RecoveredFilterVelInsBlend'), settings.deepShadowDs5RecoveredFilterVelInsBlend = 0.10; end
if ~isfield(settings, 'deepShadowDs5RecoveredFilterMeasMaxPredDiffM'), settings.deepShadowDs5RecoveredFilterMeasMaxPredDiffM = 350.0; end
if ~isfield(settings, 'deepShadowDs5RecoveredFilterClockEnable'), settings.deepShadowDs5RecoveredFilterClockEnable = 1; end
if ~isfield(settings, 'deepShadowDs5RecoveredFilterClockUpdateAlpha'), settings.deepShadowDs5RecoveredFilterClockUpdateAlpha = 0.85; end
if ~isfield(settings, 'deepShadowDs5RecoveredFilterClockRateBlend'), settings.deepShadowDs5RecoveredFilterClockRateBlend = 0.35; end
if ~isfield(settings, 'deepShadowDs5RecoveredFilterClockMeasMaxPredDiffM'), settings.deepShadowDs5RecoveredFilterClockMeasMaxPredDiffM = 350.0; end
if ~isfield(settings, 'deepShadowDs5RecoveredFilterClockHardGateEnable'), settings.deepShadowDs5RecoveredFilterClockHardGateEnable = 0; end
if ~isfield(settings, 'deepShadowDs5RecoveredFilterClockResetDiffMaxM'), settings.deepShadowDs5RecoveredFilterClockResetDiffMaxM = 1500.0; end
if ~isfield(settings, 'deepShadowDs5RecoveredFilterResetOnContractMeas'), settings.deepShadowDs5RecoveredFilterResetOnContractMeas = 1; end
if ~isfield(settings, 'deepShadowDs5RecoveredFilterResetMeasPredDiffM'), settings.deepShadowDs5RecoveredFilterResetMeasPredDiffM = 1500.0; end
if ~isfield(settings, 'deepShadowDs5RecoveredFilterAbsAnchorGateRequire'), settings.deepShadowDs5RecoveredFilterAbsAnchorGateRequire = 1; end
if ~isfield(settings, 'deepShadowDs5RecoveredFilterAuthorityRequireAbsAnchor'), settings.deepShadowDs5RecoveredFilterAuthorityRequireAbsAnchor = 1; end
if ~isfield(settings, 'deepShadowDs5RecoveredFilterPropagateWithInsVel'), settings.deepShadowDs5RecoveredFilterPropagateWithInsVel = 1; end
if ~isfield(settings, 'deepShadowDs5RecoveredFilterDopplerVelEnable'), settings.deepShadowDs5RecoveredFilterDopplerVelEnable = 1; end
if ~isfield(settings, 'deepShadowDs5RecoveredFilterDopplerVelMinSat'), settings.deepShadowDs5RecoveredFilterDopplerVelMinSat = 4; end
if ~isfield(settings, 'deepShadowDs5RecoveredFilterDopplerVelMaxResidMps'), settings.deepShadowDs5RecoveredFilterDopplerVelMaxResidMps = 85.0; end
if ~isfield(settings, 'deepShadowDs5RecoveredFilterDopplerVelBlend'), settings.deepShadowDs5RecoveredFilterDopplerVelBlend = 0.35; end
if ~isfield(settings, 'deepShadowDs5RecoveredFilterDopplerVelRobustMaxDropSat'), settings.deepShadowDs5RecoveredFilterDopplerVelRobustMaxDropSat = 2; end
if ~isfield(settings, 'deepShadowDs5RecoveredFilterPropCapEnable'), settings.deepShadowDs5RecoveredFilterPropCapEnable = 1; end
if ~isfield(settings, 'deepShadowDs5RecoveredFilterPropMaxStepM'), settings.deepShadowDs5RecoveredFilterPropMaxStepM = 120.0; end
if ~isfield(settings, 'deepShadowDs5RecoveredFilterPropMaxTotalM'), settings.deepShadowDs5RecoveredFilterPropMaxTotalM = 600.0; end
if ~isfield(settings, 'deepShadowDs5RecoveredFilterPredClampEnable'), settings.deepShadowDs5RecoveredFilterPredClampEnable = 1; end
if ~isfield(settings, 'deepShadowDs5RecoveredFilterPredMaxClockStepM'), settings.deepShadowDs5RecoveredFilterPredMaxClockStepM = 600.0; end
if ~isfield(settings, 'deepShadowDs5RecoveredFilterAuthorityMaxCoastEpochs'), settings.deepShadowDs5RecoveredFilterAuthorityMaxCoastEpochs = 12; end
if ~isfield(settings, 'deepShadowDs5RecoveredFilterAuthorityBaselineDiffMaxM'), settings.deepShadowDs5RecoveredFilterAuthorityBaselineDiffMaxM = 700.0; end
if ~isfield(settings, 'deepShadowDs5RecoveredFilterAuthorityUseBaselineDiffHardCap'), settings.deepShadowDs5RecoveredFilterAuthorityUseBaselineDiffHardCap = 0; end
if ~isfield(settings, 'deepShadowDs5RecoveredFilterAuthorityMeasPredDiffMaxM'), settings.deepShadowDs5RecoveredFilterAuthorityMeasPredDiffMaxM = 500.0; end
if ~isfield(settings, 'deepShadowDs5RecoveredFilterAuthorityAbsAnchorMaxM'), settings.deepShadowDs5RecoveredFilterAuthorityAbsAnchorMaxM = 1800.0; end
if ~isfield(settings, 'deepShadowDs5RecoveredFilterAbsAnchorEnable'), settings.deepShadowDs5RecoveredFilterAbsAnchorEnable = 1; end
if ~isfield(settings, 'deepShadowDs5RecoveredFilterAbsAnchorFreezeSec'), settings.deepShadowDs5RecoveredFilterAbsAnchorFreezeSec = settings.deepShadowDs5RefObsRecoveryStartSec; end
if ~isfield(settings, 'deepShadowDs5RecoveredFilterAbsAnchorUseInsVel'), settings.deepShadowDs5RecoveredFilterAbsAnchorUseInsVel = 1; end
if ~isfield(settings, 'deepShadowDs5RecoveredFilterAbsAnchorSoftMaxDiffM'), settings.deepShadowDs5RecoveredFilterAbsAnchorSoftMaxDiffM = 900.0; end
if ~isfield(settings, 'deepShadowDs5RecoveredFilterAbsAnchorMaxDiffM'), settings.deepShadowDs5RecoveredFilterAbsAnchorMaxDiffM = 1800.0; end
if ~isfield(settings, 'deepShadowDs5RecoveredFilterAbsAnchorBlend'), settings.deepShadowDs5RecoveredFilterAbsAnchorBlend = 0.02; end
if ~isfield(settings, 'deepShadowDs5RecoveredFilterAbsAnchorBlendHigh'), settings.deepShadowDs5RecoveredFilterAbsAnchorBlendHigh = 0.12; end
if ~isfield(settings, 'deepShadowDs5RecoveredFilterAbsAnchorAlphaMinScale'), settings.deepShadowDs5RecoveredFilterAbsAnchorAlphaMinScale = 0.15; end
if ~isfield(settings, 'deepShadowDs5RecoveredFilterAbsAnchorDopplerVelBlend'), settings.deepShadowDs5RecoveredFilterAbsAnchorDopplerVelBlend = 0.25; end
if ~isfield(settings, 'deepShadowDs5RefObsPositionAbsAnchorGateEnable'), settings.deepShadowDs5RefObsPositionAbsAnchorGateEnable = 1; end
if ~isfield(settings, 'deepShadowDs5RefObsPositionAbsAnchorSoftMaxDiffM'), settings.deepShadowDs5RefObsPositionAbsAnchorSoftMaxDiffM = settings.deepShadowDs5RecoveredFilterAbsAnchorSoftMaxDiffM; end
if ~isfield(settings, 'deepShadowDs5RefObsPositionAbsAnchorMaxDiffM'), settings.deepShadowDs5RefObsPositionAbsAnchorMaxDiffM = settings.deepShadowDs5RecoveredFilterAbsAnchorMaxDiffM; end
if ~isfield(settings, 'deepShadowDs5RefObsPositionAbsAnchorUseHardGate'), settings.deepShadowDs5RefObsPositionAbsAnchorUseHardGate = 0; end
if ~isfield(settings, 'deepShadowDs5RefObsPositionAbsAnchorScoreWeightM'), settings.deepShadowDs5RefObsPositionAbsAnchorScoreWeightM = 140.0; end
if ~isfield(settings, 'deepShadowDs5RefObsPositionAbsAnchorPriorEnable'), settings.deepShadowDs5RefObsPositionAbsAnchorPriorEnable = 1; end
if ~isfield(settings, 'deepShadowDs5RefObsPositionAbsAnchorPriorSigmaM'), settings.deepShadowDs5RefObsPositionAbsAnchorPriorSigmaM = 900.0; end
if ~isfield(settings, 'deepShadowDs5RefObsPositionAbsAnchorPriorSoftSigmaM'), settings.deepShadowDs5RefObsPositionAbsAnchorPriorSoftSigmaM = 1800.0; end
if ~isfield(settings, 'deepShadowDs5RefObsPositionClockSoftMaxM'), settings.deepShadowDs5RefObsPositionClockSoftMaxM = 1200.0; end
if ~isfield(settings, 'deepShadowDs5RefObsPositionClockHardMaxM'), settings.deepShadowDs5RefObsPositionClockHardMaxM = 3500.0; end
if ~isfield(settings, 'deepShadowDs5RefObsPositionClockScoreWeightM'), settings.deepShadowDs5RefObsPositionClockScoreWeightM = 100.0; end
if ~isfield(settings, 'deepShadowDs5RefObsPositionClockPriorEnable'), settings.deepShadowDs5RefObsPositionClockPriorEnable = 1; end
if ~isfield(settings, 'deepShadowDs5RefObsPositionClockPriorSigmaM'), settings.deepShadowDs5RefObsPositionClockPriorSigmaM = 1200.0; end
if ~isfield(settings, 'deepShadowDs5RefObsPositionCommonClockEnable'), settings.deepShadowDs5RefObsPositionCommonClockEnable = 1; end
if ~isfield(settings, 'deepShadowDs5RefObsPositionCommonClockMinSat'), settings.deepShadowDs5RefObsPositionCommonClockMinSat = 3; end
if ~isfield(settings, 'deepShadowDs5RefObsPositionCommonClockGateFloorM'), settings.deepShadowDs5RefObsPositionCommonClockGateFloorM = 250.0; end
if ~isfield(settings, 'deepShadowDs5RefObsPositionCommonClockMadScale'), settings.deepShadowDs5RefObsPositionCommonClockMadScale = 4.0; end
if ~isfield(settings, 'deepShadowDs5RefObsPositionCommonClockPriorBlend'), settings.deepShadowDs5RefObsPositionCommonClockPriorBlend = 0.15; end
if ~isfield(settings, 'deepShadowDs5RefObsPositionCommonClockPriorMaxPullM'), settings.deepShadowDs5RefObsPositionCommonClockPriorMaxPullM = 1500.0; end
if ~isfield(settings, 'deepShadowDs5RefObsPositionCommonClockBadScoreThresh'), settings.deepShadowDs5RefObsPositionCommonClockBadScoreThresh = 2.0; end
if ~isfield(settings, 'deepShadowDs5RefObsPositionCommonClockRebaseEnable'), settings.deepShadowDs5RefObsPositionCommonClockRebaseEnable = 1; end
if ~isfield(settings, 'deepShadowDs5RefObsPositionCommonClockRebasePeriodM'), settings.deepShadowDs5RefObsPositionCommonClockRebasePeriodM = settings.c * 1e-3; end
if ~isfield(settings, 'deepShadowDs5ObsContractIndependentCodeErrMaxChips'), settings.deepShadowDs5ObsContractIndependentCodeErrMaxChips = settings.deepShadowDs5RefObsTrackingCodeP95MaxChips; end
if ~isfield(settings, 'deepShadowDs5ObsContractIndependentFreqErrMaxHz'), settings.deepShadowDs5ObsContractIndependentFreqErrMaxHz = settings.deepShadowDs5RefObsTrackingFreqP95MaxHz; end
if ~isfield(settings, 'deepShadowDs5RefObsPositionRobustHardDropPrns'), settings.deepShadowDs5RefObsPositionRobustHardDropPrns = [21 27]; end
if ~isfield(settings, 'deepShadowDs5RefObsPositionRobustHardDropMinSatNum'), settings.deepShadowDs5RefObsPositionRobustHardDropMinSatNum = 5; end
if ~isfield(settings, 'deepShadowDs5RefObsPositionRobustHardDropEnable'), settings.deepShadowDs5RefObsPositionRobustHardDropEnable = 1; end
if ~isfield(settings, 'deepShadowDs5RefObsPositionRobustTargetKeepPenaltyM'), settings.deepShadowDs5RefObsPositionRobustTargetKeepPenaltyM = 120.0; end
if ~isfield(settings, 'deepShadowDs5RefObsPositionRobustSubsetEnable'), settings.deepShadowDs5RefObsPositionRobustSubsetEnable = 1; end
if ~isfield(settings, 'deepShadowDs5RefObsPositionRobustMaxDropSat'), settings.deepShadowDs5RefObsPositionRobustMaxDropSat = 2; end
if ~isfield(settings, 'deepShadowDs5RefObsPositionRobustDropPenaltyM'), settings.deepShadowDs5RefObsPositionRobustDropPenaltyM = 20.0; end
if ~isfield(settings, 'deepShadowDs5RefObsPositionRobustBadPrns'), settings.deepShadowDs5RefObsPositionRobustBadPrns = [27 9]; end
if ~isfield(settings, 'deepShadowDs5RefObsPositionRobustBadKeepPenaltyM'), settings.deepShadowDs5RefObsPositionRobustBadKeepPenaltyM = 80.0; end
if ~isfield(settings, 'deepShadowDs5RefObsPositionRobustTargetKeepPenaltyM'), settings.deepShadowDs5RefObsPositionRobustTargetKeepPenaltyM = 18.0; end
if ~isfield(settings, 'deepShadowDs5RefObsPositionRobustBadScoreDecay'), settings.deepShadowDs5RefObsPositionRobustBadScoreDecay = 0.92; end
if ~isfield(settings, 'deepShadowDs5RefObsPositionRobustBadScoreDropInc'), settings.deepShadowDs5RefObsPositionRobustBadScoreDropInc = 1.0; end
if ~isfield(settings, 'deepShadowDs5RefObsPositionRobustBadScoreKeepDec'), settings.deepShadowDs5RefObsPositionRobustBadScoreKeepDec = 0.45; end
if ~isfield(settings, 'deepShadowDs5RefObsPositionRobustBadScoreTargetExtra'), settings.deepShadowDs5RefObsPositionRobustBadScoreTargetExtra = 1.25; end
if ~isfield(settings, 'deepShadowDs5RefObsPositionRobustBadScoreResidualInc'), settings.deepShadowDs5RefObsPositionRobustBadScoreResidualInc = 0.35; end
if ~isfield(settings, 'deepShadowDs5RefObsPositionRobustBadScoreResidualThreshM'), settings.deepShadowDs5RefObsPositionRobustBadScoreResidualThreshM = 120.0; end
if ~isfield(settings, 'deepShadowDs5RefObsPositionRobustBadScoreMax'), settings.deepShadowDs5RefObsPositionRobustBadScoreMax = 12.0; end
if ~isfield(settings, 'deepShadowDs5DetrendedGateEnable'), settings.deepShadowDs5DetrendedGateEnable = 0; end
if ~isfield(settings, 'deepShadowDs5DetrendedGateStartSec'), settings.deepShadowDs5DetrendedGateStartSec = 118.0; end
if ~isfield(settings, 'deepShadowDs5DetrendedGateMinSat'), settings.deepShadowDs5DetrendedGateMinSat = 4; end
if ~isfield(settings, 'deepShadowDs5DetrendedGateFloorM'), settings.deepShadowDs5DetrendedGateFloorM = 180.0; end
if ~isfield(settings, 'deepShadowDs5DetrendedGateMadScale'), settings.deepShadowDs5DetrendedGateMadScale = 4.0; end
if ~isfield(settings, 'deepShadowDs5DetrendedGateCeilM'), settings.deepShadowDs5DetrendedGateCeilM = 1200.0; end
if ~isfield(settings, 'deepShadowDs5DetrendedGateProtectPrns'), settings.deepShadowDs5DetrendedGateProtectPrns = [27 3]; end
if ~isfield(settings, 'deepShadowBranchDs5DetrendedScoreWeight'), settings.deepShadowBranchDs5DetrendedScoreWeight = 1.5; end
if ~isfield(settings, 'deepShadowBranchDs5DetrendedP95Weight'), settings.deepShadowBranchDs5DetrendedP95Weight = 0.5; end
if ~isfield(settings, 'deepShadowBranchDs5DetrendedReadyP95MaxM'), settings.deepShadowBranchDs5DetrendedReadyP95MaxM = 250.0; end
if ~isfield(settings, 'deepShadowBranchDs5AbsoluteReadyP95MaxM'), settings.deepShadowBranchDs5AbsoluteReadyP95MaxM = 250.0; end
if ~isfield(settings, 'deepShadowDs5ClosedLoopStrongBranchAbsP95MaxM'), settings.deepShadowDs5ClosedLoopStrongBranchAbsP95MaxM = 250.0; end
if ~isfield(settings, 'deepShadowDs5ClosedLoopStrongBranchPostfitMaxM'), settings.deepShadowDs5ClosedLoopStrongBranchPostfitMaxM = 120.0; end
if ~isfield(settings, 'deepShadowDs5ClosedLoopStrongBranchDetrendedP95MaxM'), settings.deepShadowDs5ClosedLoopStrongBranchDetrendedP95MaxM = 250.0; end
if ~isfield(settings, 'deepShadowDs5BranchTakeoverAllow4Sat'), settings.deepShadowDs5BranchTakeoverAllow4Sat = 1; end
if ~isfield(settings, 'deepShadowDs5BranchTakeoverDetrendedP95MaxM'), settings.deepShadowDs5BranchTakeoverDetrendedP95MaxM = 180.0; end
if ~isfield(settings, 'deepShadowDs5BranchContinueAllow4Sat'), settings.deepShadowDs5BranchContinueAllow4Sat = 1; end
if ~isfield(settings, 'deepShadowDs5BranchContinueDetrendedP95MaxM'), settings.deepShadowDs5BranchContinueDetrendedP95MaxM = 220.0; end
if ~isfield(settings, 'deepShadowBranchDopplerGateEnable'), settings.deepShadowBranchDopplerGateEnable = 1; end
if ~isfield(settings, 'deepShadowBranchDopplerGateMps'), settings.deepShadowBranchDopplerGateMps = 60.0; end
if ~isfield(settings, 'deepShadowBranchDopplerGateMinKeep'), settings.deepShadowBranchDopplerGateMinKeep = 4; end
if ~isfield(settings, 'deepShadowRawAmbiguityEnable'), settings.deepShadowRawAmbiguityEnable = 0; end
if ~isfield(settings, 'deepShadowRawAmbiguityMaxChips'), settings.deepShadowRawAmbiguityMaxChips = 80.0; end
if ~isfield(settings, 'deepShadowRawAmbiguityStepMaxChips'), settings.deepShadowRawAmbiguityStepMaxChips = 8.0; end
if ~isfield(settings, 'deepShadowRawAmbiguityMinSat'), settings.deepShadowRawAmbiguityMinSat = 4; end
if ~isfield(settings, 'deepShadowBranchSearchEnable'), settings.deepShadowBranchSearchEnable = 0; end
if ~isfield(settings, 'deepShadowBranchUseNavQ'), settings.deepShadowBranchUseNavQ = 1; end
if ~isfield(settings, 'deepShadowBranchMinSat'), settings.deepShadowBranchMinSat = 5; end
if ~isfield(settings, 'deepShadowBranchMaxChips'), settings.deepShadowBranchMaxChips = 120.0; end
if ~isfield(settings, 'deepShadowBranchRobustSubsetEnable'), settings.deepShadowBranchRobustSubsetEnable = 1; end
if ~isfield(settings, 'deepShadowBranchRobustSubsetMinSat'), settings.deepShadowBranchRobustSubsetMinSat = 4; end
if ~isfield(settings, 'deepShadowBranchRobustSubsetMaxSat'), settings.deepShadowBranchRobustSubsetMaxSat = 5; end
if ~isfield(settings, 'deepShadowBranchRobustSubsetMaxCand'), settings.deepShadowBranchRobustSubsetMaxCand = 128; end
if ~isfield(settings, 'deepShadowBranchRobustSubsetDropPenaltyM'), settings.deepShadowBranchRobustSubsetDropPenaltyM = 15.0; end
if ~isfield(settings, 'deepShadowBranchRobustSubsetFourSatPenaltyM'), settings.deepShadowBranchRobustSubsetFourSatPenaltyM = 300.0; end
if ~isfield(settings, 'deepShadowBranchRobustDopplerWeight'), settings.deepShadowBranchRobustDopplerWeight = 0.15; end
if ~isfield(settings, 'deepShadowBranchAmbigContinuityEnable'), settings.deepShadowBranchAmbigContinuityEnable = 1; end
if ~isfield(settings, 'deepShadowBranchAmbigContinuityMaxAgeEpochs'), settings.deepShadowBranchAmbigContinuityMaxAgeEpochs = 80; end
if ~isfield(settings, 'deepShadowBranchAmbigContinuityStepMaxChips'), settings.deepShadowBranchAmbigContinuityStepMaxChips = 2.0; end
if ~isfield(settings, 'deepShadowBranchAmbigContinuityTolChips'), settings.deepShadowBranchAmbigContinuityTolChips = 0.75; end
if ~isfield(settings, 'deepShadowBranchAmbigContinuityPenaltyM'), settings.deepShadowBranchAmbigContinuityPenaltyM = 120.0; end
if ~isfield(settings, 'deepShadowBranchAmbigContinuityJumpPenaltyM'), settings.deepShadowBranchAmbigContinuityJumpPenaltyM = 250.0; end
if ~isfield(settings, 'deepShadowBranchPriorRmsMaxM'), settings.deepShadowBranchPriorRmsMaxM = 500.0; end
if ~isfield(settings, 'deepShadowBranchPostfitRmsMaxM'), settings.deepShadowBranchPostfitRmsMaxM = 300.0; end
if ~isfield(settings, 'deepShadowBranchPdopMax'), settings.deepShadowBranchPdopMax = 20.0; end
if ~isfield(settings, 'deepShadowBranchPosJumpMaxM'), settings.deepShadowBranchPosJumpMaxM = 3000.0; end
if ~isfield(settings, 'deepShadowDs5BaselineTrustedRefEnable'), settings.deepShadowDs5BaselineTrustedRefEnable = 0; end
if ~isfield(settings, 'deepShadowDs5BaselineTrustedRefUseVelocity'), settings.deepShadowDs5BaselineTrustedRefUseVelocity = 1; end
if ~isfield(settings, 'deepShadowDs5BaselineTrustedRefMaxSec'), settings.deepShadowDs5BaselineTrustedRefMaxSec = settings.deepShadowDs5RefObsRecoveryStartSec - settings.navSolPeriod / 1000; end
if ~isfield(settings, 'deepShadowDs5BranchAbsoluteConsistencyEnable'), settings.deepShadowDs5BranchAbsoluteConsistencyEnable = 0; end
if ~isfield(settings, 'deepShadowDs5BranchAbsoluteConsistencyMaxDiffM'), settings.deepShadowDs5BranchAbsoluteConsistencyMaxDiffM = 1200.0; end
if ~isfield(settings, 'deepShadowDs5BranchLocalCorrectionMaxM'), settings.deepShadowDs5BranchLocalCorrectionMaxM = 1200.0; end
if ~isfield(settings, 'deepShadowDs5BranchLocalStepMaxM'), settings.deepShadowDs5BranchLocalStepMaxM = 250.0; end
if ~isfield(settings, 'deepShadowDs5BranchLocalIterMax'), settings.deepShadowDs5BranchLocalIterMax = 3; end
if ~isfield(settings, 'deepShadowDs5BranchLocalCorrPenaltyWeight'), settings.deepShadowDs5BranchLocalCorrPenaltyWeight = 0.10; end
if ~isfield(settings, 'deepShadowDs5BranchObsGateEnable'), settings.deepShadowDs5BranchObsGateEnable = 0; end
if ~isfield(settings, 'deepShadowDs5BranchObsGateMinSat'), settings.deepShadowDs5BranchObsGateMinSat = 4; end
if ~isfield(settings, 'deepShadowDs5BranchObsGateFloorM'), settings.deepShadowDs5BranchObsGateFloorM = 120.0; end
if ~isfield(settings, 'deepShadowDs5BranchObsGateMadScale'), settings.deepShadowDs5BranchObsGateMadScale = 3.0; end
if ~isfield(settings, 'deepShadowDs5BranchObsGateCeilM'), settings.deepShadowDs5BranchObsGateCeilM = 1500.0; end
if ~isfield(settings, 'deepShadowTrustedAnchorEnable'), settings.deepShadowTrustedAnchorEnable = 0; end
if ~isfield(settings, 'deepShadowTrustedAnchorUseTruthTrj'), settings.deepShadowTrustedAnchorUseTruthTrj = 0; end
if ~isfield(settings, 'deepShadowTrustedAnchorTimeSec'), settings.deepShadowTrustedAnchorTimeSec = 90.0; end
if ~isfield(settings, 'deepShadowTrustedAnchorUseVelocity'), settings.deepShadowTrustedAnchorUseVelocity = 1; end
if ~isfield(settings, 'deepShadowTrustedAnchorAllowCurrentFallback'), settings.deepShadowTrustedAnchorAllowCurrentFallback = 1; end
if ~isfield(settings, 'deepShadowBranchTakeoverConfirmEpochs'), settings.deepShadowBranchTakeoverConfirmEpochs = 3; end
if ~isfield(settings, 'deepShadowBranchTakeoverPostfitRmsMaxM'), settings.deepShadowBranchTakeoverPostfitRmsMaxM = 100.0; end
if ~isfield(settings, 'deepShadowBranchTakeoverPdopMax'), settings.deepShadowBranchTakeoverPdopMax = 10.0; end
if ~isfield(settings, 'deepShadowBranchTakeoverMinSat'), settings.deepShadowBranchTakeoverMinSat = 5; end
if ~isfield(settings, 'deepShadowBranchTakeoverRequireTrustedPrior'), settings.deepShadowBranchTakeoverRequireTrustedPrior = 1; end
if ~isfield(settings, 'deepShadowBranchResetEnable'), settings.deepShadowBranchResetEnable = 0; end
if ~isfield(settings, 'deepShadowBranchResetOnce'), settings.deepShadowBranchResetOnce = 1; end
if ~isfield(settings, 'deepShadowBranchResetMaxJumpM'), settings.deepShadowBranchResetMaxJumpM = 50000.0; end
if ~isfield(settings, 'deepShadowBranchResetVelBlend'), settings.deepShadowBranchResetVelBlend = 0.0; end
if ~isfield(settings, 'deepShadowBranchResetClock'), settings.deepShadowBranchResetClock = 1; end
if ~isfield(settings, 'deepShadowBranchInjectEnable'), settings.deepShadowBranchInjectEnable = 0; end
if ~isfield(settings, 'deepShadowBranchInjectRScale'), settings.deepShadowBranchInjectRScale = 25.0; end
if ~isfield(settings, 'deepShadowBranchInjectMinSat'), settings.deepShadowBranchInjectMinSat = 5; end
if ~isfield(settings, 'deepShadowBranchInjectResidualGateM'), settings.deepShadowBranchInjectResidualGateM = 300.0; end
if ~isfield(settings, 'deepShadowRecoveryModeEnable'), settings.deepShadowRecoveryModeEnable = 0; end
if ~isfield(settings, 'deepShadowRecoveryHoldEpochs'), settings.deepShadowRecoveryHoldEpochs = 80; end
if ~isfield(settings, 'deepShadowRecoverySuppressMainGnss'), settings.deepShadowRecoverySuppressMainGnss = 1; end
if ~isfield(settings, 'deepShadowRecoveryContinueMinSat'), settings.deepShadowRecoveryContinueMinSat = settings.deepShadowBranchInjectMinSat; end
if ~isfield(settings, 'deepShadowRecoveryContinuePostfitRmsMaxM'), settings.deepShadowRecoveryContinuePostfitRmsMaxM = 120.0; end
if ~isfield(settings, 'deepShadowRecoveryContinuePdopMax'), settings.deepShadowRecoveryContinuePdopMax = 12.0; end
if ~isfield(settings, 'deepShadowRecoveryRequireTrustedPrior'), settings.deepShadowRecoveryRequireTrustedPrior = 1; end
if ~isfield(settings, 'deepShadowRecoveryReanchorEnable'), settings.deepShadowRecoveryReanchorEnable = 1; end
if ~isfield(settings, 'deepShadowRecoveryReanchorMinJumpM'), settings.deepShadowRecoveryReanchorMinJumpM = 500.0; end
if ~isfield(settings, 'deepShadowRecoveryReanchorCooldownEpochs'), settings.deepShadowRecoveryReanchorCooldownEpochs = 3; end
if ~isfield(settings, 'deepShadowRecoveryReanchorMaxCount'), settings.deepShadowRecoveryReanchorMaxCount = 20; end
if ~isfield(settings, 'deepShadowClosedLoopUseBranchOutput'), settings.deepShadowClosedLoopUseBranchOutput = 1; end
if ~isfield(settings, 'deepShadowClosedLoopCoastEpochs'), settings.deepShadowClosedLoopCoastEpochs = 20; end
if ~isfield(settings, 'deepShadowClosedLoopUseVelocityCoast'), settings.deepShadowClosedLoopUseVelocityCoast = 1; end
if ~isfield(settings, 'deepShadowClosedLoopAllowNavFallbackInRecovery'), settings.deepShadowClosedLoopAllowNavFallbackInRecovery = 1; end
if ~isfield(settings, 'deepShadowClosedLoopUseTrustedFallback'), settings.deepShadowClosedLoopUseTrustedFallback = 1; end
if ~isfield(settings, 'deepShadowClosedLoopTrustedBlendEnable'), settings.deepShadowClosedLoopTrustedBlendEnable = 0; end
if ~isfield(settings, 'deepShadowClosedLoopTrustedBlendBranchAlpha'), settings.deepShadowClosedLoopTrustedBlendBranchAlpha = 0.0; end
if ~isfield(settings, 'deepShadowClosedLoopTrustedBlendCoastAlpha'), settings.deepShadowClosedLoopTrustedBlendCoastAlpha = 0.0; end
if ~isfield(settings, 'deepShadowClosedLoopSafeBranchGateEnable'), settings.deepShadowClosedLoopSafeBranchGateEnable = 1; end
if ~isfield(settings, 'deepShadowClosedLoopSafeBranchMaxDiffM'), settings.deepShadowClosedLoopSafeBranchMaxDiffM = 450.0; end
if ~isfield(settings, 'deepShadowClosedLoopSafeBranchPostfitMaxM'), settings.deepShadowClosedLoopSafeBranchPostfitMaxM = 80.0; end
if ~isfield(settings, 'deepShadowDs5RefObsClosedLoopUseBaselineHold'), settings.deepShadowDs5RefObsClosedLoopUseBaselineHold = 1; end
if ~isfield(settings, 'deepShadowDs5RefObsClosedLoopPreferBaselineHold'), settings.deepShadowDs5RefObsClosedLoopPreferBaselineHold = 1; end
if ~isfield(settings, 'deepShadowDs5RefObsPositionEnable'), settings.deepShadowDs5RefObsPositionEnable = 1; end
if ~isfield(settings, 'deepShadowDs5RefObsPositionMinSat'), settings.deepShadowDs5RefObsPositionMinSat = 4; end
if ~isfield(settings, 'deepShadowDs5RefObsPositionRawAnchorLiftEnable'), settings.deepShadowDs5RefObsPositionRawAnchorLiftEnable = 1; end
if ~isfield(settings, 'deepShadowDs5RefObsPositionRawAnchorLiftMaxSpreadP95M'), settings.deepShadowDs5RefObsPositionRawAnchorLiftMaxSpreadP95M = 800.0; end
if ~isfield(settings, 'deepShadowDs5RefObsPositionRawAnchorLiftUseCodeCorr'), settings.deepShadowDs5RefObsPositionRawAnchorLiftUseCodeCorr = 1; end
if ~isfield(settings, 'deepShadowDs5RefObsPositionPostfitMaxM'), settings.deepShadowDs5RefObsPositionPostfitMaxM = 220.0; end
if ~isfield(settings, 'deepShadowDs5RefObsPositionPdopMax'), settings.deepShadowDs5RefObsPositionPdopMax = 20.0; end
if ~isfield(settings, 'deepShadowDs5RefObsPositionCorrMaxM'), settings.deepShadowDs5RefObsPositionCorrMaxM = 260.0; end
if ~isfield(settings, 'deepShadowDs5RefObsPositionAnchorDiffMaxM'), settings.deepShadowDs5RefObsPositionAnchorDiffMaxM = settings.deepShadowDs5RefObsPositionCorrMaxM; end
if ~isfield(settings, 'deepShadowDs5RefObsPositionJumpMaxM'), settings.deepShadowDs5RefObsPositionJumpMaxM = 220.0; end
if ~isfield(settings, 'deepShadowDs5RefObsPositionJumpMaxAgeEpochs'), settings.deepShadowDs5RefObsPositionJumpMaxAgeEpochs = 4; end
if ~isfield(settings, 'deepShadowDs5RefObsPositionSpeedGateEnable'), settings.deepShadowDs5RefObsPositionSpeedGateEnable = 1; end
if ~isfield(settings, 'deepShadowDs5RefObsPositionSpeedMaxMps'), settings.deepShadowDs5RefObsPositionSpeedMaxMps = 140.0; end
if ~isfield(settings, 'deepShadowDs5RefObsPositionAccelMaxMps2'), settings.deepShadowDs5RefObsPositionAccelMaxMps2 = 180.0; end
if ~isfield(settings, 'deepShadowDs5RefObsPositionDynGateMaxAgeEpochs'), settings.deepShadowDs5RefObsPositionDynGateMaxAgeEpochs = 4; end
if ~isfield(settings, 'deepShadowDs5SoftRecoveryEnable'), settings.deepShadowDs5SoftRecoveryEnable = 0; end
if ~isfield(settings, 'deepShadowDs5SoftRecoveryStartSec'), settings.deepShadowDs5SoftRecoveryStartSec = 92.0; end
if ~isfield(settings, 'deepShadowDs5SoftRecoveryRequireBranchSane'), settings.deepShadowDs5SoftRecoveryRequireBranchSane = 1; end
if ~isfield(settings, 'deepShadowDs5ClosedLiftPreferEnable'), settings.deepShadowDs5ClosedLiftPreferEnable = 0; end
if ~isfield(settings, 'deepShadowDs5ClosedLiftPreferStartSec'), settings.deepShadowDs5ClosedLiftPreferStartSec = 118.0; end
if ~isfield(settings, 'deepShadowDs5RawLiftRejectDevM'), settings.deepShadowDs5RawLiftRejectDevM = 12000.0; end
if ~isfield(settings, 'deepShadowDs5ClosedLiftCorrMinBaseSat'), settings.deepShadowDs5ClosedLiftCorrMinBaseSat = 2; end
if ~isfield(settings, 'deepShadowDs5ClosedLiftCorrMinQuality'), settings.deepShadowDs5ClosedLiftCorrMinQuality = 1; end
if ~isfield(settings, 'deepShadowDs5ClosedLiftCorrResidualGateM'), settings.deepShadowDs5ClosedLiftCorrResidualGateM = 450.0; end
if ~isfield(settings, 'deepShadowDs5ClosedLiftCorrStrictPrns'), settings.deepShadowDs5ClosedLiftCorrStrictPrns = [27 3]; end
if ~isfield(settings, 'deepShadowDs5ClosedLiftCorrStrictMinQuality'), settings.deepShadowDs5ClosedLiftCorrStrictMinQuality = 2; end
if ~isfield(settings, 'deepShadowDs5ClosedLiftCorrStrictResidualGateM'), settings.deepShadowDs5ClosedLiftCorrStrictResidualGateM = 180.0; end
if ~isfield(settings, 'deepShadowDs5ClosedLiftCorrApplyMaxM'), settings.deepShadowDs5ClosedLiftCorrApplyMaxM = 3000.0; end
if ~isfield(settings, 'deepShadowDs5SoftClampEnable'), settings.deepShadowDs5SoftClampEnable = 0; end
if ~isfield(settings, 'deepShadowDs5SoftClampStartSec'), settings.deepShadowDs5SoftClampStartSec = 150.0; end
if ~isfield(settings, 'deepShadowDs5SoftClampAlpha'), settings.deepShadowDs5SoftClampAlpha = 0.35; end
if ~isfield(settings, 'deepShadowDs5SoftClampVelBlend'), settings.deepShadowDs5SoftClampVelBlend = 0.25; end
if ~isfield(settings, 'deepShadowDs5KfSourceGateEnable'), settings.deepShadowDs5KfSourceGateEnable = 0; end
if ~isfield(settings, 'deepShadowDs5KfSourceGateStartSec'), settings.deepShadowDs5KfSourceGateStartSec = 118.0; end
if ~isfield(settings, 'deepShadowDs5KfRequireCommonDrag'), settings.deepShadowDs5KfRequireCommonDrag = 1; end
if ~isfield(settings, 'deepShadowDs5KfTrustedMinSat'), settings.deepShadowDs5KfTrustedMinSat = 3; end
if ~isfield(settings, 'deepShadowDs5KfMode2MinSat'), settings.deepShadowDs5KfMode2MinSat = 2; end
if ~isfield(settings, 'deepShadowDs5KfAllowMode3Prns'), settings.deepShadowDs5KfAllowMode3Prns = [27 3]; end
if ~isfield(settings, 'deepShadowDs5KfAllowMode3MaxDetM'), settings.deepShadowDs5KfAllowMode3MaxDetM = 120.0; end
if ~isfield(settings, 'deepShadowDs5KfAllowMode3OnlyWhenBranchSane'), settings.deepShadowDs5KfAllowMode3OnlyWhenBranchSane = 1; end
if ~isfield(settings, 'deepShadowDs5AuthorityEnable'), settings.deepShadowDs5AuthorityEnable = 0; end
if ~isfield(settings, 'deepShadowDs5AuthorityStartSec'), settings.deepShadowDs5AuthorityStartSec = 92.0; end
if ~isfield(settings, 'deepShadowDs5AuthorityConfirmEpochs'), settings.deepShadowDs5AuthorityConfirmEpochs = 3; end
if ~isfield(settings, 'deepShadowDs5AuthorityHoldEpochs'), settings.deepShadowDs5AuthorityHoldEpochs = 6; end
if ~isfield(settings, 'deepShadowDs5AuthorityRequireBranchSane'), settings.deepShadowDs5AuthorityRequireBranchSane = 1; end
if ~isfield(settings, 'deepShadowDs5AuthorityRequireClosedLoopRef'), settings.deepShadowDs5AuthorityRequireClosedLoopRef = 1; end
if ~isfield(settings, 'deepShadowDs5AuthorityRefPosMaxDiffM'), settings.deepShadowDs5AuthorityRefPosMaxDiffM = 1200.0; end
if ~isfield(settings, 'deepShadowDs5AuthorityHardRevokeEnable'), settings.deepShadowDs5AuthorityHardRevokeEnable = 1; end
if ~isfield(settings, 'deepShadowDs5AuthorityHardRevokeRefDiffM'), settings.deepShadowDs5AuthorityHardRevokeRefDiffM = settings.deepShadowDs5AuthorityRefPosMaxDiffM; end
if ~isfield(settings, 'deepShadowDs5AuthorityFastDeauthBadEpochs'), settings.deepShadowDs5AuthorityFastDeauthBadEpochs = 2; end
if ~isfield(settings, 'deepShadowDs5AuthorityTrustedMinSat'), settings.deepShadowDs5AuthorityTrustedMinSat = 3; end
if ~isfield(settings, 'deepShadowDs5AuthorityMode2MinSat'), settings.deepShadowDs5AuthorityMode2MinSat = 2; end
if ~isfield(settings, 'deepShadowDs5AuthorityMode23MinSat'), settings.deepShadowDs5AuthorityMode23MinSat = 3; end
if ~isfield(settings, 'deepShadowDs5ObserveOnlyFollowEnable'), settings.deepShadowDs5ObserveOnlyFollowEnable = 1; end
if ~isfield(settings, 'deepShadowDs5FinalOutputEnable'), settings.deepShadowDs5FinalOutputEnable = 0; end
if ~isfield(settings, 'deepShadowDs5FinalOutputHoldEpochs'), settings.deepShadowDs5FinalOutputHoldEpochs = 120; end
if ~isfield(settings, 'deepShadowDs5FinalRecoveryBaselineGateEnable'), settings.deepShadowDs5FinalRecoveryBaselineGateEnable = 1; end
if ~isfield(settings, 'deepShadowDs5FinalRecoveryBaselineMaxDiffM'), settings.deepShadowDs5FinalRecoveryBaselineMaxDiffM = 350.0; end
if ~isfield(settings, 'deepShadowDs5FinalRecoveryImproveGateEnable'), settings.deepShadowDs5FinalRecoveryImproveGateEnable = 1; end
if ~isfield(settings, 'deepShadowDs5FinalRecoveryImproveMinM'), settings.deepShadowDs5FinalRecoveryImproveMinM = 120.0; end
if ~isfield(settings, 'deepShadowClosedLoopSafeBranchPdopMax'), settings.deepShadowClosedLoopSafeBranchPdopMax = 8.0; end
if ~isfield(settings, 'deepShadowClosedLoopSafeBranchMinSat'), settings.deepShadowClosedLoopSafeBranchMinSat = 5; end
if ~isfield(settings, 'deepShadowClosedLoopSafeBranchAlpha'), settings.deepShadowClosedLoopSafeBranchAlpha = 0.25; end
if ~isfield(settings, 'deepShadowCoreServoEnable'), settings.deepShadowCoreServoEnable = 0; end
if ~isfield(settings, 'deepShadowCoreServoGain'), settings.deepShadowCoreServoGain = 0.05; end
if ~isfield(settings, 'deepShadowCoreServoStepMaxChips'), settings.deepShadowCoreServoStepMaxChips = 0.50; end
if ~isfield(settings, 'deepShadowCoreServoTotalMaxChips'), settings.deepShadowCoreServoTotalMaxChips = 40.0; end
if ~isfield(settings, 'deepShadowCoreServoMinSat'), settings.deepShadowCoreServoMinSat = 4; end
if ~isfield(settings, 'deepShadowCoreServoCoreDevMaxM'), settings.deepShadowCoreServoCoreDevMaxM = 10000.0; end
if ~isfield(settings, 'deepShadowCoreServoMinElevationDeg'), settings.deepShadowCoreServoMinElevationDeg = 0.0; end
if ~isfield(settings, 'deepShadowMainSwitchEnable'), settings.deepShadowMainSwitchEnable = 0; end
if ~isfield(settings, 'deepShadowMainSwitchImproveMinM'), settings.deepShadowMainSwitchImproveMinM = 100.0; end
if ~isfield(settings, 'deepShadowMainSwitchConfirmEpochs'), settings.deepShadowMainSwitchConfirmEpochs = 3; end
if ~isfield(settings, 'deepShadowMainSwitchHoldEpochs'), settings.deepShadowMainSwitchHoldEpochs = 6; end
if ~isfield(settings, 'deepShadowMainSwitchWaitProtectDone'), settings.deepShadowMainSwitchWaitProtectDone = 1; end
if ~isfield(settings, 'deepPrInnovationGateEnable'), settings.deepPrInnovationGateEnable = 0; end
if ~isfield(settings, 'deepPrInnovationGateK'), settings.deepPrInnovationGateK = 4.0; end
if ~isfield(settings, 'deepPrInnovationGateMinM'), settings.deepPrInnovationGateMinM = 20.0; end
if ~isfield(settings, 'deepPrInnovationGateMaxM'), settings.deepPrInnovationGateMaxM = 300.0; end
if ~isfield(settings, 'deepPrInnovationGateMinSat'), settings.deepPrInnovationGateMinSat = 4; end
if ~isfield(settings, 'deepPrInnovationGateUseMedianDetrend'), settings.deepPrInnovationGateUseMedianDetrend = 1; end
if ~isfield(settings, 'deepRecoveryHoldEpochs'), settings.deepRecoveryHoldEpochs = 16; end
if ~isfield(settings, 'deepReentryStrictEpochs'), settings.deepReentryStrictEpochs = 30; end
if ~isfield(settings, 'deepReentryTcmScale'), settings.deepReentryTcmScale = 1.50; end
if ~isfield(settings, 'deepReentryTdfScale'), settings.deepReentryTdfScale = 1.50; end
if ~isfield(settings, 'deepReentryTsatScale'), settings.deepReentryTsatScale = 1.50; end
if ~isfield(settings, 'deepReentryTzScale'), settings.deepReentryTzScale = 1.30; end
if ~isfield(settings, 'deepReentryConfirmEpochs')
    settings.deepReentryConfirmEpochs = max(settings.deepConfirmEpochs + 2, 5);
end
if ~isfield(settings, 'deepReentryHitMin')
    settings.deepReentryHitMin = max(settings.deepHitMin + 1, 1);
end
lambdaL1 = settings.c / 1575.42e6;
maxPrnId = max(64, max([trackDeepIn.PRN]));
rhoCorrByPrn = nan(1, maxPrnId);
deepModeState = 0;  % 0 normal, 1 suspect, 2 spoof
trustedAnchorValid = false;
trustedAnchorPos = nan(3,1);
trustedAnchorVel = nan(3,1);
trustedAnchorTimeSec = nan;
trustedAnchorSource = 0;  % 0 none, 1 online pre-spoof main-nav, 2 simulation truth/trj, 3 baseline-fed
branchTakeoverCnt = 0;
shadowBranchAmbigHoldChips = nan(numActChnList, 1);
shadowBranchAmbigHoldAge = inf(numActChnList, 1);
shadowBranchResetDone = false;
shadowBranchResetEpoch = nan;
shadowRecoveryMode = false;
shadowRecoveryRemain = 0;
ds5RecoveryAuthorized = false;
ds5RecoveryAuthorityRemain = 0;
ds5RecoveryAuthorityConfirmCnt = 0;
ds5RecoveryAuthorityBadCnt = 0;
shadowRecoveryReanchorCooldown = 0;
shadowRecoveryReanchorCount = 0;
shadowClosedLoopLastPos = nan(3,1);
shadowClosedLoopLastVel = nan(3,1);
shadowClosedLoopLastEpoch = nan;
shadowClosedLoopCoastAge = inf;
shadowClosedLoopLastSource = 0;
ds5RefObsPosLastAccepted = nan(3,1);
ds5RefObsPosPrevAccepted = nan(3,1);
ds5RefObsPosLastAcceptedEpoch = nan;
ds5RefObsPosPrevAcceptedEpoch = nan;
ds5RefObsDeltaConsistencyConfirmCnt = 0;
ds5RefObsBadScoreByPrn = zeros(1, maxPrnId);
shadowFinalOutputLastPos = nan(3,1);
shadowFinalOutputLastVel = nan(3,1);
shadowFinalOutputLastEpoch = nan;
shadowFinalOutputLastSource = 0;
shadowFinalOutputHoldAge = inf;
shadowFinalOutputRecoveryConfirmCount = 0;
shadowFinalOutputPublishedLastPos = nan(3,1);
shadowFinalOutputPublishedLastVel = nan(3,1);
shadowFinalOutputPublishedLastEpoch = nan;
shadowRecoveredFilterPos = nan(3,1);
shadowRecoveredFilterVel = nan(3,1);
shadowRecoveredFilterClockM = nan;
shadowRecoveredFilterClockRateMps = nan;
shadowRecoveredFilterEpoch = nan;
shadowRecoveredFilterCoastAge = inf;
shadowRecoveredFilterGoodCount = 0;
shadowRecoveredFilterBadCount = 0;
shadowRecoveredFilterAuthority = false;
shadowRecoveredFilterLastUpdatePos = nan(3,1);
shadowRecoveredFilterLastUpdateEpoch = nan;
ds5RecoveredAbsAnchorPos = nan(3,1);
ds5RecoveredAbsAnchorVel = nan(3,1);
ds5RecoveredAbsAnchorEpoch = nan;
ds5RecoveredAbsAnchorTimeSec = nan;
ds5RecoveredAbsAnchorValid = false;
ds5RecoveredAbsAnchorFrozen = false;
ds5RecoveredAbsAnchorSource = 0;
baselineOutputLastPos = nan(3,1);
baselineOutputLastVel = nan(3,1);
baselineOutputLastEpoch = nan;
baselineOutputHoldAge = inf;
navResults.metricZ = nan(1, roundTime);
navResults.hitCount = zeros(1, roundTime);
navResults.prrCorrBlend = zeros(1, roundTime);
navResults.rhoCorrRmsM = nan(1, roundTime);
navResults.prrCorrRmsMps = nan(1, roundTime);
navResults.detectArmed = false(1, roundTime);
navResults.gnssKfUpdateUsed = false(1, roundTime);
navResults.clockKfUpdateUsed = false(1, roundTime);
navResults.gnssSatAvailNum = zeros(1, roundTime);
navResults.gnssSatUsedNum = zeros(1, roundTime);
navResults.gnssGateRejectNum = zeros(1, roundTime);
navResults.shadowRecoveredSatNum = zeros(1, roundTime);
navResults.shadowRecoveryReady = false(1, roundTime);
navResults.shadowReleaseCounter = zeros(1, roundTime);
navResults.shadowRecoverySwitched = false(1, roundTime);
navResults.shadowTargetHz = nan(numActChnList, roundTime);
navResults.shadowTrackResidualHz = nan(numActChnList, roundTime);
navResults.shadowTrackCmHz = nan(1, roundTime);
navResults.shadowTrackDfHz = nan(1, roundTime);
navResults.shadowCarrErrMed = nan(1, roundTime);
navResults.recoveryHoldCounter = zeros(1, roundTime);
navResults.reentryStrictCounter = zeros(1, roundTime);
navResults.codeReacqUsedNum = zeros(1, roundTime);
navResults.codeReacqOffsetMedChips = nan(1, roundTime);
navResults.codeReacqAppliedMedChips = nan(1, roundTime);
navResults.codeReacqPeakRatioMed = nan(1, roundTime);
navResults.codePhaseCorrMedChips = nan(1, roundTime);
navResults.codePhaseCorrRmsM = nan(1, roundTime);
navResults.codeWideUsedNum = zeros(1, roundTime);
navResults.codeWideOffsetMedChips = nan(1, roundTime);
navResults.codeWidePeakRatioMed = nan(1, roundTime);
navResults.codeWideStableMax = zeros(1, roundTime);
navResults.codeWideAccumMax = zeros(1, roundTime);
navResults.rawPDiag = nan(numActChnList, roundTime);
navResults.rawPdotDiag = nan(numActChnList, roundTime);
navResults.launchTimeDiag = nan(numActChnList, roundTime);
navResults.codePhaseTaoDiag = nan(numActChnList, roundTime);
navResults.remSampleNumDiag = nan(numActChnList, roundTime);
navResults.totalSampleNumDiag = nan(numActChnList, roundTime);
navResults.numOfCoIntDiag = nan(numActChnList, roundTime);
navResults.samplePosDiag = nan(numActChnList, roundTime);
navResults.trackRecvTimeDiag = nan(numActChnList, roundTime);
navResults.trackRecvTimeNormDiag = nan(numActChnList, roundTime);
navResults.sampleClockErrSampDiag = nan(numActChnList, roundTime);
navResults.pseudorangeMsDiag = nan(numActChnList, roundTime);
navResults.pseudorangeModuloMsDiag = nan(numActChnList, roundTime);
navResults.deltaRawPMean = nan(1, roundTime);
navResults.deltaRawPMed = nan(1, roundTime);
navResults.deltaRawPP95 = nan(1, roundTime);
navResults.deltaRawPMax = nan(1, roundTime);
navResults.deltaRawPUsedNum = zeros(1, roundTime);
navResults.deltaRawPRejectNum = zeros(1, roundTime);
navResults.clkDriftHz = nan(1, roundTime);
navResults.kfClockMps = nan(1, roundTime);
navResults.kfClockPrevMps = nan(1, roundTime);
navResults.shadowAcqValidNum = zeros(1, roundTime);
navResults.shadowAcqOffsetChips = nan(numActChnList, roundTime);
navResults.shadowAcqOffsetSamples = nan(numActChnList, roundTime);
navResults.shadowAcqPeakRatio = nan(numActChnList, roundTime);
navResults.shadowAcqZeroRatio = nan(numActChnList, roundTime);
navResults.shadowAcqBestMetric = nan(numActChnList, roundTime);
navResults.shadowAcqRawValid = false(numActChnList, roundTime);
navResults.shadowAcqRawOffsetChips = nan(numActChnList, roundTime);
navResults.shadowAcqRawPeakRatio = nan(numActChnList, roundTime);
navResults.shadowAcqRawZeroRatio = nan(numActChnList, roundTime);
navResults.shadowAcqQualified = false(numActChnList, roundTime);
navResults.shadowAcqPeakWidthChips = nan(numActChnList, roundTime);
navResults.shadowAcqLocalDropRatio = nan(numActChnList, roundTime);
navResults.shadowAcqTop1OffsetChips = nan(numActChnList, roundTime);
navResults.shadowAcqTop2OffsetChips = nan(numActChnList, roundTime);
navResults.shadowAcqTop3OffsetChips = nan(numActChnList, roundTime);
navResults.shadowAcqTop1Score = nan(numActChnList, roundTime);
navResults.shadowAcqTop2Score = nan(numActChnList, roundTime);
navResults.shadowAcqTop3Score = nan(numActChnList, roundTime);
navResults.shadowAcqNearZeroPenalty = nan(numActChnList, roundTime);
navResults.shadowAcqNearZeroTrap = false(numActChnList, roundTime);
navResults.shadowAcqStableCnt = zeros(numActChnList, roundTime);
navResults.shadowAcqCandidateOffsetChips = nan(numActChnList, roundTime);
navResults.shadowCandDeltaRawP = nan(numActChnList, roundTime);
navResults.shadowCandDeltaImprove = nan(numActChnList, roundTime);
navResults.shadowCandUsed = false(numActChnList, roundTime);
navResults.shadowAcqVotePeak = zeros(numActChnList, roundTime);
navResults.shadowAcqVoteSecond = zeros(numActChnList, roundTime);
navResults.shadowAcqReadyHoldCnt = zeros(numActChnList, roundTime);
navResults.shadowAcqTrackBestHits = zeros(numActChnList, roundTime);
navResults.shadowAcqReady = false(numActChnList, roundTime);
navResults.shadowAcqReadySatNum = zeros(1, roundTime);
navResults.shadowCodeReadyForRecovery = false(numActChnList, roundTime);
navResults.shadowCodeReadySatNum = zeros(1, roundTime);
navResults.shadowRawReacqValid = false(numActChnList, roundTime);
navResults.shadowRawReacqOffsetChips = nan(numActChnList, roundTime);
navResults.shadowRawReacqFreqHz = nan(numActChnList, roundTime);
navResults.shadowRawReacqPeakRatio = nan(numActChnList, roundTime);
navResults.shadowRawReacqZeroRatio = nan(numActChnList, roundTime);
navResults.shadowRawReacqTop1OffsetChips = nan(numActChnList, roundTime);
navResults.shadowRawReacqTop2OffsetChips = nan(numActChnList, roundTime);
navResults.shadowRawReacqTop3OffsetChips = nan(numActChnList, roundTime);
navResults.shadowRawReacqTop1FreqHz = nan(numActChnList, roundTime);
navResults.shadowRawReacqTop2FreqHz = nan(numActChnList, roundTime);
navResults.shadowRawReacqTop3FreqHz = nan(numActChnList, roundTime);
navResults.shadowRawReacqTop1Metric = nan(numActChnList, roundTime);
navResults.shadowRawReacqTop2Metric = nan(numActChnList, roundTime);
navResults.shadowRawReacqTop3Metric = nan(numActChnList, roundTime);
navResults.shadowRawTrackInit = false(numActChnList, roundTime);
navResults.shadowRawTrackPending = false(numActChnList, roundTime);
navResults.shadowRawTrackPendingAge = zeros(numActChnList, roundTime);
navResults.shadowRawTrackValidateCounter = zeros(numActChnList, roundTime);
navResults.shadowRawTrackValidateWindowHits = zeros(numActChnList, roundTime);
navResults.shadowRawTrackRejectCounter = zeros(numActChnList, roundTime);
navResults.shadowRawTrackShortScore = zeros(numActChnList, roundTime);
navResults.shadowRawTrackValidatePass = false(numActChnList, roundTime);
navResults.shadowRawCandReuseUsed = false(numActChnList, roundTime);
navResults.shadowRawCandSelectIdx = zeros(numActChnList, roundTime);
navResults.shadowRawRelayUsed = false(numActChnList, roundTime);
navResults.shadowRawFailExcludeOffset = nan(numActChnList, roundTime);
navResults.shadowRawFailExcludeFreqHz = nan(numActChnList, roundTime);
navResults.shadowRawFailExcludeActive = false(numActChnList, roundTime);
navResults.shadowRawFailExcludeAge = nan(numActChnList, roundTime);
navResults.shadowRawFailExcludeApplied = false(numActChnList, roundTime);
navResults.shadowRawFailFallbackBlocked = false(numActChnList, roundTime);
navResults.shadowRawRelayCandIdx = zeros(numActChnList, roundTime);
navResults.shadowRawRelayCandOffset = nan(numActChnList, roundTime);
navResults.shadowRawRelayCandFreqHz = nan(numActChnList, roundTime);
navResults.shadowRawRelayReason = zeros(numActChnList, roundTime);
navResults.shadowRawClusterBestOffset = nan(numActChnList, roundTime);
navResults.shadowRawClusterBestFreqHz = nan(numActChnList, roundTime);
navResults.shadowRawClusterBestScore = zeros(numActChnList, roundTime);
navResults.shadowRawClusterSecondScore = zeros(numActChnList, roundTime);
navResults.shadowRawClusterBestHits = zeros(numActChnList, roundTime);
navResults.shadowRawClusterBestDistChips = nan(numActChnList, roundTime);
navResults.shadowRawCandMetricGap = nan(numActChnList, roundTime);
navResults.shadowRawCandClusterOnlyUsed = false(numActChnList, roundTime);
navResults.shadowRawCandSelectedClusterSupport = nan(numActChnList, roundTime);
navResults.shadowRawCandSelectedMetricNorm = nan(numActChnList, roundTime);
navResults.shadowRawInitClusterFirstUsed = false(numActChnList, roundTime);
navResults.shadowRawInitBlendAlpha = nan(numActChnList, roundTime);
navResults.shadowRawInitClusterScore = nan(numActChnList, roundTime);
navResults.shadowRawInitClusterHits = zeros(numActChnList, roundTime);
navResults.shadowRawInitClusterMargin = nan(numActChnList, roundTime);
navResults.shadowRawPassSatNum = zeros(1, roundTime);
navResults.shadowRawPassReadyMask = false(numActChnList, roundTime);
navResults.shadowRawPassReadyQualityBadCnt = zeros(numActChnList, roundTime);
navResults.shadowRawPassReadyRevoked = false(numActChnList, roundTime);
navResults.shadowRawPassReadyDeltaDev = nan(numActChnList, roundTime);
navResults.shadowRawPassNavDeltaMed = nan(1, roundTime);
navResults.shadowRawPassNavDeltaP95 = nan(1, roundTime);
navResults.shadowRawPassNavDeltaUsed = nan(numActChnList, roundTime);
navResults.shadowRawPassNavDeltaDetrended = nan(numActChnList, roundTime);
navResults.shadowRawPassNavSourceMode = zeros(numActChnList, roundTime);
navResults.shadowRawPassNavSourceRefDelta = nan(numActChnList, roundTime);
navResults.shadowRawPassDs5CommonApplied = false(1, roundTime);
navResults.shadowRawPassDs5CommonDragM = nan(1, roundTime);
navResults.shadowRawPassDs5CommonRateMps = nan(1, roundTime);
navResults.shadowRawPassDs5CommonBaseSatNum = zeros(1, roundTime);
navResults.shadowRawPassDs5CommonKeepSatNum = zeros(1, roundTime);
navResults.shadowRawPassDs5CommonSource = zeros(1, roundTime);
navResults.shadowRawPassDs5CommonPredM = nan(1, roundTime);
navResults.shadowRawPassDs5CommonMeasM = nan(1, roundTime);
navResults.shadowRawPassDs5CommonP95BeforeM = nan(1, roundTime);
navResults.shadowRawPassDs5CommonP95AfterM = nan(1, roundTime);
navResults.shadowDs5CommonDoppApplied = false(1, roundTime);
navResults.shadowDs5CommonDoppHz = nan(1, roundTime);
navResults.shadowDs5CommonDoppRateHzps = nan(1, roundTime);
navResults.shadowDs5CommonDoppBaseSatNum = zeros(1, roundTime);
navResults.shadowDs5CommonDoppKeepSatNum = zeros(1, roundTime);
navResults.shadowDs5CommonDoppSource = zeros(1, roundTime);
navResults.shadowDs5CommonDoppPredHz = nan(1, roundTime);
navResults.shadowDs5CommonDoppMeasHz = nan(1, roundTime);
navResults.shadowDs5CommonDoppP95BeforeHz = nan(1, roundTime);
navResults.shadowDs5CommonDoppP95AfterHz = nan(1, roundTime);
navResults.shadowDs5CommonDoppRmsBeforeHz = nan(1, roundTime);
navResults.shadowDs5CommonDoppRmsAfterHz = nan(1, roundTime);
navResults.shadowDs5CommonDoppL2AfterHz = nan(1, roundTime);
navResults.shadowDs5CommonDoppAlphaProxy = nan(1, roundTime);
navResults.shadowDs5CommonDoppAlphaGatePass = false(1, roundTime);
navResults.shadowDs5AidDeDragHz = nan(1, roundTime);
navResults.shadowRawPassDs5DetrendedGateMask = false(numActChnList, roundTime);
navResults.shadowRawPassDs5DetrendedDevM = nan(numActChnList, roundTime);
navResults.shadowRawPassDs5DetrendedGateM = nan(1, roundTime);
navResults.shadowRawPassDs5DetrendedRejectNum = zeros(1, roundTime);
navResults.shadowBranchDs5CommonApplied = false(1, roundTime);
navResults.shadowBranchDs5CommonDragM = nan(1, roundTime);
navResults.shadowBranchDs5DetrendedP95M = nan(1, roundTime);
navResults.shadowBranchDs5DetrendedScorePenaltyM = nan(1, roundTime);
navResults.shadowBranchAbsModelP95M = nan(1, roundTime);
navResults.shadowBranchModelMode = zeros(1, roundTime);  % 1 legacy prior-lift, 2 ds5 absolute-delta
navResults.shadowRawNavQualifiedSatNum = zeros(1, roundTime);
navResults.shadowRawNavQualifiedMask = false(numActChnList, roundTime);
navResults.shadowRawNavCoreSatNum = zeros(1, roundTime);
navResults.shadowRawNavCoreMask = false(numActChnList, roundTime);
navResults.shadowRawNavExpandSatNum = zeros(1, roundTime);
navResults.shadowRawNavExpandMask = false(numActChnList, roundTime);
navResults.shadowRawNavCoreMaxDevM = nan(1, roundTime);
navResults.shadowRawNavExpandMaxDevM = nan(1, roundTime);
navResults.shadowRawNavRelaxDs5Used = false(numActChnList, roundTime);
navResults.shadowRawNavRelaxDs5CoreUsed = false(numActChnList, roundTime);
navResults.shadowRawNavRelaxDs5TargetMask = false(numActChnList, roundTime);
navResults.shadowRawNavRelaxDs5BaseMask = false(numActChnList, roundTime);
navResults.shadowRawNavRelaxDs5DevM = nan(numActChnList, roundTime);
navResults.shadowRawNavRelaxDs5BaseMedM = nan(1, roundTime);
navResults.shadowRawNavRelaxDs5NavThrM = nan(1, roundTime);
navResults.shadowRawNavRelaxDs5CoreThrM = nan(1, roundTime);
navResults.shadowRawNavRelaxDs5PairLeadPrn = nan(1, roundTime);
navResults.shadowRawNavRelaxDs5PairOk = false(numActChnList, roundTime);
navResults.shadowCoreServoUsed = false(numActChnList, roundTime);
navResults.shadowCoreServoDevM = nan(numActChnList, roundTime);
navResults.shadowCoreServoStepChips = nan(numActChnList, roundTime);
navResults.shadowCoreServoCorrChips = nan(numActChnList, roundTime);
navResults.shadowCoreServoCoreMedM = nan(1, roundTime);
navResults.shadowCoreServoCoreMaxDevM = nan(1, roundTime);
navResults.shadowRawPassRawP = nan(numActChnList, roundTime);
navResults.shadowRawPassRawPdot = nan(numActChnList, roundTime);
navResults.shadowRawPassServoRawP = nan(numActChnList, roundTime);
navResults.shadowRawPassLiftRawP = nan(numActChnList, roundTime);
navResults.shadowRawPassLiftDeltaRawP = nan(numActChnList, roundTime);
navResults.shadowRawPassAmbigChips = nan(numActChnList, roundTime);
navResults.shadowRawPassClosedLiftRawP = nan(numActChnList, roundTime);
navResults.shadowRawPassClosedLiftDeltaRawP = nan(numActChnList, roundTime);
navResults.shadowRawPassClosedLiftDeltaAbsM = nan(numActChnList, roundTime);
navResults.shadowRawPassClosedLiftAmbigChips = nan(numActChnList, roundTime);
navResults.shadowRawPassClosedLiftSegmentId = nan(numActChnList, roundTime);
navResults.shadowRawPassClosedLiftQuality = nan(numActChnList, roundTime);
navResults.shadowRawPassClosedLiftRepairChips = nan(numActChnList, roundTime);
navResults.shadowRawPassClosedLiftAnchorSource = zeros(1, roundTime);
navResults.shadowRawPassSatClkCorr = nan(numActChnList, roundTime);
navResults.shadowRawPassSatClkDrift = nan(numActChnList, roundTime);
navResults.shadowRawPassSatPosX = nan(numActChnList, roundTime);
navResults.shadowRawPassSatPosY = nan(numActChnList, roundTime);
navResults.shadowRawPassSatPosZ = nan(numActChnList, roundTime);
navResults.shadowRawPassSatVelX = nan(numActChnList, roundTime);
navResults.shadowRawPassSatVelY = nan(numActChnList, roundTime);
navResults.shadowRawPassSatVelZ = nan(numActChnList, roundTime);
navResults.shadowRawNavQualifiedDeltaMed = nan(1, roundTime);
navResults.shadowRawNavQualifiedDeltaP95 = nan(1, roundTime);
navResults.shadowRawKfUpdateUsed = false(1, roundTime);
navResults.shadowRawKfUsedSatNum = zeros(1, roundTime);
navResults.shadowRawKfDeltaMed = nan(1, roundTime);
navResults.shadowRawKfDeltaP95 = nan(1, roundTime);
navResults.shadowRawKfDeltaDetrendMed = nan(1, roundTime);
navResults.shadowRawKfDeltaDetrendP95 = nan(1, roundTime);
navResults.shadowRawKfCandidateSatNum = zeros(1, roundTime);
navResults.shadowRawKfGateRejectNum = zeros(1, roundTime);
navResults.shadowRawKfSourceGateRejectNum = zeros(1, roundTime);
navResults.shadowRawKfTrustedCandNum = zeros(1, roundTime);
navResults.shadowRawKfMode1CandNum = zeros(1, roundTime);
navResults.shadowRawKfMode2CandNum = zeros(1, roundTime);
navResults.shadowRawKfMode3CandNum = zeros(1, roundTime);
navResults.shadowRawKfMode4CandNum = zeros(1, roundTime);
navResults.shadowBranchReady = false(1, roundTime);
navResults.shadowBranchSatNum = zeros(1, roundTime);
navResults.shadowBranchX = nan(1, roundTime);
navResults.shadowBranchY = nan(1, roundTime);
navResults.shadowBranchZ = nan(1, roundTime);
navResults.shadowBranchClockM = nan(1, roundTime);
navResults.shadowBranchPriorRmsM = nan(1, roundTime);
navResults.shadowBranchPostfitRmsM = nan(1, roundTime);
navResults.shadowBranchPdop = nan(1, roundTime);
navResults.shadowBranchPosJumpM = nan(1, roundTime);
navResults.shadowBranchAmbigMedChips = nan(1, roundTime);
navResults.shadowBranchRawP = nan(numActChnList, roundTime);
navResults.shadowBranchAmbigChips = nan(numActChnList, roundTime);
navResults.shadowBranchAmbigPrevChips = nan(numActChnList, roundTime);
navResults.shadowBranchAmbigStepChips = nan(numActChnList, roundTime);
navResults.shadowBranchAmbigContinuityUsed = false(numActChnList, roundTime);
navResults.shadowBranchSubsetMask = false(numActChnList, roundTime);
navResults.shadowBranchRobustScore = nan(1, roundTime);
navResults.shadowBranchDopplerGateMask = false(numActChnList, roundTime);
navResults.shadowBranchDopplerGateResidualMps = nan(numActChnList, roundTime);
navResults.shadowBranchDopplerGateSource = zeros(1, roundTime);
navResults.shadowBranchDopplerGateSign = nan(1, roundTime);
navResults.shadowBranchDopplerGateKeepFrac = nan(1, roundTime);
navResults.shadowTrustedAnchorValid = false(1, roundTime);
navResults.shadowTrustedAnchorSource = zeros(1, roundTime);
navResults.shadowTrustedAnchorBaselineFed = false(1, roundTime);
navResults.shadowTrustedAnchorX = nan(1, roundTime);
navResults.shadowTrustedAnchorY = nan(1, roundTime);
navResults.shadowTrustedAnchorZ = nan(1, roundTime);
navResults.shadowTrustedAnchorVX = nan(1, roundTime);
navResults.shadowTrustedAnchorVY = nan(1, roundTime);
navResults.shadowTrustedAnchorVZ = nan(1, roundTime);
navResults.shadowBranchPriorSource = zeros(1, roundTime);
navResults.shadowBranchPriorX = nan(1, roundTime);
navResults.shadowBranchPriorY = nan(1, roundTime);
navResults.shadowBranchPriorZ = nan(1, roundTime);
navResults.shadowBranchAbsoluteRefDiffM = nan(1, roundTime);
navResults.shadowBranchAbsoluteRefValid = false(1, roundTime);
navResults.shadowBranchAbsoluteConsistent = false(1, roundTime);
navResults.shadowBranchObsGateKeepNum = zeros(1, roundTime);
navResults.shadowBranchObsGateRejectNum = zeros(1, roundTime);
navResults.shadowBranchObsGateM = nan(1, roundTime);
navResults.shadowBranchObsRefDetP95M = nan(1, roundTime);
navResults.shadowBranchTakeoverCounter = zeros(1, roundTime);
navResults.shadowBranchTakeoverReady = false(1, roundTime);
navResults.shadowTakeoverX = nan(1, roundTime);
navResults.shadowTakeoverY = nan(1, roundTime);
navResults.shadowTakeoverZ = nan(1, roundTime);
navResults.shadowTakeoverClockM = nan(1, roundTime);
navResults.shadowTakeoverFirstEpoch = nan;
navResults.shadowBranchResetApplied = false(1, roundTime);
navResults.shadowBranchResetEpoch = nan;
navResults.shadowBranchResetJumpM = nan(1, roundTime);
navResults.shadowBranchResetReason = zeros(1, roundTime);
navResults.shadowBranchInjectUsed = false(1, roundTime);
navResults.shadowBranchInjectUsedSatNum = zeros(1, roundTime);
navResults.shadowBranchInjectRejectNum = zeros(1, roundTime);
navResults.shadowBranchInjectResidualMedM = nan(1, roundTime);
navResults.shadowBranchInjectResidualP95M = nan(1, roundTime);
navResults.shadowRecoveryMode = false(1, roundTime);
navResults.shadowRecoveryRemain = zeros(1, roundTime);
navResults.shadowRecoveryBranchSane = false(1, roundTime);
navResults.shadowRecoverySuppressMainGnss = false(1, roundTime);
navResults.shadowDs5ObserveOnlyRecovery = false(1, roundTime);
navResults.shadowDs5RecoveryAuthorized = false(1, roundTime);
navResults.shadowDs5RecoveryAuthorityEligible = false(1, roundTime);
navResults.shadowDs5RecoveryAuthorityRemain = zeros(1, roundTime);
navResults.shadowDs5RecoveryAuthorityConfirm = zeros(1, roundTime);
navResults.shadowDs5RecoveryAuthorityRefDiffM = nan(1, roundTime);
navResults.shadowDs5SoftClampApplied = false(1, roundTime);
navResults.shadowDs5SoftClampAlpha = zeros(1, roundTime);
navResults.shadowDs5SoftClampDiffM = nan(1, roundTime);
navResults.shadowRecoveryReanchorApplied = false(1, roundTime);
navResults.shadowRecoveryReanchorCount = zeros(1, roundTime);
navResults.shadowRecoveryNavBranchDiffM = nan(1, roundTime);
navResults.shadowClosedLoopX = nan(1, roundTime);
navResults.shadowClosedLoopY = nan(1, roundTime);
navResults.shadowClosedLoopZ = nan(1, roundTime);
navResults.shadowClosedLoopSource = zeros(1, roundTime);  % 1 branch, 2 coast, 3 nav, 4 trusted hold, 5 ds5-refobs hold, 6 ds5-refobs position
navResults.shadowClosedLoopCoastAge = nan(1, roundTime);
navResults.shadowClosedLoopTrustedBlendAlpha = zeros(1, roundTime);
navResults.shadowClosedLoopSafeBranchUsed = false(1, roundTime);
navResults.shadowClosedLoopSafeBranchRejected = false(1, roundTime);
navResults.shadowClosedLoopSafeBranchDiffM = nan(1, roundTime);
navResults.shadowClosedLoopSafeBranchAlpha = zeros(1, roundTime);
navResults.shadowFinalOutputX = nan(1, roundTime);
navResults.shadowFinalOutputY = nan(1, roundTime);
navResults.shadowFinalOutputZ = nan(1, roundTime);
navResults.shadowFinalOutputVX = nan(1, roundTime);
navResults.shadowFinalOutputVY = nan(1, roundTime);
navResults.shadowFinalOutputVZ = nan(1, roundTime);
navResults.shadowFinalOutputSource = zeros(1, roundTime);  % 1 baseline, 2 recovered, 3 recovered-hold, 4 trusted-fallback, 5 baseline-hold
navResults.shadowFinalOutputHoldAge = nan(1, roundTime);
navResults.shadowFinalOutputUseRecovered = false(1, roundTime);
navResults.shadowFinalOutputRecoveryConfirmCount = zeros(1, roundTime);
navResults.shadowFinalOutputContinuityDiffM = nan(1, roundTime);
navResults.shadowFinalOutputContinuityPass = false(1, roundTime);
navResults.shadowFinalOutputSlewApplied = false(1, roundTime);
navResults.shadowFinalOutputSlewStepM = nan(1, roundTime);
navResults.shadowFinalOutputHoldBaselineCompete = false(1, roundTime);
navResults.shadowFinalOutputHoldBaselineSelected = false(1, roundTime);
navResults.shadowFinalObsContractRawPass = false(1, roundTime);
navResults.shadowFinalObsContractPass = false(1, roundTime);
navResults.shadowFinalObsContractTrackingHold = false(1, roundTime);
navResults.shadowFinalObsContractBaseSeedHold = false(1, roundTime);
navResults.shadowBaselineOutputX = nan(1, roundTime);
navResults.shadowBaselineOutputY = nan(1, roundTime);
navResults.shadowBaselineOutputZ = nan(1, roundTime);
navResults.shadowBaselineOutputVX = nan(1, roundTime);
navResults.shadowBaselineOutputVY = nan(1, roundTime);
navResults.shadowBaselineOutputVZ = nan(1, roundTime);
navResults.shadowBaselineOutputValid = false(1, roundTime);
navResults.shadowBaselineOutputHoldAge = nan(1, roundTime);
navResults.shadowFinalOutputBaselineValid = false(1, roundTime);
navResults.shadowFinalOutputRecoveryGatePass = false(1, roundTime);
navResults.shadowFinalOutputRecoveryBaselineDiffM = nan(1, roundTime);
navResults.shadowFinalOutputBaselineProxyP95M = nan(1, roundTime);
navResults.shadowFinalOutputRecoveryProxyP95M = nan(1, roundTime);
navResults.shadowFinalOutputRecoveryProxyImproveM = nan(1, roundTime);
navResults.shadowRecoveredFilterX = nan(1, roundTime);
navResults.shadowRecoveredFilterY = nan(1, roundTime);
navResults.shadowRecoveredFilterZ = nan(1, roundTime);
navResults.shadowRecoveredFilterVX = nan(1, roundTime);
navResults.shadowRecoveredFilterVY = nan(1, roundTime);
navResults.shadowRecoveredFilterVZ = nan(1, roundTime);
navResults.shadowRecoveredFilterClockM = nan(1, roundTime);
navResults.shadowRecoveredFilterClockRateMps = nan(1, roundTime);
navResults.shadowRecoveredFilterClockMeasPredDiffM = nan(1, roundTime);
navResults.shadowRecoveredFilterClockReset = false(1, roundTime);
navResults.shadowRecoveredFilterValid = false(1, roundTime);
navResults.shadowRecoveredFilterAuthority = false(1, roundTime);
navResults.shadowRecoveredFilterUpdated = false(1, roundTime);
navResults.shadowRecoveredFilterCoastAge = nan(1, roundTime);
navResults.shadowRecoveredFilterGoodCount = zeros(1, roundTime);
navResults.shadowRecoveredFilterBadCount = zeros(1, roundTime);
navResults.shadowRecoveredFilterMeasPredDiffM = nan(1, roundTime);
navResults.shadowRecoveredFilterSource = zeros(1, roundTime);  % 1 refObs update, 2 propagated, 3 reset
navResults.shadowRecoveredFilterMeasPass = false(1, roundTime);
navResults.shadowRecoveredFilterFilterValidPass = false(1, roundTime);
navResults.shadowRecoveredFilterAuthorityAbsAnchorPass = false(1, roundTime);
navResults.shadowRecoveredFilterAuthorityMeasPredPass = false(1, roundTime);
navResults.shadowRecoveredFilterPropStepM = nan(1, roundTime);
navResults.shadowRecoveredFilterPropTotalM = nan(1, roundTime);
navResults.shadowRecoveredFilterPropCapPass = true(1, roundTime);
navResults.shadowRecoveredFilterAuthorityCapPass = true(1, roundTime);
navResults.shadowRecoveredFilterBaselineDiffM = nan(1, roundTime);
navResults.shadowRecoveredFilterDopplerVelValid = false(1, roundTime);
navResults.shadowRecoveredFilterDopplerVelX = nan(1, roundTime);
navResults.shadowRecoveredFilterDopplerVelY = nan(1, roundTime);
navResults.shadowRecoveredFilterDopplerVelZ = nan(1, roundTime);
navResults.shadowRecoveredFilterDopplerVelSatNum = zeros(1, roundTime);
navResults.shadowRecoveredFilterDopplerVelRmsMps = nan(1, roundTime);
navResults.shadowRecoveredFilterDopplerVelP95Mps = nan(1, roundTime);
navResults.shadowRecoveredFilterDopplerVelSign = nan(1, roundTime);
navResults.shadowRecoveredFilterDopplerVelSource = zeros(1, roundTime);
navResults.shadowRecoveredFilterAbsAnchorValid = false(1, roundTime);
navResults.shadowRecoveredFilterAbsAnchorFrozen = false(1, roundTime);
navResults.shadowRecoveredFilterAbsAnchorSource = zeros(1, roundTime);
navResults.shadowRecoveredFilterAbsAnchorAgeEpochs = nan(1, roundTime);
navResults.shadowRecoveredFilterAbsAnchorX = nan(1, roundTime);
navResults.shadowRecoveredFilterAbsAnchorY = nan(1, roundTime);
navResults.shadowRecoveredFilterAbsAnchorZ = nan(1, roundTime);
navResults.shadowRecoveredFilterAbsAnchorVX = nan(1, roundTime);
navResults.shadowRecoveredFilterAbsAnchorVY = nan(1, roundTime);
navResults.shadowRecoveredFilterAbsAnchorVZ = nan(1, roundTime);
navResults.shadowRecoveredFilterAbsAnchorDiffM = nan(1, roundTime);
navResults.shadowRecoveredFilterAbsAnchorGatePass = false(1, roundTime);
navResults.shadowRecoveredFilterAbsAnchorSoftPass = false(1, roundTime);
navResults.shadowInitRecvTimeShiftMs = nan(numActChnList, roundTime);
navResults.shadowInitSampleShift = nan(numActChnList, roundTime);
navResults.shadowInitNumCoIntShift = nan(numActChnList, roundTime);
navResults.shadowRawTrackActive = false(numActChnList, roundTime);
navResults.shadowRawTrackCarrFreqHz = nan(numActChnList, roundTime);
navResults.shadowRawTrackCodeFreqHz = nan(numActChnList, roundTime);
navResults.shadowRawTrackCarrFreqStartHz = nan(numActChnList, roundTime);
navResults.shadowRawTrackCarrCmdHz = nan(numActChnList, roundTime);
navResults.shadowRawTrackCarrBaseCmdHz = nan(numActChnList, roundTime);
navResults.shadowRawTrackCarrShadowCmdHz = nan(numActChnList, roundTime);
navResults.shadowRawTrackCarrAidFreqHz = nan(numActChnList, roundTime);
navResults.shadowRawTrackCarrShadowTargetHz = nan(numActChnList, roundTime);
navResults.shadowRawTrackCarrShadowPullHz = nan(numActChnList, roundTime);
navResults.shadowRawTrackCarrShadowWeight = nan(numActChnList, roundTime);
navResults.shadowRawTrackCarrForceShadow = false(numActChnList, roundTime);
navResults.shadowRawTrackCarrPllBypass = false(numActChnList, roundTime);
navResults.shadowRawTrackCarrOldNcoHz = nan(numActChnList, roundTime);
navResults.shadowRawTrackCarrNcoHz = nan(numActChnList, roundTime);
navResults.shadowRawTrackCarrNcoStepHz = nan(numActChnList, roundTime);
navResults.shadowRawTrackCarrOldErrorCycles = nan(numActChnList, roundTime);
navResults.shadowRawTrackCarrErrorRawCycles = nan(numActChnList, roundTime);
navResults.shadowRawTrackCarrErrorScaledCycles = nan(numActChnList, roundTime);
navResults.shadowRawTrackPromptIP = nan(numActChnList, roundTime);
navResults.shadowRawTrackPromptQP = nan(numActChnList, roundTime);
navResults.shadowRawTrackEarlyI = nan(numActChnList, roundTime);
navResults.shadowRawTrackEarlyQ = nan(numActChnList, roundTime);
navResults.shadowRawTrackLateI = nan(numActChnList, roundTime);
navResults.shadowRawTrackLateQ = nan(numActChnList, roundTime);
navResults.shadowRawTrackDllDiscrRaw = nan(numActChnList, roundTime);
navResults.shadowRawTrackDllDiscr = nan(numActChnList, roundTime);
navResults.shadowRawTrackCodeError = nan(numActChnList, roundTime);
navResults.shadowRawTrackCodeRefAppliedChips = nan(numActChnList, roundTime);
navResults.shadowRawTrackCodeRefNcoPullHz = nan(numActChnList, roundTime);
navResults.shadowRawTrackCodeRefNcoStepHz = nan(numActChnList, roundTime);
navResults.shadowDs5RefObsTrackRawSampled = false(numActChnList, roundTime);
navResults.shadowDs5RefObsTrackRawSteps = zeros(numActChnList, roundTime);
navResults.shadowDs5RefObsTrackCoasted = false(numActChnList, roundTime);
navResults.shadowDs5TrueRefHz = nan(numActChnList, roundTime);
navResults.shadowDs5TrueRefCodeChips = nan(numActChnList, roundTime);
navResults.shadowDs5TrueRefWinHz = nan(numActChnList, roundTime);
navResults.shadowDs5TrueRefWinChips = nan(numActChnList, roundTime);
navResults.shadowDs5TrueRefReady = false(numActChnList, roundTime);
navResults.shadowDs5RefObsTrackDriven = false(numActChnList, roundTime);
navResults.shadowDs5RefObsUsed = false(numActChnList, roundTime);
navResults.shadowDs5RefObsDeltaM = nan(numActChnList, roundTime);
navResults.shadowDs5RefObsBaseDeltaM = nan(numActChnList, roundTime);
navResults.shadowDs5RefObsSeedDeltaM = nan(numActChnList, roundTime);
navResults.shadowDs5RefObsSeedUseBase = false(numActChnList, roundTime);
navResults.shadowDs5RefObsLocalCorrChips = nan(numActChnList, roundTime);
navResults.shadowDs5RefObsFreqErrHz = nan(numActChnList, roundTime);
navResults.shadowDs5RefObsCodeErrChips = nan(numActChnList, roundTime);
navResults.shadowDs5ObsContractPass = false(1, roundTime);
navResults.shadowDs5ObsContractUsedSatNum = zeros(1, roundTime);
navResults.shadowDs5ObsContractCommonClockM = nan(1, roundTime);
navResults.shadowDs5ObsContractSpreadMedM = nan(1, roundTime);
navResults.shadowDs5ObsContractSpreadP95M = nan(1, roundTime);
navResults.shadowDs5ObsContractTrackingPass = false(1, roundTime);
navResults.shadowDs5ObsContractCommonClockPass = false(1, roundTime);
navResults.shadowDs5ObsContractSpreadPass = false(1, roundTime);
navResults.shadowDs5ObsContractBaseSeedPass = false(1, roundTime);
navResults.shadowDs5ObsContractBaseSeedSatNum = zeros(1, roundTime);
navResults.shadowDs5ObsContractBaseSeedFrac = nan(1, roundTime);
navResults.shadowDs5ObsContractUsableDeltaM = nan(numActChnList, roundTime);
navResults.shadowDs5ObsContractResidualM = nan(numActChnList, roundTime);
navResults.shadowDs5RefObsRecoveryPass = false(1, roundTime);
navResults.shadowDs5RefObsRecoveryUsedSatNum = zeros(1, roundTime);
navResults.shadowDs5RefObsRecoveryCodeMedChips = nan(1, roundTime);
navResults.shadowDs5RefObsRecoveryCodeP95Chips = nan(1, roundTime);
navResults.shadowDs5RefObsRecoveryFreqP95Hz = nan(1, roundTime);
navResults.shadowDs5RefObsRecoveryRawUsedSatNum = zeros(1, roundTime);
navResults.shadowDs5RefObsRecoveryRawCodeMedChips = nan(1, roundTime);
navResults.shadowDs5RefObsRecoveryRawCodeP95Chips = nan(1, roundTime);
navResults.shadowDs5RefObsRecoveryRawFreqP95Hz = nan(1, roundTime);
navResults.shadowDs5RefObsRecoveryRobustDropSatNum = zeros(1, roundTime);
navResults.shadowDs5RefObsRecoveryKeep = false(numActChnList, roundTime);
navResults.shadowDs5RefObsRecoveryBranchRequired = false(1, roundTime);
navResults.shadowDs5RefObsRecoveryBranchGatePass = false(1, roundTime);
navResults.shadowDs5RefObsTrackingPass = false(1, roundTime);
navResults.shadowDs5RefObsTrackingRawPass = false(1, roundTime);
navResults.shadowDs5RefObsTrackingHoldAge = nan(1, roundTime);
navResults.shadowDs5RefObsTrackingUsedSatGatePass = false(1, roundTime);
navResults.shadowDs5RefObsTrackingCodeMedGatePass = false(1, roundTime);
navResults.shadowDs5RefObsTrackingCodeP95GatePass = false(1, roundTime);
navResults.shadowDs5RefObsTrackingFreqGatePass = false(1, roundTime);
navResults.shadowDs5RefObsPosUsed = false(1, roundTime);
navResults.shadowDs5RefObsPosSatNum = zeros(1, roundTime);
navResults.shadowDs5RefObsPosX = nan(1, roundTime);
navResults.shadowDs5RefObsPosY = nan(1, roundTime);
navResults.shadowDs5RefObsPosZ = nan(1, roundTime);
navResults.shadowDs5RefObsPosClockM = nan(1, roundTime);
navResults.shadowDs5RefObsPosClockResidualM = nan(1, roundTime);
navResults.shadowDs5RefObsPosCommonClockM = nan(1, roundTime);
navResults.shadowDs5RefObsPosPostfitRmsM = nan(1, roundTime);
navResults.shadowDs5RefObsPosPdop = nan(1, roundTime);
navResults.shadowDs5RefObsPosCorrM = nan(1, roundTime);
navResults.shadowDs5RefObsPosAnchorDiffM = nan(1, roundTime);
navResults.shadowDs5RefObsPosJumpM = nan(1, roundTime);
navResults.shadowDs5RefObsPosSpeedMps = nan(1, roundTime);
navResults.shadowDs5RefObsPosAccelMps2 = nan(1, roundTime);
navResults.shadowDs5RefObsPosDynGatePass = false(1, roundTime);
navResults.shadowDs5RefObsPosPredVelDiffMps = nan(1, roundTime);
navResults.shadowDs5RefObsPosPredSupportPass = false(1, roundTime);
navResults.shadowDs5RefObsPosJumpGatePass = false(1, roundTime);
navResults.shadowDs5RefObsPosSpeedGatePass = false(1, roundTime);
navResults.shadowDs5RefObsPosAccelGatePass = false(1, roundTime);
navResults.shadowDs5RefObsPosDynGateMode = zeros(1, roundTime);
navResults.shadowDs5RefObsPosAnchorSource = zeros(1, roundTime);
navResults.shadowDs5RefObsPosAbsAnchorDiffM = nan(1, roundTime);
navResults.shadowDs5RefObsPosAbsAnchorGatePass = false(1, roundTime);
navResults.shadowDs5RefObsPosAbsAnchorSoftPass = false(1, roundTime);
navResults.shadowDs5RefObsPosCurrRefX = nan(1, roundTime);
navResults.shadowDs5RefObsPosCurrRefY = nan(1, roundTime);
navResults.shadowDs5RefObsPosCurrRefZ = nan(1, roundTime);
navResults.shadowDs5RefObsPosAnchorX = nan(1, roundTime);
navResults.shadowDs5RefObsPosAnchorY = nan(1, roundTime);
navResults.shadowDs5RefObsPosAnchorZ = nan(1, roundTime);
navResults.shadowDs5RefObsPosRebasedDeltaM = nan(numActChnList, roundTime);
navResults.shadowDs5RefObsPosRobustKeep = false(numActChnList, roundTime);
navResults.shadowDs5RefObsPosResidualM = nan(numActChnList, roundTime);
navResults.shadowDs5RefObsPosBadScore = nan(numActChnList, roundTime);
navResults.shadowDs5RefObsPosRawAnchorLiftUsed = false(1, roundTime);
navResults.shadowDs5RefObsPosRawAnchorLiftSpreadP95M = nan(1, roundTime);
navResults.shadowDs5RefObsPosRawAnchorLiftAmbigChips = nan(numActChnList, roundTime);
navResults.shadowDs5RefObsPosDeltaSpreadMedM = nan(1, roundTime);
navResults.shadowDs5RefObsPosDeltaSpreadP95M = nan(1, roundTime);
navResults.shadowDs5RefObsPosDeltaConsistencyPass = false(1, roundTime);
navResults.shadowDs5RefObsPosDeltaConsistencyConfirm = zeros(1, roundTime);
navResults.shadowDs5CandInTrueWindow = false(numActChnList, roundTime);
navResults.shadowDs5CandScore = nan(numActChnList, roundTime);
navResults.shadowDs5CurrentScore = nan(numActChnList, roundTime);
navResults.shadowDs5CandBetterThanCurrent = false(numActChnList, roundTime);
navResults.shadowDs5ProbationPass = false(numActChnList, roundTime);
navResults.shadowDs5Takeover = false(numActChnList, roundTime);
navResults.shadowDs5TakeoverRevoke = false(numActChnList, roundTime);
navResults.shadowRawTrackCodeOffsetChips = nan(numActChnList, roundTime);
navResults.shadowRawTrackCodePhaseChips = nan(numActChnList, roundTime);
navResults.shadowRawTrackCarrPhaseRad = nan(numActChnList, roundTime);
navResults.shadowRawTrackCarrierCycles = nan(numActChnList, roundTime);
navResults.shadowRawTrackCarrierDeltaCycles = nan(numActChnList, roundTime);
navResults.shadowRawTrackCodeDeltaChips = nan(numActChnList, roundTime);
navResults.shadowRawTrackLockRun = zeros(numActChnList, roundTime);
navResults.shadowRawTrackCycleSlip = false(numActChnList, roundTime);
navResults.shadowRawTrackDeltaRawP = nan(numActChnList, roundTime);
navResults.shadowRawTrackImprove = nan(numActChnList, roundTime);
navResults.shadowMainSwitchCounter = zeros(numActChnList, roundTime);
navResults.shadowMainSwitch = false(numActChnList, roundTime);
navResults.shadowMainHoldCounter = zeros(numActChnList, roundTime);
navResults.shadowMainFollowActive = false(numActChnList, roundTime);
navResults.shadowConsensusOffsetChips = nan(1, roundTime);
navResults.shadowConsensusSatNum = zeros(1, roundTime);
navResults.shadowConsensusStableCnt = zeros(1, roundTime);
navResults.shadowConsensusReady = false(1, roundTime);
shadowStableCnt = zeros(numActChnList, 1);
shadowRecoveredMask = false(numActChnList, 1);
trackShadow = trackDeepIn;
shadowTrackPending = false(numActChnList, 1);
shadowTrackPendingAge = zeros(numActChnList, 1);
shadowTrackActive = false(numActChnList, 1);
shadowTrackValidateCnt = zeros(numActChnList, 1);
shadowTrackRejectCnt = zeros(numActChnList, 1);
shadowTrackShortScore = zeros(numActChnList, 1);
shadowTrackLastImprove = nan(numActChnList, 1);
shadowTrackRecheckCnt = zeros(numActChnList, 1);
shadowTrackValidateHist = false(numActChnList, max(1, settings.deepShadowRawTrackValidateWindowEpochs));
shadowTrackPassReady = false(numActChnList, 1);
shadowPassReadyBadCnt = zeros(numActChnList, 1);
shadowRawCandValid = false(numActChnList, 1);
shadowRawCandAge = inf(numActChnList, 1);
shadowRawCandK = max(1, settings.deepShadowRawCandTopKUse);
shadowRawCandOffsetChips = nan(numActChnList, shadowRawCandK);
shadowRawCandOffsetSamples = nan(numActChnList, shadowRawCandK);
shadowRawCandFreqHz = nan(numActChnList, shadowRawCandK);
shadowRawCandMetric = nan(numActChnList, shadowRawCandK);
shadowRawCandPeakRatio = nan(numActChnList, shadowRawCandK);
shadowRawCandUsedMask = false(numActChnList, shadowRawCandK);
shadowRawCandSelectIdx = zeros(numActChnList, 1);
shadowRawFailExcludeValid = false(numActChnList, 1);
shadowRawFailExcludeOffset = nan(numActChnList, 1);
shadowRawFailExcludeFreqHz = nan(numActChnList, 1);
shadowRawFailExcludeAge = inf(numActChnList, 1);
shadowRawRelayCooldownCnt = zeros(numActChnList, 1);
shadowRawClusterNum = max(3, shadowRawCandK + 1);
shadowRawClusterOffset = nan(numActChnList, shadowRawClusterNum);
shadowRawClusterFreqHz = nan(numActChnList, shadowRawClusterNum);
shadowRawClusterScore = zeros(numActChnList, shadowRawClusterNum);
shadowRawClusterHits = zeros(numActChnList, shadowRawClusterNum);
shadowRawClusterMiss = zeros(numActChnList, shadowRawClusterNum);
shadowMainSwitchCnt = zeros(numActChnList, 1);
shadowMainHoldCnt = zeros(numActChnList, 1);
shadowMainFollowActive = false(numActChnList, 1);
shadowTakeoverBackup = trackDeepIn;
shadowTakeoverBackupValid = false(numActChnList, 1);
shadowTakeoverRevokeCnt = zeros(numActChnList, 1);
ds5TrueRefHzState = nan(numActChnList, 1);
ds5TrueRefCodeState = nan(numActChnList, 1);
ds5TrueRefReadyState = false(numActChnList, 1);
    shadowAcqStableCnt = zeros(numActChnList, 1);
    shadowAcqCandidateOffset = nan(numActChnList, 1);
    shadowAcqReadyMask = false(numActChnList, 1);
    shadowAcqQualifiedMask = false(numActChnList, 1);
    shadowAcqVotePeak = zeros(numActChnList, 1);
    shadowAcqVoteSecond = zeros(numActChnList, 1);
    shadowAcqReadyHoldCnt = zeros(numActChnList, 1);
    shadowAcqTrackOffset = nan(numActChnList, settings.deepShadowTrackNum);
    shadowAcqTrackScore = zeros(numActChnList, settings.deepShadowTrackNum);
    shadowAcqTrackHits = zeros(numActChnList, settings.deepShadowTrackNum);
    shadowAcqTrackMiss = zeros(numActChnList, settings.deepShadowTrackNum);
    shadowConsensusOffset = nan;
    shadowConsensusStableCnt = 0;
shadowReleaseCnt = 0;
shadowRecoveryRampRemain = 0;
recoveryHoldCnt = 0;
reentryStrictCnt = 0;
diagTrackTimeSec = 0;
diagTrackCalls = 0;
diagRawReacqTimeSec = 0;
diagRawReacqCalls = 0;
diagImuPropTimeSec = 0;
diagMainNavTimeSec = 0;
diagRhoTimeSec = 0;
diagShadowNavTimeSec = 0;
diagEkfTimeSec = 0;
diagDeepFeedbackTimeSec = 0;
diagRunWallTic = tic;
if ~isfield(settings, 'deepPerfLightNormalMode')
    settings.deepPerfLightNormalMode = 1;
end
if ~isfield(settings, 'deepPerfDiagPrint')
    settings.deepPerfDiagPrint = 1;
end
if ~isfield(settings, 'deepPerfDiagInterval')
    settings.deepPerfDiagInterval = 10;
end

for currMeasNr = 1 : roundTime
    currMeasNr;
    diagEpochWallTic = tic;
    settings.recvTime = positioningTime;    
    settings.deepModeState = deepModeState;
    shadowLogicActive = (deepModeState >= 1) || any(shadowTrackPending) || any(shadowTrackActive) || ...
        any(shadowMainFollowActive) || any(shadowRawCandValid) || (shadowRecoveryRampRemain > 0);
    lightNormalMode = settings.deepPerfLightNormalMode && ~shadowLogicActive;
    if ~lightNormalMode
        shadowRawCandAge(shadowRawCandValid) = shadowRawCandAge(shadowRawCandValid) + 1;
        shadowRawRelayCooldownCnt = max(0, shadowRawRelayCooldownCnt - 1);
        staleCand = shadowRawCandValid & (shadowRawCandAge > settings.deepShadowRawCandReuseMaxEpochs);
        shadowRawCandValid(staleCand) = false;
        shadowRawCandAge(staleCand) = inf;
        shadowRawCandUsedMask(staleCand, :) = false;
        shadowRawCandSelectIdx(staleCand) = 0;
        shadowRawFailExcludeAge(shadowRawFailExcludeValid) = shadowRawFailExcludeAge(shadowRawFailExcludeValid) + 1;
        expiredFailExclude = shadowRawFailExcludeValid & ...
            (shadowRawFailExcludeAge > settings.deepShadowRawFailExcludeMaxAgeEpochs);
        shadowRawFailExcludeValid(expiredFailExclude) = false;
        shadowRawFailExcludeOffset(expiredFailExclude) = nan;
        shadowRawFailExcludeFreqHz(expiredFailExclude) = nan;
        shadowRawFailExcludeAge(expiredFailExclude) = inf;
        if settings.deepShadowRawClusterDecayPerReacq
            clusterDecayStep = settings.deepShadowRawClusterScoreDecay ^ ...
                (1 / max(1, settings.deepShadowRawReacqIntervalEpochs));
        else
            clusterDecayStep = settings.deepShadowRawClusterScoreDecay;
        end
        shadowRawClusterScore = clusterDecayStep * shadowRawClusterScore;
    end
    shadowKfReadyNow = false;
    shadowKfFromBranchInject = false;
    shadowBranchSaneNow = false;
    shadowKfDeltaUse = [];
    shadowKfDeltaRawUse = [];
    shadowKfLOSUse = [];
    shadowKfElUse = [];
    ds5AuthorityEligibleNow = false;
    ds5ObserveOnlyNow = false;
    ds5AuthorityRefDiffNow = nan;
    recoverySwitchedNow = false;
    holdCounterNow = recoveryHoldCnt;
    strictCounterNow = reentryStrictCnt;
    holdActive = holdCounterNow > 0;
    strictActive = strictCounterNow > 0;
    navResults.recoveryHoldCounter(1, currMeasNr) = holdCounterNow;
    navResults.reentryStrictCounter(1, currMeasNr) = strictCounterNow;
    recoveryHoldCnt = max(0, recoveryHoldCnt - 1);
    reentryStrictCnt = max(0, reentryStrictCnt - 1);
    switch deepModeState
        case 2
            settings.deepAidWeight = settings.deepAidWeightSpoof;
            settings.deepClkWeight = settings.deepClkWeightSpoof;
            settings.pllNoiseBandwidth = settings.deepPllNoiseBandwidthSpoof;
            rhoBlendNow = settings.deepRhoBlendSpoof;
            useGnssKfUpdateNow = logical(settings.deepUseGnssKfUpdateSpoof);
            useKfClockUpdateNow = logical(settings.deepUseKfClockUpdateSpoof);
            rScaleNow = settings.deepRScaleSpoof;
        case 1
            settings.deepAidWeight = settings.deepAidWeightSuspect;
            settings.deepClkWeight = settings.deepClkWeightSuspect;
            settings.pllNoiseBandwidth = settings.deepPllNoiseBandwidthSuspect;
            rhoBlendNow = settings.deepRhoBlendSuspect;
            useGnssKfUpdateNow = logical(settings.deepUseGnssKfUpdateSuspect);
            useKfClockUpdateNow = logical(settings.deepUseKfClockUpdateSuspect);
            rScaleNow = settings.deepRScaleSuspect;
        otherwise
            settings.deepAidWeight = settings.deepAidWeightNormal;
            settings.deepClkWeight = settings.deepClkWeightNormal;
            settings.pllNoiseBandwidth = settings.deepPllNoiseBandwidthNormal;
            rhoBlendNow = settings.deepRhoBlendNormal;
            useGnssKfUpdateNow = logical(settings.deepUseGnssKfUpdateNormal);
            useKfClockUpdateNow = logical(settings.deepUseKfClockUpdateNormal);
            rScaleNow = settings.deepRScaleNormal;
    end
    if deepModeState == 0 && shadowRecoveryRampRemain > 0
        rampAlpha = shadowRecoveryRampRemain / max(1, settings.deepShadowRecoveryRampEpochs);
        rScaleNow = (1 - rampAlpha) * rScaleNow + rampAlpha * settings.deepRScaleSpoof;
        shadowRecoveryRampRemain = shadowRecoveryRampRemain - 1;
    end
    if shadowRecoveryRemain > 0
        shadowRecoveryMode = true;
        shadowRecoveryRemain = shadowRecoveryRemain - 1;
    else
        shadowRecoveryMode = false;
    end
    if ds5RecoveryAuthorityRemain > 0
        ds5RecoveryAuthorityRemain = ds5RecoveryAuthorityRemain - 1;
    end
    if ~shadowRecoveryMode
        ds5RecoveryAuthorityRemain = 0;
        ds5RecoveryAuthorityConfirmCnt = 0;
        ds5RecoveryAuthorityBadCnt = 0;
    end
    ds5RecoveryAuthorized = shadowRecoveryMode && (ds5RecoveryAuthorityRemain > 0);
    shadowRecoveryReanchorCooldown = max(0, shadowRecoveryReanchorCooldown - 1);
    if shadowRecoveryMode && settings.deepShadowRecoverySuppressMainGnss
        useGnssKfUpdateNow = false;
        useKfClockUpdateNow = false;
        navResults.shadowRecoverySuppressMainGnss(1, currMeasNr) = true;
    end
    navResults.shadowRecoveryMode(1, currMeasNr) = shadowRecoveryMode;
    navResults.shadowRecoveryRemain(1, currMeasNr) = shadowRecoveryRemain;
    navResults.shadowRecoveryReanchorCount(1, currMeasNr) = shadowRecoveryReanchorCount;
    navResults.prrCorrBlend(1, currMeasNr) = rhoBlendNow;
    isArmed = (currMeasNr >= settings.deepDetectArmEpoch);
    navResults.detectArmed(1, currMeasNr) = isArmed;
    if settings.deepFastMode
        trackDeepIn = localAlignFastTrackState(trackDeepIn, trackResults, trackRawChIdx, trackRawStartIdx, currMeasNr, settings, positioningTime);
        if settings.deepUseIndependentDetectTrack
            trackDeepDet = localAlignFastTrackState(trackDeepDet, trackResults, trackRawChIdx, trackRawStartIdx, currMeasNr, settings, positioningTime);
        end
    end

    if currMeasNr < 2
        navSolut_1 = postNavLoose(trackDeepIn, settings, eph, TOW);
        positioningTime = positioningTime + settings.navSolPeriod / 1000;

        for ii = 1 : numActChnList
            trackDeepIn(ii).recvTime = trackDeepIn(ii).recvTime - navSolut_1.dt / settings.c;  
        end
        
        % 鎯杩愯鑷充簬涓嬩竴娆＄粍鍚堟椂鍒?
        diagLocalTic = tic;
        while (t0_gps + (t - t0_imu)) < positioningTime
            k1 = k+nn-1;
            if k1 > size(trj.imu, 1)
                error('IMU data exhausted during deep-coupling propagation.');
            end
            wvm = trj.imu(k:k1,1:6);  t = trj.imu(k1,end);
            ins = insupdate(ins, wvm);
            kf.Phikk_1 = kffk(ins);
            kf = kfupdate(kf);

            k = k + nn;       
        end
        diagImuPropTimeSec = diagImuPropTimeSec + toc(diagLocalTic);
        
        % GNSS 杩愯鑷充簬涓嬩竴娆＄粍鍚堟椂鍒?
        for ii = 1 : numActChnList
            trackans = trackDeepIn(ii);                
            
            oldAidFreq(ii,1) = trackans.carrFreq - settings.IF;      
            
            while trackans.recvTime < positioningTime     
                trackDeepIn(ii) = trackans;
                [trackans, I_P, Q_P] = perChannelTrackOnce(trackans, settings, fid);             

                if settings.deepKeepTrackHistory && trackans.recvTime < positioningTime
                    trackProcess(ii).codeErrorList = [trackProcess(ii).codeErrorList, trackans.codeError];
                    trackProcess(ii).carrErrorList = [trackProcess(ii).carrErrorList, trackans.carrError];
                    trackProcess(ii).codeFreqList = [trackProcess(ii).codeFreqList, trackans.codeFreq];
                    trackProcess(ii).carrFreqList = [trackProcess(ii).carrFreqList, trackans.carrFreq];
                    trackProcess(ii).PLI = [trackProcess(ii).PLI, (I_P^2-Q_P^2)/(I_P^2+Q_P^2)];
                    
                    if ii == 1
                        I_P_1_list = [I_P_1_list, I_P];
                        Q_P_1_list = [Q_P_1_list, Q_P];
                    end
        
                end
            end
            trackDeepIn(ii) = trackans;
        end
        trackDeepDet = trackDeepIn;
        
    else
             
    %% 寮€濮嬬揣缁勫悎
    codeUsed = false(numActChnList, 1);
    codeOffset = nan(numActChnList, 1);
    codeApplied = nan(numActChnList, 1);
    codePeakRatio = nan(numActChnList, 1);
    codePhaseCorr = nan(numActChnList, 1);
    codeWideUsed = false(numActChnList, 1);
    codeWideOffset = nan(numActChnList, 1);
    codeWidePeakRatio = nan(numActChnList, 1);
    codeWideStableCnt = zeros(numActChnList, 1);
    codeWideAccumCnt = zeros(numActChnList, 1);
    for ii = 1:numActChnList
        if isfield(trackDeepIn(ii), 'deepCodeReacqUsed')
            codeUsed(ii) = logical(trackDeepIn(ii).deepCodeReacqUsed);
        end
        if isfield(trackDeepIn(ii), 'deepCodeReacqOffsetChips')
            codeOffset(ii) = trackDeepIn(ii).deepCodeReacqOffsetChips;
        end
        if isfield(trackDeepIn(ii), 'deepCodeReacqAppliedChips')
            codeApplied(ii) = trackDeepIn(ii).deepCodeReacqAppliedChips;
        end
        if isfield(trackDeepIn(ii), 'deepCodeReacqPeakRatio')
            codePeakRatio(ii) = trackDeepIn(ii).deepCodeReacqPeakRatio;
        end
        if isfield(trackDeepIn(ii), 'deepCodePhaseCorrChips')
            codePhaseCorr(ii) = trackDeepIn(ii).deepCodePhaseCorrChips;
        end
        if isfield(trackDeepIn(ii), 'deepCodeWideUsed')
            codeWideUsed(ii) = logical(trackDeepIn(ii).deepCodeWideUsed);
        end
        if isfield(trackDeepIn(ii), 'deepCodeWideOffsetChips')
            codeWideOffset(ii) = trackDeepIn(ii).deepCodeWideOffsetChips;
        end
        if isfield(trackDeepIn(ii), 'deepCodeWidePeakRatio')
            codeWidePeakRatio(ii) = trackDeepIn(ii).deepCodeWidePeakRatio;
        end
        if isfield(trackDeepIn(ii), 'deepCodeWideStableCnt')
            codeWideStableCnt(ii) = trackDeepIn(ii).deepCodeWideStableCnt;
        end
        if isfield(trackDeepIn(ii), 'deepCodeWideAccumCount')
            codeWideAccumCnt(ii) = trackDeepIn(ii).deepCodeWideAccumCount;
        elseif isfield(trackDeepIn(ii), 'deepCodeWideMetricCount')
            codeWideAccumCnt(ii) = trackDeepIn(ii).deepCodeWideMetricCount;
        end
        ds5RefObsShadowNow = localUseDs5ReferenceObsModel(settings, currMeasNr) && deepModeState == 2 && ...
            ii <= numel(ds5TrueRefReadyState) && ds5TrueRefReadyState(ii) && ...
            settings.deepShadowDs5RefObsAutoActivate;
        fastDs5RefObsShadowNow = settings.deepFastMode && ...
            localGetSettingValue(settings, 'deepShadowDs5FastRefObsTrackBypass', 1) ~= 0 && ...
            ds5RefObsShadowNow;
        ds5RefObsRawLimitedNow = settings.deepFastMode && ds5RefObsShadowNow && ~fastDs5RefObsShadowNow && ...
            (max(1, round(localGetSettingValue(settings, 'deepShadowDs5RefObsTrackRawStrideEpochs', 1))) > 1 || ...
             isfinite(localGetSettingValue(settings, 'deepShadowDs5RefObsTrackRawBurstMs', inf)));
        ds5RefObsRawTrackNow = true;
        ds5RefObsRawMaxSteps = inf;
        if ds5RefObsRawLimitedNow
            rawStrideEpochs = max(1, round(localGetSettingValue(settings, 'deepShadowDs5RefObsTrackRawStrideEpochs', 1)));
            rawWarmupEpochs = max(0, round(localGetSettingValue(settings, 'deepShadowDs5RefObsTrackRawWarmupEpochs', 0)));
            rawBurstMs = localGetSettingValue(settings, 'deepShadowDs5RefObsTrackRawBurstMs', inf);
            if isfinite(rawBurstMs)
                ds5RefObsRawMaxSteps = max(0, round(rawBurstMs));
            end
            refObsStartEpoch = localDs5TimedStartEpoch(settings, ...
                localGetSettingValue(settings, 'deepShadowDs5RefObsModelStartSec', 92.0));
            refObsAgeEpochs = max(1, currMeasNr - refObsStartEpoch + 1);
            hasRecentPrompt = isfinite(localGetTrackField(trackShadow(ii), 'deepPromptI', nan));
            ds5RefObsRawTrackNow = refObsAgeEpochs <= rawWarmupEpochs || ...
                mod(refObsAgeEpochs - 1, rawStrideEpochs) == 0 || ~hasRecentPrompt;
        end
        if ds5RefObsShadowNow
            if fastDs5RefObsShadowNow || (~shadowTrackPending(ii) && ~shadowTrackActive(ii))
                trackShadow(ii) = localSyncTrackState(trackShadow(ii), trackDeepIn(ii));
                shadowTrackActive(ii) = true;
                shadowTrackPending(ii) = false;
                shadowTrackPendingAge(ii) = 0;
                shadowTrackValidateCnt(ii) = 0;
                shadowTrackValidateHist(ii, :) = false;
            end
            navResults.shadowDs5RefObsTrackDriven(ii, currMeasNr) = shadowTrackActive(ii) || shadowTrackPending(ii);
        end
        if shadowTrackPending(ii) || shadowTrackActive(ii)
            settingsShadow = settings;
            settingsShadow.deepShadowTargetHz = nan;
            settingsShadow.deepShadowReacqEnableNow = 0;
            settingsShadow.deepShadowCodeRefEnableNow = 0;
            settingsShadow.deepShadowCodeRefChips = nan;
            if localUseDs5ReferenceObsModel(settings, currMeasNr) && ii <= numel(ds5TrueRefReadyState) && ds5TrueRefReadyState(ii)
                settingsShadow.deepShadowTargetHz = ds5TrueRefHzState(ii);
                settingsShadow.deepShadowReacqEnableNow = 1;
                settingsShadow.deepShadowCodeRefEnableNow = isfinite(ds5TrueRefCodeState(ii));
                settingsShadow.deepShadowCodeRefChips = ds5TrueRefCodeState(ii);
            end
            if ~isfield(trackShadow(ii), 'deepShadowInitHold') || ~isfinite(trackShadow(ii).deepShadowInitHold)
                trackShadow(ii).deepShadowInitHold = 0;
            end
            holdActiveNow = trackShadow(ii).deepShadowInitHold > 0;
            if shadowTrackPending(ii) && ~holdActiveNow
                shadowTrackPendingAge(ii) = shadowTrackPendingAge(ii) + 1;
            end
            if holdActiveNow
                settingsShadow.deepBypassDllInSpoof = 1;
                settingsShadow.deepBypassPllInSpoof = 1;
                trackShadow(ii).deepShadowInitHold = trackShadow(ii).deepShadowInitHold - 1;
            end
            rawStepsThisEpoch = 0;
            if fastDs5RefObsShadowNow || ~ds5RefObsRawTrackNow
                trackShadow(ii) = localApplyFastDs5RefObsShadowState( ...
                    trackShadow(ii), trackDeepIn(ii), ds5TrueRefHzState(ii), ds5TrueRefCodeState(ii), settings);
                navResults.shadowDs5RefObsTrackCoasted(ii, currMeasNr) = ~fastDs5RefObsShadowNow;
            else
                diagLocalTic = tic;
                if settings.deepShadowContinuousTrackEnable
                    trackShadowLocal = trackShadow(ii);
                    shadowStep = 1;
                    while trackShadowLocal.recvTime < positioningTime && shadowStep <= ds5RefObsRawMaxSteps
                        [trackShadowLocal, ~, ~] = perChannelTrackOnce_DeepIn(trackShadowLocal, settingsShadow, fid, ...
                            oldAidFreq(ii), shadowStep * (aidFreq(ii) - oldAidFreq(ii)));
                        diagTrackCalls = diagTrackCalls + 1;
                        rawStepsThisEpoch = rawStepsThisEpoch + 1;
                        shadowStep = shadowStep + 1;
                    end
                    if ds5RefObsRawLimitedNow && trackShadowLocal.recvTime < positioningTime
                        trackShadowLocal = localApplyFastDs5RefObsShadowState( ...
                            trackShadowLocal, trackDeepIn(ii), ds5TrueRefHzState(ii), ds5TrueRefCodeState(ii), settings);
                        navResults.shadowDs5RefObsTrackCoasted(ii, currMeasNr) = true;
                    end
                    trackShadow(ii) = trackShadowLocal;
                else
                    [trackShadow(ii), ~, ~] = perChannelTrackOnce_DeepIn(trackShadow(ii), settingsShadow, fid, oldAidFreq(ii), aidFreq(ii) - oldAidFreq(ii));
                    diagTrackCalls = diagTrackCalls + 1;
                    rawStepsThisEpoch = 1;
                end
                diagTrackTimeSec = diagTrackTimeSec + toc(diagLocalTic);
            end
            navResults.shadowDs5RefObsTrackRawSampled(ii, currMeasNr) = rawStepsThisEpoch > 0;
            navResults.shadowDs5RefObsTrackRawSteps(ii, currMeasNr) = rawStepsThisEpoch;
            navResults.shadowRawTrackPending(ii, currMeasNr) = shadowTrackPending(ii);
            navResults.shadowRawTrackPendingAge(ii, currMeasNr) = shadowTrackPendingAge(ii);
            navResults.shadowRawTrackActive(ii, currMeasNr) = shadowTrackActive(ii);
            navResults.shadowRawFailExcludeActive(ii, currMeasNr) = shadowRawFailExcludeValid(ii);
            navResults.shadowRawFailExcludeAge(ii, currMeasNr) = shadowRawFailExcludeAge(ii);
            navResults.shadowRawTrackCarrFreqHz(ii, currMeasNr) = trackShadow(ii).carrFreq - settings.IF;
            navResults.shadowRawTrackCodeFreqHz(ii, currMeasNr) = trackShadow(ii).codeFreq;
            navResults.shadowRawTrackCarrFreqStartHz(ii, currMeasNr) = localGetTrackField(trackShadow(ii), 'deepCarrFreqStartHz', trackShadow(ii).carrFreq - settings.IF);
            navResults.shadowRawTrackCarrCmdHz(ii, currMeasNr) = localGetTrackField(trackShadow(ii), 'deepCarrCmdHz', trackShadow(ii).carrFreq - settings.IF);
            navResults.shadowRawTrackCarrBaseCmdHz(ii, currMeasNr) = localGetTrackField(trackShadow(ii), 'deepCarrBaseCmdHz', nan);
            navResults.shadowRawTrackCarrShadowCmdHz(ii, currMeasNr) = localGetTrackField(trackShadow(ii), 'deepCarrShadowCmdHz', nan);
            navResults.shadowRawTrackCarrAidFreqHz(ii, currMeasNr) = localGetTrackField(trackShadow(ii), 'deepCarrAidFreqHz', nan);
            navResults.shadowRawTrackCarrShadowTargetHz(ii, currMeasNr) = localGetTrackField(trackShadow(ii), 'deepCarrShadowTargetHz', nan);
            navResults.shadowRawTrackCarrShadowPullHz(ii, currMeasNr) = localGetTrackField(trackShadow(ii), 'deepCarrShadowPullHz', nan);
            navResults.shadowRawTrackCarrShadowWeight(ii, currMeasNr) = localGetTrackField(trackShadow(ii), 'deepCarrShadowWeight', nan);
            navResults.shadowRawTrackCarrForceShadow(ii, currMeasNr) = logical(localGetTrackField(trackShadow(ii), 'deepCarrForceShadow', false));
            navResults.shadowRawTrackCarrPllBypass(ii, currMeasNr) = logical(localGetTrackField(trackShadow(ii), 'deepCarrPllBypass', false));
            navResults.shadowRawTrackCarrOldNcoHz(ii, currMeasNr) = localGetTrackField(trackShadow(ii), 'deepCarrOldNcoHz', nan);
            navResults.shadowRawTrackCarrNcoHz(ii, currMeasNr) = localGetTrackField(trackShadow(ii), 'deepCarrNcoHz', nan);
            navResults.shadowRawTrackCarrNcoStepHz(ii, currMeasNr) = localGetTrackField(trackShadow(ii), 'deepCarrNcoStepHz', nan);
            navResults.shadowRawTrackCarrOldErrorCycles(ii, currMeasNr) = localGetTrackField(trackShadow(ii), 'deepCarrOldErrorCycles', nan);
            navResults.shadowRawTrackCarrErrorRawCycles(ii, currMeasNr) = localGetTrackField(trackShadow(ii), 'deepCarrErrorRawCycles', nan);
            navResults.shadowRawTrackCarrErrorScaledCycles(ii, currMeasNr) = localGetTrackField(trackShadow(ii), 'deepCarrErrorScaledCycles', nan);
            navResults.shadowRawTrackPromptIP(ii, currMeasNr) = localGetTrackField(trackShadow(ii), 'deepPromptI', nan);
            navResults.shadowRawTrackPromptQP(ii, currMeasNr) = localGetTrackField(trackShadow(ii), 'deepPromptQ', nan);
            navResults.shadowRawTrackEarlyI(ii, currMeasNr) = localGetTrackField(trackShadow(ii), 'deepEarlyI', nan);
            navResults.shadowRawTrackEarlyQ(ii, currMeasNr) = localGetTrackField(trackShadow(ii), 'deepEarlyQ', nan);
            navResults.shadowRawTrackLateI(ii, currMeasNr) = localGetTrackField(trackShadow(ii), 'deepLateI', nan);
            navResults.shadowRawTrackLateQ(ii, currMeasNr) = localGetTrackField(trackShadow(ii), 'deepLateQ', nan);
            navResults.shadowRawTrackDllDiscrRaw(ii, currMeasNr) = localGetTrackField(trackShadow(ii), 'deepDllDiscrRaw', nan);
            navResults.shadowRawTrackDllDiscr(ii, currMeasNr) = localGetTrackField(trackShadow(ii), 'deepDllDiscr', nan);
            navResults.shadowRawTrackCodeError(ii, currMeasNr) = navResults.shadowRawTrackDllDiscr(ii, currMeasNr);
            navResults.shadowRawTrackCodeRefAppliedChips(ii, currMeasNr) = localGetTrackField(trackShadow(ii), 'deepShadowCodeRefAppliedChips', nan);
            navResults.shadowRawTrackCodeRefNcoPullHz(ii, currMeasNr) = localGetTrackField(trackShadow(ii), 'deepShadowCodeRefNcoPullHz', nan);
            navResults.shadowRawTrackCodeRefNcoStepHz(ii, currMeasNr) = localGetTrackField(trackShadow(ii), 'deepShadowCodeRefNcoStepHz', nan);
            navResults.shadowRawTrackCodeOffsetChips(ii, currMeasNr) = trackShadow(ii).remCodePhase;
            if isfield(trackShadow(ii), 'deepAccumCodeChips') && isfinite(trackShadow(ii).deepAccumCodeChips)
                navResults.shadowRawTrackCodePhaseChips(ii, currMeasNr) = trackShadow(ii).deepAccumCodeChips;
            else
                navResults.shadowRawTrackCodePhaseChips(ii, currMeasNr) = ...
                    trackShadow(ii).numOfCoInt * settings.codeLength + trackShadow(ii).remCodePhase;
            end
            navResults.shadowRawTrackCarrPhaseRad(ii, currMeasNr) = trackShadow(ii).remCarrPhase;
            if isfield(trackShadow(ii), 'deepAccumCarrierCycles') && isfinite(trackShadow(ii).deepAccumCarrierCycles)
                navResults.shadowRawTrackCarrierCycles(ii, currMeasNr) = trackShadow(ii).deepAccumCarrierCycles;
            else
                navResults.shadowRawTrackCarrierCycles(ii, currMeasNr) = ...
                    trackShadow(ii).numOfCoInt * ((trackShadow(ii).carrFreq - settings.IF) / 1000) + ...
                    trackShadow(ii).remCarrPhase / (2*pi);
            end
            if currMeasNr > 1 && navResults.shadowRawTrackActive(ii, currMeasNr-1)
                prevCarr = navResults.shadowRawTrackCarrierCycles(ii, currMeasNr-1);
                prevCode = navResults.shadowRawTrackCodePhaseChips(ii, currMeasNr-1);
                if isfinite(prevCarr)
                    navResults.shadowRawTrackCarrierDeltaCycles(ii, currMeasNr) = ...
                        navResults.shadowRawTrackCarrierCycles(ii, currMeasNr) - prevCarr;
                end
                if isfinite(prevCode)
                    navResults.shadowRawTrackCodeDeltaChips(ii, currMeasNr) = ...
                        navResults.shadowRawTrackCodePhaseChips(ii, currMeasNr) - prevCode;
                end
                navResults.shadowRawTrackLockRun(ii, currMeasNr) = navResults.shadowRawTrackLockRun(ii, currMeasNr-1) + 1;
                predCarrierDelta = (trackShadow(ii).carrFreq - settings.IF) * settings.navSolPeriod / 1000;
                carrierJump = abs(navResults.shadowRawTrackCarrierDeltaCycles(ii, currMeasNr) - predCarrierDelta);
                codeJump = abs(navResults.shadowRawTrackCodeDeltaChips(ii, currMeasNr) - ...
                    settings.codeLength * settings.navSolPeriod);
                navResults.shadowRawTrackCycleSlip(ii, currMeasNr) = ...
                    (isfinite(carrierJump) && carrierJump > 5.0) || ...
                    (isfinite(codeJump) && codeJump > 2.0 * settings.codeLength);
            else
                navResults.shadowRawTrackLockRun(ii, currMeasNr) = 1;
                navResults.shadowRawTrackCycleSlip(ii, currMeasNr) = false;
            end
            if shadowMainFollowActive(ii) && shadowMainHoldCnt(ii) > 0
                trackDeepIn(ii) = localSyncTrackState(trackDeepIn(ii), trackShadow(ii));
            end
        end
    end
    if ~lightNormalMode
        navResults.codeReacqUsedNum(1, currMeasNr) = sum(codeUsed);
        navResults.codeReacqOffsetMedChips(1, currMeasNr) = median(codeOffset(isfinite(codeOffset)));
        navResults.codeReacqAppliedMedChips(1, currMeasNr) = median(codeApplied(isfinite(codeApplied)));
        navResults.codeReacqPeakRatioMed(1, currMeasNr) = median(codePeakRatio(isfinite(codePeakRatio)));
        navResults.codePhaseCorrMedChips(1, currMeasNr) = median(codePhaseCorr(isfinite(codePhaseCorr)));
        navResults.codePhaseCorrRmsM(1, currMeasNr) = sqrt(mean((-codePhaseCorr(isfinite(codePhaseCorr)) / 1023 * 1e-3 * settings.c).^2));
        navResults.codeWideUsedNum(1, currMeasNr) = sum(codeWideUsed);
        navResults.codeWideOffsetMedChips(1, currMeasNr) = median(codeWideOffset(isfinite(codeWideOffset)));
        navResults.codeWidePeakRatioMed(1, currMeasNr) = median(codeWidePeakRatio(isfinite(codeWidePeakRatio)));
        navResults.codeWideStableMax(1, currMeasNr) = max(codeWideStableCnt);
        navResults.codeWideAccumMax(1, currMeasNr) = max(codeWideAccumCnt);
    end

    if settings.deepShadowAcqEnable && deepModeState == 2 && ...
            mod(currMeasNr, max(1, settings.deepShadowAcqIntervalEpochs)) == 0
        acqValidNum = 0;
        for ii = 1:numActChnList
            shadowAcq = perChannelShadowAcquire_DeepIn(trackDeepIn(ii), settings, fid, trackDeepIn(ii).carrFreq);
            if shadowAcq.valid
                acqValidNum = acqValidNum + 1;
                navResults.shadowAcqOffsetChips(ii, currMeasNr) = shadowAcq.bestOffsetChips;
                navResults.shadowAcqOffsetSamples(ii, currMeasNr) = shadowAcq.bestOffsetSamples;
                navResults.shadowAcqPeakRatio(ii, currMeasNr) = shadowAcq.peakRatio;
                navResults.shadowAcqZeroRatio(ii, currMeasNr) = shadowAcq.zeroRatio;
                navResults.shadowAcqBestMetric(ii, currMeasNr) = shadowAcq.bestMetric;
                navResults.shadowAcqRawValid(ii, currMeasNr) = shadowAcq.rawValid;
                navResults.shadowAcqRawOffsetChips(ii, currMeasNr) = shadowAcq.rawBestOffsetChips;
                navResults.shadowAcqRawPeakRatio(ii, currMeasNr) = shadowAcq.rawPeakRatio;
                navResults.shadowAcqRawZeroRatio(ii, currMeasNr) = shadowAcq.rawZeroRatio;
                navResults.shadowAcqQualified(ii, currMeasNr) = shadowAcq.qualified;
                navResults.shadowAcqPeakWidthChips(ii, currMeasNr) = shadowAcq.peakWidthChips;
                navResults.shadowAcqLocalDropRatio(ii, currMeasNr) = shadowAcq.localDropRatio;
                navResults.shadowAcqNearZeroPenalty(ii, currMeasNr) = shadowAcq.nearZeroPenalty;
                navResults.shadowAcqNearZeroTrap(ii, currMeasNr) = shadowAcq.isNearZeroTrap;
                if numel(shadowAcq.topKOffsetsChips) >= 1
                    navResults.shadowAcqTop1OffsetChips(ii, currMeasNr) = shadowAcq.topKOffsetsChips(1);
                    navResults.shadowAcqTop1Score(ii, currMeasNr) = shadowAcq.topKScores(1);
                end
                if numel(shadowAcq.topKOffsetsChips) >= 2
                    navResults.shadowAcqTop2OffsetChips(ii, currMeasNr) = shadowAcq.topKOffsetsChips(2);
                    navResults.shadowAcqTop2Score(ii, currMeasNr) = shadowAcq.topKScores(2);
                end
                if numel(shadowAcq.topKOffsetsChips) >= 3
                    navResults.shadowAcqTop3OffsetChips(ii, currMeasNr) = shadowAcq.topKOffsetsChips(3);
                    navResults.shadowAcqTop3Score(ii, currMeasNr) = shadowAcq.topKScores(3);
                end

                isBoundaryPeak = abs(shadowAcq.bestOffsetChips) >= ...
                    (settings.deepShadowAcqHalfChips - settings.deepShadowAcqBoundaryGuardChips);
                isStrongPeak = shadowAcq.peakRatio >= settings.deepShadowAcqMinPeakRatio && ...
                               shadowAcq.zeroRatio >= settings.deepShadowAcqMinZeroRatio;
                isWellShapedPeak = true;
                if isfield(shadowAcq, 'sharpRatio') && isfinite(shadowAcq.sharpRatio)
                    isWellShapedPeak = isWellShapedPeak && ...
                        shadowAcq.sharpRatio >= settings.deepShadowAcqMinSharpRatio;
                end
                if isfield(shadowAcq, 'centerPrior') && isfinite(shadowAcq.centerPrior)
                    isWellShapedPeak = isWellShapedPeak && ...
                        shadowAcq.centerPrior >= settings.deepShadowAcqMinCenterPrior;
                end
                if isfield(shadowAcq, 'symmetryRatio') && isfinite(shadowAcq.symmetryRatio)
                    isWellShapedPeak = isWellShapedPeak && ...
                        shadowAcq.symmetryRatio >= settings.deepShadowAcqMinSymmetryRatio;
                end
                if isfield(shadowAcq, 'isNearZeroTrap') && shadowAcq.isNearZeroTrap
                    isWellShapedPeak = false;
                end

                if isStrongPeak && isWellShapedPeak && ~isBoundaryPeak
                    shadowAcqQualifiedMask(ii) = true;
                    candOffsets = shadowAcq.topKOffsetsChips;
                    candScores = shadowAcq.topKScores;
                    validCand = isfinite(candOffsets) & isfinite(candScores);
                    candOffsets = candOffsets(validCand);
                    candScores = candScores(validCand);
                    if ~isempty(candOffsets)
                        shadowAcqTrackScore(ii, :) = settings.deepShadowTrackScoreDecay * shadowAcqTrackScore(ii, :);
                        matchedTrack = false(1, settings.deepShadowTrackNum);
                        for cc = 1:numel(candOffsets)
                            bestTrackIdx = 0;
                            bestTrackDist = inf;
                            for tt = 1:settings.deepShadowTrackNum
                                if matchedTrack(tt)
                                    continue;
                                end
                                if isfinite(shadowAcqTrackOffset(ii, tt))
                                    dOff = abs(candOffsets(cc) - shadowAcqTrackOffset(ii, tt));
                                else
                                    dOff = inf;
                                end
                                if dOff <= settings.deepShadowTrackAssocTolChips && dOff < bestTrackDist
                                    bestTrackDist = dOff;
                                    bestTrackIdx = tt;
                                end
                            end
                            if bestTrackIdx == 0
                                emptyIdx = find(~isfinite(shadowAcqTrackOffset(ii, :)), 1, 'first');
                                if isempty(emptyIdx)
                                    [~, emptyIdx] = min(shadowAcqTrackScore(ii, :));
                                end
                                bestTrackIdx = emptyIdx;
                                bestTrackDist = settings.deepShadowTrackAssocTolChips;
                            end
                            matchedTrack(bestTrackIdx) = true;
                            gain = settings.deepShadowTrackHitGain * ...
                                   max(0.1, min(2.0, candScores(cc) / max(shadowAcq.topKScores(1), eps)));
                            continuity = 1.0;
                            if isfinite(shadowAcqTrackOffset(ii, bestTrackIdx))
                                continuity = max(0.2, 1 - bestTrackDist / max(settings.deepShadowTrackAssocTolChips, eps));
                            end
                            shadowAcqTrackScore(ii, bestTrackIdx) = shadowAcqTrackScore(ii, bestTrackIdx) + gain * continuity;
                            if isfinite(shadowAcqTrackOffset(ii, bestTrackIdx))
                                shadowAcqTrackOffset(ii, bestTrackIdx) = 0.7 * shadowAcqTrackOffset(ii, bestTrackIdx) + ...
                                                                         0.3 * candOffsets(cc);
                            else
                                shadowAcqTrackOffset(ii, bestTrackIdx) = candOffsets(cc);
                            end
                            shadowAcqTrackHits(ii, bestTrackIdx) = shadowAcqTrackHits(ii, bestTrackIdx) + 1;
                            shadowAcqTrackMiss(ii, bestTrackIdx) = 0;
                        end
                        for tt = 1:settings.deepShadowTrackNum
                            if ~matchedTrack(tt)
                                shadowAcqTrackScore(ii, tt) = settings.deepShadowTrackMissDecay * shadowAcqTrackScore(ii, tt);
                                shadowAcqTrackMiss(ii, tt) = shadowAcqTrackMiss(ii, tt) + 1;
                                if shadowAcqTrackMiss(ii, tt) >= 3 && shadowAcqTrackScore(ii, tt) < 0.5
                                    shadowAcqTrackOffset(ii, tt) = nan;
                                    shadowAcqTrackHits(ii, tt) = 0;
                                    shadowAcqTrackMiss(ii, tt) = 0;
                                    shadowAcqTrackScore(ii, tt) = 0;
                                end
                            end
                        end
                    end
                else
                    shadowAcqTrackScore(ii, :) = settings.deepShadowTrackMissDecay * shadowAcqTrackScore(ii, :);
                    shadowAcqTrackMiss(ii, :) = shadowAcqTrackMiss(ii, :) + 1;
                end
            else
                shadowAcqTrackScore(ii, :) = settings.deepShadowTrackMissDecay * shadowAcqTrackScore(ii, :);
                shadowAcqTrackMiss(ii, :) = shadowAcqTrackMiss(ii, :) + 1;
            end

            [trackBestScore, bestTrackIdx] = max(shadowAcqTrackScore(ii, :));
            tmpTrack = shadowAcqTrackScore(ii, :);
            tmpTrack(bestTrackIdx) = -inf;
            trackSecondScore = max(tmpTrack);
            if ~isfinite(trackSecondScore)
                trackSecondScore = 0;
            end
            shadowAcqVotePeak(ii) = trackBestScore;
            shadowAcqVoteSecond(ii) = trackSecondScore;
            if isfinite(bestTrackIdx) && bestTrackIdx >= 1 && isfinite(shadowAcqTrackOffset(ii, bestTrackIdx))
                shadowAcqCandidateOffset(ii) = shadowAcqTrackOffset(ii, bestTrackIdx);
                shadowAcqStableCnt(ii) = trackBestScore;
            else
                shadowAcqCandidateOffset(ii) = nan;
                shadowAcqStableCnt(ii) = 0;
            end
        end
        bestTrackHits = zeros(numActChnList, 1);
        for ii = 1:numActChnList
            [~, bestTrackIdx] = max(shadowAcqTrackScore(ii, :));
            if ~isempty(bestTrackIdx) && isfinite(bestTrackIdx) && bestTrackIdx >= 1
                bestTrackHits(ii) = shadowAcqTrackHits(ii, bestTrackIdx);
            end
        end
        readyNow = (shadowAcqVotePeak >= settings.deepShadowTrackReadyScore) & ...
                   ((shadowAcqVotePeak - shadowAcqVoteSecond) >= settings.deepShadowTrackReadyMargin) & ...
                   (bestTrackHits >= settings.deepShadowTrackReadyHits);
        shadowAcqReadyHoldCnt(readyNow) = settings.deepShadowAcqVoteReadyHold;
        shadowAcqReadyHoldCnt(~readyNow) = max(0, shadowAcqReadyHoldCnt(~readyNow) - 1);
        shadowAcqReadyMask = readyNow | (shadowAcqReadyHoldCnt > 0);
        navResults.shadowAcqTrackBestHits(1:numActChnList, currMeasNr) = bestTrackHits;
        navResults.shadowAcqValidNum(1, currMeasNr) = acqValidNum;
    end
    if ~lightNormalMode
        navResults.shadowAcqStableCnt(1:numActChnList, currMeasNr) = shadowAcqStableCnt;
        navResults.shadowAcqCandidateOffsetChips(1:numActChnList, currMeasNr) = shadowAcqCandidateOffset;
        navResults.shadowAcqVotePeak(1:numActChnList, currMeasNr) = shadowAcqVotePeak;
        navResults.shadowAcqVoteSecond(1:numActChnList, currMeasNr) = shadowAcqVoteSecond;
        navResults.shadowAcqReadyHoldCnt(1:numActChnList, currMeasNr) = shadowAcqReadyHoldCnt;
        navResults.shadowAcqReady(1:numActChnList, currMeasNr) = shadowAcqReadyMask;
        navResults.shadowAcqReadySatNum(1, currMeasNr) = sum(shadowAcqReadyMask);
        navResults.shadowCodeReadyForRecovery(1:numActChnList, currMeasNr) = shadowAcqReadyMask & shadowAcqQualifiedMask;
        navResults.shadowCodeReadySatNum(1, currMeasNr) = sum(shadowAcqReadyMask & shadowAcqQualifiedMask);
    end

    if settings.deepShadowRawReacqEnable && deepModeState == 2 && ...
            mod(currMeasNr, max(1, settings.deepShadowRawReacqIntervalEpochs)) == 0
        for ii = 1:numActChnList
            rawCandidateReuseUsed = false;
            bestClusterScorePrev = 0;
            secondClusterScorePrev = 0;
            bestClusterHitsPrev = 0;
            if any(isfinite(shadowRawClusterOffset(ii, :)))
                [bestClusterScorePrev, bestClusterIdxPrev] = max(shadowRawClusterScore(ii, :));
                tmpClusterScorePrev = shadowRawClusterScore(ii, :);
                if ~isempty(bestClusterIdxPrev) && bestClusterIdxPrev >= 1
                    bestClusterHitsPrev = shadowRawClusterHits(ii, bestClusterIdxPrev);
                    tmpClusterScorePrev(bestClusterIdxPrev) = -inf;
                end
                secondClusterScorePrev = max(tmpClusterScorePrev);
                if ~isfinite(secondClusterScorePrev)
                    secondClusterScorePrev = 0;
                end
            end
            clusterReadyPrev = (bestClusterScorePrev >= settings.deepShadowRawClusterReadyScore) && ...
                ((bestClusterScorePrev - secondClusterScorePrev) >= settings.deepShadowRawClusterReadyMargin) && ...
                (bestClusterHitsPrev >= settings.deepShadowRawClusterReadyHits);
            hasReusableCand = shadowRawCandValid(ii) && ...
                (shadowRawCandAge(ii) <= settings.deepShadowRawCandReuseMaxEpochs);
            needFullRaw = ~shadowTrackActive(ii) && ~shadowTrackPending(ii) && ...
                (~hasReusableCand || ~clusterReadyPrev);
            shadowRaw = localInitShadowRawResult();
            if needFullRaw
                diagLocalTic = tic;
                settingsShadowRaw = settings;
                centerCarrHzRaw = trackDeepIn(ii).carrFreq;
                settingsShadowRaw.deepShadowRawReacqCodeCenterChips = 0.0;
                if localUseDs5TruePeak(settings, currMeasNr) && ii <= numel(ds5TrueRefReadyState) && ds5TrueRefReadyState(ii)
                    if isfinite(ds5TrueRefHzState(ii))
                        centerCarrHzRaw = settings.IF + ds5TrueRefHzState(ii);
                    end
                    if isfinite(ds5TrueRefCodeState(ii))
                        settingsShadowRaw.deepShadowRawReacqCodeCenterChips = ds5TrueRefCodeState(ii);
                    end
                    settingsShadowRaw.deepShadowRawReacqFreqHalfHz = min(settingsShadowRaw.deepShadowRawReacqFreqHalfHz, ...
                        localGetSettingValue(settings, 'deepShadowDs5TruePeakFreqWindowHz', settingsShadowRaw.deepShadowRawReacqFreqHalfHz));
                    settingsShadowRaw.deepShadowRawReacqFineFreqHalfHz = min(settingsShadowRaw.deepShadowRawReacqFineFreqHalfHz, ...
                        localGetSettingValue(settings, 'deepShadowDs5TruePeakFreqWindowHz', settingsShadowRaw.deepShadowRawReacqFineFreqHalfHz));
                    settingsShadowRaw.deepShadowRawReacqHalfChips = min(settingsShadowRaw.deepShadowRawReacqHalfChips, ...
                        localGetSettingValue(settings, 'deepShadowDs5TruePeakCodeWindowChips', settingsShadowRaw.deepShadowRawReacqHalfChips));
                    settingsShadowRaw.deepShadowRawReacqFineHalfChips = min(settingsShadowRaw.deepShadowRawReacqFineHalfChips, ...
                        localGetSettingValue(settings, 'deepShadowDs5TruePeakCodeWindowChips', settingsShadowRaw.deepShadowRawReacqFineHalfChips));
                end
                shadowRaw = perChannelShadowRawReacq_DeepIn(trackDeepIn(ii), settingsShadowRaw, fid, centerCarrHzRaw);
                diagRawReacqTimeSec = diagRawReacqTimeSec + toc(diagLocalTic);
                diagRawReacqCalls = diagRawReacqCalls + 1;
            end
            if shadowRaw.valid
                navResults.shadowRawReacqValid(ii, currMeasNr) = true;
                navResults.shadowRawReacqOffsetChips(ii, currMeasNr) = shadowRaw.bestOffsetChips;
                navResults.shadowRawReacqFreqHz(ii, currMeasNr) = shadowRaw.bestFreqHz;
                navResults.shadowRawReacqPeakRatio(ii, currMeasNr) = shadowRaw.peakRatio;
                navResults.shadowRawReacqZeroRatio(ii, currMeasNr) = shadowRaw.zeroRatio;
                if numel(shadowRaw.topKOffsetsChips) >= 1
                    navResults.shadowRawReacqTop1OffsetChips(ii, currMeasNr) = shadowRaw.topKOffsetsChips(1);
                    navResults.shadowRawReacqTop1FreqHz(ii, currMeasNr) = shadowRaw.topKFreqHz(1);
                    navResults.shadowRawReacqTop1Metric(ii, currMeasNr) = shadowRaw.topKMetrics(1);
                end
                if numel(shadowRaw.topKOffsetsChips) >= 2
                    navResults.shadowRawReacqTop2OffsetChips(ii, currMeasNr) = shadowRaw.topKOffsetsChips(2);
                    navResults.shadowRawReacqTop2FreqHz(ii, currMeasNr) = shadowRaw.topKFreqHz(2);
                    navResults.shadowRawReacqTop2Metric(ii, currMeasNr) = shadowRaw.topKMetrics(2);
                end
                if numel(shadowRaw.topKOffsetsChips) >= 3
                    navResults.shadowRawReacqTop3OffsetChips(ii, currMeasNr) = shadowRaw.topKOffsetsChips(3);
                    navResults.shadowRawReacqTop3FreqHz(ii, currMeasNr) = shadowRaw.topKFreqHz(3);
                    navResults.shadowRawReacqTop3Metric(ii, currMeasNr) = shadowRaw.topKMetrics(3);
                end
                weakVoteReady = settings.deepShadowRawTrackEnable && ...
                    shadowRaw.peakRatio >= settings.deepShadowRawWeakVotePeakRatioMin && ...
                    shadowRaw.zeroRatio >= settings.deepShadowRawWeakVoteZeroRatioMin;
                exploreVoteReady = settings.deepShadowRawTrackEnable && ...
                    shadowRaw.peakRatio >= settings.deepShadowRawExplorePeakRatioMin && ...
                    shadowRaw.zeroRatio >= settings.deepShadowRawExploreZeroRatioMin;
                if weakVoteReady
                    kkUse = min(shadowRawCandK, numel(shadowRaw.topKOffsetsChips));
                    shadowRawCandOffsetChips(ii, :) = nan;
                    shadowRawCandOffsetSamples(ii, :) = nan;
                    shadowRawCandFreqHz(ii, :) = nan;
                    shadowRawCandMetric(ii, :) = nan;
                    shadowRawCandPeakRatio(ii, :) = nan;
                    for jj = 1:kkUse
                        offChip = shadowRaw.topKOffsetsChips(jj);
                        freqHz = shadowRaw.topKFreqHz(jj);
                        metricVal = shadowRaw.topKMetrics(jj);
                        if ~isfinite(offChip) || ~isfinite(freqHz) || ~isfinite(metricVal)
                            continue;
                        end
                        if localUseDs5TruePeak(settings, currMeasNr) && ii <= numel(ds5TrueRefReadyState) && ds5TrueRefReadyState(ii)
                            if ~localCandidateInDs5TrueWindow(freqHz - settings.IF, offChip, ds5TrueRefHzState(ii), ...
                                    ds5TrueRefCodeState(ii), settings)
                                continue;
                            end
                        end
                        shadowRawCandOffsetChips(ii, jj) = offChip;
                        shadowRawCandOffsetSamples(ii, jj) = offChip / max(trackDeepIn(ii).codeFreq / settings.samplingFreq, eps);
                        shadowRawCandFreqHz(ii, jj) = freqHz;
                        if exploreVoteReady
                            shadowRawCandMetric(ii, jj) = metricVal;
                        else
                            shadowRawCandMetric(ii, jj) = settings.deepShadowRawWeakVoteMetricScale * metricVal;
                        end
                        shadowRawCandPeakRatio(ii, jj) = shadowRaw.topKPeakRatios(jj);
                    end
                    [shadowRawClusterOffset(ii, :), shadowRawClusterFreqHz(ii, :), ...
                        shadowRawClusterScore(ii, :), shadowRawClusterHits(ii, :), ...
                        shadowRawClusterMiss(ii, :)] = ...
                        localUpdateShadowClusters(shadowRawClusterOffset(ii, :), ...
                        shadowRawClusterFreqHz(ii, :), shadowRawClusterScore(ii, :), ...
                        shadowRawClusterHits(ii, :), shadowRawClusterMiss(ii, :), ...
                        shadowRawCandOffsetChips(ii, :), shadowRawCandFreqHz(ii, :), ...
                        shadowRawCandMetric(ii, :), settings);
                    shadowRawCandUsedMask(ii, :) = false;
                    shadowRawCandSelectIdx(ii) = 0;
                    [bestClusterScore, bestClusterIdx] = max(shadowRawClusterScore(ii, :));
                    tmpClusterScore = shadowRawClusterScore(ii, :);
                    if ~isempty(bestClusterIdx) && bestClusterIdx >= 1
                        tmpClusterScore(bestClusterIdx) = -inf;
                    end
                    secondClusterScore = max(tmpClusterScore);
                    if ~isfinite(secondClusterScore)
                        secondClusterScore = 0;
                    end
                    if ~isempty(bestClusterIdx) && bestClusterIdx >= 1 && ...
                            isfinite(shadowRawClusterOffset(ii, bestClusterIdx))
                        navResults.shadowRawClusterBestOffset(ii, currMeasNr) = shadowRawClusterOffset(ii, bestClusterIdx);
                        navResults.shadowRawClusterBestFreqHz(ii, currMeasNr) = shadowRawClusterFreqHz(ii, bestClusterIdx);
                        navResults.shadowRawClusterBestScore(ii, currMeasNr) = bestClusterScore;
                        navResults.shadowRawClusterSecondScore(ii, currMeasNr) = secondClusterScore;
                        navResults.shadowRawClusterBestHits(ii, currMeasNr) = shadowRawClusterHits(ii, bestClusterIdx);
                    end
                    strictCandidateReady = shadowRaw.peakRatio >= settings.deepShadowRawTrackPeakRatioMin && ...
                        shadowRaw.zeroRatio >= settings.deepShadowRawTrackZeroRatioMin;
                    exploratoryClusterReady = ~isempty(bestClusterIdx) && bestClusterIdx >= 1 && ...
                        isfinite(shadowRawClusterOffset(ii, bestClusterIdx)) && ...
                        bestClusterScore >= settings.deepShadowRawExploreClusterScore && ...
                        shadowRawClusterHits(ii, bestClusterIdx) >= settings.deepShadowRawExploreClusterHits && ...
                        (bestClusterScore - secondClusterScore) >= settings.deepShadowRawExploreClusterMargin;
                    persistentClusterReady = ~isempty(bestClusterIdx) && bestClusterIdx >= 1 && ...
                        isfinite(shadowRawClusterOffset(ii, bestClusterIdx)) && ...
                        bestClusterScore >= settings.deepShadowRawClusterInitScore && ...
                        shadowRawClusterHits(ii, bestClusterIdx) >= settings.deepShadowRawClusterInitHits && ...
                        (bestClusterScore - secondClusterScore) >= settings.deepShadowRawClusterInitMargin;
                    shadowRawCandValid(ii) = strictCandidateReady || exploratoryClusterReady || persistentClusterReady;
                    if shadowRawCandValid(ii)
                        shadowRawCandAge(ii) = 0;
                    else
                        shadowRawCandAge(ii) = min(shadowRawCandAge(ii), settings.deepShadowRawCandReuseMaxEpochs + 1);
                    end
                    if ~shadowTrackPending(ii) && ~shadowTrackActive(ii) && shadowRawRelayCooldownCnt(ii) <= 0
                        [candIdx, candPickDiag] = localPickShadowCandidateByCluster(shadowRawCandOffsetChips(ii, :), ...
                            shadowRawCandFreqHz(ii, :), shadowRawCandMetric(ii, :), ...
                            shadowRawCandUsedMask(ii, :), shadowRawClusterOffset(ii, :), ...
                            shadowRawClusterFreqHz(ii, :), shadowRawClusterScore(ii, :), ...
                            shadowRawClusterHits(ii, :), settings, shadowRawFailExcludeValid(ii), ...
                            shadowRawFailExcludeOffset(ii), shadowRawFailExcludeFreqHz(ii));
                        shadowRawCandSelectIdx(ii) = candIdx;
                        navResults.shadowRawCandMetricGap(ii, currMeasNr) = candPickDiag.metricGap;
                        navResults.shadowRawCandClusterOnlyUsed(ii, currMeasNr) = candPickDiag.clusterOnlyUsed;
                        navResults.shadowRawCandSelectedClusterSupport(ii, currMeasNr) = candPickDiag.selectedClusterSupport;
                        navResults.shadowRawCandSelectedMetricNorm(ii, currMeasNr) = candPickDiag.selectedMetricNorm;
                        navResults.shadowRawFailExcludeApplied(ii, currMeasNr) = candPickDiag.failExcludeApplied;
                        failFallbackBlocked = candPickDiag.failExcludeApplied && candIdx > 0 && ...
                            (~isfinite(candPickDiag.selectedClusterSupport) || ...
                            candPickDiag.selectedClusterSupport < settings.deepShadowRawFailFallbackMinClusterSupport);
                        navResults.shadowRawFailFallbackBlocked(ii, currMeasNr) = failFallbackBlocked;
                        if shadowRawCandValid(ii) && ~failFallbackBlocked && (candIdx > 0 || persistentClusterReady)
                            useClusterCenter = false;
                            startedFromDirectCluster = false;
                            shadowRawFailExcludeValid(ii) = false;
                            shadowRawFailExcludeOffset(ii) = nan;
                            shadowRawFailExcludeFreqHz(ii) = nan;
                            shadowRawFailExcludeAge(ii) = inf;
                            if candIdx > 0
                                shadowRawCandUsedMask(ii, candIdx) = true;
                            [useClusterCenter, startOffsetChips, startOffsetSamples, startFreqHz, initStartDiag] = ...
                                localBuildShadowStartPoint(trackDeepIn(ii), ...
                                shadowRawCandOffsetChips(ii, candIdx), shadowRawCandOffsetSamples(ii, candIdx), ...
                                shadowRawCandFreqHz(ii, candIdx), shadowRawClusterOffset(ii, :), ...
                                shadowRawClusterFreqHz(ii, :), shadowRawClusterScore(ii, :), ...
                                shadowRawClusterHits(ii, :), settings);
                            [bestClusterIdxNow, bestClusterDist] = localFindBestClusterMatch( ...
                                shadowRawCandOffsetChips(ii, candIdx), shadowRawCandFreqHz(ii, candIdx), ...
                                shadowRawClusterOffset(ii, :), shadowRawClusterFreqHz(ii, :), settings);
                            if bestClusterIdxNow > 0
                                navResults.shadowRawClusterBestDistChips(ii, currMeasNr) = bestClusterDist;
                            end
                            else
                                [startedFromDirectCluster, startOffsetChips, startOffsetSamples, startFreqHz, bestClusterIdxNow] = ...
                                    localGetClusterDirectInitStart(trackDeepIn(ii), shadowRawClusterOffset(ii, :), ...
                                    shadowRawClusterFreqHz(ii, :), shadowRawClusterScore(ii, :), ...
                                    shadowRawClusterHits(ii, :), settings);
                                useClusterCenter = startedFromDirectCluster;
                            end
                            if candIdx > 0 || startedFromDirectCluster
                            [trackShadow(ii), shadowTrackPending(ii), shadowTrackPendingAge(ii), ...
                                shadowTrackValidateCnt(ii), shadowTrackValidateHist(ii, :), initDiag] = ...
                                localStartShadowPending(trackDeepIn(ii), startOffsetChips, ...
                                startOffsetSamples, startFreqHz, ...
                                settings, shadowTrackValidateHist(ii, :));
                            trackShadow(ii) = localSetShadowPendingClusterMeta(trackShadow(ii), ...
                                useClusterCenter, localSelectInitOffsetForMeta(candIdx, startedFromDirectCluster, ...
                                shadowRawCandOffsetChips(ii, :), shadowRawClusterOffset(ii, :), bestClusterIdxNow), ...
                                localSelectInitFreqForMeta(candIdx, startedFromDirectCluster, ...
                                shadowRawCandFreqHz(ii, :), shadowRawClusterFreqHz(ii, :), bestClusterIdxNow), shadowRawClusterOffset(ii, :), ...
                                shadowRawClusterFreqHz(ii, :), shadowRawClusterScore(ii, :), ...
                                shadowRawClusterHits(ii, :), settings);
                            shadowTrackRejectCnt(ii) = 0;
                            shadowTrackShortScore(ii) = 0;
                            shadowTrackLastImprove(ii) = nan;
                            shadowTrackRecheckCnt(ii) = settings.deepShadowRawTrackClusterRecheckBudget;
                            shadowTrackActive(ii) = false;
                            navResults.shadowRawTrackInit(ii, currMeasNr) = true;
                            navResults.shadowInitRecvTimeShiftMs(ii, currMeasNr) = initDiag.recvTimeShiftMs;
                            navResults.shadowInitSampleShift(ii, currMeasNr) = initDiag.sampleShift;
                            navResults.shadowInitNumCoIntShift(ii, currMeasNr) = initDiag.numCoIntShift;
                            navResults.shadowRawInitClusterFirstUsed(ii, currMeasNr) = initStartDiag.clusterFirstUsed;
                            navResults.shadowRawInitBlendAlpha(ii, currMeasNr) = initStartDiag.blendAlpha;
                            navResults.shadowRawInitClusterScore(ii, currMeasNr) = initStartDiag.clusterScore;
                            navResults.shadowRawInitClusterHits(ii, currMeasNr) = initStartDiag.clusterHits;
                            navResults.shadowRawInitClusterMargin(ii, currMeasNr) = initStartDiag.clusterMargin;
                            end
                        end
                    end
                end
            elseif settings.deepShadowRawTrackEnable && shadowRawCandValid(ii) && ...
                    ~shadowTrackPending(ii) && ~shadowTrackActive(ii) && shadowRawRelayCooldownCnt(ii) <= 0
                [candIdx, candPickDiag] = localPickShadowCandidateByCluster(shadowRawCandOffsetChips(ii, :), ...
                    shadowRawCandFreqHz(ii, :), shadowRawCandMetric(ii, :), ...
                    shadowRawCandUsedMask(ii, :), shadowRawClusterOffset(ii, :), ...
                    shadowRawClusterFreqHz(ii, :), shadowRawClusterScore(ii, :), ...
                    shadowRawClusterHits(ii, :), settings, shadowRawFailExcludeValid(ii), ...
                    shadowRawFailExcludeOffset(ii), shadowRawFailExcludeFreqHz(ii));
                navResults.shadowRawCandMetricGap(ii, currMeasNr) = candPickDiag.metricGap;
                navResults.shadowRawCandClusterOnlyUsed(ii, currMeasNr) = candPickDiag.clusterOnlyUsed;
                navResults.shadowRawCandSelectedClusterSupport(ii, currMeasNr) = candPickDiag.selectedClusterSupport;
                navResults.shadowRawCandSelectedMetricNorm(ii, currMeasNr) = candPickDiag.selectedMetricNorm;
                navResults.shadowRawFailExcludeApplied(ii, currMeasNr) = candPickDiag.failExcludeApplied;
                failFallbackBlocked = candPickDiag.failExcludeApplied && candIdx > 0 && ...
                    (~isfinite(candPickDiag.selectedClusterSupport) || ...
                    candPickDiag.selectedClusterSupport < settings.deepShadowRawFailFallbackMinClusterSupport);
                navResults.shadowRawFailFallbackBlocked(ii, currMeasNr) = failFallbackBlocked;
                directClusterReady = localHasDirectClusterInit(shadowRawClusterOffset(ii, :), shadowRawClusterFreqHz(ii, :), ...
                    shadowRawClusterScore(ii, :), shadowRawClusterHits(ii, :), settings);
                if ~failFallbackBlocked && (candIdx > 0 || directClusterReady)
                    shadowRawCandSelectIdx(ii) = candIdx;
                    shadowRawFailExcludeValid(ii) = false;
                    shadowRawFailExcludeOffset(ii) = nan;
                    shadowRawFailExcludeFreqHz(ii) = nan;
                    shadowRawFailExcludeAge(ii) = inf;
                    useClusterCenter = false;
                    startedFromDirectCluster = false;
                    bestClusterIdxNow = 0;
                    if candIdx > 0
                        shadowRawCandUsedMask(ii, candIdx) = true;
                        [useClusterCenter, startOffsetChips, startOffsetSamples, startFreqHz, initStartDiag] = ...
                            localBuildShadowStartPoint(trackDeepIn(ii), ...
                            shadowRawCandOffsetChips(ii, candIdx), shadowRawCandOffsetSamples(ii, candIdx), ...
                            shadowRawCandFreqHz(ii, candIdx), shadowRawClusterOffset(ii, :), ...
                            shadowRawClusterFreqHz(ii, :), shadowRawClusterScore(ii, :), ...
                            shadowRawClusterHits(ii, :), settings);
                        [bestClusterIdxNow, ~, ~] = localFindBestClusterMatch( ...
                            shadowRawCandOffsetChips(ii, candIdx), shadowRawCandFreqHz(ii, candIdx), ...
                            shadowRawClusterOffset(ii, :), shadowRawClusterFreqHz(ii, :), settings);
                    else
                        [startedFromDirectCluster, startOffsetChips, startOffsetSamples, startFreqHz, bestClusterIdxNow] = ...
                            localGetClusterDirectInitStart(trackDeepIn(ii), shadowRawClusterOffset(ii, :), ...
                            shadowRawClusterFreqHz(ii, :), shadowRawClusterScore(ii, :), ...
                            shadowRawClusterHits(ii, :), settings);
                        useClusterCenter = startedFromDirectCluster;
                    end
                    if candIdx > 0 || startedFromDirectCluster
                    [trackShadow(ii), shadowTrackPending(ii), shadowTrackPendingAge(ii), ...
                        shadowTrackValidateCnt(ii), shadowTrackValidateHist(ii, :), initDiag] = ...
                        localStartShadowPending(trackDeepIn(ii), startOffsetChips, startOffsetSamples, ...
                        startFreqHz, settings, shadowTrackValidateHist(ii, :));
                    trackShadow(ii) = localSetShadowPendingClusterMeta(trackShadow(ii), ...
                        useClusterCenter, localSelectInitOffsetForMeta(candIdx, startedFromDirectCluster, ...
                        shadowRawCandOffsetChips(ii, :), shadowRawClusterOffset(ii, :), bestClusterIdxNow), ...
                        localSelectInitFreqForMeta(candIdx, startedFromDirectCluster, ...
                        shadowRawCandFreqHz(ii, :), shadowRawClusterFreqHz(ii, :), bestClusterIdxNow), shadowRawClusterOffset(ii, :), ...
                        shadowRawClusterFreqHz(ii, :), shadowRawClusterScore(ii, :), ...
                        shadowRawClusterHits(ii, :), settings);
                    shadowTrackRejectCnt(ii) = 0;
                    shadowTrackShortScore(ii) = 0;
                    shadowTrackLastImprove(ii) = nan;
                    shadowTrackRecheckCnt(ii) = settings.deepShadowRawTrackClusterRecheckBudget;
                    shadowTrackActive(ii) = false;
                    navResults.shadowRawTrackInit(ii, currMeasNr) = true;
                    navResults.shadowInitRecvTimeShiftMs(ii, currMeasNr) = initDiag.recvTimeShiftMs;
                    navResults.shadowInitSampleShift(ii, currMeasNr) = initDiag.sampleShift;
                    navResults.shadowInitNumCoIntShift(ii, currMeasNr) = initDiag.numCoIntShift;
                    if candIdx > 0
                        navResults.shadowRawInitClusterFirstUsed(ii, currMeasNr) = initStartDiag.clusterFirstUsed;
                        navResults.shadowRawInitBlendAlpha(ii, currMeasNr) = initStartDiag.blendAlpha;
                        navResults.shadowRawInitClusterScore(ii, currMeasNr) = initStartDiag.clusterScore;
                        navResults.shadowRawInitClusterHits(ii, currMeasNr) = initStartDiag.clusterHits;
                        navResults.shadowRawInitClusterMargin(ii, currMeasNr) = initStartDiag.clusterMargin;
                    end
                    rawCandidateReuseUsed = true;
                    end
                else
                    shadowRawCandValid(ii) = false;
                    shadowRawCandAge(ii) = inf;
                    shadowRawCandUsedMask(ii, :) = false;
                    shadowRawCandSelectIdx(ii) = 0;
                end
            end
            navResults.shadowRawCandReuseUsed(ii, currMeasNr) = rawCandidateReuseUsed;
            navResults.shadowRawCandSelectIdx(ii, currMeasNr) = shadowRawCandSelectIdx(ii);
        end
    end

    validConsensus = shadowAcqReadyMask & shadowAcqQualifiedMask & isfinite(shadowAcqCandidateOffset);
    consensusReadyNow = false;
    consensusSatNum = 0;
    consensusOffsetNow = nan;
    if sum(validConsensus) >= settings.deepShadowConsensusMinSat
        off = shadowAcqCandidateOffset(validConsensus);
        binW = max(0.25, settings.deepShadowConsensusBinChips);
        binId = round(off / binW);
        uniqBins = unique(binId);
        bestCount = 0;
        bestOff = nan;
        for bb = 1:numel(uniqBins)
            mBin = (binId == uniqBins(bb));
            c = sum(mBin);
            if c > bestCount
                bestCount = c;
                bestOff = median(off(mBin));
            end
        end
        consensusSatNum = bestCount;
        consensusOffsetNow = bestOff;
        if consensusSatNum >= settings.deepShadowConsensusMinSat
            if isfinite(shadowConsensusOffset) && ...
                    abs(consensusOffsetNow - shadowConsensusOffset) <= settings.deepShadowConsensusOffsetTolChips
                shadowConsensusStableCnt = shadowConsensusStableCnt + 1;
                shadowConsensusOffset = 0.7 * shadowConsensusOffset + 0.3 * consensusOffsetNow;
            else
                shadowConsensusOffset = consensusOffsetNow;
                shadowConsensusStableCnt = 1;
            end
            consensusReadyNow = shadowConsensusStableCnt >= settings.deepShadowConsensusStableEpochs;
        else
            shadowConsensusStableCnt = max(0, shadowConsensusStableCnt - 1);
        end
    else
        shadowConsensusStableCnt = max(0, shadowConsensusStableCnt - 1);
    end
    navResults.shadowConsensusOffsetChips(1, currMeasNr) = shadowConsensusOffset;
    navResults.shadowConsensusSatNum(1, currMeasNr) = consensusSatNum;
    navResults.shadowConsensusStableCnt(1, currMeasNr) = shadowConsensusStableCnt;
    navResults.shadowConsensusReady(1, currMeasNr) = consensusReadyNow;

    % 1. GNSS瑙傛祴鍊?
    diagLocalTic = tic;
    navSolut = postNavTight(trackDeepIn, settings, eph, TOW);
    navSolutDet = navSolut;
    if settings.deepUseIndependentDetectTrack
        settingsDet.recvTime = settings.recvTime;
        navSolutDet = postNavTight(trackDeepDet, settingsDet, eph, TOW);
    end
    diagMainNavTimeSec = diagMainNavTimeSec + toc(diagLocalTic);
    navResults.rawPDiag(1:numel(navSolut.rawP), currMeasNr) = navSolut.rawP(:);
    navResults.rawPdotDiag(1:numel(navSolut.rawP_dot), currMeasNr) = navSolut.rawP_dot(:);
    if isfield(navSolut, 'launchTime')
        navResults.launchTimeDiag(1:numel(navSolut.launchTime), currMeasNr) = navSolut.launchTime(:);
    end
    if isfield(navSolut, 'codePhaseTao')
        navResults.codePhaseTaoDiag(1:numel(navSolut.codePhaseTao), currMeasNr) = navSolut.codePhaseTao(:);
    end
    if isfield(navSolut, 'remSampleNum')
        navResults.remSampleNumDiag(1:numel(navSolut.remSampleNum), currMeasNr) = navSolut.remSampleNum(:);
    end
    if isfield(navSolut, 'totalSampleNum')
        navResults.totalSampleNumDiag(1:numel(navSolut.totalSampleNum), currMeasNr) = navSolut.totalSampleNum(:);
    end
    if isfield(navSolut, 'numOfCoInt')
        navResults.numOfCoIntDiag(1:numel(navSolut.numOfCoInt), currMeasNr) = navSolut.numOfCoInt(:);
    end
    if isfield(navSolut, 'samplePos')
        navResults.samplePosDiag(1:numel(navSolut.samplePos), currMeasNr) = navSolut.samplePos(:);
    end
    if isfield(navSolut, 'trackRecvTime')
        navResults.trackRecvTimeDiag(1:numel(navSolut.trackRecvTime), currMeasNr) = navSolut.trackRecvTime(:);
    end
    if isfield(navSolut, 'trackRecvTimeNorm')
        navResults.trackRecvTimeNormDiag(1:numel(navSolut.trackRecvTimeNorm), currMeasNr) = navSolut.trackRecvTimeNorm(:);
    end
    if isfield(navSolut, 'sampleClockErrSamples')
        navResults.sampleClockErrSampDiag(1:numel(navSolut.sampleClockErrSamples), currMeasNr) = navSolut.sampleClockErrSamples(:);
    end
    if isfield(navSolut, 'pseudorangeMs')
        navResults.pseudorangeMsDiag(1:numel(navSolut.pseudorangeMs), currMeasNr) = navSolut.pseudorangeMs(:);
    end
    if isfield(navSolut, 'pseudorangeModuloMs')
        navResults.pseudorangeModuloMsDiag(1:numel(navSolut.pseudorangeModuloMs), currMeasNr) = navSolut.pseudorangeModuloMs(:);
    end
    % 鍙繚鐣欎竴棰楀崼鏄熻娴嬪€?
%     navSolut.rawP = navSolut.rawP(1); navSolut.satPositions = navSolut.satPositions(:,1); 
%     navSolut.satVelocity = navSolut.satVelocity(:,1);  navSolut.satClkCorr = navSolut.satClkCorr(1); 
    
    [posxyz, ~] = blh2xyz(ins.pos);
    epochElapsedSec = (currMeasNr - 1) * settings.navSolPeriod / 1000;
    [baselineOutputPos, baselineOutputVel, baselineOutputValid, baselineOutputLastPos, baselineOutputLastVel, baselineOutputLastEpoch, baselineOutputHoldAgeNow] = ...
        localGetBaselineOutput(navSolutions, currMeasNr, baselineOutputLastPos, baselineOutputLastVel, baselineOutputLastEpoch, settings);
    if settings.deepShadowTrustedAnchorEnable
        baselineTrustedRefNow = localUseDs5BaselineTrustedRef(settings, currMeasNr) && baselineOutputValid && all(isfinite(baselineOutputPos));
        if baselineTrustedRefNow
            trustedAnchorValid = true;
            trustedAnchorPos = baselineOutputPos(:);
            if localGetSettingValue(settings, 'deepShadowDs5BaselineTrustedRefUseVelocity', 1) ~= 0 && all(isfinite(baselineOutputVel))
                trustedAnchorVel = baselineOutputVel(:);
            else
                trustedAnchorVel = nan(3,1);
            end
            trustedAnchorTimeSec = epochElapsedSec;
            trustedAnchorSource = 3;
        elseif settings.deepShadowTrustedAnchorUseTruthTrj
            if ~trustedAnchorValid && epochElapsedSec >= settings.deepShadowTrustedAnchorTimeSec
                [taPos, taVel, taOk] = localTrustedAnchorFromTrj(trj, t0_imu + settings.deepShadowTrustedAnchorTimeSec);
                if taOk
                    trustedAnchorValid = true;
                    trustedAnchorPos = taPos(:);
                    trustedAnchorVel = taVel(:);
                    trustedAnchorTimeSec = settings.deepShadowTrustedAnchorTimeSec;
                    trustedAnchorSource = 2;
                end
            end
        elseif deepModeState < 2 && epochElapsedSec <= settings.deepShadowTrustedAnchorTimeSec
            trustedAnchorValid = true;
            trustedAnchorPos = posxyz(:);
            trustedAnchorVel = localNedVelToEcef(ins.pos, ins.vn);
            trustedAnchorTimeSec = epochElapsedSec;
            trustedAnchorSource = 1;
        end
        navResults.shadowTrustedAnchorValid(1, currMeasNr) = trustedAnchorValid;
        navResults.shadowTrustedAnchorSource(1, currMeasNr) = trustedAnchorSource;
        navResults.shadowTrustedAnchorBaselineFed(1, currMeasNr) = baselineTrustedRefNow;
        if trustedAnchorValid
            navResults.shadowTrustedAnchorX(1, currMeasNr) = trustedAnchorPos(1);
            navResults.shadowTrustedAnchorY(1, currMeasNr) = trustedAnchorPos(2);
            navResults.shadowTrustedAnchorZ(1, currMeasNr) = trustedAnchorPos(3);
            navResults.shadowTrustedAnchorVX(1, currMeasNr) = trustedAnchorVel(1);
            navResults.shadowTrustedAnchorVY(1, currMeasNr) = trustedAnchorVel(2);
            navResults.shadowTrustedAnchorVZ(1, currMeasNr) = trustedAnchorVel(3);
        end
    end
    ds5AbsAnchorCurrPos = nan(3,1);
    ds5AbsAnchorCurrVel = nan(3,1);
    ds5AbsAnchorCurrAgeEpochs = nan;
    if localIsDs5Scenario(settings)
        absAnchorCandPos = nan(3,1);
        absAnchorCandVel = nan(3,1);
        absAnchorCandSource = 0;
        if trustedAnchorValid && all(isfinite(trustedAnchorPos))
            absAnchorCandPos = trustedAnchorPos(:);
            absAnchorCandVel = trustedAnchorVel(:);
            absAnchorCandSource = max(1, trustedAnchorSource);
        elseif baselineOutputValid && all(isfinite(baselineOutputPos))
            absAnchorCandPos = baselineOutputPos(:);
            absAnchorCandVel = baselineOutputVel(:);
            absAnchorCandSource = 10;
        elseif all(isfinite(ds5RefObsPosLastAccepted))
            absAnchorCandPos = ds5RefObsPosLastAccepted(:);
            if all(isfinite(shadowClosedLoopLastVel))
                absAnchorCandVel = shadowClosedLoopLastVel(:);
            end
            absAnchorCandSource = 20;
        end
        absAnchorFreezeSec = localGetSettingValue(settings, 'deepShadowDs5RecoveredFilterAbsAnchorFreezeSec', settings.deepShadowDs5RefObsRecoveryStartSec);
        if (~ds5RecoveredAbsAnchorValid || deepModeState < 2 || epochElapsedSec <= absAnchorFreezeSec) && all(isfinite(absAnchorCandPos))
            ds5RecoveredAbsAnchorPos = absAnchorCandPos(:);
            ds5RecoveredAbsAnchorVel = absAnchorCandVel(:);
            ds5RecoveredAbsAnchorEpoch = currMeasNr;
            ds5RecoveredAbsAnchorTimeSec = epochElapsedSec;
            ds5RecoveredAbsAnchorValid = true;
            ds5RecoveredAbsAnchorFrozen = false;
            ds5RecoveredAbsAnchorSource = absAnchorCandSource;
        elseif ds5RecoveredAbsAnchorValid && deepModeState == 2 && epochElapsedSec > absAnchorFreezeSec
            ds5RecoveredAbsAnchorFrozen = true;
        end
        [ds5AbsAnchorCurrPos, ds5AbsAnchorCurrVel, ds5AbsAnchorCurrAgeEpochs] = ...
            localProjectDs5AbsAnchor(ds5RecoveredAbsAnchorPos, ds5RecoveredAbsAnchorVel, ds5RecoveredAbsAnchorEpoch, ds5RecoveredAbsAnchorTimeSec, ds5RecoveredAbsAnchorValid, epochElapsedSec, currMeasNr, nan(3,1), settings);
    end
    diagLocalTic = tic;
    [rho, LOS, AzEl, vrs, ~, Cen] = rhoSatRec_zcj(navSolut.satPositions', posxyz, navSolut.rawP', navSolut.satVelocity', ins.vn);
    dopplerFeedback = -vrs / settings.c * 1575.42e6;   % numOfSat * 1
    [~, LOSDet, ~, vrsDet, ~, CenDet] = rhoSatRec_zcj(navSolutDet.satPositions', posxyz, ...
        navSolutDet.rawP', navSolutDet.satVelocity', ins.vn);
    diagRhoTimeSec = diagRhoTimeSec + toc(diagLocalTic);

    % Receiver clock drift term in Hz (for deep-loop NCO synthesis).
    clkDriftHz = 0;
    clkDriftMps = nan;
    if isfield(kf, 'xk') && ~isempty(kf.xk)
        clkDriftMps = kf.xk(end);
        clkDriftHz = clkDriftMps / lambdaL1;
    end
    if isfinite(settings.deepClkDriftHzLimit)
        clkDriftHz = max(-settings.deepClkDriftHzLimit, min(settings.deepClkDriftHzLimit, clkDriftHz));
    end
    navResults.deepClkDriftHz(1, currMeasNr) = clkDriftHz;
    navResults.clkDriftHz(1, currMeasNr) = clkDriftHz;
    navResults.kfClockMps(1, currMeasNr) = clkDriftMps;
    if currMeasNr > 1 && isfield(navResults, 'kfClockMps')
        navResults.kfClockPrevMps(1, currMeasNr) = navResults.kfClockMps(1, currMeasNr-1);
    end

    % Pseudorange / range-rate correction driven by INS Doppler.
    rawPUsed = navSolut.rawP(:);
    rawPdotUsed = navSolut.rawP_dot(:);
    dtNav = settings.navSolPeriod / 1000;
    for ii = 1:numActChnList
        prn = trackDeepIn(ii).PRN;
        if prn > numel(rhoCorrByPrn)
            rhoCorrByPrn(prn) = nan;
        end
        if isnan(rhoCorrByPrn(prn))
            rhoCorrByPrn(prn) = navSolut.rawP(ii);
        end
        prrInsPred = -lambdaL1 * (dopplerFeedback(ii) + clkDriftHz);  % [m/s]
        rhoInsPred = rhoCorrByPrn(prn) + prrInsPred * dtNav;
        rawPUsed(ii) = (1 - rhoBlendNow) * navSolut.rawP(ii) + rhoBlendNow * rhoInsPred;
        rawPdotUsed(ii) = (1 - rhoBlendNow) * navSolut.rawP_dot(ii) + rhoBlendNow * prrInsPred;
        rhoCorrByPrn(prn) = rawPUsed(ii);
    end
    navResults.rhoCorrRmsM(1, currMeasNr) = sqrt(mean((rawPUsed - navSolut.rawP(:)).^2));
    navResults.prrCorrRmsMps(1, currMeasNr) = sqrt(mean((rawPdotUsed - navSolut.rawP_dot(:)).^2));

    el = AzEl(:,2);
    el(el < 15*pi/180) = 1*pi/180;
    el = el(:);
    W = diag(sin(el.^2));
    delta_rawP = rawPUsed + settings.c * navSolut.satClkCorr(:) - rho;
    chip2m = settings.c * 1e-3 / 1023;
    for ii = 1:min(numActChnList, numel(delta_rawP))
        candOff = shadowAcqCandidateOffset(ii);
        if isfinite(candOff)
            shadowCandDelta = delta_rawP(ii) - candOff * chip2m;
            navResults.shadowCandDeltaRawP(ii, currMeasNr) = shadowCandDelta;
            navResults.shadowCandDeltaImprove(ii, currMeasNr) = abs(delta_rawP(ii)) - abs(shadowCandDelta);
            navResults.shadowCandUsed(ii, currMeasNr) = true;
        end
    end

    satAvailNum = numel(delta_rawP);
    satUseMask = true(satAvailNum, 1);
    if settings.deepPrInnovationGateEnable && satAvailNum > 0
        deltaForGate = delta_rawP(:);
        if settings.deepPrInnovationGateUseMedianDetrend
            deltaForGate = deltaForGate - median(deltaForGate, 'omitnan');
        end
        sinEl = max(0.1, sin(el));
        gateThr = settings.deepPrInnovationGateK * (10 ./ sinEl);
        gateThr = max(settings.deepPrInnovationGateMinM, gateThr);
        gateThr = min(settings.deepPrInnovationGateMaxM, gateThr);
        satUseMask = isfinite(deltaForGate) & isfinite(gateThr) & ...
                     (abs(deltaForGate) <= gateThr);
        minSatUse = min(max(1, settings.deepPrInnovationGateMinSat), satAvailNum);
        if sum(satUseMask) < minSatUse
            absInnov = abs(deltaForGate);
            absInnov(~isfinite(absInnov)) = inf;
            [~, sortIdx] = sort(absInnov, 'ascend');
            satUseMask(:) = false;
            satUseMask(sortIdx(1:minSatUse)) = true;
        end
    end
    satUseIdx = find(satUseMask);
    navResults.gnssSatAvailNum(1, currMeasNr) = satAvailNum;
    navResults.gnssSatUsedNum(1, currMeasNr) = numel(satUseIdx);
    navResults.gnssGateRejectNum(1, currMeasNr) = satAvailNum - numel(satUseIdx);
    navResults.deltaRawPMean(1, currMeasNr) = mean(delta_rawP, 'omitnan');
    navResults.deltaRawPMed(1, currMeasNr) = median(delta_rawP, 'omitnan');
    navResults.deltaRawPP95(1, currMeasNr) = prctile(delta_rawP(isfinite(delta_rawP)), 95);
    navResults.deltaRawPMax(1, currMeasNr) = max(delta_rawP, [], 'omitnan');
    navResults.deltaRawPUsedNum(1, currMeasNr) = numel(satUseIdx);
    navResults.deltaRawPRejectNum(1, currMeasNr) = satAvailNum - numel(satUseIdx);

    activeShadowIdx = find(shadowTrackPending | shadowTrackActive);
    if ~isempty(activeShadowIdx)
        mainSampleClockBaseRef = median(navSolut.sampleClockBase, 'omitnan');
        diagLocalTic = tic;
        settingsShadowNav = settings;
        if settings.deepShadowCoreServoEnable
            settingsShadowNav.deepCodeReacqNavCorrEnable = 1;
        end
        navSolutShadow = postNavTight(trackShadow(activeShadowIdx), settingsShadowNav, eph, TOW, mainSampleClockBaseRef);
        [rhoShadow, ~, ~, ~] = rhoSatRec_zcj(navSolutShadow.satPositions', posxyz, ...
            navSolutShadow.rawP', navSolutShadow.satVelocity', ins.vn);
        diagShadowNavTimeSec = diagShadowNavTimeSec + toc(diagLocalTic);
        deltaShadow = navSolutShadow.rawP(:) + settings.c * navSolutShadow.satClkCorr(:) - rhoShadow;
        for kk = 1:numel(activeShadowIdx)
            ii = activeShadowIdx(kk);
            prnShadow = trackShadow(ii).PRN;
            idxMain = find([trackDeepIn.PRN] == prnShadow, 1, 'first');
            if isempty(idxMain) || idxMain > numel(delta_rawP)
                continue;
            end
            navResults.shadowRawTrackDeltaRawP(ii, currMeasNr) = deltaShadow(kk);
            navResults.shadowRawTrackImprove(ii, currMeasNr) = abs(delta_rawP(idxMain)) - abs(deltaShadow(kk));
            if shadowTrackPending(ii)
                allowValidate = true;
                if isfield(trackShadow(ii), 'deepShadowInitHold') && ...
                        isfinite(trackShadow(ii).deepShadowInitHold) && ...
                        trackShadow(ii).deepShadowInitHold > 0
                    allowValidate = false;
                end
                currCandOffset = nan;
                currCandFreqAbs = trackShadow(ii).carrFreq;
                if isfield(trackShadow(ii), 'deepShadowCandOffsetChips')
                    currCandOffset = trackShadow(ii).deepShadowCandOffsetChips;
                end
                if isfield(trackShadow(ii), 'deepShadowCandFreqAbsHz') && isfinite(trackShadow(ii).deepShadowCandFreqAbsHz)
                    currCandFreqAbs = trackShadow(ii).deepShadowCandFreqAbsHz;
                end
                [bestClusterIdxNow, bestClusterDistNow, bestClusterFreqErrNow] = localFindBestClusterMatch( ...
                    currCandOffset, currCandFreqAbs, ...
                    shadowRawClusterOffset(ii, :), shadowRawClusterFreqHz(ii, :), settings);
                clusterConsistent = true;
                if bestClusterIdxNow > 0
                    clusterConsistent = bestClusterDistNow <= settings.deepShadowRawClusterValidateTolChips && ...
                        bestClusterFreqErrNow <= settings.deepShadowRawClusterValidateTolFreqHz;
                    navResults.shadowRawClusterBestDistChips(ii, currMeasNr) = bestClusterDistNow;
                end
                bestClusterScoreNow = 0;
                secondClusterScoreNow = 0;
                bestClusterHitsNow = 0;
                if bestClusterIdxNow > 0
                    bestClusterScoreNow = shadowRawClusterScore(ii, bestClusterIdxNow);
                    bestClusterHitsNow = shadowRawClusterHits(ii, bestClusterIdxNow);
                    tmpScoreNow = shadowRawClusterScore(ii, :);
                    tmpScoreNow(bestClusterIdxNow) = -inf;
                    secondClusterScoreNow = max(tmpScoreNow);
                    if ~isfinite(secondClusterScoreNow)
                        secondClusterScoreNow = 0;
                    end
                end
                clusterStrongPass = clusterConsistent && ...
                    bestClusterScoreNow >= settings.deepShadowRawClusterPassScore && ...
                    bestClusterHitsNow >= settings.deepShadowRawClusterPassHits && ...
                    (bestClusterScoreNow - secondClusterScoreNow) >= settings.deepShadowRawClusterPassMargin;
                clusterBackedPending = isfield(trackShadow(ii), 'deepShadowInitFromClusterCenter') && ...
                    trackShadow(ii).deepShadowInitFromClusterCenter && ...
                    trackShadow(ii).deepShadowInitClusterScore >= settings.deepShadowRawClusterInitScore && ...
                    trackShadow(ii).deepShadowInitClusterHits >= settings.deepShadowRawClusterInitHits;
                currImprove = navResults.shadowRawTrackImprove(ii, currMeasNr);
                histStart = max(1, currMeasNr - settings.deepShadowRawTrackValidateWindowEpochs + 1);
                recentImprove = navResults.shadowRawTrackImprove(ii, histStart:currMeasNr);
                recentImprove = recentImprove(isfinite(recentImprove));
                windowImproveMed = nan;
                windowImproveMean = nan;
                windowImproveMin = nan;
                trendHitCount = 0;
                if ~isempty(recentImprove)
                    windowImproveMed = median(recentImprove);
                    windowImproveMean = mean(recentImprove);
                    windowImproveMin = min(recentImprove);
                    trendHitCount = sum(recentImprove >= settings.deepShadowRawTrackClusterInstantFloorM);
                end
                clusterSoftValidate = allowValidate && clusterBackedPending && clusterConsistent && ...
                    isfinite(currImprove) && currImprove >= settings.deepShadowRawTrackClusterInstantFloorM && ...
                    isfinite(windowImproveMed) && windowImproveMed >= settings.deepShadowRawTrackClusterWindowImproveMinM;
                clusterTrendValidate = allowValidate && clusterBackedPending && clusterConsistent && ...
                    isfinite(windowImproveMean) && windowImproveMean >= settings.deepShadowRawTrackClusterTrendMeanMinM && ...
                    isfinite(windowImproveMin) && windowImproveMin >= settings.deepShadowRawTrackClusterTrendMinM && ...
                    trendHitCount >= settings.deepShadowRawTrackClusterTrendPassHits;
                absDeltaValidate = settings.deepShadowRawTrackValidateAbsDeltaEnable && ...
                    isfinite(deltaShadow(kk)) && ...
                    abs(deltaShadow(kk)) <= settings.deepShadowRawTrackValidateAbsDeltaMaxM;
                holdAbsDeltaValidate = settings.deepShadowRawTrackValidateDuringHoldEnable && ...
                    absDeltaValidate;
                ds5TruePeakReady = false;
                ds5CandInWindow = false;
                ds5CurrentScore = nan;
                ds5CandScore = nan;
                ds5CandBetter = false;
                ds5ProbationPass = false;
                if localUseDs5TruePeak(settings, currMeasNr) && ii <= numel(ds5TrueRefReadyState) && ds5TrueRefReadyState(ii)
                    currentFreqHzLocal = trackDeepIn(ii).carrFreq - settings.IF;
                    currentCodeChipsLocal = localGetTrackField(trackDeepIn(ii), 'deepShadowCandOffsetChips', 0.0);
                    currentMetricLocal = localGetTrackField(trackDeepIn(ii), 'deepShadowCandMetric', nan);
                    candFreqHzLocal = trackShadow(ii).carrFreq - settings.IF;
                    candCodeChipsLocal = localGetTrackField(trackShadow(ii), 'deepShadowCandOffsetChips', nan);
                    candMetricLocal = localGetTrackField(trackShadow(ii), 'deepShadowCandMetric', nan);
                    ds5CurrentScore = localScoreDs5Peak(currentFreqHzLocal, currentCodeChipsLocal, ...
                        navResults.shadowRawTrackImprove(ii, currMeasNr), currentMetricLocal, ...
                        ds5TrueRefHzState(ii), ds5TrueRefCodeState(ii), settings);
                    ds5CandScore = localScoreDs5Peak(candFreqHzLocal, candCodeChipsLocal, ...
                        navResults.shadowRawTrackImprove(ii, currMeasNr), candMetricLocal, ...
                        ds5TrueRefHzState(ii), ds5TrueRefCodeState(ii), settings);
                    ds5CandInWindow = localCandidateInDs5TrueWindow(candFreqHzLocal, candCodeChipsLocal, ...
                        ds5TrueRefHzState(ii), ds5TrueRefCodeState(ii), settings);
                    ds5CandBetter = ds5CandInWindow && isfinite(ds5CandScore) && ...
                        ds5CandScore >= ds5CurrentScore + localGetSettingValue(settings, 'deepShadowDs5TruePeakCandidateMargin', 0.10);
                    ds5ProbationPass = allowValidate && clusterConsistent && ds5CandBetter && ...
                        isfinite(currImprove) && currImprove >= localGetSettingValue(settings, 'deepShadowDs5TruePeakRevokeImproveMaxM', -150.0);
                    ds5TruePeakReady = true;
                end
                navResults.shadowDs5CandInTrueWindow(ii, currMeasNr) = ds5CandInWindow;
                navResults.shadowDs5CandScore(ii, currMeasNr) = ds5CandScore;
                navResults.shadowDs5CurrentScore(ii, currMeasNr) = ds5CurrentScore;
                navResults.shadowDs5CandBetterThanCurrent(ii, currMeasNr) = ds5CandBetter;
                navResults.shadowDs5ProbationPass(ii, currMeasNr) = ds5ProbationPass;
                validateHit = (allowValidate && clusterConsistent && ...
                    ((navResults.shadowRawTrackImprove(ii, currMeasNr) >= settings.deepShadowRawTrackValidateImproveMinM) || ...
                     (clusterStrongPass && navResults.shadowRawTrackImprove(ii, currMeasNr) >= settings.deepShadowRawClusterPassImproveRelaxM) || ...
                     clusterSoftValidate || clusterTrendValidate)) || ...
                     (allowValidate && absDeltaValidate) || holdAbsDeltaValidate;
                if ds5TruePeakReady
                    validateHit = ds5ProbationPass;
                end
                shadowTrackValidateHist(ii, :) = [shadowTrackValidateHist(ii, 2:end), validateHit];
                windowHits = sum(shadowTrackValidateHist(ii, :));
                if allowValidate && isfinite(currImprove)
                    if currImprove >= settings.deepShadowRawTrackScoreGoodImproveM
                        shadowTrackShortScore(ii) = shadowTrackShortScore(ii) + 1;
                    elseif currImprove <= settings.deepShadowRawTrackScoreBadImproveM
                        shadowTrackShortScore(ii) = shadowTrackShortScore(ii) - 1;
                    end
                    if isfinite(shadowTrackLastImprove(ii))
                        if currImprove - shadowTrackLastImprove(ii) >= settings.deepShadowRawTrackScoreRiseM
                            shadowTrackShortScore(ii) = shadowTrackShortScore(ii) + 1;
                        elseif shadowTrackLastImprove(ii) - currImprove >= settings.deepShadowRawTrackScoreRiseM
                            shadowTrackShortScore(ii) = shadowTrackShortScore(ii) - 1;
                        end
                    end
                    shadowTrackShortScore(ii) = max(-4, min(4, shadowTrackShortScore(ii)));
                end
                shadowTrackLastImprove(ii) = currImprove;
                softRejectProtect = allowValidate && clusterBackedPending && clusterConsistent && ...
                    shadowTrackPendingAge(ii) <= settings.deepShadowRawTrackClusterSoftRejectEpochs && ...
                    isfinite(currImprove) && currImprove >= settings.deepShadowRawTrackClusterRejectFloorM;
                recheckProtect = allowValidate && clusterBackedPending && clusterConsistent && ...
                    shadowTrackRecheckCnt(ii) > 0 && ...
                    shadowTrackPendingAge(ii) <= settings.deepShadowRawTrackValidateMaxEpochs + ...
                    settings.deepShadowRawTrackClusterExtraMaxEpochs;
                if allowValidate && isfinite(navResults.shadowRawTrackImprove(ii, currMeasNr)) && ...
                        navResults.shadowRawTrackImprove(ii, currMeasNr) <= settings.deepShadowRawTrackEarlyRejectImproveMaxM && ...
                        ~softRejectProtect && ~recheckProtect
                    shadowTrackRejectCnt(ii) = shadowTrackRejectCnt(ii) + 1;
                elseif allowValidate
                    shadowTrackRejectCnt(ii) = 0;
                end
                if validateHit
                    shadowTrackValidateCnt(ii) = shadowTrackValidateCnt(ii) + 1;
                else
                    shadowTrackValidateCnt(ii) = 0;
                end
                if shadowTrackValidateCnt(ii) >= settings.deepShadowRawTrackValidateEpochs || ...
                        windowHits >= settings.deepShadowRawTrackValidateWindowPassHits || ...
                        (allowValidate && shadowTrackShortScore(ii) >= settings.deepShadowRawTrackScorePassMin)
                    shadowTrackPending(ii) = false;
                    shadowTrackPendingAge(ii) = 0;
                    shadowTrackActive(ii) = true;
                    shadowTrackPassReady(ii) = true;
                    shadowPassReadyBadCnt(ii) = 0;
                    shadowTrackValidateCnt(ii) = 0;
                    shadowTrackRejectCnt(ii) = 0;
                    shadowTrackShortScore(ii) = 0;
                    shadowTrackLastImprove(ii) = nan;
                    shadowTrackRecheckCnt(ii) = 0;
                    shadowTrackValidateHist(ii, :) = false;
                    navResults.shadowRawTrackValidatePass(ii, currMeasNr) = true;
                elseif recheckProtect && ...
                        (shadowTrackRejectCnt(ii) >= settings.deepShadowRawTrackEarlyRejectEpochs || ...
                        (allowValidate && shadowTrackShortScore(ii) <= settings.deepShadowRawTrackScoreRejectMin))
                    shadowTrackRejectCnt(ii) = 0;
                    shadowTrackValidateCnt(ii) = 0;
                    shadowTrackShortScore(ii) = max(-1, shadowTrackShortScore(ii));
                    shadowTrackValidateHist(ii, :) = false;
                    shadowTrackLastImprove(ii) = nan;
                    shadowTrackRecheckCnt(ii) = shadowTrackRecheckCnt(ii) - 1;
                    if isfield(trackShadow(ii), 'deepShadowInitHold')
                        trackShadow(ii).deepShadowInitHold = max(trackShadow(ii).deepShadowInitHold, ...
                            settings.deepShadowRawTrackClusterRecheckHoldEpochs);
                    else
                        trackShadow(ii).deepShadowInitHold = settings.deepShadowRawTrackClusterRecheckHoldEpochs;
                    end
                elseif shadowTrackRejectCnt(ii) >= settings.deepShadowRawTrackEarlyRejectEpochs || ...
                        (allowValidate && shadowTrackShortScore(ii) <= settings.deepShadowRawTrackScoreRejectMin && ~softRejectProtect)
                    if shadowRawCandSelectIdx(ii) > 0
                        shadowRawFailExcludeValid(ii) = true;
                        shadowRawFailExcludeOffset(ii) = shadowRawCandOffsetChips(ii, shadowRawCandSelectIdx(ii));
                        shadowRawFailExcludeFreqHz(ii) = shadowRawCandFreqHz(ii, shadowRawCandSelectIdx(ii));
                        shadowRawFailExcludeAge(ii) = 0;
                    end
                    shadowTrackPending(ii) = false;
                    shadowTrackPendingAge(ii) = 0;
                    shadowTrackValidateCnt(ii) = 0;
                    shadowTrackRejectCnt(ii) = 0;
                    shadowTrackShortScore(ii) = 0;
                    shadowTrackLastImprove(ii) = nan;
                    shadowTrackRecheckCnt(ii) = 0;
                    shadowTrackValidateHist(ii, :) = false;
                    shadowTrackPassReady(ii) = false;
                    shadowPassReadyBadCnt(ii) = 0;
                    shadowRawCandSelectIdx(ii) = 0;
                    shadowRawRelayCooldownCnt(ii) = settings.deepShadowRawRelayCooldownEpochs;
                    navResults.shadowRawRelayUsed(ii, currMeasNr) = true;
                    navResults.shadowRawFailExcludeOffset(ii, currMeasNr) = shadowRawFailExcludeOffset(ii);
                    navResults.shadowRawFailExcludeFreqHz(ii, currMeasNr) = shadowRawFailExcludeFreqHz(ii);
                    if settings.deepShadowRawTrackEnable && shadowRawCandValid(ii) && ~shadowTrackActive(ii) && ...
                            shadowRawRelayCooldownCnt(ii) <= 0
                        candIdxRetry = localPickShadowCandidateRelay(shadowRawCandOffsetChips(ii, :), ...
                            shadowRawCandFreqHz(ii, :), shadowRawCandMetric(ii, :), ...
                            shadowRawCandUsedMask(ii, :), shadowRawFailExcludeValid(ii), ...
                            shadowRawFailExcludeOffset(ii), shadowRawFailExcludeFreqHz(ii), settings);
                        if candIdxRetry > 0
                            shadowRawCandSelectIdx(ii) = candIdxRetry;
                            shadowRawCandUsedMask(ii, candIdxRetry) = true;
                            [useClusterCenter, startOffsetChips, startOffsetSamples, startFreqHz, initStartDiag] = ...
                                localBuildShadowStartPoint(trackDeepIn(ii), ...
                                shadowRawCandOffsetChips(ii, candIdxRetry), shadowRawCandOffsetSamples(ii, candIdxRetry), ...
                                shadowRawCandFreqHz(ii, candIdxRetry), shadowRawClusterOffset(ii, :), ...
                                shadowRawClusterFreqHz(ii, :), shadowRawClusterScore(ii, :), ...
                                shadowRawClusterHits(ii, :), settings);
                            [trackShadow(ii), shadowTrackPending(ii), shadowTrackPendingAge(ii), ...
                                shadowTrackValidateCnt(ii), shadowTrackValidateHist(ii, :), initDiagRetry] = ...
                                localStartShadowPending(trackDeepIn(ii), startOffsetChips, ...
                                startOffsetSamples, startFreqHz, ...
                                settings, shadowTrackValidateHist(ii, :));
                            trackShadow(ii) = localSetShadowPendingClusterMeta(trackShadow(ii), ...
                                useClusterCenter, shadowRawCandOffsetChips(ii, candIdxRetry), ...
                                shadowRawCandFreqHz(ii, candIdxRetry), shadowRawClusterOffset(ii, :), ...
                                shadowRawClusterFreqHz(ii, :), shadowRawClusterScore(ii, :), ...
                                shadowRawClusterHits(ii, :), settings);
                            shadowTrackRejectCnt(ii) = 0;
                            shadowTrackShortScore(ii) = 0;
                            shadowTrackLastImprove(ii) = nan;
                            shadowTrackRecheckCnt(ii) = settings.deepShadowRawTrackClusterRecheckBudget;
                            navResults.shadowRawTrackInit(ii, currMeasNr) = true;
                            navResults.shadowInitRecvTimeShiftMs(ii, currMeasNr) = initDiagRetry.recvTimeShiftMs;
                            navResults.shadowInitSampleShift(ii, currMeasNr) = initDiagRetry.sampleShift;
                            navResults.shadowInitNumCoIntShift(ii, currMeasNr) = initDiagRetry.numCoIntShift;
                            navResults.shadowRawInitClusterFirstUsed(ii, currMeasNr) = initStartDiag.clusterFirstUsed;
                            navResults.shadowRawInitBlendAlpha(ii, currMeasNr) = initStartDiag.blendAlpha;
                            navResults.shadowRawInitClusterScore(ii, currMeasNr) = initStartDiag.clusterScore;
                            navResults.shadowRawInitClusterHits(ii, currMeasNr) = initStartDiag.clusterHits;
                            navResults.shadowRawInitClusterMargin(ii, currMeasNr) = initStartDiag.clusterMargin;
                            navResults.shadowRawRelayCandIdx(ii, currMeasNr) = candIdxRetry;
                            navResults.shadowRawRelayCandOffset(ii, currMeasNr) = shadowRawCandOffsetChips(ii, candIdxRetry);
                            navResults.shadowRawRelayCandFreqHz(ii, currMeasNr) = shadowRawCandFreqHz(ii, candIdxRetry);
                            navResults.shadowRawRelayReason(ii, currMeasNr) = 1;
                        end
                    end
                elseif shadowTrackPendingAge(ii) >= settings.deepShadowRawTrackValidateMaxEpochs + ...
                        double(clusterBackedPending) * settings.deepShadowRawTrackClusterExtraMaxEpochs
                    if recheckProtect
                        shadowTrackValidateCnt(ii) = 0;
                        shadowTrackRejectCnt(ii) = 0;
                        shadowTrackShortScore(ii) = max(-1, shadowTrackShortScore(ii));
                        shadowTrackValidateHist(ii, :) = false;
                        shadowTrackLastImprove(ii) = nan;
                        shadowTrackPendingAge(ii) = max(0, shadowTrackPendingAge(ii) - 1);
                        shadowTrackRecheckCnt(ii) = shadowTrackRecheckCnt(ii) - 1;
                        if isfield(trackShadow(ii), 'deepShadowInitHold')
                            trackShadow(ii).deepShadowInitHold = max(trackShadow(ii).deepShadowInitHold, ...
                                settings.deepShadowRawTrackClusterRecheckHoldEpochs);
                        else
                            trackShadow(ii).deepShadowInitHold = settings.deepShadowRawTrackClusterRecheckHoldEpochs;
                        end
                    else
                    if shadowRawCandSelectIdx(ii) > 0
                        shadowRawFailExcludeValid(ii) = true;
                        shadowRawFailExcludeOffset(ii) = shadowRawCandOffsetChips(ii, shadowRawCandSelectIdx(ii));
                        shadowRawFailExcludeFreqHz(ii) = shadowRawCandFreqHz(ii, shadowRawCandSelectIdx(ii));
                        shadowRawFailExcludeAge(ii) = 0;
                    end
                    shadowTrackPending(ii) = false;
                    shadowTrackPendingAge(ii) = 0;
                    shadowTrackValidateCnt(ii) = 0;
                    shadowTrackRejectCnt(ii) = 0;
                    shadowTrackShortScore(ii) = 0;
                    shadowTrackLastImprove(ii) = nan;
                    shadowTrackRecheckCnt(ii) = 0;
                    shadowTrackValidateHist(ii, :) = false;
                    shadowTrackPassReady(ii) = false;
                    shadowPassReadyBadCnt(ii) = 0;
                    if shadowRawCandValid(ii)
                        shadowRawCandSelectIdx(ii) = 0;
                        shadowRawRelayCooldownCnt(ii) = settings.deepShadowRawRelayCooldownEpochs;
                        navResults.shadowRawRelayUsed(ii, currMeasNr) = true;
                        navResults.shadowRawFailExcludeOffset(ii, currMeasNr) = shadowRawFailExcludeOffset(ii);
                        navResults.shadowRawFailExcludeFreqHz(ii, currMeasNr) = shadowRawFailExcludeFreqHz(ii);
                        if settings.deepShadowRawTrackEnable && ~shadowTrackActive(ii) && ...
                                shadowRawRelayCooldownCnt(ii) <= 0
                            candIdxRetry = localPickShadowCandidateRelay(shadowRawCandOffsetChips(ii, :), ...
                                shadowRawCandFreqHz(ii, :), shadowRawCandMetric(ii, :), ...
                                shadowRawCandUsedMask(ii, :), shadowRawFailExcludeValid(ii), ...
                                shadowRawFailExcludeOffset(ii), shadowRawFailExcludeFreqHz(ii), settings);
                            if candIdxRetry > 0
                                shadowRawCandSelectIdx(ii) = candIdxRetry;
                                shadowRawCandUsedMask(ii, candIdxRetry) = true;
                                [useClusterCenter, startOffsetChips, startOffsetSamples, startFreqHz, initStartDiag] = ...
                                    localBuildShadowStartPoint(trackDeepIn(ii), ...
                                    shadowRawCandOffsetChips(ii, candIdxRetry), shadowRawCandOffsetSamples(ii, candIdxRetry), ...
                                    shadowRawCandFreqHz(ii, candIdxRetry), shadowRawClusterOffset(ii, :), ...
                                    shadowRawClusterFreqHz(ii, :), shadowRawClusterScore(ii, :), ...
                                    shadowRawClusterHits(ii, :), settings);
                                [trackShadow(ii), shadowTrackPending(ii), shadowTrackPendingAge(ii), ...
                                    shadowTrackValidateCnt(ii), shadowTrackValidateHist(ii, :), initDiagRetry] = ...
                                    localStartShadowPending(trackDeepIn(ii), startOffsetChips, ...
                                    startOffsetSamples, startFreqHz, ...
                                    settings, shadowTrackValidateHist(ii, :));
                                trackShadow(ii) = localSetShadowPendingClusterMeta(trackShadow(ii), ...
                                    useClusterCenter, shadowRawCandOffsetChips(ii, candIdxRetry), ...
                                    shadowRawCandFreqHz(ii, candIdxRetry), shadowRawClusterOffset(ii, :), ...
                                    shadowRawClusterFreqHz(ii, :), shadowRawClusterScore(ii, :), ...
                                    shadowRawClusterHits(ii, :), settings);
                                shadowTrackRejectCnt(ii) = 0;
                                shadowTrackShortScore(ii) = 0;
                                shadowTrackLastImprove(ii) = nan;
                                shadowTrackRecheckCnt(ii) = settings.deepShadowRawTrackClusterRecheckBudget;
                                navResults.shadowRawTrackInit(ii, currMeasNr) = true;
                                navResults.shadowInitRecvTimeShiftMs(ii, currMeasNr) = initDiagRetry.recvTimeShiftMs;
                                navResults.shadowInitSampleShift(ii, currMeasNr) = initDiagRetry.sampleShift;
                                navResults.shadowInitNumCoIntShift(ii, currMeasNr) = initDiagRetry.numCoIntShift;
                                navResults.shadowRawInitClusterFirstUsed(ii, currMeasNr) = initStartDiag.clusterFirstUsed;
                                navResults.shadowRawInitBlendAlpha(ii, currMeasNr) = initStartDiag.blendAlpha;
                                navResults.shadowRawInitClusterScore(ii, currMeasNr) = initStartDiag.clusterScore;
                                navResults.shadowRawInitClusterHits(ii, currMeasNr) = initStartDiag.clusterHits;
                                navResults.shadowRawInitClusterMargin(ii, currMeasNr) = initStartDiag.clusterMargin;
                                navResults.shadowRawRelayCandIdx(ii, currMeasNr) = candIdxRetry;
                                navResults.shadowRawRelayCandOffset(ii, currMeasNr) = shadowRawCandOffsetChips(ii, candIdxRetry);
                                navResults.shadowRawRelayCandFreqHz(ii, currMeasNr) = shadowRawCandFreqHz(ii, candIdxRetry);
                                navResults.shadowRawRelayReason(ii, currMeasNr) = 2;
                            end
                        end
                    end
                    end
                end
                navResults.shadowRawTrackValidateCounter(ii, currMeasNr) = shadowTrackValidateCnt(ii);
                navResults.shadowRawTrackValidateWindowHits(ii, currMeasNr) = windowHits;
                navResults.shadowRawTrackRejectCounter(ii, currMeasNr) = shadowTrackRejectCnt(ii);
                navResults.shadowRawTrackShortScore(ii, currMeasNr) = shadowTrackShortScore(ii);
            elseif settings.deepShadowMainSwitchEnable && deepModeState == 2
                allowSwitchEval = true;
                if settings.deepShadowMainSwitchWaitProtectDone && ...
                        isfield(trackShadow(ii), 'deepShadowInitHold') && ...
                        isfinite(trackShadow(ii).deepShadowInitHold) && ...
                        trackShadow(ii).deepShadowInitHold > 0
                    allowSwitchEval = false;
                end
                switchPass = navResults.shadowRawTrackImprove(ii, currMeasNr) >= settings.deepShadowMainSwitchImproveMinM;
                if localUseDs5TruePeak(settings, currMeasNr) && ii <= numel(ds5TrueRefReadyState) && ds5TrueRefReadyState(ii)
                    currentFreqHzLocal = trackDeepIn(ii).carrFreq - settings.IF;
                    currentCodeChipsLocal = localGetTrackField(trackDeepIn(ii), 'deepShadowCandOffsetChips', 0.0);
                    currentMetricLocal = localGetTrackField(trackDeepIn(ii), 'deepShadowCandMetric', nan);
                    candFreqHzLocal = trackShadow(ii).carrFreq - settings.IF;
                    candCodeChipsLocal = localGetTrackField(trackShadow(ii), 'deepShadowCandOffsetChips', nan);
                    candMetricLocal = localGetTrackField(trackShadow(ii), 'deepShadowCandMetric', nan);
                    ds5CurrentScore = localScoreDs5Peak(currentFreqHzLocal, currentCodeChipsLocal, ...
                        navResults.shadowRawTrackImprove(ii, currMeasNr), currentMetricLocal, ...
                        ds5TrueRefHzState(ii), ds5TrueRefCodeState(ii), settings);
                    ds5CandScore = localScoreDs5Peak(candFreqHzLocal, candCodeChipsLocal, ...
                        navResults.shadowRawTrackImprove(ii, currMeasNr), candMetricLocal, ...
                        ds5TrueRefHzState(ii), ds5TrueRefCodeState(ii), settings);
                    switchPass = localCandidateInDs5TrueWindow(candFreqHzLocal, candCodeChipsLocal, ...
                        ds5TrueRefHzState(ii), ds5TrueRefCodeState(ii), settings) && ...
                        isfinite(ds5CandScore) && ds5CandScore >= ds5CurrentScore + ...
                        localGetSettingValue(settings, 'deepShadowDs5TruePeakCandidateMargin', 0.10);
                    navResults.shadowDs5CandInTrueWindow(ii, currMeasNr) = switchPass;
                    navResults.shadowDs5CandScore(ii, currMeasNr) = ds5CandScore;
                    navResults.shadowDs5CurrentScore(ii, currMeasNr) = ds5CurrentScore;
                    navResults.shadowDs5CandBetterThanCurrent(ii, currMeasNr) = switchPass;
                end
                if allowSwitchEval && switchPass
                    shadowMainSwitchCnt(ii) = shadowMainSwitchCnt(ii) + 1;
                else
                    shadowMainSwitchCnt(ii) = 0;
                end
                if shadowMainSwitchCnt(ii) >= settings.deepShadowMainSwitchConfirmEpochs
                    shadowTakeoverBackup(ii) = trackDeepIn(ii);
                    shadowTakeoverBackupValid(ii) = true;
                    trackDeepIn(ii) = localSyncTrackState(trackDeepIn(ii), trackShadow(ii));
                    shadowTrackActive(ii) = true;
                    shadowMainSwitchCnt(ii) = 0;
                    shadowMainHoldCnt(ii) = settings.deepShadowMainSwitchHoldEpochs;
                    shadowMainFollowActive(ii) = true;
                    shadowTakeoverRevokeCnt(ii) = 0;
                    navResults.shadowMainSwitch(ii, currMeasNr) = true;
                    navResults.shadowDs5Takeover(ii, currMeasNr) = true;
                end
                if shadowMainFollowActive(ii) && localUseDs5TruePeak(settings, currMeasNr) && ii <= numel(ds5TrueRefReadyState) && ds5TrueRefReadyState(ii)
                    liveFreqErrHz = abs((trackShadow(ii).carrFreq - settings.IF) - ds5TrueRefHzState(ii));
                    liveCodeErrChips = abs(localGetTrackField(trackShadow(ii), 'deepShadowCandOffsetChips', nan) - ds5TrueRefCodeState(ii));
                    revokeNow = (isfinite(liveFreqErrHz) && liveFreqErrHz > localGetSettingValue(settings, 'deepShadowDs5TruePeakRevokeFreqErrHz', 180.0)) || ...
                        (isfinite(liveCodeErrChips) && liveCodeErrChips > localGetSettingValue(settings, 'deepShadowDs5TruePeakRevokeCodeErrChips', 1.25)) || ...
                        (isfinite(navResults.shadowRawTrackImprove(ii, currMeasNr)) && ...
                         navResults.shadowRawTrackImprove(ii, currMeasNr) < localGetSettingValue(settings, 'deepShadowDs5TruePeakRevokeImproveMaxM', -150.0));
                    if revokeNow
                        shadowTakeoverRevokeCnt(ii) = shadowTakeoverRevokeCnt(ii) + 1;
                    else
                        shadowTakeoverRevokeCnt(ii) = 0;
                    end
                    if shadowTakeoverRevokeCnt(ii) >= localGetSettingValue(settings, 'deepShadowDs5TruePeakRevokeBadEpochs', 2)
                        if shadowTakeoverBackupValid(ii)
                            trackDeepIn(ii) = shadowTakeoverBackup(ii);
                        end
                        shadowMainFollowActive(ii) = false;
                        shadowMainHoldCnt(ii) = 0;
                        shadowTrackActive(ii) = false;
                        shadowTrackPending(ii) = false;
                        shadowTrackPendingAge(ii) = 0;
                        shadowTrackValidateCnt(ii) = 0;
                        shadowTrackValidateHist(ii, :) = false;
                        shadowTakeoverRevokeCnt(ii) = 0;
                        navResults.shadowDs5TakeoverRevoke(ii, currMeasNr) = true;
                    end
                end
            end
        end
    end
    if ~lightNormalMode
        shadowBranchAmbigHoldAge(isfinite(shadowBranchAmbigHoldAge)) = ...
            shadowBranchAmbigHoldAge(isfinite(shadowBranchAmbigHoldAge)) + 1;
        expiredAmbigHold = shadowBranchAmbigHoldAge > settings.deepShadowBranchAmbigContinuityMaxAgeEpochs;
        shadowBranchAmbigHoldChips(expiredAmbigHold) = nan;
        shadowBranchAmbigHoldAge(expiredAmbigHold) = inf;
        if settings.deepShadowRawPassUseActivePool
            passCandidateMask = shadowTrackActive(:);
        else
            passCandidateMask = shadowTrackPassReady(:);
        end
        passShadowIdx = find(passCandidateMask);
        navResults.shadowRawPassSatNum(1, currMeasNr) = numel(passShadowIdx);
        navResults.shadowRawPassReadyMask(:, currMeasNr) = passCandidateMask(:);
        branchTakeoverCandidateNow = false;
        if numel(passShadowIdx) >= 2
            mainSampleClockBaseRef = median(navSolut.sampleClockBase, 'omitnan');
            diagLocalTic = tic;
            settingsShadowNav = settings;
            if settings.deepShadowCoreServoEnable
                settingsShadowNav.deepCodeReacqNavCorrEnable = 1;
            end
            navSolutShadowPass = postNavTight(trackShadow(passShadowIdx), settingsShadowNav, eph, TOW, mainSampleClockBaseRef);
            [rhoShadowPass, ~, ~, ~] = rhoSatRec_zcj(navSolutShadowPass.satPositions', posxyz, ...
                navSolutShadowPass.rawP', navSolutShadowPass.satVelocity', ins.vn);
            diagShadowNavTimeSec = diagShadowNavTimeSec + toc(diagLocalTic);
            deltaShadowPassRaw = navSolutShadowPass.rawP(:) + settings.c * navSolutShadowPass.satClkCorr(:) - rhoShadowPass;
            [deltaShadowPass, ambigChipsLocal] = localResolveChipAmbiguity(deltaShadowPassRaw, settings);
            navResults.shadowRawPassLiftDeltaRawP(passShadowIdx, currMeasNr) = deltaShadowPass(:);
            navResults.shadowRawPassAmbigChips(passShadowIdx, currMeasNr) = ambigChipsLocal(:);
            navResults.shadowRawPassLiftRawP(passShadowIdx, currMeasNr) = ...
                navSolutShadowPass.rawP(:) - ambigChipsLocal(:) * localChipMeters(settings);
            navResults.shadowRawPassRawP(passShadowIdx, currMeasNr) = navSolutShadowPass.rawP(:);
            navResults.shadowRawPassServoRawP(passShadowIdx, currMeasNr) = navSolutShadowPass.rawP(:);
            navResults.shadowRawPassRawPdot(passShadowIdx, currMeasNr) = navSolutShadowPass.rawP_dot(:);
            navResults.shadowRawPassSatClkCorr(passShadowIdx, currMeasNr) = navSolutShadowPass.satClkCorr(:);
            navResults.shadowRawPassSatClkDrift(passShadowIdx, currMeasNr) = navSolutShadowPass.satClkDrift(:);
            navResults.shadowRawPassSatPosX(passShadowIdx, currMeasNr) = navSolutShadowPass.satPositions(1,:).';
            navResults.shadowRawPassSatPosY(passShadowIdx, currMeasNr) = navSolutShadowPass.satPositions(2,:).';
            navResults.shadowRawPassSatPosZ(passShadowIdx, currMeasNr) = navSolutShadowPass.satPositions(3,:).';
            navResults.shadowRawPassSatVelX(passShadowIdx, currMeasNr) = navSolutShadowPass.satVelocity(1,:).';
            navResults.shadowRawPassSatVelY(passShadowIdx, currMeasNr) = navSolutShadowPass.satVelocity(2,:).';
            navResults.shadowRawPassSatVelZ(passShadowIdx, currMeasNr) = navSolutShadowPass.satVelocity(3,:).';
            closedLiftAnchorPos = nan(3,1);
            closedLiftAnchorSource = 0;
            if all(isfinite(shadowClosedLoopLastPos))
                closedLiftAnchorPos = shadowClosedLoopLastPos(:);
                closedLiftAnchorSource = 1;
                if settings.deepShadowClosedLoopUseVelocityCoast && all(isfinite(shadowClosedLoopLastVel)) && isfinite(shadowClosedLoopLastEpoch)
                    closedLiftDt = (currMeasNr - shadowClosedLoopLastEpoch) * settings.navSolPeriod / 1000;
                    if isfinite(closedLiftDt) && closedLiftDt >= 0
                        closedLiftAnchorPos = closedLiftAnchorPos + closedLiftDt * shadowClosedLoopLastVel(:);
                    end
                end
            elseif trustedAnchorValid && all(isfinite(trustedAnchorPos))
                closedLiftAnchorPos = trustedAnchorPos(:);
                closedLiftAnchorSource = 2;
                if all(isfinite(trustedAnchorVel)) && isfinite(trustedAnchorTimeSec)
                    trustedDt = epochElapsedSec - trustedAnchorTimeSec;
                    if isfinite(trustedDt) && trustedDt >= 0
                        closedLiftAnchorPos = closedLiftAnchorPos + trustedDt * trustedAnchorVel(:);
                    end
                end
            else
                closedLiftAnchorPos = posxyz(:);
                closedLiftAnchorSource = 3;
            end
            closedLiftDelta = nan(numel(passShadowIdx), 1);
            qualityLocal = zeros(numel(passShadowIdx), 1);
            passPrnLocal = [trackDeepIn(passShadowIdx).PRN].';
            if all(isfinite(closedLiftAnchorPos))
                prevAmbigLocal = closedLiftPrevAmbigChips(passShadowIdx);
                lastDeltaLocal = closedLiftLastDelta(passShadowIdx);
                segmentLocal = closedLiftSegmentId(passShadowIdx);
                prnLocal = navResults.prnList(passShadowIdx);
                [closedLiftRawP, closedLiftAmbigChips, closedLiftDelta, closedLiftDeltaAbs, segmentLocal, qualityLocal, repairChipsLocal] = localLiftRawPassToAnchor( ...
                    navSolutShadowPass.rawP(:), navSolutShadowPass.satClkCorr(:), ...
                    navSolutShadowPass.satPositions, closedLiftAnchorPos, settings, ...
                    prevAmbigLocal, lastDeltaLocal, segmentLocal, prnLocal);
                navResults.shadowRawPassClosedLiftRawP(passShadowIdx, currMeasNr) = closedLiftRawP(:);
                navResults.shadowRawPassClosedLiftAmbigChips(passShadowIdx, currMeasNr) = closedLiftAmbigChips(:);
                navResults.shadowRawPassClosedLiftDeltaRawP(passShadowIdx, currMeasNr) = closedLiftDelta(:);
                navResults.shadowRawPassClosedLiftDeltaAbsM(passShadowIdx, currMeasNr) = closedLiftDeltaAbs(:);
                navResults.shadowRawPassClosedLiftSegmentId(passShadowIdx, currMeasNr) = segmentLocal(:);
                navResults.shadowRawPassClosedLiftQuality(passShadowIdx, currMeasNr) = qualityLocal(:);
                navResults.shadowRawPassClosedLiftRepairChips(passShadowIdx, currMeasNr) = repairChipsLocal(:);
                goodClosedLiftLocal = isfinite(closedLiftDelta(:)) & isfinite(closedLiftAmbigChips(:)) & qualityLocal(:) >= 1;
                tmpAmbig = closedLiftPrevAmbigChips(passShadowIdx);
                tmpDelta = closedLiftLastDelta(passShadowIdx);
                tmpSeg = closedLiftSegmentId(passShadowIdx);
                tmpEpoch = closedLiftLastGoodEpoch(passShadowIdx);
                tmpOffset = closedLiftLastOffsetToBase(passShadowIdx);
                focusPrnsLocal = localGetSettingValue(settings, 'deepShadowDs5TailSourceFallbackPrns', [27 3]);
                focusMaskLocal = ismember(passPrnLocal(:), focusPrnsLocal(:));
                [closedLiftBaseMed, closedLiftBaseCnt] = localComputeShadowNavBaseMed(deltaShadowPass, passPrnLocal, settings);
                tmpAmbig(goodClosedLiftLocal) = closedLiftAmbigChips(goodClosedLiftLocal);
                tmpEpoch(goodClosedLiftLocal) = currMeasNr;
                tmpDelta(goodClosedLiftLocal) = closedLiftDeltaAbs(goodClosedLiftLocal);
                goodClosedLiftFocus = goodClosedLiftLocal & focusMaskLocal;
                if any(goodClosedLiftFocus) && isfinite(closedLiftBaseMed) && closedLiftBaseCnt >= 2
                    tmpOffset(goodClosedLiftFocus) = closedLiftDeltaAbs(goodClosedLiftFocus) - closedLiftBaseMed;
                end
                tmpSeg(isfinite(segmentLocal)) = segmentLocal(isfinite(segmentLocal));
                closedLiftPrevAmbigChips(passShadowIdx) = tmpAmbig;
                closedLiftLastDelta(passShadowIdx) = tmpDelta;
                closedLiftLastGoodEpoch(passShadowIdx) = tmpEpoch;
                closedLiftLastOffsetToBase(passShadowIdx) = tmpOffset;
                closedLiftSegmentId(passShadowIdx) = tmpSeg;
                navResults.shadowRawPassClosedLiftAnchorSource(1, currMeasNr) = closedLiftAnchorSource;
            end
            [refObsMaskLocal, refObsDeltaLocal, refObsInfoLocal] = localBuildDs5ReferenceRelativeObs( ...
                deltaShadowPass, nan, nan, passShadowIdx, passPrnLocal, trackShadow, ...
                ds5TrueRefCodeState, ds5TrueRefReadyState, ds5TrueRefHzState, settings);
            navResults.shadowDs5RefObsUsed(passShadowIdx, currMeasNr) = refObsMaskLocal(:);
            navResults.shadowDs5RefObsDeltaM(passShadowIdx, currMeasNr) = refObsDeltaLocal(:);
            navResults.shadowDs5RefObsBaseDeltaM(passShadowIdx, currMeasNr) = refObsInfoLocal.baseDeltaM(:);
            navResults.shadowDs5RefObsSeedDeltaM(passShadowIdx, currMeasNr) = refObsInfoLocal.seedDeltaM(:);
            navResults.shadowDs5RefObsSeedUseBase(passShadowIdx, currMeasNr) = refObsInfoLocal.seedUseBase(:);
            navResults.shadowDs5RefObsLocalCorrChips(passShadowIdx, currMeasNr) = refObsInfoLocal.localCorrChips(:);
            navResults.shadowDs5RefObsFreqErrHz(passShadowIdx, currMeasNr) = refObsInfoLocal.freqErrHz(:);
            navResults.shadowDs5RefObsCodeErrChips(passShadowIdx, currMeasNr) = refObsInfoLocal.codeErrChips(:);
            obsContractClockPriorM = localGetDs5ClockPriorM(navResults, currMeasNr, shadowRecoveredFilterClockM);
            [~, navResults] = localEvaluateDs5ObservationContract(currMeasNr, navResults, settings, obsContractClockPriorM);
            [deltaShadowPassNav, navSourceModeLocal, navSourceRefDeltaLocal] = localSelectShadowNavDeltaSource( ...
                deltaShadowPass, closedLiftDeltaAbs, qualityLocal, passPrnLocal, ...
                closedLiftLastDelta(passShadowIdx), closedLiftLastGoodEpoch(passShadowIdx), ...
                closedLiftLastOffsetToBase(passShadowIdx), currMeasNr, settings, ...
                refObsMaskLocal, refObsDeltaLocal);
            navResults.shadowRawPassNavDeltaUsed(passShadowIdx, currMeasNr) = deltaShadowPassNav(:);
            navResults.shadowRawPassNavSourceMode(passShadowIdx, currMeasNr) = navSourceModeLocal(:);
            navResults.shadowRawPassNavSourceRefDelta(passShadowIdx, currMeasNr) = navSourceRefDeltaLocal(:);
            navResults.shadowRawPassNavDeltaMed(1, currMeasNr) = median(deltaShadowPassNav, 'omitnan');
            if any(isfinite(deltaShadowPassNav))
                navResults.shadowRawPassNavDeltaP95(1, currMeasNr) = prctile(deltaShadowPassNav(isfinite(deltaShadowPassNav)), 95);
            end
            [deltaShadowPassNavDetrended, ds5CommonInfoLocal, ds5CommonDragM, ds5CommonDragRateMps, ds5CommonDragLastEpoch] = ...
                localApplyDs5CommonDrag(deltaShadowPassNav, passPrnLocal, navSourceModeLocal, currMeasNr, ...
                ds5CommonDragM, ds5CommonDragRateMps, ds5CommonDragLastEpoch, settings);
            navResults.shadowRawPassNavDeltaDetrended(passShadowIdx, currMeasNr) = deltaShadowPassNavDetrended(:);
            if isstruct(ds5CommonInfoLocal)
                navResults.shadowRawPassDs5CommonApplied(1, currMeasNr) = ds5CommonInfoLocal.applied;
                navResults.shadowRawPassDs5CommonDragM(1, currMeasNr) = ds5CommonInfoLocal.commonM;
                navResults.shadowRawPassDs5CommonRateMps(1, currMeasNr) = ds5CommonInfoLocal.rateMps;
                navResults.shadowRawPassDs5CommonBaseSatNum(1, currMeasNr) = ds5CommonInfoLocal.baseSatNum;
                navResults.shadowRawPassDs5CommonKeepSatNum(1, currMeasNr) = ds5CommonInfoLocal.keepSatNum;
                navResults.shadowRawPassDs5CommonSource(1, currMeasNr) = ds5CommonInfoLocal.source;
                navResults.shadowRawPassDs5CommonPredM(1, currMeasNr) = ds5CommonInfoLocal.predM;
                navResults.shadowRawPassDs5CommonMeasM(1, currMeasNr) = ds5CommonInfoLocal.measM;
                navResults.shadowRawPassDs5CommonP95BeforeM(1, currMeasNr) = ds5CommonInfoLocal.p95BeforeM;
                navResults.shadowRawPassDs5CommonP95AfterM(1, currMeasNr) = ds5CommonInfoLocal.p95AfterM;
            end
            if localUseDs5TruePeak(settings, currMeasNr)
                chipMetersLocal = localChipMeters(settings);
                codeRefLocal = nan(size(deltaShadowPassNavDetrended));
                goodCodeRefLocal = isfinite(deltaShadowPassNavDetrended) & abs(deltaShadowPassNavDetrended) <= ...
                    localGetSettingValue(settings, 'deepShadowDs5TruePeakCodeWindowMaxChips', 1.50) * chipMetersLocal;
                codeRefLocal(goodCodeRefLocal) = -deltaShadowPassNavDetrended(goodCodeRefLocal) / max(chipMetersLocal, eps);
                ds5TrueRefCodeState(passShadowIdx(goodCodeRefLocal)) = codeRefLocal(goodCodeRefLocal);
                ds5TrueRefReadyState(passShadowIdx(goodCodeRefLocal)) = true;
            end
            [deltaShadowPassNavGate, ds5DetrendedGateInfoLocal] = ...
                localApplyDs5DetrendedOutlierGate(deltaShadowPassNavDetrended, passPrnLocal, navSourceModeLocal, currMeasNr, settings);
            navResults.shadowRawPassDs5DetrendedGateMask(passShadowIdx, currMeasNr) = ds5DetrendedGateInfoLocal.keepMask(:);
            navResults.shadowRawPassDs5DetrendedDevM(passShadowIdx, currMeasNr) = ds5DetrendedGateInfoLocal.devM(:);
            navResults.shadowRawPassDs5DetrendedGateM(1, currMeasNr) = ds5DetrendedGateInfoLocal.gateM;
            navResults.shadowRawPassDs5DetrendedRejectNum(1, currMeasNr) = ds5DetrendedGateInfoLocal.rejectNum;
            [navQualifiedMaskLocal, navCoreMaskLocal, navExpandMaskLocal, navCoreMaxDevM, navExpandMaxDevM, navRelaxInfoLocal] = ...
                localSelectNavQualifiedSubset(deltaShadowPassNavGate, passPrnLocal, currMeasNr, settings);
            if settings.deepShadowRawPassReadyAgingEnable && numel(passShadowIdx) >= settings.deepShadowRawPassReadyMinSet
                finitePass = isfinite(deltaShadowPassNavGate);
                if sum(finitePass) >= settings.deepShadowRawPassReadyMinSet
                    coreMed = median(deltaShadowPassNavGate(navQualifiedMaskLocal & finitePass), 'omitnan');
                    if ~isfinite(coreMed)
                        coreMed = median(deltaShadowPassNavGate(finitePass), 'omitnan');
                    end
                    passDev = abs(deltaShadowPassNavGate - coreMed);
                    navResults.shadowRawPassReadyDeltaDev(passShadowIdx, currMeasNr) = passDev;
                    qualityBadLocal = finitePass & ...
                        (~navQualifiedMaskLocal | ...
                         passDev > settings.deepShadowRawPassReadyDevThrM | ...
                         abs(deltaShadowPassNavGate) > settings.deepShadowRawPassReadyAbsThrM);
                    qualityGoodLocal = finitePass & ~qualityBadLocal;
                    shadowPassReadyBadCnt(passShadowIdx(qualityBadLocal)) = ...
                        shadowPassReadyBadCnt(passShadowIdx(qualityBadLocal)) + 1;
                    shadowPassReadyBadCnt(passShadowIdx(qualityGoodLocal)) = 0;
                    revokeLocal = qualityBadLocal & ...
                        (shadowPassReadyBadCnt(passShadowIdx) >= settings.deepShadowRawPassReadyBadEpochs);
                    if any(revokeLocal)
                        revokeIdx = passShadowIdx(revokeLocal);
                        navResults.shadowRawPassReadyRevoked(revokeIdx, currMeasNr) = true;
                        if settings.deepShadowRawPassReadyRevokeEnable
                            shadowTrackPassReady(revokeIdx) = false;
                            shadowPassReadyBadCnt(revokeIdx) = 0;
                            navQualifiedMaskLocal(revokeLocal) = false;
                            navResults.shadowRawPassReadyMask(revokeIdx, currMeasNr) = false;
                            passShadowIdx = passShadowIdx(~revokeLocal);
                            passPrnLocal = passPrnLocal(~revokeLocal);
                            deltaShadowPass = deltaShadowPass(~revokeLocal);
                            deltaShadowPassNav = deltaShadowPassNav(~revokeLocal);
                            deltaShadowPassNavDetrended = deltaShadowPassNavDetrended(~revokeLocal);
                            deltaShadowPassNavGate = deltaShadowPassNavGate(~revokeLocal);
                            navSourceModeLocal = navSourceModeLocal(~revokeLocal);
                            navQualifiedMaskLocal = navQualifiedMaskLocal(~revokeLocal);
                            navResults.shadowRawPassSatNum(1, currMeasNr) = numel(passShadowIdx);
                        else
                            % Tracking-recovery mode: keep active/pass-ready tracks alive.
                            % Bad stars are excluded from this epoch's nav-qualified subset
                            % but can re-enter automatically when they agree again.
                            navQualifiedMaskLocal(revokeLocal) = false;
                        end
                    end
                end
            end
            navResults.shadowRawPassReadyQualityBadCnt(:, currMeasNr) = shadowPassReadyBadCnt(:);
            navResults.shadowRawNavQualifiedSatNum(1, currMeasNr) = sum(navQualifiedMaskLocal);
            navResults.shadowRawNavCoreSatNum(1, currMeasNr) = sum(navCoreMaskLocal);
            navResults.shadowRawNavExpandSatNum(1, currMeasNr) = sum(navExpandMaskLocal);
            navResults.shadowRawNavCoreMaxDevM(1, currMeasNr) = navCoreMaxDevM;
            navResults.shadowRawNavExpandMaxDevM(1, currMeasNr) = navExpandMaxDevM;
            if isstruct(navRelaxInfoLocal)
                navResults.shadowRawNavRelaxDs5Used(passShadowIdx, currMeasNr) = navRelaxInfoLocal.used(:);
                navResults.shadowRawNavRelaxDs5CoreUsed(passShadowIdx, currMeasNr) = navRelaxInfoLocal.coreUsed(:);
                navResults.shadowRawNavRelaxDs5TargetMask(passShadowIdx, currMeasNr) = navRelaxInfoLocal.targetMask(:);
                navResults.shadowRawNavRelaxDs5BaseMask(passShadowIdx, currMeasNr) = navRelaxInfoLocal.baseMask(:);
                navResults.shadowRawNavRelaxDs5DevM(passShadowIdx, currMeasNr) = navRelaxInfoLocal.devM(:);
                navResults.shadowRawNavRelaxDs5PairOk(passShadowIdx, currMeasNr) = navRelaxInfoLocal.pairOk(:);
                navResults.shadowRawNavRelaxDs5BaseMedM(1, currMeasNr) = navRelaxInfoLocal.baseMedM;
                navResults.shadowRawNavRelaxDs5NavThrM(1, currMeasNr) = navRelaxInfoLocal.navThrM;
                navResults.shadowRawNavRelaxDs5CoreThrM(1, currMeasNr) = navRelaxInfoLocal.coreThrM;
                navResults.shadowRawNavRelaxDs5PairLeadPrn(1, currMeasNr) = navRelaxInfoLocal.pairLeadPrn;
            end
            navQualifiedIdx = passShadowIdx(navQualifiedMaskLocal);
            navCoreIdx = passShadowIdx(navCoreMaskLocal);
            navExpandIdx = passShadowIdx(navExpandMaskLocal);
            navResults.shadowRawNavQualifiedMask(navQualifiedIdx, currMeasNr) = true;
            navResults.shadowRawNavCoreMask(navCoreIdx, currMeasNr) = true;
            navResults.shadowRawNavExpandMask(navExpandIdx, currMeasNr) = true;
            if settings.deepShadowBranchSearchEnable
                branchMaskLocal = navCoreMaskLocal;
                dopplerGateResidualLocal = nan(numel(passShadowIdx), 1);
                if settings.deepShadowBranchUseNavQ
                    branchMaskLocal = navQualifiedMaskLocal;
                end
                if settings.deepShadowBranchDopplerGateEnable
                    if trustedAnchorValid && all(isfinite(trustedAnchorPos))
                        dopplerPriorPos = trustedAnchorPos(:);
                        dopplerPriorVel = trustedAnchorVel(:);
                    else
                        dopplerPriorPos = posxyz(:);
                        dopplerPriorVel = localNedVelToEcef(ins.pos, ins.vn);
                    end
                    [dopplerGateKeepLocal, dopplerGateResidualLocal, dopplerGateSource, dopplerGateSign] = ...
                        localShadowBranchDopplerGate(passShadowIdx, currMeasNr, navResults, ...
                        navSolutShadowPass, dopplerPriorPos, dopplerPriorVel, settings);
                    navResults.shadowBranchDopplerGateMask(passShadowIdx(dopplerGateKeepLocal), currMeasNr) = true;
                    navResults.shadowBranchDopplerGateResidualMps(passShadowIdx, currMeasNr) = dopplerGateResidualLocal(:);
                    navResults.shadowBranchDopplerGateSource(1, currMeasNr) = dopplerGateSource;
                    navResults.shadowBranchDopplerGateSign(1, currMeasNr) = dopplerGateSign;
                    navResults.shadowBranchDopplerGateKeepFrac(1, currMeasNr) = ...
                        sum(dopplerGateKeepLocal) / max(1, numel(dopplerGateKeepLocal));
                    branchMaskLocal = localApplyDopplerGateToBranchMask(branchMaskLocal, ...
                        dopplerGateKeepLocal, dopplerGateResidualLocal, settings.deepShadowBranchMinSat);
                end
                branchLocalIdx = find(branchMaskLocal);
                if numel(branchLocalIdx) >= settings.deepShadowBranchMinSat
                    branchRawPInput = navSolutShadowPass.rawP(branchLocalIdx);
                    branchCommonApplied = false;
                    branchCommonM = nan;
                    if localUseDs5CommonDrag(settings, currMeasNr) && isstruct(ds5CommonInfoLocal) && ds5CommonInfoLocal.applied && isfinite(ds5CommonInfoLocal.commonM)
                        branchCommonApplied = true;
                        branchCommonM = ds5CommonInfoLocal.commonM;
                    end
                    navResults.shadowBranchDs5CommonApplied(1, currMeasNr) = branchCommonApplied;
                    navResults.shadowBranchDs5CommonDragM(1, currMeasNr) = branchCommonM;
                    satPosBranch = navSolutShadowPass.satPositions(:, branchLocalIdx);
                    branchAbsoluteRefPos = posxyz(:);
                    if localIsDs5Scenario(settings)
                        branchAbsoluteRefPos = nan(3,1);
                    end
                    if localUseDs5BaselineTrustedRef(settings, currMeasNr) && baselineOutputValid && all(isfinite(baselineOutputPos))
                        branchAbsoluteRefPos = baselineOutputPos(:);
                    elseif trustedAnchorValid && all(isfinite(trustedAnchorPos))
                        branchAbsoluteRefPos = trustedAnchorPos(:);
                    end
                    priors = localBuildTrustedBranchPriors(branchAbsoluteRefPos, trustedAnchorValid, trustedAnchorPos, ...
                        trustedAnchorVel, trustedAnchorTimeSec, epochElapsedSec, trustedAnchorSource, settings);
                    branchLocalGlobalIdx = passShadowIdx(branchLocalIdx);
                    branchPrevAmbig = shadowBranchAmbigHoldChips(branchLocalGlobalIdx);
                    branchDetrendedLocal = deltaShadowPassNavDetrended(branchLocalIdx);
                    branchAbsModelP95 = nan;
                    branchModelMode = 1;
                    branchDeltaLocal = [];
                    if localIsDs5Scenario(settings)
                        branchDeltaLocal = localRebaseShadowNavDelta(deltaShadowPassNav(branchLocalIdx), satPosBranch, posxyz(:), branchAbsoluteRefPos);
                        [branchObsKeepMask, branchObsGateInfo] = localBuildDs5BranchObsConsistencyMask(branchDeltaLocal, satPosBranch, branchAbsoluteRefPos, settings);
                        navResults.shadowBranchObsGateKeepNum(1, currMeasNr) = branchObsGateInfo.keepNum;
                        navResults.shadowBranchObsGateRejectNum(1, currMeasNr) = branchObsGateInfo.rejectNum;
                        navResults.shadowBranchObsGateM(1, currMeasNr) = branchObsGateInfo.gateM;
                        navResults.shadowBranchObsRefDetP95M(1, currMeasNr) = branchObsGateInfo.refDetP95M;
                        if any(~branchObsKeepMask) && sum(branchObsKeepMask) >= settings.deepShadowBranchMinSat
                            branchLocalIdx = branchLocalIdx(branchObsKeepMask);
                            branchLocalGlobalIdx = branchLocalGlobalIdx(branchObsKeepMask);
                            branchRawPInput = branchRawPInput(branchObsKeepMask);
                            satPosBranch = satPosBranch(:, branchObsKeepMask);
                            branchPrevAmbig = branchPrevAmbig(branchObsKeepMask);
                            branchDetrendedLocal = branchDetrendedLocal(branchObsKeepMask);
                            branchDeltaLocal = branchDeltaLocal(branchObsKeepMask);
                        end
                        [branchRawP, branchAmbig, priorDet, branchPos, branchRes, branchPdop, priorSource, priorPos, branchUseMask, branchScore, branchDetrendedP95, branchDetrendedPenalty, branchAbsModelP95] = ...
                            localSelectBestBranchPriorDs5Absolute(branchRawPInput, branchDeltaLocal, ...
                            navSolutShadowPass.satClkCorr(branchLocalIdx), satPosBranch, branchAbsoluteRefPos, priors, settings, ...
                            dopplerGateResidualLocal(branchLocalIdx), branchPrevAmbig, branchDetrendedLocal, currMeasNr);
                        branchModelMode = 2;
                    else
                        if localUseDs5CommonDrag(settings, currMeasNr) && isstruct(ds5CommonInfoLocal) && ds5CommonInfoLocal.applied && isfinite(ds5CommonInfoLocal.commonM)
                            branchRawPInput = branchRawPInput - ds5CommonInfoLocal.commonM;
                        end
                        [branchRawP, branchAmbig, priorDet, branchPos, branchRes, branchPdop, priorSource, priorPos, branchUseMask, branchScore, branchDetrendedP95, branchDetrendedPenalty] = ...
                            localSelectBestBranchPrior(branchRawPInput, ...
                            navSolutShadowPass.satClkCorr(branchLocalIdx), satPosBranch, priors, settings, ...
                            dopplerGateResidualLocal(branchLocalIdx), branchPrevAmbig, branchDetrendedLocal, currMeasNr);
                    end
                    navResults.shadowBranchDs5DetrendedP95M(1, currMeasNr) = branchDetrendedP95;
                    navResults.shadowBranchDs5DetrendedScorePenaltyM(1, currMeasNr) = branchDetrendedPenalty;
                    navResults.shadowBranchAbsModelP95M(1, currMeasNr) = branchAbsModelP95;
                    navResults.shadowBranchModelMode(1, currMeasNr) = branchModelMode;
                    if all(isfinite(branchPos(1:4)))
                        branchUseMask = logical(branchUseMask(:));
                        if numel(branchUseMask) ~= numel(branchLocalIdx)
                            branchUseMask = true(numel(branchLocalIdx), 1);
                        end
                        branchGlobalIdx = passShadowIdx(branchLocalIdx(branchUseMask));
                        branchPrevSelected = shadowBranchAmbigHoldChips(branchGlobalIdx);
                        navResults.shadowBranchRawP(branchGlobalIdx, currMeasNr) = branchRawP(branchUseMask);
                        navResults.shadowBranchAmbigChips(branchGlobalIdx, currMeasNr) = branchAmbig(branchUseMask);
                        navResults.shadowBranchAmbigPrevChips(branchGlobalIdx, currMeasNr) = branchPrevSelected;
                        navResults.shadowBranchAmbigStepChips(branchGlobalIdx, currMeasNr) = branchAmbig(branchUseMask) - branchPrevSelected(:).';
                        navResults.shadowBranchAmbigContinuityUsed(branchGlobalIdx, currMeasNr) = isfinite(branchPrevSelected);
                        shadowBranchAmbigHoldChips(branchGlobalIdx) = branchAmbig(branchUseMask);
                        shadowBranchAmbigHoldAge(branchGlobalIdx) = 0;
                        navResults.shadowBranchSubsetMask(branchGlobalIdx, currMeasNr) = true;
                        navResults.shadowBranchSatNum(1, currMeasNr) = numel(branchGlobalIdx);
                        navResults.shadowBranchRobustScore(1, currMeasNr) = branchScore;
                        navResults.shadowBranchX(1, currMeasNr) = branchPos(1);
                        navResults.shadowBranchY(1, currMeasNr) = branchPos(2);
                        navResults.shadowBranchZ(1, currMeasNr) = branchPos(3);
                        navResults.shadowBranchClockM(1, currMeasNr) = branchPos(4);
                        navResults.shadowBranchPriorRmsM(1, currMeasNr) = sqrt(mean(priorDet.^2, 'omitnan'));
                        navResults.shadowBranchPostfitRmsM(1, currMeasNr) = sqrt(mean(branchRes.^2, 'omitnan'));
                        navResults.shadowBranchPdop(1, currMeasNr) = branchPdop;
                        navResults.shadowBranchPosJumpM(1, currMeasNr) = norm(branchPos(1:3) - priorPos(:));
                        navResults.shadowBranchAmbigMedChips(1, currMeasNr) = median(abs(branchAmbig), 'omitnan');
                        navResults.shadowBranchPriorSource(1, currMeasNr) = priorSource;
                        navResults.shadowBranchPriorX(1, currMeasNr) = priorPos(1);
                        navResults.shadowBranchPriorY(1, currMeasNr) = priorPos(2);
                        navResults.shadowBranchPriorZ(1, currMeasNr) = priorPos(3);
                        branchAbsoluteRefValid = localIsDs5Scenario(settings) && all(isfinite(branchAbsoluteRefPos));
                        branchAbsoluteRefDiffM = nan;
                        branchAbsoluteGateEnable = localGetSettingValue(settings, 'deepShadowDs5BranchAbsoluteConsistencyEnable', 0) ~= 0;
                        branchAbsoluteConsistent = ~branchAbsoluteGateEnable;
                        if branchAbsoluteRefValid
                            branchAbsoluteRefDiffM = norm(branchPos(1:3) - branchAbsoluteRefPos(:));
                            if branchAbsoluteGateEnable
                                branchAbsoluteConsistent = isfinite(branchAbsoluteRefDiffM) && ...
                                    branchAbsoluteRefDiffM <= localGetSettingValue(settings, 'deepShadowDs5BranchAbsoluteConsistencyMaxDiffM', 1200.0);
                            else
                                branchAbsoluteConsistent = true;
                            end
                        end
                        navResults.shadowBranchAbsoluteRefValid(1, currMeasNr) = branchAbsoluteRefValid;
                        navResults.shadowBranchAbsoluteRefDiffM(1, currMeasNr) = branchAbsoluteRefDiffM;
                        navResults.shadowBranchAbsoluteConsistent(1, currMeasNr) = branchAbsoluteConsistent;
                        if localIsDs5Scenario(settings)
                            navResults.shadowBranchReady(1, currMeasNr) = ...
                                navResults.shadowBranchPostfitRmsM(1, currMeasNr) <= settings.deepShadowBranchPostfitRmsMaxM && ...
                                navResults.shadowBranchPdop(1, currMeasNr) <= settings.deepShadowBranchPdopMax && ...
                                isfinite(branchDetrendedP95) && branchDetrendedP95 <= settings.deepShadowBranchDs5DetrendedReadyP95MaxM && ...
                                isfinite(branchAbsModelP95) && branchAbsModelP95 <= settings.deepShadowBranchDs5AbsoluteReadyP95MaxM && ...
                                branchAbsoluteConsistent;
                        else
                            navResults.shadowBranchReady(1, currMeasNr) = ...
                                navResults.shadowBranchPriorRmsM(1, currMeasNr) <= settings.deepShadowBranchPriorRmsMaxM && ...
                                navResults.shadowBranchPostfitRmsM(1, currMeasNr) <= settings.deepShadowBranchPostfitRmsMaxM && ...
                                navResults.shadowBranchPdop(1, currMeasNr) <= settings.deepShadowBranchPdopMax && ...
                                navResults.shadowBranchPosJumpM(1, currMeasNr) <= settings.deepShadowBranchPosJumpMaxM;
                            if localUseDs5CommonDrag(settings, currMeasNr)
                                navResults.shadowBranchReady(1, currMeasNr) = navResults.shadowBranchReady(1, currMeasNr) && ...
                                    isfinite(branchDetrendedP95) && branchDetrendedP95 <= settings.deepShadowBranchDs5DetrendedReadyP95MaxM;
                            end
                        end
                    end
                end
            end
            if isfield(settings, 'deepShadowBranchSearchEnable') && settings.deepShadowBranchSearchEnable
                trustedPriorOk = true;
                if settings.deepShadowBranchTakeoverRequireTrustedPrior
                    trustedPriorOk = navResults.shadowBranchPriorSource(1, currMeasNr) ~= 1 && ...
                        navResults.shadowBranchPriorSource(1, currMeasNr) ~= 0;
                end
                recoveryTrustedPriorOk = true;
                if settings.deepShadowRecoveryRequireTrustedPrior
                    recoveryTrustedPriorOk = navResults.shadowBranchPriorSource(1, currMeasNr) ~= 1 && ...
                        navResults.shadowBranchPriorSource(1, currMeasNr) ~= 0;
                end
                branchTakeoverMinSatNow = settings.deepShadowBranchTakeoverMinSat;
                recoveryContinueMinSatNow = settings.deepShadowRecoveryContinueMinSat;
                if localUseDs5CommonDrag(settings, currMeasNr) && ...
                        isfinite(branchDetrendedP95) && branchDetrendedP95 <= settings.deepShadowDs5BranchTakeoverDetrendedP95MaxM && ...
                        isfield(settings, 'deepShadowDs5BranchTakeoverAllow4Sat') && settings.deepShadowDs5BranchTakeoverAllow4Sat
                    branchTakeoverMinSatNow = min(branchTakeoverMinSatNow, 4);
                end
                if localUseDs5CommonDrag(settings, currMeasNr) && ...
                        isfinite(branchDetrendedP95) && branchDetrendedP95 <= settings.deepShadowDs5BranchContinueDetrendedP95MaxM && ...
                        isfield(settings, 'deepShadowDs5BranchContinueAllow4Sat') && settings.deepShadowDs5BranchContinueAllow4Sat
                    recoveryContinueMinSatNow = min(recoveryContinueMinSatNow, 4);
                end
                branchTakeoverCandidateNow = navResults.shadowBranchReady(1, currMeasNr) && ...
                    trustedPriorOk && ...
                    navResults.shadowBranchSatNum(1, currMeasNr) >= branchTakeoverMinSatNow && ...
                    navResults.shadowBranchPostfitRmsM(1, currMeasNr) <= settings.deepShadowBranchTakeoverPostfitRmsMaxM && ...
                    navResults.shadowBranchPdop(1, currMeasNr) <= settings.deepShadowBranchTakeoverPdopMax;
                shadowBranchSaneNow = navResults.shadowBranchReady(1, currMeasNr) && ...
                    recoveryTrustedPriorOk && ...
                    navResults.shadowBranchSatNum(1, currMeasNr) >= recoveryContinueMinSatNow && ...
                    navResults.shadowBranchPostfitRmsM(1, currMeasNr) <= settings.deepShadowRecoveryContinuePostfitRmsMaxM && ...
                    navResults.shadowBranchPdop(1, currMeasNr) <= settings.deepShadowRecoveryContinuePdopMax;
                navResults.shadowRecoveryBranchSane(1, currMeasNr) = shadowBranchSaneNow;
                if localUseDs5SoftRecovery(settings, currMeasNr)
                    softRecoveryOk = navResults.shadowBranchReady(1, currMeasNr);
                    if settings.deepShadowDs5SoftRecoveryRequireBranchSane
                        softRecoveryOk = shadowBranchSaneNow;
                    end
                    if softRecoveryOk
                        shadowRecoveryMode = true;
                        shadowRecoveryRemain = max(shadowRecoveryRemain, settings.deepShadowRecoveryHoldEpochs);
                        navResults.shadowRecoveryMode(1, currMeasNr) = shadowRecoveryMode;
                        navResults.shadowRecoveryRemain(1, currMeasNr) = shadowRecoveryRemain;
                    end
                end
            end
            if branchTakeoverCandidateNow
                branchTakeoverCnt = branchTakeoverCnt + 1;
            else
                branchTakeoverCnt = 0;
            end
            navResults.shadowBranchTakeoverCounter(1, currMeasNr) = branchTakeoverCnt;
            navResults.shadowBranchTakeoverReady(1, currMeasNr) = branchTakeoverCnt >= settings.deepShadowBranchTakeoverConfirmEpochs;
            if navResults.shadowBranchTakeoverReady(1, currMeasNr)
                navResults.shadowTakeoverX(1, currMeasNr) = navResults.shadowBranchX(1, currMeasNr);
                navResults.shadowTakeoverY(1, currMeasNr) = navResults.shadowBranchY(1, currMeasNr);
                navResults.shadowTakeoverZ(1, currMeasNr) = navResults.shadowBranchZ(1, currMeasNr);
                navResults.shadowTakeoverClockM(1, currMeasNr) = navResults.shadowBranchClockM(1, currMeasNr);
                if ~isfinite(navResults.shadowTakeoverFirstEpoch)
                    navResults.shadowTakeoverFirstEpoch = currMeasNr;
                end
            end
            if settings.deepShadowCoreServoEnable && numel(navCoreIdx) >= settings.deepShadowCoreServoMinSat && ...
                    isfinite(navCoreMaxDevM) && navCoreMaxDevM <= settings.deepShadowCoreServoCoreDevMaxM
                coreDeltaLocal = deltaShadowPass(navCoreMaskLocal);
                coreMedServo = median(coreDeltaLocal, 'omitnan');
                coreDevServo = coreDeltaLocal - coreMedServo;
                corePassLocalIdx = find(navCoreMaskLocal);
                for ss = 1:numel(corePassLocalIdx)
                    jjLocal = corePassLocalIdx(ss);
                    jjGlobal = passShadowIdx(jjLocal);
                    devM = coreDevServo(ss);
                    if ~isfinite(devM)
                        continue;
                    end
                    ambigStepChip = 0;
                    if settings.deepShadowRawAmbiguityEnable && numel(navCoreIdx) >= settings.deepShadowRawAmbiguityMinSat && ...
                            jjLocal <= numel(ambigChipsLocal) && isfinite(ambigChipsLocal(jjLocal))
                        ambigStepChip = max(-settings.deepShadowRawAmbiguityStepMaxChips, ...
                            min(settings.deepShadowRawAmbiguityStepMaxChips, ambigChipsLocal(jjLocal)));
                    end
                    stepChip = ambigStepChip + settings.deepShadowCoreServoGain * devM / localChipMeters(settings);
                    stepChip = max(-settings.deepShadowCoreServoStepMaxChips, ...
                        min(settings.deepShadowCoreServoStepMaxChips, stepChip));
                    oldCorr = 0;
                    if isfield(trackShadow(jjGlobal), 'deepCodePhaseCorrChips') && ...
                            isfinite(trackShadow(jjGlobal).deepCodePhaseCorrChips)
                        oldCorr = trackShadow(jjGlobal).deepCodePhaseCorrChips;
                    end
                    newCorr = max(-settings.deepShadowCoreServoTotalMaxChips, ...
                        min(settings.deepShadowCoreServoTotalMaxChips, oldCorr + stepChip));
                    appliedStep = newCorr - oldCorr;
                    trackShadow(jjGlobal).deepCodePhaseCorrChips = newCorr;
                    navResults.shadowRawPassServoRawP(jjGlobal, currMeasNr) = ...
                        navResults.shadowRawPassServoRawP(jjGlobal, currMeasNr) - appliedStep * localChipMeters(settings);
                    navResults.shadowCoreServoUsed(jjGlobal, currMeasNr) = abs(appliedStep) > 0;
                    navResults.shadowCoreServoDevM(jjGlobal, currMeasNr) = devM;
                    navResults.shadowCoreServoStepChips(jjGlobal, currMeasNr) = appliedStep;
                    navResults.shadowCoreServoCorrChips(jjGlobal, currMeasNr) = newCorr;
                end
                navResults.shadowCoreServoCoreMedM(1, currMeasNr) = coreMedServo;
                navResults.shadowCoreServoCoreMaxDevM(1, currMeasNr) = max(abs(coreDevServo), [], 'omitnan');
            end
            if settings.deepShadowUseNavQualifiedForKf && numel(navQualifiedIdx) >= settings.deepShadowKfMinSat
                [~, LOSShadowPass, AzElShadowPass, ~] = rhoSatRec_zcj(navSolutShadowPass.satPositions', posxyz, ...
                    navSolutShadowPass.rawP', navSolutShadowPass.satVelocity', ins.vn);
                elShadowPass = AzElShadowPass(:,2);
                elShadowPass(elShadowPass < 15*pi/180) = 1*pi/180;
                shadowKfDeltaUse = deltaShadowPass(navQualifiedMaskLocal);
                shadowKfDeltaRawUse = shadowKfDeltaUse;
                shadowKfSourceModeUse = ones(size(shadowKfDeltaUse));
                shadowKfPrnUse = passPrnLocal(navQualifiedMaskLocal);
                shadowKfMinSatUse = settings.deepShadowKfMinSat;
                if localUseDs5CommonDrag(settings, currMeasNr)
                    shadowKfDeltaUse = deltaShadowPassNavGate(navQualifiedMaskLocal);
                    shadowKfSourceModeUse = navSourceModeLocal(navQualifiedMaskLocal);
                end
                shadowKfLOSUse = LOSShadowPass(navQualifiedMaskLocal, :);
                shadowKfElUse = elShadowPass(navQualifiedMaskLocal);
                navResults.shadowRawKfCandidateSatNum(1, currMeasNr) = numel(shadowKfDeltaUse);
                navResults.shadowRawKfMode1CandNum(1, currMeasNr) = sum(shadowKfSourceModeUse == 1);
                navResults.shadowRawKfMode2CandNum(1, currMeasNr) = sum(shadowKfSourceModeUse == 2);
                navResults.shadowRawKfMode3CandNum(1, currMeasNr) = sum(shadowKfSourceModeUse == 3);
                navResults.shadowRawKfMode4CandNum(1, currMeasNr) = sum(shadowKfSourceModeUse == 4);
                if localUseDs5KfSourceGate(settings, currMeasNr)
                    if localGetSettingValue(settings, 'deepShadowDs5KfRequireCommonDrag', 1) ~= 0 && ...
                            ~localUseDs5CommonDrag(settings, currMeasNr)
                        gateMaskSourceKf = false(size(shadowKfDeltaUse));
                        kfMinSatLocal = localGetSettingValue(settings, 'deepShadowDs5KfTrustedMinSat', settings.deepShadowKfMinSat);
                    else
                        [gateMaskSourceKf, kfMinSatLocal] = localBuildDs5KfSourceTrustMask( ...
                            shadowKfSourceModeUse, shadowKfPrnUse, shadowKfDeltaUse, shadowBranchSaneNow, settings);
                    end
                    shadowKfMinSatUse = kfMinSatLocal;
                    navResults.shadowRawKfTrustedCandNum(1, currMeasNr) = sum(gateMaskSourceKf);
                    navResults.shadowRawKfSourceGateRejectNum(1, currMeasNr) = numel(shadowKfDeltaUse) - sum(gateMaskSourceKf);
                    shadowKfDeltaUse = shadowKfDeltaUse(gateMaskSourceKf);
                    shadowKfDeltaRawUse = shadowKfDeltaRawUse(gateMaskSourceKf);
                    shadowKfLOSUse = shadowKfLOSUse(gateMaskSourceKf, :);
                    shadowKfElUse = shadowKfElUse(gateMaskSourceKf);
                    shadowKfSourceModeUse = shadowKfSourceModeUse(gateMaskSourceKf);
                else
                    navResults.shadowRawKfTrustedCandNum(1, currMeasNr) = sum(isfinite(shadowKfDeltaUse));
                end
                if settings.deepShadowKfMedianDetrend && ~isempty(shadowKfDeltaUse)
                    shadowKfDeltaUse = shadowKfDeltaUse - median(shadowKfDeltaUse, 'omitnan');
                end
                gateMaskShadowKf = isfinite(shadowKfDeltaUse) & ...
                    (abs(shadowKfDeltaUse) <= settings.deepShadowKfDetrendGateM);
                navResults.shadowRawKfGateRejectNum(1, currMeasNr) = numel(shadowKfDeltaUse) - sum(gateMaskShadowKf);
                shadowKfDeltaUse = shadowKfDeltaUse(gateMaskShadowKf);
                shadowKfDeltaRawUse = shadowKfDeltaRawUse(gateMaskShadowKf);
                shadowKfLOSUse = shadowKfLOSUse(gateMaskShadowKf, :);
                shadowKfElUse = shadowKfElUse(gateMaskShadowKf);
                shadowKfReadyNow = numel(shadowKfDeltaUse) >= shadowKfMinSatUse;
                if localUseDs5Authority(settings, currMeasNr) && shadowRecoveryMode
                    mode2CountNow = sum(shadowKfSourceModeUse == 2);
                    mode23CountNow = sum(shadowKfSourceModeUse == 2 | shadowKfSourceModeUse == 3);
                    authorityRefPosNow = localBuildDs5AuthorityRefPos(shadowClosedLoopLastPos, shadowClosedLoopLastVel, shadowClosedLoopLastEpoch, ...
                        trustedAnchorValid, trustedAnchorPos, trustedAnchorVel, trustedAnchorTimeSec, epochElapsedSec, currMeasNr, settings);
                    [ds5AuthorityEligibleNow, ds5AuthorityRefDiffNow] = localEvaluateDs5Authority( ...
                        shadowBranchSaneNow, numel(shadowKfDeltaUse), mode2CountNow, mode23CountNow, posxyz(:), authorityRefPosNow, settings);
                    if ds5AuthorityEligibleNow
                        ds5RecoveryAuthorityConfirmCnt = ds5RecoveryAuthorityConfirmCnt + 1;
                        ds5RecoveryAuthorityBadCnt = 0;
                    else
                        ds5RecoveryAuthorityConfirmCnt = 0;
                        ds5RecoveryAuthorityBadCnt = ds5RecoveryAuthorityBadCnt + 1;
                    end
                    if ds5RecoveryAuthorityConfirmCnt >= localGetSettingValue(settings, 'deepShadowDs5AuthorityConfirmEpochs', 3)
                        ds5RecoveryAuthorityRemain = max(ds5RecoveryAuthorityRemain, localGetSettingValue(settings, 'deepShadowDs5AuthorityHoldEpochs', 6));
                    end
                    if localShouldHardRevokeDs5Authority(ds5RecoveryAuthorized, ds5AuthorityEligibleNow, ds5AuthorityRefDiffNow, shadowBranchSaneNow, settings) || ...
                            localShouldFastDeauthorizeDs5Authority(ds5RecoveryAuthorized, ds5AuthorityEligibleNow, ds5RecoveryAuthorityBadCnt, settings)
                        ds5RecoveryAuthorityRemain = 0;
                        ds5RecoveryAuthorityConfirmCnt = 0;
                        ds5RecoveryAuthorityBadCnt = 0;
                    end
                    ds5RecoveryAuthorized = ds5RecoveryAuthorityRemain > 0;
                    if ~ds5RecoveryAuthorized
                        shadowKfReadyNow = false;
                    end
                end

            end
            branchInjectAllowedNow = settings.deepShadowBranchInjectEnable && ...
                (navResults.shadowBranchTakeoverReady(1, currMeasNr) || (shadowRecoveryMode && shadowBranchSaneNow));
            if branchInjectAllowedNow && navResults.shadowBranchSatNum(1, currMeasNr) >= settings.deepShadowBranchInjectMinSat
                branchGlobalIdx = find(isfinite(navResults.shadowBranchRawP(:, currMeasNr)));
                if numel(branchGlobalIdx) >= settings.deepShadowBranchInjectMinSat
                    branchRawPInject = navResults.shadowBranchRawP(branchGlobalIdx, currMeasNr);
                    satPosInject = [navResults.shadowRawPassSatPosX(branchGlobalIdx, currMeasNr).'; ...
                                    navResults.shadowRawPassSatPosY(branchGlobalIdx, currMeasNr).'; ...
                                    navResults.shadowRawPassSatPosZ(branchGlobalIdx, currMeasNr).'];
                    satClkInject = navResults.shadowRawPassSatClkCorr(branchGlobalIdx, currMeasNr);
                    satVelInject = [navResults.shadowRawPassSatVelX(branchGlobalIdx, currMeasNr).'; ...
                                    navResults.shadowRawPassSatVelY(branchGlobalIdx, currMeasNr).'; ...
                                    navResults.shadowRawPassSatVelZ(branchGlobalIdx, currMeasNr).'];
                    obsInject = branchRawPInject(:)' + settings.c * satClkInject(:)';
                    injectRefPos = posxyz(:);
                    if all(isfinite([navResults.shadowTakeoverX(1, currMeasNr), navResults.shadowTakeoverY(1, currMeasNr), navResults.shadowTakeoverZ(1, currMeasNr)]))
                        injectRefPos = [navResults.shadowTakeoverX(1, currMeasNr); ...
                                        navResults.shadowTakeoverY(1, currMeasNr); ...
                                        navResults.shadowTakeoverZ(1, currMeasNr)];
                    end
                    dposInject = bsxfun(@minus, satPosInject, injectRefPos(:));
                    rhoInject = sqrt(sum(dposInject.^2, 1));
                    deltaInjectRaw = obsInject(:) - rhoInject(:);
                    deltaInject = deltaInjectRaw - median(deltaInjectRaw, 'omitnan');
                    resetVelForLos = ins.vn;
                    if settings.deepShadowBranchResetVelBlend > 0 && trustedAnchorValid && all(isfinite(trustedAnchorVel))
                        [tmpLatDeg, tmpLonDeg, tmpH] = cart2geo(injectRefPos(1), injectRefPos(2), injectRefPos(3), 5);
                        resetVelForLos = localEcefVelToNed([tmpLatDeg*pi/180; tmpLonDeg*pi/180; tmpH], trustedAnchorVel);
                    end
                    [~, LOSInject, AzElInject, ~] = rhoSatRec_zcj(satPosInject', injectRefPos(:), ...
                        branchRawPInject(:), satVelInject', resetVelForLos);
                    elInject = AzElInject(:,2);
                    elInject(elInject < 15*pi/180) = 1*pi/180;
                    gateInject = isfinite(deltaInject) & abs(deltaInject) <= settings.deepShadowBranchInjectResidualGateM;
                    navResults.shadowBranchInjectRejectNum(1, currMeasNr) = numel(deltaInject) - sum(gateInject);
                    if sum(gateInject) >= settings.deepShadowBranchInjectMinSat
                        shadowKfDeltaUse = deltaInject(gateInject);
                        shadowKfDeltaRawUse = deltaInjectRaw(gateInject);
                        shadowKfLOSUse = LOSInject(gateInject, :);
                        shadowKfElUse = elInject(gateInject);
                        shadowKfReadyNow = true;
                        shadowKfFromBranchInject = true;
                        navResults.shadowRawKfCandidateSatNum(1, currMeasNr) = numel(shadowKfDeltaUse);
                    end
                end
            end
            if any(navQualifiedMaskLocal)
                deltaQualified = deltaShadowPassNavGate(navQualifiedMaskLocal);
                navResults.shadowRawNavQualifiedDeltaMed(1, currMeasNr) = median(deltaQualified, 'omitnan');
                if any(isfinite(deltaQualified))
                    navResults.shadowRawNavQualifiedDeltaP95(1, currMeasNr) = prctile(deltaQualified(isfinite(deltaQualified)), 95);
                end
            end
        end
    end
    if ~lightNormalMode
        navResults.shadowMainSwitchCounter(1:numActChnList, currMeasNr) = shadowMainSwitchCnt;
        navResults.shadowMainHoldCounter(1:numActChnList, currMeasNr) = shadowMainHoldCnt;
        navResults.shadowMainFollowActive(1:numActChnList, currMeasNr) = shadowMainFollowActive;
    end
    navResults.diagTrackTimeSec(1, currMeasNr) = diagTrackTimeSec;
    navResults.diagTrackCalls(1, currMeasNr) = diagTrackCalls;
    navResults.diagRawReacqTimeSec(1, currMeasNr) = diagRawReacqTimeSec;
    navResults.diagRawReacqCalls(1, currMeasNr) = diagRawReacqCalls;
    navResults.diagImuPropTimeSec(1, currMeasNr) = diagImuPropTimeSec;
    navResults.diagMainNavTimeSec(1, currMeasNr) = diagMainNavTimeSec;
    navResults.diagRhoTimeSec(1, currMeasNr) = diagRhoTimeSec;
    navResults.diagShadowNavTimeSec(1, currMeasNr) = diagShadowNavTimeSec;
    navResults.diagEkfTimeSec(1, currMeasNr) = diagEkfTimeSec;
    navResults.diagDeepFeedbackTimeSec(1, currMeasNr) = diagDeepFeedbackTimeSec;
    navResults.diagEpochWallSec(1, currMeasNr) = toc(diagEpochWallTic);
    navResults.diagRunWallSec(1, currMeasNr) = toc(diagRunWallTic);
    if settings.deepPerfDiagPrint && (mod(currMeasNr, max(1, settings.deepPerfDiagInterval)) == 0 || currMeasNr == roundTime)
        fprintf(['[DIAG] epoch %d/%d (%.1fs) mode=%d epochWall=%.2fs totalWall=%.1fs ' ...
                 'imu=%.1fs nav=%.1fs rho=%.1fs shNav=%.1fs ekf=%.1fs ' ...
                 'trackCalls=%d trackTime=%.1fs rawCalls=%d rawTime=%.1fs\n'], ...
            currMeasNr, roundTime, (currMeasNr - 1) * settings.navSolPeriod / 1000, ...
            deepModeState, navResults.diagEpochWallSec(1, currMeasNr), ...
            navResults.diagRunWallSec(1, currMeasNr), diagImuPropTimeSec, ...
            diagMainNavTimeSec, diagRhoTimeSec, diagShadowNavTimeSec, diagEkfTimeSec, diagTrackCalls, ...
            diagTrackTimeSec, diagRawReacqCalls, diagRawReacqTimeSec);
    end

    % 2. EKF
    diagLocalTic = tic;
    shadowKfUpdateUsedNow = false;
    [currX, currY, currZ] = geo2cart(ins.avp(7,1), ins.avp(8,1), ins.avp(9,1), 5);
    currBranchDiffM = nan;
    if all(isfinite([navResults.shadowBranchX(1, currMeasNr), navResults.shadowBranchY(1, currMeasNr), navResults.shadowBranchZ(1, currMeasNr)]))
        currBranchDiffM = norm([navResults.shadowBranchX(1, currMeasNr); ...
                                navResults.shadowBranchY(1, currMeasNr); ...
                                navResults.shadowBranchZ(1, currMeasNr)] - [currX; currY; currZ]);
        navResults.shadowRecoveryNavBranchDiffM(1, currMeasNr) = currBranchDiffM;
    end
    initialResetNow = settings.deepShadowBranchResetEnable && ...
        navResults.shadowBranchTakeoverReady(1, currMeasNr) && ...
        (~settings.deepShadowBranchResetOnce || ~shadowBranchResetDone) && ...
        all(isfinite([navResults.shadowTakeoverX(1, currMeasNr), navResults.shadowTakeoverY(1, currMeasNr), navResults.shadowTakeoverZ(1, currMeasNr)]));
    reanchorNow = settings.deepShadowBranchResetEnable && settings.deepShadowRecoveryReanchorEnable && ...
        shadowRecoveryMode && shadowBranchSaneNow && shadowRecoveryReanchorCooldown <= 0 && ...
        shadowRecoveryReanchorCount < settings.deepShadowRecoveryReanchorMaxCount && ...
        isfinite(currBranchDiffM) && currBranchDiffM >= settings.deepShadowRecoveryReanchorMinJumpM;
    branchResetReadyNow = initialResetNow || reanchorNow;
    if branchResetReadyNow
        if initialResetNow
            branchResetPos = [navResults.shadowTakeoverX(1, currMeasNr); ...
                              navResults.shadowTakeoverY(1, currMeasNr); ...
                              navResults.shadowTakeoverZ(1, currMeasNr)];
            branchResetClockM = navResults.shadowTakeoverClockM(1, currMeasNr);
        else
            branchResetPos = [navResults.shadowBranchX(1, currMeasNr); ...
                              navResults.shadowBranchY(1, currMeasNr); ...
                              navResults.shadowBranchZ(1, currMeasNr)];
            branchResetClockM = navResults.shadowBranchClockM(1, currMeasNr);
        end
        branchResetJumpM = norm(branchResetPos - [currX; currY; currZ]);
        navResults.shadowBranchResetJumpM(1, currMeasNr) = branchResetJumpM;
        if isfinite(branchResetJumpM) && branchResetJumpM <= settings.deepShadowBranchResetMaxJumpM
            [resetLatDeg, resetLonDeg, resetH] = cart2geo(branchResetPos(1), branchResetPos(2), branchResetPos(3), 5);
            ins.pos = [resetLatDeg*pi/180; resetLonDeg*pi/180; resetH];
            ins.avp(7:9,1) = ins.pos;
            if settings.deepShadowBranchResetVelBlend > 0 && trustedAnchorValid && all(isfinite(trustedAnchorVel))
                resetVelNed = localEcefVelToNed(ins.pos, trustedAnchorVel);
                blend = max(0, min(1, settings.deepShadowBranchResetVelBlend));
                ins.vn = (1 - blend) * ins.vn + blend * resetVelNed(:);
                ins.avp(4:6,1) = ins.vn;
            end
            if numel(kf.xk) >= 9
                kf.xk(4:9) = 0;
            end
            if settings.deepShadowBranchResetClock && isfinite(branchResetClockM) && numel(kf.xk) >= 2
                kf.xk(end-1) = branchResetClockM;
            end
            shadowBranchResetDone = true;
            shadowBranchResetEpoch = currMeasNr;
            if settings.deepShadowRecoveryModeEnable
                shadowRecoveryMode = true;
                shadowRecoveryRemain = settings.deepShadowRecoveryHoldEpochs;
            end
            navResults.shadowBranchResetApplied(1, currMeasNr) = initialResetNow;
            navResults.shadowRecoveryReanchorApplied(1, currMeasNr) = reanchorNow;
            if reanchorNow
                shadowRecoveryReanchorCount = shadowRecoveryReanchorCount + 1;
                shadowRecoveryReanchorCooldown = settings.deepShadowRecoveryReanchorCooldownEpochs;
            end
            navResults.shadowRecoveryMode(1, currMeasNr) = shadowRecoveryMode;
            navResults.shadowRecoveryRemain(1, currMeasNr) = shadowRecoveryRemain;
            navResults.shadowRecoveryReanchorCount(1, currMeasNr) = shadowRecoveryReanchorCount;
            navResults.shadowBranchResetEpoch = currMeasNr;
            navResults.shadowBranchResetReason(1, currMeasNr) = 1 + 10 * reanchorNow;
        else
            navResults.shadowBranchResetReason(1, currMeasNr) = 2 + 10 * reanchorNow;
        end
    end
    if shadowKfReadyNow
        if ~isfield(kf, 'Phikk_1') || isempty(kf.Phikk_1)
            kf.Phikk_1 = kffk(ins);
        end
        WShadowUse = diag(sin(shadowKfElUse.^2));
        if ~isempty(shadowKfDeltaUse)
            kf.Hk = kfhk(ins, shadowKfLOSUse);
            shadowRScaleUse = settings.deepShadowKfRScale;
            if shadowKfFromBranchInject
                shadowRScaleUse = settings.deepShadowBranchInjectRScale;
            end
            kf.Rk = (WShadowUse^-1 * 10^2) * max(1e-3, shadowRScaleUse);
            kf = kfupdate(kf, shadowKfDeltaUse);
            [kf, ins] = kffeedback(kf, ins, 1, 'avp');
            shadowKfUpdateUsedNow = true;
            navResults.shadowRawKfUsedSatNum(1, currMeasNr) = numel(shadowKfDeltaUse);
            if shadowKfFromBranchInject
                navResults.shadowBranchInjectUsed(1, currMeasNr) = true;
                navResults.shadowBranchInjectUsedSatNum(1, currMeasNr) = numel(shadowKfDeltaUse);
                navResults.shadowBranchInjectResidualMedM(1, currMeasNr) = median(shadowKfDeltaUse, 'omitnan');
                if any(isfinite(shadowKfDeltaUse))
                    navResults.shadowBranchInjectResidualP95M(1, currMeasNr) = prctile(abs(shadowKfDeltaUse(isfinite(shadowKfDeltaUse))), 95);
                end
            end
            navResults.shadowRawKfDeltaMed(1, currMeasNr) = median(shadowKfDeltaRawUse, 'omitnan');
            if any(isfinite(shadowKfDeltaRawUse))
                navResults.shadowRawKfDeltaP95(1, currMeasNr) = prctile(shadowKfDeltaRawUse(isfinite(shadowKfDeltaRawUse)), 95);
            end
            navResults.shadowRawKfDeltaDetrendMed(1, currMeasNr) = median(shadowKfDeltaUse, 'omitnan');
            if any(isfinite(shadowKfDeltaUse))
                navResults.shadowRawKfDeltaDetrendP95(1, currMeasNr) = prctile(shadowKfDeltaUse(isfinite(shadowKfDeltaUse)), 95);
            end
        end
    elseif useGnssKfUpdateNow
        if ~isfield(kf, 'Phikk_1') || isempty(kf.Phikk_1)
            kf.Phikk_1 = kffk(ins);
        end
        LOSUse = LOS(satUseIdx, :);
        elUse = el(satUseIdx);
        deltaUse = delta_rawP(satUseIdx);
        WUse = diag(sin(elUse.^2));
        if ~isempty(deltaUse)
            kf.Hk = kfhk(ins, LOSUse);
            kf.Rk = (WUse^-1 * 10^2) * max(1e-3, rScaleNow);
            kf = kfupdate(kf, deltaUse);
            [kf, ins] = kffeedback(kf, ins, 1, 'avp');
        end
    elseif useKfClockUpdateNow
        if ~isfield(kf, 'Phikk_1') || isempty(kf.Phikk_1)
            kf.Phikk_1 = kffk(ins);
        end
        LOSUse = LOS(satUseIdx, :);
        elUse = el(satUseIdx);
        deltaUse = delta_rawP(satUseIdx);
        WUse = diag(sin(elUse.^2));
        if ~isempty(deltaUse)
            kf.Hk = kfhk(ins, LOSUse);
            kf.Rk = (WUse^-1 * 10^2) * max(1e-3, rScaleNow);
            kf = kfupdate(kf, deltaUse);
        end
    end
    diagEkfTimeSec = diagEkfTimeSec + toc(diagLocalTic);
    navResults.shadowRawKfUpdateUsed(1, currMeasNr) = shadowKfUpdateUsedNow;
    if localUseDs5Authority(settings, currMeasNr) && shadowRecoveryMode
        ds5ObserveOnlyNow = ~ds5RecoveryAuthorized;
    end
    navResults.shadowDs5ObserveOnlyRecovery(1, currMeasNr) = ds5ObserveOnlyNow;
    navResults.shadowDs5RecoveryAuthorized(1, currMeasNr) = ds5RecoveryAuthorized;
    navResults.shadowDs5RecoveryAuthorityEligible(1, currMeasNr) = ds5AuthorityEligibleNow;
    navResults.shadowDs5RecoveryAuthorityRemain(1, currMeasNr) = ds5RecoveryAuthorityRemain;
    navResults.shadowDs5RecoveryAuthorityConfirm(1, currMeasNr) = ds5RecoveryAuthorityConfirmCnt;
    navResults.shadowDs5RecoveryAuthorityRefDiffM(1, currMeasNr) = ds5AuthorityRefDiffNow;
    [ds5RefObsRecoveryNow, navResults] = localEvaluateDs5RefObsRecovery(currMeasNr, navResults, baselineOutputPos, baselineOutputValid, settings);
    if ds5RefObsRecoveryNow && settings.deepShadowRecoveryModeEnable
        shadowRecoveryMode = true;
        shadowRecoveryRemain = max(shadowRecoveryRemain, settings.deepShadowRecoveryHoldEpochs);
        navResults.shadowRecoveryMode(1, currMeasNr) = shadowRecoveryMode;
        navResults.shadowRecoveryRemain(1, currMeasNr) = shadowRecoveryRemain;
    end
    navResults.gnssKfUpdateUsed(1, currMeasNr) = useGnssKfUpdateNow;
    navResults.clockKfUpdateUsed(1, currMeasNr) = useKfClockUpdateNow;

    % 3. Convert to ECEF for output
    [posX, posY, posZ] = geo2cart(ins.avp(7,1), ins.avp(8,1), ins.avp(9,1), 5);
    pos_XYZ = [posX; posY; posZ];
    Cenu2xyz = [-sin(ins.pos(2))                  cos(ins.pos(2))   0
                -sin(ins.pos(1))*cos(ins.pos(2)) -sin(ins.pos(1))*sin(ins.pos(2))  cos(ins.pos(1))
                 cos(ins.pos(1))*cos(ins.pos(2))  cos(ins.pos(1))*sin(ins.pos(2))  sin(ins.pos(1))];
    vxyz = Cenu2xyz' * ins.vn;
    navResults.X(1, currMeasNr) = posX;  navResults.Y(1, currMeasNr) = posY;
    navResults.Z(1, currMeasNr) = posZ;  navResults.dt(1, currMeasNr) = kf.xk(end-1);
    navResults.VX(1, currMeasNr)= vxyz(1);
    navResults.VY(1, currMeasNr)= vxyz(2);
    navResults.VZ(1, currMeasNr)= vxyz(3);
    closedLoopPos = nan(3,1);
    closedLoopSource = 0;
    closedLoopSafeBranchUsedNow = false;
    closedLoopSafeBranchRejectedNow = false;
    closedLoopSafeBranchDiffMNow = nan;
    closedLoopSafeBranchAlphaNow = 0;
    ds5RefObsClosedLoopHoldUsed = false;
    if shadowRecoveryMode
        if settings.deepShadowClosedLoopAllowNavFallbackInRecovery
            closedLoopPos = [posX; posY; posZ];
            closedLoopSource = 3;
        else
            closedLoopPos = nan(3,1);
            closedLoopSource = 0;
        end
        if localIsDs5Scenario(settings) && ds5RefObsRecoveryNow && ...
                localGetSettingValue(settings, 'deepShadowDs5RefObsClosedLoopUseBaselineHold', 1) ~= 0
            refObsHoldPos = nan(3,1);
            refObsHoldVel = nan(3,1);
            refObsAnchorSource = 0;
            if shadowRecoveredFilterAuthority && all(isfinite(shadowRecoveredFilterPos))
                refObsHoldPos = shadowRecoveredFilterPos(:);
                refObsAnchorSource = 4;
                if all(isfinite(shadowRecoveredFilterVel))
                    refObsHoldVel = shadowRecoveredFilterVel(:);
                    if isfinite(shadowRecoveredFilterEpoch)
                        filtDt = (currMeasNr - shadowRecoveredFilterEpoch) * settings.navSolPeriod / 1000;
                        if isfinite(filtDt) && filtDt > 0
                            refObsHoldPos = refObsHoldPos + filtDt * refObsHoldVel(:);
                        end
                    end
                end
            elseif all(isfinite(ds5RefObsPosLastAccepted)) && isfinite(ds5RefObsPosLastAcceptedEpoch) && ...
                    (currMeasNr - ds5RefObsPosLastAcceptedEpoch) <= localGetSettingValue(settings, 'deepShadowDs5RecoveredFilterMaxCoastEpochs', 80)
                refObsHoldPos = ds5RefObsPosLastAccepted(:);
                refObsAnchorSource = 5;
                if all(isfinite(shadowClosedLoopLastVel))
                    refObsHoldVel = shadowClosedLoopLastVel(:);
                    accDt = (currMeasNr - ds5RefObsPosLastAcceptedEpoch) * settings.navSolPeriod / 1000;
                    if isfinite(accDt) && accDt > 0
                        refObsHoldPos = refObsHoldPos + accDt * refObsHoldVel(:);
                    end
                end
            elseif all(isfinite(shadowClosedLoopLastPos)) && isfinite(shadowClosedLoopLastEpoch)
                coastDt = (currMeasNr - shadowClosedLoopLastEpoch) * settings.navSolPeriod / 1000;
                refObsHoldPos = shadowClosedLoopLastPos(:);
                refObsAnchorSource = 3;
                if settings.deepShadowClosedLoopUseVelocityCoast && all(isfinite(shadowClosedLoopLastVel))
                    refObsHoldPos = refObsHoldPos + coastDt * shadowClosedLoopLastVel(:);
                    refObsHoldVel = shadowClosedLoopLastVel(:);
                end
            elseif trustedAnchorValid && all(isfinite(trustedAnchorPos))
                refObsHoldPos = trustedAnchorPos(:);
                refObsAnchorSource = 2;
                if all(isfinite(trustedAnchorVel)) && isfinite(trustedAnchorTimeSec)
                    trustedDt = epochElapsedSec - trustedAnchorTimeSec;
                    if isfinite(trustedDt) && trustedDt >= 0
                        refObsHoldPos = trustedAnchorPos(:) + trustedDt * trustedAnchorVel(:);
                    end
                    refObsHoldVel = trustedAnchorVel(:);
                end
            elseif baselineOutputValid && all(isfinite(baselineOutputPos))
                refObsHoldPos = baselineOutputPos(:);
                refObsAnchorSource = 1;
                if all(isfinite(baselineOutputVel))
                    refObsHoldVel = baselineOutputVel(:);
                end
            end
            refObsPos = refObsHoldPos;
            refObsClockM = nan;
            refObsClockResidualM = nan;
            refObsCommonClockM = nan;
            refObsPostfitM = nan;
            refObsPdop = nan;
            refObsCorrM = nan;
            refObsAnchorDiffM = nan;
            refObsJumpM = nan;
            refObsUsedSat = 0;
            refObsSpeedMps = nan;
            refObsAccelMps2 = nan;
            refObsDynGatePass = true;
            refObsPredVelDiffMps = nan;
            refObsPredSupportPass = false;
            refObsJumpGatePass = false;
            refObsSpeedGatePass = false;
            refObsAccelGatePass = false;
            refObsDynGateMode = 0;
            refObsRebasedDeltaM = nan(numActChnList, 1);
            refObsRobustKeep = false(numActChnList, 1);
            refObsResidualM = nan(numActChnList, 1);
            refObsDeltaSpreadMedM = nan;
            refObsDeltaSpreadP95M = nan;
            refObsDeltaConsistencyPass = false;
            refObsAbsAnchorDiffM = nan;
            refObsAbsAnchorSoftPass = false;
            refObsAbsAnchorGatePass = false;
            refObsBadScore = nan(numActChnList, 1);
            refObsRawAnchorLiftUsed = false;
            refObsRawAnchorLiftSpreadP95M = nan;
            refObsRawAnchorLiftAmbigChips = nan(numActChnList, 1);
            refObsPosUsedNow = false;
            refObsClockPriorM = localGetDs5ClockPriorM(navResults, currMeasNr, shadowRecoveredFilterClockM);
            if all(isfinite(refObsHoldPos)) && localGetSettingValue(settings, 'deepShadowDs5RefObsPositionEnable', 1) ~= 0
                [refObsPosUsedNow, refObsPos, refObsClockM, refObsClockResidualM, refObsCommonClockM, refObsPostfitM, refObsPdop, refObsCorrM, refObsUsedSat, refObsJumpM, refObsAnchorDiffM, refObsSpeedMps, refObsAccelMps2, refObsDynGatePass, ...
                    refObsPredVelDiffMps, refObsPredSupportPass, refObsJumpGatePass, refObsSpeedGatePass, refObsAccelGatePass, refObsDynGateMode, refObsRebasedDeltaM, refObsRobustKeep, refObsResidualM, ...
                    refObsDeltaSpreadMedM, refObsDeltaSpreadP95M, refObsDeltaConsistencyPass, refObsAbsAnchorDiffM, refObsAbsAnchorSoftPass, refObsAbsAnchorGatePass, ds5RefObsBadScoreByPrn, refObsBadScore, ...
                    refObsRawAnchorLiftUsed, refObsRawAnchorLiftSpreadP95M, refObsRawAnchorLiftAmbigChips] = ...
                    localBuildDs5RefObsRecoveredPosition(currMeasNr, navResults, posxyz(:), refObsHoldPos(:), refObsHoldVel(:), ...
                    ds5AbsAnchorCurrPos(:), refObsClockPriorM, ds5RefObsPosLastAccepted, ds5RefObsPosLastAcceptedEpoch, ds5RefObsPosPrevAccepted, ds5RefObsPosPrevAcceptedEpoch, ds5RefObsDeltaConsistencyConfirmCnt, ds5RefObsBadScoreByPrn, settings);
            end
            if refObsDeltaConsistencyPass
                ds5RefObsDeltaConsistencyConfirmCnt = ds5RefObsDeltaConsistencyConfirmCnt + 1;
            else
                ds5RefObsDeltaConsistencyConfirmCnt = 0;
            end
            navResults.shadowDs5RefObsPosUsed(1, currMeasNr) = refObsPosUsedNow;
            navResults.shadowDs5RefObsPosSatNum(1, currMeasNr) = refObsUsedSat;
            navResults.shadowDs5RefObsPosClockM(1, currMeasNr) = refObsClockM;
            navResults.shadowDs5RefObsPosClockResidualM(1, currMeasNr) = refObsClockResidualM;
            navResults.shadowDs5RefObsPosCommonClockM(1, currMeasNr) = refObsCommonClockM;
            navResults.shadowDs5RefObsPosPostfitRmsM(1, currMeasNr) = refObsPostfitM;
            navResults.shadowDs5RefObsPosPdop(1, currMeasNr) = refObsPdop;
            navResults.shadowDs5RefObsPosCorrM(1, currMeasNr) = refObsCorrM;
            navResults.shadowDs5RefObsPosAnchorDiffM(1, currMeasNr) = refObsAnchorDiffM;
            navResults.shadowDs5RefObsPosJumpM(1, currMeasNr) = refObsJumpM;
            navResults.shadowDs5RefObsPosSpeedMps(1, currMeasNr) = refObsSpeedMps;
            navResults.shadowDs5RefObsPosAccelMps2(1, currMeasNr) = refObsAccelMps2;
            navResults.shadowDs5RefObsPosDynGatePass(1, currMeasNr) = refObsDynGatePass;
            navResults.shadowDs5RefObsPosPredVelDiffMps(1, currMeasNr) = refObsPredVelDiffMps;
            navResults.shadowDs5RefObsPosPredSupportPass(1, currMeasNr) = refObsPredSupportPass;
            navResults.shadowDs5RefObsPosJumpGatePass(1, currMeasNr) = refObsJumpGatePass;
            navResults.shadowDs5RefObsPosSpeedGatePass(1, currMeasNr) = refObsSpeedGatePass;
            navResults.shadowDs5RefObsPosAccelGatePass(1, currMeasNr) = refObsAccelGatePass;
            navResults.shadowDs5RefObsPosDynGateMode(1, currMeasNr) = refObsDynGateMode;
            navResults.shadowDs5RefObsPosAnchorSource(1, currMeasNr) = refObsAnchorSource;
            navResults.shadowDs5RefObsPosCurrRefX(1, currMeasNr) = posxyz(1);
            navResults.shadowDs5RefObsPosCurrRefY(1, currMeasNr) = posxyz(2);
            navResults.shadowDs5RefObsPosCurrRefZ(1, currMeasNr) = posxyz(3);
            navResults.shadowDs5RefObsPosAnchorX(1, currMeasNr) = refObsHoldPos(1);
            navResults.shadowDs5RefObsPosAnchorY(1, currMeasNr) = refObsHoldPos(2);
            navResults.shadowDs5RefObsPosAnchorZ(1, currMeasNr) = refObsHoldPos(3);
            navResults.shadowDs5RefObsPosRebasedDeltaM(:, currMeasNr) = refObsRebasedDeltaM(:);
            navResults.shadowDs5RefObsPosRobustKeep(:, currMeasNr) = refObsRobustKeep(:);
            navResults.shadowDs5RefObsPosResidualM(:, currMeasNr) = refObsResidualM(:);
            navResults.shadowDs5RefObsPosBadScore(:, currMeasNr) = refObsBadScore(:);
            navResults.shadowDs5RefObsPosRawAnchorLiftUsed(1, currMeasNr) = refObsRawAnchorLiftUsed;
            navResults.shadowDs5RefObsPosRawAnchorLiftSpreadP95M(1, currMeasNr) = refObsRawAnchorLiftSpreadP95M;
            navResults.shadowDs5RefObsPosRawAnchorLiftAmbigChips(:, currMeasNr) = refObsRawAnchorLiftAmbigChips(:);
            navResults.shadowDs5RefObsPosDeltaSpreadMedM(1, currMeasNr) = refObsDeltaSpreadMedM;
            navResults.shadowDs5RefObsPosDeltaSpreadP95M(1, currMeasNr) = refObsDeltaSpreadP95M;
            navResults.shadowDs5RefObsPosDeltaConsistencyPass(1, currMeasNr) = refObsDeltaConsistencyPass;
            navResults.shadowDs5RefObsPosDeltaConsistencyConfirm(1, currMeasNr) = ds5RefObsDeltaConsistencyConfirmCnt;
            navResults.shadowDs5RefObsPosAbsAnchorDiffM(1, currMeasNr) = refObsAbsAnchorDiffM;
            navResults.shadowDs5RefObsPosAbsAnchorSoftPass(1, currMeasNr) = refObsAbsAnchorSoftPass;
            navResults.shadowDs5RefObsPosAbsAnchorGatePass(1, currMeasNr) = refObsAbsAnchorGatePass;
            if all(isfinite(refObsPos))
                closedLoopPos = refObsPos(:);
                if refObsPosUsedNow
                    closedLoopSource = 6;
                else
                    closedLoopSource = 5;
                end
                ds5RefObsClosedLoopHoldUsed = true;
                shadowClosedLoopLastPos = closedLoopPos;
                if all(isfinite(refObsHoldVel))
                    shadowClosedLoopLastVel = refObsHoldVel(:);
                else
                    shadowClosedLoopLastVel = localNedVelToEcef(ins.pos, ins.vn);
                end
                shadowClosedLoopLastEpoch = currMeasNr;
                shadowClosedLoopCoastAge = 0;
                shadowClosedLoopLastSource = closedLoopSource;
                navResults.shadowDs5RefObsPosX(1, currMeasNr) = closedLoopPos(1);
                navResults.shadowDs5RefObsPosY(1, currMeasNr) = closedLoopPos(2);
                navResults.shadowDs5RefObsPosZ(1, currMeasNr) = closedLoopPos(3);
                if refObsPosUsedNow
                    ds5RefObsPosPrevAccepted = ds5RefObsPosLastAccepted;
                    ds5RefObsPosPrevAcceptedEpoch = ds5RefObsPosLastAcceptedEpoch;
                    ds5RefObsPosLastAccepted = closedLoopPos(:);
                    ds5RefObsPosLastAcceptedEpoch = currMeasNr;
                end
            end
        end
        if ~ds5RefObsClosedLoopHoldUsed && settings.deepShadowClosedLoopUseBranchOutput && ...
                (shadowBranchSaneNow || ds5RefObsRecoveryNow) && ...
                all(isfinite([navResults.shadowBranchX(1, currMeasNr), navResults.shadowBranchY(1, currMeasNr), navResults.shadowBranchZ(1, currMeasNr)]))
            branchClosedLoopPos = [navResults.shadowBranchX(1, currMeasNr); ...
                                   navResults.shadowBranchY(1, currMeasNr); ...
                                   navResults.shadowBranchZ(1, currMeasNr)];
            safeRefPos = nan(3,1);
            if all(isfinite(shadowClosedLoopLastPos))
                safeRefPos = shadowClosedLoopLastPos;
                if settings.deepShadowClosedLoopUseVelocityCoast && all(isfinite(shadowClosedLoopLastVel)) && isfinite(shadowClosedLoopLastEpoch)
                    safeRefDt = (currMeasNr - shadowClosedLoopLastEpoch) * settings.navSolPeriod / 1000;
                    safeRefPos = shadowClosedLoopLastPos + safeRefDt * shadowClosedLoopLastVel(:);
                end
            elseif trustedAnchorValid && all(isfinite(trustedAnchorPos))
                safeRefPos = trustedAnchorPos(:);
                if all(isfinite(trustedAnchorVel)) && isfinite(trustedAnchorTimeSec)
                    trustedDt = epochElapsedSec - trustedAnchorTimeSec;
                    if isfinite(trustedDt) && trustedDt >= 0
                        safeRefPos = trustedAnchorPos(:) + trustedDt * trustedAnchorVel(:);
                    end
                end
            end
            safeBranchOk = true;
            ds5StrongBranchRecovery = false;
            if localIsDs5Scenario(settings)
                branchAbsP95Now = navResults.shadowBranchAbsModelP95M(1, currMeasNr);
                branchDetP95Now = navResults.shadowBranchDs5DetrendedP95M(1, currMeasNr);
                ds5StrongBranchRecovery = shadowBranchSaneNow && ...
                    isfinite(branchAbsP95Now) && branchAbsP95Now <= localGetSettingValue(settings, 'deepShadowDs5ClosedLoopStrongBranchAbsP95MaxM', 250.0) && ...
                    navResults.shadowBranchPostfitRmsM(1, currMeasNr) <= localGetSettingValue(settings, 'deepShadowDs5ClosedLoopStrongBranchPostfitMaxM', 120.0) && ...
                    (~isfinite(branchDetP95Now) || branchDetP95Now <= localGetSettingValue(settings, 'deepShadowDs5ClosedLoopStrongBranchDetrendedP95MaxM', 250.0));
            end
            if settings.deepShadowClosedLoopSafeBranchGateEnable
                safeBranchOk = false;
                if ds5StrongBranchRecovery
                    safeBranchOk = true;
                elseif all(isfinite(safeRefPos))
                    closedLoopSafeBranchDiffMNow = norm(branchClosedLoopPos - safeRefPos);
                    safeBranchOk = closedLoopSafeBranchDiffMNow <= settings.deepShadowClosedLoopSafeBranchMaxDiffM && ...
                        navResults.shadowBranchPostfitRmsM(1, currMeasNr) <= settings.deepShadowClosedLoopSafeBranchPostfitMaxM && ...
                        navResults.shadowBranchPdop(1, currMeasNr) <= settings.deepShadowClosedLoopSafeBranchPdopMax && ...
                        navResults.shadowBranchSatNum(1, currMeasNr) >= settings.deepShadowClosedLoopSafeBranchMinSat;
                end
            end
            if safeBranchOk
                closedLoopPos = branchClosedLoopPos;
                if settings.deepShadowClosedLoopSafeBranchGateEnable && all(isfinite(safeRefPos))
                    if ds5StrongBranchRecovery
                        closedLoopSafeBranchAlphaNow = 1.0;
                    else
                        closedLoopSafeBranchAlphaNow = max(0, min(1, settings.deepShadowClosedLoopSafeBranchAlpha));
                        closedLoopPos = (1 - closedLoopSafeBranchAlphaNow) * safeRefPos + ...
                            closedLoopSafeBranchAlphaNow * branchClosedLoopPos;
                    end
                end
                closedLoopSource = 1;
                closedLoopSafeBranchUsedNow = true;
                shadowClosedLoopLastPos = closedLoopPos;
                if trustedAnchorValid && all(isfinite(trustedAnchorVel))
                    shadowClosedLoopLastVel = trustedAnchorVel(:);
                else
                    shadowClosedLoopLastVel = localNedVelToEcef(ins.pos, ins.vn);
                end
                shadowClosedLoopLastEpoch = currMeasNr;
                shadowClosedLoopCoastAge = 0;
                shadowClosedLoopLastSource = 1;
            else
                closedLoopSafeBranchRejectedNow = true;
                if all(isfinite(shadowClosedLoopLastPos)) && isfinite(shadowClosedLoopLastEpoch) && ...
                        (~settings.deepShadowClosedLoopAllowNavFallbackInRecovery || ...
                         (currMeasNr - shadowClosedLoopLastEpoch) <= settings.deepShadowClosedLoopCoastEpochs)
                    coastDt = (currMeasNr - shadowClosedLoopLastEpoch) * settings.navSolPeriod / 1000;
                    closedLoopPos = shadowClosedLoopLastPos;
                    if settings.deepShadowClosedLoopUseVelocityCoast && all(isfinite(shadowClosedLoopLastVel))
                        closedLoopPos = shadowClosedLoopLastPos + coastDt * shadowClosedLoopLastVel(:);
                    end
                    closedLoopSource = 2;
                    shadowClosedLoopCoastAge = currMeasNr - shadowClosedLoopLastEpoch;
                    shadowClosedLoopLastSource = 2;
                elseif ~settings.deepShadowClosedLoopAllowNavFallbackInRecovery && ...
                        settings.deepShadowClosedLoopUseTrustedFallback && trustedAnchorValid && all(isfinite(trustedAnchorPos))
                    closedLoopPos = trustedAnchorPos(:);
                    if all(isfinite(trustedAnchorVel)) && isfinite(trustedAnchorTimeSec)
                        trustedDt = epochElapsedSec - trustedAnchorTimeSec;
                        if isfinite(trustedDt) && trustedDt >= 0
                            closedLoopPos = trustedAnchorPos(:) + trustedDt * trustedAnchorVel(:);
                        end
                    end
                    closedLoopSource = 4;
                    shadowClosedLoopCoastAge = nan;
                    shadowClosedLoopLastSource = 4;
                end
            end
        elseif ~ds5RefObsClosedLoopHoldUsed && settings.deepShadowClosedLoopUseBranchOutput && all(isfinite(shadowClosedLoopLastPos)) && ...
                isfinite(shadowClosedLoopLastEpoch) && ...
                (~settings.deepShadowClosedLoopAllowNavFallbackInRecovery || ...
                 (currMeasNr - shadowClosedLoopLastEpoch) <= settings.deepShadowClosedLoopCoastEpochs)
            coastDt = (currMeasNr - shadowClosedLoopLastEpoch) * settings.navSolPeriod / 1000;
            closedLoopPos = shadowClosedLoopLastPos;
            if settings.deepShadowClosedLoopUseVelocityCoast && all(isfinite(shadowClosedLoopLastVel))
                closedLoopPos = shadowClosedLoopLastPos + coastDt * shadowClosedLoopLastVel(:);
            end
            closedLoopSource = 2;
            shadowClosedLoopCoastAge = currMeasNr - shadowClosedLoopLastEpoch;
            shadowClosedLoopLastSource = 2;
        elseif ~ds5RefObsClosedLoopHoldUsed && ~settings.deepShadowClosedLoopAllowNavFallbackInRecovery && ...
                settings.deepShadowClosedLoopUseTrustedFallback && trustedAnchorValid && all(isfinite(trustedAnchorPos))
            closedLoopPos = trustedAnchorPos(:);
            if all(isfinite(trustedAnchorVel)) && isfinite(trustedAnchorTimeSec)
                trustedDt = epochElapsedSec - trustedAnchorTimeSec;
                if isfinite(trustedDt) && trustedDt >= 0
                    closedLoopPos = trustedAnchorPos(:) + trustedDt * trustedAnchorVel(:);
                end
            end
            closedLoopSource = 4;
            shadowClosedLoopCoastAge = nan;
            shadowClosedLoopLastSource = 4;
        end
        closedLoopTrustedBlendAlpha = 0;
        if settings.deepShadowClosedLoopTrustedBlendEnable && ...
                (closedLoopSource == 1 || closedLoopSource == 2) && ...
                trustedAnchorValid && all(isfinite(trustedAnchorPos))
            trustedClosedLoopPos = trustedAnchorPos(:);
            if all(isfinite(trustedAnchorVel)) && isfinite(trustedAnchorTimeSec)
                trustedDt = epochElapsedSec - trustedAnchorTimeSec;
                if isfinite(trustedDt) && trustedDt >= 0
                    trustedClosedLoopPos = trustedAnchorPos(:) + trustedDt * trustedAnchorVel(:);
                end
            end
            if closedLoopSource == 1
                closedLoopTrustedBlendAlpha = settings.deepShadowClosedLoopTrustedBlendBranchAlpha;
            else
                closedLoopTrustedBlendAlpha = settings.deepShadowClosedLoopTrustedBlendCoastAlpha;
            end
            closedLoopTrustedBlendAlpha = max(0, min(1, closedLoopTrustedBlendAlpha));
            if closedLoopTrustedBlendAlpha > 0
                closedLoopPos = (1 - closedLoopTrustedBlendAlpha) * closedLoopPos + ...
                    closedLoopTrustedBlendAlpha * trustedClosedLoopPos;
                if closedLoopSource == 1
                    shadowClosedLoopLastPos = closedLoopPos;
                end
            end
        end
        navResults.shadowClosedLoopX(1, currMeasNr) = closedLoopPos(1);
        navResults.shadowClosedLoopY(1, currMeasNr) = closedLoopPos(2);
        navResults.shadowClosedLoopZ(1, currMeasNr) = closedLoopPos(3);
        navResults.shadowClosedLoopSource(1, currMeasNr) = closedLoopSource;
        navResults.shadowClosedLoopCoastAge(1, currMeasNr) = shadowClosedLoopCoastAge;
        navResults.shadowClosedLoopTrustedBlendAlpha(1, currMeasNr) = closedLoopTrustedBlendAlpha;
        navResults.shadowClosedLoopSafeBranchUsed(1, currMeasNr) = closedLoopSafeBranchUsedNow;
        navResults.shadowClosedLoopSafeBranchRejected(1, currMeasNr) = closedLoopSafeBranchRejectedNow;
        navResults.shadowClosedLoopSafeBranchDiffM(1, currMeasNr) = closedLoopSafeBranchDiffMNow;
        navResults.shadowClosedLoopSafeBranchAlpha(1, currMeasNr) = closedLoopSafeBranchAlphaNow;
        if shadowRecoveryMode && localUseDs5SoftClamp(settings, currMeasNr) && ~shadowKfUpdateUsedNow && ...
                ~useGnssKfUpdateNow && ~useKfClockUpdateNow && all(isfinite(closedLoopPos)) && ...
                (~localUseDs5Authority(settings, currMeasNr) || logical(localGetSettingValue(settings, 'deepShadowDs5ObserveOnlyFollowEnable', 1)))
            currNavEcef = [navResults.X(1, currMeasNr); navResults.Y(1, currMeasNr); navResults.Z(1, currMeasNr)];
            clampDiffM = norm(currNavEcef - closedLoopPos(:));
            clampAlpha = max(0, min(1, localGetSettingValue(settings, 'deepShadowDs5SoftClampAlpha', 0.35)));
            if isfinite(clampDiffM) && clampDiffM > 1.0 && clampAlpha > 0
                clampPos = (1 - clampAlpha) * currNavEcef + clampAlpha * closedLoopPos(:);
                [clampLatDeg, clampLonDeg, clampH] = cart2geo(clampPos(1), clampPos(2), clampPos(3), 5);
                ins.pos = [clampLatDeg*pi/180; clampLonDeg*pi/180; clampH];
                ins.avp(7:9,1) = ins.pos;
                velBlend = max(0, min(1, localGetSettingValue(settings, 'deepShadowDs5SoftClampVelBlend', 0.25)));
                if velBlend > 0 && all(isfinite(shadowClosedLoopLastVel))
                    clampVelNed = localEcefVelToNed(ins.pos, shadowClosedLoopLastVel(:));
                    ins.vn = (1 - velBlend) * ins.vn + velBlend * clampVelNed(:);
                    ins.avp(4:6,1) = ins.vn;
                end
                [posX, posY, posZ] = geo2cart(ins.avp(7,1), ins.avp(8,1), ins.avp(9,1), 5);
                Cenu2xyz = [-sin(ins.pos(2))                  cos(ins.pos(2))   0
                            -sin(ins.pos(1))*cos(ins.pos(2)) -sin(ins.pos(1))*sin(ins.pos(2))  cos(ins.pos(1))
                             cos(ins.pos(1))*cos(ins.pos(2))  cos(ins.pos(1))*sin(ins.pos(2))  sin(ins.pos(1))];
                vxyz = Cenu2xyz' * ins.vn;
                navResults.X(1, currMeasNr) = posX;
                navResults.Y(1, currMeasNr) = posY;
                navResults.Z(1, currMeasNr) = posZ;
                navResults.VX(1, currMeasNr) = vxyz(1);
                navResults.VY(1, currMeasNr) = vxyz(2);
                navResults.VZ(1, currMeasNr) = vxyz(3);
                navResults.shadowDs5SoftClampApplied(1, currMeasNr) = true;
                navResults.shadowDs5SoftClampAlpha(1, currMeasNr) = clampAlpha;
                navResults.shadowDs5SoftClampDiffM(1, currMeasNr) = clampDiffM;
            end
        end
    end

    mainOutputPos = [navResults.X(1, currMeasNr); navResults.Y(1, currMeasNr); navResults.Z(1, currMeasNr)];
    mainOutputVel = [navResults.VX(1, currMeasNr); navResults.VY(1, currMeasNr); navResults.VZ(1, currMeasNr)];

    recoveredFilterUpdatedNow = false;
    recoveredFilterSourceNow = 0;
    recoveredFilterMeasPredDiffNow = nan;
    recoveredFilterDopplerVelNow = nan(3,1);
    recoveredFilterDopplerVelValidNow = false;
    recoveredFilterDopplerVelSatNumNow = 0;
    recoveredFilterDopplerVelRmsMpsNow = nan;
    recoveredFilterDopplerVelP95MpsNow = nan;
    recoveredFilterDopplerVelSignNow = nan;
    recoveredFilterDopplerVelSourceNow = 0;
    recoveredFilterPropStepMNow = nan;
    recoveredFilterPropTotalMNow = nan;
    recoveredFilterPropCapPassNow = true;
    recoveredFilterAuthorityCapPassNow = true;
    recoveredFilterBaselineDiffMNow = nan;
    recoveredFilterAbsAnchorDiffMNow = nan;
    recoveredFilterAbsAnchorSoftPassNow = true;
    recoveredFilterAbsAnchorGatePassNow = false;
    recoveredFilterClockMeasPredDiffNow = nan;
    recoveredFilterClockResetNow = false;
    recoveredFilterResetNow = false;
    recoveredFilterMeasPassNow = false;
    recoveredFilterFilterValidPassNow = false;
    recoveredFilterAuthorityAbsAnchorPassNow = false;
    recoveredFilterAuthorityMeasPredPassNow = false;
    if localIsDs5Scenario(settings) && localGetSettingValue(settings, 'deepShadowDs5RecoveredFilterEnable', 1) ~= 0 && ...
            (deepModeState == 2 || shadowRecoveryMode)
        [ds5AbsAnchorCurrPos, ds5AbsAnchorCurrVel, ds5AbsAnchorCurrAgeEpochs] = ...
            localProjectDs5AbsAnchor(ds5RecoveredAbsAnchorPos, ds5RecoveredAbsAnchorVel, ds5RecoveredAbsAnchorEpoch, ds5RecoveredAbsAnchorTimeSec, ds5RecoveredAbsAnchorValid, epochElapsedSec, currMeasNr, mainOutputVel(:), settings);
        measAvail = all(isfinite(closedLoopPos)) && closedLoopSource == 6;
        filterHasState = all(isfinite(shadowRecoveredFilterPos)) && isfinite(shadowRecoveredFilterEpoch);
        predPos = shadowRecoveredFilterPos;
        predVel = shadowRecoveredFilterVel;
        predClockM = shadowRecoveredFilterClockM;
        if filterHasState
            dtFilterSec = (currMeasNr - shadowRecoveredFilterEpoch) * localGetSettingValue(settings, 'navSolPeriod', 500.0) / 1000.0;
            if isfinite(dtFilterSec) && dtFilterSec > 0
                if all(isfinite(predVel))
                    predPos = shadowRecoveredFilterPos(:) + dtFilterSec * predVel(:);
                elseif localGetSettingValue(settings, 'deepShadowDs5RecoveredFilterPropagateWithInsVel', 1) ~= 0 && all(isfinite(mainOutputVel))
                    predPos = shadowRecoveredFilterPos(:) + dtFilterSec * mainOutputVel(:);
                    predVel = mainOutputVel(:);
                end
                if localGetSettingValue(settings, 'deepShadowDs5RecoveredFilterClockEnable', 1) ~= 0 && ...
                        isfinite(shadowRecoveredFilterClockM) && isfinite(shadowRecoveredFilterClockRateMps)
                    predClockM = shadowRecoveredFilterClockM + dtFilterSec * shadowRecoveredFilterClockRateMps;
                end
                if localGetSettingValue(settings, 'deepShadowDs5RecoveredFilterPredClampEnable', 1) ~= 0
                    predMaxStepM = localGetSettingValue(settings, 'deepShadowDs5RecoveredFilterPropMaxStepM', 120.0);
                    predMaxTotalM = localGetSettingValue(settings, 'deepShadowDs5RecoveredFilterPropMaxTotalM', 600.0);
                    if all(isfinite(shadowRecoveredFilterPos)) && all(isfinite(predPos))
                        stepVec = predPos(:) - shadowRecoveredFilterPos(:);
                        stepNorm = norm(stepVec);
                        if isfinite(predMaxStepM) && predMaxStepM > 0 && isfinite(stepNorm) && stepNorm > predMaxStepM
                            predPos = shadowRecoveredFilterPos(:) + stepVec * (predMaxStepM / stepNorm);
                        end
                    end
                    if all(isfinite(shadowRecoveredFilterLastUpdatePos)) && all(isfinite(predPos))
                        totalVec = predPos(:) - shadowRecoveredFilterLastUpdatePos(:);
                        totalNorm = norm(totalVec);
                        if isfinite(predMaxTotalM) && predMaxTotalM > 0 && isfinite(totalNorm) && totalNorm > predMaxTotalM
                            predPos = shadowRecoveredFilterLastUpdatePos(:) + totalVec * (predMaxTotalM / totalNorm);
                        end
                    end
                    if all(isfinite(predPos)) && all(isfinite(shadowRecoveredFilterPos)) && isfinite(dtFilterSec) && dtFilterSec > 0
                        predVel = (predPos(:) - shadowRecoveredFilterPos(:)) / dtFilterSec;
                    end
                    predMaxClockStepM = localGetSettingValue(settings, 'deepShadowDs5RecoveredFilterPredMaxClockStepM', 600.0);
                    if isfinite(predClockM) && isfinite(shadowRecoveredFilterClockM) && isfinite(predMaxClockStepM) && predMaxClockStepM > 0
                        clockStepM = predClockM - shadowRecoveredFilterClockM;
                        if abs(clockStepM) > predMaxClockStepM
                            predClockM = shadowRecoveredFilterClockM + sign(clockStepM) * predMaxClockStepM;
                            if localGetSettingValue(settings, 'deepShadowDs5RecoveredFilterClockEnable', 1) ~= 0
                                shadowRecoveredFilterClockRateMps = 0;
                            end
                        end
                    end
                end
            end
        elseif measAvail
            predPos = closedLoopPos(:);
            predVel = mainOutputVel(:);
            if isfinite(refObsClockM)
                predClockM = refObsClockM;
            end
        end
        if all(isfinite(mainOutputVel))
            insVelBlend = max(0, min(1, localGetSettingValue(settings, 'deepShadowDs5RecoveredFilterVelInsBlend', 0.10)));
            if all(isfinite(predVel))
                predVel = (1 - insVelBlend) * predVel(:) + insVelBlend * mainOutputVel(:);
            else
                predVel = mainOutputVel(:);
            end
        end
        dopplerVelPos = predPos;
        if measAvail && all(isfinite(closedLoopPos))
            dopplerVelPos = closedLoopPos(:);
        end
        if localGetSettingValue(settings, 'deepShadowDs5RecoveredFilterDopplerVelEnable', 1) ~= 0 && all(isfinite(dopplerVelPos))
            [recoveredFilterDopplerVelNow, recoveredFilterDopplerVelValidNow, recoveredFilterDopplerVelSatNumNow, ...
                recoveredFilterDopplerVelRmsMpsNow, recoveredFilterDopplerVelP95MpsNow, recoveredFilterDopplerVelSignNow, recoveredFilterDopplerVelSourceNow] = ...
                localEstimateDs5RecoveredDopplerVelocity(currMeasNr, navResults, dopplerVelPos(:), predVel(:), settings);
            if recoveredFilterDopplerVelValidNow
                dopplerVelBlend = max(0, min(1, localGetSettingValue(settings, 'deepShadowDs5RecoveredFilterDopplerVelBlend', 0.35)));
                if all(isfinite(predVel))
                    predVel = (1 - dopplerVelBlend) * predVel(:) + dopplerVelBlend * recoveredFilterDopplerVelNow(:);
                else
                    predVel = recoveredFilterDopplerVelNow(:);
                end
            end
        end
        measPass = false;
        if measAvail
            if all(isfinite(predPos)) && filterHasState
                recoveredFilterMeasPredDiffNow = norm(closedLoopPos(:) - predPos(:));
            end
            predDiffMaxM = localGetSettingValue(settings, 'deepShadowDs5RecoveredFilterMeasMaxPredDiffM', 350.0);
            measPass = ~filterHasState || ~isfinite(recoveredFilterMeasPredDiffNow) || ...
                ~isfinite(predDiffMaxM) || predDiffMaxM <= 0 || recoveredFilterMeasPredDiffNow <= predDiffMaxM;
            if all(isfinite(ds5AbsAnchorCurrPos))
                recoveredFilterAbsAnchorDiffMNow = norm(closedLoopPos(:) - ds5AbsAnchorCurrPos(:));
                absAnchorSoftMaxM = localGetSettingValue(settings, 'deepShadowDs5RecoveredFilterAbsAnchorSoftMaxDiffM', 900.0);
                absAnchorMaxM = localGetSettingValue(settings, 'deepShadowDs5RecoveredFilterAbsAnchorMaxDiffM', 1800.0);
                recoveredFilterAbsAnchorSoftPassNow = ~isfinite(absAnchorSoftMaxM) || absAnchorSoftMaxM <= 0 || recoveredFilterAbsAnchorDiffMNow <= absAnchorSoftMaxM;
                recoveredFilterAbsAnchorGatePassNow = ~isfinite(absAnchorMaxM) || absAnchorMaxM <= 0 || recoveredFilterAbsAnchorDiffMNow <= absAnchorMaxM;
                if localGetSettingValue(settings, 'deepShadowDs5RecoveredFilterAbsAnchorGateRequire', 1) ~= 0
                    measPass = measPass && recoveredFilterAbsAnchorGatePassNow;
                end
            end
            if localGetSettingValue(settings, 'deepShadowDs5RecoveredFilterClockEnable', 1) ~= 0 && refObsPosUsedNow && isfinite(refObsClockM) && isfinite(predClockM)
                recoveredFilterClockMeasPredDiffNow = abs(refObsClockM - predClockM);
                clockPredDiffMaxM = localGetSettingValue(settings, 'deepShadowDs5RecoveredFilterClockMeasMaxPredDiffM', 350.0);
                if localGetSettingValue(settings, 'deepShadowDs5RecoveredFilterClockHardGateEnable', 0) ~= 0
                    measPass = measPass && (~isfinite(clockPredDiffMaxM) || clockPredDiffMaxM <= 0 || recoveredFilterClockMeasPredDiffNow <= clockPredDiffMaxM);
                else
                    clockResetDiffMaxM = localGetSettingValue(settings, 'deepShadowDs5RecoveredFilterClockResetDiffMaxM', max(clockPredDiffMaxM, 1500.0));
                    if isfinite(clockResetDiffMaxM) && clockResetDiffMaxM > 0 && isfinite(recoveredFilterClockMeasPredDiffNow) && ...
                            recoveredFilterClockMeasPredDiffNow > clockResetDiffMaxM
                        recoveredFilterClockResetNow = true;
                    end
                end
            end
            resetDiffMaxM = localGetSettingValue(settings, 'deepShadowDs5RecoveredFilterResetMeasPredDiffM', 1500.0);
            if localGetSettingValue(settings, 'deepShadowDs5RecoveredFilterResetOnContractMeas', 1) ~= 0 && refObsPosUsedNow && ...
                    filterHasState && isfinite(recoveredFilterMeasPredDiffNow) && isfinite(resetDiffMaxM) && ...
                    resetDiffMaxM > 0 && recoveredFilterMeasPredDiffNow > resetDiffMaxM
                recoveredFilterResetNow = true;
                measPass = true;
            end
            recoveredFilterMeasPassNow = measPass;
            if measPass
                updateAlpha = max(0, min(1, localGetSettingValue(settings, 'deepShadowDs5RecoveredFilterUpdateAlpha', 0.85)));
                if recoveredFilterResetNow
                    updateAlpha = 1.0;
                end
                if all(isfinite(ds5AbsAnchorCurrPos)) && ~recoveredFilterAbsAnchorSoftPassNow
                    softMaxM = localGetSettingValue(settings, 'deepShadowDs5RecoveredFilterAbsAnchorSoftMaxDiffM', 900.0);
                    hardMaxM = localGetSettingValue(settings, 'deepShadowDs5RecoveredFilterAbsAnchorMaxDiffM', 1800.0);
                    alphaMinScale = max(0, min(1, localGetSettingValue(settings, 'deepShadowDs5RecoveredFilterAbsAnchorAlphaMinScale', 0.15)));
                    if isfinite(softMaxM) && isfinite(hardMaxM) && hardMaxM > softMaxM && isfinite(recoveredFilterAbsAnchorDiffMNow)
                        blendFrac = min(1, max(0, (recoveredFilterAbsAnchorDiffMNow - softMaxM) / max(hardMaxM - softMaxM, eps)));
                        updateAlpha = updateAlpha * (1 - blendFrac * (1 - alphaMinScale));
                    end
                end
                oldFilterPos = shadowRecoveredFilterPos;
                oldFilterEpoch = shadowRecoveredFilterEpoch;
                oldClockM = shadowRecoveredFilterClockM;
                if all(isfinite(predPos)) && filterHasState
                    shadowRecoveredFilterPos = (1 - updateAlpha) * predPos(:) + updateAlpha * closedLoopPos(:);
                else
                    shadowRecoveredFilterPos = closedLoopPos(:);
                end
                if all(isfinite(ds5AbsAnchorCurrPos)) && localGetSettingValue(settings, 'deepShadowDs5RecoveredFilterAbsAnchorEnable', 1) ~= 0 && ~recoveredFilterResetNow
                    absAnchorBlend = max(0, min(1, localGetSettingValue(settings, 'deepShadowDs5RecoveredFilterAbsAnchorBlend', 0.02)));
                    if ~recoveredFilterAbsAnchorSoftPassNow
                        absAnchorBlendHigh = max(absAnchorBlend, min(1, localGetSettingValue(settings, 'deepShadowDs5RecoveredFilterAbsAnchorBlendHigh', 0.12)));
                        softMaxM = localGetSettingValue(settings, 'deepShadowDs5RecoveredFilterAbsAnchorSoftMaxDiffM', 900.0);
                        hardMaxM = localGetSettingValue(settings, 'deepShadowDs5RecoveredFilterAbsAnchorMaxDiffM', 1800.0);
                        if isfinite(softMaxM) && isfinite(hardMaxM) && hardMaxM > softMaxM && isfinite(recoveredFilterAbsAnchorDiffMNow)
                            blendFrac = min(1, max(0, (recoveredFilterAbsAnchorDiffMNow - softMaxM) / max(hardMaxM - softMaxM, eps)));
                            absAnchorBlend = absAnchorBlend + blendFrac * (absAnchorBlendHigh - absAnchorBlend);
                        else
                            absAnchorBlend = absAnchorBlendHigh;
                        end
                    end
                    shadowRecoveredFilterPos = (1 - absAnchorBlend) * shadowRecoveredFilterPos(:) + absAnchorBlend * ds5AbsAnchorCurrPos(:);
                end
                measVel = nan(3,1);
                if all(isfinite(oldFilterPos)) && isfinite(oldFilterEpoch)
                    measDtSec = (currMeasNr - oldFilterEpoch) * localGetSettingValue(settings, 'navSolPeriod', 500.0) / 1000.0;
                    if isfinite(measDtSec) && measDtSec > 0
                        measVel = (closedLoopPos(:) - oldFilterPos(:)) / measDtSec;
                    end
                end
                measVelBlend = max(0, min(1, localGetSettingValue(settings, 'deepShadowDs5RecoveredFilterVelMeasBlend', 0.35)));
                if all(isfinite(measVel)) && all(isfinite(predVel))
                    shadowRecoveredFilterVel = (1 - measVelBlend) * predVel(:) + measVelBlend * measVel(:);
                elseif all(isfinite(predVel))
                    shadowRecoveredFilterVel = predVel(:);
                elseif all(isfinite(mainOutputVel))
                    shadowRecoveredFilterVel = mainOutputVel(:);
                end
                if localGetSettingValue(settings, 'deepShadowDs5RecoveredFilterClockEnable', 1) ~= 0 && refObsPosUsedNow && isfinite(refObsClockM)
                    clockAlpha = max(0, min(1, localGetSettingValue(settings, 'deepShadowDs5RecoveredFilterClockUpdateAlpha', 0.85)));
                    if recoveredFilterClockResetNow
                        shadowRecoveredFilterClockM = refObsClockM;
                    elseif isfinite(predClockM) && filterHasState
                        shadowRecoveredFilterClockM = (1 - clockAlpha) * predClockM + clockAlpha * refObsClockM;
                    else
                        shadowRecoveredFilterClockM = refObsClockM;
                    end
                    if recoveredFilterClockResetNow
                        shadowRecoveredFilterClockRateMps = 0;
                    elseif isfinite(oldClockM) && isfinite(oldFilterEpoch)
                        measDtSec = (currMeasNr - oldFilterEpoch) * localGetSettingValue(settings, 'navSolPeriod', 500.0) / 1000.0;
                        if isfinite(measDtSec) && measDtSec > 0
                            measClockRateMps = (refObsClockM - oldClockM) / measDtSec;
                            clockRateBlend = max(0, min(1, localGetSettingValue(settings, 'deepShadowDs5RecoveredFilterClockRateBlend', 0.35)));
                            if isfinite(shadowRecoveredFilterClockRateMps)
                                shadowRecoveredFilterClockRateMps = (1 - clockRateBlend) * shadowRecoveredFilterClockRateMps + clockRateBlend * measClockRateMps;
                            else
                                shadowRecoveredFilterClockRateMps = measClockRateMps;
                            end
                        end
                    else
                        shadowRecoveredFilterClockRateMps = 0;
                    end
                end
                shadowRecoveredFilterEpoch = currMeasNr;
                shadowRecoveredFilterCoastAge = 0;
                shadowRecoveredFilterLastUpdatePos = shadowRecoveredFilterPos(:);
                shadowRecoveredFilterLastUpdateEpoch = currMeasNr;
                if recoveredFilterResetNow
                    shadowRecoveredFilterGoodCount = 1;
                else
                    shadowRecoveredFilterGoodCount = shadowRecoveredFilterGoodCount + 1;
                end
                shadowRecoveredFilterBadCount = 0;
                recoveredFilterUpdatedNow = true;
                if recoveredFilterResetNow
                    recoveredFilterSourceNow = 3;
                else
                    recoveredFilterSourceNow = 1;
                end
            else
                shadowRecoveredFilterBadCount = shadowRecoveredFilterBadCount + 1;
            end
        elseif filterHasState && all(isfinite(predPos)) && ...
                shadowRecoveredFilterCoastAge < localGetSettingValue(settings, 'deepShadowDs5RecoveredFilterMaxCoastEpochs', 80)
            propCapEnable = localGetSettingValue(settings, 'deepShadowDs5RecoveredFilterPropCapEnable', 1) ~= 0;
            if all(isfinite(shadowRecoveredFilterPos))
                recoveredFilterPropStepMNow = norm(predPos(:) - shadowRecoveredFilterPos(:));
            end
            if all(isfinite(shadowRecoveredFilterLastUpdatePos))
                recoveredFilterPropTotalMNow = norm(predPos(:) - shadowRecoveredFilterLastUpdatePos(:));
            end
            propMaxStepM = localGetSettingValue(settings, 'deepShadowDs5RecoveredFilterPropMaxStepM', 120.0);
            propMaxTotalM = localGetSettingValue(settings, 'deepShadowDs5RecoveredFilterPropMaxTotalM', 600.0);
            propCapTolM = max(0, localGetSettingValue(settings, 'deepShadowDs5RecoveredFilterPropCapTolM', 1e-3));
            if propCapEnable
                recoveredFilterPropCapPassNow = (~isfinite(propMaxStepM) || propMaxStepM <= 0 || ~isfinite(recoveredFilterPropStepMNow) || recoveredFilterPropStepMNow <= propMaxStepM + propCapTolM) && ...
                    (~isfinite(propMaxTotalM) || propMaxTotalM <= 0 || ~isfinite(recoveredFilterPropTotalMNow) || recoveredFilterPropTotalMNow <= propMaxTotalM + propCapTolM);
            end
            if recoveredFilterPropCapPassNow
                shadowRecoveredFilterPos = predPos(:);
                if all(isfinite(predVel))
                    shadowRecoveredFilterVel = predVel(:);
                end
                if localGetSettingValue(settings, 'deepShadowDs5RecoveredFilterClockEnable', 1) ~= 0 && isfinite(predClockM)
                    shadowRecoveredFilterClockM = predClockM;
                end
                shadowRecoveredFilterEpoch = currMeasNr;
                if isfinite(shadowRecoveredFilterCoastAge)
                    shadowRecoveredFilterCoastAge = shadowRecoveredFilterCoastAge + 1;
                else
                    shadowRecoveredFilterCoastAge = 1;
                end
                recoveredFilterSourceNow = 2;
            else
                shadowRecoveredFilterBadCount = shadowRecoveredFilterBadCount + 1;
                shadowRecoveredFilterAuthority = false;
            end
        elseif ~filterHasState
            shadowRecoveredFilterBadCount = shadowRecoveredFilterBadCount + 1;
        end
        maxCoastEpochs = localGetSettingValue(settings, 'deepShadowDs5RecoveredFilterMaxCoastEpochs', 80);
        maxBadEpochs = localGetSettingValue(settings, 'deepShadowDs5RecoveredFilterMaxBadEpochs', 24);
        confirmEpochs = localGetSettingValue(settings, 'deepShadowDs5RecoveredFilterConfirmEpochs', 3);
        filterValidNow = all(isfinite(shadowRecoveredFilterPos)) && isfinite(shadowRecoveredFilterEpoch) && ...
            isfinite(shadowRecoveredFilterCoastAge) && shadowRecoveredFilterCoastAge <= maxCoastEpochs;
        recoveredFilterFilterValidPassNow = filterValidNow;
        authorityMaxCoastEpochs = localGetSettingValue(settings, 'deepShadowDs5RecoveredFilterAuthorityMaxCoastEpochs', 12);
        authorityBaselineDiffMaxM = localGetSettingValue(settings, 'deepShadowDs5RecoveredFilterAuthorityBaselineDiffMaxM', 700.0);
        authorityUseBaselineDiffHardCap = localGetSettingValue(settings, 'deepShadowDs5RecoveredFilterAuthorityUseBaselineDiffHardCap', 0) ~= 0;
        authorityMeasPredDiffMaxM = localGetSettingValue(settings, 'deepShadowDs5RecoveredFilterAuthorityMeasPredDiffMaxM', 500.0);
        authorityAbsAnchorMaxM = localGetSettingValue(settings, 'deepShadowDs5RecoveredFilterAuthorityAbsAnchorMaxM', ...
            localGetSettingValue(settings, 'deepShadowDs5RecoveredFilterAbsAnchorMaxDiffM', 1800.0));
        if baselineOutputValid && all(isfinite(baselineOutputPos)) && all(isfinite(shadowRecoveredFilterPos))
            recoveredFilterBaselineDiffMNow = norm(shadowRecoveredFilterPos(:) - baselineOutputPos(:));
        end
        authorityRequireAbsAnchor = localGetSettingValue(settings, 'deepShadowDs5RecoveredFilterAuthorityRequireAbsAnchor', 1) ~= 0;
        authorityAbsAnchorPass = ~authorityRequireAbsAnchor || ~isfinite(authorityAbsAnchorMaxM) || authorityAbsAnchorMaxM <= 0 || ...
            ~isfinite(recoveredFilterAbsAnchorDiffMNow) || recoveredFilterAbsAnchorDiffMNow <= authorityAbsAnchorMaxM;
        recoveredFilterAuthorityAbsAnchorPassNow = authorityAbsAnchorPass;
        recoveredFilterAuthorityMeasPredPassNow = ~isfinite(authorityMeasPredDiffMaxM) || authorityMeasPredDiffMaxM <= 0 || ...
            ~isfinite(recoveredFilterMeasPredDiffNow) || recoveredFilterMeasPredDiffNow <= authorityMeasPredDiffMaxM;
        recoveredFilterAuthorityCapPassNow = (~isfinite(authorityMaxCoastEpochs) || authorityMaxCoastEpochs <= 0 || ~isfinite(shadowRecoveredFilterCoastAge) || shadowRecoveredFilterCoastAge <= authorityMaxCoastEpochs) && ...
            (~authorityUseBaselineDiffHardCap || ~isfinite(authorityBaselineDiffMaxM) || authorityBaselineDiffMaxM <= 0 || ~isfinite(recoveredFilterBaselineDiffMNow) || recoveredFilterBaselineDiffMNow <= authorityBaselineDiffMaxM) && ...
            authorityAbsAnchorPass && ...
            recoveredFilterAuthorityMeasPredPassNow;
        if shadowRecoveredFilterBadCount > maxBadEpochs || ~filterValidNow || ~recoveredFilterAuthorityCapPassNow
            shadowRecoveredFilterAuthority = false;
        elseif shadowRecoveredFilterGoodCount >= confirmEpochs
            shadowRecoveredFilterAuthority = true;
        end
    end
    navResults.shadowRecoveredFilterValid(1, currMeasNr) = all(isfinite(shadowRecoveredFilterPos)) && isfinite(shadowRecoveredFilterEpoch);
    navResults.shadowRecoveredFilterAuthority(1, currMeasNr) = shadowRecoveredFilterAuthority;
    navResults.shadowRecoveredFilterUpdated(1, currMeasNr) = recoveredFilterUpdatedNow;
    navResults.shadowRecoveredFilterCoastAge(1, currMeasNr) = shadowRecoveredFilterCoastAge;
    navResults.shadowRecoveredFilterGoodCount(1, currMeasNr) = shadowRecoveredFilterGoodCount;
    navResults.shadowRecoveredFilterBadCount(1, currMeasNr) = shadowRecoveredFilterBadCount;
    navResults.shadowRecoveredFilterMeasPredDiffM(1, currMeasNr) = recoveredFilterMeasPredDiffNow;
    navResults.shadowRecoveredFilterSource(1, currMeasNr) = recoveredFilterSourceNow;
    navResults.shadowRecoveredFilterMeasPass(1, currMeasNr) = recoveredFilterMeasPassNow;
    navResults.shadowRecoveredFilterFilterValidPass(1, currMeasNr) = recoveredFilterFilterValidPassNow;
    navResults.shadowRecoveredFilterAuthorityAbsAnchorPass(1, currMeasNr) = recoveredFilterAuthorityAbsAnchorPassNow;
    navResults.shadowRecoveredFilterAuthorityMeasPredPass(1, currMeasNr) = recoveredFilterAuthorityMeasPredPassNow;
    navResults.shadowRecoveredFilterPropStepM(1, currMeasNr) = recoveredFilterPropStepMNow;
    navResults.shadowRecoveredFilterPropTotalM(1, currMeasNr) = recoveredFilterPropTotalMNow;
    navResults.shadowRecoveredFilterPropCapPass(1, currMeasNr) = recoveredFilterPropCapPassNow;
    navResults.shadowRecoveredFilterAuthorityCapPass(1, currMeasNr) = recoveredFilterAuthorityCapPassNow;
    navResults.shadowRecoveredFilterBaselineDiffM(1, currMeasNr) = recoveredFilterBaselineDiffMNow;
    navResults.shadowRecoveredFilterAbsAnchorValid(1, currMeasNr) = ds5RecoveredAbsAnchorValid && all(isfinite(ds5AbsAnchorCurrPos));
    navResults.shadowRecoveredFilterAbsAnchorFrozen(1, currMeasNr) = ds5RecoveredAbsAnchorFrozen;
    navResults.shadowRecoveredFilterAbsAnchorSource(1, currMeasNr) = ds5RecoveredAbsAnchorSource;
    navResults.shadowRecoveredFilterAbsAnchorAgeEpochs(1, currMeasNr) = ds5AbsAnchorCurrAgeEpochs;
    navResults.shadowRecoveredFilterAbsAnchorDiffM(1, currMeasNr) = recoveredFilterAbsAnchorDiffMNow;
    navResults.shadowRecoveredFilterAbsAnchorSoftPass(1, currMeasNr) = recoveredFilterAbsAnchorSoftPassNow;
    navResults.shadowRecoveredFilterAbsAnchorGatePass(1, currMeasNr) = recoveredFilterAbsAnchorGatePassNow;
    if all(isfinite(ds5AbsAnchorCurrPos))
        navResults.shadowRecoveredFilterAbsAnchorX(1, currMeasNr) = ds5AbsAnchorCurrPos(1);
        navResults.shadowRecoveredFilterAbsAnchorY(1, currMeasNr) = ds5AbsAnchorCurrPos(2);
        navResults.shadowRecoveredFilterAbsAnchorZ(1, currMeasNr) = ds5AbsAnchorCurrPos(3);
    end
    if all(isfinite(ds5AbsAnchorCurrVel))
        navResults.shadowRecoveredFilterAbsAnchorVX(1, currMeasNr) = ds5AbsAnchorCurrVel(1);
        navResults.shadowRecoveredFilterAbsAnchorVY(1, currMeasNr) = ds5AbsAnchorCurrVel(2);
        navResults.shadowRecoveredFilterAbsAnchorVZ(1, currMeasNr) = ds5AbsAnchorCurrVel(3);
    end
    navResults.shadowRecoveredFilterDopplerVelValid(1, currMeasNr) = recoveredFilterDopplerVelValidNow;
    navResults.shadowRecoveredFilterDopplerVelSatNum(1, currMeasNr) = recoveredFilterDopplerVelSatNumNow;
    navResults.shadowRecoveredFilterDopplerVelRmsMps(1, currMeasNr) = recoveredFilterDopplerVelRmsMpsNow;
    navResults.shadowRecoveredFilterDopplerVelP95Mps(1, currMeasNr) = recoveredFilterDopplerVelP95MpsNow;
    navResults.shadowRecoveredFilterDopplerVelSign(1, currMeasNr) = recoveredFilterDopplerVelSignNow;
    navResults.shadowRecoveredFilterDopplerVelSource(1, currMeasNr) = recoveredFilterDopplerVelSourceNow;
    if all(isfinite(recoveredFilterDopplerVelNow))
        navResults.shadowRecoveredFilterDopplerVelX(1, currMeasNr) = recoveredFilterDopplerVelNow(1);
        navResults.shadowRecoveredFilterDopplerVelY(1, currMeasNr) = recoveredFilterDopplerVelNow(2);
        navResults.shadowRecoveredFilterDopplerVelZ(1, currMeasNr) = recoveredFilterDopplerVelNow(3);
    end
    if all(isfinite(shadowRecoveredFilterPos))
        navResults.shadowRecoveredFilterX(1, currMeasNr) = shadowRecoveredFilterPos(1);
        navResults.shadowRecoveredFilterY(1, currMeasNr) = shadowRecoveredFilterPos(2);
        navResults.shadowRecoveredFilterZ(1, currMeasNr) = shadowRecoveredFilterPos(3);
    end
    if all(isfinite(shadowRecoveredFilterVel))
        navResults.shadowRecoveredFilterVX(1, currMeasNr) = shadowRecoveredFilterVel(1);
        navResults.shadowRecoveredFilterVY(1, currMeasNr) = shadowRecoveredFilterVel(2);
        navResults.shadowRecoveredFilterVZ(1, currMeasNr) = shadowRecoveredFilterVel(3);
    end
    navResults.shadowRecoveredFilterClockM(1, currMeasNr) = shadowRecoveredFilterClockM;
    navResults.shadowRecoveredFilterClockRateMps(1, currMeasNr) = shadowRecoveredFilterClockRateMps;
    navResults.shadowRecoveredFilterClockMeasPredDiffM(1, currMeasNr) = recoveredFilterClockMeasPredDiffNow;
    navResults.shadowRecoveredFilterClockReset(1, currMeasNr) = recoveredFilterClockResetNow;

    finalRecoveryCandidatePos = closedLoopPos;
    finalRecoveryCandidateVel = shadowClosedLoopLastVel;
    finalRecoveryCandidateSource = closedLoopSource;
    finalDs5RefObsRecoveryNow = false;
    ds5ObsContractFinalPass = true;
    ds5ObsContractFinalRawPass = true;
    ds5ObsContractFinalTrackingHold = false;
    ds5ObsContractFinalBaseSeedHold = false;
    if localGetSettingValue(settings, 'deepShadowDs5ObsContractEnable', 1) ~= 0 && ...
            (localGetSettingValue(settings, 'deepShadowDs5FinalRequireObsContractForRecovered', 1) ~= 0 || ...
             localGetSettingValue(settings, 'deepShadowDs5FinalRequireObsContractForHold', 1) ~= 0)
        ds5ObsContractFinalRawPass = isfield(navResults, 'shadowDs5ObsContractPass') && ...
            currMeasNr <= numel(navResults.shadowDs5ObsContractPass) && ...
            logical(navResults.shadowDs5ObsContractPass(1, currMeasNr));
        ds5ObsContractFinalPass = ds5ObsContractFinalRawPass;
        if ~ds5ObsContractFinalPass && localGetSettingValue(settings, 'deepShadowDs5FinalAllowObsContractTrackingHold', 0) ~= 0 && ...
                shadowRecoveredFilterAuthority && recoveredFilterAuthorityCapPassNow
            obsMinSat = max(4, round(localGetSettingValue(settings, 'deepShadowDs5ObsContractMinSat', 4)));
            trackingOnlyDrop = isfield(navResults, 'shadowDs5ObsContractUsedSatNum') && ...
                isfield(navResults, 'shadowDs5ObsContractCommonClockPass') && ...
                isfield(navResults, 'shadowDs5ObsContractSpreadPass') && ...
                isfield(navResults, 'shadowDs5ObsContractBaseSeedPass') && ...
                isfield(navResults, 'shadowDs5ObsContractTrackingPass') && ...
                currMeasNr <= numel(navResults.shadowDs5ObsContractUsedSatNum) && ...
                navResults.shadowDs5ObsContractUsedSatNum(1, currMeasNr) >= obsMinSat && ...
                logical(navResults.shadowDs5ObsContractCommonClockPass(1, currMeasNr)) && ...
                logical(navResults.shadowDs5ObsContractSpreadPass(1, currMeasNr)) && ...
                logical(navResults.shadowDs5ObsContractBaseSeedPass(1, currMeasNr)) && ...
                ~logical(navResults.shadowDs5ObsContractTrackingPass(1, currMeasNr));
            if trackingOnlyDrop
                ds5ObsContractFinalPass = true;
                ds5ObsContractFinalTrackingHold = true;
            end
        end
        if ~ds5ObsContractFinalPass && localGetSettingValue(settings, 'deepShadowDs5FinalAllowObsContractBaseSeedHold', 0) ~= 0 && ...
                shadowRecoveredFilterAuthority && recoveredFilterAuthorityCapPassNow
            obsMinSat = max(4, round(localGetSettingValue(settings, 'deepShadowDs5ObsContractMinSat', 4)));
            baseSeedOnlyDrop = isfield(navResults, 'shadowDs5ObsContractUsedSatNum') && ...
                isfield(navResults, 'shadowDs5ObsContractCommonClockPass') && ...
                isfield(navResults, 'shadowDs5ObsContractSpreadPass') && ...
                isfield(navResults, 'shadowDs5ObsContractBaseSeedPass') && ...
                isfield(navResults, 'shadowDs5ObsContractTrackingPass') && ...
                currMeasNr <= numel(navResults.shadowDs5ObsContractUsedSatNum) && ...
                navResults.shadowDs5ObsContractUsedSatNum(1, currMeasNr) >= obsMinSat && ...
                logical(navResults.shadowDs5ObsContractCommonClockPass(1, currMeasNr)) && ...
                logical(navResults.shadowDs5ObsContractSpreadPass(1, currMeasNr)) && ...
                logical(navResults.shadowDs5ObsContractTrackingPass(1, currMeasNr)) && ...
                ~logical(navResults.shadowDs5ObsContractBaseSeedPass(1, currMeasNr));
            if baseSeedOnlyDrop
                ds5ObsContractFinalPass = true;
                ds5ObsContractFinalBaseSeedHold = true;
            end
        end
    end
    if localGetSettingValue(settings, 'deepShadowDs5FinalRecoveredAuthorityEnable', 1) ~= 0 && ...
            shadowRecoveredFilterAuthority && all(isfinite(shadowRecoveredFilterPos)) && ds5ObsContractFinalPass
        finalRecoveryCandidatePos = shadowRecoveredFilterPos(:);
        finalRecoveryCandidateVel = shadowRecoveredFilterVel(:);
        finalRecoveryCandidateSource = 6;
        finalDs5RefObsRecoveryNow = true;
    end
    navResults.shadowBaselineOutputX(1, currMeasNr) = baselineOutputPos(1);
    navResults.shadowBaselineOutputY(1, currMeasNr) = baselineOutputPos(2);
    navResults.shadowBaselineOutputZ(1, currMeasNr) = baselineOutputPos(3);
    navResults.shadowBaselineOutputVX(1, currMeasNr) = baselineOutputVel(1);
    navResults.shadowBaselineOutputVY(1, currMeasNr) = baselineOutputVel(2);
    navResults.shadowBaselineOutputVZ(1, currMeasNr) = baselineOutputVel(3);
    navResults.shadowBaselineOutputValid(1, currMeasNr) = baselineOutputValid;
    navResults.shadowBaselineOutputHoldAge(1, currMeasNr) = baselineOutputHoldAgeNow;

    baselineProxyP95Now = localComputePosResidualProxyP95(baselineOutputPos, baselineOutputValid, navSolut.satPositions, rawPUsed, navSolut.satClkCorr, settings);
    recoveryProxyP95Now = localComputePosResidualProxyP95(finalRecoveryCandidatePos, all(isfinite(finalRecoveryCandidatePos)), navSolut.satPositions, rawPUsed, navSolut.satClkCorr, settings);
    recoveryProxyImproveNow = baselineProxyP95Now - recoveryProxyP95Now;

    finalOutputPos = mainOutputPos;
    finalOutputVel = mainOutputVel;
    finalOutputSource = 0;
    finalOutputUseRecovered = false;
    finalOutputHoldAgeNow = nan;
    finalOutputRecoveryGatePass = false;
    finalOutputRecoveryBaselineDiffM = nan;
    finalOutputContinuityDiffM = nan;
    finalOutputContinuityPass = true;
    finalOutputSlewApplied = false;
    finalOutputSlewStepM = nan;
    finalOutputHoldBaselineCompete = false;
    finalOutputHoldBaselineSelected = false;
    if localUseDs5FinalOutput(settings, currMeasNr)
        spoofConfirmedNow = (deepModeState == 2);
        [finalOutputPos, finalOutputVel, finalOutputSource, finalOutputUseRecovered, shadowFinalOutputLastPos, ...
            shadowFinalOutputLastVel, shadowFinalOutputLastEpoch, shadowFinalOutputLastSource, shadowFinalOutputHoldAge, shadowFinalOutputRecoveryConfirmCount, ...
            shadowFinalOutputPublishedLastPos, shadowFinalOutputPublishedLastVel, shadowFinalOutputPublishedLastEpoch, finalOutputRecoveryGatePass, finalOutputRecoveryBaselineDiffM, finalOutputContinuityDiffM, finalOutputContinuityPass, ...
            finalOutputSlewApplied, finalOutputSlewStepM, finalOutputHoldBaselineCompete, finalOutputHoldBaselineSelected] = ...
            localBuildDs5FinalOutput(mainOutputPos, mainOutputVel, baselineOutputPos, baselineOutputVel, baselineOutputValid, ...
                baselineProxyP95Now, recoveryProxyP95Now, finalRecoveryCandidatePos, finalRecoveryCandidateSource, currMeasNr, spoofConfirmedNow, ...
                finalRecoveryCandidateVel, shadowClosedLoopLastEpoch, shadowFinalOutputLastPos, shadowFinalOutputLastVel, ...
                shadowFinalOutputLastEpoch, shadowFinalOutputLastSource, shadowFinalOutputHoldAge, shadowFinalOutputRecoveryConfirmCount, ...
                shadowFinalOutputPublishedLastPos, shadowFinalOutputPublishedLastVel, shadowFinalOutputPublishedLastEpoch, finalDs5RefObsRecoveryNow, ...
                shadowRecoveredFilterAuthority && recoveredFilterAuthorityCapPassNow, recoveredFilterBaselineDiffMNow, ds5ObsContractFinalPass, settings);
        finalOutputHoldAgeNow = shadowFinalOutputHoldAge;
    end
    navResults.shadowFinalOutputBaselineValid(1, currMeasNr) = baselineOutputValid;
    navResults.shadowFinalOutputRecoveryGatePass(1, currMeasNr) = finalOutputRecoveryGatePass;
    navResults.shadowFinalOutputRecoveryBaselineDiffM(1, currMeasNr) = finalOutputRecoveryBaselineDiffM;
    navResults.shadowFinalOutputBaselineProxyP95M(1, currMeasNr) = baselineProxyP95Now;
    navResults.shadowFinalOutputRecoveryProxyP95M(1, currMeasNr) = recoveryProxyP95Now;
    navResults.shadowFinalOutputRecoveryProxyImproveM(1, currMeasNr) = recoveryProxyImproveNow;
    navResults.shadowFinalOutputX(1, currMeasNr) = finalOutputPos(1);
    navResults.shadowFinalOutputY(1, currMeasNr) = finalOutputPos(2);
    navResults.shadowFinalOutputZ(1, currMeasNr) = finalOutputPos(3);
    navResults.shadowFinalOutputVX(1, currMeasNr) = finalOutputVel(1);
    navResults.shadowFinalOutputVY(1, currMeasNr) = finalOutputVel(2);
    navResults.shadowFinalOutputVZ(1, currMeasNr) = finalOutputVel(3);
    navResults.shadowFinalOutputSource(1, currMeasNr) = finalOutputSource;
    navResults.shadowFinalOutputHoldAge(1, currMeasNr) = finalOutputHoldAgeNow;
    navResults.shadowFinalOutputUseRecovered(1, currMeasNr) = finalOutputUseRecovered;
    navResults.shadowFinalOutputRecoveryConfirmCount(1, currMeasNr) = shadowFinalOutputRecoveryConfirmCount;
    navResults.shadowFinalOutputContinuityDiffM(1, currMeasNr) = finalOutputContinuityDiffM;
    navResults.shadowFinalOutputContinuityPass(1, currMeasNr) = finalOutputContinuityPass;
    navResults.shadowFinalOutputSlewApplied(1, currMeasNr) = finalOutputSlewApplied;
    navResults.shadowFinalOutputSlewStepM(1, currMeasNr) = finalOutputSlewStepM;
    navResults.shadowFinalOutputHoldBaselineCompete(1, currMeasNr) = finalOutputHoldBaselineCompete;
    navResults.shadowFinalOutputHoldBaselineSelected(1, currMeasNr) = finalOutputHoldBaselineSelected;
    navResults.shadowFinalObsContractRawPass(1, currMeasNr) = ds5ObsContractFinalRawPass;
    navResults.shadowFinalObsContractPass(1, currMeasNr) = ds5ObsContractFinalPass;
    navResults.shadowFinalObsContractTrackingHold(1, currMeasNr) = ds5ObsContractFinalTrackingHold;
    navResults.shadowFinalObsContractBaseSeedHold(1, currMeasNr) = ds5ObsContractFinalBaseSeedHold;

    % Doppler residual metrics: detector-channel GNSS measured - INS predicted.
    prnList = [trackDeepIn.PRN]';
    dopplerMeas = nan(numel(prnList), 1);
    dopplerPred = nan(numel(prnList), 1);
    LOSnMap = nan(numel(prnList), 3);
    if settings.deepDetectUseTrackResults
        dopplerPred = dopplerFeedback(:) + clkDriftHz;
        LOSnMap = LOS * Cen;
        for ii = 1:numel(prnList)
            chIdx = trackRawChIdx(ii);
            if settings.deepFastMode
                idxTrack = trackRawStartIdx(ii) + max(0, round((currMeasNr - 1) * settings.navSolPeriod));
            else
                idxTrack = trackRawStartIdx(ii) + max(0, round(trackDeepIn(ii).numOfCoInt));
            end
            carrArr = trackResults(chIdx).carrFreq;
            if idxTrack >= 1 && idxTrack <= numel(carrArr)
                dopplerMeas(ii) = carrArr(idxTrack) - settings.IF;
            end
        end
    else
        prnDetList = [trackDeepDet.PRN]';
        dopplerMeasAll = -navSolutDet.rawP_dot(:) / settings.c * 1575.42e6;
        dopplerPredAll = -vrsDet(:) / settings.c * 1575.42e6 + clkDriftHz;
        LOSnDetAll = LOSDet * CenDet;
        for ii = 1:numel(prnList)
            prn = prnList(ii);
            jj = find(prnDetList == prn, 1, 'first');
            if ~isempty(jj) && jj <= numel(dopplerMeasAll) && jj <= numel(dopplerPredAll)
                dopplerMeas(ii) = dopplerMeasAll(jj);
                dopplerPred(ii) = dopplerPredAll(jj);
                LOSnMap(ii, :) = LOSnDetAll(jj, :);
            end
        end
    end

    biasHz = zeros(numel(prnList), 1);
    for ii = 1:numel(prnList)
        prn = prnList(ii);
        if prn <= numel(settings.deepBiasByPrnHz)
            biasHz(ii) = settings.deepBiasByPrnHz(prn);
        end
    end

    navResults.dopplerMeasHz(1:numel(dopplerMeas), currMeasNr) = dopplerMeas;
    navResults.dopplerPredHz(1:numel(dopplerPred), currMeasNr) = dopplerPred;
    if localUseDs5TruePeak(settings, currMeasNr)
        tmpTrueRefHz = dopplerPred;
        if isfinite(ds5CommonDoppHz)
            tmpTrueRefHz = dopplerPred - ds5CommonDoppHz;
        end
        ds5TrueRefHzState(1:min(numel(tmpTrueRefHz), numActChnList)) = tmpTrueRefHz(1:min(numel(tmpTrueRefHz), numActChnList));
        ds5TrueRefReadyState(1:min(numel(tmpTrueRefHz), numActChnList)) = isfinite(tmpTrueRefHz(1:min(numel(tmpTrueRefHz), numActChnList)));
        navResults.shadowDs5TrueRefHz(1:min(numel(tmpTrueRefHz), numActChnList), currMeasNr) = tmpTrueRefHz(1:min(numel(tmpTrueRefHz), numActChnList));
        navResults.shadowDs5TrueRefCodeChips(:, currMeasNr) = ds5TrueRefCodeState(:);
        navResults.shadowDs5TrueRefWinHz(:, currMeasNr) = localGetSettingValue(settings, 'deepShadowDs5TruePeakFreqWindowHz', 125.0);
        navResults.shadowDs5TrueRefWinChips(:, currMeasNr) = localGetSettingValue(settings, 'deepShadowDs5TruePeakCodeWindowChips', 0.75);
        navResults.shadowDs5TrueRefReady(:, currMeasNr) = ds5TrueRefReadyState(:);
    end

    [ds5CommonDoppInfoLocal, ds5CommonDoppHz, ds5CommonDoppRateHzps, ds5CommonDoppLastEpoch] = ...
        localEstimateDs5CommonDopp(dopplerMeas, dopplerPred, biasHz, prnList, currMeasNr, ...
        ds5CommonDoppHz, ds5CommonDoppRateHzps, ds5CommonDoppLastEpoch, settings);
    if isstruct(ds5CommonDoppInfoLocal)
        navResults.shadowDs5CommonDoppApplied(1, currMeasNr) = ds5CommonDoppInfoLocal.applied;
        navResults.shadowDs5CommonDoppHz(1, currMeasNr) = ds5CommonDoppInfoLocal.commonHz;
        navResults.shadowDs5CommonDoppRateHzps(1, currMeasNr) = ds5CommonDoppInfoLocal.rateHzps;
        navResults.shadowDs5CommonDoppBaseSatNum(1, currMeasNr) = ds5CommonDoppInfoLocal.baseSatNum;
        navResults.shadowDs5CommonDoppKeepSatNum(1, currMeasNr) = ds5CommonDoppInfoLocal.keepSatNum;
        navResults.shadowDs5CommonDoppSource(1, currMeasNr) = ds5CommonDoppInfoLocal.source;
        navResults.shadowDs5CommonDoppPredHz(1, currMeasNr) = ds5CommonDoppInfoLocal.predHz;
        navResults.shadowDs5CommonDoppMeasHz(1, currMeasNr) = ds5CommonDoppInfoLocal.measHz;
        navResults.shadowDs5CommonDoppP95BeforeHz(1, currMeasNr) = ds5CommonDoppInfoLocal.p95BeforeHz;
        navResults.shadowDs5CommonDoppP95AfterHz(1, currMeasNr) = ds5CommonDoppInfoLocal.p95AfterHz;
        navResults.shadowDs5CommonDoppRmsBeforeHz(1, currMeasNr) = ds5CommonDoppInfoLocal.rmsBeforeHz;
        navResults.shadowDs5CommonDoppRmsAfterHz(1, currMeasNr) = ds5CommonDoppInfoLocal.rmsAfterHz;
        navResults.shadowDs5CommonDoppL2AfterHz(1, currMeasNr) = ds5CommonDoppInfoLocal.l2AfterHz;
        navResults.shadowDs5CommonDoppAlphaProxy(1, currMeasNr) = ds5CommonDoppInfoLocal.alphaProxy;
        navResults.shadowDs5CommonDoppAlphaGatePass(1, currMeasNr) = ds5CommonDoppInfoLocal.alphaGatePass;
    end

    % Tracking-channel residual (uses current deep-coupled channel NCO output).
    dopplerTrack = nan(numel(prnList), 1);
    carrErrTrack = nan(numel(prnList), 1);
    for ii = 1:numel(prnList)
        if ii <= numel(trackDeepIn)
            dopplerTrack(ii) = trackDeepIn(ii).carrFreq - settings.IF;
            carrErrTrack(ii) = trackDeepIn(ii).carrError;
        end
    end
    validTrack = isfinite(dopplerTrack) & isfinite(dopplerPred);
    residualTrackFull = nan(numel(prnList), 1);
    % Track residual is in the deep-loop/NCO domain; do not reuse detector
    % PRN bias calibration from trackResults/raw Doppler.
    residualTrackFull(validTrack) = dopplerTrack(validTrack) - dopplerPred(validTrack);
    navResults.shadowTrackResidualHz(1:numel(residualTrackFull), currMeasNr) = residualTrackFull;
    if any(validTrack)
        residualTrack = residualTrackFull(validTrack);
        medTrack = median(residualTrack);
        navResults.shadowTrackCmHz(1, currMeasNr) = abs(medTrack);
        navResults.shadowTrackDfHz(1, currMeasNr) = median(abs(residualTrack - medTrack));
    end
    navResults.shadowCarrErrMed(1, currMeasNr) = median(abs(carrErrTrack(isfinite(carrErrTrack))));

    shadowTargetHz = dopplerPred;
    if localUseDs5CommonDopp(settings, currMeasNr) && isfinite(ds5CommonDoppHz)
        shadowTargetHz = dopplerPred - ds5CommonDoppHz;
    end
    validDopp = isfinite(dopplerMeas) & isfinite(dopplerPred);
    if any(validDopp)
        residualFull = nan(numel(prnList), 1);
        residualFull(validDopp) = dopplerMeas(validDopp) - dopplerPred(validDopp) - biasHz(validDopp);
        navResults.residualHz(1:numel(residualFull), currMeasNr) = residualFull;
        residualHz = residualFull(validDopp);
        medResRaw = median(residualHz);
        navResults.metricCmRawHz(1, currMeasNr) = abs(medResRaw);
        navResults.metricDfRawHz(1, currMeasNr) = median(abs(residualHz - medResRaw));

        % Optional clean-template subtraction before slow-trend detrending.
        residualBaseFull = residualFull;
        if settings.deepUseReferenceResidual && ~isempty(settings.deepRefResidualHz) && ...
                size(settings.deepRefResidualHz, 1) >= numel(residualFull) && ...
                size(settings.deepRefResidualHz, 2) >= currMeasNr
            refCol = settings.deepRefResidualHz(1:numel(residualFull), currMeasNr);
            idxRef = validDopp & isfinite(refCol);
            residualBaseFull(idxRef) = residualBaseFull(idxRef) - refCol(idxRef);
        end
        navResults.residualBaseHz(1:numel(residualBaseFull), currMeasNr) = residualBaseFull;

        % Slow INS-model mismatch introduces long-term residual drift.
        residualDetFull = residualBaseFull;
        wDet = max(1, round(settings.deepDetrendWindowEpoch));
        if currMeasNr > 1 && wDet > 1
            epSt = max(1, currMeasNr - wDet);
            histRes = navResults.residualBaseHz(:, epSt:currMeasNr-1);
            baseRes = median(histRes, 2, 'omitnan');
            baseRes(~isfinite(baseRes)) = 0;
            residualDetFull(validDopp) = residualBaseFull(validDopp) - baseRes(validDopp);
        end
        residualDet = residualDetFull(validDopp);
        medRes = median(residualDet);
        navResults.metricCmHz(1, currMeasNr) = abs(medRes);
        navResults.metricDfHz(1, currMeasNr) = median(abs(residualDet - medRes));
        navResults.residualDetHz(1:numel(residualDetFull), currMeasNr) = residualDetFull;
        if settings.deepShadowReacqEnable
            if deepModeState == 2 && settings.deepShadowTargetPureInsInSpoof
                shadowTargetHz(validDopp) = dopplerPred(validDopp);
            else
                clipHz = max(0.1, settings.deepShadowResidualClipHz);
                residualClip = max(-clipHz, min(clipHz, residualDetFull(validDopp)));
                % Keep the shadow target in the same de-dragged frame as trueRef.
                % residualClip is only a local correction term and must not undo
                % the previously removed DS5 common Doppler component.
                shadowTargetHz(validDopp) = shadowTargetHz(validDopp) + residualClip;
            end
        end

        % Normalized innovation metric: sigma_i^2 = h_i*P*h_i' + R_i
        navResults.metricZ(1, currMeasNr) = nan;
        zNorm = nan(size(residualDetFull));
        if isfield(kf, 'Pxk') && ~isempty(kf.Pxk)
            velIdx = settings.deepVelStateIdx(:)';
            clkIdx = settings.deepClkStateIdx;
            for ii = 1:numel(residualDetFull)
                if ~validDopp(ii) || ~all(isfinite(LOSnMap(ii, :)))
                    continue;
                end
                h = zeros(1, size(kf.Pxk, 1));
                if all(velIdx >= 1) && all(velIdx <= size(kf.Pxk, 1)) && numel(velIdx) == 3
                    h(velIdx) = LOSnMap(ii, :) / lambdaL1;
                end
                if clkIdx >= 1 && clkIdx <= size(kf.Pxk, 1)
                    h(clkIdx) = 1 / lambdaL1;
                end
                sig2 = h * kf.Pxk * h' + settings.deepSigmaMeasHz^2;
                if sig2 > 0
                    zNorm(ii) = abs(residualDetFull(ii)) / sqrt(sig2);
                end
            end
            navResults.metricZ(1, currMeasNr) = median(zNorm(validDopp & isfinite(zNorm)));
        end

        tcmGateNow = settings.deepTcmHz;
        tdfGateNow = settings.deepTdfHz;
        tzGateNow = settings.deepTz;
        tsatGateNow = settings.deepTsatHz;
        hitMinNow = settings.deepHitMin;
        confirmNeedNow = settings.deepConfirmEpochs;
        if strictActive
            tcmGateNow = settings.deepReentryTcmScale * tcmGateNow;
            tdfGateNow = settings.deepReentryTdfScale * tdfGateNow;
            tzGateNow = settings.deepReentryTzScale * tzGateNow;
            tsatGateNow = settings.deepReentryTsatScale * tsatGateNow;
            hitMinNow = max(hitMinNow, settings.deepReentryHitMin);
            confirmNeedNow = max(confirmNeedNow, settings.deepReentryConfirmEpochs);
        end

        hitCntRes = sum(abs(residualDetFull(validDopp)) > tsatGateNow);
        hitCntZ = sum(zNorm(validDopp) > settings.deepKappaZ, 'omitnan');
        navResults.hitCount(1, currMeasNr) = max(hitCntRes, hitCntZ);

        if settings.deepSpoofDetectEnable && isArmed
            if holdActive && deepModeState < 2
                confirmCnt = 0;
                deepModeState = 0;
            else
                trig = ((navResults.metricCmHz(1, currMeasNr) > tcmGateNow) || ...
                        (navResults.metricDfHz(1, currMeasNr) > tdfGateNow) || ...
                        (navResults.metricZ(1, currMeasNr) > tzGateNow)) && ...
                        (navResults.hitCount(1, currMeasNr) >= hitMinNow);
                if trig
                    confirmCnt = confirmCnt + 1;
                    if deepModeState < 1
                        deepModeState = 1;
                    end
                else
                    confirmCnt = 0;
                    if deepModeState == 1
                        deepModeState = 0;
                    end
                end
                confirmMinSec = localGetSettingValue(settings, 'deepSpoofConfirmMinSec', -inf);
                forceConfirmSec = localGetSettingValue(settings, 'deepSpoofForceConfirmSec', nan);
                if isfinite(forceConfirmSec) && epochElapsedSec >= forceConfirmSec
                    deepModeState = 2;
                elseif confirmCnt >= confirmNeedNow && (~isfinite(confirmMinSec) || epochElapsedSec >= confirmMinSec)
                    deepModeState = 2;
                end
            end
            if settings.deepSpoofLatch && currMeasNr > 1 && navResults.spoofState(1, currMeasNr-1) == 2 && ...
                    ~(settings.deepShadowAllowReturnGnss && recoverySwitchedNow)
                deepModeState = 2;
            end
        elseif ~isArmed && deepModeState < 2
            confirmCnt = 0;
            deepModeState = 0;
        end

        if settings.deepShadowReacqEnable && deepModeState == 2
            if settings.deepShadowUseTrackResidualGate
                shadowResGate = residualTrackFull;
                shadowValidGate = validTrack;
            else
                shadowResGate = residualDetFull;
                shadowValidGate = validDopp;
            end
            shadowGood = shadowValidGate & isfinite(shadowResGate) & ...
                         (abs(shadowResGate) <= settings.deepShadowLockGateHz);
            if settings.deepShadowUseCarrErrorGate
                shadowGood = shadowGood & isfinite(carrErrTrack) & ...
                             (abs(carrErrTrack) <= settings.deepShadowCarrErrGateCycles);
            end
            if settings.deepShadowUseCodeReadyForRecovery
                shadowGood = shadowGood & shadowAcqReadyMask & shadowAcqQualifiedMask;
            end
            shadowStableCnt(shadowGood) = shadowStableCnt(shadowGood) + 1;
            shadowStableCnt(~shadowGood) = 0;
            shadowRecoveredMask = shadowStableCnt >= settings.deepShadowStableEpochs;
            recoveredSatNum = sum(shadowRecoveredMask);
            detectorReleaseOk = (navResults.metricCmHz(1, currMeasNr) <= settings.deepShadowReleaseTcmHz) && ...
                                (navResults.metricDfHz(1, currMeasNr) <= settings.deepShadowReleaseTdfHz) && ...
                                (navResults.metricZ(1, currMeasNr) <= settings.deepShadowReleaseTz);
            trackReleaseOk = isfinite(navResults.shadowTrackCmHz(1, currMeasNr)) && ...
                             isfinite(navResults.shadowTrackDfHz(1, currMeasNr)) && ...
                             (navResults.shadowTrackCmHz(1, currMeasNr) <= settings.deepShadowReleaseTrackCmHz) && ...
                             (navResults.shadowTrackDfHz(1, currMeasNr) <= settings.deepShadowReleaseTrackDfHz);
            if settings.deepShadowReleaseUseTrackMetric
                metricReleaseOk = trackReleaseOk;
                if settings.deepShadowReleaseNeedDetectorMetric
                    metricReleaseOk = metricReleaseOk && detectorReleaseOk;
                end
            else
                metricReleaseOk = detectorReleaseOk;
            end
            navReleaseOk = true;
            if settings.deepShadowReleaseUseNavResidual
                absDelta = abs(delta_rawP(isfinite(delta_rawP)));
                if isempty(absDelta)
                    navReleaseOk = false;
                else
                    navReleaseOk = (median(absDelta) <= settings.deepShadowReleaseNavMedM) && ...
                                   (prctile(absDelta, 95) <= settings.deepShadowReleaseNavP95M) && ...
                                   (max(absDelta) <= settings.deepShadowReleaseNavMaxM);
                end
            end
            recoveryReady = metricReleaseOk && navReleaseOk && ...
                            (recoveredSatNum >= settings.deepShadowMinRecoveredSat);
            if recoveryReady
                shadowReleaseCnt = shadowReleaseCnt + 1;
            else
                shadowReleaseCnt = 0;
            end
            navResults.shadowRecoveredSatNum(1, currMeasNr) = recoveredSatNum;
            navResults.shadowRecoveryReady(1, currMeasNr) = recoveryReady;
            navResults.shadowReleaseCounter(1, currMeasNr) = shadowReleaseCnt;
            if settings.deepShadowAllowReturnGnss && ...
                    shadowReleaseCnt >= settings.deepShadowReleaseConfirmEpochs
                deepModeState = 0;
                confirmCnt = 0;
                shadowReleaseCnt = 0;
                shadowRecoveryRampRemain = settings.deepShadowRecoveryRampEpochs;
                recoveryHoldCnt = settings.deepRecoveryHoldEpochs;
                reentryStrictCnt = settings.deepReentryStrictEpochs;
                recoverySwitchedNow = true;
                navResults.shadowRecoverySwitched(1, currMeasNr) = true;
            end
        else
            if deepModeState ~= 2
                shadowStableCnt = max(0, shadowStableCnt - 1);
                shadowRecoveredMask = shadowStableCnt >= settings.deepShadowStableEpochs;
                shadowReleaseCnt = 0;
            end
            navResults.shadowRecoveredSatNum(1, currMeasNr) = sum(shadowRecoveredMask);
            navResults.shadowRecoveryReady(1, currMeasNr) = false;
            navResults.shadowReleaseCounter(1, currMeasNr) = shadowReleaseCnt;
        end
    else
        shadowTargetHz = dopplerPred;
        if localUseDs5CommonDopp(settings, currMeasNr) && isfinite(ds5CommonDoppHz)
            shadowTargetHz = dopplerPred - ds5CommonDoppHz;
        end
        navResults.shadowRecoveredSatNum(1, currMeasNr) = sum(shadowRecoveredMask);
        navResults.shadowRecoveryReady(1, currMeasNr) = false;
        navResults.shadowReleaseCounter(1, currMeasNr) = shadowReleaseCnt;
    end
    navResults.shadowTargetHz(1:numel(shadowTargetHz), currMeasNr) = shadowTargetHz;
    navResults.spoofState(1, currMeasNr) = deepModeState;

    % 4. Clock-bias time alignment for next GNSS tracking step.
    clkBiasCorrM = 0;
    if (useGnssKfUpdateNow || useKfClockUpdateNow) && ...
            isfield(kf, 'xk') && numel(kf.xk) >= 2
        clkBiasCorrM = kf.xk(end-1);
    end
    for ii = 1 : numActChnList
        trackDeepIn(ii).recvTime = trackDeepIn(ii).recvTime - clkBiasCorrM / settings.c;
        if settings.deepUseIndependentDetectTrack
            trackDeepDet(ii).recvTime = trackDeepDet(ii).recvTime - clkBiasCorrM / settings.c;
        end
    end

    % 5. Next positioning time
    positioningTime = positioningTime + settings.navSolPeriod / 1000;

    aidFreq = localApplyDs5CommonDoppToAid(dopplerFeedback, ds5CommonDoppHz, settings, currMeasNr);
    navResults.shadowDs5AidDeDragHz(1, currMeasNr) = ds5CommonDoppHz;
    switch deepModeState
        case 2
            settings.deepAidWeight = settings.deepAidWeightSpoof;
            settings.deepClkWeight = settings.deepClkWeightSpoof;
            settings.pllNoiseBandwidth = settings.deepPllNoiseBandwidthSpoof;
        case 1
            settings.deepAidWeight = settings.deepAidWeightSuspect;
            settings.deepClkWeight = settings.deepClkWeightSuspect;
            settings.pllNoiseBandwidth = settings.deepPllNoiseBandwidthSuspect;
        otherwise
            settings.deepAidWeight = settings.deepAidWeightNormal;
            settings.deepClkWeight = settings.deepClkWeightNormal;
            settings.pllNoiseBandwidth = settings.deepPllNoiseBandwidthNormal;
    end
    settings.deepModeState = deepModeState;
    settings.deepClkDriftHz = clkDriftHz;
    settings.deepInterpDiv = max(1, settings.deepInterpDiv);
    shadowEnableNow = settings.deepShadowReacqEnable && (deepModeState >= 1);
    
    %% 娣辩粍鍚堝弽棣堟ā鍧?
    diagDeepFeedbackTic = tic;
    if settings.deepFastMode
        while (t0_gps + (t - t0_imu)) < positioningTime
            k1 = k + nn - 1;
            if k1 > size(trj.imu, 1)
                warning('IMU data exhausted before the next update; stop deep loop.');
                break;
            end
            wvm = trj.imu(k:k1,1:6);  t = trj.imu(k1,end);
            ins = insupdate(ins, wvm);
            kf.Phikk_1 = kffk(ins);
            kf = kfupdate(kf);
            k = k + nn;
        end
    else
        mainHoldActiveEpoch = shadowMainHoldCnt > 0;
        while (t0_gps + (t - t0_imu)) < positioningTime
            
            % 娌堣仾鐨勬搷浣滐紝鍙槻姝arrNco杩囧ぇ锛屾垜寰堥毦棰嗘偀鍏朵腑濂ュ
            if currMeasNr == 2
                for iii = 1 : numActChnList
                    trackDeepIn(iii).carrNco = 0;
                end
            end
                   
            % 6.1 璺宠繃褰撳墠鐩稿共绉垎
            for ii = 1 : numActChnList    
                % 璇ユ柟绋嬪彲鍙傝€冧竴浜涜鏂囷紝鎬讳箣姝ｇ‘鎬ф湁寰呴獙璇?
                if ii <= numel(shadowTargetHz) && isfinite(shadowTargetHz(ii))
                    settings.deepShadowTargetHz = shadowTargetHz(ii);
                else
                    settings.deepShadowTargetHz = nan;
                end
                settings.deepShadowReacqEnableNow = shadowEnableNow;
                settingsMain = settings;
                if ii <= numel(mainHoldActiveEpoch) && mainHoldActiveEpoch(ii)
                    settingsMain.deepBypassDllInSpoof = 1;
                    settingsMain.deepBypassPllInSpoof = 1;
                end
                diagLocalTic = tic;
                [trackDeepIn(ii), I_P, Q_P] = perChannelTrackOnce_DeepIn(trackDeepIn(ii), settingsMain, fid, oldAidFreq(ii), aidFreq(ii) - oldAidFreq(ii));
                diagTrackTimeSec = diagTrackTimeSec + toc(diagLocalTic);
                diagTrackCalls = diagTrackCalls + 1;
                if settings.deepUseIndependentDetectTrack
                    [trackDeepDet(ii), ~, ~] = perChannelTrackOnce(trackDeepDet(ii), settingsDet, fidDet);
                end
                
                if settings.deepKeepTrackHistory && ii == 1
                    I_P_1_list = [I_P_1_list, I_P];
                    Q_P_1_list = [Q_P_1_list, Q_P];
                end
                
                if settings.deepKeepTrackHistory
                    trackProcess(ii).codeErrorList = [trackProcess(ii).codeErrorList, trackDeepIn(ii).codeError];
                    trackProcess(ii).carrErrorList = [trackProcess(ii).carrErrorList, trackDeepIn(ii).carrError];
                    trackProcess(ii).codeFreqList = [trackProcess(ii).codeFreqList, trackDeepIn(ii).codeFreq];
                    trackProcess(ii).carrFreqList = [trackProcess(ii).carrFreqList, trackDeepIn(ii).carrFreq];
                    trackProcess(ii).PLI = [trackProcess(ii).PLI, (I_P^2-Q_P^2)/(I_P^2+Q_P^2)];
                end
            end
            
            % 6.3 鎯杩愯鑷充笅涓€娆℃洿鏂版椂鍒伙紝鍗虫儻瀵兼洿鏂颁竴娆?
            k1 = k+nn-1;
            if k1 > size(trj.imu, 1)
                warning('IMU data exhausted before the next update; stop deep loop.');
                break;
            end
            wvm = trj.imu(k:k1,1:6);  t = trj.imu(k1,end);
            ins = insupdate(ins, wvm);
            kf.Phikk_1 = kffk(ins);
            kf = kfupdate(kf);

            k = k + nn;       
            
            % 6.4 璺熻釜鐜繍琛岃嚦鎯鏇存柊鏃跺埢
            for ii = 1 : numActChnList
                trackans = trackDeepIn(ii);
                if settings.deepUseIndependentDetectTrack
                    trackansDet = trackDeepDet(ii);
                end
                
                kkk = 1;
                
                imuGpsTime = t0_gps + (t - t0_imu);
                while trackans.recvTime < imuGpsTime
                    trackDeepIn(ii) = trackans;
                    % 鍙弬鑰冩煇浜涜鏂囷紝鎴戜篃鍙槸鎳備釜澶ф
                    if ii <= numel(shadowTargetHz) && isfinite(shadowTargetHz(ii))
                        settings.deepShadowTargetHz = shadowTargetHz(ii);
                    else
                        settings.deepShadowTargetHz = nan;
                    end
                    settings.deepShadowReacqEnableNow = shadowEnableNow;
                    settingsMain = settings;
                    if ii <= numel(mainHoldActiveEpoch) && mainHoldActiveEpoch(ii)
                        settingsMain.deepBypassDllInSpoof = 1;
                        settingsMain.deepBypassPllInSpoof = 1;
                    end
                    diagLocalTic = tic;
                    [trackans, I_P, Q_P] = perChannelTrackOnce_DeepIn(trackans, settingsMain, fid, oldAidFreq(ii), kkk * (aidFreq(ii) - oldAidFreq(ii)));
                    diagTrackTimeSec = diagTrackTimeSec + toc(diagLocalTic);
                    diagTrackCalls = diagTrackCalls + 1;
                    if settings.deepUseIndependentDetectTrack
                        [trackansDet, ~, ~] = perChannelTrackOnce(trackansDet, settingsDet, fidDet);
                    end
                    
                    kkk = kkk + 1;

                    if settings.deepKeepTrackHistory && trackans.recvTime < imuGpsTime
                        trackProcess(ii).codeErrorList = [trackProcess(ii).codeErrorList, trackans.codeError];
                        trackProcess(ii).carrErrorList = [trackProcess(ii).carrErrorList, trackans.carrError];
                        trackProcess(ii).codeFreqList = [trackProcess(ii).codeFreqList, trackans.codeFreq];
                        trackProcess(ii).carrFreqList = [trackProcess(ii).carrFreqList, trackans.carrFreq];
                        trackProcess(ii).PLI = [trackProcess(ii).PLI, (I_P^2-Q_P^2)/(I_P^2+Q_P^2)];
                        
                        if ii == 1
                            I_P_1_list = [I_P_1_list, I_P];
                            Q_P_1_list = [Q_P_1_list, Q_P];
                        end
                    end
                end
                trackDeepIn(ii) = trackans;
                if settings.deepUseIndependentDetectTrack
                    trackDeepDet(ii) = trackansDet;
                end
            end
                  
            
            % 6.5 閲嶆柊璁＄畻鎯鐨勫弽棣堥噺
            settings.recvTime = settings.recvTime + nts;
            navSolut = postNavTight(trackDeepIn, settings, eph, TOW);  % GNSS瑙傛祴鍊?
            [posxyz, ~] = blh2xyz(ins.pos);
            % correctedP = navSolut.rawP + navSolut.satClkCorr .* settings.c - kf.xk(end-1);  
            correctedP = navSolut.rawP;  % 鍜屼笂涓€琛屽尯鍒笉澶?
            [~, ~, ~, vrs] = rhoSatRec_zcj(navSolut.satPositions', posxyz, correctedP', navSolut.satVelocity', ins.vn);
            dopplerFeedback = -vrs / settings.c * 1575.42e6;  
            
            oldAidFreq = aidFreq;
            aidFreq = localApplyDs5CommonDoppToAid(dopplerFeedback, ds5CommonDoppHz, settings, currMeasNr);
        end
        shadowMainHoldCnt(mainHoldActiveEpoch) = max(shadowMainHoldCnt(mainHoldActiveEpoch) - 1, 0);
        holdExpired = shadowMainFollowActive & (shadowMainHoldCnt <= 0);
        shadowMainFollowActive(holdExpired) = false;
        shadowTrackActive(holdExpired) = false;
        shadowTrackPending(holdExpired) = false;
        shadowTrackPendingAge(holdExpired) = 0;
        shadowTrackValidateCnt(holdExpired) = 0;
        shadowTrackValidateHist(holdExpired, :) = false;
        shadowTrackPassReady(holdExpired) = false;
        shadowPassReadyBadCnt(holdExpired) = 0;
    end
    diagDeepFeedbackTimeSec = diagDeepFeedbackTimeSec + toc(diagDeepFeedbackTic);
       
    
    end  % if currMeasNr == 1
end

function dst = localSyncTrackState(dst, src)
fieldsToCopy = { ...
    'SamplePos', 'remCodePhase', 'remCarrPhase', 'codeFreq', 'carrFreq', ...
    'codeNco', 'carrNco', 'codeError', 'carrError', 'numOfCoInt', ...
    'recvTime', 'deepShadowFreqHz', 'deepShadowErrHz', 'deepPrevMode', ...
    'deepCarrFreqStartHz', 'deepCarrFreqEndHz', 'deepCarrCmdHz', ...
    'deepCarrBaseCmdHz', 'deepCarrShadowCmdHz', 'deepCarrAidFreqHz', ...
    'deepCarrShadowTargetHz', 'deepCarrShadowPullHz', 'deepCarrShadowWeight', ...
    'deepCarrForceShadow', 'deepCarrPllBypass', 'deepCarrOldNcoHz', ...
    'deepCarrNcoHz', 'deepCarrNcoStepHz', 'deepCarrOldErrorCycles', ...
    'deepCarrErrorRawCycles', 'deepCarrErrorScaledCycles', ...
    'deepShadowCodeRefUsed', 'deepShadowCodeRefOffsetChips', 'deepShadowCodeRefMetric', ...
    'deepShadowCodeRefPeakRatio', 'deepShadowCodeRefAppliedChips', ...
    'deepShadowCodeRefNcoPullHz', 'deepShadowCodeRefNcoStepHz', ...
    'absoluteSample', 'deepShadowCandOffsetChips', 'deepShadowCandFreqAbsHz', ...
    'deepAccumCarrierCycles', 'deepAccumCodeChips'};
for kk = 1:numel(fieldsToCopy)
    fn = fieldsToCopy{kk};
    if isfield(dst, fn) && isfield(src, fn)
        dst.(fn) = src.(fn);
    end
end
end

function trackVec = localAlignFastTrackState(trackVec, trackResults, rawChIdx, rawStartIdx, currMeasNr, settings, recvTime)
if isempty(trackVec) || isempty(trackResults) || isempty(rawChIdx) || isempty(rawStartIdx)
    return;
end
navStepMs = localGetSettingValue(settings, 'navSolPeriod', 500.0);
for ii = 1:numel(trackVec)
    if ii > numel(rawChIdx) || ii > numel(rawStartIdx)
        continue;
    end
    sampleClockBase = nan;
    if isfield(trackVec(ii), 'recvTime') && isfield(trackVec(ii), 'SamplePos') && ...
            isfinite(trackVec(ii).recvTime) && isfinite(trackVec(ii).SamplePos)
        sampleClockBase = trackVec(ii).recvTime - trackVec(ii).SamplePos / settings.samplingFreq;
    end
    chIdx = rawChIdx(ii);
    if ~isfinite(chIdx) || chIdx < 1 || chIdx > numel(trackResults)
        continue;
    end
    idx = rawStartIdx(ii) + max(0, round((currMeasNr - 1) * navStepMs));
    if ~isfinite(idx)
        continue;
    end
    idx = max(1, round(idx));
    idxMax = localTrackFieldLength(trackResults(chIdx), 'carrFreq');
    if idxMax <= 0
        continue;
    end
    idx = min(idx, idxMax);
    trackVec(ii) = localSetTrackFieldFromResults(trackVec(ii), trackResults(chIdx), 'carrFreq', idx);
    trackVec(ii) = localSetTrackFieldFromResults(trackVec(ii), trackResults(chIdx), 'codeFreq', idx);
    trackVec(ii) = localSetTrackFieldFromResults(trackVec(ii), trackResults(chIdx), 'remCodePhase', idx);
    trackVec(ii) = localSetTrackFieldFromResults(trackVec(ii), trackResults(chIdx), 'remCarrPhase', idx);
    trackVec(ii) = localSetTrackFieldFromResults(trackVec(ii), trackResults(chIdx), 'pllDiscr', idx, 'carrError');
    trackVec(ii) = localSetTrackFieldFromResults(trackVec(ii), trackResults(chIdx), 'dllDiscr', idx, 'codeError');
    sampleAligned = false;
    if isfield(trackResults(chIdx), 'absoluteSample') && numel(trackResults(chIdx).absoluteSample) >= idx
        sampleNow = trackResults(chIdx).absoluteSample(idx);
        if isfinite(sampleNow)
            trackVec(ii).SamplePos = sampleNow;
            if isfinite(sampleClockBase)
                trackVec(ii).recvTime = sampleClockBase + sampleNow / settings.samplingFreq;
                sampleAligned = true;
            end
        end
    end
    if ~sampleAligned && isfinite(recvTime)
        trackVec(ii).recvTime = recvTime;
    end
    if isfinite(rawStartIdx(ii))
        trackVec(ii).numOfCoInt = max(0, idx - round(rawStartIdx(ii)));
    end
end
end

function trackOne = localApplyFastDs5RefObsShadowState(trackOne, trackMain, trueRefHz, trueRefCodeChips, settings)
oldTrack = trackOne;
dtCandidates = [];
if isfield(oldTrack, 'recvTime') && isfield(trackMain, 'recvTime') && ...
        isfinite(oldTrack.recvTime) && isfinite(trackMain.recvTime)
    dtCandidates(end+1) = trackMain.recvTime - oldTrack.recvTime; %#ok<AGROW>
end
if isfield(oldTrack, 'SamplePos') && isfield(trackMain, 'SamplePos') && ...
        isfinite(oldTrack.SamplePos) && isfinite(trackMain.SamplePos)
    dtCandidates(end+1) = (trackMain.SamplePos - oldTrack.SamplePos) / settings.samplingFreq; %#ok<AGROW>
end
dtCandidates = dtCandidates(isfinite(dtCandidates) & dtCandidates > 0);
maxCoastSec = max(2.0, 5.0 * localGetSettingValue(settings, 'navSolPeriod', 500.0) / 1000.0);
dtCandidates = dtCandidates(dtCandidates <= maxCoastSec);
if isempty(dtCandidates)
    dtSec = 0.0;
else
    dtSec = max(dtCandidates);
end
trackOne = localSyncTrackState(trackOne, trackMain);
baseHz = trackMain.carrFreq - settings.IF;
if ~isfinite(baseHz)
    baseHz = trackOne.carrFreq - settings.IF;
end
targetHz = baseHz;
if isfinite(trueRefHz)
    targetHz = trueRefHz;
end
oldNcoHz = localGetTrackField(trackOne, 'carrNco', 0.0);
oldErrCycles = localGetTrackField(trackOne, 'carrError', 0.0);
pullHz = targetHz - baseHz;

trackOne.carrFreq = settings.IF + targetHz;
trackOne.codeFreq = settings.codeFreqBasis - localGetTrackField(trackOne, 'codeNco', 0.0) + targetHz / 1540;
if isfinite(trueRefCodeChips)
    trackOne.remCodePhase = mod(trueRefCodeChips, settings.codeLength);
elseif isfield(oldTrack, 'remCodePhase') && isfinite(oldTrack.remCodePhase)
    trackOne.remCodePhase = mod(oldTrack.remCodePhase + trackOne.codeFreq * dtSec, settings.codeLength);
end
trackOne.carrError = 0.0;
oldCodeCorr = localGetTrackField(oldTrack, 'deepCodePhaseCorrChips', nan);
if isfinite(trueRefCodeChips)
    trackOne.deepCodePhaseCorrChips = 0.0;
elseif isfinite(oldCodeCorr)
    trackOne.deepCodePhaseCorrChips = oldCodeCorr;
else
    trackOne.deepCodePhaseCorrChips = 0.0;
end
trackOne.deepShadowFreqHz = targetHz;
trackOne.deepShadowErrHz = 0.0;
trackOne.deepCarrFreqStartHz = baseHz;
trackOne.deepCarrFreqEndHz = targetHz;
trackOne.deepCarrCmdHz = targetHz;
trackOne.deepCarrBaseCmdHz = baseHz;
trackOne.deepCarrShadowCmdHz = targetHz;
trackOne.deepCarrAidFreqHz = 0.0;
trackOne.deepCarrShadowTargetHz = targetHz;
trackOne.deepCarrShadowPullHz = pullHz;
trackOne.deepCarrShadowWeight = 1.0;
trackOne.deepCarrForceShadow = true;
trackOne.deepCarrPllBypass = true;
trackOne.deepCarrOldNcoHz = oldNcoHz;
trackOne.deepCarrNcoHz = pullHz;
trackOne.deepCarrNcoStepHz = pullHz - oldNcoHz;
trackOne.deepCarrOldErrorCycles = oldErrCycles;
trackOne.deepCarrErrorRawCycles = 0.0;
trackOne.deepCarrErrorScaledCycles = 0.0;
trackOne.deepPrevMode = localGetSettingValue(settings, 'deepModeState', 2);
diagFields = {'deepPromptI', 'deepPromptQ', 'deepEarlyI', 'deepEarlyQ', ...
    'deepLateI', 'deepLateQ', 'deepDllDiscrRaw', 'deepDllDiscr'};
for kk = 1:numel(diagFields)
    fn = diagFields{kk};
    trackOne.(fn) = nan;
    if isfield(oldTrack, fn) && isfinite(oldTrack.(fn))
        trackOne.(fn) = oldTrack.(fn);
    end
end
trackOne.deepShadowCodeRefAppliedChips = 0.0;
trackOne.deepShadowCodeRefNcoPullHz = 0.0;
trackOne.deepShadowCodeRefNcoStepHz = 0.0;
if isfield(oldTrack, 'deepAccumCarrierCycles') && isfinite(oldTrack.deepAccumCarrierCycles) && ...
        dtSec > 0
    trackOne.deepAccumCarrierCycles = oldTrack.deepAccumCarrierCycles + targetHz * dtSec;
end
if isfield(oldTrack, 'deepAccumCodeChips') && isfinite(oldTrack.deepAccumCodeChips) && ...
        dtSec > 0
    trackOne.deepAccumCodeChips = oldTrack.deepAccumCodeChips + trackOne.codeFreq * dtSec;
end

if isfinite(trueRefCodeChips) && isfinite(trackOne.remCodePhase)
    effCode = trackOne.remCodePhase + localGetTrackField(trackOne, 'deepCodePhaseCorrChips', 0.0);
    trackOne.deepShadowCodeRefUsed = true;
    trackOne.deepShadowCodeRefOffsetChips = localWrapChipDiff(trueRefCodeChips - effCode);
    trackOne.deepShadowCodeRefMetric = nan;
    trackOne.deepShadowCodeRefPeakRatio = inf;
else
    trackOne.deepShadowCodeRefUsed = false;
    trackOne.deepShadowCodeRefOffsetChips = nan;
    trackOne.deepShadowCodeRefMetric = nan;
    trackOne.deepShadowCodeRefPeakRatio = nan;
end
end

function startEpoch = localDs5TimedStartEpoch(settings, startSec)
navSolPeriodMs = localGetSettingValue(settings, 'navSolPeriod', 500.0);
dtSec = max(eps, navSolPeriodMs / 1000.0);
startEpoch = max(1, floor(double(startSec) / dtSec) + 1);
end

function n = localTrackFieldLength(trackResult, fieldName)
n = 0;
if isfield(trackResult, fieldName)
    n = numel(trackResult.(fieldName));
end
end

function trackOne = localSetTrackFieldFromResults(trackOne, trackResult, resultField, idx, trackField)
if nargin < 5 || isempty(trackField)
    trackField = resultField;
end
if ~isfield(trackResult, resultField) || numel(trackResult.(resultField)) < idx
    return;
end
val = trackResult.(resultField)(idx);
if isfinite(val)
    trackOne.(trackField) = val;
end
end

function tf = localUseDs5TruePeak(settings, currMeasNr)
tf = false;
if ~isfield(settings, 'deepShadowDs5TruePeakEnable') || ~settings.deepShadowDs5TruePeakEnable
    return;
end
if ~localIsDs5Scenario(settings)
    return;
end
startSec = localGetSettingValue(settings, 'deepShadowDs5TruePeakStartSec', 92.0);
navSolPeriodMs = localGetSettingValue(settings, 'navSolPeriod', 500.0);
timeSec = (double(currMeasNr) - 1.0) * navSolPeriodMs / 1000.0;
tf = timeSec >= startSec;
end

function tf = localCandidateInDs5TrueWindow(freqHz, codeChips, refHz, refCodeChips, settings)
tf = isfinite(freqHz) && isfinite(refHz);
if ~tf
    return;
end
freqWinHz = localGetSettingValue(settings, 'deepShadowDs5TruePeakFreqWindowHz', 125.0);
tf = abs(freqHz - refHz) <= freqWinHz;
if ~tf
    return;
end
if localGetSettingValue(settings, 'deepShadowDs5TruePeakUseCodeRef', 1) && isfinite(refCodeChips) && isfinite(codeChips)
    codeWinChips = localGetSettingValue(settings, 'deepShadowDs5TruePeakCodeWindowChips', 0.75);
    tf = abs(codeChips - refCodeChips) <= codeWinChips;
end
end

function score = localScoreDs5Peak(freqHz, codeChips, improveM, metricVal, refHz, refCodeChips, settings)
score = -inf;
if ~isfinite(freqHz) || ~isfinite(refHz)
    return;
end
freqWinHz = max(localGetSettingValue(settings, 'deepShadowDs5TruePeakFreqWindowHz', 125.0), eps);
freqScore = max(0.0, 1.0 - abs(freqHz - refHz) / freqWinHz);
codeScore = 0.5;
if localGetSettingValue(settings, 'deepShadowDs5TruePeakUseCodeRef', 1) && isfinite(refCodeChips) && isfinite(codeChips)
    codeWinChips = max(localGetSettingValue(settings, 'deepShadowDs5TruePeakCodeWindowChips', 0.75), eps);
    codeScore = max(0.0, 1.0 - abs(codeChips - refCodeChips) / codeWinChips);
end
residFloorM = abs(localGetSettingValue(settings, 'deepShadowDs5TruePeakRevokeImproveMaxM', -150.0));
if isfinite(improveM)
    residScore = min(1.0, max(0.0, (improveM + residFloorM) / max(2.0 * residFloorM, eps)));
else
    residScore = 0.0;
end
metricScore = 0.0;
if isfinite(metricVal)
    metricScore = min(1.0, max(0.0, log(max(metricVal, eps) + 1.0) / log(2.0)));
end
score = localGetSettingValue(settings, 'deepShadowDs5TruePeakFreqWeight', 0.40) * freqScore + ...
    localGetSettingValue(settings, 'deepShadowDs5TruePeakCodeWeight', 0.20) * codeScore + ...
    localGetSettingValue(settings, 'deepShadowDs5TruePeakResidualWeight', 0.20) * residScore + ...
    localGetSettingValue(settings, 'deepShadowDs5TruePeakMetricWeight', 0.20) * metricScore;
end

function val = localGetTrackField(trackState, fieldName, defaultVal)
val = defaultVal;
if isfield(trackState, fieldName) && isfinite(trackState.(fieldName))
    val = trackState.(fieldName);
end
end

function shadowRaw = localInitShadowRawResult()
shadowRaw = struct();
shadowRaw.valid = false;
shadowRaw.bestOffsetChips = nan;
shadowRaw.bestFreqHz = nan;
shadowRaw.peakRatio = nan;
shadowRaw.zeroRatio = nan;
shadowRaw.topKOffsetsChips = nan(1, 3);
shadowRaw.topKFreqHz = nan(1, 3);
shadowRaw.topKMetrics = nan(1, 3);
shadowRaw.topKPeakRatios = nan(1, 3);
end

function [trackShadow, pendingFlag, pendingAge, validateCnt, validateHistRow, initDiag] = ...
    localStartShadowPending(trackMain, candOffsetChips, candOffsetSamples, candFreqHz, settings, validateHistRow)
trackShadow = trackMain;
initDiag = struct('recvTimeShiftMs', nan, 'sampleShift', nan, 'numCoIntShift', nan);
codePhaseStepShadow = trackMain.codeFreq / settings.samplingFreq;
if ~isfinite(candOffsetSamples)
    sampleAdjustInt = round(candOffsetChips / max(codePhaseStepShadow, eps));
else
    sampleAdjustInt = round(candOffsetSamples);
end
chipResidual = candOffsetChips - sampleAdjustInt * codePhaseStepShadow;
rawCodePhase = trackMain.remCodePhase + chipResidual;
codeMsAdjust = floor(rawCodePhase / 1023.0);
trackShadow.remCodePhase = rawCodePhase - codeMsAdjust * 1023.0;
if trackShadow.remCodePhase < 0
    trackShadow.remCodePhase = trackShadow.remCodePhase + 1023.0;
    codeMsAdjust = codeMsAdjust - 1;
end
trackShadow.numOfCoInt = trackMain.numOfCoInt + codeMsAdjust;
trackShadow.SamplePos = max(0, trackMain.SamplePos + sampleAdjustInt);
sampleClockBase = trackMain.recvTime - trackMain.SamplePos / settings.samplingFreq;
trackShadow.recvTime = sampleClockBase + trackShadow.SamplePos / settings.samplingFreq;
if isfield(trackShadow, 'absoluteSample')
    trackShadow.absoluteSample = trackShadow.SamplePos;
end
trackShadow.carrFreq = candFreqHz;
trackShadow.deepShadowFreqHz = candFreqHz - settings.IF;
trackShadow.deepShadowErrHz = 0;
trackShadow.deepShadowCandOffsetChips = candOffsetChips;
trackShadow.deepShadowCandFreqAbsHz = candFreqHz;
trackShadow.deepShadowInitHold = settings.deepShadowRawTrackProtectEpochs;
initDiag.recvTimeShiftMs = (trackShadow.recvTime - trackMain.recvTime) * 1e3;
initDiag.sampleShift = trackShadow.SamplePos - trackMain.SamplePos;
initDiag.numCoIntShift = trackShadow.numOfCoInt - trackMain.numOfCoInt;
pendingFlag = true;
pendingAge = 0;
validateCnt = 0;
validateHistRow(:) = false;
end

function [clusterOffset, clusterFreqHz, clusterScore, clusterHits, clusterMiss] = ...
    localUpdateShadowClusters(clusterOffset, clusterFreqHz, clusterScore, clusterHits, clusterMiss, candOffsets, candFreqHz, candMetrics, settings)
assocTol = settings.deepShadowRawClusterAssocTolChips;
assocTolFreq = settings.deepShadowRawClusterAssocTolFreqHz;
hitGain = settings.deepShadowRawClusterHitGain;
missDecay = settings.deepShadowRawClusterMissDecay;
matched = false(size(clusterOffset));
validCand = isfinite(candOffsets) & isfinite(candFreqHz) & isfinite(candMetrics);
candOffsets = candOffsets(validCand);
candFreqHz = candFreqHz(validCand);
candMetrics = candMetrics(validCand);
for ii = 1:numel(candOffsets)
    bestIdx = 0;
    bestDist = inf;
    for jj = 1:numel(clusterOffset)
        if matched(jj)
            continue;
        end
        if isfinite(clusterOffset(jj))
            d = abs(candOffsets(ii) - clusterOffset(jj));
        else
            d = inf;
        end
        df = inf;
        if isfinite(clusterFreqHz(jj))
            df = abs(candFreqHz(ii) - clusterFreqHz(jj));
        end
        if d <= assocTol && df <= assocTolFreq
            jointDist = d / max(assocTol, eps) + df / max(assocTolFreq, eps);
            if jointDist < bestDist
                bestDist = jointDist;
                bestIdx = jj;
            end
        end
    end
    if bestIdx == 0
        emptyIdx = find(~isfinite(clusterOffset), 1, 'first');
        if isempty(emptyIdx)
            [~, emptyIdx] = min(clusterScore);
        end
        bestIdx = emptyIdx;
    end
    matched(bestIdx) = true;
    if isfinite(clusterOffset(bestIdx))
        clusterOffset(bestIdx) = 0.7 * clusterOffset(bestIdx) + 0.3 * candOffsets(ii);
        clusterFreqHz(bestIdx) = 0.7 * clusterFreqHz(bestIdx) + 0.3 * candFreqHz(ii);
    else
        clusterOffset(bestIdx) = candOffsets(ii);
        clusterFreqHz(bestIdx) = candFreqHz(ii);
    end
    clusterScore(bestIdx) = clusterScore(bestIdx) + ...
        hitGain * max(0.1, min(2.0, candMetrics(ii) / max(max(candMetrics), eps)));
    clusterHits(bestIdx) = clusterHits(bestIdx) + 1;
    clusterMiss(bestIdx) = 0;
end
for jj = 1:numel(clusterOffset)
    if ~matched(jj)
        clusterScore(jj) = missDecay * clusterScore(jj);
        clusterMiss(jj) = clusterMiss(jj) + 1;
        if clusterMiss(jj) >= 3 && clusterScore(jj) < 0.5
            clusterOffset(jj) = nan;
            clusterFreqHz(jj) = nan;
            clusterScore(jj) = 0;
            clusterHits(jj) = 0;
            clusterMiss(jj) = 0;
        end
    end
end
end

function [candIdx, diag] = localPickShadowCandidateByCluster(candOffsets, candFreqHz, candMetrics, usedMask, clusterOffset, clusterFreqHz, clusterScore, clusterHits, settings, failExcludeValid, failExcludeOffset, failExcludeFreqHz)
if nargin < 10, failExcludeValid = false; end
if nargin < 11, failExcludeOffset = nan; end
if nargin < 12, failExcludeFreqHz = nan; end
diag = struct('metricGap', nan, 'clusterOnlyUsed', false, ...
    'selectedClusterSupport', nan, 'selectedMetricNorm', nan, ...
    'failExcludeApplied', false);
validMask = isfinite(candOffsets) & isfinite(candFreqHz) & isfinite(candMetrics) & ~usedMask;
if failExcludeValid && isfinite(failExcludeOffset) && isfinite(failExcludeFreqHz)
    sameFailBasin = abs(candOffsets - failExcludeOffset) <= settings.deepShadowRawFailExcludeTolChips & ...
        abs(candFreqHz - failExcludeFreqHz) <= settings.deepShadowRawFailExcludeTolFreqHz;
    diag.failExcludeApplied = any(validMask & sameFailBasin);
    validMask = validMask & ~sameFailBasin;
end
if ~any(validMask)
    candIdx = 0;
    return;
end

[bestScore, bestClusterIdx] = max(clusterScore);
tmp = clusterScore;
if ~isempty(bestClusterIdx) && bestClusterIdx >= 1
    tmp(bestClusterIdx) = -inf;
end
secondScore = max(tmp);
if ~isfinite(secondScore)
    secondScore = 0;
end

metricVals = candMetrics(validMask);
metricLog = log(max(metricVals, eps));
metricMax = max(metricLog);
metricSpan = max(abs(metricMax), 1.0);
metricNorm = zeros(size(candMetrics));
metricNorm(validMask) = 1 - max(0, metricMax - log(max(candMetrics(validMask), eps))) / metricSpan;
metricNorm(validMask) = min(1, max(0, metricNorm(validMask)));
metricSortedRaw = sort(metricVals, 'descend');
metricGap = 1.0;
if numel(metricSortedRaw) >= 2
    metricGap = log(max(metricSortedRaw(1), eps) / max(metricSortedRaw(2), eps));
end
diag.metricGap = metricGap;
metricWeight = settings.deepShadowRawCandMetricWeight;
if metricGap <= settings.deepShadowRawCandMetricGapSoft
    metricWeight = 0.5 * metricWeight;
end
metricWeight = min(1.0, max(0.15, metricWeight));
clusterWeight = max(0, settings.deepShadowRawCandClusterWeight);
dominantWeight = max(0, settings.deepShadowRawCandDominantWeight);

useClusterPreference = ~isempty(bestClusterIdx) && bestClusterIdx >= 1 && ...
    isfinite(clusterOffset(bestClusterIdx)) && ...
    isfinite(clusterFreqHz(bestClusterIdx)) && ...
    bestScore >= settings.deepShadowRawClusterReadyScore && ...
    clusterHits(bestClusterIdx) >= settings.deepShadowRawClusterReadyHits && ...
    (bestScore - secondScore) >= settings.deepShadowRawClusterReadyMargin;

score = -inf(size(candMetrics));
score(validMask) = metricWeight * metricNorm(validMask);
clusterAssocTolChips = settings.deepShadowRawClusterAssocTolChips;
clusterAssocTolFreqHz = settings.deepShadowRawClusterAssocTolFreqHz;
clusterSupportVec = -inf(size(candMetrics));
matchedBestCluster = false(size(candMetrics));
for kk = find(validMask)
    [matchIdx, matchDist, matchFreqErr] = localFindBestClusterMatch( ...
        candOffsets(kk), candFreqHz(kk), clusterOffset, clusterFreqHz, settings);
    if matchIdx > 0 && isfinite(clusterScore(matchIdx))
        closeChip = max(0, 1 - matchDist / max(clusterAssocTolChips, eps));
        closeFreq = max(0, 1 - matchFreqErr / max(clusterAssocTolFreqHz, eps));
        continuity = 0.5 * (closeChip + closeFreq);
        scoreReady = clusterScore(matchIdx) / max(settings.deepShadowRawClusterReadyScore, eps);
        hitReady = clusterHits(matchIdx) / max(settings.deepShadowRawClusterReadyHits, 1);
        clusterSupport = continuity * min(1.5, 0.5 * scoreReady + 0.5 * hitReady);
        clusterSupportVec(kk) = clusterSupport;
        matchedBestCluster(kk) = useClusterPreference && matchIdx == bestClusterIdx;
        score(kk) = score(kk) + clusterWeight * clusterSupport;
        if useClusterPreference && matchIdx == bestClusterIdx
            dominance = min(1.0, max(0.0, (bestScore - secondScore) / ...
                max(settings.deepShadowRawClusterReadyMargin, eps)));
            score(kk) = score(kk) + dominantWeight * continuity * dominance;
        end
    end
end
if useClusterPreference && metricGap <= settings.deepShadowRawCandClusterOnlyGapSoft
    clusterOnlyMask = validMask & matchedBestCluster & ...
        isfinite(clusterSupportVec) & ...
        (clusterSupportVec >= settings.deepShadowRawCandClusterOnlyMinSupport);
    if any(clusterOnlyMask)
        score(~clusterOnlyMask) = -inf;
        diag.clusterOnlyUsed = true;
    end
end
if useClusterPreference
    d = abs(candOffsets - clusterOffset(bestClusterIdx));
    df = abs(candFreqHz - clusterFreqHz(bestClusterIdx));
    closeMask = (d <= settings.deepShadowRawClusterAssocTolChips) & ...
        (df <= settings.deepShadowRawClusterAssocTolFreqHz);
    if any(validMask & closeMask)
        score(~closeMask) = -inf;
    end
end

[~, candIdx] = max(score);
if isempty(candIdx) || ~isfinite(score(candIdx))
    candIdx = 0;
else
    diag.selectedClusterSupport = clusterSupportVec(candIdx);
    diag.selectedMetricNorm = metricNorm(candIdx);
end
end

function candIdx = localPickShadowCandidateRelay(candOffsets, candFreqHz, candMetrics, usedMask, failExcludeValid, failExcludeOffset, failExcludeFreqHz, settings)
validMask = isfinite(candOffsets) & isfinite(candFreqHz) & isfinite(candMetrics) & ~usedMask;
if failExcludeValid && isfinite(failExcludeOffset) && isfinite(failExcludeFreqHz)
    d = abs(candOffsets - failExcludeOffset);
    df = abs(candFreqHz - failExcludeFreqHz);
    sameFailBasin = d <= settings.deepShadowRawFailExcludeTolChips & ...
        df <= settings.deepShadowRawFailExcludeTolFreqHz;
    validMask = validMask & ~sameFailBasin;
else
    d = zeros(size(candOffsets));
    df = zeros(size(candFreqHz));
end
if ~any(validMask)
    candIdx = 0;
    return;
end

score = candMetrics;
score(~validMask) = -inf;
if failExcludeValid && isfinite(failExcludeOffset) && isfinite(failExcludeFreqHz)
    % For relay, prefer candidates that are both strong and far away from the failed basin.
    relayScore = score;
    relayScore(validMask) = relayScore(validMask) + ...
        0.25 * d(validMask) / max(settings.deepShadowRawFailExcludeTolChips, eps) + ...
        0.25 * df(validMask) / max(settings.deepShadowRawFailExcludeTolFreqHz, eps);
    [~, candIdx] = max(relayScore);
else
    [~, candIdx] = max(score);
end
if isempty(candIdx) || ~isfinite(score(candIdx))
    candIdx = 0;
end
end

function [useClusterCenter, startOffsetChips, startOffsetSamples, startFreqHz, diag] = ...
    localBuildShadowStartPoint(trackMain, candOffsetChips, candOffsetSamples, candFreqHz, ...
    clusterOffsetRow, clusterFreqRow, clusterScoreRow, clusterHitsRow, settings)
useClusterCenter = false;
startOffsetChips = candOffsetChips;
startOffsetSamples = candOffsetSamples;
startFreqHz = candFreqHz;
diag = struct('clusterFirstUsed', false, 'blendAlpha', 1.0, ...
    'clusterScore', nan, 'clusterHits', 0, 'clusterMargin', nan);

[bestIdx, bestDistChips, bestFreqErrHz] = localFindBestClusterMatch( ...
    candOffsetChips, candFreqHz, clusterOffsetRow, clusterFreqRow, settings);
if bestIdx <= 0
    return;
end

tmp = clusterScoreRow;
tmp(bestIdx) = -inf;
secondScore = max(tmp);
if ~isfinite(secondScore)
    secondScore = 0;
end
diag.clusterScore = clusterScoreRow(bestIdx);
diag.clusterHits = clusterHitsRow(bestIdx);
diag.clusterMargin = clusterScoreRow(bestIdx) - secondScore;

scoreReady = clusterScoreRow(bestIdx) >= max(settings.deepShadowRawClusterReadyScore, settings.deepShadowRawExploreClusterScore);
hitReady = clusterHitsRow(bestIdx) >= max(settings.deepShadowRawClusterReadyHits, settings.deepShadowRawExploreClusterHits);
marginReady = (clusterScoreRow(bestIdx) - secondScore) >= min(settings.deepShadowRawClusterReadyMargin, settings.deepShadowRawExploreClusterMargin + 0.15);
closeReady = bestDistChips <= 0.75 * settings.deepShadowRawClusterAssocTolChips && ...
    bestFreqErrHz <= 0.75 * settings.deepShadowRawClusterAssocTolFreqHz;

if ~(scoreReady && hitReady && marginReady && closeReady)
    return;
end
if ~isfinite(clusterOffsetRow(bestIdx)) || ~isfinite(clusterFreqRow(bestIdx))
    return;
end

useClusterCenter = true;
clusterFirstReady = clusterScoreRow(bestIdx) >= settings.deepShadowRawClusterInitUseCenterScore && ...
    clusterHitsRow(bestIdx) >= settings.deepShadowRawClusterInitUseCenterHits && ...
    (clusterScoreRow(bestIdx) - secondScore) >= settings.deepShadowRawClusterInitUseCenterMargin;
if clusterFirstReady
    blendAlpha = min(1, max(0, settings.deepShadowRawClusterInitCenterBlendAlpha));
    diag.clusterFirstUsed = true;
else
    blendAlpha = min(1, max(0, settings.deepShadowRawClusterInitBlendAlpha));
end
diag.blendAlpha = blendAlpha;
startOffsetChips = blendAlpha * candOffsetChips + (1 - blendAlpha) * clusterOffsetRow(bestIdx);
startFreqHz = blendAlpha * candFreqHz + (1 - blendAlpha) * clusterFreqRow(bestIdx);
codePhaseStepShadow = trackMain.codeFreq / settings.samplingFreq;
startOffsetSamples = startOffsetChips / max(codePhaseStepShadow, eps);
end

function ready = localHasDirectClusterInit(clusterOffsetRow, clusterFreqRow, clusterScoreRow, clusterHitsRow, settings)
[bestScore, bestIdx] = max(clusterScoreRow);
tmp = clusterScoreRow;
if ~isempty(bestIdx) && bestIdx >= 1
    tmp(bestIdx) = -inf;
end
secondScore = max(tmp);
if ~isfinite(secondScore)
    secondScore = 0;
end

ready = ~isempty(bestIdx) && bestIdx >= 1 && ...
    isfinite(clusterOffsetRow(bestIdx)) && isfinite(clusterFreqRow(bestIdx)) && ...
    bestScore >= settings.deepShadowRawClusterDirectInitScore && ...
    clusterHitsRow(bestIdx) >= settings.deepShadowRawClusterDirectInitHits && ...
    (bestScore - secondScore) >= settings.deepShadowRawClusterDirectInitMargin;
end

function [ready, startOffsetChips, startOffsetSamples, startFreqHz, bestIdx] = ...
    localGetClusterDirectInitStart(trackMain, clusterOffsetRow, clusterFreqRow, clusterScoreRow, clusterHitsRow, settings)
ready = false;
startOffsetChips = nan;
startOffsetSamples = nan;
startFreqHz = nan;
bestIdx = 0;

[bestScore, bestIdx] = max(clusterScoreRow);
tmp = clusterScoreRow;
if ~isempty(bestIdx) && bestIdx >= 1
    tmp(bestIdx) = -inf;
end
secondScore = max(tmp);
if ~isfinite(secondScore)
    secondScore = 0;
end

if isempty(bestIdx) || bestIdx < 1 || ...
        ~isfinite(clusterOffsetRow(bestIdx)) || ~isfinite(clusterFreqRow(bestIdx))
    bestIdx = 0;
    return;
end
if bestScore < settings.deepShadowRawClusterDirectInitScore || ...
        clusterHitsRow(bestIdx) < settings.deepShadowRawClusterDirectInitHits || ...
        (bestScore - secondScore) < settings.deepShadowRawClusterDirectInitMargin
    return;
end

ready = true;
startOffsetChips = clusterOffsetRow(bestIdx);
startFreqHz = clusterFreqRow(bestIdx);
codePhaseStepShadow = trackMain.codeFreq / settings.samplingFreq;
startOffsetSamples = startOffsetChips / max(codePhaseStepShadow, eps);
end

function initOffset = localSelectInitOffsetForMeta(candIdx, startedFromDirectCluster, candOffsets, clusterOffsetRow, bestClusterIdx)
initOffset = nan;
if candIdx > 0 && candIdx <= numel(candOffsets)
    initOffset = candOffsets(candIdx);
elseif startedFromDirectCluster && bestClusterIdx > 0 && bestClusterIdx <= numel(clusterOffsetRow)
    initOffset = clusterOffsetRow(bestClusterIdx);
end
end

function initFreq = localSelectInitFreqForMeta(candIdx, startedFromDirectCluster, candFreqHz, clusterFreqRow, bestClusterIdx)
initFreq = nan;
if candIdx > 0 && candIdx <= numel(candFreqHz)
    initFreq = candFreqHz(candIdx);
elseif startedFromDirectCluster && bestClusterIdx > 0 && bestClusterIdx <= numel(clusterFreqRow)
    initFreq = clusterFreqRow(bestClusterIdx);
end
end

function trackShadow = localSetShadowPendingClusterMeta(trackShadow, useClusterCenter, candOffsetChips, candFreqHz, ...
    clusterOffsetRow, clusterFreqRow, clusterScoreRow, clusterHitsRow, settings)
trackShadow.deepShadowInitFromClusterCenter = useClusterCenter;
trackShadow.deepShadowInitClusterScore = 0;
trackShadow.deepShadowInitClusterHits = 0;
trackShadow.deepShadowInitClusterMargin = 0;

[bestIdx, ~, ~] = localFindBestClusterMatch(candOffsetChips, candFreqHz, clusterOffsetRow, clusterFreqRow, settings);
if bestIdx <= 0
    return;
end

tmp = clusterScoreRow;
tmp(bestIdx) = -inf;
secondScore = max(tmp);
if ~isfinite(secondScore)
    secondScore = 0;
end

trackShadow.deepShadowInitClusterScore = clusterScoreRow(bestIdx);
trackShadow.deepShadowInitClusterHits = clusterHitsRow(bestIdx);
trackShadow.deepShadowInitClusterMargin = clusterScoreRow(bestIdx) - secondScore;
end

function [bestIdx, bestDistChips, bestFreqErrHz] = localFindBestClusterMatch(candOffset, candFreqHz, clusterOffset, clusterFreqHz, settings)
bestIdx = 0;
bestDist = inf;
bestDistChips = nan;
bestFreqErrHz = nan;
if ~isfinite(candOffset) || ~isfinite(candFreqHz)
    return;
end
for jj = 1:numel(clusterOffset)
    if ~isfinite(clusterOffset(jj)) || ~isfinite(clusterFreqHz(jj))
        continue;
    end
    d = abs(candOffset - clusterOffset(jj));
    df = abs(candFreqHz - clusterFreqHz(jj));
    joint = d / max(settings.deepShadowRawClusterAssocTolChips, eps) + ...
        df / max(settings.deepShadowRawClusterAssocTolFreqHz, eps);
    if joint < bestDist
        bestDist = joint;
        bestIdx = jj;
        bestDistChips = d;
        bestFreqErrHz = df;
    end
end
end



function [keepLocal, residualLocal, sourceId, signChoice] = localShadowBranchDopplerGate(passShadowIdx, currMeasNr, navResults, navSolutShadowPass, priorPos, priorVel, settings)
numPass = numel(passShadowIdx);
keepLocal = true(numPass, 1);
residualLocal = nan(numPass, 1);
sourceId = 0;
signChoice = -1;
if numPass < settings.deepShadowBranchDopplerGateMinKeep || currMeasNr <= 1 || ...
        ~all(isfinite(priorPos)) || ~all(isfinite(priorVel))
    return;
end
fields = {'shadowRawTrackCarrAidFreqHz', 'shadowRawTrackCarrNcoHz', ...
    'shadowRawTrackCarrFreqHz', 'shadowRawTrackCarrCmdHz'};
sourceScores = inf(1, numel(fields));
sourceSigns = -ones(1, numel(fields));
sourceResiduals = nan(numPass, numel(fields));
for ff = 1:numel(fields)
    fn = fields{ff};
    if ~isfield(navResults, fn)
        continue;
    end
    [sgn, score, det] = localScoreDopplerSource(passShadowIdx, currMeasNr, navResults, ...
        navSolutShadowPass, priorPos, priorVel, settings, fn);
    sourceScores(ff) = score;
    sourceSigns(ff) = sgn;
    sourceResiduals(:,ff) = det(:);
end
[bestScore, bestIdx] = min(sourceScores);
if ~isfinite(bestScore)
    return;
end
sourceId = bestIdx;
signChoice = sourceSigns(bestIdx);
residualLocal = sourceResiduals(:,bestIdx);
finiteResidual = isfinite(residualLocal);
if sum(finiteResidual) < settings.deepShadowBranchDopplerGateMinKeep
    return;
end
keepLocal = finiteResidual & abs(residualLocal) <= settings.deepShadowBranchDopplerGateMps;
if sum(keepLocal) < settings.deepShadowBranchDopplerGateMinKeep
    [~, ord] = sort(abs(residualLocal), 'ascend', 'MissingPlacement', 'last');
    keepLocal(:) = false;
    keepLocal(ord(1:min(settings.deepShadowBranchDopplerGateMinKeep, sum(finiteResidual)))) = true;
end
end

function [signChoice, score, detBest] = localScoreDopplerSource(passShadowIdx, currMeasNr, navResults, navSolutShadowPass, priorPos, priorVel, settings, fieldName)
signs = [-1, 1];
scores = inf(1, 2);
dets = nan(numel(passShadowIdx), 2);
lambdaL1 = settings.c / 1575.42e6;
for ssn = 1:2
    sgn = signs(ssn);
    rawResidual = nan(numel(passShadowIdx), 1);
    for ll = 1:numel(passShadowIdx)
        globalIdx = passShadowIdx(ll);
        freqHz = navResults.(fieldName)(globalIdx, currMeasNr);
        if ~isfinite(freqHz)
            continue;
        end
        satPos = navSolutShadowPass.satPositions(:, ll);
        satVel = navSolutShadowPass.satVelocity(:, ll);
        if ~all(isfinite([satPos; satVel]))
            continue;
        end
        rho = norm(satPos(:) - priorPos(:));
        if rho <= 0 || ~isfinite(rho)
            continue;
        end
        los = (satPos(:) - priorPos(:)) / rho;
        satClkRateMps = 0;
        if isfield(navSolutShadowPass, 'satClkDrift') && numel(navSolutShadowPass.satClkDrift) >= ll && ...
                isfinite(navSolutShadowPass.satClkDrift(ll))
            satClkRateMps = settings.c * navSolutShadowPass.satClkDrift(ll);
        end
        obsRate = sgn * lambdaL1 * freqHz;
        predNoClock = los.' * (satVel(:) - priorVel(:)) - satClkRateMps;
        rawResidual(ll) = obsRate - predNoClock;
    end
    finiteResidual = isfinite(rawResidual);
    if sum(finiteResidual) >= settings.deepShadowBranchDopplerGateMinKeep
        det = rawResidual - median(rawResidual(finiteResidual), 'omitnan');
        dets(:,ssn) = det(:);
        scores(ssn) = median(abs(det(finiteResidual)), 'omitnan');
    end
end
[score, best] = min(scores);
signChoice = signs(best);
detBest = dets(:,best);
end

function branchMaskOut = localApplyDopplerGateToBranchMask(branchMaskIn, dopplerKeepLocal, dopplerResidualLocal, minSat)
branchMaskOut = branchMaskIn(:) & dopplerKeepLocal(:);
if sum(branchMaskOut) < minSat
    cand = find(branchMaskIn(:) & isfinite(dopplerResidualLocal(:)));
    if numel(cand) >= minSat
        [~, ord] = sort(abs(dopplerResidualLocal(cand)), 'ascend');
        branchMaskOut(:) = false;
        branchMaskOut(cand(ord(1:minSat))) = true;
    else
        branchMaskOut = branchMaskIn(:);
    end
end
end

function chipM = localChipMeters(settings)
chipM = settings.c / settings.codeFreqBasis;
end




function priors = localBuildTrustedBranchPriors(currentPos, trustedValid, trustedPos, trustedVel, trustedTimeSec, epochSec, trustedSource, settings)
priors = struct('pos', {}, 'source', {});
allowTrustedAnchor = trustedValid;
if localIsDs5Scenario(settings)
    % DS5 online recovery must not inherit truth/trj anchors.
    allowTrustedAnchor = trustedValid && trustedSource ~= 2;
end
if allowTrustedAnchor
    priors(end+1).pos = trustedPos(:); %#ok<AGROW>
    priors(end).source = 10 + trustedSource;  % hold
    if settings.deepShadowTrustedAnchorUseVelocity && all(isfinite(trustedVel)) && isfinite(trustedTimeSec)
        priors(end+1).pos = trustedPos(:) + (epochSec - trustedTimeSec) * trustedVel(:); %#ok<AGROW>
        priors(end).source = 20 + trustedSource;  % velocity propagation
    end
end
if isempty(priors) || settings.deepShadowTrustedAnchorAllowCurrentFallback
    priors(end+1).pos = currentPos(:); %#ok<AGROW>
    priors(end).source = 1;
end
end

function [bestRawP, bestAmbig, bestPriorDet, bestPos, bestRes, bestPdop, bestSource, bestPriorPos, bestUseMask, bestScore, bestDetrendedP95, bestDetrendedPenalty] = ...
    localSelectBestBranchPrior(rawP, satClkCorr, satPos, priors, settings, dopplerResidualMps, prevAmbigChips, detrendedResidual, currMeasNr)
rawP = rawP(:)';
satClkCorr = satClkCorr(:)';
nSat = numel(rawP);
if nargin < 6 || isempty(dopplerResidualMps)
    dopplerResidualMps = nan(1, nSat);
else
    dopplerResidualMps = dopplerResidualMps(:)';
end
if nargin < 7 || isempty(prevAmbigChips)
    prevAmbigChips = nan(1, nSat);
else
    prevAmbigChips = prevAmbigChips(:)';
end
bestScore = inf;
bestRawP = nan(size(rawP));
bestAmbig = nan(size(rawP));
bestPriorDet = nan(size(rawP));
bestPos = nan(4,1);
bestRes = nan(size(rawP));
bestPdop = nan;
bestSource = 0;
bestPriorPos = nan(3,1);
bestUseMask = false(size(rawP));
bestDetrendedP95 = nan;
bestDetrendedPenalty = nan;
if nargin < 8 || isempty(detrendedResidual)
    detrendedResidual = nan(1, nSat);
else
    detrendedResidual = detrendedResidual(:)';
end
subsetMasks = localBuildBranchSubsetMasks(nSat, settings);
for pp = 1:numel(priors)
    priorPos = priors(pp).pos(:);
    sourcePenalty = 0;
    if priors(pp).source == 1
        sourcePenalty = 500; % Current navigation branch is a fallback, not trusted.
    end
    for cc = 1:size(subsetMasks, 1)
        useMask = logical(subsetMasks(cc,:));
        useN = sum(useMask);
        if useN < 4
            continue;
        end
        [rawPLiftSub, ambigSub, detSub] = localLiftToPositionBranchRawP(...
            rawP(useMask), satClkCorr(useMask), satPos(:,useMask), priorPos, settings, prevAmbigChips(useMask));
        obs = rawPLiftSub(:)' + settings.c * satClkCorr(useMask);
        [pos, res, pdop] = localRangeOnlySpp(satPos(:,useMask), obs);
        if ~all(isfinite(pos(1:4))) || ~isfinite(pdop)
            continue;
        end
        priorRms = sqrt(mean(detSub.^2, 'omitnan'));
        postRms = sqrt(mean(res.^2, 'omitnan'));
        jumpM = norm(pos(1:3) - priorPos(:));
        dropPenalty = settings.deepShadowBranchRobustSubsetDropPenaltyM * max(0, nSat - useN);
        fourPenalty = settings.deepShadowBranchRobustSubsetFourSatPenaltyM * double(useN <= 4);
        dopplerPenalty = 0;
        dopUse = abs(dopplerResidualMps(useMask));
        if any(isfinite(dopUse))
            dopplerPenalty = settings.deepShadowBranchRobustDopplerWeight * median(dopUse, 'omitnan');
        end
        ambigPenalty = 0;
        if settings.deepShadowBranchAmbigContinuityEnable
            prevUse = prevAmbigChips(useMask);
            ambigDiff = abs(ambigSub - prevUse);
            ambigDiff = ambigDiff(isfinite(ambigDiff));
            if ~isempty(ambigDiff)
                ambigExcess = max(0, ambigDiff - settings.deepShadowBranchAmbigContinuityTolChips);
                ambigPenalty = settings.deepShadowBranchAmbigContinuityPenaltyM * median(ambigExcess, 'omitnan') + ...
                    settings.deepShadowBranchAmbigContinuityJumpPenaltyM * mean(ambigDiff > settings.deepShadowBranchAmbigContinuityStepMaxChips, 'omitnan');
            end
        end
        detrendedPenalty = 0;
        detrendedP95 = nan;
        if localUseDs5CommonDrag(settings, currMeasNr)
            detUse = abs(detrendedResidual(useMask));
            detUse = detUse(isfinite(detUse));
            if ~isempty(detUse)
                detrendedP95 = prctile(detUse, 95);
                detrendedPenalty = settings.deepShadowBranchDs5DetrendedScoreWeight * median(detUse, 'omitnan') + ...
                    settings.deepShadowBranchDs5DetrendedP95Weight * detrendedP95;
            end
        end
        score = priorRms + 2 * postRms + 0.02 * jumpM + 20 * max(0, pdop - 8) + ...
            sourcePenalty + dropPenalty + fourPenalty + dopplerPenalty + ambigPenalty + detrendedPenalty;
        if score < bestScore
            bestScore = score;
            bestRawP(:) = nan;
            bestAmbig(:) = nan;
            bestPriorDet(:) = nan;
            bestRes(:) = nan;
            bestRawP(useMask) = rawPLiftSub;
            bestAmbig(useMask) = ambigSub;
            bestPriorDet(useMask) = detSub;
            bestRes(useMask) = res;
            bestPos = pos;
            bestPdop = pdop;
            bestSource = priors(pp).source;
            bestPriorPos = priorPos;
            bestUseMask = useMask;
            bestDetrendedP95 = detrendedP95;
            bestDetrendedPenalty = detrendedPenalty;
        end
    end
end
end

function [bestRawP, bestAmbig, bestPriorDet, bestPos, bestRes, bestPdop, bestSource, bestPriorPos, bestUseMask, bestScore, bestDetrendedP95, bestDetrendedPenalty, bestAbsModelP95] = ...
    localSelectBestBranchPriorDs5Absolute(rawPRaw, deltaUse, satClkCorr, satPos, deltaRefPos, priors, settings, dopplerResidualMps, prevAmbigChips, detrendedResidual, currMeasNr)
rawPRaw = rawPRaw(:)';
deltaUse = deltaUse(:)';
satClkCorr = satClkCorr(:)';
nSat = numel(rawPRaw);
if nargin < 8 || isempty(dopplerResidualMps)
    dopplerResidualMps = nan(1, nSat);
else
    dopplerResidualMps = dopplerResidualMps(:)';
end
if nargin < 9 || isempty(prevAmbigChips)
    prevAmbigChips = nan(1, nSat);
else
    prevAmbigChips = prevAmbigChips(:)';
end
if nargin < 10 || isempty(detrendedResidual)
    detrendedResidual = nan(1, nSat);
else
    detrendedResidual = detrendedResidual(:)';
end
bestScore = inf;
bestRawP = nan(size(rawPRaw));
bestAmbig = nan(size(rawPRaw));
bestPriorDet = nan(size(rawPRaw));
bestPos = nan(4,1);
bestRes = nan(size(rawPRaw));
bestPdop = nan;
bestSource = 0;
bestPriorPos = nan(3,1);
bestUseMask = false(size(rawPRaw));
bestDetrendedP95 = nan;
bestDetrendedPenalty = nan;
bestAbsModelP95 = nan;
subsetMasks = localBuildBranchSubsetMasks(nSat, settings);
rhoRefAll = sqrt(sum((satPos - deltaRefPos(:)).^2, 1));
obsAll = rhoRefAll(:)' + deltaUse;
chipM = localChipMeters(settings);
for pp = 1:numel(priors)
    priorPos = priors(pp).pos(:);
    sourcePenalty = 0;
    if priors(pp).source == 1
        sourcePenalty = 500;
    end
    for cc = 1:size(subsetMasks, 1)
        useMask = logical(subsetMasks(cc,:));
        useN = sum(useMask);
        if useN < 4
            continue;
        end
        obs = obsAll(useMask);
        if ~all(isfinite(obs))
            continue;
        end
        [priorClock, detSub] = localResidualAgainstPos(obs, satPos(:,useMask), priorPos);
        if ~isfinite(priorClock)
            continue;
        end
        [pos, res, pdop, corrNorm] = localSolveDs5BranchLocalCorrection(satPos(:,useMask), obs, deltaRefPos, settings);
        if ~all(isfinite(pos(1:4))) || ~isfinite(pdop)
            continue;
        end
        if isfinite(localGetSettingValue(settings, 'deepShadowDs5BranchLocalCorrectionMaxM', 1200.0)) && ...
                corrNorm > localGetSettingValue(settings, 'deepShadowDs5BranchLocalCorrectionMaxM', 1200.0)
            continue;
        end
        priorRms = sqrt(mean(detSub.^2, 'omitnan'));
        postRms = sqrt(mean(res.^2, 'omitnan'));
        jumpM = corrNorm;
        dropPenalty = settings.deepShadowBranchRobustSubsetDropPenaltyM * max(0, nSat - useN);
        fourPenalty = settings.deepShadowBranchRobustSubsetFourSatPenaltyM * double(useN <= 4);
        dopplerPenalty = 0;
        dopUse = abs(dopplerResidualMps(useMask));
        if any(isfinite(dopUse))
            dopplerPenalty = settings.deepShadowBranchRobustDopplerWeight * median(dopUse, 'omitnan');
        end
        ambigSub = nan(1, useN);
        ambigPenalty = 0;
        if isfinite(chipM) && chipM > 0
            rawObs = rawPRaw(useMask) + settings.c * satClkCorr(useMask);
            ambigSub = round((rawObs - obs) / chipM);
            ambigSub = max(-settings.deepShadowBranchMaxChips, min(settings.deepShadowBranchMaxChips, ambigSub));
            if settings.deepShadowBranchAmbigContinuityEnable
                prevUse = prevAmbigChips(useMask);
                ambigDiff = abs(ambigSub - prevUse);
                ambigDiff = ambigDiff(isfinite(ambigDiff));
                if ~isempty(ambigDiff)
                    ambigExcess = max(0, ambigDiff - settings.deepShadowBranchAmbigContinuityTolChips);
                    ambigPenalty = 0.25 * (settings.deepShadowBranchAmbigContinuityPenaltyM * median(ambigExcess, 'omitnan') + ...
                        settings.deepShadowBranchAmbigContinuityJumpPenaltyM * mean(ambigDiff > settings.deepShadowBranchAmbigContinuityStepMaxChips, 'omitnan'));
                end
            end
        end
        detrendedPenalty = 0;
        detrendedP95 = nan;
        detUse = abs(detrendedResidual(useMask));
        detUse = detUse(isfinite(detUse));
        if ~isempty(detUse)
            detrendedP95 = prctile(detUse, 95);
            detrendedPenalty = settings.deepShadowBranchDs5DetrendedScoreWeight * median(detUse, 'omitnan') + ...
                settings.deepShadowBranchDs5DetrendedP95Weight * detrendedP95;
        end
        absModelP95 = prctile(abs(res(isfinite(res))), 95);
        corrPenalty = localGetSettingValue(settings, 'deepShadowDs5BranchLocalCorrPenaltyWeight', 0.10) * corrNorm;
        score = 2 * postRms + corrPenalty + 20 * max(0, pdop - 8) + ...
            sourcePenalty + dropPenalty + fourPenalty + dopplerPenalty + ambigPenalty + detrendedPenalty + 0.05 * min(priorRms, 2000.0);
        if score < bestScore
            bestScore = score;
            bestRawP(:) = nan;
            bestAmbig(:) = nan;
            bestPriorDet(:) = nan;
            bestRes(:) = nan;
            bestRawP(useMask) = obs - settings.c * satClkCorr(useMask);
            bestAmbig(useMask) = ambigSub;
            bestPriorDet(useMask) = detSub;
            bestRes(useMask) = res;
            bestPos = pos;
            bestPdop = pdop;
            bestSource = priors(pp).source;
            bestPriorPos = priorPos;
            bestUseMask = useMask;
            bestDetrendedP95 = detrendedP95;
            bestDetrendedPenalty = detrendedPenalty;
            bestAbsModelP95 = absModelP95;
        end
    end
end
end

function [pos, res, pdop, corrNorm] = localSolveDs5BranchLocalCorrection(satPos, obs, refPos, settings)
pos = nan(4,1);
res = nan(numel(obs), 1);
pdop = nan;
corrNorm = nan;
if ~all(isfinite(refPos))
    return;
end
obs = obs(:);
refPos = refPos(:);
nSat = numel(obs);
if size(satPos,2) ~= nSat || nSat < 4
    return;
end
posEcef = refPos;
maxIter = max(1, round(localGetSettingValue(settings, 'deepShadowDs5BranchLocalIterMax', 3)));
stepMaxM = localGetSettingValue(settings, 'deepShadowDs5BranchLocalStepMaxM', 250.0);
corrMaxM = localGetSettingValue(settings, 'deepShadowDs5BranchLocalCorrectionMaxM', 1200.0);
rho = sqrt(sum((satPos - posEcef).^2, 1)).';
clockBias = median(obs - rho, 'omitnan');
if ~isfinite(clockBias)
    return;
end
for it = 1:maxIter
    dpos = satPos - posEcef;
    rho = sqrt(sum(dpos.^2, 1)).';
    if ~all(isfinite(rho)) || any(rho <= 1)
        return;
    end
    H = [-(dpos(1,:).')./rho  -(dpos(2,:).')./rho  -(dpos(3,:).')./rho  ones(nSat,1)];
    y = obs - rho - clockBias;
    if any(~isfinite(H(:))) || any(~isfinite(y))
        return;
    end
    step = H \ y;
    if ~all(isfinite(step))
        step = pinv(H) * y;
    end
    if ~all(isfinite(step))
        return;
    end
    stepPos = step(1:3);
    stepNorm = norm(stepPos);
    if isfinite(stepMaxM) && stepMaxM > 0 && stepNorm > stepMaxM
        stepPos = stepPos * (stepMaxM / stepNorm);
        stepNorm = stepMaxM;
    end
    newPos = posEcef + stepPos;
    totalCorr = newPos - refPos;
    totalNorm = norm(totalCorr);
    if isfinite(corrMaxM) && corrMaxM > 0 && totalNorm > corrMaxM
        newPos = refPos + totalCorr * (corrMaxM / totalNorm);
        totalCorr = newPos - refPos;
        totalNorm = corrMaxM;
    end
    posEcef = newPos;
    clockBias = clockBias + step(4);
    if stepNorm < 1.0
        break;
    end
end
dpos = satPos - posEcef;
rho = sqrt(sum(dpos.^2, 1)).';
H = [-(dpos(1,:).')./rho  -(dpos(2,:).')./rho  -(dpos(3,:).')./rho  ones(nSat,1)];
res = obs - rho - clockBias;
Q = pinv(H' * H);
if all(isfinite(diag(Q)))
    pdop = sqrt(max(0, trace(Q(1:3,1:3))));
end
pos = [posEcef; clockBias];
corrNorm = norm(posEcef - refPos);
end

function [keepMask, info] = localBuildDs5BranchObsConsistencyMask(deltaUse, satPos, refPos, settings)
keepMask = true(size(deltaUse));
info = struct('keepNum', numel(deltaUse), 'rejectNum', 0, 'gateM', nan, 'refDetP95M', nan);
if localGetSettingValue(settings, 'deepShadowDs5BranchObsGateEnable', 0) == 0
    return;
end
if isempty(deltaUse) || ~all(isfinite(refPos))
    keepMask(:) = false;
    info.keepNum = 0;
    info.rejectNum = numel(deltaUse);
    return;
end
rhoRef = sqrt(sum((satPos - refPos(:)).^2, 1));
obs = rhoRef(:)' + deltaUse(:)';
[~, det] = localResidualAgainstPos(obs, satPos, refPos);
detAbs = abs(det(:)');
detFinite = detAbs(isfinite(detAbs));
minSat = max(1, round(localGetSettingValue(settings, 'deepShadowDs5BranchObsGateMinSat', 4)));
if isempty(detFinite)
    keepMask(:) = false;
    info.keepNum = 0;
    info.rejectNum = numel(deltaUse);
    return;
end
medAbs = median(detFinite, 'omitnan');
madAbs = median(abs(detFinite - medAbs), 'omitnan');
gateM = medAbs + localGetSettingValue(settings, 'deepShadowDs5BranchObsGateMadScale', 3.0) * 1.4826 * madAbs;
gateM = max(localGetSettingValue(settings, 'deepShadowDs5BranchObsGateFloorM', 120.0), gateM);
ceilM = localGetSettingValue(settings, 'deepShadowDs5BranchObsGateCeilM', 1500.0);
if isfinite(ceilM)
    gateM = min(ceilM, gateM);
end
keepMask = isfinite(detAbs) & (detAbs <= gateM);
if sum(keepMask) < minSat
    detRank = detAbs;
    detRank(~isfinite(detRank)) = inf;
    [~, ord] = sort(detRank, 'ascend');
    keepMask(:) = false;
    keepMask(ord(1:min(minSat, numel(ord)))) = true;
end
info.keepNum = sum(keepMask);
info.rejectNum = numel(keepMask) - info.keepNum;
info.gateM = gateM;
info.refDetP95M = prctile(detFinite, 95);
end

function [clockBias, det] = localResidualAgainstPos(obs, satPos, refPos)
rho = sqrt(sum((satPos - refPos(:)).^2, 1));
res = obs(:)' - rho;
clockBias = median(res, 'omitnan');
det = res - clockBias;
end

function subsetMasks = localBuildBranchSubsetMasks(nSat, settings)
subsetMasks = true(1, nSat);
if nSat <= 0 || ~settings.deepShadowBranchRobustSubsetEnable
    return;
end
minSat = max(4, min(nSat, settings.deepShadowBranchRobustSubsetMinSat));
maxSat = min(nSat, max(minSat, settings.deepShadowBranchRobustSubsetMaxSat));
maxCand = max(1, settings.deepShadowBranchRobustSubsetMaxCand);
for kk = minSat:maxSat
    if kk == nSat
        continue;
    end
    comb = nchoosek(1:nSat, kk);
    for rr = 1:size(comb, 1)
        m = false(1, nSat);
        m(comb(rr,:)) = true;
        subsetMasks(end+1,:) = m; %#ok<AGROW>
        if size(subsetMasks, 1) >= maxCand
            return;
        end
    end
end
end

function [posEcef, velEcef, ok] = localTrustedAnchorFromTrj(trj, tAbs)
posEcef = nan(3,1);
velEcef = nan(3,1);
ok = false;
if ~isfield(trj, 'avp') || size(trj.avp, 2) < 10
    return;
end
[~, idx] = min(abs(trj.avp(:,end) - tAbs));
if isempty(idx) || idx < 1
    return;
end
blh = trj.avp(idx, 7:9).';
[posEcef, ~] = blh2xyz(blh);
velNed = trj.avp(idx, 4:6).';
velEcef = localNedVelToEcef(blh, velNed);
ok = all(isfinite(posEcef)) && all(isfinite(velEcef));
end

function velEcef = localNedVelToEcef(blh, velNed)
Cenu2xyz = [-sin(blh(2))                  cos(blh(2))   0
            -sin(blh(1))*cos(blh(2)) -sin(blh(1))*sin(blh(2))  cos(blh(1))
             cos(blh(1))*cos(blh(2))  cos(blh(1))*sin(blh(2))  sin(blh(1))];
velEcef = Cenu2xyz' * velNed(:);
end


function velNed = localEcefVelToNed(blh, velEcef)
Cenu2xyz = [-sin(blh(2))                  cos(blh(2))   0
            -sin(blh(1))*cos(blh(2)) -sin(blh(1))*sin(blh(2))  cos(blh(1))
             cos(blh(1))*cos(blh(2))  cos(blh(1))*sin(blh(2))  sin(blh(1))];
velNed = Cenu2xyz * velEcef(:);
end

function [rawPLift, ambigChips, det, clockBias] = localLiftToPositionBranchRawP(rawP, satClkCorr, satPos, priorPos, settings, prevAmbigChips)
chipM = localChipMeters(settings);
obsRaw = rawP(:)' + settings.c * satClkCorr(:)';
rho = sqrt(sum((satPos - priorPos(:)).^2, 1));
r = obsRaw - rho;
clockBias = median(r, 'omitnan');
detRaw = r - clockBias;
ambigChips = round(detRaw / chipM);
% Continuity is handled as a branch-subset score penalty, not as a hard
% clamp here. Reacquired real-signal tracks sometimes need large chip lifts;
% hard clamping them reduced branch availability too aggressively.
ambigChips = max(-settings.deepShadowBranchMaxChips, min(settings.deepShadowBranchMaxChips, ambigChips));
rawPLift = rawP(:)' - ambigChips * chipM;
r2 = rawPLift + settings.c * satClkCorr(:)' - rho;
clockBias = median(r2, 'omitnan');
det = r2 - clockBias;
end

function [pos, res, pdop] = localRangeOnlySpp(satPos, obs, initPos)
if nargin < 3 || isempty(initPos)
    pos = [0; 0; 0; median(obs, 'omitnan') - 2.2e7];
else
    initPos = initPos(:);
    if numel(initPos) >= 4 && all(isfinite(initPos(1:4)))
        pos = initPos(1:4);
    elseif numel(initPos) >= 3 && all(isfinite(initPos(1:3)))
        rho0 = sqrt(sum((satPos - initPos(1:3)).^2, 1));
        clock0 = median(obs(:)' - rho0, 'omitnan');
        pos = [initPos(1:3); clock0];
    else
        pos = [0; 0; 0; median(obs, 'omitnan') - 2.2e7];
    end
end
res = nan(size(obs));
pdop = nan;
if numel(obs) < 4
    pos(:) = nan;
    return;
end
for iter = 1:12
    rho = sqrt(sum((satPos - pos(1:3)).^2, 1));
    H = [-(satPos(1,:)' - pos(1))./rho(:), ...
         -(satPos(2,:)' - pos(2))./rho(:), ...
         -(satPos(3,:)' - pos(3))./rho(:), ...
         ones(numel(obs), 1)];
    y = obs(:) - rho(:) - pos(4);
    if rank(H) < 4
        pos(:) = nan;
        return;
    end
    dx = H \ y;
    pos = pos + dx;
    if norm(dx(1:3)) < 1e-3 && abs(dx(4)) < 1e-3
        break;
    end
end
rho = sqrt(sum((satPos - pos(1:3)).^2, 1));
res = obs(:)' - rho - pos(4);
Q = pinv(H' * H);
pdop = sqrt(trace(Q(1:3,1:3)));
end

function [rawPLift, ambigChips, deltaLift, deltaLiftAbs, segmentId, quality, repairChips] = localLiftRawPassToAnchor(rawP, satClkCorr, satPos, anchorPos, settings, prevAmbigChips, lastDeltaLift, prevSegmentId, prnList)
chipM = localChipMeters(settings);
rawP = rawP(:);
satClkCorr = satClkCorr(:);
n = numel(rawP);
rawPLift = nan(n,1);
ambigChips = nan(n,1);
deltaLift = nan(n,1);
deltaLiftAbs = nan(n,1);
segmentId = zeros(n,1);
quality = zeros(n,1);      % 0 invalid/skip, 1 new or repaired segment seed, 2 time-continuous segment
repairChips = nan(n,1);
if nargin < 6 || isempty(prevAmbigChips), prevAmbigChips = nan(n,1); end
if nargin < 7 || isempty(lastDeltaLift), lastDeltaLift = nan(n,1); end
if nargin < 8 || isempty(prevSegmentId), prevSegmentId = zeros(n,1); end
if nargin < 9 || isempty(prnList), prnList = (1:n).'; end
prevAmbigChips = prevAmbigChips(:);
lastDeltaLift = lastDeltaLift(:);
prevSegmentId = prevSegmentId(:);
prnList = prnList(:);
if numel(prevAmbigChips) < n, prevAmbigChips(end+1:n,1) = nan; end
if numel(lastDeltaLift) < n, lastDeltaLift(end+1:n,1) = nan; end
if numel(prevSegmentId) < n, prevSegmentId(end+1:n,1) = 0; end
if numel(prnList) < n, prnList(end+1:n,1) = (numel(prnList)+1:n).'; end
if ~isfinite(chipM) || chipM <= 0 || ~all(isfinite(anchorPos(:)))
    return;
end
rho = sqrt(sum((satPos - anchorPos(:)).^2, 1));
deltaRaw = rawP(:) + settings.c * satClkCorr(:) - rho(:);
good = isfinite(deltaRaw) & isfinite(rawP) & isfinite(satClkCorr) & isfinite(rho(:));
if sum(good) < 3
    return;
end
commonClock = median(deltaRaw(good), 'omitnan');
detRaw = deltaRaw - commonClock;
rawAmbig = round(detRaw / chipM);
rawAmbig(~good) = nan;
if isfield(settings, 'deepShadowRawAmbiguityMaxChips')
    rawAmbig(good) = max(-settings.deepShadowRawAmbiguityMaxChips, min(settings.deepShadowRawAmbiguityMaxChips, rawAmbig(good)));
end

halfChipM = 0.50 * chipM;
if isfield(settings, 'deepShadowClosedLiftHalfChipM'), halfChipM = settings.deepShadowClosedLiftHalfChipM; end
seedGateM = 1.50 * chipM;
if isfield(settings, 'deepShadowClosedLiftSeedGateM'), seedGateM = settings.deepShadowClosedLiftSeedGateM; end
qualityStepGateM = 1.50 * chipM;
if isfield(settings, 'deepShadowClosedLiftQualityStepGateM'), qualityStepGateM = settings.deepShadowClosedLiftQualityStepGateM; end
repairMaxChips = 8;
if isfield(settings, 'deepShadowClosedLiftRepairMaxChips'), repairMaxChips = settings.deepShadowClosedLiftRepairMaxChips; end
strictPrn = [27; 3];
if isfield(settings, 'deepShadowClosedLiftStrictPrnList') && ~isempty(settings.deepShadowClosedLiftStrictPrnList)
    strictPrn = settings.deepShadowClosedLiftStrictPrnList(:);
end
strictReopenStepM = max(0.35 * chipM, 120.0);
if isfield(settings, 'deepShadowClosedLiftStrictReopenStepM'), strictReopenStepM = settings.deepShadowClosedLiftStrictReopenStepM; end
strictAbortStepM = max(0.90 * chipM, 260.0);
if isfield(settings, 'deepShadowClosedLiftStrictAbortStepM'), strictAbortStepM = settings.deepShadowClosedLiftStrictAbortStepM; end
strictAbortDevM = 600.0;
if isfield(settings, 'deepShadowClosedLiftStrictAbortDevM'), strictAbortDevM = settings.deepShadowClosedLiftStrictAbortDevM; end
abortDevM = 900.0;
if isfield(settings, 'deepShadowClosedLiftAbortDevM'), abortDevM = settings.deepShadowClosedLiftAbortDevM; end
abortRepairChips = 12;
if isfield(settings, 'deepShadowClosedLiftAbortRepairChips'), abortRepairChips = settings.deepShadowClosedLiftAbortRepairChips; end

segmentId = prevSegmentId;
segmentId(~isfinite(segmentId) | segmentId < 0) = 0;
chosenDelta = nan(n,1);
seed = false(n,1);
continuous = false(n,1);
repairSeed = false(n,1);
rawDeltaLiftAll = deltaRaw - rawAmbig * chipM;
rawCommonLift = median(rawDeltaLiftAll(good & isfinite(rawDeltaLiftAll)), 'omitnan');

for ii = 1:n
    if ~good(ii) || ~isfinite(rawAmbig(ii))
        continue;
    end
    isStrict = any(prnList(ii) == strictPrn);
    reopenGateM = seedGateM;
    if isStrict
        reopenGateM = min(seedGateM, strictReopenStepM);
    end
    rawDelta = rawP(ii) - rawAmbig(ii) * chipM + settings.c * satClkCorr(ii) - rho(ii);
    if isfinite(prevAmbigChips(ii)) && isfinite(lastDeltaLift(ii))
        prevDelta = rawP(ii) - prevAmbigChips(ii) * chipM + settings.c * satClkCorr(ii) - rho(ii);
        prevLiftAll = rawDeltaLiftAll;
        prevLiftAll(ii) = prevDelta;
        prevGood = good & isfinite(prevLiftAll);
        prevCommon = rawCommonLift;
        if sum(prevGood) >= 3
            prevCommon = median(prevLiftAll(prevGood), 'omitnan');
        end
        prevDet = prevDelta - prevCommon;
        rawDet = rawDelta - rawCommonLift;
        stepPrev = abs(prevDet - lastDeltaLift(ii));
        stepRaw = abs(rawDet - lastDeltaLift(ii));
        if stepPrev <= halfChipM
            ambigChips(ii) = prevAmbigChips(ii);
            chosenDelta(ii) = prevDelta;
            continuous(ii) = true;
        elseif stepRaw <= reopenGateM && stepRaw < stepPrev
            ambigChips(ii) = rawAmbig(ii);
            chosenDelta(ii) = rawDelta;
            segmentId(ii) = segmentId(ii) + 1;
            seed(ii) = true;
        elseif ~isStrict && abs(rawAmbig(ii) - prevAmbigChips(ii)) <= repairMaxChips && stepPrev <= reopenGateM
            ambigChips(ii) = prevAmbigChips(ii);
            chosenDelta(ii) = prevDelta;
            seed(ii) = true;
            repairSeed(ii) = true;
        else
            segmentId(ii) = segmentId(ii) + 1;
        end
    else
        ambigChips(ii) = rawAmbig(ii);
        chosenDelta(ii) = rawDelta;
        segmentId(ii) = max(1, segmentId(ii) + 1);
        seed(ii) = true;
    end
end

valid = good & isfinite(ambigChips) & isfinite(chosenDelta);
if sum(valid) < 3
    return;
end
medLift = median(chosenDelta(valid), 'omitnan');
deltaDet = chosenDelta - medLift;

for ii = 1:n
    if ~valid(ii)
        continue;
    end
    if continuous(ii)
        stepDet = abs(deltaDet(ii) - lastDeltaLift(ii));
        if isfinite(stepDet) && stepDet <= qualityStepGateM
            quality(ii) = 2;
        elseif isfinite(stepDet) && stepDet <= seedGateM
            quality(ii) = 1;
        else
            quality(ii) = 0;
        end
    elseif seed(ii)
        quality(ii) = 1;
    end
end

repairChips(valid) = rawAmbig(valid) - ambigChips(valid);
dev = abs(deltaDet);
madLift = median(dev(valid), 'omitnan');
gateM = max(halfChipM, 4 * max(madLift, 50));
bad = valid & dev > gateM;
for ii = 1:n
    if ~valid(ii)
        continue;
    end
    isStrict = any(prnList(ii) == strictPrn);
    abortNow = false;
    if bad(ii)
        abortNow = true;
    elseif isStrict && seed(ii) && dev(ii) > strictAbortDevM
        abortNow = true;
    elseif isStrict && continuous(ii)
        stepDet = abs(deltaDet(ii) - lastDeltaLift(ii));
        if isfinite(stepDet) && stepDet > strictAbortStepM
            abortNow = true;
        end
    elseif repairSeed(ii) && abs(repairChips(ii)) > abortRepairChips
        abortNow = true;
    elseif dev(ii) > abortDevM
        abortNow = true;
    end
    if abortNow
        quality(ii) = 0;
        segmentId(ii) = max(segmentId(ii), prevSegmentId(ii) + 1);
        ambigChips(ii) = nan;
        chosenDelta(ii) = nan;
    end
end

valid = valid & quality > 0 & isfinite(ambigChips) & isfinite(chosenDelta);
rawPLift(valid) = rawP(valid) - ambigChips(valid) * chipM;
deltaLift(valid) = deltaDet(valid);
deltaLiftAbs(valid) = chosenDelta(valid);
repairChips(valid) = rawAmbig(valid) - ambigChips(valid);
end

function [deltaLift, ambigChips] = localResolveChipAmbiguity(deltaVec, settings)
deltaLift = deltaVec;
ambigChips = zeros(size(deltaVec));
if ~isfield(settings, 'deepShadowRawAmbiguityEnable') || ~settings.deepShadowRawAmbiguityEnable
    return;
end
good = isfinite(deltaVec);
if sum(good) < settings.deepShadowRawAmbiguityMinSat
    return;
end
chipM = localChipMeters(settings);
if ~isfinite(chipM) || chipM <= 0
    return;
end
refMed = median(deltaVec(good), 'omitnan');
rawChips = round((deltaVec - refMed) / chipM);
rawChips(~good) = 0;
rawChips = max(-settings.deepShadowRawAmbiguityMaxChips, min(settings.deepShadowRawAmbiguityMaxChips, rawChips));
deltaLift = deltaVec - rawChips * chipM;
ambigChips = rawChips;
end

function [deltaLift, ambigChips] = localLiftDs5RawAnchorDelta(rawDelta, rawValidMask, navResults, currMeasNr, settings)
rawDelta = rawDelta(:);
deltaLift = nan(size(rawDelta));
ambigChips = nan(size(rawDelta));
if isempty(rawDelta)
    return;
end
chipM = localChipMeters(settings);
if ~isfinite(chipM) || chipM <= 0
    return;
end
good = isfinite(rawDelta);
if sum(good) < max(1, round(localGetSettingValue(settings, 'deepShadowDs5RefObsPositionMinSat', 4)))
    return;
end
refMed = median(rawDelta(good), 'omitnan');
rawChips = round((rawDelta - refMed) / chipM);
rawChips(~good) = nan;
deltaLift(good) = rawDelta(good) - rawChips(good) * chipM;
ambigChips(good) = rawChips(good);
if localGetSettingValue(settings, 'deepShadowDs5RefObsPositionRawAnchorLiftUseCodeCorr', 1) ~= 0 && ...
        isfield(navResults, 'shadowDs5RefObsLocalCorrChips') && currMeasNr <= size(navResults.shadowDs5RefObsLocalCorrChips, 2)
    rawIdx = find(rawValidMask(:));
    localCorr = navResults.shadowDs5RefObsLocalCorrChips(rawIdx, currMeasNr);
    corrGood = isfinite(localCorr(:)) & isfinite(deltaLift(:));
    deltaLift(corrGood) = deltaLift(corrGood) + localCorr(corrGood) * chipM;
end
end

function [deltaUse, sourceMode, refDelta] = localSelectShadowNavDeltaSource(deltaLift, deltaClosedLiftAbs, qualityLocal, prnVec, lastGoodDelta, lastGoodEpoch, lastGoodOffsetToBase, currMeasNr, settings, refObsMask, refObsDelta)
deltaUse = deltaLift;
sourceMode = ones(size(deltaLift));
refDelta = nan(size(deltaLift));
if isempty(deltaLift)
    return;
end
deltaLift = deltaLift(:);
deltaUse = deltaLift;
sourceMode = ones(size(deltaLift));
refDelta = nan(size(deltaLift));
if nargin < 2 || isempty(deltaClosedLiftAbs), deltaClosedLiftAbs = nan(size(deltaLift)); end
if nargin < 3 || isempty(qualityLocal), qualityLocal = zeros(size(deltaLift)); end
if nargin < 4 || isempty(prnVec), prnVec = nan(size(deltaLift)); end
if nargin < 5 || isempty(lastGoodDelta), lastGoodDelta = nan(size(deltaLift)); end
if nargin < 6 || isempty(lastGoodEpoch), lastGoodEpoch = nan(size(deltaLift)); end
if nargin < 7 || isempty(lastGoodOffsetToBase), lastGoodOffsetToBase = nan(size(deltaLift)); end
if nargin < 10 || isempty(refObsMask), refObsMask = false(size(deltaLift)); end
if nargin < 11 || isempty(refObsDelta), refObsDelta = nan(size(deltaLift)); end
deltaClosedLiftAbs = deltaClosedLiftAbs(:);
qualityLocal = qualityLocal(:);
prnVec = prnVec(:);
lastGoodDelta = lastGoodDelta(:);
lastGoodEpoch = lastGoodEpoch(:);
lastGoodOffsetToBase = lastGoodOffsetToBase(:);
[baseMed, baseCnt] = localComputeShadowNavBaseMed(deltaLift, prnVec, settings);
if localUseDs5ReferenceObsModel(settings, currMeasNr)
    for ii = 1:numel(deltaUse)
        if ii <= numel(refObsMask) && refObsMask(ii)
            deltaUse(ii) = refObsDelta(ii);
            sourceMode(ii) = 2;
            refDelta(ii) = baseMed;
        end
    end
end
preferClosedLift = localUseDs5ClosedLiftPrefer(settings, currMeasNr);
rawRejectDevM = localGetSettingValue(settings, 'deepShadowDs5RawLiftRejectDevM', 12000.0);
for ii = 1:numel(deltaUse)
    if ~isfinite(deltaLift(ii))
        continue;
    end
    if preferClosedLift && isfinite(baseMed)
        rawDevNow = abs(deltaLift(ii) - baseMed);
        if rawDevNow > rawRejectDevM
            deltaUse(ii) = nan;
            sourceMode(ii) = 4;
            continue;
        end
    end
    if sourceMode(ii) == 2
        continue;
    end
    if preferClosedLift
        [corrOk, corrDelta] = localBuildDs5ClosedLiftCorrectedDelta(baseMed, baseCnt, deltaLift(ii), ...
            deltaClosedLiftAbs(ii), qualityLocal(ii), prnVec(ii), settings);
        if corrOk
            deltaUse(ii) = corrDelta;
            sourceMode(ii) = 2;
        end
    end
end
if ~localUseDs5TailSourceFallback(settings, currMeasNr)
    return;
end
focusPrns = localGetSettingValue(settings, 'deepShadowDs5TailSourceFallbackPrns', [27 3]);
if isempty(focusPrns)
    return;
end
devThr = localGetSettingValue(settings, 'deepShadowDs5TailSourceFallbackDevThrM', 1500.0);
jumpThr = localGetSettingValue(settings, 'deepShadowDs5TailSourceFallbackJumpThrM', 600.0);
navSolPeriodMs = localGetSettingValue(settings, 'navSolPeriod', 500.0);
holdMaxAgeSec = localGetSettingValue(settings, 'deepShadowDs5TailSourceFallbackHoldMaxAgeSec', 120.0);
useClosedLift = localGetSettingValue(settings, 'deepShadowDs5TailSourceFallbackUseClosedLift', 1) ~= 0;
useHold = localGetSettingValue(settings, 'deepShadowDs5TailSourceFallbackUseHold', 1) ~= 0;
for ii = 1:numel(deltaUse)
    if sourceMode(ii) == 2 || ~isfinite(deltaLift(ii)) || ~ismember(prnVec(ii), focusPrns(:))
        continue;
    end
    rawDev = abs(deltaLift(ii) - baseMed);
    if useClosedLift
        [corrOk, corrDelta] = localBuildDs5ClosedLiftCorrectedDelta(baseMed, baseCnt, deltaLift(ii), ...
            deltaClosedLiftAbs(ii), qualityLocal(ii), prnVec(ii), settings);
        if corrOk
            deltaUse(ii) = corrDelta;
            sourceMode(ii) = 2;
            continue;
        end
    end
    holdAgeSec = inf;
    if isfinite(lastGoodEpoch(ii))
        holdAgeSec = (double(currMeasNr) - double(lastGoodEpoch(ii))) * navSolPeriodMs / 1000.0;
    end
    if useHold && isfinite(lastGoodDelta(ii)) && isfinite(holdAgeSec) && holdAgeSec <= holdMaxAgeSec && ...
            ((isfinite(rawDev) && rawDev > devThr) || abs(deltaLift(ii) - lastGoodDelta(ii)) > jumpThr)
        if isfinite(baseMed) && baseCnt >= 2 && isfinite(lastGoodOffsetToBase(ii))
            deltaUse(ii) = baseMed + lastGoodOffsetToBase(ii);
        else
            deltaUse(ii) = lastGoodDelta(ii);
        end
        sourceMode(ii) = 3;
    end
end
end

function [useMask, deltaRefObs, info] = localBuildDs5ReferenceRelativeObs(deltaLift, baseMed, baseCnt, passShadowIdx, prnVec, trackShadow, trueRefCodeState, trueRefReadyState, trueRefHzState, settings)
deltaLift = deltaLift(:);
passShadowIdx = passShadowIdx(:);
useMask = false(size(deltaLift));
deltaRefObs = nan(size(deltaLift));
info = struct('baseDeltaM', nan(size(deltaLift)), 'localCorrChips', nan(size(deltaLift)), ...
    'freqErrHz', nan(size(deltaLift)), 'codeErrChips', nan(size(deltaLift)), ...
    'seedDeltaM', nan(size(deltaLift)), 'seedUseBase', false(size(deltaLift)));
if isempty(deltaLift)
    return;
end
if ~isfinite(baseMed) || ~isfinite(baseCnt)
    [baseMed, baseCnt] = localComputeShadowNavBaseMed(deltaLift, prnVec, settings);
end
if ~isfinite(baseMed)
    return;
end
minBaseSat = localGetSettingValue(settings, 'deepShadowDs5RefObsMinBaseSat', 3);
if baseCnt < minBaseSat
    return;
end
chipM = localChipMeters(settings);
maxCorrChips = localGetSettingValue(settings, 'deepShadowDs5RefObsLocalCorrMaxChips', 1.50);
useCodeCorr = localGetSettingValue(settings, 'deepShadowDs5RefObsUseCodeCorr', 1) ~= 0;
requireActive = localGetSettingValue(settings, 'deepShadowDs5RefObsRequireActiveTrack', 0) ~= 0;
useBaseFallback = localGetSettingValue(settings, 'deepShadowDs5RefObsSeedUseBaseFallback', 1) ~= 0;
seedMaxDevFromBaseM = localGetSettingValue(settings, 'deepShadowDs5RefObsSeedMaxDevFromBaseM', 1500.0);
for ii = 1:numel(passShadowIdx)
    jj = passShadowIdx(ii);
    if jj < 1 || jj > numel(trackShadow) || jj > numel(trueRefReadyState) || ~trueRefReadyState(jj)
        continue;
    end
    effCode = trackShadow(jj).remCodePhase;
    if useCodeCorr
        effCode = effCode + localGetTrackField(trackShadow(jj), 'deepCodePhaseCorrChips', 0.0);
    end
    if ~isfinite(effCode) || jj > numel(trueRefCodeState) || ~isfinite(trueRefCodeState(jj))
        continue;
    end
    localCorrChips = localWrapChipDiff(trueRefCodeState(jj) - effCode);
    info.baseDeltaM(ii) = baseMed;
    info.localCorrChips(ii) = localCorrChips;
    info.codeErrChips(ii) = -localCorrChips;
    if jj <= numel(trueRefHzState) && isfinite(trueRefHzState(jj))
        info.freqErrHz(ii) = (trackShadow(jj).carrFreq - settings.IF) - trueRefHzState(jj);
    end
    if requireActive && ~all(isfinite([trackShadow(jj).carrFreq, trackShadow(jj).codeFreq]))
        continue;
    end
    if abs(localCorrChips) > maxCorrChips
        continue;
    end
    seedDeltaM = deltaLift(ii);
    seedUseBaseNow = false;
    if (~isfinite(seedDeltaM) || ...
            (isfinite(baseMed) && isfinite(seedMaxDevFromBaseM) && seedMaxDevFromBaseM > 0 && isfinite(seedDeltaM) && abs(seedDeltaM - baseMed) > seedMaxDevFromBaseM)) ...
            && useBaseFallback && isfinite(baseMed)
        seedDeltaM = baseMed;
        seedUseBaseNow = true;
    end
    if ~isfinite(seedDeltaM)
        continue;
    end
    info.seedDeltaM(ii) = seedDeltaM;
    info.seedUseBase(ii) = seedUseBaseNow;
    deltaRefObs(ii) = seedDeltaM + localCorrChips * chipM;
    useMask(ii) = isfinite(deltaRefObs(ii));
end
end

function diffWrap = localWrapChipDiff(diffChips)
chipLen = 1023.0;
diffWrap = mod(diffChips + chipLen / 2, chipLen) - chipLen / 2;
end

function tf = localUseDs5ReferenceObsModel(settings, currMeasNr)
tf = false;
if ~isfield(settings, 'deepShadowDs5RefObsModelEnable') || ~settings.deepShadowDs5RefObsModelEnable
    return;
end
if ~localIsDs5Scenario(settings)
    return;
end
startSec = localGetSettingValue(settings, 'deepShadowDs5RefObsModelStartSec', 92.0);
navSolPeriodMs = localGetSettingValue(settings, 'navSolPeriod', 500.0);
timeSec = (double(currMeasNr) - 1.0) * navSolPeriodMs / 1000.0;
tf = timeSec >= startSec;
end

function [ok, correctedDelta] = localBuildDs5ClosedLiftCorrectedDelta(baseMed, baseCnt, baseDelta, closedLiftDeltaAbs, qualityLocal, prnVal, settings)
ok = false;
correctedDelta = baseDelta;
if ~isfinite(baseMed) || ~isfinite(baseDelta) || ~isfinite(closedLiftDeltaAbs)
    return;
end
minBaseSat = localGetSettingValue(settings, 'deepShadowDs5ClosedLiftCorrMinBaseSat', 2);
if baseCnt < minBaseSat
    return;
end
strictPrns = localGetSettingValue(settings, 'deepShadowDs5ClosedLiftCorrStrictPrns', [27 3]);
isStrict = isfinite(prnVal) && any(prnVal == strictPrns(:));
minQuality = localGetSettingValue(settings, 'deepShadowDs5ClosedLiftCorrMinQuality', 1);
if isStrict
    minQuality = localGetSettingValue(settings, 'deepShadowDs5ClosedLiftCorrStrictMinQuality', 2);
end
if qualityLocal < minQuality
    return;
end
correctedDeltaCand = closedLiftDeltaAbs;
corrResidual = correctedDeltaCand - baseDelta;
corrResidualGateM = localGetSettingValue(settings, 'deepShadowDs5ClosedLiftCorrResidualGateM', 450.0);
if isStrict
    corrResidualGateM = localGetSettingValue(settings, 'deepShadowDs5ClosedLiftCorrStrictResidualGateM', 180.0);
end
applyMaxM = localGetSettingValue(settings, 'deepShadowDs5ClosedLiftCorrApplyMaxM', 3000.0);
if abs(corrResidual) > max(corrResidualGateM, applyMaxM)
    return;
end
ok = true;
correctedDelta = correctedDeltaCand;
end

function [baseMed, baseCnt] = localComputeShadowNavBaseMed(deltaLift, prnVec, settings)
baseMed = nan;
baseCnt = 0;
if nargin < 1 || isempty(deltaLift)
    return;
end
deltaLift = deltaLift(:);
if nargin < 2 || isempty(prnVec)
    prnVec = nan(size(deltaLift));
else
    prnVec = prnVec(:);
end
focusPrns = localGetSettingValue(settings, 'deepShadowDs5TailSourceFallbackPrns', [27 3]);
baseMask = isfinite(deltaLift) & ~ismember(prnVec, focusPrns(:));
baseCnt = sum(baseMask);
if baseCnt > 0
    baseMed = median(deltaLift(baseMask), 'omitnan');
end
if ~isfinite(baseMed)
    finiteMask = isfinite(deltaLift);
    baseCnt = sum(finiteMask);
    if baseCnt > 0
        baseMed = median(deltaLift(finiteMask), 'omitnan');
    end
end
end

function tf = localUseDs5TailSourceFallback(settings, currMeasNr)
tf = false;
if ~isfield(settings, 'deepShadowDs5TailSourceFallbackEnable') || ~settings.deepShadowDs5TailSourceFallbackEnable
    return;
end
if ~localIsDs5Scenario(settings)
    return;
end
startSec = localGetSettingValue(settings, 'deepShadowDs5TailSourceFallbackStartSec', 118.0);
navSolPeriodMs = localGetSettingValue(settings, 'navSolPeriod', 500.0);
timeSec = (double(currMeasNr) - 1.0) * navSolPeriodMs / 1000.0;
tf = timeSec >= startSec;
end

function [keepMask, coreMask, expandMask, coreMaxDevM, expandMaxDevM, relaxInfo] = localSelectNavQualifiedSubset(deltaVec, prnVec, currMeasNr, settings)
keepMask = false(size(deltaVec));
coreMask = false(size(deltaVec));
expandMask = false(size(deltaVec));
coreMaxDevM = nan;
expandMaxDevM = nan;
relaxInfo = localInitDs5RelaxInfo(deltaVec);
good = find(isfinite(deltaVec));
if isempty(good)
    return;
end
if numel(good) >= 4
    dAll = deltaVec(good);
    bestLocal = localBestConsistentSubset(dAll, 4);
    coreGood = good(bestLocal);
    coreMed = median(deltaVec(coreGood), 'omitnan');
    coreDev = abs(deltaVec(coreGood) - coreMed);
    if any(~isfinite(coreDev))
        return;
    end
    coreMaxDevM = max(coreDev);
    if coreMaxDevM > settings.deepShadowRawNavCoreDevThrM
        return;
    end
    coreMask(coreGood) = true;
    keepMask(coreGood) = true;
    if isfield(settings, 'deepShadowRawNavExpandEnable') && settings.deepShadowRawNavExpandEnable
        devAll = abs(dAll - coreMed);
        expandLocal = devAll <= settings.deepShadowRawNavExpandDevThrM;
        expandMask(good(expandLocal)) = true;
        keepMask(good(expandLocal)) = true;
        if any(expandLocal)
            expandMaxDevM = max(devAll(expandLocal));
        end
    else
        expandMask = coreMask;
        expandMaxDevM = coreMaxDevM;
    end
    [keepMask, coreMask, expandMask, coreMaxDevM, expandMaxDevM, relaxInfo] = ...
        localRelaxDs5TailNavSubset(deltaVec, prnVec, currMeasNr, keepMask, coreMask, expandMask, coreMaxDevM, expandMaxDevM, settings, relaxInfo);
    return;
end
if numel(good) <= 2
    keepMask(good) = true;
    coreMask(good) = true;
    expandMask(good) = true;
    coreMaxDevM = max(abs(deltaVec(good) - median(deltaVec(good), 'omitnan')), [], 'omitnan');
    expandMaxDevM = coreMaxDevM;
    return;
end
d = deltaVec(good);
med0 = median(d, 'omitnan');
dev = abs(d - med0);
mad0 = median(dev, 'omitnan');
thr = max(300, 2.5 * max(mad0, 50));
sel = dev <= thr;
if sum(sel) < 2
    [~, idx] = sort(dev, 'ascend');
    sel = false(size(sel));
    sel(idx(1:min(2, numel(idx)))) = true;
end
keepMask(good(sel)) = true;
coreMask(good(sel)) = true;
expandMask(good(sel)) = true;
coreMaxDevM = max(dev(sel), [], 'omitnan');
expandMaxDevM = coreMaxDevM;
[keepMask, coreMask, expandMask, coreMaxDevM, expandMaxDevM, relaxInfo] = ...
    localRelaxDs5TailNavSubset(deltaVec, prnVec, currMeasNr, keepMask, coreMask, expandMask, coreMaxDevM, expandMaxDevM, settings, relaxInfo);
end

function [keepMask, coreMask, expandMask, coreMaxDevM, expandMaxDevM, relaxInfo] = localRelaxDs5TailNavSubset(deltaVec, prnVec, currMeasNr, keepMask, coreMask, expandMask, coreMaxDevM, expandMaxDevM, settings, relaxInfo)
if nargin < 9 || ~isstruct(relaxInfo)
    relaxInfo = localInitDs5RelaxInfo(deltaVec);
end
if ~localUseDs5TailNavRelax(settings, currMeasNr)
    return;
end
if isempty(prnVec) || numel(prnVec) ~= numel(deltaVec)
    return;
end
targetPrns = localGetSettingValue(settings, 'deepShadowDs5TailNavRelaxPrns', [27 3]);
if isempty(targetPrns)
    return;
end
targetMask = isfinite(deltaVec) & ismember(prnVec(:), targetPrns(:));
relaxInfo.targetMask = targetMask;
if ~any(targetMask)
    return;
end
minBaseSat = localGetSettingValue(settings, 'deepShadowDs5TailNavRelaxMinBaseSat', 3);
baseMask = isfinite(deltaVec) & keepMask & ~ismember(prnVec(:), targetPrns(:));
if sum(baseMask) < minBaseSat
    baseMask = isfinite(deltaVec) & ~ismember(prnVec(:), targetPrns(:));
end
relaxInfo.baseMask = baseMask;
if sum(baseMask) < minBaseSat
    return;
end
baseVals = deltaVec(baseMask);
baseMed = median(baseVals, 'omitnan');
baseDev = abs(baseVals - baseMed);
baseMad = median(baseDev, 'omitnan');
minThr = localGetSettingValue(settings, 'deepShadowDs5TailNavRelaxMinThrM', 250.0);
maxThr = localGetSettingValue(settings, 'deepShadowDs5TailNavRelaxMaxThrM', 2500.0);
madScale = localGetSettingValue(settings, 'deepShadowDs5TailNavRelaxMadScale', 4.0);
coreScale = localGetSettingValue(settings, 'deepShadowDs5TailNavRelaxCoreScale', 0.75);
pairThr = localGetSettingValue(settings, 'deepShadowDs5TailNavRelaxPairThrM', 1200.0);
navThr = min(max(minThr, madScale * max(baseMad, minThr / max(madScale, 1))), maxThr);
coreThr = min(navThr, max(minThr, coreScale * navThr));
relaxInfo.baseMedM = baseMed;
relaxInfo.navThrM = navThr;
relaxInfo.coreThrM = coreThr;
candIdx = find(targetMask);
candDev = abs(deltaVec(candIdx) - baseMed);
relaxInfo.devM(candIdx) = candDev;
admit = candDev <= navThr;
if ~any(admit)
    return;
end
admitIdx = candIdx(admit);
admitDev = candDev(admit);
keepPair = true(size(admitIdx));
if numel(admitIdx) > 1
    [~, order] = sort(admitDev, 'ascend');
    leadIdx = admitIdx(order(1));
    relaxInfo.pairLeadPrn = prnVec(leadIdx);
    keepPair = abs(deltaVec(admitIdx) - deltaVec(leadIdx)) <= pairThr;
    relaxInfo.pairOk(admitIdx) = keepPair;
    admitIdx = admitIdx(keepPair);
elseif numel(admitIdx) == 1
    relaxInfo.pairLeadPrn = prnVec(admitIdx(1));
    relaxInfo.pairOk(admitIdx) = true;
end
if isempty(admitIdx)
    return;
end
keepMask(admitIdx) = true;
expandMask(admitIdx) = true;
relaxInfo.used(admitIdx) = true;
coreIdx = admitIdx(abs(deltaVec(admitIdx) - baseMed) <= coreThr);
if ~isempty(coreIdx)
    coreMask(coreIdx) = true;
    relaxInfo.coreUsed(coreIdx) = true;
end
coreVals = deltaVec(coreMask & isfinite(deltaVec));
if ~isempty(coreVals)
    coreMed = median(coreVals, 'omitnan');
    coreMaxDevM = max(abs(coreVals - coreMed), [], 'omitnan');
end
expandVals = deltaVec(expandMask & isfinite(deltaVec));
if ~isempty(expandVals)
    expandMed = median(expandVals, 'omitnan');
    expandMaxDevM = max(abs(expandVals - expandMed), [], 'omitnan');
end
end

function info = localInitDs5RelaxInfo(deltaVec)
n = numel(deltaVec);
info = struct('used', false(n,1), 'coreUsed', false(n,1), 'targetMask', false(n,1), ...
    'baseMask', false(n,1), 'devM', nan(n,1), 'baseMedM', nan, 'navThrM', nan, ...
    'coreThrM', nan, 'pairLeadPrn', nan, 'pairOk', false(n,1));
end

function [deltaOut, info, commonM, rateMps, lastEpoch] = localApplyDs5CommonDrag(deltaIn, prnVec, sourceMode, currMeasNr, prevCommonM, prevRateMps, prevEpoch, settings)
deltaOut = deltaIn;
commonM = prevCommonM;
rateMps = prevRateMps;
lastEpoch = prevEpoch;
info = struct('applied', false, 'commonM', nan, 'rateMps', prevRateMps, 'baseSatNum', 0, ...
    'keepSatNum', 0, 'source', 0, 'predM', nan, 'measM', nan, 'p95BeforeM', nan, 'p95AfterM', nan);
if isempty(deltaIn)
    return;
end
deltaIn = deltaIn(:);
deltaOut = deltaIn;
if nargin < 2 || isempty(prnVec)
    prnVec = nan(size(deltaIn));
else
    prnVec = prnVec(:);
end
if nargin < 3 || isempty(sourceMode)
    sourceMode = zeros(size(deltaIn));
else
    sourceMode = sourceMode(:);
end
finiteAll = isfinite(deltaIn);
if any(finiteAll)
    info.p95BeforeM = prctile(abs(deltaIn(finiteAll)), 95);
end
if ~localUseDs5CommonDrag(settings, currMeasNr)
    if any(finiteAll)
        info.p95AfterM = info.p95BeforeM;
    end
    return;
end
focusPrns = localGetSettingValue(settings, 'deepShadowDs5CommonDragFocusPrns', [27 3]);
minBaseSat = localGetSettingValue(settings, 'deepShadowDs5CommonDragMinBaseSat', 3);
gateFloorM = localGetSettingValue(settings, 'deepShadowDs5CommonDragGateFloorM', 250.0);
madScale = localGetSettingValue(settings, 'deepShadowDs5CommonDragMadScale', 4.0);
maxInnovM = localGetSettingValue(settings, 'deepShadowDs5CommonDragMaxInnovM', 6000.0);
maxRateMps = localGetSettingValue(settings, 'deepShadowDs5CommonDragMaxRateMps', 4000.0);
navSolPeriodMs = localGetSettingValue(settings, 'navSolPeriod', 500.0);
dtSec = navSolPeriodMs / 1000.0;
predM = nan;
if isfinite(prevCommonM)
    predM = prevCommonM;
    if isfinite(prevEpoch) && isfinite(prevRateMps) && currMeasNr > prevEpoch
        predM = prevCommonM + prevRateMps * ((double(currMeasNr) - double(prevEpoch)) * dtSec);
    end
end
baseMask = finiteAll & ~ismember(prnVec, focusPrns(:));
baseSatNum = sum(baseMask);
keepMask = false(size(deltaIn));
measM = nan;
sourceLocal = 0;
if baseSatNum >= minBaseSat
    baseVals = deltaIn(baseMask);
    baseMed0 = median(baseVals, 'omitnan');
    baseDev0 = abs(baseVals - baseMed0);
    baseMad0 = median(baseDev0, 'omitnan');
    gateM = max(gateFloorM, madScale * max(baseMad0, 50.0));
    keepLocal = baseDev0 <= gateM;
    baseIdx = find(baseMask);
    keepMask(baseIdx(keepLocal)) = true;
    if sum(keepMask) >= minBaseSat
        measM = median(deltaIn(keepMask), 'omitnan');
        sourceLocal = 1;
    end
end
if ~isfinite(measM) && any(finiteAll)
    measM = median(deltaIn(finiteAll), 'omitnan');
    keepMask = finiteAll;
    sourceLocal = 1;
end
commonCand = measM;
if isfinite(predM)
    if isfinite(commonCand)
        innovM = commonCand - predM;
        innovM = max(-maxInnovM, min(maxInnovM, innovM));
        commonCand = predM + innovM;
        sourceLocal = 3;
    else
        commonCand = predM;
        sourceLocal = 2;
    end
end
if ~isfinite(commonCand)
    if any(finiteAll)
        info.p95AfterM = info.p95BeforeM;
    end
    return;
end
deltaOut(finiteAll) = deltaIn(finiteAll) - commonCand;
info.commonM = commonCand;
info.baseSatNum = baseSatNum;
info.predM = predM;
info.measM = measM;
if any(isfinite(deltaOut))
    info.p95AfterM = prctile(abs(deltaOut(isfinite(deltaOut))), 95);
end
rateNew = prevRateMps;
if isfinite(prevCommonM)
    rateNew = (commonCand - prevCommonM) / dtSec;
end
if isfinite(rateNew)
    rateNew = max(-maxRateMps, min(maxRateMps, rateNew));
else
    rateNew = 0.0;
end
commonM = commonCand;
rateMps = rateNew;
lastEpoch = currMeasNr;
info.rateMps = rateMps;
end

function tf = localUseDs5CommonDrag(settings, currMeasNr)
tf = false;
if ~isfield(settings, 'deepShadowDs5CommonDragEnable') || ~settings.deepShadowDs5CommonDragEnable
    return;
end
if ~localIsDs5Scenario(settings)
    return;
end
startSec = localGetSettingValue(settings, 'deepShadowDs5CommonDragStartSec', 118.0);
navSolPeriodMs = localGetSettingValue(settings, 'navSolPeriod', 500.0);
timeSec = (double(currMeasNr) - 1.0) * navSolPeriodMs / 1000.0;
tf = timeSec >= startSec;
end

function [info, commonHz, rateHzps, lastEpoch] = localEstimateDs5CommonDopp(dopplerMeas, dopplerPred, biasHz, prnList, currMeasNr, prevCommonHz, prevRateHzps, prevEpoch, settings)
commonHz = prevCommonHz;
rateHzps = prevRateHzps;
lastEpoch = prevEpoch;
info = struct('applied', false, 'commonHz', nan, 'rateHzps', prevRateHzps, 'baseSatNum', 0, ...
    'keepSatNum', 0, 'source', 0, 'predHz', nan, 'measHz', nan, 'p95BeforeHz', nan, 'p95AfterHz', nan, ...
    'rmsBeforeHz', nan, 'rmsAfterHz', nan, 'l2AfterHz', nan, 'alphaProxy', nan, 'alphaGatePass', false);
if isempty(dopplerMeas) || isempty(dopplerPred)
    return;
end
residualHz = dopplerMeas(:) - dopplerPred(:) - biasHz(:);
finiteAll = isfinite(residualHz);
if any(finiteAll)
    residualBefore = residualHz(finiteAll);
    info.p95BeforeHz = prctile(abs(residualBefore), 95);
    info.rmsBeforeHz = sqrt(mean(residualBefore.^2, 'omitnan'));
end
if ~localUseDs5CommonDopp(settings, currMeasNr)
    if any(finiteAll)
        info.p95AfterHz = info.p95BeforeHz;
        info.rmsAfterHz = info.rmsBeforeHz;
        residualFinite = residualHz(finiteAll);
        info.l2AfterHz = sqrt(sum(residualFinite.^2));
        info.alphaProxy = 0.0;
    end
    return;
end
if nargin < 4 || isempty(prnList)
    prnList = nan(size(residualHz));
else
    prnList = prnList(:);
end
focusPrns = localGetSettingValue(settings, 'deepShadowDs5CommonDoppFocusPrns', [27 3]);
minBaseSat = localGetSettingValue(settings, 'deepShadowDs5CommonDoppMinBaseSat', 3);
gateFloorHz = localGetSettingValue(settings, 'deepShadowDs5CommonDoppGateFloorHz', 10.0);
madScale = localGetSettingValue(settings, 'deepShadowDs5CommonDoppMadScale', 4.0);
maxInnovHz = localGetSettingValue(settings, 'deepShadowDs5CommonDoppMaxInnovHz', 80.0);
maxRateHzps = localGetSettingValue(settings, 'deepShadowDs5CommonDoppMaxRateHzps', 120.0);
navSolPeriodMs = localGetSettingValue(settings, 'navSolPeriod', 500.0);
dtSec = navSolPeriodMs / 1000.0;
predHz = nan;
if isfinite(prevCommonHz)
    predHz = prevCommonHz;
    if isfinite(prevEpoch) && isfinite(prevRateHzps) && currMeasNr > prevEpoch
        predHz = prevCommonHz + prevRateHzps * ((double(currMeasNr) - double(prevEpoch)) * dtSec);
    end
end
baseMask = finiteAll & ~ismember(prnList, focusPrns(:));
info.baseSatNum = sum(baseMask);
keepMask = false(size(residualHz));
measHz = nan;
sourceLocal = 0;
if info.baseSatNum >= minBaseSat
    baseVals = residualHz(baseMask);
    baseMed0 = median(baseVals, 'omitnan');
    baseDev0 = abs(baseVals - baseMed0);
    baseMad0 = median(baseDev0, 'omitnan');
    gateHz = max(gateFloorHz, madScale * max(baseMad0, 2.0));
    keepLocal = baseDev0 <= gateHz;
    baseIdx = find(baseMask);
    keepMask(baseIdx(keepLocal)) = true;
    if sum(keepMask) >= minBaseSat
        measHz = median(residualHz(keepMask), 'omitnan');
        sourceLocal = 1;
    end
end
if ~isfinite(measHz) && any(finiteAll)
    measHz = median(residualHz(finiteAll), 'omitnan');
    keepMask = finiteAll;
    sourceLocal = 1;
end
commonCand = measHz;
if isfinite(predHz)
    if isfinite(commonCand)
        innovHz = commonCand - predHz;
        innovHz = max(-maxInnovHz, min(maxInnovHz, innovHz));
        commonCand = predHz + innovHz;
        sourceLocal = 3;
    else
        commonCand = predHz;
        sourceLocal = 2;
    end
end
if ~isfinite(commonCand)
    if any(finiteAll)
        info.p95AfterHz = info.p95BeforeHz;
        info.rmsAfterHz = info.rmsBeforeHz;
        residualFinite = residualHz(finiteAll);
        info.l2AfterHz = sqrt(sum(residualFinite.^2));
        info.alphaProxy = 0.0;
    end
    return;
end
detrendedHz = residualHz(finiteAll) - commonCand;
if ~isempty(detrendedHz)
    info.p95AfterHz = prctile(abs(detrendedHz), 95);
    finiteDetrended = isfinite(detrendedHz);
    if any(finiteDetrended)
        detrendedFinite = detrendedHz(finiteDetrended);
        info.rmsAfterHz = sqrt(mean(detrendedFinite.^2, 'omitnan'));
        info.l2AfterHz = sqrt(sum(detrendedFinite.^2));
    end
end
if isfinite(info.rmsBeforeHz) && info.rmsBeforeHz > eps && isfinite(info.rmsAfterHz)
    info.alphaProxy = max(0.0, 1.0 - info.rmsAfterHz / max(info.rmsBeforeHz, eps));
end
alphaGateEnable = localGetSettingValue(settings, 'deepShadowDs5CommonDoppAlphaGateEnable', 0) ~= 0;
alphaProxyMin = localGetSettingValue(settings, 'deepShadowDs5CommonDoppAlphaProxyMin', 0.0);
info.alphaGatePass = ~alphaGateEnable || (isfinite(info.alphaProxy) && info.alphaProxy >= alphaProxyMin);
if alphaGateEnable && ~info.alphaGatePass
    return;
end
info.applied = true;
info.commonHz = commonCand;
info.keepSatNum = sum(keepMask);
info.source = sourceLocal;
info.predHz = predHz;
info.measHz = measHz;
rateNew = prevRateHzps;
if isfinite(prevCommonHz)
    rateNew = (commonCand - prevCommonHz) / dtSec;
end
if isfinite(rateNew)
    rateNew = max(-maxRateHzps, min(maxRateHzps, rateNew));
else
    rateNew = 0.0;
end
commonHz = commonCand;
rateHzps = rateNew;
lastEpoch = currMeasNr;
info.rateHzps = rateHzps;
end

function aidFreqOut = localApplyDs5CommonDoppToAid(aidFreqIn, commonHz, settings, currMeasNr)
aidFreqOut = aidFreqIn;
if ~localUseDs5CommonDopp(settings, currMeasNr)
    return;
end
if ~localGetSettingValue(settings, 'deepShadowDs5CommonDoppApplyToAidEnable', 1)
    return;
end
if ~isfinite(commonHz)
    return;
end
applyHz = commonHz;
maxApplyHz = localGetSettingValue(settings, 'deepShadowDs5CommonDoppAidApplyMaxHz', 200.0);
if isfinite(maxApplyHz)
    applyHz = max(-maxApplyHz, min(maxApplyHz, applyHz));
end
aidFreqOut = aidFreqIn - applyHz;
end

function tf = localUseDs5CommonDopp(settings, currMeasNr)
tf = false;
if ~isfield(settings, 'deepShadowDs5CommonDoppEnable') || ~settings.deepShadowDs5CommonDoppEnable
    return;
end
if ~localIsDs5Scenario(settings)
    return;
end
startSec = localGetSettingValue(settings, 'deepShadowDs5CommonDoppStartSec', 92.0);
navSolPeriodMs = localGetSettingValue(settings, 'navSolPeriod', 500.0);
timeSec = (double(currMeasNr) - 1.0) * navSolPeriodMs / 1000.0;
tf = timeSec >= startSec;
end

function [deltaOut, info] = localApplyDs5DetrendedOutlierGate(deltaIn, prnVec, sourceMode, currMeasNr, settings)
deltaOut = deltaIn;
if nargin < 2 || isempty(prnVec)
    prnVec = nan(size(deltaIn));
else
    prnVec = prnVec(:);
end
if nargin < 3 || isempty(sourceMode)
    sourceMode = zeros(size(deltaIn));
else
    sourceMode = sourceMode(:);
end
deltaIn = deltaIn(:);
n = numel(deltaIn);
info = struct('keepMask', isfinite(deltaIn), 'devM', nan(n,1), 'gateM', nan, 'rejectNum', 0);
if isempty(deltaIn) || ~localUseDs5DetrendedGate(settings, currMeasNr)
    return;
end
minSat = localGetSettingValue(settings, 'deepShadowDs5DetrendedGateMinSat', 4);
protectPrns = localGetSettingValue(settings, 'deepShadowDs5DetrendedGateProtectPrns', [27 3]);
gateFloorM = localGetSettingValue(settings, 'deepShadowDs5DetrendedGateFloorM', 180.0);
madScale = localGetSettingValue(settings, 'deepShadowDs5DetrendedGateMadScale', 4.0);
gateCeilM = localGetSettingValue(settings, 'deepShadowDs5DetrendedGateCeilM', 1200.0);
finiteMask = isfinite(deltaIn);
baseMask = finiteMask & ~ismember(prnVec, protectPrns(:));
if sum(baseMask) < minSat
    baseMask = finiteMask;
end
if sum(baseMask) < minSat
    return;
end
baseVals = deltaIn(baseMask);
baseMed = median(baseVals, 'omitnan');
baseDev = abs(baseVals - baseMed);
baseMad = median(baseDev, 'omitnan');
gateM = max(gateFloorM, madScale * max(baseMad, 50.0));
gateM = min(gateM, gateCeilM);
fullDev = abs(deltaIn - baseMed);
keepMask = finiteMask & (fullDev <= gateM | ismember(prnVec, protectPrns(:)));
if sum(keepMask) < minSat
    keepMask = finiteMask;
end
deltaOut(~keepMask) = nan;
info.keepMask = keepMask;
info.devM = fullDev;
info.gateM = gateM;
info.rejectNum = sum(finiteMask & ~keepMask);
end

function [trustMask, minSatUse] = localBuildDs5KfSourceTrustMask(sourceMode, prnVec, deltaDetrended, branchSaneNow, settings)
trustMask = false(size(sourceMode));
minSatUse = localGetSettingValue(settings, 'deepShadowDs5KfTrustedMinSat', localGetSettingValue(settings, 'deepShadowKfMinSat', 4));
if isempty(sourceMode)
    return;
end
sourceMode = sourceMode(:);
prnVec = prnVec(:);
deltaDetrended = deltaDetrended(:);
finiteMask = isfinite(deltaDetrended);
mode2Mask = finiteMask & (sourceMode == 2);
trustMask = mode2Mask;
mode2MinSat = localGetSettingValue(settings, 'deepShadowDs5KfMode2MinSat', 2);
allowMode3Prns = localGetSettingValue(settings, 'deepShadowDs5KfAllowMode3Prns', [27 3]);
mode3MaxDetM = localGetSettingValue(settings, 'deepShadowDs5KfAllowMode3MaxDetM', 120.0);
requireBranchSane = localGetSettingValue(settings, 'deepShadowDs5KfAllowMode3OnlyWhenBranchSane', 1) ~= 0;
if sum(mode2Mask) >= mode2MinSat
    mode3Mask = finiteMask & (sourceMode == 3) & ismember(prnVec, allowMode3Prns(:)) & ...
        (abs(deltaDetrended) <= mode3MaxDetM);
    if requireBranchSane && ~branchSaneNow
        mode3Mask(:) = false;
    end
    trustMask = trustMask | mode3Mask;
end
end

function tf = localUseDs5KfSourceGate(settings, currMeasNr)
tf = false;
if ~isfield(settings, 'deepShadowDs5KfSourceGateEnable') || ~settings.deepShadowDs5KfSourceGateEnable
    return;
end
if ~localIsDs5Scenario(settings)
    return;
end
startSec = localGetSettingValue(settings, 'deepShadowDs5KfSourceGateStartSec', 118.0);
navSolPeriodMs = localGetSettingValue(settings, 'navSolPeriod', 500.0);
timeSec = (double(currMeasNr) - 1.0) * navSolPeriodMs / 1000.0;
if timeSec < startSec
    return;
end
tf = true;
end

function refPos = localBuildDs5AuthorityRefPos(lastPos, lastVel, lastEpoch, trustedValid, trustedPos, trustedVel, trustedTimeSec, epochElapsedSec, currMeasNr, settings)
refPos = nan(3,1);
if all(isfinite(lastPos))
    refPos = lastPos(:);
    if localGetSettingValue(settings, 'deepShadowClosedLoopUseVelocityCoast', 1) && all(isfinite(lastVel)) && isfinite(lastEpoch)
        dtSec = (currMeasNr - lastEpoch) * localGetSettingValue(settings, 'navSolPeriod', 500.0) / 1000.0;
        if isfinite(dtSec) && dtSec >= 0
            refPos = refPos + dtSec * lastVel(:);
        end
    end
elseif trustedValid && all(isfinite(trustedPos))
    refPos = trustedPos(:);
    if all(isfinite(trustedVel)) && isfinite(trustedTimeSec)
        dtSec = epochElapsedSec - trustedTimeSec;
        if isfinite(dtSec) && dtSec >= 0
            refPos = refPos + dtSec * trustedVel(:);
        end
    end
end
end

function [eligible, refDiffM] = localEvaluateDs5Authority(branchSaneNow, trustedSatNum, mode2SatNum, mode23SatNum, currPos, refPos, settings)
eligible = true;
refDiffM = nan;
if localGetSettingValue(settings, 'deepShadowDs5AuthorityRequireBranchSane', 1) ~= 0 && ~branchSaneNow
    eligible = false;
end
if trustedSatNum < localGetSettingValue(settings, 'deepShadowDs5AuthorityTrustedMinSat', 3)
    eligible = false;
end
if mode2SatNum < localGetSettingValue(settings, 'deepShadowDs5AuthorityMode2MinSat', 2)
    eligible = false;
end
if mode23SatNum < localGetSettingValue(settings, 'deepShadowDs5AuthorityMode23MinSat', 3)
    eligible = false;
end
if localGetSettingValue(settings, 'deepShadowDs5AuthorityRequireClosedLoopRef', 1) ~= 0
    if ~all(isfinite(refPos))
        eligible = false;
    else
        refDiffM = norm(currPos(:) - refPos(:));
        if ~(isfinite(refDiffM) && refDiffM <= localGetSettingValue(settings, 'deepShadowDs5AuthorityRefPosMaxDiffM', 1200.0))
            eligible = false;
        end
    end
end
end

function tf = localShouldHardRevokeDs5Authority(isAuthorized, eligibleNow, refDiffM, branchSaneNow, settings)
tf = false;
if ~isAuthorized || eligibleNow
    return;
end
if localGetSettingValue(settings, 'deepShadowDs5AuthorityHardRevokeEnable', 1) == 0
    return;
end
if localGetSettingValue(settings, 'deepShadowDs5AuthorityRequireBranchSane', 1) ~= 0 && ~branchSaneNow
    tf = true;
    return;
end
refGateM = localGetSettingValue(settings, 'deepShadowDs5AuthorityHardRevokeRefDiffM', ...
    localGetSettingValue(settings, 'deepShadowDs5AuthorityRefPosMaxDiffM', 1200.0));
tf = ~isfinite(refDiffM) || (isfinite(refGateM) && isfinite(refDiffM) && refDiffM > refGateM);
end

function tf = localShouldFastDeauthorizeDs5Authority(isAuthorized, eligibleNow, badCnt, settings)
tf = false;
if ~isAuthorized || eligibleNow
    return;
end
maxBadEpochs = localGetSettingValue(settings, 'deepShadowDs5AuthorityFastDeauthBadEpochs', 2);
tf = badCnt >= max(1, maxBadEpochs);
end

function tf = localUseDs5BaselineTrustedRef(settings, currMeasNr)
tf = false;
if ~localIsDs5Scenario(settings)
    return;
end
tf = localGetSettingValue(settings, 'deepShadowDs5BaselineTrustedRefEnable', 0) ~= 0;
if ~tf
    return;
end
maxSec = localGetSettingValue(settings, 'deepShadowDs5BaselineTrustedRefMaxSec', inf);
if isfinite(maxSec)
    navSolPeriodMs = localGetSettingValue(settings, 'navSolPeriod', 500.0);
    timeSec = (double(currMeasNr) - 1.0) * navSolPeriodMs / 1000.0;
    tf = timeSec <= maxSec;
end
end

function deltaRef = localRebaseShadowNavDelta(deltaCurr, satPos, currRefPos, newRefPos)
deltaRef = deltaCurr;
if isempty(deltaCurr) || ~all(isfinite(currRefPos)) || ~all(isfinite(newRefPos))
    return;
end
currRho = sqrt(sum((satPos - currRefPos(:)).^2, 1));
newRho = sqrt(sum((satPos - newRefPos(:)).^2, 1));
deltaRef = deltaCurr(:).' + currRho(:).' - newRho(:).';
deltaRef = reshape(deltaRef, size(deltaCurr));
end

function tf = localUseDs5FinalOutput(settings, currMeasNr)
tf = false;
if ~isfield(settings, 'deepShadowDs5FinalOutputEnable') || ~settings.deepShadowDs5FinalOutputEnable
    return;
end
tf = true;
end

function [outPos, outVel, validNow, lastPos, lastVel, lastEpoch, holdAge] = ...
    localGetBaselineOutput(navSolutions, currMeasNr, lastPos, lastVel, lastEpoch, settings)
outPos = nan(3,1);
outVel = nan(3,1);
validNow = false;
holdAge = inf;
if exist('navSolutions', 'var') && isstruct(navSolutions) && isfield(navSolutions, 'X') && numel(navSolutions.X) >= currMeasNr
    candPos = [navSolutions.X(1, currMeasNr); navSolutions.Y(1, currMeasNr); navSolutions.Z(1, currMeasNr)];
    if all(isfinite(candPos))
        outPos = candPos(:);
        if all(isfield(navSolutions, {'VX','VY','VZ'})) && numel(navSolutions.VX) >= currMeasNr
            candVel = [navSolutions.VX(1, currMeasNr); navSolutions.VY(1, currMeasNr); navSolutions.VZ(1, currMeasNr)];
            if all(isfinite(candVel))
                outVel = candVel(:);
            end
        end
        validNow = true;
        lastPos = outPos;
        if all(isfinite(outVel))
            lastVel = outVel;
        end
        lastEpoch = currMeasNr;
        holdAge = 0;
        return;
    end
end
if all(isfinite(lastPos)) && isfinite(lastEpoch)
    dtSec = (currMeasNr - lastEpoch) * localGetSettingValue(settings, 'navSolPeriod', 500.0) / 1000.0;
    outPos = lastPos(:);
    if all(isfinite(lastVel))
        outPos = outPos + dtSec * lastVel(:);
        outVel = lastVel(:);
    end
    validNow = true;
    holdAge = currMeasNr - lastEpoch;
end
end

function [pos, res, pdop, corrNorm] = localSolveDs5RefObsLocalCorrection(satPos, obs, refPos, clockPriorM, absAnchorPos, settings)
pos = nan(4,1);
res = nan(numel(obs), 1);
pdop = nan;
corrNorm = nan;
if ~all(isfinite(refPos))
    return;
end
obs = obs(:);
refPos = refPos(:);
nSat = numel(obs);
if size(satPos,2) ~= nSat || nSat < 4
    return;
end
posEcef = refPos;
maxIter = max(1, round(localGetSettingValue(settings, 'deepShadowDs5BranchLocalIterMax', 3)));
stepMaxM = localGetSettingValue(settings, 'deepShadowDs5BranchLocalStepMaxM', 250.0);
corrMaxM = localGetSettingValue(settings, 'deepShadowDs5BranchLocalCorrectionMaxM', 1200.0);
rho = sqrt(sum((satPos - posEcef).^2, 1)).';
if localGetSettingValue(settings, 'deepShadowDs5RefObsPositionClockPriorEnable', 1) ~= 0 && isfinite(clockPriorM)
    clockBias = clockPriorM;
else
    clockBias = median(obs - rho, 'omitnan');
end
if ~isfinite(clockBias)
    return;
end
for it = 1:maxIter
    dpos = satPos - posEcef;
    rho = sqrt(sum(dpos.^2, 1)).';
    if ~all(isfinite(rho)) || any(rho <= 1)
        return;
    end
    H = [-(dpos(1,:).')./rho  -(dpos(2,:).')./rho  -(dpos(3,:).')./rho  ones(nSat,1)];
    y = obs - rho - clockBias;
    if any(~isfinite(H(:))) || any(~isfinite(y))
        return;
    end
    Haug = H;
    yaug = y;
    if localGetSettingValue(settings, 'deepShadowDs5RefObsPositionAbsAnchorPriorEnable', 1) ~= 0 && all(isfinite(absAnchorPos))
        absAnchorSoftMaxM = localGetSettingValue(settings, 'deepShadowDs5RefObsPositionAbsAnchorSoftMaxDiffM', 900.0);
        absAnchorDiffM = norm(posEcef - absAnchorPos(:));
        priorSigmaM = localGetSettingValue(settings, 'deepShadowDs5RefObsPositionAbsAnchorPriorSigmaM', 900.0);
        if isfinite(absAnchorSoftMaxM) && absAnchorDiffM > absAnchorSoftMaxM
            priorSigmaM = max(priorSigmaM, localGetSettingValue(settings, 'deepShadowDs5RefObsPositionAbsAnchorPriorSoftSigmaM', 1800.0));
        end
        if isfinite(priorSigmaM) && priorSigmaM > 0
            Haug = [Haug; [diag([1 1 1] / priorSigmaM), zeros(3,1)]]; %#ok<AGROW>
            yaug = [yaug; (absAnchorPos(:) - posEcef(:)) / priorSigmaM]; %#ok<AGROW>
        end
    end
    if localGetSettingValue(settings, 'deepShadowDs5RefObsPositionClockPriorEnable', 1) ~= 0 && isfinite(clockPriorM)
        clockPriorSigmaM = localGetSettingValue(settings, 'deepShadowDs5RefObsPositionClockPriorSigmaM', 1200.0);
        if isfinite(clockPriorSigmaM) && clockPriorSigmaM > 0
            Haug = [Haug; 0 0 0 1 / clockPriorSigmaM]; %#ok<AGROW>
            yaug = [yaug; (clockPriorM - clockBias) / clockPriorSigmaM]; %#ok<AGROW>
        end
    end
    step = pinv(Haug) * yaug;
    if ~all(isfinite(step))
        return;
    end
    stepPos = step(1:3);
    stepNorm = norm(stepPos);
    if isfinite(stepMaxM) && stepMaxM > 0 && stepNorm > stepMaxM
        stepPos = stepPos * (stepMaxM / stepNorm);
        stepNorm = stepMaxM;
    end
    newPos = posEcef + stepPos;
    totalCorr = newPos - refPos;
    totalNorm = norm(totalCorr);
    if isfinite(corrMaxM) && corrMaxM > 0 && totalNorm > corrMaxM
        newPos = refPos + totalCorr * (corrMaxM / totalNorm);
        totalCorr = newPos - refPos;
        totalNorm = corrMaxM;
    end
    posEcef = newPos;
    clockBias = clockBias + step(4);
    if stepNorm < 1.0
        break;
    end
end
dpos = satPos - posEcef;
rho = sqrt(sum(dpos.^2, 1)).';
H = [-(dpos(1,:).')./rho  -(dpos(2,:).')./rho  -(dpos(3,:).')./rho  ones(nSat,1)];
res = obs - rho - clockBias;
Q = pinv(H' * H);
if all(isfinite(diag(Q)))
    pdop = sqrt(max(0, trace(Q(1:3,1:3))));
end
pos = [posEcef; clockBias];
corrNorm = norm(posEcef - refPos);
end

function [velOut, ok, satNum, rmsMps, p95Mps, signChoice, sourceId] = localEstimateDs5RecoveredDopplerVelocity(currMeasNr, navResults, posEcef, priorVel, settings)
velOut = nan(3,1);
ok = false;
satNum = 0;
rmsMps = nan;
p95Mps = nan;
signChoice = nan;
sourceId = 0;
if currMeasNr <= 1 || ~all(isfinite(posEcef)) || ~isfield(navResults, 'shadowDs5RefObsUsed')
    return;
end
lambdaL1 = settings.c / 1575.42e6;
minSat = max(4, round(localGetSettingValue(settings, 'deepShadowDs5RecoveredFilterDopplerVelMinSat', 4)));
fields = {'shadowRawTrackCarrCmdHz', 'shadowRawTrackCarrFreqHz', 'shadowRawTrackCarrAidFreqHz'};
used = navResults.shadowDs5RefObsUsed(:, currMeasNr) ~= 0;
satPosAll = [navResults.shadowRawPassSatPosX(:, currMeasNr).'; ...
             navResults.shadowRawPassSatPosY(:, currMeasNr).'; ...
             navResults.shadowRawPassSatPosZ(:, currMeasNr).'];
satVelAll = [navResults.shadowRawPassSatVelX(:, currMeasNr).'; ...
             navResults.shadowRawPassSatVelY(:, currMeasNr).'; ...
             navResults.shadowRawPassSatVelZ(:, currMeasNr).'];
clkRateMps = nan(size(used));
if isfield(navResults, 'shadowRawPassSatClkDrift')
    clkDrift = navResults.shadowRawPassSatClkDrift(:, currMeasNr);
    clkRateMps(isfinite(clkDrift)) = settings.c * clkDrift(isfinite(clkDrift));
end
if isfield(navResults, 'shadowRawPassSatClkCorr') && currMeasNr > 1
    needClk = ~isfinite(clkRateMps);
    dtSec = localGetSettingValue(settings, 'navSolPeriod', 500.0) / 1000.0;
    if isfinite(dtSec) && dtSec > 0 && any(needClk)
        clkNow = navResults.shadowRawPassSatClkCorr(:, currMeasNr);
        clkPrev = navResults.shadowRawPassSatClkCorr(:, currMeasNr - 1);
        clkDiff = settings.c * (clkNow(:) - clkPrev(:)) / dtSec;
        clkRateMps(needClk & isfinite(clkDiff)) = clkDiff(needClk & isfinite(clkDiff));
    end
end
clkRateMps(~isfinite(clkRateMps)) = 0;
bestScore = inf;
bestVel = nan(3,1);
bestSat = 0;
bestRms = nan;
bestP95 = nan;
bestSign = nan;
bestSource = 0;
for ff = 1:numel(fields)
    fieldName = fields{ff};
    if ~isfield(navResults, fieldName)
        continue;
    end
    freqAll = navResults.(fieldName)(:, currMeasNr);
    valid = used(:) & isfinite(freqAll(:)) & all(isfinite(satPosAll), 1).' & all(isfinite(satVelAll), 1).';
    idxAll = find(valid);
    if numel(idxAll) < minSat
        continue;
    end
    maxDrop = min(max(0, round(localGetSettingValue(settings, 'deepShadowDs5RecoveredFilterDopplerVelRobustMaxDropSat', 2))), numel(idxAll) - minSat);
    for dropNum = 0:maxDrop
        if dropNum == 0
            keepSets = true(1, numel(idxAll));
        else
            dropSets = nchoosek(1:numel(idxAll), dropNum);
            keepSets = true(size(dropSets,1), numel(idxAll));
            for kk = 1:size(dropSets,1)
                keepSets(kk, dropSets(kk,:)) = false;
            end
        end
        for rr = 1:size(keepSets,1)
            idx = idxAll(keepSets(rr,:));
            if numel(idx) < minSat
                continue;
            end
            for sgn = [-1 1]
                [velNow, rmsNow, p95Now] = localSolveDs5DopplerVelocitySubset(idx, freqAll, satPosAll, satVelAll, clkRateMps, posEcef, sgn, lambdaL1);
                if ~all(isfinite(velNow)) || ~isfinite(rmsNow)
                    continue;
                end
                priorPenalty = 0;
                if all(isfinite(priorVel))
                    priorPenalty = 0.02 * norm(velNow(:) - priorVel(:));
                end
                score = rmsNow + priorPenalty + 4.0 * dropNum;
                if score < bestScore
                    bestScore = score;
                    bestVel = velNow(:);
                    bestSat = numel(idx);
                    bestRms = rmsNow;
                    bestP95 = p95Now;
                    bestSign = sgn;
                    bestSource = ff;
                end
            end
        end
    end
end
maxRms = localGetSettingValue(settings, 'deepShadowDs5RecoveredFilterDopplerVelMaxResidMps', 85.0);
if all(isfinite(bestVel)) && bestSat >= minSat && (~isfinite(maxRms) || maxRms <= 0 || bestRms <= maxRms)
    velOut = bestVel(:);
    ok = true;
    satNum = bestSat;
    rmsMps = bestRms;
    p95Mps = bestP95;
    signChoice = bestSign;
    sourceId = bestSource;
end
end

function [velOut, rmsMps, p95Mps] = localSolveDs5DopplerVelocitySubset(idx, freqAll, satPosAll, satVelAll, clkRateMps, posEcef, signChoice, lambdaL1)
velOut = nan(3,1);
rmsMps = nan;
p95Mps = nan;
A = nan(numel(idx), 4);
y = nan(numel(idx), 1);
for rr = 1:numel(idx)
    ii = idx(rr);
    satPos = satPosAll(:, ii);
    satVel = satVelAll(:, ii);
    rho = norm(satPos(:) - posEcef(:));
    if ~isfinite(rho) || rho <= 0
        continue;
    end
    los = (satPos(:) - posEcef(:)) / rho;
    obsRate = signChoice * lambdaL1 * freqAll(ii);
    A(rr,:) = [-los(:).', 1];
    y(rr) = obsRate - los(:).' * satVel(:) + clkRateMps(ii);
end
good = all(isfinite(A), 2) & isfinite(y);
if sum(good) < 4
    return;
end
Ag = A(good,:);
yg = y(good);
if rank(Ag) < 4
    return;
end
x = Ag \ yg;
resid = Ag * x - yg;
finiteResid = isfinite(resid);
if sum(finiteResid) < 4
    return;
end
velOut = x(1:3);
rmsMps = sqrt(mean(resid(finiteResid).^2, 'omitnan'));
p95Mps = prctile(abs(resid(finiteResid)), 95);
end
function [deltaDetrended, commonClockM] = localEstimateDs5RefObsCommonClock(deltaUse, prnUse, badScoreByPrn, clockPriorM, settings)
deltaDetrended = deltaUse(:);
commonClockM = nan;
if isempty(deltaUse)
    return;
end
deltaUse = deltaUse(:);
good = isfinite(deltaUse);
if ~any(good)
    return;
end
commonClockEnable = localGetSettingValue(settings, 'deepShadowDs5RefObsPositionCommonClockEnable', 1) ~= 0;
if ~commonClockEnable
    commonClockRawM = median(deltaUse(good), 'omitnan');
    commonClockM = localRebaseDs5CommonClockBranch(commonClockRawM, clockPriorM, settings);
    if isfinite(commonClockRawM)
        deltaDetrended(good) = deltaUse(good) - commonClockRawM;
    end
    return;
end
seedM = median(deltaUse(good), 'omitnan');
priorBlend = max(0, min(1, localGetSettingValue(settings, 'deepShadowDs5RefObsPositionCommonClockPriorBlend', 0.15)));
priorMaxPullM = localGetSettingValue(settings, 'deepShadowDs5RefObsPositionCommonClockPriorMaxPullM', 1500.0);
if isfinite(clockPriorM) && isfinite(seedM) && isfinite(priorBlend) && priorBlend > 0
    priorPullM = clockPriorM - seedM;
    if isfinite(priorMaxPullM) && priorMaxPullM > 0
        priorPullM = max(-priorMaxPullM, min(priorMaxPullM, priorPullM));
    end
    seedM = seedM + priorBlend * priorPullM;
end
dev = abs(deltaUse - seedM);
devFinite = dev(good & isfinite(dev));
madM = median(devFinite, 'omitnan');
gateFloorM = localGetSettingValue(settings, 'deepShadowDs5RefObsPositionCommonClockGateFloorM', 250.0);
madScale = localGetSettingValue(settings, 'deepShadowDs5RefObsPositionCommonClockMadScale', 4.0);
if ~isfinite(madM)
    madM = gateFloorM;
end
gateM = max(gateFloorM, madScale * max(madM, 50.0));
keep = good & isfinite(dev) & dev <= gateM;
if ~isempty(prnUse) && numel(prnUse) == numel(deltaUse) && ~isempty(badScoreByPrn)
    badScoreThresh = localGetSettingValue(settings, 'deepShadowDs5RefObsPositionCommonClockBadScoreThresh', 2.0);
    for ii = 1:numel(prnUse)
        prn = prnUse(ii);
        if ~keep(ii) || ~isfinite(prn) || prn < 1 || prn > numel(badScoreByPrn)
            continue;
        end
        scoreNow = badScoreByPrn(prn);
        if isfinite(scoreNow) && scoreNow >= badScoreThresh
            keep(ii) = false;
        end
    end
end
minSat = max(1, round(localGetSettingValue(settings, 'deepShadowDs5RefObsPositionCommonClockMinSat', 3)));
baseKeep = keep;
if ~isempty(prnUse) && numel(prnUse) == numel(deltaUse)
    excludePrns = localGetSettingValue(settings, 'deepShadowDs5RefObsPositionCommonClockExcludePrns', [21 27 9]);
    fallbackPrns = localGetSettingValue(settings, 'deepShadowDs5RefObsPositionCommonClockFallbackPrns', [9 21 27]);
    if ~isempty(excludePrns)
        baseKeep = keep & ~ismember(prnUse(:), excludePrns(:));
        if sum(baseKeep) < minSat
            for ii = 1:numel(fallbackPrns)
                addMask = keep & prnUse(:) == fallbackPrns(ii);
                baseKeep(addMask) = true;
                if sum(baseKeep) >= minSat
                    break;
                end
            end
        end
    end
end
if sum(baseKeep) < minSat
    baseKeep = good & isfinite(dev) & dev <= max(2.0 * gateM, gateFloorM);
end
if sum(baseKeep) < minSat
    baseKeep = good;
end
commonClockRawM = median(deltaUse(baseKeep), 'omitnan');
if ~isfinite(commonClockRawM)
    commonClockRawM = median(deltaUse(good), 'omitnan');
end
commonClockM = localRebaseDs5CommonClockBranch(commonClockRawM, clockPriorM, settings);
if isfinite(commonClockRawM)
    deltaDetrended(good) = deltaUse(good) - commonClockRawM;
end
end

function clockPriorM = localGetDs5ClockPriorM(navResults, currMeasNr, fallbackClockM)
clockPriorM = nan;
if nargin >= 3 && isfinite(fallbackClockM)
    clockPriorM = fallbackClockM;
    return;
end
fields = {'shadowRecoveredFilterClockM', 'dt'};
for ff = 1:numel(fields)
    fieldName = fields{ff};
    if ~isfield(navResults, fieldName)
        continue;
    end
    vals = navResults.(fieldName);
    if isempty(vals)
        continue;
    end
    vals = vals(:).';
    maxIdx = min(max(1, currMeasNr), numel(vals));
    if isfinite(vals(maxIdx))
        clockPriorM = vals(maxIdx);
        return;
    end
    prevIdx = find(isfinite(vals(1:maxIdx)), 1, 'last');
    if ~isempty(prevIdx)
        clockPriorM = vals(prevIdx);
        return;
    end
end
end

function commonClockM = localRebaseDs5CommonClockBranch(commonClockM, clockPriorM, settings)
if ~isfinite(commonClockM) || ~isfinite(clockPriorM) || ...
        localGetSettingValue(settings, 'deepShadowDs5RefObsPositionCommonClockRebaseEnable', 1) == 0
    return;
end
periodM = localGetSettingValue(settings, 'deepShadowDs5RefObsPositionCommonClockRebasePeriodM', settings.c * 1e-3);
if ~isfinite(periodM) || periodM <= 0
    return;
end
branch = round((commonClockM - clockPriorM) / periodM);
if isfinite(branch)
    commonClockM = commonClockM - branch * periodM;
end
end

function [tf, navResults] = localEvaluateDs5ObservationContract(currMeasNr, navResults, settings, clockPriorM)
if nargin < 4
    clockPriorM = nan;
end
tf = false;
if currMeasNr < 1 || ~isfield(navResults, 'shadowDs5RefObsUsed') || ...
        ~localGetSettingValue(settings, 'deepShadowDs5ObsContractEnable', 1)
    return;
end
numCh = size(navResults.shadowDs5RefObsUsed, 1);
usableDeltaAll = nan(numCh, 1);
residualAll = nan(numCh, 1);
usedSat = 0;
commonClockM = nan;
spreadMedM = nan;
spreadP95M = nan;
trackingPass = false;
commonClockPass = false;
spreadPass = false;
baseSeedPass = false;
baseSeedCnt = 0;
baseSeedFrac = nan;
if currMeasNr > size(navResults.shadowDs5RefObsUsed, 2) || ~localUseDs5ReferenceObsModel(settings, currMeasNr)
    navResults = localStoreDs5ObservationContract(navResults, currMeasNr, tf, usedSat, commonClockM, ...
        spreadMedM, spreadP95M, trackingPass, commonClockPass, spreadPass, baseSeedPass, ...
        baseSeedCnt, baseSeedFrac, usableDeltaAll, residualAll);
    return;
end
used = navResults.shadowDs5RefObsUsed(:, currMeasNr) ~= 0;
if localGetSettingValue(settings, 'deepShadowDs5ObsContractUseRecoveryKeep', 0) ~= 0 && ...
        isfield(navResults, 'shadowDs5RefObsRecoveryKeep') && currMeasNr <= size(navResults.shadowDs5RefObsRecoveryKeep, 2)
    recoveryKeep = logical(navResults.shadowDs5RefObsRecoveryKeep(:, currMeasNr));
    if any(recoveryKeep)
        used = used & recoveryKeep(:);
    end
end
delta = navResults.shadowDs5RefObsDeltaM(:, currMeasNr);
valid = used(:) & isfinite(delta(:));
usedSat = sum(valid);
if usedSat > 0
    validIdx = find(valid);
    seedUseBase = false(numCh, 1);
    if isfield(navResults, 'shadowDs5RefObsSeedUseBase')
        seedUseBase = navResults.shadowDs5RefObsSeedUseBase(:, currMeasNr) ~= 0;
    end
    independentTracking = valid(:);
    if isfield(navResults, 'shadowDs5TrueRefReady') && currMeasNr <= size(navResults.shadowDs5TrueRefReady, 2)
        independentTracking = independentTracking & logical(navResults.shadowDs5TrueRefReady(:, currMeasNr));
    end
    if isfield(navResults, 'shadowDs5RefObsTrackDriven') && currMeasNr <= size(navResults.shadowDs5RefObsTrackDriven, 2)
        independentTracking = independentTracking & logical(navResults.shadowDs5RefObsTrackDriven(:, currMeasNr));
    end
    if isfield(navResults, 'shadowRawTrackCodeRefNcoPullHz') && currMeasNr <= size(navResults.shadowRawTrackCodeRefNcoPullHz, 2)
        independentTracking = independentTracking & isfinite(navResults.shadowRawTrackCodeRefNcoPullHz(:, currMeasNr));
    end
    if isfield(navResults, 'shadowRawTrackPromptIP') && currMeasNr <= size(navResults.shadowRawTrackPromptIP, 2)
        independentTracking = independentTracking & isfinite(navResults.shadowRawTrackPromptIP(:, currMeasNr));
    end
    codeErrForInd = abs(navResults.shadowDs5RefObsCodeErrChips(:, currMeasNr));
    freqErrForInd = abs(navResults.shadowDs5RefObsFreqErrHz(:, currMeasNr));
    codeMaxInd = localGetSettingValue(settings, 'deepShadowDs5ObsContractIndependentCodeErrMaxChips', ...
        localGetSettingValue(settings, 'deepShadowDs5RefObsTrackingCodeP95MaxChips', 1.60));
    freqMaxInd = localGetSettingValue(settings, 'deepShadowDs5ObsContractIndependentFreqErrMaxHz', ...
        localGetSettingValue(settings, 'deepShadowDs5RefObsTrackingFreqP95MaxHz', 350.0));
    independentTracking = independentTracking & isfinite(codeErrForInd(:)) & codeErrForInd(:) <= codeMaxInd & ...
        isfinite(freqErrForInd(:)) & freqErrForInd(:) <= freqMaxInd;
    seedUseBaseEffective = seedUseBase(:) & ~independentTracking(:);
    baseSeedCnt = sum(seedUseBaseEffective(valid));
    baseSeedFrac = baseSeedCnt / max(usedSat, 1);
    prnUse = nan(numel(validIdx), 1);
    if isfield(navResults, 'prnList') && numel(navResults.prnList) >= max(validIdx)
        prnUse = double(navResults.prnList(validIdx));
    end
    if ~isfinite(clockPriorM)
        clockPriorM = localGetDs5ClockPriorM(navResults, currMeasNr, nan);
    end
    [deltaDetrended, commonClockM] = localEstimateDs5RefObsCommonClock(delta(valid), prnUse(:), [], clockPriorM, settings);
    usableDeltaAll(validIdx) = deltaDetrended(:);
    finiteDet = deltaDetrended(isfinite(deltaDetrended));
    if numel(finiteDet) >= max(1, min(usedSat, 2))
        residualUse = deltaDetrended(:) - median(finiteDet, 'omitnan');
        residualAll(validIdx) = residualUse(:);
    end
    minSat = max(4, round(localGetSettingValue(settings, 'deepShadowDs5ObsContractMinSat', 4)));
    if numel(finiteDet) >= minSat
        dev = abs(finiteDet - median(finiteDet, 'omitnan'));
        spreadMedM = median(dev, 'omitnan');
        spreadP95M = prctile(dev, 95);
    end

    codeErr = abs(navResults.shadowDs5RefObsCodeErrChips(:, currMeasNr));
    freqErr = abs(navResults.shadowDs5RefObsFreqErrHz(:, currMeasNr));
    codeValid = valid & isfinite(codeErr);
    freqValid = valid & isfinite(freqErr);
    codeMed = nan;
    codeP95 = nan;
    freqP95 = nan;
    if any(codeValid)
        codeMed = median(codeErr(codeValid), 'omitnan');
        codeP95 = prctile(codeErr(codeValid), 95);
    end
    if any(freqValid)
        freqP95 = prctile(freqErr(freqValid), 95);
    end
    trackingPass = usedSat >= minSat && ...
        isfinite(codeMed) && codeMed <= localGetSettingValue(settings, 'deepShadowDs5RefObsTrackingCodeMedMaxChips', 0.75) && ...
        isfinite(codeP95) && codeP95 <= localGetSettingValue(settings, 'deepShadowDs5RefObsTrackingCodeP95MaxChips', 1.60) && ...
        isfinite(freqP95) && freqP95 <= localGetSettingValue(settings, 'deepShadowDs5RefObsTrackingFreqP95MaxHz', 350.0);
    if ~trackingPass && localGetSettingValue(settings, 'deepShadowDs5ObsContractRobustTrackingEnable', 0) ~= 0
        robustMaxDropSat = max(0, round(localGetSettingValue(settings, 'deepShadowDs5ObsContractRobustTrackingMaxDropSat', ...
            localGetSettingValue(settings, 'deepShadowDs5RefObsRecoveryRobustMaxDropSat', 2))));
        [trackingKeep, ~, trackingCodeMed, trackingCodeP95, trackingFreqP95] = ...
            localSelectDs5RefObsRecoverySubset(codeValid(:), codeErr(:), freqErr(:), minSat, robustMaxDropSat, ...
            localGetSettingValue(settings, 'deepShadowDs5RefObsTrackingCodeMedMaxChips', 0.75), ...
            localGetSettingValue(settings, 'deepShadowDs5RefObsTrackingCodeP95MaxChips', 1.60), ...
            localGetSettingValue(settings, 'deepShadowDs5RefObsTrackingFreqP95MaxHz', 350.0));
        trackingPass = localDs5RefObsRecoveryMetricsPass(trackingKeep, trackingCodeMed, trackingCodeP95, trackingFreqP95, minSat, ...
            localGetSettingValue(settings, 'deepShadowDs5RefObsTrackingCodeMedMaxChips', 0.75), ...
            localGetSettingValue(settings, 'deepShadowDs5RefObsTrackingCodeP95MaxChips', 1.60), ...
            localGetSettingValue(settings, 'deepShadowDs5RefObsTrackingFreqP95MaxHz', 350.0));
    end
    requireCommonClock = localGetSettingValue(settings, 'deepShadowDs5ObsContractRequireCommonClock', 1) ~= 0;
    commonClockPass = (~requireCommonClock) || isfinite(commonClockM);
    spreadMaxM = localGetSettingValue(settings, 'deepShadowDs5ObsContractSpreadP95MaxM', 800.0);
    spreadPass = isfinite(spreadP95M) && (~isfinite(spreadMaxM) || spreadMaxM <= 0 || spreadP95M <= spreadMaxM);
    maxBaseSeedFrac = localGetSettingValue(settings, 'deepShadowDs5ObsContractMaxBaseSeedFrac', 0.75);
    if localGetSettingValue(settings, 'deepShadowDs5ObsContractNoBaselineSeedForRecovery', 0) ~= 0
        maxBaseSeedFrac = 0;
    end
    baseSeedPass = isfinite(baseSeedFrac) && (~isfinite(maxBaseSeedFrac) || maxBaseSeedFrac < 0 || baseSeedFrac <= maxBaseSeedFrac);
    requireTracking = localGetSettingValue(settings, 'deepShadowDs5ObsContractRequireTrackingPass', 1) ~= 0;
    tf = usedSat >= minSat && commonClockPass && spreadPass && baseSeedPass && ...
        ((~requireTracking) || trackingPass);
end
navResults = localStoreDs5ObservationContract(navResults, currMeasNr, tf, usedSat, commonClockM, ...
    spreadMedM, spreadP95M, trackingPass, commonClockPass, spreadPass, baseSeedPass, ...
    baseSeedCnt, baseSeedFrac, usableDeltaAll, residualAll);
end

function navResults = localStoreDs5ObservationContract(navResults, currMeasNr, tf, usedSat, commonClockM, spreadMedM, spreadP95M, trackingPass, commonClockPass, spreadPass, baseSeedPass, baseSeedCnt, baseSeedFrac, usableDeltaAll, residualAll)
navResults.shadowDs5ObsContractPass(1, currMeasNr) = tf;
navResults.shadowDs5ObsContractUsedSatNum(1, currMeasNr) = usedSat;
navResults.shadowDs5ObsContractCommonClockM(1, currMeasNr) = commonClockM;
navResults.shadowDs5ObsContractSpreadMedM(1, currMeasNr) = spreadMedM;
navResults.shadowDs5ObsContractSpreadP95M(1, currMeasNr) = spreadP95M;
navResults.shadowDs5ObsContractTrackingPass(1, currMeasNr) = trackingPass;
navResults.shadowDs5ObsContractCommonClockPass(1, currMeasNr) = commonClockPass;
navResults.shadowDs5ObsContractSpreadPass(1, currMeasNr) = spreadPass;
navResults.shadowDs5ObsContractBaseSeedPass(1, currMeasNr) = baseSeedPass;
navResults.shadowDs5ObsContractBaseSeedSatNum(1, currMeasNr) = baseSeedCnt;
navResults.shadowDs5ObsContractBaseSeedFrac(1, currMeasNr) = baseSeedFrac;
navResults.shadowDs5ObsContractUsableDeltaM(:, currMeasNr) = usableDeltaAll(:);
navResults.shadowDs5ObsContractResidualM(:, currMeasNr) = residualAll(:);
end

function [ok, outPos, outClockM, outClockResidualM, outCommonClockM, postfitRmsM, pdop, corrM, usedSat, jumpM, anchorDiffM, speedMps, accelMps2, dynGatePass, predVelDiffMps, predSupportPass, jumpGatePass, speedGatePass, accelGatePass, dynGateMode, rebasedDeltaAll, robustKeepAll, residualAll, deltaSpreadMedM, deltaSpreadP95M, deltaConsistencyPass, absAnchorDiffM, absAnchorSoftPass, absAnchorGatePass, badScoreByPrn, badScoreAll, rawAnchorLiftUsed, rawAnchorLiftSpreadP95M, rawAnchorLiftAmbigChipsAll] = ...
    localBuildDs5RefObsRecoveredPosition(currMeasNr, navResults, currRefPos, anchorPos, anchorVel, absAnchorPos, clockPriorM, prevAcceptedPos, prevAcceptedEpoch, prevPrevAcceptedPos, prevPrevAcceptedEpoch, prevDeltaConsistencyConfirmCnt, badScoreByPrn, settings)
ok = false;
outPos = anchorPos(:);
outClockM = nan;
outClockResidualM = nan;
outCommonClockM = nan;
postfitRmsM = nan;
pdop = nan;
corrM = nan;
jumpM = nan;
anchorDiffM = nan;
speedMps = nan;
accelMps2 = nan;
dynGatePass = true;
predVelDiffMps = nan;
predSupportPass = false;
jumpGatePass = false;
speedGatePass = false;
accelGatePass = false;
dynGateMode = 0;
usedSat = 0;
numCh = size(navResults.shadowDs5RefObsUsed, 1);
rebasedDeltaAll = nan(numCh, 1);
robustKeepAll = false(numCh, 1);
residualAll = nan(numCh, 1);
badScoreAll = nan(numCh, 1);
rawAnchorLiftUsed = false;
rawAnchorLiftSpreadP95M = nan;
rawAnchorLiftAmbigChipsAll = nan(numCh, 1);
deltaSpreadMedM = nan;
deltaSpreadP95M = nan;
deltaConsistencyPass = false;
absAnchorDiffM = nan;
absAnchorSoftPass = true;
absAnchorGatePass = true;
if ~all(isfinite(currRefPos)) || ~all(isfinite(anchorPos))
    return;
end
if localGetSettingValue(settings, 'deepShadowDs5ObsContractEnable', 1) ~= 0 && ...
        (~isfield(navResults, 'shadowDs5ObsContractPass') || currMeasNr > numel(navResults.shadowDs5ObsContractPass) || ...
         ~logical(navResults.shadowDs5ObsContractPass(1, currMeasNr)))
    return;
end
used = navResults.shadowDs5RefObsUsed(:, currMeasNr) ~= 0;
delta = navResults.shadowDs5RefObsDeltaM(:, currMeasNr);
satPos = [navResults.shadowRawPassSatPosX(:, currMeasNr).'; ...
          navResults.shadowRawPassSatPosY(:, currMeasNr).'; ...
          navResults.shadowRawPassSatPosZ(:, currMeasNr).'];
valid = used(:).' & isfinite(delta(:).') & all(isfinite(satPos), 1);
usedSat = sum(valid);
minSat = max(4, round(localGetSettingValue(settings, 'deepShadowDs5RefObsPositionMinSat', 4)));
if usedSat < minSat
    return;
end
satUseAll = satPos(:, valid);
deltaUseAll = localRebaseShadowNavDelta(delta(valid), satUseAll, currRefPos(:), anchorPos(:));
validIdx = find(valid);
prnAll = nan(size(validIdx));
if isfield(navResults, 'prnList') && numel(navResults.prnList) >= max(validIdx)
    prnAll = double(navResults.prnList(validIdx));
end
if localGetSettingValue(settings, 'deepShadowDs5RefObsPositionRawAnchorLiftEnable', 1) ~= 0
    rawPAll = nan(numCh, 1);
    satClkAll = nan(numCh, 1);
    if isfield(navResults, 'shadowRawPassRawP') && currMeasNr <= size(navResults.shadowRawPassRawP, 2)
        rawPAll = navResults.shadowRawPassRawP(:, currMeasNr);
    end
    if isfield(navResults, 'shadowRawPassSatClkCorr') && currMeasNr <= size(navResults.shadowRawPassSatClkCorr, 2)
        satClkAll = navResults.shadowRawPassSatClkCorr(:, currMeasNr);
    end
    rawValid = valid(:) & isfinite(rawPAll(:)) & isfinite(satClkAll(:));
    if sum(rawValid) >= minSat
        satRaw = satPos(:, rawValid);
        rhoRaw = sqrt(sum((satRaw - anchorPos(:)).^2, 1)).';
        rawDelta = rawPAll(rawValid) + settings.c * satClkAll(rawValid) - rhoRaw;
        [rawDeltaLift, rawAmbigChips] = localLiftDs5RawAnchorDelta(rawDelta, rawValid, navResults, currMeasNr, settings);
        rawAnchorLiftAmbigChipsAll(rawValid) = rawAmbigChips(:);
        rawLiftFinite = rawDeltaLift(isfinite(rawDeltaLift));
        if numel(rawLiftFinite) >= minSat
            rawLiftDev = abs(rawLiftFinite - median(rawLiftFinite, 'omitnan'));
            rawAnchorLiftSpreadP95M = prctile(rawLiftDev, 95);
        end
        rawLiftMaxP95M = localGetSettingValue(settings, 'deepShadowDs5RefObsPositionRawAnchorLiftMaxSpreadP95M', ...
            localGetSettingValue(settings, 'deepShadowDs5RefObsPositionDeltaSpreadP95MaxM', 800.0));
        if isfinite(rawAnchorLiftSpreadP95M) && (~isfinite(rawLiftMaxP95M) || rawLiftMaxP95M <= 0 || rawAnchorLiftSpreadP95M <= rawLiftMaxP95M)
            rawIdx = find(rawValid);
            rawKeep = ismember(validIdx, rawIdx);
            if sum(rawKeep) >= minSat
                [~, locInRaw] = ismember(validIdx(rawKeep), rawIdx);
                deltaUseAll(rawKeep) = rawDeltaLift(locInRaw);
                rawAnchorLiftUsed = true;
            end
        end
    end
end
rebasedDeltaAll(validIdx) = deltaUseAll(:);
[deltaUseAllDetrended, commonClockGateM] = localEstimateDs5RefObsCommonClock(deltaUseAll(:), prnAll(:), badScoreByPrn, clockPriorM, settings);
if isfinite(commonClockGateM) && ~isfinite(outCommonClockM)
    outCommonClockM = commonClockGateM;
end
deltaFinite = deltaUseAllDetrended(isfinite(deltaUseAllDetrended));
if numel(deltaFinite) >= minSat
    deltaDev = abs(deltaFinite - median(deltaFinite, 'omitnan'));
    deltaSpreadMedM = median(deltaDev, 'omitnan');
    deltaSpreadP95M = prctile(deltaDev, 95);
end
consistencyEnable = localGetSettingValue(settings, 'deepShadowDs5RefObsPositionDeltaConsistencyEnable', 1) ~= 0;
if consistencyEnable
    spreadMaxM = localGetSettingValue(settings, 'deepShadowDs5RefObsPositionDeltaSpreadP95MaxM', 800.0);
    confirmEpochs = max(1, round(localGetSettingValue(settings, 'deepShadowDs5RefObsPositionDeltaConsistencyConfirmEpochs', 2)));
    deltaConsistencyPass = isfinite(deltaSpreadP95M) && deltaSpreadP95M <= spreadMaxM;
    if ~deltaConsistencyPass || (prevDeltaConsistencyConfirmCnt + 1) < confirmEpochs
        return;
    end
else
    deltaConsistencyPass = true;
end
[pos, res, pdop, corrM, usedSat, keepMask, commonClockM] = localSolveDs5RefObsBestSubset(satUseAll, deltaUseAll, anchorPos(:), absAnchorPos(:), clockPriorM, prnAll(:), badScoreByPrn, minSat, settings);
if numel(keepMask) == numel(validIdx)
    robustKeepAll(validIdx(keepMask)) = true;
    residualAll(validIdx(keepMask)) = res(:);
end
if numel(prnAll) == numel(validIdx)
    [badScoreByPrn, badScoreUsed] = localUpdateDs5RefObsBadScores(prnAll(:), keepMask(:), res(:), badScoreByPrn, settings);
    badScoreAll(validIdx) = badScoreUsed(:);
end
if ~all(isfinite(pos(1:4))) || ~isfinite(pdop) || ~isfinite(corrM)
    return;
end
outClockResidualM = pos(4);
outCommonClockM = commonClockM;
if isfinite(outCommonClockM) && isfinite(outClockResidualM)
    outClockM = outCommonClockM + outClockResidualM;
elseif isfinite(outClockResidualM)
    outClockM = outClockResidualM;
end
postfitRmsM = sqrt(mean(res(isfinite(res)).^2, 'omitnan'));
if ~isfinite(postfitRmsM)
    return;
end
anchorDiffM = norm(pos(1:3) - anchorPos(:));
if localGetSettingValue(settings, 'deepShadowDs5RefObsPositionAbsAnchorGateEnable', 1) ~= 0 && all(isfinite(absAnchorPos))
    absAnchorDiffM = norm(pos(1:3) - absAnchorPos(:));
    absAnchorSoftMaxM = localGetSettingValue(settings, 'deepShadowDs5RefObsPositionAbsAnchorSoftMaxDiffM', 900.0);
    absAnchorMaxM = localGetSettingValue(settings, 'deepShadowDs5RefObsPositionAbsAnchorMaxDiffM', 1800.0);
    absAnchorSoftPass = ~isfinite(absAnchorSoftMaxM) || absAnchorSoftMaxM <= 0 || (~isfinite(absAnchorDiffM)) || absAnchorDiffM <= absAnchorSoftMaxM;
    absAnchorGatePass = ~isfinite(absAnchorMaxM) || absAnchorMaxM <= 0 || (~isfinite(absAnchorDiffM)) || absAnchorDiffM <= absAnchorMaxM;
end
predSupportAnchorMax = localGetSettingValue(settings, 'deepShadowDs5RefObsPositionPredSupportMaxM', 140.0);
predSupportPostfitMax = localGetSettingValue(settings, 'deepShadowDs5RefObsPositionPredSupportPostfitMaxM', 180.0);
predSupportVelDiffMax = localGetSettingValue(settings, 'deepShadowDs5RefObsPositionPredVelDiffMaxMps', 90.0);
if all(isfinite(prevAcceptedPos)) && isfinite(prevAcceptedEpoch)
    jumpAge = currMeasNr - prevAcceptedEpoch;
    if isfinite(jumpAge) && jumpAge > 0 && jumpAge <= localGetSettingValue(settings, 'deepShadowDs5RefObsPositionJumpMaxAgeEpochs', 4)
        jumpM = norm(pos(1:3) - prevAcceptedPos(:));
    end
end
dynGateEnable = localGetSettingValue(settings, 'deepShadowDs5RefObsPositionSpeedGateEnable', 1) ~= 0;
if dynGateEnable && all(isfinite(prevAcceptedPos)) && isfinite(prevAcceptedEpoch)
    dtSec = (currMeasNr - prevAcceptedEpoch) * localGetSettingValue(settings, 'navSolPeriod', 500.0) / 1000.0;
    dynAgeMax = localGetSettingValue(settings, 'deepShadowDs5RefObsPositionDynGateMaxAgeEpochs', 4);
    if isfinite(dtSec) && dtSec > 0 && (currMeasNr - prevAcceptedEpoch) <= dynAgeMax
        currVel = (pos(1:3) - prevAcceptedPos(:)) / dtSec;
        speedMps = norm(currVel);
        speedMax = localGetSettingValue(settings, 'deepShadowDs5RefObsPositionSpeedMaxMps', 140.0);
        speedGatePass = ~isfinite(speedMax) || speedMax <= 0 || speedMps <= speedMax;
        if all(isfinite(prevPrevAcceptedPos)) && isfinite(prevPrevAcceptedEpoch)
            prevDtSec = (prevAcceptedEpoch - prevPrevAcceptedEpoch) * localGetSettingValue(settings, 'navSolPeriod', 500.0) / 1000.0;
            if isfinite(prevDtSec) && prevDtSec > 0 && (prevAcceptedEpoch - prevPrevAcceptedEpoch) <= dynAgeMax
                prevVel = (prevAcceptedPos(:) - prevPrevAcceptedPos(:)) / prevDtSec;
                accelMps2 = norm(currVel - prevVel) / max(dtSec, eps);
                accelMax = localGetSettingValue(settings, 'deepShadowDs5RefObsPositionAccelMaxMps2', 180.0);
                accelGatePass = ~isfinite(accelMax) || accelMax <= 0 || accelMps2 <= accelMax;
            else
                accelGatePass = true;
            end
        else
            accelGatePass = true;
        end
        if all(isfinite(anchorVel))
            predVelDiffMps = norm(currVel - anchorVel(:));
        end
    else
        speedGatePass = true;
        accelGatePass = true;
    end
else
    speedGatePass = true;
    accelGatePass = true;
end
postfitMax = localGetSettingValue(settings, 'deepShadowDs5RefObsPositionPostfitMaxM', 220.0);
pdopMax = localGetSettingValue(settings, 'deepShadowDs5RefObsPositionPdopMax', 20.0);
corrMax = localGetSettingValue(settings, 'deepShadowDs5RefObsPositionCorrMaxM', 260.0);
anchorDiffMax = localGetSettingValue(settings, 'deepShadowDs5RefObsPositionAnchorDiffMaxM', corrMax);
jumpMax = localGetSettingValue(settings, 'deepShadowDs5RefObsPositionJumpMaxM', 220.0);
jumpGatePass = ~isfinite(jumpM) || ~isfinite(jumpMax) || jumpM <= jumpMax;
predSupportPass = anchorDiffM <= predSupportAnchorMax && postfitRmsM <= predSupportPostfitMax;
if predSupportPass && isfinite(predVelDiffMps)
    predSupportPass = predVelDiffMps <= predSupportVelDiffMax;
end
if predSupportPass && localGetSettingValue(settings, 'deepShadowDs5RefObsPositionRelaxWhenPredSupport', 1) ~= 0
    jumpRelaxedMax = localGetSettingValue(settings, 'deepShadowDs5RefObsPositionJumpRelaxedMaxM', max(jumpMax, 320.0));
    speedRelaxedMax = localGetSettingValue(settings, 'deepShadowDs5RefObsPositionSpeedRelaxedMaxMps', max(localGetSettingValue(settings, 'deepShadowDs5RefObsPositionSpeedMaxMps', 140.0), 260.0));
    accelRelaxedMax = localGetSettingValue(settings, 'deepShadowDs5RefObsPositionAccelRelaxedMaxMps2', max(localGetSettingValue(settings, 'deepShadowDs5RefObsPositionAccelMaxMps2', 180.0), 320.0));
    jumpGatePass = jumpGatePass || ~isfinite(jumpM) || ~isfinite(jumpRelaxedMax) || jumpM <= jumpRelaxedMax;
    speedGatePass = speedGatePass || ~isfinite(speedMps) || ~isfinite(speedRelaxedMax) || speedMps <= speedRelaxedMax;
    accelGatePass = accelGatePass || ~isfinite(accelMps2) || ~isfinite(accelRelaxedMax) || accelMps2 <= accelRelaxedMax;
    dynGateMode = 2;
else
    dynGateMode = 1;
end
if localGetSettingValue(settings, 'deepShadowDs5RefObsPositionAdaptiveDynGateEnable', 1) ~= 0
    adaptiveDeltaMax = localGetSettingValue(settings, 'deepShadowDs5RefObsPositionAdaptiveDeltaSpreadP95MaxM', 400.0);
    adaptivePostfitMax = localGetSettingValue(settings, 'deepShadowDs5RefObsPositionAdaptivePostfitMaxM', postfitMax);
    adaptiveCorrMax = localGetSettingValue(settings, 'deepShadowDs5RefObsPositionAdaptiveCorrMaxM', corrMax);
    adaptiveAnchorMax = localGetSettingValue(settings, 'deepShadowDs5RefObsPositionAdaptiveAnchorDiffMaxM', anchorDiffMax);
    adaptivePredVelMax = localGetSettingValue(settings, 'deepShadowDs5RefObsPositionAdaptivePredVelDiffMaxMps', 220.0);
    adaptiveQualityPass = deltaConsistencyPass && usedSat >= minSat && ...
        isfinite(deltaSpreadP95M) && deltaSpreadP95M <= adaptiveDeltaMax && ...
        isfinite(postfitRmsM) && postfitRmsM <= adaptivePostfitMax && ...
        isfinite(corrM) && corrM <= adaptiveCorrMax && ...
        isfinite(anchorDiffM) && anchorDiffM <= adaptiveAnchorMax;
    if adaptiveQualityPass && isfinite(predVelDiffMps) && isfinite(adaptivePredVelMax) && adaptivePredVelMax > 0
        adaptiveQualityPass = predVelDiffMps <= adaptivePredVelMax;
    end
    if adaptiveQualityPass
        adaptiveJumpMax = localGetSettingValue(settings, 'deepShadowDs5RefObsPositionAdaptiveJumpMaxM', 450.0);
        adaptiveSpeedMax = localGetSettingValue(settings, 'deepShadowDs5RefObsPositionAdaptiveSpeedMaxMps', 450.0);
        adaptiveAccelMax = localGetSettingValue(settings, 'deepShadowDs5RefObsPositionAdaptiveAccelMaxMps2', 700.0);
        jumpGatePass = jumpGatePass || ~isfinite(jumpM) || ~isfinite(adaptiveJumpMax) || adaptiveJumpMax <= 0 || jumpM <= adaptiveJumpMax;
        speedGatePass = speedGatePass || ~isfinite(speedMps) || ~isfinite(adaptiveSpeedMax) || adaptiveSpeedMax <= 0 || speedMps <= adaptiveSpeedMax;
        accelGatePass = accelGatePass || ~isfinite(accelMps2) || ~isfinite(adaptiveAccelMax) || adaptiveAccelMax <= 0 || accelMps2 <= adaptiveAccelMax;
        dynGateMode = 3;
    end
end
dynGatePass = jumpGatePass && speedGatePass && accelGatePass;
useAbsAnchorHardGate = localGetSettingValue(settings, 'deepShadowDs5RefObsPositionAbsAnchorUseHardGate', 0) ~= 0;
absAnchorAcceptPass = ~useAbsAnchorHardGate || absAnchorGatePass;
if postfitRmsM <= postfitMax && pdop <= pdopMax && corrM <= corrMax && ...
        anchorDiffM <= anchorDiffMax && absAnchorAcceptPass && jumpGatePass && dynGatePass
    ok = true;
    outPos = pos(1:3);
end
end
function [bestPos, bestRes, bestPdop, bestCorrM, bestUsedSat, bestKeepMask, bestCommonClockM] = localSolveDs5RefObsBestSubset(satUseAll, deltaUseAll, anchorPos, absAnchorPos, clockPriorM, prnUseAll, badScoreByPrn, minSat, settings)
bestPos = nan(4,1);
bestRes = nan(numel(deltaUseAll), 1);
bestPdop = nan;
bestCorrM = nan;
bestUsedSat = 0;
nSat = size(satUseAll, 2);
bestKeepMask = false(1, nSat);
bestCommonClockM = nan;
if nSat < minSat
    return;
end
maxDrop = 0;
if localGetSettingValue(settings, 'deepShadowDs5RefObsPositionRobustSubsetEnable', 1) ~= 0
    maxDrop = min(max(0, round(localGetSettingValue(settings, 'deepShadowDs5RefObsPositionRobustMaxDropSat', 2))), nSat - minSat);
end
bestScore = inf;
hardDropEnable = localGetSettingValue(settings, 'deepShadowDs5RefObsPositionRobustHardDropEnable', 1) ~= 0;
hardDropPrns = localGetSettingValue(settings, 'deepShadowDs5RefObsPositionRobustHardDropPrns', [21 27]);
hardDropMinSatNum = max(minSat + 1, round(localGetSettingValue(settings, 'deepShadowDs5RefObsPositionRobustHardDropMinSatNum', 5)));
isHardDropTargetAll = false(1, nSat);
if numel(prnUseAll) == nSat && ~isempty(hardDropPrns)
    isHardDropTargetAll = ismember(prnUseAll(:).', hardDropPrns(:).');
end
for dropNum = 0:maxDrop
    if dropNum == 0
        keepSets = true(1, nSat);
    else
        dropSets = nchoosek(1:nSat, dropNum);
        keepSets = true(size(dropSets,1), nSat);
        for rr = 1:size(dropSets,1)
            keepSets(rr, dropSets(rr,:)) = false;
        end
    end
    for rr = 1:size(keepSets,1)
        keep = keepSets(rr,:);
        if sum(keep) < minSat
            continue;
        end
        if hardDropEnable && nSat >= hardDropMinSatNum && any(isHardDropTargetAll)
            keepTargetNum = sum(keep & isHardDropTargetAll);
            keepNonTargetNum = sum(keep & ~isHardDropTargetAll);
            minTargetNeeded = max(0, minSat - keepNonTargetNum);
            excessTargetKept = max(0, keepTargetNum - minTargetNeeded);
            if excessTargetKept > 0
                continue;
            end
        end
        satUse = satUseAll(:, keep);
        deltaUse = deltaUseAll(keep);
        prnUse = prnUseAll(keep);
        [deltaUseDetrended, commonClockM] = localEstimateDs5RefObsCommonClock(deltaUse(:), prnUse(:), badScoreByPrn, clockPriorM, settings);
        rhoAnchor = sqrt(sum((satUse - anchorPos(:)).^2, 1));
        obs = rhoAnchor(:).' + deltaUseDetrended(:).';
        clockResidualPriorM = nan;
        if isfinite(clockPriorM) && isfinite(commonClockM)
            clockResidualPriorM = clockPriorM - commonClockM;
        end
        [pos, res, pdop, corrM] = localSolveDs5RefObsLocalCorrection(satUse, obs, anchorPos(:), clockResidualPriorM, absAnchorPos(:), settings);
        if ~all(isfinite(pos(1:4))) || ~isfinite(pdop) || ~isfinite(corrM)
            continue;
        end
        rmsM = sqrt(mean(res(isfinite(res)).^2, 'omitnan'));
        if ~isfinite(rmsM)
            continue;
        end
        dropPenalty = localGetSettingValue(settings, 'deepShadowDs5RefObsPositionRobustDropPenaltyM', 20.0) * dropNum;
        score = rmsM + 0.05 * corrM + dropPenalty;
        if all(isfinite(absAnchorPos))
            absAnchorDiffM = norm(pos(1:3) - absAnchorPos(:));
            absAnchorSoftMaxM = localGetSettingValue(settings, 'deepShadowDs5RefObsPositionAbsAnchorSoftMaxDiffM', 900.0);
            absAnchorHardMaxM = localGetSettingValue(settings, 'deepShadowDs5RefObsPositionAbsAnchorMaxDiffM', 1800.0);
            absAnchorScoreWeightM = localGetSettingValue(settings, 'deepShadowDs5RefObsPositionAbsAnchorScoreWeightM', 140.0);
            if isfinite(absAnchorDiffM) && isfinite(absAnchorSoftMaxM) && isfinite(absAnchorHardMaxM) && absAnchorHardMaxM > absAnchorSoftMaxM && absAnchorDiffM > absAnchorSoftMaxM
                score = score + absAnchorScoreWeightM * min(1, max(0, (absAnchorDiffM - absAnchorSoftMaxM) / max(absAnchorHardMaxM - absAnchorSoftMaxM, eps)));
            end
        end
        clockSoftMaxM = localGetSettingValue(settings, 'deepShadowDs5RefObsPositionClockSoftMaxM', 1200.0);
        clockHardMaxM = localGetSettingValue(settings, 'deepShadowDs5RefObsPositionClockHardMaxM', 3500.0);
        clockScoreWeightM = localGetSettingValue(settings, 'deepShadowDs5RefObsPositionClockScoreWeightM', 100.0);
        clockAbsM = abs(pos(4));
        if isfinite(clockAbsM) && isfinite(clockSoftMaxM) && isfinite(clockHardMaxM) && clockHardMaxM > clockSoftMaxM && clockAbsM > clockSoftMaxM
            score = score + clockScoreWeightM * min(1, max(0, (clockAbsM - clockSoftMaxM) / max(clockHardMaxM - clockSoftMaxM, eps)));
        end
        if numel(prnUseAll) == nSat && ~isempty(badScoreByPrn)
            badPrns = localGetSettingValue(settings, 'deepShadowDs5RefObsPositionRobustBadPrns', [27 9]);
            keepPrn = prnUseAll(keep);
            keepPrn = keepPrn(isfinite(keepPrn) & keepPrn >= 1 & keepPrn <= numel(badScoreByPrn));
            keepBadPenaltyM = localGetSettingValue(settings, 'deepShadowDs5RefObsPositionRobustBadKeepPenaltyM', 80.0);
            targetKeepPenaltyM = localGetSettingValue(settings, 'deepShadowDs5RefObsPositionRobustTargetKeepPenaltyM', 18.0);
            if ~isempty(keepPrn)
                score = score + keepBadPenaltyM * sum(badScoreByPrn(keepPrn)) / max(sum(keep), 1);
                if ~isempty(badPrns)
                    score = score + targetKeepPenaltyM * sum(ismember(keepPrn(:), badPrns(:)));
                end
            end
        end
        if hardDropEnable && nSat >= hardDropMinSatNum && any(isHardDropTargetAll) && numel(prnUseAll) == nSat
            keepPrnAll = prnUseAll(keep);
            keepPrnAll = keepPrnAll(isfinite(keepPrnAll));
            if ~isempty(keepPrnAll)
                score = score + localGetSettingValue(settings, 'deepShadowDs5RefObsPositionRobustTargetKeepPenaltyM', 120.0) * ...
                    sum(ismember(keepPrnAll(:), hardDropPrns(:)));
            end
        end
        if score < bestScore
            bestScore = score;
            bestPos = pos;
            bestRes = res;
            bestPdop = pdop;
            bestCorrM = corrM;
            bestUsedSat = sum(keep);
            bestKeepMask = keep;
            bestCommonClockM = commonClockM;
        end
    end
end
end
function [badScoreByPrn, badScoreUsed] = localUpdateDs5RefObsBadScores(prnUseAll, keepMask, residualKeep, badScoreByPrn, settings)
badScoreUsed = nan(numel(prnUseAll), 1);
if isempty(prnUseAll) || isempty(badScoreByPrn)
    return;
end
keepMask = logical(keepMask(:));
prnUseAll = double(prnUseAll(:));
decay = localGetSettingValue(settings, 'deepShadowDs5RefObsPositionRobustBadScoreDecay', 0.92);
dropInc = localGetSettingValue(settings, 'deepShadowDs5RefObsPositionRobustBadScoreDropInc', 1.0);
keepDec = localGetSettingValue(settings, 'deepShadowDs5RefObsPositionRobustBadScoreKeepDec', 0.45);
targetExtra = localGetSettingValue(settings, 'deepShadowDs5RefObsPositionRobustBadScoreTargetExtra', 1.25);
residInc = localGetSettingValue(settings, 'deepShadowDs5RefObsPositionRobustBadScoreResidualInc', 0.35);
residThreshM = localGetSettingValue(settings, 'deepShadowDs5RefObsPositionRobustBadScoreResidualThreshM', 120.0);
scoreMax = localGetSettingValue(settings, 'deepShadowDs5RefObsPositionRobustBadScoreMax', 12.0);
targetPrns = localGetSettingValue(settings, 'deepShadowDs5RefObsPositionRobustBadPrns', [27 9]);
for ii = 1:numel(prnUseAll)
    prn = prnUseAll(ii);
    if ~isfinite(prn) || prn < 1 || prn > numel(badScoreByPrn)
        continue;
    end
    scoreNow = badScoreByPrn(prn);
    if ~isfinite(scoreNow)
        scoreNow = 0;
    end
    scoreNow = scoreNow * decay;
    isTarget = ~isempty(targetPrns) && any(targetPrns(:) == prn);
    if ii <= numel(keepMask) && keepMask(ii)
        scoreNow = max(0, scoreNow - keepDec);
        if ii <= numel(residualKeep) && isfinite(residualKeep(ii)) && abs(residualKeep(ii)) >= residThreshM
            scoreNow = scoreNow + residInc + double(isTarget) * 0.5 * residInc;
        end
    else
        scoreNow = scoreNow + dropInc + double(isTarget) * targetExtra;
    end
    if isfinite(scoreMax) && scoreMax > 0
        scoreNow = min(scoreMax, max(0, scoreNow));
    else
        scoreNow = max(0, scoreNow);
    end
    badScoreByPrn(prn) = scoreNow;
    badScoreUsed(ii) = scoreNow;
end
end
function [posNow, velNow, ageEpochs] = localProjectDs5AbsAnchor(anchorPos, anchorVel, anchorEpoch, anchorTimeSec, anchorValid, epochElapsedSec, currMeasNr, fallbackVel, settings)
posNow = nan(3,1);
velNow = nan(3,1);
ageEpochs = nan;
if ~anchorValid || ~all(isfinite(anchorPos))
    return;
end
posNow = anchorPos(:);
velNow = anchorVel(:);
ageEpochs = currMeasNr - anchorEpoch;
if (~all(isfinite(velNow)) || norm(velNow(:)) <= 0) && ...
        localGetSettingValue(settings, 'deepShadowDs5RecoveredFilterAbsAnchorUseInsVel', 1) ~= 0 && all(isfinite(fallbackVel))
    velNow = fallbackVel(:);
end
if isfinite(anchorTimeSec) && isfinite(epochElapsedSec) && epochElapsedSec >= anchorTimeSec && all(isfinite(velNow))
    posNow = posNow + (epochElapsedSec - anchorTimeSec) * velNow(:);
end
end
function [tf, navResults] = localEvaluateDs5RefObsRecovery(currMeasNr, navResults, baselinePos, baselineValid, settings)
tf = false;
numCh = size(navResults.shadowDs5RefObsUsed, 1);
validCnt = 0;
codeMed = nan;
codeP95 = nan;
freqP95 = nan;
rawValidCnt = 0;
rawCodeMed = nan;
rawCodeP95 = nan;
rawFreqP95 = nan;
recoveryKeep = false(numCh, 1);
robustDropSatNum = 0;
branchGatePass = false;
trackingPass = false;
trackingRawPass = false;
trackingHoldAge = nan;
trackingUsedSatGatePass = false;
trackingCodeMedGatePass = false;
trackingCodeP95GatePass = false;
trackingFreqGatePass = false;
requireBranch = localGetSettingValue(settings, 'deepShadowDs5RefObsRecoveryRequireBranch', 0) ~= 0;
obsContractPass = true;
if localGetSettingValue(settings, 'deepShadowDs5RefObsRecoveryEnable', 1) && localUseDs5ReferenceObsModel(settings, currMeasNr)
    minSat = localGetSettingValue(settings, 'deepShadowDs5RefObsRecoveryMinSat', 4);
    minUsedSat = localGetSettingValue(settings, 'deepShadowDs5RefObsRecoveryMinUsedSat', 4);
    codeMedMax = localGetSettingValue(settings, 'deepShadowDs5RefObsRecoveryCodeMedMaxChips', 0.50);
    codeP95Max = localGetSettingValue(settings, 'deepShadowDs5RefObsRecoveryCodeP95MaxChips', 1.20);
    freqP95Max = localGetSettingValue(settings, 'deepShadowDs5RefObsRecoveryFreqP95MaxHz', 300.0);
    trackingCodeMedMax = localGetSettingValue(settings, 'deepShadowDs5RefObsTrackingCodeMedMaxChips', 0.75);
    trackingCodeP95Max = localGetSettingValue(settings, 'deepShadowDs5RefObsTrackingCodeP95MaxChips', 1.60);
    trackingFreqP95Max = localGetSettingValue(settings, 'deepShadowDs5RefObsTrackingFreqP95MaxHz', 350.0);
    trackingHoldMaxEpochs = max(0, round(localGetSettingValue(settings, 'deepShadowDs5RefObsTrackingHoldMaxEpochs', 8)));
    branchPostfitMax = localGetSettingValue(settings, 'deepShadowDs5RefObsRecoveryBranchPostfitMaxM', 250.0);
    baselineDiffMax = localGetSettingValue(settings, 'deepShadowDs5RefObsRecoveryBaselineDiffMaxM', 2500.0);
    used = navResults.shadowDs5RefObsUsed(:, currMeasNr) ~= 0;
    codeErr = abs(navResults.shadowDs5RefObsCodeErrChips(:, currMeasNr));
    freqErr = abs(navResults.shadowDs5RefObsFreqErrHz(:, currMeasNr));
    valid = used & isfinite(codeErr);
    rawValidCnt = sum(valid);
    if rawValidCnt > 0
        rawCodeMed = median(codeErr(valid), 'omitnan');
        rawCodeP95 = prctile(codeErr(valid), 95);
    end
    freqValid = used & isfinite(freqErr);
    if any(freqValid)
        rawFreqP95 = prctile(freqErr(freqValid), 95);
    end
    recoveryKeep = valid(:);
    robustEnable = localGetSettingValue(settings, 'deepShadowDs5RefObsRecoveryRobustEnable', 0) ~= 0;
    if robustEnable
        maxDropSat = max(0, round(localGetSettingValue(settings, 'deepShadowDs5RefObsRecoveryRobustMaxDropSat', 2)));
        [recoveryKeep, robustDropSatNum, codeMed, codeP95, freqP95] = ...
            localSelectDs5RefObsRecoverySubset(valid(:), codeErr(:), freqErr(:), minUsedSat, maxDropSat, ...
            codeMedMax, codeP95Max, freqP95Max);
    else
        codeMed = rawCodeMed;
        codeP95 = rawCodeP95;
        freqP95 = rawFreqP95;
    end
    validCnt = sum(recoveryKeep);
    refObsGatePass = validCnt >= minUsedSat && ...
        isfinite(codeMed) && codeMed <= codeMedMax && ...
        isfinite(codeP95) && codeP95 <= codeP95Max && ...
        isfinite(freqP95) && freqP95 <= freqP95Max;
    trackingUsedSatGatePass = rawValidCnt >= minUsedSat;
    trackingCodeMedGatePass = isfinite(rawCodeMed) && rawCodeMed <= trackingCodeMedMax;
    trackingCodeP95GatePass = isfinite(rawCodeP95) && rawCodeP95 <= trackingCodeP95Max;
    trackingFreqGatePass = isfinite(rawFreqP95) && rawFreqP95 <= trackingFreqP95Max;
    trackingRawPass = trackingUsedSatGatePass && trackingCodeMedGatePass && ...
        trackingCodeP95GatePass && trackingFreqGatePass;
    if localGetSettingValue(settings, 'deepShadowDs5RefObsTrackingEnable', 1) ~= 0
        if trackingRawPass
            trackingPass = true;
            trackingHoldAge = 0;
        elseif currMeasNr > 1 && navResults.shadowDs5RefObsTrackingPass(1, currMeasNr-1)
            prevHoldAge = navResults.shadowDs5RefObsTrackingHoldAge(1, currMeasNr-1);
            if ~isfinite(prevHoldAge)
                prevHoldAge = 0;
            end
            trackingHoldAge = prevHoldAge + 1;
            trackingPass = trackingHoldAge <= trackingHoldMaxEpochs;
        end
    else
        trackingPass = refObsGatePass;
        trackingRawPass = refObsGatePass;
        trackingHoldAge = 0;
    end

    if isfield(navResults, 'shadowDs5RefObsRecoveryKeep') && currMeasNr <= size(navResults.shadowDs5RefObsRecoveryKeep, 2)
        navResults.shadowDs5RefObsRecoveryKeep(:, currMeasNr) = recoveryKeep(:);
    end
    if localGetSettingValue(settings, 'deepShadowDs5ObsContractEnable', 1) ~= 0
        if ~isfield(navResults, 'shadowDs5ObsContractPass') || currMeasNr > numel(navResults.shadowDs5ObsContractPass)
            obsContractPass = false;
        else
            [obsContractPass, navResults] = localEvaluateDs5ObservationContract(currMeasNr, navResults, settings);
            obsContractPass = obsContractPass || logical(navResults.shadowDs5ObsContractPass(1, currMeasNr));
        end
    end

    branchSat = navResults.shadowBranchSatNum(1, currMeasNr);
    branchPostfit = navResults.shadowBranchPostfitRmsM(1, currMeasNr);
    branchPos = [navResults.shadowBranchX(1, currMeasNr); navResults.shadowBranchY(1, currMeasNr); navResults.shadowBranchZ(1, currMeasNr)];
    branchGatePass = branchSat >= minSat && all(isfinite(branchPos)) && ...
        isfinite(branchPostfit) && branchPostfit <= branchPostfitMax;
    if branchGatePass && baselineValid && all(isfinite(baselinePos))
        branchBaseDiff = norm(branchPos - baselinePos(:));
        branchGatePass = isfinite(branchBaseDiff) && branchBaseDiff <= baselineDiffMax;
    end
    tf = refObsGatePass && obsContractPass && (~requireBranch || branchGatePass);
end
navResults.shadowDs5RefObsRecoveryUsedSatNum(1, currMeasNr) = validCnt;
navResults.shadowDs5RefObsRecoveryCodeMedChips(1, currMeasNr) = codeMed;
navResults.shadowDs5RefObsRecoveryCodeP95Chips(1, currMeasNr) = codeP95;
navResults.shadowDs5RefObsRecoveryFreqP95Hz(1, currMeasNr) = freqP95;
navResults.shadowDs5RefObsRecoveryRawUsedSatNum(1, currMeasNr) = rawValidCnt;
navResults.shadowDs5RefObsRecoveryRawCodeMedChips(1, currMeasNr) = rawCodeMed;
navResults.shadowDs5RefObsRecoveryRawCodeP95Chips(1, currMeasNr) = rawCodeP95;
navResults.shadowDs5RefObsRecoveryRawFreqP95Hz(1, currMeasNr) = rawFreqP95;
navResults.shadowDs5RefObsRecoveryRobustDropSatNum(1, currMeasNr) = robustDropSatNum;
navResults.shadowDs5RefObsRecoveryKeep(:, currMeasNr) = recoveryKeep(:);
navResults.shadowDs5RefObsRecoveryBranchRequired(1, currMeasNr) = requireBranch;
navResults.shadowDs5RefObsRecoveryBranchGatePass(1, currMeasNr) = branchGatePass;
navResults.shadowDs5RefObsRecoveryPass(1, currMeasNr) = tf;
navResults.shadowDs5RefObsTrackingPass(1, currMeasNr) = trackingPass;
navResults.shadowDs5RefObsTrackingRawPass(1, currMeasNr) = trackingRawPass;
navResults.shadowDs5RefObsTrackingHoldAge(1, currMeasNr) = trackingHoldAge;
navResults.shadowDs5RefObsTrackingUsedSatGatePass(1, currMeasNr) = trackingUsedSatGatePass;
navResults.shadowDs5RefObsTrackingCodeMedGatePass(1, currMeasNr) = trackingCodeMedGatePass;
navResults.shadowDs5RefObsTrackingCodeP95GatePass(1, currMeasNr) = trackingCodeP95GatePass;
navResults.shadowDs5RefObsTrackingFreqGatePass(1, currMeasNr) = trackingFreqGatePass;
end

function [bestKeep, bestDropSatNum, bestCodeMed, bestCodeP95, bestFreqP95] = ...
    localSelectDs5RefObsRecoverySubset(valid, codeErr, freqErr, minUsedSat, maxDropSat, codeMedMax, codeP95Max, freqP95Max)
valid = valid(:) & isfinite(codeErr(:));
codeErr = codeErr(:);
freqErr = freqErr(:);
bestKeep = valid;
[bestCodeMed, bestCodeP95, bestFreqP95] = localComputeDs5RefObsRecoveryMetrics(bestKeep, codeErr, freqErr);
bestDropSatNum = 0;
idx = find(valid);
nSat = numel(idx);
if nSat <= 0
    return;
end
maxDropSat = min(max(0, round(maxDropSat)), max(0, nSat - max(1, round(minUsedSat))));
bestPass = localDs5RefObsRecoveryMetricsPass(bestKeep, bestCodeMed, bestCodeP95, bestFreqP95, minUsedSat, codeMedMax, codeP95Max, freqP95Max);
bestViolation = localDs5RefObsRecoveryMetricViolation(bestCodeMed, bestCodeP95, bestFreqP95, codeMedMax, codeP95Max, freqP95Max);
for dropNum = 0:maxDropSat
    if dropNum == 0
        dropSets = zeros(1, 0);
    else
        dropSets = nchoosek(1:nSat, dropNum);
    end
    for rr = 1:size(dropSets, 1)
        keep = valid;
        if dropNum > 0
            keep(idx(dropSets(rr, :))) = false;
        end
        [codeMed, codeP95, freqP95] = localComputeDs5RefObsRecoveryMetrics(keep, codeErr, freqErr);
        pass = localDs5RefObsRecoveryMetricsPass(keep, codeMed, codeP95, freqP95, minUsedSat, codeMedMax, codeP95Max, freqP95Max);
        violation = localDs5RefObsRecoveryMetricViolation(codeMed, codeP95, freqP95, codeMedMax, codeP95Max, freqP95Max);
        if localIsBetterDs5RefObsRecoverySubset(pass, dropNum, violation, codeP95, bestPass, bestDropSatNum, bestViolation, bestCodeP95)
            bestKeep = keep;
            bestDropSatNum = dropNum;
            bestCodeMed = codeMed;
            bestCodeP95 = codeP95;
            bestFreqP95 = freqP95;
            bestPass = pass;
            bestViolation = violation;
        end
    end
end
end

function [codeMed, codeP95, freqP95] = localComputeDs5RefObsRecoveryMetrics(keep, codeErr, freqErr)
codeMed = nan;
codeP95 = nan;
freqP95 = nan;
if any(keep)
    codeUse = codeErr(keep & isfinite(codeErr));
    if ~isempty(codeUse)
        codeMed = median(codeUse, 'omitnan');
        codeP95 = prctile(codeUse, 95);
    end
    freqUse = freqErr(keep & isfinite(freqErr));
    if ~isempty(freqUse)
        freqP95 = prctile(freqUse, 95);
    end
end
end

function tf = localDs5RefObsRecoveryMetricsPass(keep, codeMed, codeP95, freqP95, minUsedSat, codeMedMax, codeP95Max, freqP95Max)
tf = sum(keep) >= minUsedSat && ...
    isfinite(codeMed) && codeMed <= codeMedMax && ...
    isfinite(codeP95) && codeP95 <= codeP95Max && ...
    isfinite(freqP95) && freqP95 <= freqP95Max;
end

function score = localDs5RefObsRecoveryMetricViolation(codeMed, codeP95, freqP95, codeMedMax, codeP95Max, freqP95Max)
score = localMetricExcess(codeMed, codeMedMax) + localMetricExcess(codeP95, codeP95Max) + localMetricExcess(freqP95, freqP95Max);
end

function score = localMetricExcess(value, limit)
if ~isfinite(value) || ~isfinite(limit) || limit <= 0
    score = 1e6;
else
    score = max(0, (value - limit) / max(abs(limit), eps));
end
end

function tf = localIsBetterDs5RefObsRecoverySubset(pass, dropNum, violation, codeP95, bestPass, bestDropNum, bestViolation, bestCodeP95)
if pass ~= bestPass
    tf = pass;
    return;
end
if pass
    if dropNum ~= bestDropNum
        tf = dropNum < bestDropNum;
        return;
    end
else
    if abs(violation - bestViolation) > 1e-12
        tf = violation < bestViolation;
        return;
    end
    if dropNum ~= bestDropNum
        tf = dropNum < bestDropNum;
        return;
    end
end
if ~isfinite(codeP95)
    codeP95 = inf;
end
if ~isfinite(bestCodeP95)
    bestCodeP95 = inf;
end
tf = codeP95 < bestCodeP95;
end

function [outPos, outVel, outSource, useRecovered, lastPos, lastVel, lastEpoch, lastSource, holdAge, recoverConfirmCount, pubLastPos, pubLastVel, pubLastEpoch, recoveryGatePass, recoveryBaselineDiffM, continuityDiffM, continuityPass, slewApplied, slewStepM, holdBaselineCompete, holdBaselineSelected] = ...
    localBuildDs5FinalOutput(mainPos, mainVel, baselinePos, baselineVel, baselineValid, baselineProxyP95M, recoveryProxyP95M, closedLoopPos, closedLoopSource, currMeasNr, ...
    spoofConfirmedNow, closedLoopVel, closedLoopEpoch, lastPos, lastVel, lastEpoch, lastSource, holdAge, recoverConfirmCount, pubLastPos, pubLastVel, pubLastEpoch, ds5RefObsRecoveryNow, recoveredHoldAllowedNow, recoveredBaselineDiffNow, obsContractPassNow, settings)
outPos = mainPos(:);
outVel = mainVel(:);
outSource = 0;
useRecovered = false;
recoveryGatePass = false;
recoveryBaselineDiffM = nan;
continuityDiffM = nan;
continuityPass = true;
slewApplied = false;
slewStepM = nan;
holdBaselineCompete = false;
holdBaselineSelected = false;
if ~spoofConfirmedNow
    if baselineValid && all(isfinite(baselinePos))
        outPos = baselinePos(:);
        if all(isfinite(baselineVel))
            outVel = baselineVel(:);
        end
        outSource = 1;
        holdAge = 0;
        recoverConfirmCount = 0;
        pubLastPos = outPos;
        pubLastVel = outVel;
        pubLastEpoch = currMeasNr;
        return;
    end
    outSource = 1;
    recoverConfirmCount = 0;
    pubLastPos = outPos;
    pubLastVel = outVel;
    pubLastEpoch = currMeasNr;
    return;
end
qualitySelect = localGetSettingValue(settings, 'deepShadowDs5FinalQualitySelectEnable', 1) ~= 0;
forceRecovered = localGetSettingValue(settings, 'deepShadowDs5FinalForceRefObsRecovered', 0) ~= 0;
if qualitySelect
    recoveryValid = all(isfinite(closedLoopPos)) && closedLoopSource == 6;
else
    recoveryValid = all(isfinite(closedLoopPos)) && closedLoopSource ~= 3 && closedLoopSource ~= 5;
end
obsContractGatesRecovered = localGetSettingValue(settings, 'deepShadowDs5ObsContractEnable', 1) ~= 0 && ...
    localGetSettingValue(settings, 'deepShadowDs5FinalRequireObsContractForRecovered', 1) ~= 0;
obsContractGatesHold = localGetSettingValue(settings, 'deepShadowDs5ObsContractEnable', 1) ~= 0 && ...
    localGetSettingValue(settings, 'deepShadowDs5FinalRequireObsContractForHold', 1) ~= 0;
if obsContractGatesRecovered && ~obsContractPassNow
    recoveryValid = false;
end
if recoveryValid && baselineValid && all(isfinite(baselinePos))
    recoveryBaselineDiffM = norm(closedLoopPos(:) - baselinePos(:));
else
    recoveryBaselineDiffM = nan;
end
if recoveryValid
    baselineUsable = baselineValid && all(isfinite(baselinePos));
    allowNoBaselineRecovered = localGetSettingValue(settings, 'deepShadowDs5FinalAllowNoBaselineRecovered', 0) ~= 0;
    qualityImproveMinM = localGetSettingValue(settings, 'deepShadowDs5FinalQualityImproveMinM', -10.0);
    qualityDiffMaxM = localGetSettingValue(settings, 'deepShadowDs5FinalQualityBaselineDiffMaxM', 180.0);
    if baselineUsable
        qualityDiffPass = ~isfinite(recoveryBaselineDiffM) || recoveryBaselineDiffM <= qualityDiffMaxM;
    else
        qualityDiffPass = allowNoBaselineRecovered;
    end
    qualityImprovePass = isfinite(baselineProxyP95M) && isfinite(recoveryProxyP95M) && ...
        recoveryProxyP95M <= baselineProxyP95M - qualityImproveMinM;
    recoveredAuthorityNow = localGetSettingValue(settings, 'deepShadowDs5FinalRecoveredAuthorityEnable', 1) ~= 0 && ds5RefObsRecoveryNow;
    authorityRequireBaselineGate = localGetSettingValue(settings, 'deepShadowDs5FinalRecoveredAuthorityRequireBaselineGate', 1) ~= 0;
    authorityMaxBaselineDiffM = localGetSettingValue(settings, 'deepShadowDs5FinalRecoveredAuthorityMaxBaselineDiffM', qualityDiffMaxM);
    authorityBaselinePass = true;
    if recoveredAuthorityNow && authorityRequireBaselineGate
        if baselineUsable
            authorityBaselinePass = isfinite(recoveryBaselineDiffM) && ...
                (~isfinite(authorityMaxBaselineDiffM) || authorityMaxBaselineDiffM <= 0 || recoveryBaselineDiffM <= authorityMaxBaselineDiffM);
        else
            authorityBaselinePass = allowNoBaselineRecovered;
        end
    end
    recoveryGatePass = (recoveredAuthorityNow && authorityBaselinePass) || ...
        (qualityDiffPass && qualityImprovePass);
    if recoveryGatePass
        if forceRecovered && ds5RefObsRecoveryNow
            recoverConfirmCount = localGetSettingValue(settings, 'deepShadowDs5FinalRecoveredConfirmEpochs', 3);
        else
            recoverConfirmCount = min(localGetSettingValue(settings, 'deepShadowDs5FinalRecoveredConfirmEpochs', 3), recoverConfirmCount + 1);
        end
    else
        recoverConfirmCount = max(0, recoverConfirmCount - 1);
    end
    if recoverConfirmCount >= localGetSettingValue(settings, 'deepShadowDs5FinalRecoveredConfirmEpochs', 3) && recoveryGatePass
        if all(isfinite(pubLastPos)) && isfinite(pubLastEpoch)
            dtSec = (currMeasNr - pubLastEpoch) * localGetSettingValue(settings, 'navSolPeriod', 500.0) / 1000.0;
            ageEpochs = currMeasNr - pubLastEpoch;
            if isfinite(dtSec) && dtSec > 0 && ageEpochs <= localGetSettingValue(settings, 'deepShadowDs5FinalContinuityMaxAgeEpochs', 6) && ...
                    localGetSettingValue(settings, 'deepShadowDs5FinalContinuityGateEnable', 1) ~= 0
                predPos = pubLastPos(:);
                if all(isfinite(pubLastVel))
                    predPos = predPos + dtSec * pubLastVel(:);
                end
                continuityDiffM = norm(closedLoopPos(:) - predPos);
                continuityMaxM = localGetSettingValue(settings, 'deepShadowDs5FinalContinuityMaxPredDiffM', 60.0);
                continuityPass = ~isfinite(continuityMaxM) || continuityMaxM <= 0 || continuityDiffM <= continuityMaxM;
            end
        end
        if continuityPass || localGetSettingValue(settings, 'deepShadowDs5FinalContinuitySlewEnable', 1) ~= 0
            outPos = closedLoopPos(:);
            if ~continuityPass && all(isfinite(pubLastPos)) && isfinite(continuityDiffM)
                continuityMaxM = localGetSettingValue(settings, 'deepShadowDs5FinalContinuityMaxPredDiffM', 60.0);
                if isfinite(continuityMaxM) && continuityMaxM > 0 && continuityDiffM > continuityMaxM
                    deltaPos = closedLoopPos(:) - pubLastPos(:);
                    deltaNorm = norm(deltaPos);
                    if isfinite(deltaNorm) && deltaNorm > continuityMaxM
                        outPos = pubLastPos(:) + deltaPos * (continuityMaxM / deltaNorm);
                        slewApplied = true;
                        slewStepM = norm(outPos - pubLastPos(:));
                    end
                end
            end
            if slewApplied && all(isfinite(pubLastPos)) && isfinite(pubLastEpoch)
                slewDtSec = (currMeasNr - pubLastEpoch) * localGetSettingValue(settings, 'navSolPeriod', 500.0) / 1000.0;
                if isfinite(slewDtSec) && slewDtSec > 0
                    outVel = (outPos(:) - pubLastPos(:)) / slewDtSec;
                end
            elseif all(isfinite(closedLoopVel))
                outVel = closedLoopVel(:);
            elseif all(isfinite(pubLastVel))
                outVel = pubLastVel(:);
            end
            outSource = 2;
            useRecovered = true;
            lastPos = outPos;
            lastVel = outVel;
            lastEpoch = currMeasNr;
            lastSource = closedLoopSource;
            holdAge = 0;
            pubLastPos = outPos;
            pubLastVel = outVel;
            pubLastEpoch = currMeasNr;
            return;
        end
    end
elseif recoverConfirmCount > 0
    recoverConfirmCount = max(0, recoverConfirmCount - 1);
end
recoveredHoldAuthorityOk = ~localGetSettingValue(settings, 'deepShadowDs5FinalRecoveredAuthorityEnable', 1) || recoveredHoldAllowedNow || ...
    (lastSource == 6 && localGetSettingValue(settings, 'deepShadowDs5FinalRecoveredHoldAllowAuthorityGap', 0) ~= 0);
recoveredHoldObsOk = (~obsContractGatesHold || obsContractPassNow) || ...
    (lastSource == 6 && localGetSettingValue(settings, 'deepShadowDs5FinalRecoveredHoldAllowObsGap', 0) ~= 0);
if recoveredHoldAuthorityOk && recoveredHoldObsOk && all(isfinite(lastPos)) && isfinite(lastEpoch) && ...
        (currMeasNr - lastEpoch) <= localGetSettingValue(settings, 'deepShadowDs5FinalOutputHoldEpochs', 120)
    dtSec = (currMeasNr - lastEpoch) * localGetSettingValue(settings, 'navSolPeriod', 500.0) / 1000.0;
    holdPos = lastPos(:);
    holdVel = lastVel;
    if all(isfinite(holdVel))
        holdPos = holdPos + dtSec * holdVel(:);
    end
    baselineUsable = baselineValid && all(isfinite(baselinePos));
    recoveredHoldHealthy = true;
    if lastSource == 6
        recoveredHoldHealthy = recoveredHoldAllowedNow || ...
            localGetSettingValue(settings, 'deepShadowDs5FinalRecoveredHoldAllowAuthorityGap', 0) ~= 0;
        if recoveredHoldHealthy && isfinite(recoveredBaselineDiffNow)
            recoveredHoldBaselineDiffMaxM = localGetSettingValue(settings, 'deepShadowDs5FinalRecoveredHoldBaselineDiffMaxM', ...
                localGetSettingValue(settings, 'deepShadowDs5RecoveredFilterAuthorityBaselineDiffMaxM', 700.0));
            recoveredHoldHealthy = ~isfinite(recoveredHoldBaselineDiffMaxM) || recoveredHoldBaselineDiffMaxM <= 0 || ...
                recoveredBaselineDiffNow <= recoveredHoldBaselineDiffMaxM;
        end
    end
    if localGetSettingValue(settings, 'deepShadowDs5FinalHoldBaselineCompeteEnable', 1) ~= 0 && baselineUsable
        holdBaselineCompete = true;
        holdProxyPass = ~isfinite(baselineProxyP95M) || ~isfinite(recoveryProxyP95M) || ...
            baselineProxyP95M <= recoveryProxyP95M + localGetSettingValue(settings, 'deepShadowDs5FinalHoldBaselineProxyMarginM', 20.0);
        holdDiffM = norm(holdPos - baselinePos(:));
        holdClosePass = ~isfinite(holdDiffM) || holdDiffM <= localGetSettingValue(settings, 'deepShadowDs5FinalHoldBaselineDiffMaxM', 220.0);
        holdAgeEpochs = currMeasNr - lastEpoch;
        holdAgePass = holdAgeEpochs >= localGetSettingValue(settings, 'deepShadowDs5FinalHoldBaselineMinAgeEpochs', 2);
        holdForceDiffM = localGetSettingValue(settings, 'deepShadowDs5FinalHoldBaselineForceDiffM', 220.0);
        holdForceByDiff = isfinite(holdDiffM) && isfinite(holdForceDiffM) && holdForceDiffM > 0 && holdDiffM > holdForceDiffM;
        holdMaxAgeEpochs = localGetSettingValue(settings, 'deepShadowDs5FinalHoldBaselineMaxAgeEpochs', 6);
        holdForceByAge = isfinite(holdMaxAgeEpochs) && holdMaxAgeEpochs >= 0 && holdAgeEpochs > holdMaxAgeEpochs;
        holdBaselineSelected = (lastSource == 6 && ~recoveredHoldHealthy) || ...
            (holdAgePass && ((holdProxyPass && holdClosePass) || holdForceByDiff || holdForceByAge));
        if lastSource == 6 && recoveredHoldHealthy && ...
                localGetSettingValue(settings, 'deepShadowDs5FinalRecoveredHoldSuppressBaselineCompete', 0) ~= 0
            holdBaselineSelected = false;
        end
    end
    if localGetSettingValue(settings, 'deepShadowDs5FinalBaselineOffAblation', 0) ~= 0
        holdBaselineSelected = false;
    end
    if holdBaselineSelected && baselineUsable
        outPos = baselinePos(:);
        if all(isfinite(baselineVel))
            outVel = baselineVel(:);
        end
        outSource = 1;
        useRecovered = false;
        holdAge = 0;
        % Keep recovery hysteresis state; gate logic above owns increment/decay.
        pubLastPos = outPos;
        pubLastVel = outVel;
        pubLastEpoch = currMeasNr;
        return;
    end
    if localGetSettingValue(settings, 'deepShadowDs5FinalBaselineOffAblation', 0) == 0 && ...
            lastSource == 6 && ~recoveredHoldHealthy && baselineUsable
        outPos = baselinePos(:);
        if all(isfinite(baselineVel))
            outVel = baselineVel(:);
        end
        outSource = 5;
        useRecovered = false;
        holdAge = 0;
        pubLastPos = outPos;
        pubLastVel = outVel;
        pubLastEpoch = currMeasNr;
        return;
    end
    outPos = holdPos;
    if all(isfinite(holdVel))
        outVel = holdVel(:);
    end
    outSource = 3;
    useRecovered = true;
    holdAge = currMeasNr - lastEpoch;
    pubLastPos = outPos;
    pubLastVel = outVel;
    pubLastEpoch = currMeasNr;
    return;
end
holdAge = inf;
if closedLoopSource == 4 && all(isfinite(closedLoopPos))
    outPos = closedLoopPos(:);
    if all(isfinite(closedLoopVel))
        outVel = closedLoopVel(:);
    end
    outSource = 4;
    useRecovered = true;
    holdAge = 0;
    recoverConfirmCount = 0;
    pubLastPos = outPos;
    pubLastVel = outVel;
    pubLastEpoch = currMeasNr;
    return;
end
if recoveredHoldAuthorityOk && localGetSettingValue(settings, 'deepShadowDs5FinalBaselineOffAblation', 0) ~= 0 && ...
        (~obsContractGatesHold || obsContractPassNow) && all(isfinite(lastPos)) && isfinite(lastEpoch)
    outPos = lastPos(:);
    if all(isfinite(lastVel))
        outVel = lastVel(:);
    end
    outSource = 3;
    useRecovered = true;
    holdAge = currMeasNr - lastEpoch;
    pubLastPos = outPos;
    pubLastVel = outVel;
    pubLastEpoch = currMeasNr;
    return;
end
if localGetSettingValue(settings, 'deepShadowDs5FinalBaselineOffAblation', 0) ~= 0
    outPos = mainPos(:);
    outVel = mainVel(:);
    outSource = 0;
    useRecovered = false;
    holdAge = inf;
    pubLastPos = outPos;
    pubLastVel = outVel;
    pubLastEpoch = currMeasNr;
    return;
end
if baselineValid && all(isfinite(baselinePos))
    outPos = baselinePos(:);
    if all(isfinite(baselineVel))
        outVel = baselineVel(:);
    end
    outSource = 5;
    useRecovered = false;
    holdAge = 0;
    % Keep recovery hysteresis state; gate logic above owns increment/decay.
    pubLastPos = outPos;
    pubLastVel = outVel;
    pubLastEpoch = currMeasNr;
    return;
end
end
function proxyP95M = localComputePosResidualProxyP95(posEcef, validNow, satPosEcef, rawPUsed, satClkCorr, settings)
proxyP95M = nan;
if ~validNow || ~all(isfinite(posEcef))
    return;
end
if isempty(rawPUsed) || isempty(satClkCorr) || isempty(satPosEcef)
    return;
end
numSat = min([numel(rawPUsed), numel(satClkCorr), size(satPosEcef,2)]);
if numSat < 4
    return;
end
dx = satPosEcef(1,1:numSat) - posEcef(1);
dy = satPosEcef(2,1:numSat) - posEcef(2);
dz = satPosEcef(3,1:numSat) - posEcef(3);
rho = sqrt(dx.^2 + dy.^2 + dz.^2);
delta = rawPUsed(1:numSat).' + settings.c * satClkCorr(1:numSat).' - rho(:);
delta = delta(isfinite(delta));
if numel(delta) < 4
    return;
end
deltaDet = delta - median(delta, 'omitnan');
proxyP95M = prctile(abs(deltaDet), 95);
end

function tf = localUseDs5Authority(settings, currMeasNr)
tf = false;
if ~isfield(settings, 'deepShadowDs5AuthorityEnable') || ~settings.deepShadowDs5AuthorityEnable
    return;
end
if ~localIsDs5Scenario(settings)
    return;
end
startSec = localGetSettingValue(settings, 'deepShadowDs5AuthorityStartSec', 92.0);
navSolPeriodMs = localGetSettingValue(settings, 'navSolPeriod', 500.0);
timeSec = (double(currMeasNr) - 1.0) * navSolPeriodMs / 1000.0;
tf = timeSec >= startSec;
end

function tf = localUseDs5ClosedLiftPrefer(settings, currMeasNr)
tf = false;
if ~isfield(settings, 'deepShadowDs5ClosedLiftPreferEnable') || ~settings.deepShadowDs5ClosedLiftPreferEnable
    return;
end
if ~localIsDs5Scenario(settings)
    return;
end
startSec = localGetSettingValue(settings, 'deepShadowDs5ClosedLiftPreferStartSec', 118.0);
navSolPeriodMs = localGetSettingValue(settings, 'navSolPeriod', 500.0);
timeSec = (double(currMeasNr) - 1.0) * navSolPeriodMs / 1000.0;
tf = timeSec >= startSec;
end

function tf = localUseDs5SoftClamp(settings, currMeasNr)
tf = false;
if ~isfield(settings, 'deepShadowDs5SoftClampEnable') || ~settings.deepShadowDs5SoftClampEnable
    return;
end
if ~localIsDs5Scenario(settings)
    return;
end
startSec = localGetSettingValue(settings, 'deepShadowDs5SoftClampStartSec', 150.0);
navSolPeriodMs = localGetSettingValue(settings, 'navSolPeriod', 500.0);
timeSec = (double(currMeasNr) - 1.0) * navSolPeriodMs / 1000.0;
tf = timeSec >= startSec;
end

function tf = localUseDs5SoftRecovery(settings, currMeasNr)
tf = false;
if ~isfield(settings, 'deepShadowDs5SoftRecoveryEnable') || ~settings.deepShadowDs5SoftRecoveryEnable
    return;
end
if ~localIsDs5Scenario(settings)
    return;
end
startSec = localGetSettingValue(settings, 'deepShadowDs5SoftRecoveryStartSec', 92.0);
navSolPeriodMs = localGetSettingValue(settings, 'navSolPeriod', 500.0);
timeSec = (double(currMeasNr) - 1.0) * navSolPeriodMs / 1000.0;
tf = timeSec >= startSec;
end

function tf = localUseDs5DetrendedGate(settings, currMeasNr)
tf = false;
if ~isfield(settings, 'deepShadowDs5DetrendedGateEnable') || ~settings.deepShadowDs5DetrendedGateEnable
    return;
end
if ~localIsDs5Scenario(settings)
    return;
end
startSec = localGetSettingValue(settings, 'deepShadowDs5DetrendedGateStartSec', 118.0);
navSolPeriodMs = localGetSettingValue(settings, 'navSolPeriod', 500.0);
timeSec = (double(currMeasNr) - 1.0) * navSolPeriodMs / 1000.0;
tf = timeSec >= startSec;
end

function tf = localUseDs5TailNavRelax(settings, currMeasNr)
tf = false;
if ~isfield(settings, 'deepShadowDs5TailNavRelaxEnable') || ~settings.deepShadowDs5TailNavRelaxEnable
    return;
end
if ~localIsDs5Scenario(settings)
    return;
end
startSec = localGetSettingValue(settings, 'deepShadowDs5TailNavRelaxStartSec', 150.0);
navSolPeriodMs = localGetSettingValue(settings, 'navSolPeriod', 500.0);
timeSec = (double(currMeasNr) - 1.0) * navSolPeriodMs / 1000.0;
tf = timeSec >= startSec;
end

function tf = localIsDs5Scenario(settings)
tf = false;
if isfield(settings, 'deepShadowScenarioMode') && ~isempty(settings.deepShadowScenarioMode)
    modeStr = lower(char(string(settings.deepShadowScenarioMode)));
    if contains(modeStr, 'ds5') || contains(modeStr, 'clock_drag')
        tf = true;
        return;
    end
    if contains(modeStr, 'ds6') || contains(modeStr, 'geometry_drag')
        tf = false;
        return;
    end
end
if isfield(settings, 'fileName') && ~isempty(settings.fileName)
    fileLower = lower(char(string(settings.fileName)));
    if contains(fileLower, 'ds5')
        tf = true;
        return;
    end
    if contains(fileLower, 'ds6')
        tf = false;
        return;
    end
end
end

function val = localGetSettingValue(settings, fieldName, defaultVal)
if isfield(settings, fieldName)
    val = settings.(fieldName);
    if ~isempty(val)
        return;
    end
end
val = defaultVal;
end

function bestLocal = localBestConsistentSubset(deltaVec, targetNum)
n = numel(deltaVec);
targetNum = min(max(1, targetNum), n);
comb = nchoosek(1:n, targetNum);
bestScore = inf;
bestLocal = comb(1, :);
for ii = 1:size(comb, 1)
    idx = comb(ii, :);
    d = deltaVec(idx);
    medD = median(d, 'omitnan');
    dev = abs(d - medD);
    score = max(dev) + 0.25 * median(dev, 'omitnan');
    if score < bestScore
        bestScore = score;
        bestLocal = idx;
    end
end
end
