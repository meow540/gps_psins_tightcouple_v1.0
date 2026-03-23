%% GNSS\INS缂備焦鐦遍崨顖滅厑闂佸憡鑹鹃悧鐐垫濠靛洨鈻旂€广儱鎳忛崐鍗灻瑰鍐ㄧ暰NSS_SDR,闂佸搫鐗滈崜姘跺几閸愨晝顩烽柡鍫㈡暩閻熸捇鏌熺喊妯轰壕闂佸搫鐗嗛ˇ鐢稿焵椤掍礁鍝哄ù婊堢畺瀵即顢涘▎鎰敪闂佸搫娲︾€笛勬櫠閻ｅ本鍋樼€光偓鐎ｎ剛顦梺鍏兼緲婵傛梻绮径鎰強妞ゆ牗鐟ч鍗炩槈閹垮啩閭柍褜鍓氬畝鍛婄閻愵剚浜ら柟閭﹀灱閺€鐣岀磽娴ｈ灏版繛纰卞亰瀹曘儲鎯旈垾铏珒闁哄鏅滈崝姗€銆侀幋鐐碘枖閻庯絺鏅濋鍗炩槈閹垮啩閭柍褜鍓氬畝鍛婄?
% trackResults, channel, TOW, eph, subFrameStart 闂傚倸娲犻崑鎾绘偡閺囨氨鍔嶇憸棰佺窔瀹曟粌顓奸崨顓熺彲婵犮垼娉涘ú銈夊Χ?
close all;

% Ensure project functions take precedence over PSINS name conflicts
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
    for fi = 1 : numel(fn)
        settings.(fn{fi}) = settingsOverride.(fn{fi});
    end
end
if ~isfield(settings, 'offlineReplayFromTrackResults')
    settings.offlineReplayFromTrackResults = 0;
end
collectTrackDebug = (~isfield(settings, 'skipQuickPlot') || ~settings.skipQuickPlot) || ...
                    (isfield(settings, 'plotTracking') && settings.plotTracking);
fid = -1;
if ~settings.offlineReplayFromTrackResults
    [fid, ~] = fopen(settings.fileName, 'rb');
    if fid < 0
        error('Cannot open raw IF file: %s', settings.fileName);
    end
    cleanupObj = onCleanup(@() fclose(fid)); %#ok<NASGU>
end


%% INS闂佺儵鏅濋…鍫ュ矗瑜庣粚閬嶅焺閸愌呯闂佸憡甯楃换鍌烇綖閹版澘绀岄柡宥囨暩缁€澶愭⒒閸ワ絽浜鹃梺鍦帛閸旀洘鏅堕悩纰樺亾閻熼偊妲兼い銉ワ攻缁虹晫寰婇崲顢疦S
glvs
ggpsvars
psinstypedef('test_SINS_GPS_tightly_def');
trjFile = "E:\fgi_result\ins_simulation\ins1_trj.mat";
if exist('settingsOverride', 'var') && isstruct(settingsOverride) && isfield(settingsOverride, 'trjFile')
    trjFile = settingsOverride.trjFile;
end
trj = trjfile(trjFile);
if ~isfield(settings, 'trjStartOffsetSec'), settings.trjStartOffsetSec = 0; end
trjStartOffsetSec = max(0, settings.trjStartOffsetSec);
t0_gps = TOW;
positioningTime = t0_gps + settings.navSolPeriod / 1000;
[nn, ts, nts] = nnts(2, diff(trj.imu(1:2,end)));   
[~, avpStartIdx] = min(abs(trj.avp(:,end) - trjStartOffsetSec));
t0_imu = trj.avp(avpStartIdx, end);
imuStartIdx = find(trj.imu(:,end) >= t0_imu, 1, 'first');
if isempty(imuStartIdx)
    error('IMU trajectory does not cover requested start offset %.3f s.', trjStartOffsetSec);
end
avp0 = trj.avp(avpStartIdx, 1:9)';  % use one self-consistent AVP source for Chapter-1
% Do not overwrite only position/velocity from dataset GNSS while keeping
% attitude from the IMU trajectory. That mixed initialization creates an
% inconsistent state before the tight-coupled filter starts.
if ~isfield(settings, 'initAvpUseDatasetGnss')
    settings.initAvpUseDatasetGnss = 0;
end

% IMU error profile selection (default: ins1 = high precision).
if ~isfield(settings, 'imuerrProfile')
    settings.imuerrProfile = 'ins1';
end
if ~isfield(settings, 'initAttErrArcmin') || ~isfield(settings, 'initVelErrMps') || ~isfield(settings, 'initPosErrM')
    switch lower(settings.imuerrProfile)
        case 'ins1'
            settings.initAttErrArcmin = [0.05; 0.05; 0.2];
            settings.initVelErrMps = 0.005;
            settings.initPosErrM = [0.10; 0.10; 0.30];
        case 'ins2'
            settings.initAttErrArcmin = [0.20; 0.20; 1.0];
            settings.initVelErrMps = 0.02;
            settings.initPosErrM = [0.30; 0.30; 0.80];
        otherwise % ins3
            settings.initAttErrArcmin = [1.0; 1.0; 5.0];
            settings.initVelErrMps = 0.08;
            settings.initPosErrM = [1.0; 1.0; 3.0];
    end
end
initAttErrArcmin = settings.initAttErrArcmin(:);
initPosErrM = settings.initPosErrM(:);
davp = avperrset(initAttErrArcmin(1:3), settings.initVelErrMps, initPosErrM(1:3));
if ~isfield(settings, 'initAvpPerturbEnable'), settings.initAvpPerturbEnable = 0; end
if settings.initAvpPerturbEnable
    ins0 = avpadderr(avp0, davp);
else
    ins0 = avp0;
end

ins = insinit(ins0, ts);

if ~exist('imuerr', 'var')
    switch lower(settings.imuerrProfile)
        case 'ins1'
            imuerr = imuerrset(0.01, 100, 0.003, 40);
        case 'ins2'
            imuerr = imuerrset(2, 500, 0.01, 100);
        otherwise
            imuerr = imuerrset(20, 10000, 0.4, 1000); % ins3
    end
end

kf = kfinit(ins, davp, imuerr);

imuLen = size(trj.imu, 1);
imuEndTime = trj.imu(end, end);



%% INS闁哄鏅滈崝姗€銆侀幋锔藉殜闁瑰墎鐡斿ù鏇炩槈閹垮啩绨介柟浼欑悼缁辨帡宕熼锝呪偓銈夋煛閸愵厽纭鹃�?
k = imuStartIdx;
k1 = imuStartIdx;                  
t = trj.imu(k1, end);   
while (t0_gps + (t - t0_imu)) < positioningTime     
    k1 = k+nn-1;
    if k1 > imuLen
        error('IMU data exhausted before first GNSS epoch.');
    end
    wvm = trj.imu(k:k1,1:6);  t = trj.imu(k1,end);
    ins = insupdate(ins, wvm);
    kf.Phikk_1 = kffk(ins);
    kf = kfupdate(kf);

    k = k + nn;      
end

activeChnList = find([trackResults.status] ~= '-');
valid = subFrameStart(activeChnList) > 1;
activeChnList = activeChnList(valid);
numActChnList = numel(activeChnList);
if ~isfield(settings, 'tcMinSatForInit'), settings.tcMinSatForInit = 4; end
if numActChnList < settings.tcMinSatForInit
    error('Too few valid satellites for tight coupling: %d < %d.', numActChnList, settings.tcMinSatForInit);
end
BitSyncTime = subFrameStart - 1;
ii = 1;
for ch = activeChnList   
    trackDeepIn(ii).PRN = trackResults(ch).PRN;   
    trackDeepIn(ii).srcCh = ch;  % back-reference to source channel in trackResults

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
    

    trackDeepIn(ii).numOfCoInt = 0;

    % 闁荤姳鐒﹀妯肩礊瀹ュ棛鈹嶉柍銉ュ暱婵炲洭鎮归搹鐟版灆闂佸弶绮撻幆鍐礋椤斿墽鐣虹紓浣割儏椤戞垹妲愬┑鍥╃懝濠㈣泛鎽滈懝楣冩偣鐎ｎ亜鏆㈤柣?
    if collectTrackDebug
        trackProcess(ii).codeErrorList = [];
        trackProcess(ii).carrErrorList = [];
        trackProcess(ii).codeFreqList = [];
        trackProcess(ii).carrFreqList = []; 
        trackProcess(ii).PLI = [];           % Phase Lock Indicator 闁荤姴娴勭粻鎺楋綖閸℃稑绀冩い鏍仦閸?濠殿喗绻愮粻鎴﹀Φ?闂佹眹鍔岀€氼厾鎹㈡担铏圭＜闁告洦鍋呴崐銈夋偣娴ｅ搫鍔嬮柡?
    
    
    end

    ii = ii + 1;
end


% Filter channels without complete ephemeris
validEph = false(1, numActChnList);
for ii = 1 : numActChnList
    prn = trackDeepIn(ii).PRN;
    validEph(ii) = ~isempty(eph(prn).IODC) && ~isempty(eph(prn).IODE_sf2) && ...
                   ~isempty(eph(prn).IODE_sf3) && isscalar(eph(prn).a_f0) && ...
                   isscalar(eph(prn).a_f1) && isscalar(eph(prn).a_f2) && isscalar(eph(prn).t_oc);
end
trackDeepIn = trackDeepIn(validEph);
numActChnList = numel(trackDeepIn);
if numActChnList < settings.tcMinSatForInit
    error('Too few satellites with valid ephemeris for tight coupling: %d < %d.', numActChnList, settings.tcMinSatForInit);
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

I_P_1_list = [];  % debug trace for channel-1 prompt I
Q_P_1_list = [];  % debug trace for channel-1 prompt Q

for ii = 1 : numActChnList
    trackans = trackDeepIn(ii);
    while trackans.recvTime < positioningTime     
        trackDeepIn(ii) = trackans;
        if settings.offlineReplayFromTrackResults
            srcCh = trackans.srcCh;
            idxSrc = BitSyncTime(srcCh) + trackans.numOfCoInt + 1;
            if idxSrc > numel(trackResults(srcCh).absoluteSample)
                error('Offline replay ran out of trackResults samples before first epoch.');
            end
            trackans.numOfCoInt = trackans.numOfCoInt + 1;
            trackans.SamplePos = trackResults(srcCh).absoluteSample(idxSrc);
            trackans.codeFreq = trackResults(srcCh).codeFreq(idxSrc);
            trackans.remCodePhase = trackResults(srcCh).remCodePhase(idxSrc);
            trackans.carrFreq = trackResults(srcCh).carrFreq(idxSrc);
            trackans.remCarrPhase = trackResults(srcCh).remCarrPhase(idxSrc);
            trackans.codeError = trackResults(srcCh).dllDiscr(idxSrc);
            trackans.codeNco = trackResults(srcCh).dllDiscrFilt(idxSrc);
            trackans.carrError = trackResults(srcCh).pllDiscr(idxSrc);
            trackans.carrNco = trackResults(srcCh).pllDiscrFilt(idxSrc);
            trackans.recvTime = recvTimeforFirstFrameperChannel(ii) + ...
                                (trackans.SamplePos - SamplePosatFirstFrame(ii)) / settings.samplingFreq;
            I_P = 0;
            Q_P = 0;
        else
            [trackans, I_P, Q_P] = perChannelTrackOnce(trackans, settings, fid);
        end
        
        if collectTrackDebug
            if ii == 1
                I_P_1_list = [I_P_1_list, I_P];
                Q_P_1_list = [Q_P_1_list, Q_P];
            end
            
            trackProcess(ii).codeErrorList = [trackProcess(ii).codeErrorList, trackans.codeError];
            trackProcess(ii).carrErrorList = [trackProcess(ii).carrErrorList, trackans.carrError];
            trackProcess(ii).codeFreqList = [trackProcess(ii).codeFreqList, trackans.codeFreq];
            trackProcess(ii).carrFreqList = [trackProcess(ii).carrFreqList, trackans.carrFreq];
            trackProcess(ii).PLI = [trackProcess(ii).PLI, (I_P^2-Q_P^2)/(I_P^2+Q_P^2)];
        end
    end
end


roundTime = floor(settings.msToProcess / settings.navSolPeriod);
maxEpochsByImu = floor((imuEndTime - t0_imu) / (settings.navSolPeriod / 1000));
if roundTime > maxEpochsByImu
    roundTime = maxEpochsByImu;
end

if ~isfield(settings, 'verboseEpochPrint'), settings.verboseEpochPrint = 0; end
if ~isfield(settings, 'skipQuickPlot'), settings.skipQuickPlot = 0; end
if ~isfield(settings, 'spoofDetMinElevDeg'), settings.spoofDetMinElevDeg = 20; end
if ~isfield(settings, 'spoofDetMinSat'), settings.spoofDetMinSat = 4; end
if ~isfield(settings, 'spoofDetMinHitSat'), settings.spoofDetMinHitSat = settings.spoofDetMinSat; end
if ~isfield(settings, 'spoofDetConfirmEpochs'), settings.spoofDetConfirmEpochs = 2; end
if ~isfield(settings, 'spoofDetMetricSmoothWin'), settings.spoofDetMetricSmoothWin = 3; end
if ~isfield(settings, 'spoofDetTemplateEpochOffset'), settings.spoofDetTemplateEpochOffset = 0; end
if ~isfield(settings, 'spoofDetArmTimeSec'), settings.spoofDetArmTimeSec = 0; end
if ~isfield(settings, 'spoofDetPostArmGraceSec'), settings.spoofDetPostArmGraceSec = 0; end
if ~isfield(settings, 'spoofDetCommonThresholdHz')
    if isfield(settings, 'spoofDetMetricThresholdHz')
        settings.spoofDetCommonThresholdHz = settings.spoofDetMetricThresholdHz;
    else
        settings.spoofDetCommonThresholdHz = 4.5;
    end
end
if ~isfield(settings, 'spoofDetDiffThresholdHz')
    if isfield(settings, 'spoofDetMetricThresholdHz')
        settings.spoofDetDiffThresholdHz = settings.spoofDetMetricThresholdHz;
    else
        settings.spoofDetDiffThresholdHz = settings.spoofDetCommonThresholdHz;
    end
end
if ~isfield(settings, 'spoofDetSatThresholdHz')
    settings.spoofDetSatThresholdHz = max(settings.spoofDetCommonThresholdHz, settings.spoofDetDiffThresholdHz);
end
if ~isfield(settings, 'gnssTakeoverMode'), settings.gnssTakeoverMode = 'ins_only_latched'; end
if ~isfield(settings, 'virtualNHCEnable'), settings.virtualNHCEnable = 0; end
if ~isfield(settings, 'virtualNHCMinHorSpeed'), settings.virtualNHCMinHorSpeed = 0.2; end
if ~isfield(settings, 'virtualNHCNoiseStdMps'), settings.virtualNHCNoiseStdMps = [0.08; 0.08]; end
if ~isfield(settings, 'virtualZUPTEnable'), settings.virtualZUPTEnable = 1; end
if ~isfield(settings, 'virtualZUPTSpeedTh'), settings.virtualZUPTSpeedTh = 0.2; end
if ~isfield(settings, 'virtualZUPTNoiseStdMps'), settings.virtualZUPTNoiseStdMps = [0.03; 0.03; 0.03]; end
if ~isfield(settings, 'tcUseRangeRateUpdate'), settings.tcUseRangeRateUpdate = 1; end
if ~isfield(settings, 'tcRangeNoiseStdM'), settings.tcRangeNoiseStdM = 2.0; end
if ~isfield(settings, 'tcRangeRateNoiseStdMps'), settings.tcRangeRateNoiseStdMps = 0.10; end
if ~isfield(settings, 'tcNavMinElevDeg'), settings.tcNavMinElevDeg = max(10, settings.elevationMask); end
if ~isfield(settings, 'tcNavMinSat'), settings.tcNavMinSat = 4; end
if ~isfield(settings, 'insTakeoverPrrAssistEnable'), settings.insTakeoverPrrAssistEnable = 1; end
if ~isfield(settings, 'insTakeoverPrrAssistPolicy'), settings.insTakeoverPrrAssistPolicy = 'always'; end
if ~isfield(settings, 'insTakeoverPrrAssistMcmOverDiffMarginHz'), settings.insTakeoverPrrAssistMcmOverDiffMarginHz = inf; end
if ~isfield(settings, 'insTakeoverPrrAssistMinDiffHz'), settings.insTakeoverPrrAssistMinDiffHz = 0.0; end
% Weak-assisted takeover keeps only the differential Doppler part after

% common-mode spoof removal. The clip must be large enough to constrain

% vehicle dynamics; overly small values behave almost like pure INS.

if ~isfield(settings, 'insTakeoverPrrResidualClipHz'), settings.insTakeoverPrrResidualClipHz = 10.0; end
if ~isfield(settings, 'insTakeoverPrrNoiseStdMps'), settings.insTakeoverPrrNoiseStdMps = 0.1; end
if ~isfield(settings, 'gnssRecoveryEnable'), settings.gnssRecoveryEnable = 1; end
if ~isfield(settings, 'spoofReleaseCommonScale'), settings.spoofReleaseCommonScale = 0.35; end
if ~isfield(settings, 'spoofReleaseDiffScale'), settings.spoofReleaseDiffScale = 0.35; end
if ~isfield(settings, 'spoofReleaseSatScale'), settings.spoofReleaseSatScale = 0.35; end
if ~isfield(settings, 'spoofReleaseMinHitSat'), settings.spoofReleaseMinHitSat = 0; end
if ~isfield(settings, 'spoofReleaseConfirmEpochs'), settings.spoofReleaseConfirmEpochs = 20; end
if ~isfield(settings, 'spoofRecoveryMinLockSec'), settings.spoofRecoveryMinLockSec = 20; end
if ~isfield(settings, 'gnssRampDurationSec'), settings.gnssRampDurationSec = 10; end
if ~isfield(settings, 'gnssRampPrScaleStart'), settings.gnssRampPrScaleStart = 10; end
if ~isfield(settings, 'gnssRampPrrScaleStart'), settings.gnssRampPrrScaleStart = 5; end
if ~isfield(settings, 'virtualSpeedHoldEnable'), settings.virtualSpeedHoldEnable = 0; end
if ~isfield(settings, 'virtualSpeedHoldMinMps'), settings.virtualSpeedHoldMinMps = 1.0; end
if ~isfield(settings, 'virtualSpeedHoldNoiseStdMps'), settings.virtualSpeedHoldNoiseStdMps = 0.8; end
if ~isfield(settings, 'virtualHeightHoldEnable'), settings.virtualHeightHoldEnable = 1; end
if ~isfield(settings, 'virtualHeightHoldNoiseStdM'), settings.virtualHeightHoldNoiseStdM = 12; end
if ~isfield(settings, 'virtualPitchRollHoldEnable'), settings.virtualPitchRollHoldEnable = 0; end
if ~isfield(settings, 'virtualPitchRollHoldNoiseStdRad'), settings.virtualPitchRollHoldNoiseStdRad = [0.5; 0.5] * pi / 180; end
if ~isfield(settings, 'virtualVertVelHoldEnable'), settings.virtualVertVelHoldEnable = 0; end
if ~isfield(settings, 'virtualVertVelHoldNoiseStdMps'), settings.virtualVertVelHoldNoiseStdMps = 0.10; end
if ~isfield(settings, 'leverArm_b'), settings.leverArm_b = [0; 0; 0]; end
if ~isfield(settings, 'kfGuardEnable'), settings.kfGuardEnable = 1; end
if ~isfield(settings, 'kfGuardStateAbsMax'), settings.kfGuardStateAbsMax = 1e9; end
if ~isfield(settings, 'trustedGnssFeedbackStr'), settings.trustedGnssFeedbackStr = 'avped'; end
if ~isfield(settings, 'takeoverPrrFeedbackStr'), settings.takeoverPrrFeedbackStr = 'avp'; end
if ~isfield(settings, 'spoofMitigationMode'), settings.spoofMitigationMode = 'ins_only'; end
settings.spoofMitigationMode = 'ins_only';

leverArm_b = settings.leverArm_b(:);
if numel(leverArm_b) ~= 3
    error('settings.leverArm_b must be a 3x1 vector.');
end

navResults = [];
navResults.X = zeros(1, roundTime);
navResults.Y = zeros(1, roundTime);
navResults.Z = zeros(1, roundTime);
navResults.dt = zeros(1, roundTime);
navResults.VX = zeros(1, roundTime);
navResults.VY = zeros(1, roundTime);
navResults.VZ = zeros(1, roundTime);
navResults.df = zeros(1, roundTime);
navResults.prnList = [trackDeepIn.PRN];

navResults.modeNormal = true(1, roundTime);
navResults.modeInsOnly = false(1, roundTime);
navResults.modeInsTakeover = false(1, roundTime);
navResults.modeGnssRamp = false(1, roundTime);
navResults.modeKinConstraint = false(1, roundTime);
navResults.modeVirtualNHC = false(1, roundTime);
navResults.modeVirtualZUPT = false(1, roundTime);
navResults.modeVirtualSpeedHold = false(1, roundTime);
navResults.modeVirtualHeightHold = false(1, roundTime);
navResults.modeVirtualPitchRollHold = false(1, roundTime);
navResults.modeVirtualVertVelHold = false(1, roundTime);
navResults.modeTakeoverPrrAssist = false(1, roundTime);
navResults.kinCorrectionNorm = zeros(1, roundTime);

navResults.spoofFlag = false(1, roundTime);
navResults.spoofAlarm = false(1, roundTime);
navResults.recoveryCandidate = false(1, roundTime);
navResults.recoveryCounter = zeros(1, roundTime);
navResults.modeState = zeros(1, roundTime); % 0:normal,1:ins_only,2:gnss_ramp

navResults.dopplerGpsMeasHz = nan(numActChnList, roundTime);
navResults.dopplerInsPredHz = nan(numActChnList, roundTime);
navResults.dopplerRecvClkHz = nan(1, roundTime);
navResults.dopplerPredGeomHz = nan(numActChnList, roundTime);
navResults.dopplerSatClkHz = nan(numActChnList, roundTime);
navResults.dopplerResidualRawHz = nan(numActChnList, roundTime);
navResults.dopplerResidualBiasCorrectedHz = nan(numActChnList, roundTime);
navResults.dopplerResidualTemplateDevHz = nan(numActChnList, roundTime);
navResults.dopplerResidualHz = nan(numActChnList, roundTime);
navResults.dopplerSuppressedHz = nan(numActChnList, roundTime);
navResults.dopplerSuppressAlpha = nan(numActChnList, roundTime);

navResults.detectorArmed = false(1, roundTime);
navResults.detectorCauseCode = zeros(1, roundTime); % 0:none,1:common,2:diff,3:both
navResults.detectorMetricCommonRawHz = nan(1, roundTime);
navResults.detectorMetricCommonHz = nan(1, roundTime);
navResults.detectorMetricDiffRawHz = nan(1, roundTime);
navResults.detectorMetricDiffHz = nan(1, roundTime);
navResults.detectorHitSatNum = zeros(1, roundTime);
navResults.detectorReleaseHitSatNum = zeros(1, roundTime);
navResults.detectorValidSatNum = zeros(1, roundTime);
navResults.detectorThresholdCommonHz = settings.spoofDetCommonThresholdHz * ones(1, roundTime);
navResults.detectorThresholdDiffHz = settings.spoofDetDiffThresholdHz * ones(1, roundTime);
navResults.detectorThresholdSatHz = settings.spoofDetSatThresholdHz * ones(1, roundTime);
navResults.detectorReleaseThresholdCommonHz = settings.spoofDetCommonThresholdHz * settings.spoofReleaseCommonScale * ones(1, roundTime);
navResults.detectorReleaseThresholdDiffHz = settings.spoofDetDiffThresholdHz * settings.spoofReleaseDiffScale * ones(1, roundTime);
navResults.detectorReleaseThresholdSatHz = settings.spoofDetSatThresholdHz * settings.spoofReleaseSatScale * ones(1, roundTime);

navResults.mitigationLevel = zeros(1, roundTime); % 0:normal,1:ins_only,2:gnss_ramp
navResults.qualityFallback = false(1, roundTime);
navResults.qualityFallbackCode = zeros(1, roundTime); % 0:none,1:invalid meas,2:kf fail,5:nhc fail,6:zupt fail
navResults.gnssUpdateUsed = false(1, roundTime);
navResults.gnssWeightScalePr = ones(1, roundTime);
navResults.gnssWeightScalePrr = ones(1, roundTime);

baselinePrnList = navResults.prnList;
if isfield(settings, 'spoofDetBaselinePrnList')
    baselinePrnList = settings.spoofDetBaselinePrnList;
elseif isfield(settings, 'baselinePrnList')
    baselinePrnList = settings.baselinePrnList;
end
baselineBiasHzSource = zeros(numel(baselinePrnList), 1);
if isfield(settings, 'spoofDetSatBiasHz')
    baselineBiasHzSource = settings.spoofDetSatBiasHz(:);
elseif isfield(settings, 'baselineSatBiasHz')
    baselineBiasHzSource = settings.baselineSatBiasHz(:);
end
if numel(baselineBiasHzSource) ~= numel(baselinePrnList)
    baselineBiasHzSource = zeros(numel(baselinePrnList), 1);
end
baselineSatBiasHz = localMapBaselineByPrn(navResults.prnList, baselinePrnList, baselineBiasHzSource, 0);

templatePrnList = [];
if isfield(settings, 'spoofDetTemplatePrnList')
    templatePrnList = settings.spoofDetTemplatePrnList;
elseif isfield(settings, 'baselineTemplatePrnList')
    templatePrnList = settings.baselineTemplatePrnList;
end
templateResidualSource = [];
if isfield(settings, 'spoofDetTemplateResidualHz')
    templateResidualSource = settings.spoofDetTemplateResidualHz;
elseif isfield(settings, 'baselineTemplateResidualHz')
    templateResidualSource = settings.baselineTemplateResidualHz;
end
templateResidualHz = localMapTemplateByPrn(navResults.prnList, templatePrnList, templateResidualSource, roundTime, settings.spoofDetTemplateEpochOffset);

l1Freq = 1575.42e6;
MODE_NORMAL = 0;
MODE_INS_ONLY = 1;
MODE_GNSS_RAMP = 2;
navMode = MODE_NORMAL;
confirmCounter = 0;
releaseCounter = 0;
rampCounter = 0;
spoofStartEpoch = -1;
lastAlarmEpoch = -1;
releaseConfirmEpochs = max(1, round(settings.spoofReleaseConfirmEpochs));
rampEpochs = max(1, round(settings.gnssRampDurationSec * 1000 / settings.navSolPeriod));
lockEpochs = max(0, ceil(settings.spoofRecoveryMinLockSec * 1000 / settings.navSolPeriod));
releaseCommonThresholdHz = settings.spoofDetCommonThresholdHz * settings.spoofReleaseCommonScale;
releaseDiffThresholdHz = settings.spoofDetDiffThresholdHz * settings.spoofReleaseDiffScale;
releaseSatThresholdHz = settings.spoofDetSatThresholdHz * settings.spoofReleaseSatScale;
metricCommonHistRaw = nan(1, roundTime);
metricDiffHistRaw = nan(1, roundTime);
stopEpoch = roundTime;
takeoverForwardSpeedRefMps = nan;
takeoverHeightRefM = nan;
takeoverPitchRollRefRad = [nan; nan];

for currMeasNr = 1 : roundTime
    if settings.verboseEpochPrint
        fprintf('currMeasNr = %d\n', currMeasNr);
    end
    currTimeSec = (currMeasNr - 1) * settings.navSolPeriod / 1000;
    settings.recvTime = positioningTime;
    navSolut_1 = [];

    navSolut = postNavTight(trackDeepIn, settings, eph, TOW);

    [posxyz, ~] = blh2xyz(ins.pos);
    antVelN = ins.vn;
    if any(abs(leverArm_b) > 0)
        antVelN = antVelN + ins.Cnb * cross(ins.wib, leverArm_b);
    end

    [geomRhoDet, LOSDet, AzElDet, v_r_s] = rhoSatRec_zcj(navSolut.satPositions', posxyz, navSolut.rawP', navSolut.satVelocity', antVelN);
    dopplerPredGeomHz = -v_r_s / settings.c * l1Freq;
    dopplerMeasHz = [trackDeepIn.carrFreq]' - settings.IF;
    recvClkDopplerHz = -kf.xk(end) / settings.c * l1Freq;
    satClkDopplerHz = navSolut.satClkDrift(:) * l1Freq;
    dopplerPredRawHz = dopplerPredGeomHz + satClkDopplerHz + recvClkDopplerHz;
    dopplerResidualRawHz = dopplerMeasHz - dopplerPredRawHz;
    dopplerResidualBcHz = dopplerResidualRawHz - baselineSatBiasHz;
    dopplerResidualDevHz = dopplerResidualBcHz - templateResidualHz(:, currMeasNr);

    validDet = isfinite(dopplerResidualDevHz) & isfinite(dopplerResidualRawHz) & ...
               (AzElDet(:,2) >= settings.spoofDetMinElevDeg * pi / 180);
    validDetNum = sum(validDet);

    metricCommonRawHz = nan;
    metricDiffRawHz = nan;
    metricCommonHz = nan;
    metricDiffHz = nan;
    hitSatNum = 0;
    releaseHitSatNum = 0;
    detectorCause = 0;
    if validDetNum >= settings.spoofDetMinSat
        commonCenterHz = median(dopplerResidualDevHz(validDet), 'omitnan');
        metricCommonRawHz = abs(commonCenterHz);
        metricDiffRawHz = median(abs(dopplerResidualDevHz(validDet) - commonCenterHz), 'omitnan');
        hitSatNum = sum(abs(dopplerResidualDevHz(validDet)) > settings.spoofDetSatThresholdHz);
        releaseHitSatNum = sum(abs(dopplerResidualDevHz(validDet)) > releaseSatThresholdHz);
    end

    metricCommonHistRaw(1, currMeasNr) = metricCommonRawHz;
    metricDiffHistRaw(1, currMeasNr) = metricDiffRawHz;
    smoothWin = max(1, round(settings.spoofDetMetricSmoothWin));
    idxHist = max(1, currMeasNr - smoothWin + 1) : currMeasNr;
    metricCommonHz = median(metricCommonHistRaw(idxHist), 'omitnan');
    metricDiffHz = median(metricDiffHistRaw(idxHist), 'omitnan');

    hitCommon = isfinite(metricCommonHz) && (metricCommonHz > settings.spoofDetCommonThresholdHz);
    hitDiff = isfinite(metricDiffHz) && (metricDiffHz > settings.spoofDetDiffThresholdHz);
    if hitCommon && hitDiff
        detectorCause = 3;
    elseif hitCommon
        detectorCause = 1;
    elseif hitDiff
        detectorCause = 2;
    end

    detectorArmed = currTimeSec >= (settings.spoofDetArmTimeSec + settings.spoofDetPostArmGraceSec);
    detectHit = detectorArmed && ...
                (validDetNum >= settings.spoofDetMinSat) && ...
                (hitSatNum >= settings.spoofDetMinHitSat) && ...
                (hitCommon || hitDiff);
    lockActive = (lastAlarmEpoch > 0) && ((currMeasNr - lastAlarmEpoch) < lockEpochs);
    releaseStable = detectorArmed && settings.gnssRecoveryEnable && ...
                    (navMode == MODE_INS_ONLY) && ...
                    (validDetNum >= settings.spoofDetMinSat) && ...
                    isfinite(metricCommonHz) && isfinite(metricDiffHz) && ...
                    isfinite(metricCommonRawHz) && isfinite(metricDiffRawHz) && ...
                    (metricCommonHz < releaseCommonThresholdHz) && ...
                    (metricDiffHz < releaseDiffThresholdHz) && ...
                    (metricCommonRawHz < releaseCommonThresholdHz) && ...
                    (metricDiffRawHz < releaseDiffThresholdHz) && ...
                    (hitSatNum == 0) && ...
                    (releaseHitSatNum <= settings.spoofReleaseMinHitSat) && ...
                    ~lockActive;

    alarmRaised = false;
    recoveryAccepted = false;
    if navMode ~= MODE_INS_ONLY
        if detectHit
            confirmCounter = confirmCounter + 1;
        else
            confirmCounter = 0;
        end
        if confirmCounter >= settings.spoofDetConfirmEpochs
            navMode = MODE_INS_ONLY;
            confirmCounter = 0;
            releaseCounter = 0;
            rampCounter = 0;
            lastAlarmEpoch = currMeasNr;
            if spoofStartEpoch < 0
                spoofStartEpoch = currMeasNr;
            end
            alarmRaised = true;
            fprintf('[ALARM] Spoofing detected at epoch %d, Mcm = %.3f Hz, Mdf = %.3f Hz\n', ...
                currMeasNr, metricCommonHz, metricDiffHz);
        end
    else
        confirmCounter = 0;
        if releaseStable
            releaseCounter = releaseCounter + 1;
        else
            releaseCounter = 0;
        end
        if releaseCounter >= releaseConfirmEpochs
            navMode = MODE_GNSS_RAMP;
            rampCounter = 1;
            releaseCounter = 0;
            recoveryAccepted = true;
            fprintf('[RECOVER] GNSS ramp entered at epoch %d, Mcm = %.3f Hz, Mdf = %.3f Hz\n', ...
                currMeasNr, metricCommonHz, metricDiffHz);
        end
    end
    if alarmRaised || ((navMode == MODE_INS_ONLY) && ~isfinite(takeoverForwardSpeedRefMps))
        vbTakeover = ins.Cnb' * ins.vn;
        takeoverForwardSpeedRefMps = vbTakeover(1);
        takeoverHeightRefM = ins.pos(3);
        takeoverPitchRollRefRad = ins.att(1:2);
    end

    modeNormal = (navMode == MODE_NORMAL);
    modeInsTakeover = (navMode == MODE_INS_ONLY);
    modeGnssRamp = (navMode == MODE_GNSS_RAMP);
    gnssPrScale = 1;
    gnssPrrScale = 1;
    rampBlend = 0;
    if modeGnssRamp
        rampFrac = min(1, max(rampCounter, 1) / rampEpochs);
        gnssPrScale = settings.gnssRampPrScaleStart + (1 - settings.gnssRampPrScaleStart) * rampFrac;
        gnssPrrScale = settings.gnssRampPrrScaleStart + (1 - settings.gnssRampPrrScaleStart) * rampFrac;
        rampBlend = max(0, 1 - rampFrac);
    end

    modeTakeoverPrrAssist = false;
    if modeInsTakeover
        dopplerSuppressAlpha = ones(size(dopplerMeasHz));
        allowTakeoverPrrAssist = settings.insTakeoverPrrAssistEnable && ...
                                 localTakeoverPrrAssistAllowed(metricCommonHz, metricDiffHz, settings);
        if allowTakeoverPrrAssist && (validDetNum >= settings.spoofDetMinSat) && isfinite(commonCenterHz)
            cleanTrendHz = baselineSatBiasHz + templateResidualHz(:, currMeasNr);
            cleanTrendHz(~isfinite(cleanTrendHz)) = 0;
            residualDiffHz = dopplerResidualDevHz - commonCenterHz;
            residualDiffHz(~isfinite(residualDiffHz)) = 0;
            clipHz = settings.insTakeoverPrrResidualClipHz;
            residualDiffHz = min(max(residualDiffHz, -clipHz), clipHz);
            dopplerSuppressedHz = dopplerPredRawHz + cleanTrendHz + residualDiffHz;
            modeTakeoverPrrAssist = true;
        else
            dopplerSuppressedHz = dopplerPredRawHz;
        end
    elseif modeGnssRamp
        dopplerSuppressAlpha = rampBlend * ones(size(dopplerMeasHz));
        dopplerSuppressedHz = rampBlend .* dopplerPredRawHz + (1 - rampBlend) .* dopplerMeasHz;
    else
        dopplerSuppressAlpha = zeros(size(dopplerMeasHz));
        dopplerSuppressedHz = dopplerMeasHz;
    end

    gnssUpdateUsed = false;
    qualityFallbackCode = 0;
    modeVirtualNHC = false;
    modeVirtualZUPT = false;
    modeVirtualSpeedHold = false;
    modeVirtualHeightHold = false;
    modeVirtualPitchRollHold = false;
    modeVirtualVertVelHold = false;
    speedHoldResidual = nan;
    heightHoldResidual = nan;
    pitchRollResidualRad = [nan; nan];
    vertVelResidualMps = nan;
    nhcResidualBody = [nan; nan];
    zuptResidualN = [nan; nan; nan];

    if currMeasNr == 1
        navSolut_1 = postNavLoose(trackDeepIn, settings, eph, TOW);
        if ~isempty(navSolut_1)
            for ii = 1 : numActChnList
                trackDeepIn(ii).recvTime = trackDeepIn(ii).recvTime - navSolut_1.dt / settings.c;
            end
        end
    else
        delta_rawP = navSolut.rawP' + settings.c * navSolut.satClkCorr' - geomRhoDet;
        if ~modeInsTakeover
            navMaskPr = localBuildNavMeasurementMask(delta_rawP, AzElDet(:,2), settings.tcNavMinElevDeg, settings.tcNavMinSat);
            if any(navMaskPr)
                el = max(AzElDet(navMaskPr,2), settings.tcNavMinElevDeg * pi / 180);
                Hpr = kfhk(ins, LOSDet(navMaskPr, :));
                Rpr = diag(((settings.tcRangeNoiseStdM.^2) * gnssPrScale) ./ (sin(el).^2));
                [kf, ins, prUpdateUsed, prFailCode] = localGuardedMeasurementUpdate( ...
                    kf, ins, delta_rawP(navMaskPr), Hpr, Rpr, 'M', settings.trustedGnssFeedbackStr, settings);
            else
                prUpdateUsed = false;
                prFailCode = 1;
            end
            gnssUpdateUsed = prUpdateUsed;
            if prUpdateUsed
                for ii = 1 : numActChnList
                    trackDeepIn(ii).recvTime = trackDeepIn(ii).recvTime - kf.xk(end-1) / settings.c;
                end
            else
                qualityFallbackCode = prFailCode;
            end
        end
        if settings.tcUseRangeRateUpdate && (((~modeInsTakeover) && gnssUpdateUsed) || (modeInsTakeover && modeTakeoverPrrAssist))
            [posxyzRate, ~] = blh2xyz(ins.pos);
            antVelNRate = ins.vn;
            if any(abs(leverArm_b) > 0)
                antVelNRate = antVelNRate + ins.Cnb * cross(ins.wib, leverArm_b);
            end
            [~, LOSRate, AzElRate, v_r_s_rate, ~, CenRate] = rhoSatRec_zcj( ...
                navSolut.satPositions', posxyzRate, navSolut.rawP', navSolut.satVelocity', antVelNRate);
            elRate = max(AzElRate(:,2), 15 * pi / 180);
            if modeInsTakeover
                rawPdotAssist = -dopplerSuppressedHz * settings.c / l1Freq;
                delta_rawPdot = rawPdotAssist + settings.c * navSolut.satClkDrift' - v_r_s_rate;
                prrNoiseStd = settings.insTakeoverPrrNoiseStdMps;
            else
                delta_rawPdot = navSolut.rawP_dot' + settings.c * navSolut.satClkDrift' - v_r_s_rate;
                prrNoiseStd = settings.tcRangeRateNoiseStdMps * sqrt(gnssPrrScale);
            end
            navMaskPrr = localBuildNavMeasurementMask(delta_rawPdot, AzElRate(:,2), settings.tcNavMinElevDeg, settings.tcNavMinSat);
            if any(navMaskPrr)
                Hprr = localBuildRangeRateModel(LOSRate(navMaskPrr, :), CenRate, numel(kf.xk));
                Rprr = localBuildRangeRateCov(elRate(navMaskPrr), prrNoiseStd);
                if modeInsTakeover
                    prrFbStr = settings.takeoverPrrFeedbackStr;
                else
                    prrFbStr = settings.trustedGnssFeedbackStr;
                end
                [kf, ins, prrUpdateUsed, ~] = localGuardedMeasurementUpdate( ...
                    kf, ins, delta_rawPdot(navMaskPrr), Hprr, Rprr, 'M', prrFbStr, settings);
            else
                prrUpdateUsed = false;
            end
            gnssUpdateUsed = gnssUpdateUsed || prrUpdateUsed;
        end
    end

    if modeInsTakeover
        if settings.virtualZUPTEnable
            [kf, ins, modeVirtualZUPT, zuptResidualN, zuptFailCode] = localApplyVirtualZUPT(kf, ins, settings);
            if (~modeVirtualZUPT) && (zuptFailCode > 0) && (qualityFallbackCode == 0)
                qualityFallbackCode = zuptFailCode;
            end
        end
        if settings.virtualPitchRollHoldEnable
            [kf, ins, modeVirtualPitchRollHold, pitchRollResidualRad, attFailCode] = localApplyVirtualPitchRollHold(kf, ins, takeoverPitchRollRefRad, settings);
            if (~modeVirtualPitchRollHold) && (attFailCode > 0) && (qualityFallbackCode == 0)
                qualityFallbackCode = attFailCode;
            end
        end
        if settings.virtualVertVelHoldEnable && ~modeVirtualZUPT
            [kf, ins, modeVirtualVertVelHold, vertVelResidualMps, vertFailCode] = localApplyVirtualVertVelHold(kf, ins, settings);
            if (~modeVirtualVertVelHold) && (vertFailCode > 0) && (qualityFallbackCode == 0)
                qualityFallbackCode = vertFailCode;
            end
        end
        if settings.virtualHeightHoldEnable
            [kf, ins, modeVirtualHeightHold, heightHoldResidual, heightFailCode] = localApplyVirtualHeightHold(kf, ins, takeoverHeightRefM, settings);
            if (~modeVirtualHeightHold) && (heightFailCode > 0) && (qualityFallbackCode == 0)
                qualityFallbackCode = heightFailCode;
            end
        end
        if settings.virtualNHCEnable
            [kf, ins, modeVirtualNHC, nhcResidualBody, nhcFailCode] = localApplyVirtualNHC(kf, ins, settings);
            if (~modeVirtualNHC) && (nhcFailCode > 0) && (qualityFallbackCode == 0)
                qualityFallbackCode = nhcFailCode;
            end
        end
        if settings.virtualSpeedHoldEnable && ~modeVirtualZUPT
            [kf, ins, modeVirtualSpeedHold, speedHoldResidual, speedFailCode] = localApplyVirtualSpeedHold(kf, ins, takeoverForwardSpeedRefMps, settings);
            if (~modeVirtualSpeedHold) && (speedFailCode > 0) && (qualityFallbackCode == 0)
                qualityFallbackCode = speedFailCode;
            end
        end
    end

    kinResidualNorm = 0;
    if any(isfinite(nhcResidualBody))
        kinResidualNorm = kinResidualNorm + norm(nhcResidualBody(isfinite(nhcResidualBody)));
    end
    if any(isfinite(zuptResidualN))
        kinResidualNorm = kinResidualNorm + norm(zuptResidualN(isfinite(zuptResidualN)));
    end
    if any(isfinite(pitchRollResidualRad))
        kinResidualNorm = kinResidualNorm + norm(pitchRollResidualRad(isfinite(pitchRollResidualRad)));
    end
    if isfinite(vertVelResidualMps), kinResidualNorm = kinResidualNorm + abs(vertVelResidualMps); end
    if isfinite(speedHoldResidual), kinResidualNorm = kinResidualNorm + abs(speedHoldResidual); end
    if isfinite(heightHoldResidual), kinResidualNorm = kinResidualNorm + abs(heightHoldResidual); end

    navResults.modeNormal(1, currMeasNr) = modeNormal;
    navResults.modeInsOnly(1, currMeasNr) = modeInsTakeover;
    navResults.modeInsTakeover(1, currMeasNr) = modeInsTakeover;
    navResults.modeGnssRamp(1, currMeasNr) = modeGnssRamp;
    navResults.modeKinConstraint(1, currMeasNr) = modeVirtualNHC || modeVirtualZUPT || modeVirtualSpeedHold || modeVirtualHeightHold || modeVirtualPitchRollHold || modeVirtualVertVelHold;
    navResults.modeVirtualNHC(1, currMeasNr) = modeVirtualNHC;
    navResults.modeVirtualZUPT(1, currMeasNr) = modeVirtualZUPT;
    navResults.modeVirtualSpeedHold(1, currMeasNr) = modeVirtualSpeedHold;
    navResults.modeVirtualHeightHold(1, currMeasNr) = modeVirtualHeightHold;
    navResults.modeVirtualPitchRollHold(1, currMeasNr) = modeVirtualPitchRollHold;
    navResults.modeVirtualVertVelHold(1, currMeasNr) = modeVirtualVertVelHold;
    navResults.modeTakeoverPrrAssist(1, currMeasNr) = modeTakeoverPrrAssist;
    navResults.kinCorrectionNorm(1, currMeasNr) = kinResidualNorm;

    navResults.spoofFlag(1, currMeasNr) = ~modeNormal;
    navResults.spoofAlarm(1, currMeasNr) = alarmRaised;
    navResults.recoveryCandidate(1, currMeasNr) = releaseStable;
    navResults.recoveryCounter(1, currMeasNr) = releaseCounter;
    navResults.modeState(1, currMeasNr) = navMode;

    navResults.dopplerGpsMeasHz(:, currMeasNr) = dopplerMeasHz;
    navResults.dopplerInsPredHz(:, currMeasNr) = dopplerPredRawHz;
    navResults.dopplerRecvClkHz(1, currMeasNr) = recvClkDopplerHz;
    navResults.dopplerPredGeomHz(:, currMeasNr) = dopplerPredGeomHz;
    navResults.dopplerSatClkHz(:, currMeasNr) = satClkDopplerHz;
    navResults.dopplerResidualRawHz(:, currMeasNr) = dopplerResidualRawHz;
    navResults.dopplerResidualBiasCorrectedHz(:, currMeasNr) = dopplerResidualBcHz;
    navResults.dopplerResidualTemplateDevHz(:, currMeasNr) = dopplerResidualDevHz;
    navResults.dopplerResidualHz(:, currMeasNr) = dopplerResidualDevHz;
    navResults.dopplerSuppressedHz(:, currMeasNr) = dopplerSuppressedHz;
    navResults.dopplerSuppressAlpha(:, currMeasNr) = dopplerSuppressAlpha;

    navResults.detectorArmed(1, currMeasNr) = detectorArmed;
    navResults.detectorCauseCode(1, currMeasNr) = detectorCause;
    navResults.detectorMetricCommonRawHz(1, currMeasNr) = metricCommonRawHz;
    navResults.detectorMetricCommonHz(1, currMeasNr) = metricCommonHz;
    navResults.detectorMetricDiffRawHz(1, currMeasNr) = metricDiffRawHz;
    navResults.detectorMetricDiffHz(1, currMeasNr) = metricDiffHz;
    navResults.detectorHitSatNum(1, currMeasNr) = hitSatNum;
    navResults.detectorReleaseHitSatNum(1, currMeasNr) = releaseHitSatNum;
    navResults.detectorValidSatNum(1, currMeasNr) = validDetNum;

    navResults.mitigationLevel(1, currMeasNr) = navMode;
    navResults.qualityFallback(1, currMeasNr) = (qualityFallbackCode > 0);
    navResults.qualityFallbackCode(1, currMeasNr) = qualityFallbackCode;
    navResults.gnssUpdateUsed(1, currMeasNr) = gnssUpdateUsed;
    navResults.gnssWeightScalePr(1, currMeasNr) = gnssPrScale;
    navResults.gnssWeightScalePrr(1, currMeasNr) = gnssPrrScale;

    if modeGnssRamp
        if rampCounter >= rampEpochs
            navMode = MODE_NORMAL;
            rampCounter = 0;
            confirmCounter = 0;
            fprintf('[RECOVER] GNSS ramp completed at epoch %d.\n', currMeasNr);
        else
            rampCounter = rampCounter + 1;
        end
    end

    [posX, posY, posZ] = geo2cart(ins.avp(7,1), ins.avp(8,1), ins.avp(9,1), 5);
    Cenu2xyz = [-sin(ins.pos(2))                  cos(ins.pos(2))   0; ...
                -sin(ins.pos(1))*cos(ins.pos(2)) -sin(ins.pos(1))*sin(ins.pos(2))  cos(ins.pos(1)); ...
                 cos(ins.pos(1))*cos(ins.pos(2))  cos(ins.pos(1))*sin(ins.pos(2))  sin(ins.pos(1))];
    vxyz = Cenu2xyz' * ins.vn;
    navResults.X(1, currMeasNr) = posX;
    navResults.Y(1, currMeasNr) = posY;
    navResults.Z(1, currMeasNr) = posZ;
    navResults.VX(1, currMeasNr) = vxyz(1);
    navResults.VY(1, currMeasNr) = vxyz(2);
    navResults.VZ(1, currMeasNr) = vxyz(3);
    if currMeasNr == 1 && ~isempty(navSolut_1)
        navResults.dt(1, currMeasNr) = navSolut_1.dt;
        navResults.df(1, currMeasNr) = navSolut_1.df;
    else
        navResults.dt(1, currMeasNr) = kf.xk(end-1);
        navResults.df(1, currMeasNr) = kf.xk(end);
    end

    positioningTime = positioningTime + settings.navSolPeriod / 1000;

    while (t0_gps + (t - t0_imu)) < positioningTime
        k1 = k + nn - 1;
        if k1 > imuLen
            stopEpoch = currMeasNr - 1;
            break;
        end
        wvm = trj.imu(k:k1,1:6);  t = trj.imu(k1,end);
        ins = insupdate(ins, wvm);
        kf.Phikk_1 = kffk(ins);
        kf = kfupdate(kf);
        k = k + nn;
    end
    if stopEpoch < currMeasNr
        disp('IMU data exhausted, stopping tight coupling early.');
        break;
    end

    for ii = 1 : numActChnList
        trackans = trackDeepIn(ii);
        while trackans.recvTime < positioningTime
            trackDeepIn(ii) = trackans;
            if settings.offlineReplayFromTrackResults
                srcCh = trackans.srcCh;
                idxSrc = BitSyncTime(srcCh) + trackans.numOfCoInt + 1;
                if idxSrc > numel(trackResults(srcCh).absoluteSample)
                    stopEpoch = currMeasNr - 1;
                    break;
                end
                trackans.numOfCoInt = trackans.numOfCoInt + 1;
                trackans.SamplePos = trackResults(srcCh).absoluteSample(idxSrc);
                trackans.codeFreq = trackResults(srcCh).codeFreq(idxSrc);
                trackans.remCodePhase = trackResults(srcCh).remCodePhase(idxSrc);
                trackans.carrFreq = trackResults(srcCh).carrFreq(idxSrc);
                trackans.remCarrPhase = trackResults(srcCh).remCarrPhase(idxSrc);
                trackans.codeError = trackResults(srcCh).dllDiscr(idxSrc);
                trackans.codeNco = trackResults(srcCh).dllDiscrFilt(idxSrc);
                trackans.carrError = trackResults(srcCh).pllDiscr(idxSrc);
                trackans.carrNco = trackResults(srcCh).pllDiscrFilt(idxSrc);
                trackans.recvTime = recvTimeforFirstFrameperChannel(ii) + ...
                                    (trackans.SamplePos - SamplePosatFirstFrame(ii)) / settings.samplingFreq;
                I_P = 0;
                Q_P = 0;
            else
                [trackans, I_P, Q_P] = perChannelTrackOnce(trackans, settings, fid);
            end

            if collectTrackDebug && (trackans.recvTime < positioningTime)
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
        if stopEpoch < currMeasNr
            break;
        end
    end
    if stopEpoch < currMeasNr
        disp('GNSS trackResults exhausted, stopping tight coupling early.');
        break;
    end
end

if stopEpoch < roundTime
    navResults = localTrimNavResults(navResults, stopEpoch, roundTime);
end

if spoofStartEpoch > 0
    fprintf('[ALARM] INS takeover entered at epoch %d.\n', spoofStartEpoch);
else
    disp('[INFO] No spoofing alarm triggered during this run.');
end
if any(navResults.modeGnssRamp)
    fprintf('[INFO] GNSS ramp epochs = %d\n', sum(navResults.modeGnssRamp));
end
if any(navResults.modeVirtualNHC)
    fprintf('[INFO] Virtual NHC epochs = %d\n', sum(navResults.modeVirtualNHC));
end
if any(navResults.modeVirtualZUPT)
    fprintf('[INFO] Virtual ZUPT epochs = %d\n', sum(navResults.modeVirtualZUPT));
end
if any(navResults.qualityFallback)
    fprintf('[INFO] Quality-fallback epochs = %d (invalid=%d, kf-fail=%d, nhc-fail=%d, zupt-fail=%d).\n', ...
        sum(navResults.qualityFallback), ...
        sum(navResults.qualityFallbackCode == 1), ...
        sum(navResults.qualityFallbackCode == 2), ...
        sum(navResults.qualityFallbackCode == 5), ...
        sum(navResults.qualityFallbackCode == 6));
end

%% Quick result plots (skip invalid first solution)
if exist('navResults', 'var') && ~settings.skipQuickPlot
    X = navResults.X; Y = navResults.Y; Z = navResults.Z;
    r = sqrt(X.^2 + Y.^2 + Z.^2);
    idx = find(isfinite(r) & (r > 1e6));
    if numel(idx) >= 2
        Xv = X(idx); Yv = Y(idx); Zv = Z(idx);
        dX = Xv - Xv(1); dY = Yv - Yv(1); dZ = Zv - Zv(1);
        figure; plot(dX, dY); axis equal; grid on;
        title('Relative XY (valid only)');

        [lat0, lon0, ~] = cart2geo(Xv(1), Yv(1), Zv(1), 5);
        lat0 = deg2rad(lat0); lon0 = deg2rad(lon0);
        R = [-sin(lon0)  cos(lon0) 0; ...
             -sin(lat0)*cos(lon0) -sin(lat0)*sin(lon0) cos(lat0); ...
              cos(lat0)*cos(lon0)  cos(lat0)*sin(lon0) sin(lat0)];
        enu = R * [dX; dY; dZ];
        figure; plot(enu(1,:), enu(2,:)); axis equal; grid on;
        title('ENU Trajectory'); xlabel('E (m)'); ylabel('N (m)');

        maxDisp = max(sqrt(dX.^2 + dY.^2));
        fprintf('max horizontal displacement (valid only) = %.2f m\n', maxDisp);
    else
        disp('Not enough valid navResults points for plotting.');
    end
end

function allowed = localTakeoverPrrAssistAllowed(metricCommonHz, metricDiffHz, settings)
policy = lower(strtrim(settings.insTakeoverPrrAssistPolicy));
switch policy
    case 'off'
        allowed = false;
    case 'conditional'
        allowed = isfinite(metricCommonHz) && isfinite(metricDiffHz) && ...
                  (metricDiffHz >= settings.insTakeoverPrrAssistMinDiffHz) && ...
                  ((metricCommonHz - metricDiffHz) <= settings.insTakeoverPrrAssistMcmOverDiffMarginHz);
    otherwise
        allowed = true;
end
end

function valMap = localMapBaselineByPrn(currPrnList, basePrnList, baseVals, defaultVal)
basePrnList = basePrnList(:);
baseVals = baseVals(:);
valMap = defaultVal * ones(numel(currPrnList), 1);
if isempty(basePrnList) || isempty(baseVals)
    return;
end
for ii = 1 : numel(currPrnList)
    idx = find(basePrnList == currPrnList(ii), 1, 'first');
    if ~isempty(idx) && idx <= numel(baseVals) && isfinite(baseVals(idx))
        valMap(ii) = baseVals(idx);
    end
end
end

function templateMap = localMapTemplateByPrn(currPrnList, templatePrnList, templateResidualSource, roundTime, epochOffset)
templateMap = zeros(numel(currPrnList), roundTime);
if nargin < 5 || isempty(epochOffset)
    epochOffset = 0;
end
if isempty(templatePrnList) || isempty(templateResidualSource)
    return;
end

templatePrnList = templatePrnList(:);
if ~ismatrix(templateResidualSource)
    return;
end

epochOffset = round(epochOffset);
srcEpochNum = size(templateResidualSource, 2);
dstStart = 1 + max(-epochOffset, 0);
srcStart = 1 + max(epochOffset, 0);
copyEpochNum = min(roundTime - dstStart + 1, srcEpochNum - srcStart + 1);
if copyEpochNum <= 0
    return;
end

dstIdx = dstStart : (dstStart + copyEpochNum - 1);
srcIdx = srcStart : (srcStart + copyEpochNum - 1);
for ii = 1 : numel(currPrnList)
    idx = find(templatePrnList == currPrnList(ii), 1, 'first');
    if isempty(idx) || idx > size(templateResidualSource, 1)
        continue;
    end
    rowVals = templateResidualSource(idx, srcIdx);
    rowVals(~isfinite(rowVals)) = 0;
    templateMap(ii, dstIdx) = rowVals;
end
end

function Hk = localBuildRangeRateModel(LOS, Cne, nStates)
m = size(LOS, 1);
Hk = zeros(m, nStates);
Hk(:, 4:6) = LOS * Cne;
Hk(:, end) = 1;
end

function Rk = localBuildRangeRateCov(elRad, noiseStdMps)
elRad = elRad(:);
if isscalar(noiseStdMps)
    sigma = repmat(noiseStdMps, numel(elRad), 1);
else
    sigma = noiseStdMps(:);
    if numel(sigma) ~= numel(elRad)
        sigma = repmat(sigma(1), numel(elRad), 1);
    end
end
Rk = diag((sigma.^2) ./ (sin(elRad).^2));
end

function navMask = localBuildNavMeasurementMask(residual, elRad, minElevDeg, minSat)
residual = residual(:);
elRad = elRad(:);
navMask = isfinite(residual) & isfinite(elRad) & (elRad >= minElevDeg * pi / 180);
if nnz(navMask) < minSat
    navMask(:) = false;
end
end

function kf = localSetMeasurementModel(kf, Hk, Rk)
[kf.m, n] = size(Hk);
if n ~= numel(kf.xk)
    error('Measurement model state dimension mismatch.');
end
kf.Hk = Hk;
kf.Rk = Rk;
kf.Kk = zeros(numel(kf.xk), kf.m);
kf.measstop = zeros(kf.m, 1);
kf.measlost = zeros(kf.m, 1);
end

function [kf, ins, used, failCode] = localGuardedMeasurementUpdate(kf, ins, yk, Hk, Rk, measMode, fbstr, settings)
used = false;
failCode = 0;
if isempty(yk) || isempty(Hk) || isempty(Rk) || ...
   ~all(isfinite(yk(:))) || ~all(isfinite(Hk(:))) || ~all(isfinite(Rk(:)))
    failCode = 1;
    return;
end
if any(diag(Rk) <= 0)
    failCode = 1;
    return;
end
kfPrev = kf;
insPrev = ins;
kf = localSetMeasurementModel(kf, Hk, Rk);

guardWarnModified = false;
if settings.kfGuardEnable
    warnSing = warning('query', 'MATLAB:singularMatrix');
    warnNear = warning('query', 'MATLAB:nearlySingularMatrix');
    warnIll  = warning('query', 'MATLAB:illConditionedMatrix');
    warning('error', 'MATLAB:singularMatrix');
    warning('error', 'MATLAB:nearlySingularMatrix');
    warning('error', 'MATLAB:illConditionedMatrix');
    guardWarnModified = true;
end

try
    kf = kfupdate(kf, yk, measMode);
    [kf, ins] = kffeedback(kf, ins, settings.navSolPeriod / 1000, fbstr);
    stateFinite = all(isfinite(kf.xk)) && all(abs(kf.xk) <= settings.kfGuardStateAbsMax);
    insFinite = all(isfinite(ins.avp(:))) && all(isfinite(ins.vn(:)));
    if ~stateFinite || ~insFinite
        error('KFGuard:InvalidState', 'KF update generated non-finite/diverged state.');
    end
    used = true;
catch
    kf = kfPrev;
    ins = insPrev;
    failCode = 2;
end

if guardWarnModified
    warning(warnSing.state, 'MATLAB:singularMatrix');
    warning(warnNear.state, 'MATLAB:nearlySingularMatrix');
    warning(warnIll.state, 'MATLAB:illConditionedMatrix');
end
end

function insOut = localPerturbInsError(insIn, dx)
insOut = insIn;
if any(dx(1:3) ~= 0)
    insOut.qnb = qaddphi(insOut.qnb, dx(1:3));
    [insOut.qnb, insOut.att, insOut.Cnb] = attsyn(insOut.qnb);
end
if any(dx(4:6) ~= 0)
    insOut.vn = insOut.vn + dx(4:6);
    insOut.avp(4:6) = insOut.vn;
    insOut.Mpvvn = insOut.Mpv * insOut.vn;
end
end

function residual = localNHCResidual(ins)
vb = ins.Cnb' * ins.vn;
residual = -vb(2:3);
end

function H = localFiniteDiffH(ins, residualFcn, nStates)
r0 = residualFcn(ins);
measDim = numel(r0);
H = zeros(measDim, nStates);
steps = [1e-7; 1e-7; 1e-7; 1e-4; 1e-4; 1e-4];
for idx = 1 : min(6, nStates)
    dx = zeros(6,1);
    dx(idx) = steps(idx);
    rp = residualFcn(localPerturbInsError(ins, dx));
    rm = residualFcn(localPerturbInsError(ins, -dx));
    H(:, idx) = (rp - rm) / (2 * steps(idx));
end
end

function [kf, ins, applied, residualBody, failCode] = localApplyVirtualNHC(kf, ins, settings)
applied = false;
failCode = 0;
residualBody = [nan; nan];
vb = ins.Cnb' * ins.vn;
if norm(vb(1:2)) < settings.virtualNHCMinHorSpeed
    return;
end
residualBody = -vb(2:3);
Hk = localFiniteDiffH(ins, @localNHCResidual, numel(kf.xk));
noiseStd = settings.virtualNHCNoiseStdMps(:);
if numel(noiseStd) == 1
    noiseStd = repmat(noiseStd, 2, 1);
end
Rk = diag(noiseStd(1:2).^2);
[kf, ins, applied, failCode] = localGuardedMeasurementUpdate(kf, ins, residualBody, Hk, Rk, 'M', 'avped', settings);
if ~applied && failCode == 0
    failCode = 5;
end
end

function [kf, ins, applied, residualN, failCode] = localApplyVirtualZUPT(kf, ins, settings)
applied = false;
failCode = 0;
residualN = [nan; nan; nan];
if norm(ins.vn) > settings.virtualZUPTSpeedTh
    return;
end
residualN = -ins.vn;
Hk = zeros(3, numel(kf.xk));
Hk(:, 4:6) = -eye(3);
noiseStd = settings.virtualZUPTNoiseStdMps(:);
if numel(noiseStd) == 1
    noiseStd = repmat(noiseStd, 3, 1);
end
Rk = diag(noiseStd(1:3).^2);
[kf, ins, applied, failCode] = localGuardedMeasurementUpdate(kf, ins, residualN, Hk, Rk, 'M', 'avped', settings);
if ~applied && failCode == 0
    failCode = 6;
end
end

function residual = localForwardSpeedResidual(ins, speedRef)
vb = ins.Cnb' * ins.vn;
residual = vb(1) - speedRef;
end

function [kf, ins, applied, residualFwd, failCode] = localApplyVirtualSpeedHold(kf, ins, speedRef, settings)
applied = false;
failCode = 0;
residualFwd = nan;
if ~isfinite(speedRef) || (abs(speedRef) < settings.virtualSpeedHoldMinMps)
    return;
end
residualFwd = localForwardSpeedResidual(ins, speedRef);
Hk = localFiniteDiffH(ins, @(insArg) localForwardSpeedResidual(insArg, speedRef), numel(kf.xk));
Rk = settings.virtualSpeedHoldNoiseStdMps.^2;
[kf, ins, applied, failCode] = localGuardedMeasurementUpdate(kf, ins, residualFwd, Hk, Rk, 'M', 'avped', settings);
if ~applied && failCode == 0
    failCode = 7;
end
end

function [kf, ins, applied, residualH, failCode] = localApplyVirtualHeightHold(kf, ins, heightRef, settings)
applied = false;
failCode = 0;
residualH = nan;
if ~isfinite(heightRef)
    return;
end
residualH = ins.pos(3) - heightRef;
Hk = zeros(1, numel(kf.xk));
Hk(9) = 1;
Rk = settings.virtualHeightHoldNoiseStdM.^2;
[kf, ins, applied, failCode] = localGuardedMeasurementUpdate(kf, ins, residualH, Hk, Rk, 'M', 'avped', settings);
if ~applied && failCode == 0
    failCode = 8;
end
end

function [kf, ins, applied, residualRP, failCode] = localApplyVirtualPitchRollHold(kf, ins, attRefRad, settings)
applied = false;
failCode = 0;
residualRP = [nan; nan];
if numel(attRefRad) < 2 || ~all(isfinite(attRefRad(1:2)))
    return;
end
residualRP = attRefRad(1:2) - ins.att(1:2);
Hk = zeros(2, numel(kf.xk));
Hk(:, 1:2) = -eye(2);
noiseStd = settings.virtualPitchRollHoldNoiseStdRad(:);
if numel(noiseStd) == 1
    noiseStd = repmat(noiseStd, 2, 1);
end
Rk = diag(noiseStd(1:2).^2);
[kf, ins, applied, failCode] = localGuardedMeasurementUpdate(kf, ins, residualRP, Hk, Rk, 'M', 'avped', settings);
if ~applied && failCode == 0
    failCode = 9;
end
end

function [kf, ins, applied, residualVz, failCode] = localApplyVirtualVertVelHold(kf, ins, settings)
applied = false;
failCode = 0;
residualVz = nan;
residualVz = -ins.vn(3);
Hk = zeros(1, numel(kf.xk));
Hk(6) = -1;
Rk = settings.virtualVertVelHoldNoiseStdMps.^2;
[kf, ins, applied, failCode] = localGuardedMeasurementUpdate(kf, ins, residualVz, Hk, Rk, 'M', 'avped', settings);
if ~applied && failCode == 0
    failCode = 10;
end
end

function navResults = localTrimNavResults(navResults, stopEpoch, fullEpoch)
fn = fieldnames(navResults);
for ii = 1 : numel(fn)
    val = navResults.(fn{ii});
    if ~(isnumeric(val) || islogical(val))
        continue;
    end
    if ismatrix(val) && size(val, 2) == fullEpoch
        navResults.(fn{ii}) = val(:, 1:stopEpoch);
        if size(val, 1) == 1
            navResults.(fn{ii}) = navResults.(fn{ii})(1, :);
        end
    end
end
end



function [avpOut, used] = localOverrideAvpPosVelFromDataset(avpIn, datasetInitNav, towSec)
avpOut = avpIn;
used = false;
requiredFields = {'X', 'Y', 'Z', 'VX', 'VY', 'VZ'};
for ii = 1 : numel(requiredFields)
    if ~isfield(datasetInitNav, requiredFields{ii}) || isempty(datasetInitNav.(requiredFields{ii}))
        return;
    end
end

x = datasetInitNav.X(:);
y = datasetInitNav.Y(:);
z = datasetInitNav.Z(:);
vx = datasetInitNav.VX(:);
vy = datasetInitNav.VY(:);
vz = datasetInitNav.VZ(:);
valid = isfinite(x) & isfinite(y) & isfinite(z) & isfinite(vx) & isfinite(vy) & isfinite(vz);
if isfield(datasetInitNav, 'receiverTime') && ~isempty(datasetInitNav.receiverTime)
    recvTime = datasetInitNav.receiverTime(:);
    valid = valid & isfinite(recvTime);
else
    recvTime = nan(size(x));
end
if ~any(valid)
    return;
end

validIdx = find(valid);
if any(isfinite(recvTime(validIdx)))
    [~, kLocal] = min(abs(recvTime(validIdx) - towSec));
    idx = validIdx(kLocal);
else
    idx = validIdx(1);
end

if isfinite(recvTime(idx))
    dtSec = towSec - recvTime(idx);
    x0 = x(idx) + vx(idx) * dtSec;
    y0 = y(idx) + vy(idx) * dtSec;
    z0 = z(idx) + vz(idx) * dtSec;
else
    x0 = x(idx);
    y0 = y(idx);
    z0 = z(idx);
end

[latDeg, lonDeg, h] = cart2geo(x0, y0, z0, 5);
lat = latDeg * pi / 180;
lon = lonDeg * pi / 180;
Cenu2xyz = [-sin(lon),                  cos(lon),                  0; ...
            -sin(lat) * cos(lon), -sin(lat) * sin(lon),  cos(lat); ...
             cos(lat) * cos(lon),  cos(lat) * sin(lon),  sin(lat)];
vn = Cenu2xyz * [vx(idx); vy(idx); vz(idx)];

avpOut(4:6) = vn;
avpOut(7:9) = [lat; lon; h];
used = all(isfinite(avpOut(4:9)));
if ~used
    avpOut = avpIn;
end
end

