%% GNSS INS 深组合，在紧组合的基础上对跟踪环路进行辅助
% trackResults, channel, TOW, eph, subFrameStart 需要提前准备好
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

%% INS模块初始化
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

%% INS运行至首次组合时刻
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


%% GNSS跟踪结构体初始化
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

%% GNSS运行至于首次组合时刻
I_P_1_list = [];
Q_P_1_list = [];


for ii = 1 : numActChnList
    trackans = trackDeepIn(ii);
    while trackans.recvTime < positioningTime    
        trackDeepIn(ii) = trackans;
        [trackans, I_P, Q_P] = perChannelTrackOnce(trackans, settings, fid);
        
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
    trackDeepIn(ii) = trackans;
end
trackDeepDet = trackDeepIn;


%% 深组合
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

settings.pllNoiseBandwidth = 3;    % 对于test_522这组数据，PLL的噪声带宽为3时刚刚失锁
settings.dllNoiseBandwidth = 2;

oldAidFreq = zeros(numActChnList, 1);
confirmCnt = 0;
if ~isfield(settings, 'deepSpoofDetectEnable'), settings.deepSpoofDetectEnable = 0; end
if ~isfield(settings, 'deepTcmHz'), settings.deepTcmHz = 5.0; end
if ~isfield(settings, 'deepTdfHz'), settings.deepTdfHz = 3.0; end
if ~isfield(settings, 'deepTsatHz'), settings.deepTsatHz = 8.0; end
if ~isfield(settings, 'deepHitMin'), settings.deepHitMin = 3; end
if ~isfield(settings, 'deepConfirmEpochs'), settings.deepConfirmEpochs = 3; end
if ~isfield(settings, 'deepDetectArmEpoch'), settings.deepDetectArmEpoch = 1; end
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
if ~isfield(settings, 'deepRScaleNormal'), settings.deepRScaleNormal = 1.0; end
if ~isfield(settings, 'deepRScaleSuspect'), settings.deepRScaleSuspect = 4.0; end
if ~isfield(settings, 'deepRScaleSpoof'), settings.deepRScaleSpoof = 10.0; end
if ~isfield(settings, 'deepUseReferenceResidual'), settings.deepUseReferenceResidual = 0; end
if ~isfield(settings, 'deepRefResidualHz'), settings.deepRefResidualHz = []; end
if ~isfield(settings, 'deepDetectUseTrackResults'), settings.deepDetectUseTrackResults = 1; end
if ~isfield(settings, 'deepFastMode'), settings.deepFastMode = 1; end
if ~isfield(settings, 'deepDetrendWindowEpoch'), settings.deepDetrendWindowEpoch = 80; end
lambdaL1 = settings.c / 1575.42e6;
maxPrnId = max(64, max([trackDeepIn.PRN]));
rhoCorrByPrn = nan(1, maxPrnId);
deepModeState = 0;  % 0 normal, 1 suspect, 2 spoof
navResults.metricZ = nan(1, roundTime);
navResults.hitCount = zeros(1, roundTime);
navResults.prrCorrBlend = zeros(1, roundTime);
navResults.rhoCorrRmsM = nan(1, roundTime);
navResults.prrCorrRmsMps = nan(1, roundTime);
navResults.detectArmed = false(1, roundTime);
navResults.gnssKfUpdateUsed = false(1, roundTime);
navResults.clockKfUpdateUsed = false(1, roundTime);

for currMeasNr = 1 : roundTime
    currMeasNr;
    settings.recvTime = positioningTime;    
    settings.deepModeState = deepModeState;
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
    if useGnssKfUpdateNow
        useKfClockUpdateNow = false;
    end
    navResults.prrCorrBlend(1, currMeasNr) = rhoBlendNow;
    isArmed = (currMeasNr >= settings.deepDetectArmEpoch);
    navResults.detectArmed(1, currMeasNr) = isArmed;
    
    % 先进行一次单点定位，缩小时间差
    if currMeasNr < 2
        navSolut_1 = postNavLoose(trackDeepIn, settings, eph, TOW);
        positioningTime = positioningTime + settings.navSolPeriod / 1000;
        
        % 钟差修正
        for ii = 1 : numActChnList
            trackDeepIn(ii).recvTime = trackDeepIn(ii).recvTime - navSolut_1.dt / settings.c;  
        end
        
        % 惯导运行至于下一次组合时刻
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
        
        % GNSS 运行至于下一次组合时刻
        for ii = 1 : numActChnList
            trackans = trackDeepIn(ii);                
            
            oldAidFreq(ii,1) = trackans.carrFreq - settings.IF;      
            
            while trackans.recvTime < positioningTime     
                trackDeepIn(ii) = trackans;
                [trackans, I_P, Q_P] = perChannelTrackOnce(trackans, settings, fid);             

                if trackans.recvTime < positioningTime
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
             
    %% 开始紧组合
    % 1. GNSS观测值
    navSolut = postNavTight(trackDeepIn, settings, eph, TOW);
    navSolutDet = navSolut;
    if settings.deepUseIndependentDetectTrack
        settingsDet.recvTime = settings.recvTime;
        navSolutDet = postNavTight(trackDeepDet, settingsDet, eph, TOW);
    end
    % 只保留一颗卫星观测值
%     navSolut.rawP = navSolut.rawP(1); navSolut.satPositions = navSolut.satPositions(:,1); 
%     navSolut.satVelocity = navSolut.satVelocity(:,1);  navSolut.satClkCorr = navSolut.satClkCorr(1); 
    
    [posxyz, ~] = blh2xyz(ins.pos);
    [rho, LOS, AzEl, vrs, ~, Cen] = rhoSatRec_zcj(navSolut.satPositions', posxyz, navSolut.rawP', navSolut.satVelocity', ins.vn);
    dopplerFeedback = -vrs / settings.c * 1575.42e6;   % numOfSat * 1
    [~, LOSDet, ~, vrsDet, ~, CenDet] = rhoSatRec_zcj(navSolutDet.satPositions', posxyz, ...
        navSolutDet.rawP', navSolutDet.satVelocity', ins.vn);

    % Receiver clock drift term in Hz (for deep-loop NCO synthesis).
    clkDriftHz = 0;
    if isfield(kf, 'xk') && ~isempty(kf.xk)
        clkDriftMps = kf.xk(end);
        clkDriftHz = clkDriftMps / lambdaL1;
    end
    if isfinite(settings.deepClkDriftHzLimit)
        clkDriftHz = max(-settings.deepClkDriftHzLimit, min(settings.deepClkDriftHzLimit, clkDriftHz));
    end
    navResults.deepClkDriftHz(1, currMeasNr) = clkDriftHz;

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
    W = diag(sin(el.^2));
    delta_rawP = rawPUsed + settings.c * navSolut.satClkCorr(:) - rho;

    % 2. EKF
    if useGnssKfUpdateNow
        if ~isfield(kf, 'Phikk_1') || isempty(kf.Phikk_1)
            kf.Phikk_1 = kffk(ins);
        end
        kf.Hk = kfhk(ins, LOS);
        kf.Rk = (W^-1 * 10^2) * max(1e-3, rScaleNow);
        kf = kfupdate(kf, delta_rawP);
        [kf, ins] = kffeedback(kf, ins, 1, 'avp');
    elseif useKfClockUpdateNow
        if ~isfield(kf, 'Phikk_1') || isempty(kf.Phikk_1)
            kf.Phikk_1 = kffk(ins);
        end
        kf.Hk = kfhk(ins, LOS);
        kf.Rk = (W^-1 * 10^2) * max(1e-3, rScaleNow);
        kf = kfupdate(kf, delta_rawP);
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

        hitCntRes = sum(abs(residualDetFull(validDopp)) > settings.deepTsatHz);
        hitCntZ = sum(zNorm(validDopp) > settings.deepKappaZ, 'omitnan');
        navResults.hitCount(1, currMeasNr) = max(hitCntRes, hitCntZ);

        if settings.deepSpoofDetectEnable && isArmed
            trig = ((navResults.metricCmHz(1, currMeasNr) > settings.deepTcmHz) || ...
                    (navResults.metricDfHz(1, currMeasNr) > settings.deepTdfHz) || ...
                    (navResults.metricZ(1, currMeasNr) > settings.deepTz)) && ...
                    (navResults.hitCount(1, currMeasNr) >= settings.deepHitMin);
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
            if confirmCnt >= settings.deepConfirmEpochs
                deepModeState = 2;
            end
            if settings.deepSpoofLatch && currMeasNr > 1 && navResults.spoofState(1, currMeasNr-1) == 2
                deepModeState = 2;
            end
        elseif ~isArmed && deepModeState < 2
            confirmCnt = 0;
            deepModeState = 0;
        end
    end
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

    aidFreq = dopplerFeedback;
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
    
    %% 深组合反馈模块
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
        while (t0_gps + (t - t0_imu)) < positioningTime
            
            % 沈聪的操作，可防止carrNco过大，我很难领悟其中奥妙
            if currMeasNr == 2
                for iii = 1 : numActChnList
                    trackDeepIn(iii).carrNco = 0;
                end
            end
                   
            % 6.1 跳过当前相干积分
            for ii = 1 : numActChnList    
                % 该方程可参考一些论文，总之正确性有待验证
                [trackDeepIn(ii), I_P, Q_P] = perChannelTrackOnce_DeepIn(trackDeepIn(ii), settings, fid, oldAidFreq(ii), aidFreq(ii) - oldAidFreq(ii));
                if settings.deepUseIndependentDetectTrack
                    [trackDeepDet(ii), ~, ~] = perChannelTrackOnce(trackDeepDet(ii), settingsDet, fidDet);
                end
                
                if ii == 1
                    I_P_1_list = [I_P_1_list, I_P];
                    Q_P_1_list = [Q_P_1_list, Q_P];
                end
                
                trackProcess(ii).codeErrorList = [trackProcess(ii).codeErrorList, trackDeepIn(ii).codeError];
                trackProcess(ii).carrErrorList = [trackProcess(ii).carrErrorList, trackDeepIn(ii).carrError];
                trackProcess(ii).codeFreqList = [trackProcess(ii).codeFreqList, trackDeepIn(ii).codeFreq];
                trackProcess(ii).carrFreqList = [trackProcess(ii).carrFreqList, trackDeepIn(ii).carrFreq];   
                trackProcess(ii).PLI = [trackProcess(ii).PLI, (I_P^2-Q_P^2)/(I_P^2+Q_P^2)];
            end
            
            % 6.3 惯导运行至下一次更新时刻，即惯导更新一次
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
            
            % 6.4 跟踪环运行至惯导更新时刻
            for ii = 1 : numActChnList
                trackans = trackDeepIn(ii);
                if settings.deepUseIndependentDetectTrack
                    trackansDet = trackDeepDet(ii);
                end
                
                kkk = 1;
                
                imuGpsTime = t0_gps + (t - t0_imu);
                while trackans.recvTime < imuGpsTime
                    trackDeepIn(ii) = trackans;
                    % 可参考某些论文，我也只是懂个大概
                    [trackans, I_P, Q_P] = perChannelTrackOnce_DeepIn(trackans, settings, fid, oldAidFreq(ii), kkk * (aidFreq(ii) - oldAidFreq(ii)));
                    if settings.deepUseIndependentDetectTrack
                        [trackansDet, ~, ~] = perChannelTrackOnce(trackansDet, settingsDet, fidDet);
                    end
                    
                    kkk = kkk + 1;

                    if trackans.recvTime < imuGpsTime
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
                  
            
            % 6.5 重新计算惯导的反馈量
            settings.recvTime = settings.recvTime + nts;
            navSolut = postNavTight(trackDeepIn, settings, eph, TOW);  % GNSS观测值
            [posxyz, ~] = blh2xyz(ins.pos);
            % correctedP = navSolut.rawP + navSolut.satClkCorr .* settings.c - kf.xk(end-1);  
            correctedP = navSolut.rawP;  % 和上一行区别不大
            [~, ~, ~, vrs] = rhoSatRec_zcj(navSolut.satPositions', posxyz, correctedP', navSolut.satVelocity', ins.vn);
            dopplerFeedback = -vrs / settings.c * 1575.42e6;  
            
            oldAidFreq = aidFreq;
            aidFreq = dopplerFeedback;
        end
    end
       
    
    end  % if currMeasNr == 1
end
