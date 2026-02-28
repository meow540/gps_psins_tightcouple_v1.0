yhuuuuu%% GNSS\INS紧组合，不同于GNSS_SDR,本文件下所有通道是交替执行，而不是一个通道运行结束后再运行下一个通道
% trackResults, channel, TOW, eph, subFrameStart 需要提前准备好
close all;

% Ensure project functions take precedence over PSINS name conflicts
addpath include
addpath geoFunctions
addpath acquire_zcj
addpath track_zcj
addpath PVT
addpath deepIntegration

settings = initSettings();
[fid, ~] = fopen(settings.fileName, 'rb');
cleanupObj = onCleanup(@() fclose(fid));


%% INS相关信息初始化，需提前安装好PSINS
glvs
ggpsvars
psinstypedef('test_SINS_GPS_tightly_def');
trj = trjfile("E:\fgi_result\ins_simulation\trj_sc522.mat");
t0_imu = trj.imu(1,end);
t0_gps = TOW;
positioningTime = t0_gps + settings.navSolPeriod / 1000;
[nn, ts, nts] = nnts(2, diff(trj.imu(1:2,end)));   
avp0 = trj.avp0;  % 省去初始对准
davp = avperrset([10;10;60], 0.5, [2; 2; 6]);     

ins = insinit(avpadderr(trj.avp0, davp), ts);

imuerr = imuerrset(5,1000,0.05,30);  
trj.imu = imuadderr(trj.imu, imuerr);

kf = kfinit(ins, davp, imuerr);


%% INS运行至首个紧组合时刻
k = 1;
k1 = 1;                  
t = trj.imu(k1, end);   
while (t0_gps + (t - t0_imu)) < positioningTime     
    k1 = k+nn-1;
    wvm = trj.imu(k:k1,1:6);  t = trj.imu(k1,end);
    ins = insupdate(ins, wvm);
    kf.Phikk_1 = kffk(ins);
    kf = kfupdate(kf);

    k = k + nn;      
end

%% 跟踪通道初始化，需要依赖传统跟踪环和接收机的一些信息，本代码认为信号成功跟踪的通道都能帧同步成功
activeChnList = find([trackResults.status] ~= '-');   
valid = subFrameStart(activeChnList) > 1;
activeChnList = activeChnList(valid);
numActChnList = numel(activeChnList);
if numActChnList < 4
    error('Too few valid satellites for tight coupling.');
end
BitSyncTime = subFrameStart - 1;
ii = 1;
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
    

    trackDeepIn(ii).numOfCoInt = 0;

    % 记录信号跟踪的过程，便于调试
    trackProcess(ii).codeErrorList = [];
    trackProcess(ii).carrErrorList = [];
    trackProcess(ii).codeFreqList = [];
    trackProcess(ii).carrFreqList = []; 
    trackProcess(ii).PLI = [];           % Phase Lock Indicator 详见冯鑫(武大)的深组合论文
    
    
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
if numActChnList < 4
    error('Too few satellites with valid ephemeris for tight coupling.');
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

%% GNSS运行至首次组合时刻
I_P_1_list = [];  % 记录跟踪信息，便于调试
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
end


%% 紧组合
roundTime = floor(settings.msToProcess / settings.navSolPeriod);
navResults = [];
navResults.X = zeros(1, roundTime); navResults.Y = zeros(1, roundTime); navResults.Z = zeros(1, roundTime); navResults.dt = zeros(1, roundTime);
navResults.VX = zeros(1, roundTime); navResults.VY = zeros(1, roundTime); navResults.VZ = zeros(1, roundTime);navResults.df = zeros(1, roundTime);

% settings.pllNoiseBandwidth = 3;

for currMeasNr = 1 : roundTime
    currMeasNr
    settings.recvTime = positioningTime;    
     
    % 第一次定位之前，钟差过大，难以收敛，因此先使用一次单点定位以补偿钟差
    % 钟差大到什么程度卡尔曼滤波就收敛不了？我以后再探索。
    if currMeasNr == 1
        navSolut_1 = postNavLoose(trackDeepIn, settings, eph, TOW);
        % 钟差修正
        for ii = 1 : numActChnList
            trackDeepIn(ii).recvTime = trackDeepIn(ii).recvTime - navSolut_1.dt / settings.c;  
        end
    else
        
    %% 紧组合开始
    % 1. GNSS伪距，伪距率
    navSolut = postNavTight(trackDeepIn, settings, eph, TOW);
    % 尝试只保留一颗可见卫星
%     navSolut.rawP = navSolut.rawP(1); navSolut.satPositions = navSolut.satPositions(:,1); 
%     navSolut.satVelocity = navSolut.satVelocity(:,1);  navSolut.satClkCorr = navSolut.satClkCorr(1); 
    

    [posxyz, ~] = blh2xyz(ins.pos);     
    % rhoSatRec包含地球自转校正，对于紧组合来说是必须的?
    [rho, LOS, AzEl] = rhoSatRec(navSolut.satPositions', posxyz, navSolut.rawP');
    el = max(AzEl(:,2), 15*pi/180);
    delta_rawP = navSolut.rawP' + settings.c * navSolut.satClkCorr' - rho;   % 实测距离-理论距离 
    
    % 2. EKF
    kf.Hk = kfhk(ins, LOS);     % 观测矩阵H
    kf.Rk = diag((10^2) ./ (sin(el).^2));        % measurement noise covariance
    kf = kfupdate(kf, delta_rawP);
    [kf, ins] = kffeedback(kf, ins, 1, 'avp');  
    
    % 3. 将计算结果转换回ECEF坐标系并记录
    [posX, posY, posZ] = geo2cart(ins.avp(7,1), ins.avp(8,1), ins.avp(9,1), 5);
    Cenu2xyz = [-sin(ins.pos(2))                  cos(ins.pos(2))   0
                -sin(ins.pos(1))*cos(ins.pos(2)) -sin(ins.pos(1))*sin(ins.pos(2))  cos(ins.pos(1))
                 cos(ins.pos(1))*cos(ins.pos(2))  cos(ins.pos(1))*sin(ins.pos(2))  sin(ins.pos(1))];
    vxyz = Cenu2xyz' * ins.vn;
    navResults.X(1, currMeasNr) = posX;  navResults.Y(1, currMeasNr) = posY;
    navResults.Z(1, currMeasNr) = posZ;  navResults.dt(1, currMeasNr) = kf.xk(end-1);
    navResults.VX(1, currMeasNr)= vxyz(1);
    navResults.VY(1, currMeasNr)= vxyz(2);
    navResults.VZ(1, currMeasNr)= vxyz(3); navResults.df(1, currMeasNr) = kf.xk(end);
    
    % 4. 钟差修正
    for ii = 1 : numActChnList
        trackDeepIn(ii).recvTime = trackDeepIn(ii).recvTime - kf.xk(end-1) / settings.c;  
    end
    
    end
    
    % 5. 下一次紧组合时刻
    positioningTime = positioningTime + settings.navSolPeriod / 1000;

    % 6. INS运行至下一次紧组合时刻
    while (t0_gps + (t - t0_imu)) < positioningTime     
        k1 = k+nn-1;
        wvm = trj.imu(k:k1,1:6);  t = trj.imu(k1,end);
        ins = insupdate(ins, wvm);
        kf.Phikk_1 = kffk(ins);
        kf = kfupdate(kf);
     
        k = k + nn;       
        
    end

    % 7. GNSS运行至下一次紧组合时刻
    for ii = 1 : numActChnList
        trackans = trackDeepIn(ii);
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
    end
    
    

end

%% Quick result plots (skip invalid first solution)
if exist('navResults', 'var')
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
        R = [-sin(lon0)  cos(lon0) 0;
             -sin(lat0)*cos(lon0) -sin(lat0)*sin(lon0) cos(lat0);
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









