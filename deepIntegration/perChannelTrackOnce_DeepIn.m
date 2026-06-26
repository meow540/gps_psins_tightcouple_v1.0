function  [trackans, I_P, Q_P] = perChannelTrackOnce_DeepIn(trackans, settings, fid, dppl, delta_dppl)
%% 某通道进行一次相干积分，和原本的tracking函数无区别
%
% 输入参数:
%        - trackans: 一个通道的跟踪结构体
%        - settings: 接收机相关参数
%        - fid: 中频数据文件
%        - dppl: 上一时刻惯导反馈的频率值
%        - delta_dppl: 当前惯导反馈量减去上一时刻惯导的反馈量[Hz]
%
% 杈撳嚭鍙傛暟:
%        - trackans: 跟踪结构体
%
%--------------------------------------------------------------------------
%% 鍒濆鍖栦竴浜涘彉閲?
%--- DLL variables --------------------------------------------------------
% Define early-late offset (in chips)
earlyLateSpc = settings.dllCorrelatorSpacing;

% Summation interval
PDIcode = 0.001;

% Calculate filter coefficient values
if ~isfield(settings, 'dllGain')
    settings.dllGain = 1;
end
[tau1code, tau2code] = calcLoopCoef(settings.dllNoiseBandwidth, ...
                                    settings.dllDampingRatio, ...
                                    settings.dllGain);

%--- PLL variables --------------------------------------------------------
% Summation interval
PDIcarr = 0.001;

% Select effective PLL bandwidth by deep-coupling mode.
deepModeState = 0;
if isfield(settings, 'deepModeState')
    deepModeState = settings.deepModeState;
end
pllBw = settings.pllNoiseBandwidth;
if deepModeState == 1 && isfield(settings, 'deepPllNoiseBandwidthSuspect')
    pllBw = settings.deepPllNoiseBandwidthSuspect;
elseif deepModeState == 2 && isfield(settings, 'deepPllNoiseBandwidthSpoof')
    pllBw = settings.deepPllNoiseBandwidthSpoof;
end

% Calculate filter coefficient values
[tau1carr, tau2carr] = calcLoopCoef(pllBw, ...
                                    settings.pllDampingRatio, ...
                                    0.25);

% Move the starting point of processing. skipNumberOfBytes 宸茬粡绠楀湪SamplePos涓簡锛屼笉蹇呭啀娆¤绠?
sampleBytes = settings.fileType * settings.dataFormat;
if ~isfinite(sampleBytes) || sampleBytes <= 0
    sampleBytes = 1;
end
if ~isfield(trackans, 'SamplePos') || ~isfinite(trackans.SamplePos) || trackans.SamplePos < 0
    trackans.SamplePos = 0;
end
trackans.SamplePos = floor(trackans.SamplePos);
startBytePos = sampleBytes * trackans.SamplePos;
if ~isfinite(startBytePos) || startBytePos < 0
    startBytePos = 0;
    trackans.SamplePos = 0;
end
if fseek(fid, startBytePos, 'bof') ~= 0
    trackans.SamplePos = 0;
    if fseek(fid, 0, 'bof') ~= 0
        disp('Tracking guard: unable to seek to a safe file position, exiting!');
        return
    end
end

%--------------------------------------------------------------------------
% Get a vector with the C/A code sampled 1x/chip
caCode = generateCAcode(trackans.PRN);
% Then make it possible to do early and late versions
caCode = [caCode(1023) caCode caCode(1)];

% define initial code frequency basis of NCO
codeFreq = trackans.codeFreq;
% define residual code phase (in chips)
remCodePhase = trackans.remCodePhase;
% define carrier frequency which is used over whole tracking period
carrFreq = trackans.carrFreq;
% carrFreqBasis = trackans.carrFreqBasis;
% define residual carrier phase
remCarrPhase = trackans.remCarrPhase;
codeFreqStart = codeFreq;
carrFreqStart = carrFreq;

% Runtime guards: long-window DS5 runs occasionally drive the tracking
% state into invalid numeric regions, which can cascade into oversized
% sample reads and native heap failures on Windows MATLAB. Clamp state
% here before any block-size or buffer allocation is derived from it.
if ~isfield(settings, 'deepTrackGuardEnable'), settings.deepTrackGuardEnable = 1; end
if ~isfield(settings, 'deepTrackGuardCodeFreqMinHz'), settings.deepTrackGuardCodeFreqMinHz = 0.90 * settings.codeFreqBasis; end
if ~isfield(settings, 'deepTrackGuardCodeFreqMaxHz'), settings.deepTrackGuardCodeFreqMaxHz = 1.10 * settings.codeFreqBasis; end
if ~isfield(settings, 'deepTrackGuardCarrOffsetMaxHz'), settings.deepTrackGuardCarrOffsetMaxHz = 20000.0; end
if ~isfield(settings, 'deepTrackGuardMinBlockSize'), settings.deepTrackGuardMinBlockSize = 1; end
if ~isfield(settings, 'deepTrackGuardMaxBlockSize'), settings.deepTrackGuardMaxBlockSize = max(1, round(2.5 * settings.samplingFreq / 1000)); end
if settings.deepTrackGuardEnable
    if ~isfinite(codeFreq) || codeFreq <= 0
        codeFreq = settings.codeFreqBasis;
    end
    codeFreq = min(max(codeFreq, settings.deepTrackGuardCodeFreqMinHz), settings.deepTrackGuardCodeFreqMaxHz);
    if ~isfinite(remCodePhase)
        remCodePhase = 0;
    end
    remCodePhase = mod(remCodePhase, settings.codeLength);
    if ~isfinite(carrFreq)
        carrFreq = settings.IF;
    end
    carrOffsetHz = carrFreq - settings.IF;
    carrOffsetHz = min(max(carrOffsetHz, -settings.deepTrackGuardCarrOffsetMaxHz), settings.deepTrackGuardCarrOffsetMaxHz);
    carrFreq = settings.IF + carrOffsetHz;
    if ~isfinite(remCarrPhase)
        remCarrPhase = 0;
    end
    remCarrPhase = rem(remCarrPhase, 2*pi);
    codeFreqStart = codeFreq;
    carrFreqStart = carrFreq;
end

% code tracking loop parameters
oldCodeNco   = trackans.codeNco;
oldCodeError = trackans.codeError;

% carrier/Costas loop parameters
oldCarrNco   = trackans.carrNco;
oldCarrError = trackans.carrError;
prevModeState = 0;
if isfield(trackans, 'deepPrevMode')
    prevModeState = trackans.deepPrevMode;
end
if ~isfield(trackans, 'deepShadowFreqHz') || ~isfinite(trackans.deepShadowFreqHz)
    trackans.deepShadowFreqHz = carrFreq - settings.IF;
end
if ~isfield(trackans, 'deepAccumCarrierCycles') || ~isfinite(trackans.deepAccumCarrierCycles)
    trackans.deepAccumCarrierCycles = 0;
end
if ~isfield(trackans, 'deepAccumCodeChips') || ~isfinite(trackans.deepAccumCodeChips)
    trackans.deepAccumCodeChips = 0;
end
if ~isfield(settings, 'deepShadowTargetHz'), settings.deepShadowTargetHz = nan; end
if ~isfield(settings, 'deepShadowMaxPullHzPerStep'), settings.deepShadowMaxPullHzPerStep = 2.0; end
if ~isfield(settings, 'deepShadowWeightSuspect'), settings.deepShadowWeightSuspect = 0.50; end
if ~isfield(settings, 'deepShadowWeightSpoof'), settings.deepShadowWeightSpoof = 0.90; end
if ~isfield(settings, 'deepForceShadowNcoInSuspect'), settings.deepForceShadowNcoInSuspect = 0; end
if ~isfield(settings, 'deepForceShadowNcoInSpoof'), settings.deepForceShadowNcoInSpoof = 1; end
if ~isfield(settings, 'deepBypassPllInSuspect'), settings.deepBypassPllInSuspect = 0; end
if ~isfield(settings, 'deepBypassPllInSpoof'), settings.deepBypassPllInSpoof = 1; end
if ~isfield(settings, 'deepCodeErrorScaleSuspect'), settings.deepCodeErrorScaleSuspect = 1.0; end
if ~isfield(settings, 'deepCodeErrorScaleSpoof'), settings.deepCodeErrorScaleSpoof = 1.0; end
if ~isfield(settings, 'deepBypassDllInSuspect'), settings.deepBypassDllInSuspect = 0; end
if ~isfield(settings, 'deepBypassDllInSpoof'), settings.deepBypassDllInSpoof = 0; end
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
if ~isfield(settings, 'deepShadowCodeRefEnableNow'), settings.deepShadowCodeRefEnableNow = 0; end
if ~isfield(settings, 'deepShadowCodeRefChips'), settings.deepShadowCodeRefChips = nan; end
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
if ~isfield(trackans, 'deepCodeReacqStableCnt'), trackans.deepCodeReacqStableCnt = 0; end
if ~isfield(trackans, 'deepCodePhaseCorrChips'), trackans.deepCodePhaseCorrChips = 0; end
if ~isfield(trackans, 'deepCodeWideCounter'), trackans.deepCodeWideCounter = 0; end
if ~isfield(trackans, 'deepCodeWideCandidateChips'), trackans.deepCodeWideCandidateChips = nan; end
if ~isfield(trackans, 'deepCodeWideStableCnt'), trackans.deepCodeWideStableCnt = 0; end
if ~isfield(trackans, 'deepCodeWideOffsets'), trackans.deepCodeWideOffsets = []; end
if ~isfield(trackans, 'deepCodeWideMetricAccum'), trackans.deepCodeWideMetricAccum = []; end
if ~isfield(trackans, 'deepCodeWideMetricCount'), trackans.deepCodeWideMetricCount = 0; end
if isfield(settings, 'deepShadowReacqEnableNow')
    shadowReacqEnable = logical(settings.deepShadowReacqEnableNow);
else
    if ~isfield(settings, 'deepShadowReacqEnable'), settings.deepShadowReacqEnable = 0; end
    shadowReacqEnable = logical(settings.deepShadowReacqEnable);
end

%% 寮?濮嬭窡韪?
% Find the size of a "block" or code period in whole samples
codePhaseStep = codeFreq / settings.samplingFreq;
if ~isfinite(codePhaseStep) || codePhaseStep <= 0
    codePhaseStep = settings.codeFreqBasis / settings.samplingFreq;
    codeFreq = settings.codeFreqBasis;
    codeFreqStart = codeFreq;
end
blksize = ceil((settings.codeLength - remCodePhase) / codePhaseStep);
if settings.deepTrackGuardEnable
    blksize = min(max(blksize, settings.deepTrackGuardMinBlockSize), settings.deepTrackGuardMaxBlockSize);
end
safeSamplePos = trackans.SamplePos;
if settings.deepTrackGuardEnable
    savedBytePos = ftell(fid);
    if ~isfinite(savedBytePos) || savedBytePos < 0
        savedBytePos = 0;
    end
    if fseek(fid, 0, 'eof') == 0
        deepTrackFileBytes = ftell(fid);
    else
        deepTrackFileBytes = nan;
    end
    if fseek(fid, savedBytePos, 'bof') ~= 0
        fseek(fid, 0, 'bof');
    end
    maxSamplePosAvail = floor(deepTrackFileBytes / sampleBytes);
    if ~isfinite(maxSamplePosAvail) || maxSamplePosAvail <= 0
        disp('Tracking guard: invalid tracking file size, exiting!');
        return
    end
    maxStartSamplePos = max(0, maxSamplePosAvail - blksize);
    safeSamplePos = min(max(0, floor(trackans.SamplePos)), maxStartSamplePos);
    remainingSamples = maxSamplePosAvail - safeSamplePos;
    if remainingSamples < settings.deepTrackGuardMinBlockSize
        disp('Tracking guard: insufficient samples remain for tracking block, exiting!');
        return
    end
    if remainingSamples < blksize
        blksize = max(settings.deepTrackGuardMinBlockSize, floor(remainingSamples));
    end
    trackans.SamplePos = safeSamplePos;
    if fseek(fid, sampleBytes * safeSamplePos, 'bof') ~= 0
        disp('Tracking guard: unable to apply safe tracking seek, exiting!');
        return
    end
end

trackans.recvTime = trackans.recvTime + blksize / settings.samplingFreq; % [s]

% Read in the appropriate number of samples to process this interation
[rawSignal, samplesRead] = fread(fid, settings.fileType * blksize, settings.dataType);

rawSignal = transpose(rawSignal);  % transpose vector
if settings.fileType == 2
    rawSignalI = rawSignal(1:2:end);
    rawSignalQ = rawSignal(2:2:end);
    rawSignal  = rawSignalI + 1j * rawSignalQ; 
end

% If did not read in enough samples, then could be out of data - better exit 
if (samplesRead ~= settings.fileType * blksize)
    disp('Not able to read the specified number of samples  for tracking, exiting!')
    return
end

%--------------------------------------------------------------------------
% Define index into early code vector
remCodePhaseStart = remCodePhase;
tcode       = (remCodePhase-earlyLateSpc) : ...
              codePhaseStep : ...
              ((blksize-1)*codePhaseStep+remCodePhase-earlyLateSpc);
tcode2      = ceil(tcode) + 1;
earlyCode   = caCode(tcode2);

% Define index into late code vector
tcode       = (remCodePhase+earlyLateSpc) : ...
              codePhaseStep : ...
              ((blksize-1)*codePhaseStep+remCodePhase+earlyLateSpc);
tcode2      = ceil(tcode) + 1;
lateCode    = caCode(tcode2);

% Define index into prompt code vector
tcode       = remCodePhase : ...
              codePhaseStep : ...
              ((blksize-1)*codePhaseStep+remCodePhase);
tcode2      = ceil(tcode) + 1;
promptCode  = caCode(tcode2);

remCodePhase = (tcode(blksize) + codePhaseStep) - 1023.0;

% Generate the carrier frequency to mix the signal to baseband -----------
time        = (0:blksize) ./ settings.samplingFreq;

% Get the argument to sin/cos functions
trigarg     = ((carrFreq * 2.0 * pi) .* time) + remCarrPhase;
remCarrPhase = rem(trigarg(blksize+1), (2 * pi));

carr = exp(-1j* trigarg(1:blksize));
qBasebandSignal = imag(carr .* rawSignal);
iBasebandSignal = real(carr .* rawSignal);

% Now get early, late, and prompt values for each
I_E = sum(earlyCode  .* iBasebandSignal);
Q_E = sum(earlyCode  .* qBasebandSignal);
I_P = sum(promptCode .* iBasebandSignal);
Q_P = sum(promptCode .* qBasebandSignal);
I_L = sum(lateCode   .* iBasebandSignal);
Q_L = sum(lateCode   .* qBasebandSignal);

% In suspect/spoof mode, do a bounded local code-phase peak search around
% the current code phase. This is intentionally local and rate-limited: the
% carrier/NCO recovery branch supplies the frequency constraint, while this
% block prevents the DLL from being pulled rapidly toward a spoofed code peak.
codeReacqUsed = false;
codeReacqOffset = 0;
codeReacqApplied = 0;
codeReacqMetric = nan;
codeReacqPeakRatio = nan;
codeWideUsed = false;
codeWideOffset = nan;
codeWidePeakRatio = nan;
codeWideStableCnt = trackans.deepCodeWideStableCnt;
codeWideAccumCount = trackans.deepCodeWideMetricCount;
if settings.deepCodeReacqEnable && deepModeState >= settings.deepCodeReacqMinMode
    if trackans.deepCodeReacqStableCnt >= settings.deepCodeReacqNarrowAfter
        searchHalf = settings.deepCodeReacqNarrowHalfChips;
    else
        searchHalf = settings.deepCodeReacqHalfChips;
    end
    searchHalf = max(0, searchHalf);
    stepChip = max(eps, settings.deepCodeReacqStepChips);
    offsets = -searchHalf:stepChip:searchHalf;
    if isempty(offsets) || all(abs(offsets) > eps)
        offsets = unique([offsets, 0]);
    end
    [bestOffset, bestMag, secondMag, zeroMag, bestIE, bestQE, bestIP, bestQP, bestIL, bestQL] = ...
        localCodePeakSearch(offsets, remCodePhaseStart, codePhaseStep, blksize, ...
                            earlyLateSpc, caCode, iBasebandSignal, qBasebandSignal);
    codeReacqMetric = bestMag;
    codeReacqPeakRatio = bestMag / max(secondMag, eps);
    zeroRatio = bestMag / max(zeroMag, eps);
    confidentPeak = isfinite(bestMag) && bestMag >= settings.deepCodeReacqMinPromptMag && ...
                    codeReacqPeakRatio >= settings.deepCodeReacqPeakRatio && ...
                    zeroRatio >= settings.deepCodeReacqZeroRatio;
    if confidentPeak
        codeReacqUsed = abs(bestOffset) > eps;
        codeReacqOffset = bestOffset;
        I_E = bestIE; Q_E = bestQE;
        I_P = bestIP; Q_P = bestQP;
        I_L = bestIL; Q_L = bestQL;

        % Do not directly move remCodePhase here. remCodePhase is the
        % residual phase after this 1 ms block; wrapping a small negative
        % correction to ~1023 chips makes the next block size collapse and
        % corrupts pseudorange. The local peak is used for discriminator
        % samples, and optionally exported as a bounded pseudorange/code
        % phase correction for postNavTight.
        codeReacqApplied = 0;
        if settings.deepCodeReacqNavCorrEnable
            targetCorr = bestOffset;
            corrGain = max(0, min(1, settings.deepCodeReacqNavCorrGain));
            corrStep = max(0, settings.deepCodeReacqNavCorrStepChips);
            corrLimit = max(0, settings.deepCodeReacqNavCorrLimitChips);
            wantedCorr = (1 - corrGain) * trackans.deepCodePhaseCorrChips + ...
                         corrGain * targetCorr;
            dCorr = wantedCorr - trackans.deepCodePhaseCorrChips;
            dCorr = max(-corrStep, min(corrStep, dCorr));
            trackans.deepCodePhaseCorrChips = trackans.deepCodePhaseCorrChips + dCorr;
            trackans.deepCodePhaseCorrChips = max(-corrLimit, ...
                min(corrLimit, trackans.deepCodePhaseCorrChips));
        end
        trackans.deepCodeReacqStableCnt = trackans.deepCodeReacqStableCnt + 1;
    else
        trackans.deepCodeReacqStableCnt = max(0, trackans.deepCodeReacqStableCnt - 1);
        if settings.deepCodeReacqNavCorrEnable && deepModeState < settings.deepCodeReacqMinMode
            trackans.deepCodePhaseCorrChips = settings.deepCodeReacqNavCorrDecay * ...
                trackans.deepCodePhaseCorrChips;
        end
    end
else
    trackans.deepCodeReacqStableCnt = max(0, trackans.deepCodeReacqStableCnt - 1);
    if settings.deepCodeReacqNavCorrEnable && deepModeState < settings.deepCodeReacqMinMode
        trackans.deepCodePhaseCorrChips = settings.deepCodeReacqNavCorrDecay * ...
            trackans.deepCodePhaseCorrChips;
    end
end

codeRefUsed = false;
codeRefOffset = nan;
codeRefMetric = nan;
codeRefPeakRatio = nan;
codeRefAppliedChips = 0;
codeRefNcoPullHz = 0;
codeRefNcoStepHz = 0;
if settings.deepShadowCodeRefEnableNow && isfinite(settings.deepShadowCodeRefChips)
    targetOffset = mod((settings.deepShadowCodeRefChips - remCodePhaseStart) + settings.codeLength / 2, settings.codeLength) - settings.codeLength / 2;
    searchHalf = max(0, settings.deepShadowDs5CodeRefHalfChips);
    stepChip = max(eps, settings.deepShadowDs5CodeRefStepChips);
    offsets = targetOffset + (-searchHalf:stepChip:searchHalf);
    if isempty(offsets)
        offsets = targetOffset;
    end
    if ~any(abs(offsets - targetOffset) < 1e-12)
        offsets = unique([offsets, targetOffset]);
    end
    if ~any(abs(offsets) < 1e-12)
        offsets = unique([offsets, 0]);
    end
    [bestOffset, bestMag, secondMag, zeroMag, bestIE, bestQE, bestIP, bestQP, bestIL, bestQL] = ...
        localCodePeakSearch(offsets, remCodePhaseStart, codePhaseStep, blksize, ...
                            earlyLateSpc, caCode, iBasebandSignal, qBasebandSignal);
    codeRefMetric = bestMag;
    codeRefPeakRatio = bestMag / max(secondMag, eps);
    zeroRatio = bestMag / max(zeroMag, eps);
    confidentPeak = isfinite(bestMag) && bestMag >= settings.deepShadowDs5CodeRefMinPromptMag && ...
        codeRefPeakRatio >= settings.deepShadowDs5CodeRefPeakRatioMin && ...
        zeroRatio >= settings.deepShadowDs5CodeRefZeroRatioMin;
    if confidentPeak
        codeRefUsed = true;
        codeRefOffset = bestOffset;
        I_E = bestIE; Q_E = bestQE;
        I_P = bestIP; Q_P = bestQP;
        I_L = bestIL; Q_L = bestQL;
        corrBefore = trackans.deepCodePhaseCorrChips;
        wantedCorr = (1 - max(0, min(1, settings.deepShadowDs5CodeRefApplyGain))) * ...
            corrBefore + max(0, min(1, settings.deepShadowDs5CodeRefApplyGain)) * bestOffset;
        dCorr = wantedCorr - corrBefore;
        dCorr = max(-settings.deepShadowDs5CodeRefStepMaxChips, min(settings.deepShadowDs5CodeRefStepMaxChips, dCorr));
        corrAfter = corrBefore + dCorr;
        corrAfter = max(-settings.deepShadowDs5CodeRefTotalMaxChips, ...
            min(settings.deepShadowDs5CodeRefTotalMaxChips, corrAfter));
        codeRefAppliedChips = corrAfter - corrBefore;
        trackans.deepCodePhaseCorrChips = corrAfter;
    end
end

if settings.deepCodeWideReacqEnable && deepModeState >= settings.deepCodeWideReacqMinMode
    trackans.deepCodeWideCounter = trackans.deepCodeWideCounter + 1;
    intervalMs = max(1, round(settings.deepCodeWideReacqIntervalMs));
    wideHalf = max(0, settings.deepCodeWideReacqHalfChips);
    wideStep = max(eps, settings.deepCodeWideReacqStepChips);
    wideOffsets = -wideHalf:wideStep:wideHalf;
    if isempty(wideOffsets) || all(abs(wideOffsets) > eps)
        wideOffsets = unique([wideOffsets, 0]);
    end
    if isempty(trackans.deepCodeWideOffsets) || ...
            numel(trackans.deepCodeWideOffsets) ~= numel(wideOffsets) || ...
            any(abs(trackans.deepCodeWideOffsets - wideOffsets) > 1e-9)
        trackans.deepCodeWideOffsets = wideOffsets;
        trackans.deepCodeWideMetricAccum = zeros(size(wideOffsets));
        trackans.deepCodeWideMetricCount = 0;
    end

    wideMetric = localCodeMetricVector(wideOffsets, remCodePhaseStart, codePhaseStep, ...
        blksize, caCode, iBasebandSignal, qBasebandSignal);
    trackans.deepCodeWideMetricAccum = trackans.deepCodeWideMetricAccum + wideMetric;
    trackans.deepCodeWideMetricCount = trackans.deepCodeWideMetricCount + 1;

    accumNeed = max(1, round(settings.deepCodeWideReacqAccumMs));
    if trackans.deepCodeWideCounter >= intervalMs || trackans.deepCodeWideMetricCount >= accumNeed
        trackans.deepCodeWideCounter = 0;
        metricAccum = trackans.deepCodeWideMetricAccum;
        accumOffsets = trackans.deepCodeWideOffsets;
        zeroMask = abs(accumOffsets) <= max(settings.deepCodeWideReacqExcludeChips, settings.deepCodeReacqHalfChips);
        zeroMag = max(metricAccum(zeroMask));
        if isempty(zeroMag) || ~isfinite(zeroMag)
            zeroMag = 0;
        end
        searchMetric = metricAccum;
        if settings.deepCodeWideReacqUseSecondPeak
            searchMetric(zeroMask) = -inf;
        end
        [wideBestMag, bestIdx] = max(searchMetric);
        wideBestOffset = accumOffsets(bestIdx);
        tmpMetric = searchMetric;
        tmpMetric(bestIdx) = -inf;
        wideSecondMag = max(tmpMetric);
        if ~isfinite(wideSecondMag)
            wideSecondMag = 0;
        end
        codeWideOffset = wideBestOffset;
        codeWidePeakRatio = wideBestMag / max(wideSecondMag, eps);
        wideZeroRatio = wideBestMag / max(zeroMag, eps);
        wideOk = isfinite(wideBestMag) && wideBestMag > 0 && ...
                 codeWidePeakRatio >= settings.deepCodeWideReacqAccumRatio && ...
                 wideZeroRatio >= settings.deepCodeWideReacqZeroRatio && ...
                 abs(wideBestOffset) > max(settings.deepCodeWideReacqExcludeChips, settings.deepCodeReacqHalfChips);
        if wideOk
            if isfinite(trackans.deepCodeWideCandidateChips) && ...
                    abs(wideBestOffset - trackans.deepCodeWideCandidateChips) <= settings.deepCodeWideReacqCandidateTolChips
                trackans.deepCodeWideStableCnt = trackans.deepCodeWideStableCnt + 1;
                trackans.deepCodeWideCandidateChips = 0.5 * trackans.deepCodeWideCandidateChips + ...
                                                      0.5 * wideBestOffset;
            else
                trackans.deepCodeWideCandidateChips = wideBestOffset;
                trackans.deepCodeWideStableCnt = 1;
            end
        else
            trackans.deepCodeWideStableCnt = max(0, trackans.deepCodeWideStableCnt - 1);
        end
        trackans.deepCodeWideMetricAccum = zeros(size(trackans.deepCodeWideMetricAccum));
        trackans.deepCodeWideMetricCount = 0;

        if settings.deepCodeReacqNavCorrEnable && ...
                trackans.deepCodeWideStableCnt >= settings.deepCodeWideReacqStableEpochs
            codeWideUsed = true;
            targetCorr = trackans.deepCodeWideCandidateChips;
            corrGain = max(0, min(1, settings.deepCodeWideReacqCorrGain));
            corrStep = max(0, settings.deepCodeWideReacqCorrStepChips);
            corrLimit = max(0, settings.deepCodeWideReacqCorrLimitChips);
            wantedCorr = (1 - corrGain) * trackans.deepCodePhaseCorrChips + ...
                         corrGain * targetCorr;
            dCorr = wantedCorr - trackans.deepCodePhaseCorrChips;
            dCorr = max(-corrStep, min(corrStep, dCorr));
            trackans.deepCodePhaseCorrChips = trackans.deepCodePhaseCorrChips + dCorr;
            trackans.deepCodePhaseCorrChips = max(-corrLimit, ...
                min(corrLimit, trackans.deepCodePhaseCorrChips));
        end
    end
else
    trackans.deepCodeWideCounter = 0;
    trackans.deepCodeWideStableCnt = max(0, trackans.deepCodeWideStableCnt - 1);
    trackans.deepCodeWideMetricCount = 0;
    if ~isempty(trackans.deepCodeWideMetricAccum)
        trackans.deepCodeWideMetricAccum(:) = 0;
    end
end
codeWideStableCnt = trackans.deepCodeWideStableCnt;
codeWideAccumCount = trackans.deepCodeWideMetricCount;

% Find PLL error and update code NCO --------------------------------------
% Implement carrier loop discriminator (phase detector)
carrError = atan(Q_P / I_P) / (2.0 * pi);
carrErrorRaw = carrError;

% State transition smoothing: align/soften internal loop state on spoof entry.
if deepModeState == 2 && prevModeState < 2
    if ~isfield(settings, 'deepAlignCarrNcoOnSpoof'), settings.deepAlignCarrNcoOnSpoof = 1; end
    if settings.deepAlignCarrNcoOnSpoof
        if ~isfield(settings, 'deepAidWeight'), settings.deepAidWeight = 1.0; end
        if ~isfield(settings, 'deepClkWeight'), settings.deepClkWeight = 1.0; end
        if ~isfield(settings, 'deepClkDriftHz'), settings.deepClkDriftHz = 0.0; end
        % carrNco stores the residual loop-control term, not the absolute
        % carrier command. Rebase it from the current command so spoof entry
        % keeps the same absolute carrier frequency without double-counting
        % aid/clock terms in baseCmdHz below.
        oldCarrNco = (carrFreq - settings.IF) - ...
            (settings.deepAidWeight * dppl + settings.deepClkWeight * settings.deepClkDriftHz);
    end
    if ~isfield(settings, 'deepResetCarrErrorOnSpoof'), settings.deepResetCarrErrorOnSpoof = 1; end
    if settings.deepResetCarrErrorOnSpoof
        oldCarrError = 0;
    end
end

% Downweight or freeze discriminator contribution in suspect/spoof mode.
if ~isfield(settings, 'deepCarrErrorScaleSuspect'), settings.deepCarrErrorScaleSuspect = 0.5; end
if ~isfield(settings, 'deepCarrErrorScaleSpoof'), settings.deepCarrErrorScaleSpoof = 0.1; end
if deepModeState == 1
    carrError = carrError * settings.deepCarrErrorScaleSuspect;
elseif deepModeState == 2
    if ~isfield(settings, 'deepFreezeCarrErrorInSpoof'), settings.deepFreezeCarrErrorInSpoof = 0; end
    if settings.deepFreezeCarrErrorInSpoof
        carrError = 0;
    else
        carrError = carrError * settings.deepCarrErrorScaleSpoof;
    end
end

% Implement carrier loop filter and generate NCO command
pllBypassNow = (deepModeState == 2 && settings.deepBypassPllInSpoof) || ...
   (deepModeState == 1 && settings.deepBypassPllInSuspect);
if pllBypassNow
    carrNco = oldCarrNco;
else
    carrNco = oldCarrNco + (tau2carr/tau1carr) * ...
        (carrError - oldCarrError) + carrError * (PDIcarr/tau1carr);
end
% oldCarrNco   = carrNco;
% oldCarrError = carrError;

% Modify carrier frequency based on PLL output + INS aid + clock-drift aid.
if ~isfield(settings, 'deepInterpDiv'), settings.deepInterpDiv = 20; end
if ~isfield(settings, 'deepAidWeight'), settings.deepAidWeight = 1.0; end
if ~isfield(settings, 'deepClkWeight'), settings.deepClkWeight = 1.0; end
if ~isfield(settings, 'deepClkDriftHz'), settings.deepClkDriftHz = 0.0; end
if ~isfield(settings, 'deepCarrNcoStepLimitHz'), settings.deepCarrNcoStepLimitHz = inf; end

% Optional carrier-NCO step clipping to avoid abrupt loop transients.
if isfinite(settings.deepCarrNcoStepLimitHz)
    dn = carrNco - oldCarrNco;
    if dn > settings.deepCarrNcoStepLimitHz
        carrNco = oldCarrNco + settings.deepCarrNcoStepLimitHz;
    elseif dn < -settings.deepCarrNcoStepLimitHz
        carrNco = oldCarrNco - settings.deepCarrNcoStepLimitHz;
    end
end

aidFreqNow = dppl + delta_dppl / max(1, settings.deepInterpDiv);
baseCmdHz = carrNco + settings.deepAidWeight * aidFreqNow + ...
            settings.deepClkWeight * settings.deepClkDriftHz;
shadowCmdHz = trackans.deepShadowFreqHz;
shadowTargetHz = nan;
dShadowHz = 0;
shadowWeight = 0;
forceShadowNow = false;
if shadowReacqEnable && deepModeState >= 1
    shadowTargetHz = settings.deepShadowTargetHz;
    if ~isfinite(shadowTargetHz)
        shadowTargetHz = settings.deepAidWeight * aidFreqNow + ...
                         settings.deepClkWeight * settings.deepClkDriftHz;
    end
    dShadowHz = shadowTargetHz - shadowCmdHz;
    if isfinite(settings.deepShadowMaxPullHzPerStep)
        dShadowHz = max(-settings.deepShadowMaxPullHzPerStep, ...
                        min(settings.deepShadowMaxPullHzPerStep, dShadowHz));
    end
    shadowCmdHz = shadowCmdHz + dShadowHz;
    if deepModeState == 2
        shadowWeight = settings.deepShadowWeightSpoof;
    else
        shadowWeight = settings.deepShadowWeightSuspect;
    end
    shadowWeight = max(0.0, min(1.0, shadowWeight));
    forceShadowNow = (deepModeState == 2 && settings.deepForceShadowNcoInSpoof) || ...
                     (deepModeState == 1 && settings.deepForceShadowNcoInSuspect);
    if forceShadowNow
        cmdHz = shadowCmdHz;
    else
        cmdHz = (1 - shadowWeight) * baseCmdHz + shadowWeight * shadowCmdHz;
    end
else
cmdHz = baseCmdHz;
    shadowCmdHz = cmdHz;
end
if settings.deepTrackGuardEnable
    cmdHz = min(max(cmdHz, -settings.deepTrackGuardCarrOffsetMaxHz), settings.deepTrackGuardCarrOffsetMaxHz);
    shadowCmdHz = min(max(shadowCmdHz, -settings.deepTrackGuardCarrOffsetMaxHz), settings.deepTrackGuardCarrOffsetMaxHz);
end
carrFreq = settings.IF + cmdHz;

trackans.carrFreq = carrFreq;   
trackans.deepShadowFreqHz = shadowCmdHz;
trackans.deepShadowErrHz = cmdHz - shadowCmdHz;
trackans.deepCarrFreqStartHz = carrFreqStart - settings.IF;
trackans.deepCarrFreqEndHz = cmdHz;
trackans.deepCarrCmdHz = cmdHz;
trackans.deepCarrBaseCmdHz = baseCmdHz;
trackans.deepCarrShadowCmdHz = shadowCmdHz;
trackans.deepCarrAidFreqHz = aidFreqNow;
trackans.deepCarrShadowTargetHz = shadowTargetHz;
trackans.deepCarrShadowPullHz = dShadowHz;
trackans.deepCarrShadowWeight = shadowWeight;
trackans.deepCarrForceShadow = forceShadowNow;
trackans.deepCarrPllBypass = pllBypassNow;
trackans.deepCarrOldNcoHz = oldCarrNco;
trackans.deepCarrNcoHz = carrNco;
trackans.deepCarrNcoStepHz = carrNco - oldCarrNco;
trackans.deepCarrOldErrorCycles = oldCarrError;
trackans.deepCarrErrorRawCycles = carrErrorRaw;
trackans.deepCarrErrorScaledCycles = carrError;

% Find DLL error and update code NCO --------------------------------------
codeError = (sqrt(I_E * I_E + Q_E * Q_E) - sqrt(I_L * I_L + Q_L * Q_L)) / ...
                (sqrt(I_E * I_E + Q_E * Q_E) + sqrt(I_L * I_L + Q_L * Q_L));
codeErrorRaw = codeError;

% Reduce spoof pull on DLL discriminator in suspect/spoof mode.
if deepModeState == 1
    codeError = codeError * settings.deepCodeErrorScaleSuspect;
elseif deepModeState == 2
    if settings.deepFreezeCodeErrorInSpoof
        codeError = 0;
    else
        codeError = codeError * settings.deepCodeErrorScaleSpoof;
    end
end
            
% Implement code loop filter and generate NCO command
if (deepModeState == 2 && settings.deepBypassDllInSpoof) || ...
   (deepModeState == 1 && settings.deepBypassDllInSuspect)
    codeNco = oldCodeNco;
else
    codeNco = oldCodeNco + (tau2code/tau1code) * ...
        (codeError - oldCodeError) + codeError * (PDIcode/tau1code);
end
if codeRefUsed && settings.deepShadowDs5CodeRefNcoPullEnable && isfinite(codeRefAppliedChips)
    codeRefNcoPullHz = settings.deepShadowDs5CodeRefNcoGain * codeRefAppliedChips / max(PDIcode, eps);
    maxPullHz = settings.deepShadowDs5CodeRefNcoMaxHz;
    if isfinite(maxPullHz) && maxPullHz > 0
        codeRefNcoPullHz = max(-maxPullHz, min(maxPullHz, codeRefNcoPullHz));
    end
    % Positive chip correction should advance local code, which means a
    % positive code-frequency pull and therefore a negative codeNco step.
    codeRefNcoStepHz = -codeRefNcoPullHz;
    codeNco = codeNco + codeRefNcoStepHz;
end
if isfinite(settings.deepCodeNcoStepLimitHz)
    dcn = codeNco - oldCodeNco;
    if dcn > settings.deepCodeNcoStepLimitHz
        codeNco = oldCodeNco + settings.deepCodeNcoStepLimitHz;
    elseif dcn < -settings.deepCodeNcoStepLimitHz
        codeNco = oldCodeNco - settings.deepCodeNcoStepLimitHz;
    end
end
% oldCodeNco   = codeNco;
% oldCodeError = codeError;

% Modify code freq based on NCO command
codeFreq = settings.codeFreqBasis - codeNco + (carrFreq - settings.IF) / 1540;   %% PLL Aided DLL correct answer!
if settings.deepTrackGuardEnable
    codeFreq = min(max(codeFreq, settings.deepTrackGuardCodeFreqMinHz), settings.deepTrackGuardCodeFreqMaxHz);
end
trackans.codeFreq = codeFreq;

currBytePos = ftell(fid);
if isfinite(currBytePos) && currBytePos >= 0
    trackans.SamplePos = currBytePos / sampleBytes;
else
    trackans.SamplePos = safeSamplePos + blksize;
end

trackans.codeError          = codeError;
trackans.codeNco            = codeNco;
trackans.carrError          = carrError;
trackans.carrNco            = carrNco;
trackans.deepPrevMode       = deepModeState;
trackans.deepPromptI        = I_P;
trackans.deepPromptQ        = Q_P;
trackans.deepEarlyI         = I_E;
trackans.deepEarlyQ         = Q_E;
trackans.deepLateI          = I_L;
trackans.deepLateQ          = Q_L;
trackans.deepDllDiscrRaw    = codeErrorRaw;
trackans.deepDllDiscr       = codeError;
trackans.deepCodeReacqUsed  = codeReacqUsed;
trackans.deepCodeReacqOffsetChips = codeReacqOffset;
trackans.deepCodeReacqAppliedChips = codeReacqApplied;
trackans.deepCodeReacqMetric = codeReacqMetric;
trackans.deepCodeReacqPeakRatio = codeReacqPeakRatio;
trackans.deepCodeReacqStableCnt = trackans.deepCodeReacqStableCnt;
trackans.deepCodePhaseCorrChips = trackans.deepCodePhaseCorrChips;
trackans.deepCodeWideUsed = codeWideUsed;
trackans.deepCodeWideOffsetChips = codeWideOffset;
trackans.deepCodeWidePeakRatio = codeWidePeakRatio;
trackans.deepCodeWideStableCnt = codeWideStableCnt;
trackans.deepCodeWideAccumCount = codeWideAccumCount;
trackans.deepCodeWideCandidateChips = trackans.deepCodeWideCandidateChips;
trackans.deepCodeWideCounter = trackans.deepCodeWideCounter;
trackans.deepShadowCodeRefUsed = codeRefUsed;
trackans.deepShadowCodeRefOffsetChips = codeRefOffset;
trackans.deepShadowCodeRefMetric = codeRefMetric;
trackans.deepShadowCodeRefPeakRatio = codeRefPeakRatio;
trackans.deepShadowCodeRefAppliedChips = codeRefAppliedChips;
trackans.deepShadowCodeRefNcoPullHz = codeRefNcoPullHz;
trackans.deepShadowCodeRefNcoStepHz = codeRefNcoStepHz;

trackans.remCodePhase       = remCodePhase;
trackans.remCarrPhase       = remCarrPhase;
trackans.deepAccumCarrierCycles = trackans.deepAccumCarrierCycles + (carrFreqStart - settings.IF) * PDIcarr;
trackans.deepAccumCodeChips = trackans.deepAccumCodeChips + codeFreqStart * PDIcode;

trackans.numOfCoInt         = trackans.numOfCoInt + 1; 
end

function [bestOffset, bestMag, secondMag, zeroMag, bestIE, bestQE, bestIP, bestQP, bestIL, bestQL] = ...
    localCodePeakSearch(offsets, remCodePhaseStart, codePhaseStep, blksize, ...
                        earlyLateSpc, caCode, iBasebandSignal, qBasebandSignal)
bestOffset = 0;
bestMag = -inf;
secondMag = -inf;
zeroMag = nan;
bestIE = 0; bestQE = 0; bestIP = 0; bestQP = 0; bestIL = 0; bestQL = 0;

for kk = 1:numel(offsets)
    off = offsets(kk);
    [IE, QE, IP, QP, IL, QL] = localCorrelateAtOffset(off, remCodePhaseStart, ...
        codePhaseStep, blksize, earlyLateSpc, caCode, iBasebandSignal, qBasebandSignal);
    mag = hypot(IP, QP);
    if abs(off) < 1e-12
        zeroMag = mag;
    end
    if mag > bestMag
        secondMag = bestMag;
        bestMag = mag;
        bestOffset = off;
        bestIE = IE; bestQE = QE; bestIP = IP; bestQP = QP; bestIL = IL; bestQL = QL;
    elseif mag > secondMag
        secondMag = mag;
    end
end

if ~isfinite(zeroMag)
    [~, ~, IP0, QP0, ~, ~] = localCorrelateAtOffset(0, remCodePhaseStart, ...
        codePhaseStep, blksize, earlyLateSpc, caCode, iBasebandSignal, qBasebandSignal);
    zeroMag = hypot(IP0, QP0);
end
if ~isfinite(secondMag)
    secondMag = bestMag;
end
end

function [IE, QE, IP, QP, IL, QL] = localCorrelateAtOffset(offsetChips, remCodePhaseStart, ...
    codePhaseStep, blksize, earlyLateSpc, caCode, iBasebandSignal, qBasebandSignal)
tPrompt = (remCodePhaseStart + offsetChips) : codePhaseStep : ...
          ((blksize - 1) * codePhaseStep + remCodePhaseStart + offsetChips);
tEarly = tPrompt - earlyLateSpc;
tLate = tPrompt + earlyLateSpc;

promptCode = localCodeLookup(caCode, tPrompt);
earlyCode = localCodeLookup(caCode, tEarly);
lateCode = localCodeLookup(caCode, tLate);

IE = sum(earlyCode .* iBasebandSignal);
QE = sum(earlyCode .* qBasebandSignal);
IP = sum(promptCode .* iBasebandSignal);
QP = sum(promptCode .* qBasebandSignal);
IL = sum(lateCode .* iBasebandSignal);
QL = sum(lateCode .* qBasebandSignal);
end

function code = localCodeLookup(caCode, tcode)
idx = ceil(tcode) + 1;
idx = mod(idx - 1, 1023) + 1;
code = caCode(idx);
end

function metric = localCodeMetricVector(offsets, remCodePhaseStart, codePhaseStep, ...
    blksize, caCode, iBasebandSignal, qBasebandSignal)
metric = zeros(size(offsets));
for kk = 1:numel(offsets)
    off = offsets(kk);
    tPrompt = (remCodePhaseStart + off) : codePhaseStep : ...
              ((blksize - 1) * codePhaseStep + remCodePhaseStart + off);
    promptCode = localCodeLookup(caCode, tPrompt);
    IP = sum(promptCode .* iBasebandSignal);
    QP = sum(promptCode .* qBasebandSignal);
    metric(kk) = hypot(IP, QP);
end
end
