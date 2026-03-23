function dop = compute_doppler_plot_series(navResults, settings)
% Build Doppler series for plotting with the same bias compensation idea
% used by the detector.
%
% Raw GNSS carrier Doppler and INS-predicted Doppler can differ by:
% 1) a large epoch-wise common bias (receiver clock / front-end / time sync),
% 2) a smaller satellite-dependent constant bias.
%
% The detector already removes the epoch-wise common bias before thresholding.
% For plotting we apply the same compensation, then remove a residual
% satellite-wise bias estimated from the clean pre-arm segment.

satNum = size(navResults.dopplerGpsMeasHz, 1);
epochNum = size(navResults.dopplerGpsMeasHz, 2);
dt = settings.navSolPeriod / 1000;
t = (0 : epochNum - 1) * dt;

gpsRaw = navResults.dopplerGpsMeasHz;
insRaw = navResults.dopplerInsPredHz;
if isfield(navResults, 'dopplerSuppressedHz')
    supRaw = navResults.dopplerSuppressedHz;
else
    supRaw = insRaw;
end
if isfield(navResults, 'dopplerSuppressAlpha')
    alpha = navResults.dopplerSuppressAlpha;
else
    alpha = ones(size(gpsRaw));
end
alpha = min(1, max(0, alpha));

if isfield(settings, 'spoofDetArmTimeSec')
    baseMask = t < settings.spoofDetArmTimeSec;
else
    baseMask = t < min(100, max(t) / 4);
end
if nnz(baseMask) < 5
    baseMask = t < min(100, max(t) / 3);
end

rawDiff = gpsRaw - insRaw;
epochBiasHz = median(rawDiff, 1, 'omitnan');
epochBiasHz(~isfinite(epochBiasHz)) = 0;
epochBiasMat = repmat(epochBiasHz, satNum, 1);

debiasedDiff = rawDiff - epochBiasMat;
satBiasHz = median(debiasedDiff(:, baseMask), 2, 'omitnan');
satBiasHz(~isfinite(satBiasHz)) = 0;
satBiasMat = repmat(satBiasHz, 1, epochNum);

alignBiasHz = epochBiasMat + satBiasMat;
insAligned = insRaw + alignBiasHz;
supAligned = supRaw + alpha .* alignBiasHz;

dop = struct();
dop.tSec = t;
dop.gpsRawHz = gpsRaw;
dop.insAlignedHz = insAligned;
dop.supAlignedHz = supAligned;
dop.epochBiasHz = epochBiasHz;
dop.satBiasHz = satBiasHz;
dop.alignBiasHz = alignBiasHz;
end
