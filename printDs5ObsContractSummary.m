function printDs5ObsContractSummary(resultFile, windowStartSec)
if nargin < 2 || isempty(windowStartSec)
    windowStartSec = 180;
end
S = load(resultFile, 'navResults');
R = S.navResults;
n = numel(R.X);
t = (0:n-1) * 0.5;
mask = t >= windowStartSec;

fprintf('\n--- DS5 observation-contract summary from %.1fs ---\n', windowStartSec);
printBoolCount(R, 'shadowDs5ObsContractPass', mask, 'obsContractPass');
printBoolCount(R, 'shadowDs5ObsContractTrackingPass', mask, 'obsContractTrackingPass');
printBoolCount(R, 'shadowDs5ObsContractCommonClockPass', mask, 'obsContractCommonClockPass');
printBoolCount(R, 'shadowDs5ObsContractSpreadPass', mask, 'obsContractSpreadPass');
printBoolCount(R, 'shadowDs5ObsContractBaseSeedPass', mask, 'obsContractBaseSeedPass');
if isfield(R, 'shadowDs5ObsContractStrictRawIndependentPass')
    printBoolCount(R, 'shadowDs5ObsContractStrictRawIndependentPass', mask, 'strictRawIndependentPass');
end
printBoolCount(R, 'shadowDs5RefObsTrackRawSampled', mask, 'refObsTrackRawSampled');
printBoolCount(R, 'shadowDs5RefObsTrackCoasted', mask, 'refObsTrackCoasted');
printBoolCount(R, 'shadowDs5RefObsRecoveryPass', mask, 'refObsRecoveryPass');
printBoolCount(R, 'shadowDs5RefObsPosDeltaConsistencyPass', mask, 'deltaConsistencyPass');
printBoolCount(R, 'shadowDs5RefObsPosUsed', mask, 'refObsPositionUsed');
printBoolCount(R, 'shadowDs5RefObsPosRawAnchorLiftUsed', mask, 'refObsRawAnchorLiftUsed');
printBoolCount(R, 'shadowDs5RefObsPosJumpGatePass', mask, 'refObsJumpGatePass');
printBoolCount(R, 'shadowDs5RefObsPosDynGatePass', mask, 'refObsDynGatePass');
printBoolCount(R, 'shadowDs5RefObsPosAbsAnchorGatePass', mask, 'refObsAbsAnchorGatePass');
printBoolCount(R, 'shadowRecoveredFilterMeasPass', mask, 'recoveredFilterMeasPass');
printBoolCount(R, 'shadowRecoveredFilterFilterValidPass', mask, 'recoveredFilterFilterValidPass');
printBoolCount(R, 'shadowRecoveredFilterAuthorityAbsAnchorPass', mask, 'recoveredFilterAuthorityAbsAnchorPass');
printBoolCount(R, 'shadowRecoveredFilterAuthorityMeasPredPass', mask, 'recoveredFilterAuthorityMeasPredPass');
printBoolCount(R, 'shadowRecoveredFilterAuthority', mask, 'recoveredFilterAuthority');
printBoolCount(R, 'shadowRecoveredFilterAuthorityCapPass', mask, 'recoveredFilterAuthorityCapPass');
printBoolCount(R, 'shadowFinalOutputRecoveryGatePass', mask, 'finalOutputRecoveryGatePass');
printBoolCount(R, 'shadowFinalOutputContinuityPass', mask, 'finalOutputContinuityPass');
printBoolCount(R, 'shadowFinalOutputUseRecovered', mask, 'finalOutputUseRecovered');
printBoolCount(R, 'shadowFinalObsContractRawPass', mask, 'finalObsContractRawPass');
printBoolCount(R, 'shadowFinalObsContractPass', mask, 'finalObsContractPass');
printBoolCount(R, 'shadowFinalObsContractTrackingHold', mask, 'finalObsContractTrackingHold');
printBoolCount(R, 'shadowFinalObsContractBaseSeedHold', mask, 'finalObsContractBaseSeedHold');

printNumSummary(R, 'shadowDs5ObsContractSpreadP95M', mask, 'obsContract spreadP95 m');
printNumSummary(R, 'shadowDs5ObsContractCommonClockM', mask, 'obsContract commonClock m');
printNumSummary(R, 'shadowDs5ObsContractBaseSeedFrac', mask, 'obsContract effectiveBaseSeedFrac');
if isfield(R, 'shadowDs5ObsContractStrictRawIndependentSatNum')
    printNumSummary(R, 'shadowDs5ObsContractStrictRawIndependentSatNum', mask, 'strictRawIndependent sat');
    printNumSummary(R, 'shadowDs5ObsContractRawSampledUsedSatNum', mask, 'strictRaw rawSampledUsed sat');
    printNumSummary(R, 'shadowDs5ObsContractRawNonCoastedUsedSatNum', mask, 'strictRaw rawNonCoastedUsed sat');
    printNumSummary(R, 'shadowDs5ObsContractNoBaseSeedUsedSatNum', mask, 'strictRaw noBaseSeedUsed sat');
    printNumSummary(R, 'shadowDs5ObsContractQualityIndependentSatNum', mask, 'strictRaw qualityIndependent sat');
end
printStrictRawIndependentDerived(R, mask);
printNumSummary(R, 'shadowDs5RefObsPosRawAnchorLiftSpreadP95M', mask, 'refObs rawAnchorLift spreadP95 m');
printNumSummary(R, 'shadowDs5RefObsPosPostfitRmsM', mask, 'refObs position postfitRms m');
printNumSummary(R, 'shadowDs5RefObsPosCorrM', mask, 'refObs position corr m');
printNumSummary(R, 'shadowDs5RefObsPosAnchorDiffM', mask, 'refObs position anchorDiff m');
printNumSummary(R, 'shadowDs5RefObsPosJumpM', mask, 'refObs position jump m');
printNumSummary(R, 'shadowDs5RefObsPosSpeedMps', mask, 'refObs position speed mps');
printNumSummary(R, 'shadowDs5RefObsPosAccelMps2', mask, 'refObs position accel mps2');
printNumSummary(R, 'shadowRecoveredFilterMeasPredDiffM', mask, 'recoveredFilter measPredDiff m');
printNumSummary(R, 'shadowRecoveredFilterAbsAnchorDiffM', mask, 'recoveredFilter absAnchorDiff m');
printNumSummary(R, 'shadowRecoveredFilterBaselineDiffM', mask, 'recoveredFilter baselineDiff m');
printNumSummary(R, 'shadowFinalOutputRecoveryBaselineDiffM', mask, 'finalOutput recoveryBaselineDiff m');
printNumSummary(R, 'shadowFinalOutputContinuityDiffM', mask, 'finalOutput continuityDiff m');
printFinalSourceCounts(R, mask);
end

function printStrictRawIndependentDerived(R, mask)
requiredFields = {'shadowDs5RefObsUsed', 'shadowDs5RefObsTrackRawSampled', ...
    'shadowDs5RefObsTrackCoasted', 'shadowDs5RefObsSeedUseBase', ...
    'shadowDs5RefObsCodeErrChips', 'shadowDs5RefObsFreqErrHz'};
for ii = 1:numel(requiredFields)
    if ~isfield(R, requiredFields{ii})
        return;
    end
end

idx = find(mask);
maxCol = min([numel(mask), size(R.shadowDs5RefObsUsed, 2), ...
    size(R.shadowDs5RefObsTrackRawSampled, 2), size(R.shadowDs5RefObsTrackCoasted, 2), ...
    size(R.shadowDs5RefObsSeedUseBase, 2), size(R.shadowDs5RefObsCodeErrChips, 2), ...
    size(R.shadowDs5RefObsFreqErrHz, 2)]);
idx = idx(idx <= maxCol);
if isempty(idx)
    fprintf('strictRawIndependentMin4 derived: no samples\n');
    return;
end

used = logical(R.shadowDs5RefObsUsed(:, idx));
rawSampled = logical(R.shadowDs5RefObsTrackRawSampled(:, idx));
coasted = logical(R.shadowDs5RefObsTrackCoasted(:, idx));
seedUseBase = logical(R.shadowDs5RefObsSeedUseBase(:, idx));
codeErr = abs(R.shadowDs5RefObsCodeErrChips(:, idx));
freqErr = abs(R.shadowDs5RefObsFreqErrHz(:, idx));

quality = used & ~seedUseBase & isfinite(codeErr) & codeErr <= 1.60 & ...
    isfinite(freqErr) & freqErr <= 350.0;
if isfield(R, 'shadowDs5TrueRefReady') && size(R.shadowDs5TrueRefReady, 2) >= max(idx)
    quality = quality & logical(R.shadowDs5TrueRefReady(:, idx));
end
if isfield(R, 'shadowDs5RefObsTrackDriven') && size(R.shadowDs5RefObsTrackDriven, 2) >= max(idx)
    quality = quality & logical(R.shadowDs5RefObsTrackDriven(:, idx));
end
if isfield(R, 'shadowRawTrackCodeRefNcoPullHz') && size(R.shadowRawTrackCodeRefNcoPullHz, 2) >= max(idx)
    quality = quality & isfinite(R.shadowRawTrackCodeRefNcoPullHz(:, idx));
end
if isfield(R, 'shadowRawTrackPromptIP') && size(R.shadowRawTrackPromptIP, 2) >= max(idx)
    quality = quality & isfinite(R.shadowRawTrackPromptIP(:, idx));
end

minSat = 4;
strictRaw = quality & rawSampled & ~coasted;
fprintf('strictRawIndependentMin4 derived: %d/%d\n', nnz(sum(strictRaw, 1) >= minSat), numel(idx));
fprintf('strictRaw rawSampledUsed>=4 derived: %d/%d\n', nnz(sum(used & rawSampled, 1) >= minSat), numel(idx));
fprintf('strictRaw rawNonCoastedUsed>=4 derived: %d/%d\n', nnz(sum(used & rawSampled & ~coasted, 1) >= minSat), numel(idx));
fprintf('strictRaw noBaseSeedQuality>=4 derived: %d/%d\n', nnz(sum(quality, 1) >= minSat), numel(idx));
end

function printBoolCount(R, fieldName, mask, label)
if ~isfield(R, fieldName)
    fprintf('%s: missing field %s\n', label, fieldName);
    return;
end
xRaw = R.(fieldName);
x = isfinite(xRaw) & xRaw ~= 0;
if ismatrix(x) && size(x, 2) == numel(mask) && size(x, 1) > 1
    v = x(:, mask);
    fprintf('%s: %d/%d\n', label, nnz(v), numel(v));
elseif ismatrix(x) && size(x, 1) == numel(mask) && size(x, 2) > 1
    v = x(mask, :);
    fprintf('%s: %d/%d\n', label, nnz(v), numel(v));
else
    x = x(:).';
    v = false(1, numel(mask));
    m = min(numel(mask), numel(x));
    v(1:m) = x(1:m);
    fprintf('%s: %d/%d\n', label, sum(v & mask), sum(mask));
end
end

function printNumSummary(R, fieldName, mask, label)
if ~isfield(R, fieldName)
    fprintf('%s: missing field %s\n', label, fieldName);
    return;
end
x = R.(fieldName);
x = x(:).';
v = nan(1, numel(mask));
m = min(numel(mask), numel(x));
v(1:m) = x(1:m);
v = v(mask & isfinite(v));
if isempty(v)
    fprintf('%s: no finite samples\n', label);
else
    fprintf('%s: med/p95/max = %.3f / %.3f / %.3f\n', ...
        label, median(v, 'omitnan'), prctile(v, 95), max(v, [], 'omitnan'));
end
end

function printFinalSourceCounts(R, mask)
if ~isfield(R, 'shadowFinalOutputSource')
    fprintf('finalOutputSource: missing field\n');
    return;
end
src = R.shadowFinalOutputSource(:).';
srcUse = zeros(1, numel(mask));
m = min(numel(mask), numel(src));
srcUse(1:m) = src(1:m);
fprintf('finalOutput source counts: baseline=%d recovered=%d recoveredHold=%d trusted=%d baselineHold=%d zero=%d\n', ...
    sum(srcUse(mask) == 1), sum(srcUse(mask) == 2), sum(srcUse(mask) == 3), ...
    sum(srcUse(mask) == 4), sum(srcUse(mask) == 5), sum(srcUse(mask) == 0));
end
