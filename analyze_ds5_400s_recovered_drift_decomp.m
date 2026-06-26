clear;
clc;

summaryStartSec = 180.0;
resultFile = localPickResultFile();
if exist(resultFile, 'file') ~= 2
    error('Missing result file: %s', resultFile);
end

S = load(resultFile, 'navResults', 'settingsOverride');
R = S.navResults;
cfg = struct();
if isfield(S, 'settingsOverride')
    cfg = S.settingsOverride;
end
n = numel(R.X);
t = (0:n-1) * 0.5;
mask = t >= summaryStartSec;

ds5Base = nan(3, n);
if exist('ds5_400s.mat', 'file') == 2
    B = load('ds5_400s.mat', 'navSolutions');
    nb = min(n, numel(B.navSolutions.X));
    ds5Base(:, 1:nb) = [B.navSolutions.X(1:nb); B.navSolutions.Y(1:nb); B.navSolutions.Z(1:nb)];
end

cleanBase = nan(3, n);
if exist('cleandynamic_400s.mat', 'file') == 2
    C = load('cleandynamic_400s.mat', 'navSolutions');
    nc = min(n, numel(C.navSolutions.X));
    cleanBase(:, 1:nc) = [C.navSolutions.X(1:nc); C.navSolutions.Y(1:nc); C.navSolutions.Z(1:nc)];
end

fprintf('\n=== DS5 400s recovered drift decomposition ===\n');
fprintf('Result: %s\n', resultFile);
fprintf('Window: %.1f..%.1f s (%d epochs)\n', min(t(mask)), max(t(mask)), nnz(mask));
fprintf('\n--- Runtime config audit ---\n');
printCfgVal(cfg, 'deepShadowDs5ValidationProofMode');
printCfgVal(cfg, 'deepShadowDs5ValidationAnchorModeName');
printCfgVal(cfg, 'deepShadowTrustedAnchorUseTruthTrj');
printCfgVal(cfg, 'deepShadowTrustedAnchorPreferTruthTrj');
printCfgVal(cfg, 'deepShadowTrustedAnchorTruthUseCurrentEpoch');
printCfgVal(cfg, 'deepShadowTrustedAnchorUseInsPropagated');
printCfgVal(cfg, 'deepShadowTrustedAnchorInsPropagatedUseCurrent');
printCfgVal(cfg, 'deepShadowTrustedAnchorUseBaselineInsAlign');
printCfgVal(cfg, 'deepShadowTrustedAnchorAllowBaselineFedAnchor');
printCfgVal(cfg, 'deepShadowTrustedAnchorAllowHigherPriorityReplace');
printCfgVal(cfg, 'deepShadowTrustedAnchorRefreshSameSource');
printCfgVal(cfg, 'deepShadowDs5BaselineTrustedRefUseVelocity');
printCfgVal(cfg, 'deepShadowDs5RecoveredFilterAbsAnchorFreezeSec');
printCfgVal(cfg, 'deepShadowDs5RecoveredFilterAbsAnchorUseInsVel');
printCfgVal(cfg, 'deepShadowDs5RecoveredFilterAbsAnchorRefreshSameSource');
printCfgVal(cfg, 'deepShadowDs5RefObsPositionPreferAbsAnchor');
printCfgVal(cfg, 'deepShadowDs5RefObsPositionAbsAnchorUseHardGate');
printCfgVal(cfg, 'deepShadowDs5RefObsPositionAbsAnchorSoftMaxDiffM');
printCfgVal(cfg, 'deepShadowDs5RefObsPositionAbsAnchorMaxDiffM');
printCfgVal(cfg, 'deepShadowDs5RefObsPositionAbsAnchorPriorEnable');
printCfgVal(cfg, 'deepShadowDs5RefObsPositionAbsAnchorPriorSigmaM');
printCfgVal(cfg, 'deepShadowDs5RefObsPositionAbsAnchorPriorSoftSigmaM');
printCfgVal(cfg, 'deepShadowDs5RefObsPositionCorrMaxM');
printCfgVal(cfg, 'deepShadowDs5RefObsPositionJumpMaxM');
printCfgVal(cfg, 'deepShadowDs5RefObsPositionCommonClockRebaseEnable');
printCfgVal(cfg, 'deepShadowDs5RefObsPositionCommonClockRebaseEachDelta');
printCfgVal(cfg, 'deepShadowDs5RefObsPositionCommonClockRebasePeriodM');
if cfgNum(cfg, 'deepShadowDs5ValidationProofMode', 0) ~= 0 || ...
        cfgNum(cfg, 'deepShadowTrustedAnchorUseTruthTrj', 0) ~= 0
    fprintf('CONFIG WARNING: this is a proof/truth-anchor validation, not an online deployable validation.\n');
else
    fprintf('CONFIG NOTE: this MAT does not request truth/trj anchor; treat it as online-anchor validation.\n');
end
if cfgNum(cfg, 'deepShadowDs5BaselineTrustedRefUseVelocity', 1) ~= 0 || ...
        cfgNum(cfg, 'deepShadowDs5RecoveredFilterAbsAnchorUseInsVel', 1) ~= 0
    fprintf('CONFIG NOTE: this MAT can use velocity-projected absAnchor; rerun after anchor-freeze override for final validation.\n');
else
    fprintf('CONFIG NOTE: this MAT requests frozen absAnchor propagation; residual drift is not from anchor velocity projection.\n');
end

mainNav = getVec(R, 'X', 'Y', 'Z');
recovered = getVec(R, 'shadowRecoveredFilterX', 'shadowRecoveredFilterY', 'shadowRecoveredFilterZ');
finalOut = getVec(R, 'shadowFinalOutputX', 'shadowFinalOutputY', 'shadowFinalOutputZ');
baselineOut = getVec(R, 'shadowBaselineOutputX', 'shadowBaselineOutputY', 'shadowBaselineOutputZ');
trustedAnchor = getVec(R, 'shadowTrustedAnchorX', 'shadowTrustedAnchorY', 'shadowTrustedAnchorZ');
refObs = getVec(R, 'shadowDs5RefObsPosX', 'shadowDs5RefObsPosY', 'shadowDs5RefObsPosZ');
refObsAnchor = getVec(R, 'shadowDs5RefObsPosAnchorX', 'shadowDs5RefObsPosAnchorY', 'shadowDs5RefObsPosAnchorZ');
absAnchor = getVec(R, 'shadowRecoveredFilterAbsAnchorX', 'shadowRecoveredFilterAbsAnchorY', 'shadowRecoveredFilterAbsAnchorZ');
absAnchorVel = getVec(R, 'shadowRecoveredFilterAbsAnchorVX', 'shadowRecoveredFilterAbsAnchorVY', 'shadowRecoveredFilterAbsAnchorVZ');

printDist('DS5 raw baseline-clean baseline', dist3(ds5Base, cleanBase), mask);
printDist('mainNav-clean baseline', dist3(mainNav, cleanBase), mask);
printDist('baselineOutput-clean baseline', dist3(baselineOut, cleanBase), mask);
printDist('recovered-clean baseline', dist3(recovered, cleanBase), mask);
printDist('refObs-clean baseline', dist3(refObs, cleanBase), mask);
printDist('final-clean baseline', dist3(finalOut, cleanBase), mask);
printDist('absAnchor-clean baseline', dist3(absAnchor, cleanBase), mask);
printDist('recovered-DS5 raw baseline', dist3(recovered, ds5Base), mask);
printDist('baselineOutput-DS5 raw baseline', dist3(baselineOut, ds5Base), mask);
printDist('refObs-DS5 raw baseline', dist3(refObs, ds5Base), mask);
printDist('absAnchor-DS5 raw baseline', dist3(absAnchor, ds5Base), mask);
printDist('recovered-mainNav direct', dist3(recovered, mainNav), mask);
printDist('baselineOutput-mainNav direct', dist3(baselineOut, mainNav), mask);
printDist('final-mainNav direct', dist3(finalOut, mainNav), mask);
printDist('recovered-final direct', dist3(recovered, finalOut), mask);
printDist('trustedAnchor-mainNav direct', dist3(trustedAnchor, mainNav), mask);
printDist('trustedAnchor-clean baseline', dist3(trustedAnchor, cleanBase), mask);
printDist('trustedAnchor-DS5 raw baseline', dist3(trustedAnchor, ds5Base), mask);
printDist('refObs-recovered direct', dist3(refObs, recovered), mask);
printDist('refObs-anchor direct', dist3(refObs, refObsAnchor), mask);
printDist('refObs-absAnchor direct', dist3(refObs, absAnchor), mask);
printDist('recovered-absAnchor direct', dist3(recovered, absAnchor), mask);
printDist('DS5 raw baseline-absAnchor direct', dist3(ds5Base, absAnchor), mask);
printDist('refObsAnchor-absAnchor direct', dist3(refObsAnchor, absAnchor), mask);

idxAnchor = find(mask & all(isfinite(absAnchor), 1));
if ~isempty(idxAnchor)
    frozenAnchor = repmat(absAnchor(:, idxAnchor(1)), 1, n);
    printDist('frozen first absAnchor-clean baseline', dist3(frozenAnchor, cleanBase), mask);
    printDist('projected absAnchor-clean baseline', dist3(absAnchor, cleanBase), mask);
    printDist('absAnchor displacement from first anchor', dist3(absAnchor, frozenAnchor), mask);
    printNum('absAnchor velocity norm', sqrt(sum(absAnchorVel.^2, 1)), mask);
end

printField('reported recoveredFilter baselineDiff', R, 'shadowRecoveredFilterBaselineDiffM', mask);
printField('reported recoveredFilter absAnchorDiff', R, 'shadowRecoveredFilterAbsAnchorDiffM', mask);
printField('reported finalOutput recoveryBaselineDiff', R, 'shadowFinalOutputRecoveryBaselineDiffM', mask);
printField('refObs position anchorDiff', R, 'shadowDs5RefObsPosAnchorDiffM', mask);
printField('refObs position corr', R, 'shadowDs5RefObsPosCorrM', mask);
printField('obsContract commonClock', R, 'shadowDs5ObsContractCommonClockM', mask);
printField('refObs position clockM', R, 'shadowDs5RefObsPosClockM', mask);
printField('recoveredFilter clockM', R, 'shadowRecoveredFilterClockM', mask);
printField('recoveredFilter clockRateMps', R, 'shadowRecoveredFilterClockRateMps', mask);

fprintf('\n--- Common-clock branch audit ---\n');
clockPeriodM = cfgNum(cfg, 'deepShadowDs5RefObsPositionCommonClockRebasePeriodM', 299792.458);
commonClock = getField(R, 'shadowDs5ObsContractCommonClockM');
filterClock = getField(R, 'shadowRecoveredFilterClockM');
printBranchCounts('obsContract commonClock branch', commonClock, clockPeriodM, mask);
printNum('obsContract commonClock modulo branch', commonClock - clockPeriodM .* round(commonClock ./ clockPeriodM), mask);
printBranchCounts('recoveredFilter clock branch', filterClock, clockPeriodM, mask);
printNum('recoveredFilter clock modulo branch', filterClock - clockPeriodM .* round(filterClock ./ clockPeriodM), mask);

fprintf('\n--- Source counts ---\n');
printCounts(R, 'shadowRecoveredFilterSource', mask);
printCounts(R, 'shadowRecoveredFilterAbsAnchorSource', mask);
printCounts(R, 'shadowTrustedAnchorSource', mask);
printCounts(R, 'shadowDs5RefObsPosAnchorSource', mask);
printCounts(R, 'shadowFinalOutputSource', mask);

fprintf('\n--- Correlations in 180s window ---\n');
reportCorr('absAnchorDiff vs commonClock', getField(R, 'shadowRecoveredFilterAbsAnchorDiffM'), getField(R, 'shadowDs5ObsContractCommonClockM'), mask);
reportCorr('baselineDiff vs commonClock', getField(R, 'shadowRecoveredFilterBaselineDiffM'), getField(R, 'shadowDs5ObsContractCommonClockM'), mask);
reportCorr('absAnchorDiff vs refObs-anchor', getField(R, 'shadowRecoveredFilterAbsAnchorDiffM'), getField(R, 'shadowDs5RefObsPosAnchorDiffM'), mask);
reportCorr('baselineDiff vs final continuity', getField(R, 'shadowRecoveredFilterBaselineDiffM'), getField(R, 'shadowFinalOutputContinuityDiffM'), mask);
reportCorr('baselineOutput-clean baseline', dist3(baselineOut, cleanBase), getField(R, 'shadowDs5ObsContractCommonClockM'), mask);

fprintf('\n--- Segment summaries ---\n');
edges = [180 220 260 300 340 380 inf];
for ii = 1:numel(edges)-1
    seg = mask & t >= edges(ii) & t < edges(ii+1);
    if ~any(seg), continue; end
    if isfinite(edges(ii+1))
        label = sprintf('%.0f-%.0fs', edges(ii), edges(ii+1));
    else
        label = sprintf('%.0f-end', edges(ii));
    end
    fprintf('\n[%s] n=%d\n', label, nnz(seg));
    printDist('  recovered-clean', dist3(recovered, cleanBase), seg);
    printDist('  recovered-DS5 raw baseline', dist3(recovered, ds5Base), seg);
    printDist('  recovered-absAnchor', dist3(recovered, absAnchor), seg);
    printDist('  refObs-clean', dist3(refObs, cleanBase), seg);
    printDist('  refObs-DS5 raw baseline', dist3(refObs, ds5Base), seg);
    printDist('  refObs-absAnchor', dist3(refObs, absAnchor), seg);
    printField('  commonClock', R, 'shadowDs5ObsContractCommonClockM', seg);
end

fprintf('\n--- Worst recovered-baseline epochs ---\n');
d = dist3(recovered, ds5Base);
idx = find(mask & isfinite(d));
[~, ord] = sort(d(idx), 'descend');
idx = idx(ord(1:min(12, numel(ord))));
for kk = 1:numel(idx)
    ii = idx(kk);
    fprintf('t=%6.1f diff=%8.2f absAnchor=%8.2f refObsBase=%8.2f commonClock=%10.2f source=%g anchorSrc=%g finalSrc=%g\n', ...
        t(ii), d(ii), getAt(R, 'shadowRecoveredFilterAbsAnchorDiffM', ii), ...
        dist3At(recovered, cleanBase, ii), getAt(R, 'shadowDs5ObsContractCommonClockM', ii), ...
        getAt(R, 'shadowRecoveredFilterSource', ii), getAt(R, 'shadowRecoveredFilterAbsAnchorSource', ii), ...
        getAt(R, 'shadowFinalOutputSource', ii));
end

function resultFile = localPickResultFile()
envFile = strtrim(getenv('DS5_ANALYZE_RESULT_FILE'));
if ~isempty(envFile)
    resultFile = envFile;
    return;
end
anchorMode = strtrim(getenv('DS5_HEAP_SAFE_ANCHOR_MODE'));
if strcmpi(anchorMode, 'online_loose')
    preferred = 'rt_deep_goal2_ds5_heap_safe_true_tracking_goal_400s_online_loose_ds5_399s.mat';
elseif strcmpi(anchorMode, 'online')
    preferred = 'rt_deep_goal2_ds5_heap_safe_true_tracking_goal_400s_online_ds5_399s.mat';
elseif strcmpi(anchorMode, 'online_ins')
    preferred = 'rt_deep_goal2_ds5_heap_safe_true_tracking_goal_400s_online_ins_ds5_399s.mat';
elseif strcmpi(anchorMode, 'online_prop')
    preferred = 'rt_deep_goal2_ds5_heap_safe_true_tracking_goal_400s_online_prop_ds5_399s.mat';
elseif strcmpi(anchorMode, 'online_base')
    preferred = 'rt_deep_goal2_ds5_heap_safe_true_tracking_goal_400s_online_base_ds5_399s.mat';
else
    preferred = 'rt_deep_goal2_ds5_heap_safe_true_tracking_goal_400s_ds5_399s.mat';
end
if exist(preferred, 'file') == 2
    resultFile = preferred;
    return;
end
candidates = dir('rt_deep_goal2_ds5_heap_safe_true_tracking_goal_400s*_ds5_*s.mat');
if isempty(candidates)
    resultFile = preferred;
    return;
end
[~, newestIdx] = max([candidates.datenum]);
resultFile = candidates(newestIdx).name;
fprintf('Preferred result not found; using newest candidate: %s\n', resultFile);
end

function V = getVec(R, fx, fy, fz)
n = numel(R.X);
V = nan(3, n);
if isfield(R, fx), V(1, :) = R.(fx)(1:n); end
if isfield(R, fy), V(2, :) = R.(fy)(1:n); end
if isfield(R, fz), V(3, :) = R.(fz)(1:n); end
end

function x = getField(R, f)
if isfield(R, f)
    x = R.(f);
else
    x = nan(size(R.X));
end
x = x(:).';
end

function x = getAt(R, f, idx)
x = nan;
if isfield(R, f) && numel(R.(f)) >= idx
    x = R.(f)(idx);
end
end

function d = dist3(A, B)
d = sqrt(sum((A - B).^2, 1));
end

function d = dist3At(A, B, idx)
d = nan;
if size(A, 2) >= idx && size(B, 2) >= idx
    d = norm(A(:, idx) - B(:, idx));
end
end

function printDist(label, x, mask)
printNum(label, x, mask);
end

function printField(label, R, f, mask)
printNum(label, getField(R, f), mask);
end

function printNum(label, x, mask)
x = x(:).';
v = x(mask & isfinite(x));
if isempty(v)
    fprintf('%s: no finite samples\n', label);
    return;
end
fprintf('%s: med/p95/max/min = %.3f / %.3f / %.3f / %.3f\n', ...
    label, median(v, 'omitnan'), prctile(v, 95), max(v), min(v));
end

function printCounts(R, f, mask)
if ~isfield(R, f)
    fprintf('%s: missing\n', f);
    return;
end
x = R.(f)(:).';
v = x(mask & isfinite(x));
if isempty(v)
    fprintf('%s: no finite samples\n', f);
    return;
end
vals = unique(v);
fprintf('%s:', f);
for ii = 1:numel(vals)
    fprintf(' %g=%d', vals(ii), nnz(v == vals(ii)));
end
fprintf('\n');
end

function reportCorr(label, a, b, mask)
a = a(:).';
b = b(:).';
good = mask & isfinite(a) & isfinite(b);
if nnz(good) < 3
    fprintf('%s: insufficient samples\n', label);
    return;
end
c = corrcoef(a(good), b(good));
fprintf('%s: r=%.4f n=%d\n', label, c(1,2), nnz(good));
end

function printCfgVal(cfg, f)
if isfield(cfg, f)
    v = cfg.(f);
    if isnumeric(v) || islogical(v)
        if isscalar(v)
            fprintf('%s=%g\n', f, double(v));
        else
            fprintf('%s=[%d values]\n', f, numel(v));
        end
    else
        fprintf('%s=<%s>\n', f, class(v));
    end
else
    fprintf('%s=<missing>\n', f);
end
end

function v = cfgNum(cfg, f, defaultValue)
v = defaultValue;
if isfield(cfg, f) && isnumeric(cfg.(f)) && isscalar(cfg.(f)) && isfinite(cfg.(f))
    v = double(cfg.(f));
end
end

function printBranchCounts(label, x, periodM, mask)
x = x(:).';
if ~isfinite(periodM) || periodM <= 0
    fprintf('%s: invalid period\n', label);
    return;
end
branch = round(x ./ periodM);
v = branch(mask & isfinite(branch));
if isempty(v)
    fprintf('%s: no finite samples\n', label);
    return;
end
vals = unique(v);
fprintf('%s:', label);
for ii = 1:numel(vals)
    fprintf(' %g=%d', vals(ii), nnz(v == vals(ii)));
end
fprintf('\n');
end
