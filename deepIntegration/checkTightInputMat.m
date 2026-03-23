function report = checkTightInputMat(matFile, minSat)
% checkTightInputMat Validate whether a MAT workspace can drive realtime_tightCouple.
%
% Usage:
%   report = checkTightInputMat('ds5_6prn_150s.mat');
%   report = checkTightInputMat('myworkspace_400s.mat', 4);
%
% It checks:
%   1) required variables exist
%   2) tracked channels and frame-sync channels
%   3) each frame-sync channel has enough 30s nav bits for ephemeris decode
%   4) eph structure validity by PRN
%   5) minimum satellite count for tight coupling

if nargin < 2
    minSat = 4;
end

report = struct();
report.file = matFile;
report.ok = false;
report.messages = {};

if ~exist(matFile, 'file')
    report.messages{end+1} = sprintf('File does not exist: %s', matFile); %#ok<AGROW>
    printReport(report);
    return;
end

ws = whos('-file', matFile);
varNames = {ws.name};
required = {'trackResults', 'channel', 'TOW', 'eph', 'subFrameStart'};
missing = setdiff(required, varNames);
if ~isempty(missing)
    report.messages{end+1} = ['Missing variables: ', strjoin(missing, ', ')]; %#ok<AGROW>
    printReport(report);
    return;
end

S = load(matFile, 'trackResults', 'channel', 'TOW', 'eph', 'subFrameStart');

trackResults = S.trackResults;
subFrameStart = S.subFrameStart;
eph = S.eph;

if isempty(trackResults) || ~isstruct(trackResults)
    report.messages{end+1} = 'trackResults is empty or invalid.'; %#ok<AGROW>
    printReport(report);
    return;
end

if numel(subFrameStart) < numel(trackResults)
    report.messages{end+1} = 'subFrameStart length is smaller than trackResults channels.'; %#ok<AGROW>
    printReport(report);
    return;
end

status = [trackResults.status];
trackedCh = find(status ~= '-');
frameSyncCh = trackedCh(subFrameStart(trackedCh) > 1);

report.numChannels = numel(trackResults);
report.numTracked = numel(trackedCh);
report.numFrameSync = numel(frameSyncCh);
report.trackedChannels = trackedCh;
report.frameSyncChannels = frameSyncCh;

hasEnoughBits = false(1, numel(frameSyncCh));
prnList = zeros(1, numel(frameSyncCh));
for i = 1:numel(frameSyncCh)
    ch = frameSyncCh(i);
    prnList(i) = trackResults(ch).PRN;
    iP = trackResults(ch).I_P;
    sf = subFrameStart(ch);
    startIdx = sf - 20;
    endIdx = sf + 1500 * 20 - 1;
    hasEnoughBits(i) = ~isempty(iP) && startIdx >= 1 && endIdx <= numel(iP);
end

report.frameSyncPrn = prnList;
report.hasEnough30sBits = hasEnoughBits;
report.numEnough30sBits = sum(hasEnoughBits);

if isempty(eph)
    report.ephEmpty = true;
    report.ephValidByPrn = false(size(prnList));
    report.messages{end+1} = 'eph is empty (likely not enough valid subframes/ephemeris decode failed).'; %#ok<AGROW>
else
    report.ephEmpty = false;
    ephValid = false(size(prnList));
    for i = 1:numel(prnList)
        ephValid(i) = isValidEphForPrn(eph, prnList(i));
    end
    report.ephValidByPrn = ephValid;
end

validChMask = hasEnoughBits & report.ephValidByPrn;
report.validPrnForTight = prnList(validChMask);
report.numValidForTight = numel(report.validPrnForTight);

if report.numValidForTight < minSat
    report.messages{end+1} = sprintf('Valid satellites for tight coupling = %d < %d.', report.numValidForTight, minSat); %#ok<AGROW>
    if report.numFrameSync > 0 && report.numEnough30sBits < report.numFrameSync
        report.messages{end+1} = 'Some channels have preamble but not enough 30s data window for ephemeris decode.'; %#ok<AGROW>
    end
else
    report.ok = true;
end

printReport(report);
end


function tf = isValidEphForPrn(eph, prn)
tf = false;
if isempty(eph) || numel(eph) < prn || isempty(eph(prn))
    return;
end

needed = {'IODC', 'IODE_sf2', 'IODE_sf3', 'a_f0', 'a_f1', 'a_f2', 't_oc'};
for i = 1:numel(needed)
    if ~isfield(eph(prn), needed{i})
        return;
    end
end

tf = ~isempty(eph(prn).IODC) && ~isempty(eph(prn).IODE_sf2) && ...
     ~isempty(eph(prn).IODE_sf3) && isscalar(eph(prn).a_f0) && ...
     isscalar(eph(prn).a_f1) && isscalar(eph(prn).a_f2) && ...
     isscalar(eph(prn).t_oc);
end


function printReport(r)
fprintf('\n==== Tight-Coupling MAT Check ====\n');
fprintf('File: %s\n', r.file);
if isfield(r, 'numChannels')
    fprintf('Channels: total=%d, tracked=%d, frameSync=%d\n', ...
        r.numChannels, r.numTracked, r.numFrameSync);
end
if isfield(r, 'frameSyncPrn') && ~isempty(r.frameSyncPrn)
    fprintf('FrameSync PRNs: ');
    fprintf('%d ', r.frameSyncPrn);
    fprintf('\n');
end
if isfield(r, 'hasEnough30sBits') && ~isempty(r.hasEnough30sBits)
    fprintf('Enough 30s bits per FrameSync PRN: ');
    for i = 1:numel(r.frameSyncPrn)
        fprintf('[%d:%d] ', r.frameSyncPrn(i), r.hasEnough30sBits(i));
    end
    fprintf('\n');
end
if isfield(r, 'ephValidByPrn') && ~isempty(r.ephValidByPrn)
    fprintf('EPH valid per FrameSync PRN: ');
    for i = 1:numel(r.frameSyncPrn)
        fprintf('[%d:%d] ', r.frameSyncPrn(i), r.ephValidByPrn(i));
    end
    fprintf('\n');
end
if isfield(r, 'validPrnForTight')
    fprintf('Valid PRNs for tight-coupling: ');
    fprintf('%d ', r.validPrnForTight);
    fprintf('\n');
    fprintf('Valid count: %d\n', r.numValidForTight);
end
if ~isempty(r.messages)
    fprintf('Messages:\n');
    for i = 1:numel(r.messages)
        fprintf('  - %s\n', r.messages{i});
    end
end
fprintf('Result: %s\n', ternary(r.ok, 'PASS', 'FAIL'));
fprintf('==================================\n\n');
end


function out = ternary(cond, a, b)
if cond
    out = a;
else
    out = b;
end
end

