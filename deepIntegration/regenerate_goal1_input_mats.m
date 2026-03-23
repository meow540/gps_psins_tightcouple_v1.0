function summaryTbl = regenerate_goal1_input_mats(datasetList)
% Regenerate Goal-1 clean/ds5/ds6 400 s MAT inputs from raw IF files
% using one explicit unified parameter set.
%
% Usage:
%   regenerate_goal1_input_mats()
%   regenerate_goal1_input_mats({'clean','ds5'})

if nargin < 1 || isempty(datasetList)
    datasetList = {'clean', 'ds5', 'ds6'};
end
if ischar(datasetList) || isstring(datasetList)
    datasetList = cellstr(datasetList);
end

rows = repmat(struct(), numel(datasetList), 1);
for ii = 1:numel(datasetList)
    s = reprocess_goal1_raw_dataset(datasetList{ii});
    rows(ii).dataset = s.datasetTag;
    rows(ii).trackedChannels = s.trackedChannels;
    rows(ii).frameSyncChannels = s.frameSyncChannels;
    rows(ii).validEphSatellites = s.validEphSatellites;
    rows(ii).navEpochs = s.navEpochs;
    rows(ii).output = string(s.output);
end

summaryTbl = struct2table(rows);
disp(summaryTbl);

thisDir = fileparts(mfilename('fullpath'));
csvFile = fullfile(thisDir, 'figures', 'goal1_unified_raw_rebuild_summary.csv');
writetable(summaryTbl, csvFile);
fprintf('[RAW] Summary written: %s\n', csvFile);
end
