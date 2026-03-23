function summaryTbl = run_goal1_unified_400s_batch(opts)
% Run unified Goal-1 settings on four 400s cases:
% ds5-ins1, ds5-ins2, ds6-ins1, ds6-ins2 using NORMAL -> INS_ONLY -> GNSS_RAMP -> NORMAL.

if nargin < 1 || isempty(opts)
    opts = struct();
end

cases = {
    'ds5', 'ins1';
    'ds5', 'ins2';
    'ds6', 'ins1';
    'ds6', 'ins2';
    };

rows = repmat(struct(), size(cases, 1), 1);
for i = 1:size(cases, 1)
    ds = cases{i,1};
    pr = cases{i,2};
    fprintf('\n[RUN] %s + %s\n', upper(ds), upper(pr));
    s = run_goal1_unified_400s_insonly(ds, pr, opts);
    rows(i).case = sprintf('%s_%s', ds, pr);
    rows(i).alarms = s.alarms;
    rows(i).firstAlarmSec = s.firstAlarmSec;
    rows(i).insOnlyEpochs = s.insOnlyEpochs;
    rows(i).gnssRampEpochs = s.gnssRampEpochs;
    rows(i).mitigationEpochs = s.insTakeoverEpochs;
    rows(i).endErr3D = s.endErr3D;
    rows(i).endErrH = s.endErrH;
    rows(i).output = s.output;
end

summaryTbl = struct2table(rows);
disp(summaryTbl);

outCsv = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'deepIntegration', 'figures', 'goal1_unified_400s_summary.csv');
writetable(summaryTbl, outCsv);
fprintf('[INFO] Summary written: %s\n', outCsv);
end
