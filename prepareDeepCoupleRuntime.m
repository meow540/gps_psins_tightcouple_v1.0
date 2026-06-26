function deepCoupleScript = prepareDeepCoupleRuntime(projectRoot)
% Ensure DeepCouple_perINStime resolves to the editable .m under deepIntegration.

if nargin < 1 || isempty(projectRoot)
    projectRoot = fileparts(mfilename('fullpath'));
end

deepIntegrationDir = fullfile(projectRoot, 'deepIntegration');
deepCoupleScript = fullfile(deepIntegrationDir, 'DeepCouple_perINStime.m');
if exist(deepCoupleScript, 'file') ~= 2
    error('GOAL2:MissingDeepCoupleScript', 'Missing expected DeepCouple script: %s', deepCoupleScript);
end

legacyPacked = fullfile(projectRoot, 'DeepCouple_perINStime.p');
if exist(legacyPacked, 'file') == 2
    error('GOAL2:LegacyPackedDeepCouplePresent', ...
        'Packed DeepCouple still present and may shadow edits: %s', legacyPacked);
end

if ~contains(path, [projectRoot pathsep]) && ~strcmp(path, projectRoot)
    addpath(projectRoot);
end
addpath(deepIntegrationDir, '-begin');
rehash;
clear('DeepCouple_perINStime');

resolvedDeepCouple = which('DeepCouple_perINStime');
if ~strcmpi(resolvedDeepCouple, deepCoupleScript)
    allResolved = which('DeepCouple_perINStime', '-all');
    if ischar(allResolved)
        allResolved = cellstr(allResolved);
    end
    detail = strjoin(allResolved, ' | ');
    error('GOAL2:DeepCoupleResolutionMismatch', ...
        'DeepCouple_perINStime resolved to %s, expected %s. all=%s', ...
        resolvedDeepCouple, deepCoupleScript, detail);
end
end
