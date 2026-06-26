clear;
clc;

projectRoot = fileparts(mfilename('fullpath'));
cd(projectRoot);

setenv('DS5_HEAP_SAFE_ANCHOR_MODE', 'online');
setenv('DS5_HEAP_SAFE_END_SEC', '400');

fprintf('\n=== DS5 400s heap-safe ONLINE validation ===\n');
fprintf('Anchor mode: online, truth/trj anchor disabled.\n\n');

run(fullfile(projectRoot, 'run_ds5_400s_heap_safe_true_tracking_goal.m'));
