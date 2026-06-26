clear;
clc;

projectRoot = fileparts(mfilename('fullpath'));
cd(projectRoot);

setenv('DS5_HEAP_SAFE_ANCHOR_MODE', 'online_loose');
setenv('DS5_HEAP_SAFE_END_SEC', '400');

fprintf('\n=== DS5 400s heap-safe ONLINE_LOOSE validation ===\n');
fprintf('Anchor mode: online_loose, truth/trj anchor disabled, absAnchor hard gate off, absAnchor prior off.\n\n');

run(fullfile(projectRoot, 'run_ds5_400s_heap_safe_true_tracking_goal.m'));
