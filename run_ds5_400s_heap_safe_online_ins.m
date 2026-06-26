clear;
clc;

projectRoot = fileparts(mfilename('fullpath'));
cd(projectRoot);

setenv('DS5_HEAP_SAFE_ANCHOR_MODE', 'online_ins');
setenv('DS5_HEAP_SAFE_END_SEC', '400');

fprintf('\n=== DS5 400s heap-safe ONLINE_INS validation ===\n');
fprintf('Anchor mode: online_ins, truth/trj anchor disabled, baseline-aligned trusted anchor disabled, absAnchor gate/prior off.\n\n');

run(fullfile(projectRoot, 'run_ds5_400s_heap_safe_true_tracking_goal.m'));
