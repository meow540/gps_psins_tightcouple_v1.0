clear;
clc;

projectRoot = fileparts(mfilename('fullpath'));
cd(projectRoot);

setenv('DS5_HEAP_SAFE_ANCHOR_MODE', 'proof');
setenv('DS5_HEAP_SAFE_END_SEC', '400');

fprintf('\n=== DS5 400s heap-safe PROOF validation ===\n');
fprintf('Anchor mode: proof, truth/trj anchor enabled for validation.\n\n');

run(fullfile(projectRoot, 'run_ds5_400s_heap_safe_true_tracking_goal.m'));
