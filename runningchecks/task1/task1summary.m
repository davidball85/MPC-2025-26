% run_all_checks.m
clear;  % keep this here only if you want a clean workspace at start

checks_dir = fileparts(mfilename('fullpath'));
project_root = fileparts(checks_dir);
addpath(genpath(project_root));

check_files = dir(fullfile(checks_dir, 'check_*.m'));

fprintf('\n============================================\n');
fprintf('RUNNING ALL CHECKS (%s)\n', datestr(now,'yyyy-mm-dd HH:MM:SS'));
fprintf('Project root: %s\n', project_root);
fprintf('Checks folder: %s\n', checks_dir);
fprintf('Found %d check scripts.\n', numel(check_files));
fprintf('============================================\n\n');

for k = 1:numel(check_files)
    fprintf('\n--------------------------------------------\n');
    fprintf('(%d/%d) %s\n', k, numel(check_files), check_files(k).name);
    fprintf('--------------------------------------------\n');
    run(fullfile(checks_dir, check_files(k).name));
end

fprintf('\n============================================\n');
fprintf('ALL CHECKS COMPLETED (%s)\n', datestr(now,'yyyy-mm-dd HH:MM:SS'));
fprintf('============================================\n');
