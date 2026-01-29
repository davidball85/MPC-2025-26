function S = task3_summary()
% task3_summary
% Runs Task 3 checks in order and prints a report-ready console summary.
%
% Extra behaviour:
%   - Any figures created by a check are automatically exported to PNG
%     into the same folder as this script (runningchecks/task3).
%   - Figures remain docked/visible as normal in MATLAB.

% ----- Self-contained setup -----
thisFile   = mfilename('fullpath');
task3Dir   = fileparts(thisFile);
projectRoot = fileparts(fileparts(task3Dir));   % .../runningchecks/task3 -> project root

addpath(genpath(projectRoot));

% Single source of truth: load config (constants first, then tuning)
run(fullfile(projectRoot, 'config', 'config_constants.m'));
run(fullfile(projectRoot, 'config', 'config_tuning.m'));

clc;

outdir = task3Dir;  % Save PNGs alongside the task3 scripts

checks = { ...
    'check_01_augmented_model', ...
    'check_02_kalman_observer', ...
    'check_03_target_calc', ...
    'check_04_offset_free_mpc' ...
};

S.results = struct('name', {}, 'pass', {}, 'notes', {}, 'error', {});
S.allPass = true;
S.timestamp = datestr(now,'yyyy-mm-dd HH:MM:SS');
S.projectRoot = projectRoot;

fprintf('============================================\n');
fprintf('RUNNING TASK 3 CHECKS (%s)\n', S.timestamp);
fprintf('Project: MPC 2025-26\n');
fprintf('Project root: %s\n', projectRoot);
fprintf('Checks folder: %s\n', task3Dir);
fprintf('============================================\n\n');

for i = 1:numel(checks)
    fname = checks{i};
    fprintf('--------------------------------------------\n');
    fprintf('(%d/%d) %s\n', i, numel(checks), fname);
    fprintf('--------------------------------------------\n');

    % Capture figure handles BEFORE running the check
    figs_before = findall(0, 'Type', 'figure');

    % Run the check
    if exist(fname,'file') ~= 2
        R = struct('pass', false, 'notes', {{}}, 'error', sprintf('File not found: %s.m', fname));
    else
        try
            R = feval(fname);
        catch ME
            R = struct('pass', false, 'notes', {{}}, 'error', ME.message);
        end
    end

    % Export any new figures created by this check
    % (does not affect docking or visibility)
    try
        export_new_figures(figs_before, sprintf('task3_%s', fname), outdir);
    catch MEexp
        % Don't fail the check just because export failed
        if ~isfield(R,'notes') || isempty(R.notes)
            R.notes = {};
        end
        R.notes{end+1} = sprintf('PNG export warning: %s', MEexp.message);
    end

    % Store results
    S.results(i).name  = fname;
    S.results(i).pass  = logical(R.pass);
    S.results(i).notes = R.notes;
    S.results(i).error = R.error;

    % Print result
    if R.pass
        fprintf('Result: PASS\n');
    else
        fprintf('Result: FAIL\n');
    end

    % Always print notes (report-ready)
    for k = 1:numel(R.notes)
        fprintf('  - %s\n', R.notes{k});
    end

    % Print error if present
    if ~isempty(R.error)
        fprintf('  Error: %s\n', R.error);
    end

    if ~R.pass
        fprintf('  Suggested next step:\n');
        fprintf('    - Open %s.m and fix the reported issue.\n', fname);
        fprintf('    - Re-run task3_summary.\n');
        S.allPass = false;
    end

    fprintf('\n');
end

fprintf('============================================\n');
if S.allPass
    fprintf('TASK 3 CHECK SUMMARY: ALL PASS\n');
else
    fprintf('TASK 3 CHECK SUMMARY: FAILURES PRESENT\n');
end
fprintf('============================================\n');

end
