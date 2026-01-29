function S = task3_summary()
% task3_summary
% Runs Task 3 checks in order.

clc;

checks = { ...
    'check_01_augmented_model', ...
    'check_02_kalman_observer', ...
    'check_03_target_calc', ...
    'check_04_offset_free_mpc' ...
};

S.results = struct('name', {}, 'pass', {}, 'notes', {}, 'error', {});
S.allPass = true;

fprintf('============================================\n');
fprintf('RUNNING TASK 3 CHECKS (%s)\n', datestr(now,'yyyy-mm-dd HH:MM:SS'));
fprintf('Project: MPC 2025-26\n');
fprintf('============================================\n\n');

for i = 1:numel(checks)
    fname = checks{i};
    fprintf('--------------------------------------------\n');
    fprintf('(%d/%d) %s\n', i, numel(checks), fname);
    fprintf('--------------------------------------------\n');

    if exist(fname,'file') ~= 2
        R = struct('pass', false, 'notes', {{}}, 'error', sprintf('File not found: %s.m', fname));
    else
        R = feval(fname);
    end

    S.results(i).name  = fname;
    S.results(i).pass  = logical(R.pass);
    S.results(i).notes = R.notes;
    S.results(i).error = R.error;

       if R.pass
        fprintf('Result: PASS\n');
    else
        fprintf('Result: FAIL\n');
    end

    % Always print notes (these are report-ready)
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
