function S = task2_summary()
% task2_summary
%
% Runs the Task 2 running checks in order and prints a clean report.
%
% Output
%   S.results  struct array with fields: name, pass, notes, error
%   S.allPass  overall pass/fail

clc;

checks = { ...
    % %'check_01_yalmip_setup', ...
    'check_02_constraints', ...
    'check_03_wall_logic' ...
    'check_04_independent_qp_match' ...
    %'check_05_tuning_sweep' ...
};

S.results = struct('name', {}, 'pass', {}, 'notes', {}, 'error', {});
S.allPass = true;

fprintf('============================================\n');
fprintf('RUNNING TASK 2 CHECKS (%s)\n', datestr(now,'yyyy-mm-dd HH:MM:SS'));
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
        for k = 1:numel(R.notes)
            fprintf('  - %s\n', R.notes{k});
        end
    else
        fprintf('Result: FAIL\n');
        if ~isempty(R.error)
            fprintf('  Error: %s\n', R.error);
        end
        fprintf('  Suggested next step:\n');
        fprintf('    - Open %s.m and fix the reported issue.\n', fname);
        fprintf('    - Re-run task2_summary.\n');
        S.allPass = false;
    end

    fprintf('\n');
end

fprintf('============================================\n');
if S.allPass
    fprintf('TASK 2 CHECK SUMMARY: ALL PASS\n');
else
    fprintf('TASK 2 CHECK SUMMARY: FAILURES PRESENT\n');
end
fprintf('============================================\n');


end
