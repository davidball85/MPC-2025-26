function task3_summary()
% task3_summary
% Runs Task 3 checks and prints report-friendly notes.

clc;
fprintf('============================================\n');
fprintf('RUNNING TASK 3 CHECKS (%s)\n', datestr(now));
fprintf('Folder: %s\n', fileparts(mfilename('fullpath')));
fprintf('============================================\n\n');

checks = { ...
    @check_01_augmented_model, ...
    @check_02_kf_disturbance_estimation ...
};

for i = 1:numel(checks)
    fn = checks{i};
    fprintf('--------------------------------------------\n');
    fprintf('(%d/%d) %s\n', i, numel(checks), func2str(fn));
    fprintf('--------------------------------------------\n');

    R = fn();

    if R.pass
        fprintf('PASS\n');
        for k = 1:numel(R.notes)
            fprintf('  - %s\n', R.notes{k});
        end
    else
        fprintf('FAIL: %s\n', R.error);
    end

    fprintf('\n');
end

fprintf('Done.\n');
end
