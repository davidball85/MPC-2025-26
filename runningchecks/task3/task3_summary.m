function out = task3_summary()
% task3_summary
%
% Runs Task 3 checks AND generates a Task 3 logs struct for animation.
%
% Key behaviour:
%   - Finds project root from THIS file's location (not from pwd).
%   - Runs check_*.m in the same folder as this script (or in runningchecks/task3).
%   - Generates logs using a noise-free measurement path by default,
%     matching the typical runningchecks assumptions.
%
% Output:
%   out.results, out.allPass, out.timestamp, out.projectRoot, out.logs

clc;

out = struct();
out.results = [];
out.allPass = false;
out.timestamp = datestr(now, 'yyyy-mm-dd HH:MM:SS');
out.projectRoot = '';
out.logs = [];

% =========================
% Toggles
% =========================
OPT.runChecks       = true;
OPT.generateLogs    = true;
OPT.assignLogsBase  = true;

% IMPORTANT: default OFF to match your earlier "good" plots
OPT.addMeasNoise    = false;

% Starting depth for logs generation
OPT.startDepth_m    = 0;

% =========================
% Determine folders robustly
% =========================
thisFile = mfilename('fullpath');
thisDir  = fileparts(thisFile);

% If task3_summary.m is stored inside runningchecks/task3:
% project root is two levels up from thisDir.
% If it's stored at project root, thisDir IS project root.
projCand1 = thisDir;
projCand2 = fileparts(fileparts(thisDir));

% Prefer the candidate that actually contains runningchecks/task3
if exist(fullfile(projCand1, 'runningchecks', 'task3'), 'dir') == 7
    projRoot = projCand1;
elseif exist(fullfile(projCand2, 'runningchecks', 'task3'), 'dir') == 7
    projRoot = projCand2;
else
    % fallback: use pwd
    projRoot = pwd;
end

out.projectRoot = projRoot;

% Checks folder:
% If this file lives alongside check_*.m, use thisDir.
% Otherwise use <projRoot>/runningchecks/task3.
if ~isempty(dir(fullfile(thisDir,'check_*.m')))
    checksFolder = thisDir;
else
    checksFolder = fullfile(projRoot, 'runningchecks', 'task3');
end

fprintf('============================================\n');
fprintf('RUNNING TASK 3 CHECKS (%s)\n', out.timestamp);
fprintf('Project: MPC 2025-26\n');
fprintf('Project root: %s\n', out.projectRoot);
fprintf('Checks folder: %s\n', checksFolder);
fprintf('============================================\n\n');

% Run from project root so relative save paths match old behaviour
origPwd = pwd;
cleanupObj = onCleanup(@() cd(origPwd));
if exist(projRoot,'dir') == 7
    cd(projRoot);
end

% =========================
% (A) Run checks
% =========================
if OPT.runChecks && exist(checksFolder,'dir') == 7
    addpath(checksFolder);

    checkFiles = dir(fullfile(checksFolder, 'check_*.m'));
    if isempty(checkFiles)
        warning('No check_*.m files found in %s', checksFolder);
    else
        [~,ix] = sort({checkFiles.name});
        checkFiles = checkFiles(ix);

        results = repmat(struct('name','','pass',false,'notes',{{}},'error',''), 1, numel(checkFiles));

        for i = 1:numel(checkFiles)
            fname = checkFiles(i).name;
            [~, fcn] = fileparts(fname);

            fprintf('--------------------------------------------\n');
            fprintf('(%d/%d) %s\n', i, numel(checkFiles), fname);
            fprintf('--------------------------------------------\n');

            try
                R = feval(fcn);

                results(i).name = fname;
                results(i).pass = isfield(R,'pass') && logical(R.pass);
                if isfield(R,'notes'), results(i).notes = R.notes; end
                if isfield(R,'error'), results(i).error = R.error; end

                if results(i).pass
                    fprintf('Result: PASS\n');
                else
                    fprintf('Result: FAIL\n');
                    if ~isempty(results(i).error)
                        fprintf('  Error: %s\n', results(i).error);
                    end
                end

                if ~isempty(results(i).notes)
                    for k = 1:numel(results(i).notes)
                        fprintf('  - %s\n', results(i).notes{k});
                    end
                end

            catch ME
                results(i).name  = fname;
                results(i).pass  = false;
                results(i).error = ME.message;

                fprintf('Result: FAIL\n');
                fprintf('  Error: %s\n', ME.message);
                fprintf('  Suggested next step:\n');
                fprintf('    - Open %s and fix the reported issue.\n', fname);
                fprintf('    - Re-run task3_summary.\n');
            end

            fprintf('\n');
        end

        out.results = results;
        out.allPass = all([results.pass]);

        if out.allPass
            fprintf('============================================\n');
            fprintf('TASK 3 CHECK SUMMARY: ALL PASS\n');
            fprintf('============================================\n\n');
        else
            fprintf('============================================\n');
            fprintf('TASK 3 CHECK SUMMARY: FAILURES PRESENT\n');
            fprintf('============================================\n\n');
        end
    end
end

% =========================
% (B) Generate logs for animation
% =========================
if OPT.generateLogs
    logs = local_run_task3_sim(OPT.startDepth_m, OPT.addMeasNoise);
    out.logs = logs;

    if OPT.assignLogsBase
        assignin('base','logs',logs);
        fprintf('Assigned logs to base workspace as variable: logs\n');
    end
end

% =========================================================
% (C) Extra diagnostic plot: thrust effort vs disturbance
% =========================================================
try
    C = config_constants();

    task3_plot_effort_vs_disturbance(out.logs, C, ...
        'UsePhysicalDhat', true, ...
        'ExportPNG', true);

catch ME
    warning('Could not generate effort vs disturbance plot: %s', ME.message);
end


end

% =====================================================================
function logs = local_run_task3_sim(startDepth_m, addMeasNoise)

C = config_constants();
T = config_tuning();

% Linear + discretise
[Ac, Bc, Cmeas, Dc] = auv_linearise(C);
[Ad, Bd, ~, ~]      = auv_discretise(Ac, Bc, Cmeas, Dc, C.Ts);

% Augmented model
[A_aug, B_aug, C_aug, Bw] = auv_build_augmented_model(Ad, Bd, Cmeas);

model.A  = Ad;
model.B  = Bd;
model.Bw = Bw;
model.Cy = Cmeas;

% KF init (must exist in your project)
est = kalman_augmented_init(A_aug, B_aug, C_aug, T);

x_ref = T.x_ref_task3(:);
u_eq  = auv_equilibrium(x_ref, C);

Tend = 80;
K    = round(Tend / C.Ts);
t    = (0:K)' * C.Ts;

% initial state
x = zeros(6,1);
x(C.idx.zp) = startDepth_m;

% true disturbance profile
d_true = C.dist.surge_bias_N * ones(K,1);
if isfield(C,'dist') && isfield(C.dist,'enable') && C.dist.enable
    k_step = round(C.dist.surge_step_time_s / C.Ts) + 1;
    k_step = max(1, min(K, k_step));
    d_true(k_step:end) = C.dist.surge_step_N;
end

% logs
logs.t = t;
logs.x = zeros(6,K+1); logs.x(:,1) = x;
logs.u = zeros(3,K);
logs.y = zeros(2,K);
logs.xhat = zeros(6,K);
logs.dhat = zeros(1,K);
logs.uss = zeros(3,K);
logs.d_true = d_true;

for k = 1:K
    y_true = Cmeas*x;

    % DEFAULT: no noise, matches typical runningchecks assumptions
    y = y_true;

    if addMeasNoise
        if isfield(T,'kf') && isfield(T.kf,'R') && all(size(T.kf.R)==[2 2])
            v = [sqrt(T.kf.R(1,1))*randn; sqrt(T.kf.R(2,2))*randn];
            y = y_true + v;
        end
    end

    if k == 1
        u_prev = u_eq;
    else
        u_prev = logs.u(:,k-1);
    end

    est = kalman_augmented_step(est, u_prev, y);

    x_hat = est.xhat(1:6);
    d_hat = est.xhat(end);

    OUT = mpc_offsetfree_wrapper(x_hat, d_hat, x_ref, model, C, T);
    u_cmd = OUT.u_cmd;

    C.d_surge = d_true(k);
    x = auv_step_nonlinear(x, u_cmd, C);

    logs.u(:,k) = u_cmd;
    logs.y(:,k) = y;
    logs.x(:,k+1) = x;
    logs.xhat(:,k) = x_hat;
    logs.dhat(k) = d_hat;
    logs.uss(:,k) = OUT.u_ss;
end

% For animation: provide a "physically interpretable" version too
logs.dhat_phys = logs.dhat;
tailN = min(60, numel(logs.dhat));
dhat_tail = mean(logs.dhat(end-tailN+1:end));
stepSign = sign(C.dist.surge_step_N);
if stepSign == 0, stepSign = 1; end
if sign(dhat_tail) ~= stepSign
    logs.dhat_phys = -logs.dhat_phys;
end

end
