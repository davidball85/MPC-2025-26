function R = check_04_offset_free_mpc()
% check_04_offset_free_mpc
%
% Purpose
%   Runs the full Task 3 simulation (offset-free MPC) and exports report-ready plots.
%   Adds a *baseline* comparison against the standard MPC (Task 2 controller) under
%   the same surge disturbance step.
%
% What this adds (extra info for write-up)
%   - Standard vs Offset-Free speed tracking after a constant disturbance
%   - Steady-state speed error metrics for both controllers
%   - Control effort (surge thrust) comparison
%
% Outputs
%   R.pass, R.notes, R.error

R = struct('pass', false, 'notes', {{}}, 'error', '');

try
    Ccfg = config_constants();
    Tcfg = config_tuning();

    % ----------------------------
    % (A) Offset-free MPC run (Task 3 main)
    % ----------------------------
    logs_of = task3_main();  % produces task3_main_results.png as before

    u_ref  = Tcfg.x_ref_task3(Ccfg.idx.u);
    u_of_final = logs_of.x(Ccfg.idx.u, end);
    ss_err_of  = abs(u_of_final - u_ref);

    R.notes{end+1} = sprintf('Offset-free MPC final speed: %.4f m/s (ref %.2f), |ss err| = %.4g', ...
        u_of_final, u_ref, ss_err_of);

    % ----------------------------
    % (B) Standard MPC baseline under same disturbance
    % ----------------------------
    logs_std = local_run_standard_mpc_with_disturbance(Ccfg, Tcfg);

    u_std_final = logs_std.x(Ccfg.idx.u, end);
    ss_err_std  = abs(u_std_final - u_ref);

    R.notes{end+1} = sprintf('Standard MPC final speed:    %.4f m/s (ref %.2f), |ss err| = %.4g', ...
        u_std_final, u_ref, ss_err_std);

    % ----------------------------
    % (C) Metrics after the disturbance step
    % ----------------------------
    k_step = round(Ccfg.dist.surge_step_time_s / Ccfg.Ts) + 1;

    % Use last 10 seconds for "steady-state"
    K_of  = size(logs_of.x,2) - 1;
    K_std = size(logs_std.x,2) - 1;
    n_tail_of  = min(round(10 / Ccfg.Ts), K_of);
    n_tail_std = min(round(10 / Ccfg.Ts), K_std);

    u_of_tail  = logs_of.x(Ccfg.idx.u, end-n_tail_of:end);
    u_std_tail = logs_std.x(Ccfg.idx.u, end-n_tail_std:end);

    ss_mean_of  = mean(u_of_tail);
    ss_mean_std = mean(u_std_tail);

    R.notes{end+1} = sprintf('Steady-state mean speed (last 10s): offset-free=%.4f, standard=%.4f (ref %.2f).', ...
        ss_mean_of, ss_mean_std, u_ref);

    % Basic settling time after the step (to within +/-2%% of ref)
    band = 0.02 * abs(u_ref);
    Ts_of  = local_settling_time(logs_of.t, logs_of.x(Ccfg.idx.u,:), k_step, u_ref, band);
    Ts_std = local_settling_time(logs_std.t, logs_std.x(Ccfg.idx.u,:), k_step, u_ref, band);

    if isfinite(Ts_of)
        R.notes{end+1} = sprintf('Settling time (±2%% band, after step): offset-free ~ %.2f s.', Ts_of);
    else
        R.notes{end+1} = 'Settling time (±2% band, after step): offset-free did not settle within sim horizon.';
    end

    if isfinite(Ts_std)
        R.notes{end+1} = sprintf('Settling time (±2%% band, after step): standard    ~ %.2f s.', Ts_std);
    else
        R.notes{end+1} = 'Settling time (±2% band, after step): standard did not settle within sim horizon.';
    end

    R.notes{end+1} = sprintf('Disturbance step: %.1f N at t=%.1f s (from config).', ...
        Ccfg.dist.surge_step_N, Ccfg.dist.surge_step_time_s);

    % ----------------------------
    % (D) Comparison plot: Standard vs Offset-Free
    % ----------------------------
    fig = figure(340); clf(fig);
    fig.Name = 'Task3 Check 04: Standard vs Offset-Free MPC comparison';
    fig.NumberTitle = 'off';

    tiledlayout(fig, 3, 1);

    % Speed
    nexttile;
    plot(logs_of.t, logs_of.x(Ccfg.idx.u,:), logs_std.t, logs_std.x(Ccfg.idx.u,:));
    grid on;
    ylabel('u (m/s)');
    title('Speed tracking under constant surge disturbance');
    legend('offset-free MPC','standard MPC','Location','best');

    % Surge thrust
    nexttile;
    plot(logs_of.t(1:end-1), logs_of.u(1,:), logs_std.t(1:end-1), logs_std.u(1,:));
    grid on;
    ylabel('T_{surge} (N)');
    title('Surge thrust input (channel 1)');
    legend('offset-free MPC','standard MPC','Location','best');

    % Disturbance estimate + step marker
    nexttile;

    d_true_of = local_build_disturbance_profile(Ccfg, size(logs_of.u,2));
    plot(logs_of.t(1:end-1), d_true_of, logs_of.t(1:end-1), -logs_of.dhat);

    grid on;
    xlabel('Time (s)'); ylabel('d (N)');
    title('Surge disturbance: true vs estimated (offset-free path)');
    legend('true','estimated (sign-corrected)','Location','best');


    out_png = fullfile(fileparts(mfilename('fullpath')), 'task3_check04_standard_vs_offsetfree.png');
    saveas(fig, out_png);
    R.notes{end+1} = ['Saved plot: ', out_png];

    % ----------------------------
    % (E) Pass / fail condition (Task 3 requirement)
    % ----------------------------
    if ss_err_of < 0.02
        R.notes{end+1} = 'PASS criterion: offset-free speed steady-state error < 0.02 m/s.';
        R.pass = true;
    else
        R.notes{end+1} = 'FAIL criterion: offset-free speed steady-state error not small enough.';
        R.pass = false;
    end

catch ME
    R.pass  = false;
    R.error = ME.message;
end

end

% =====================================================================
function logs = local_run_standard_mpc_with_disturbance(C, T)
% Runs the standard MPC controller (Task 2 style) on the nonlinear plant,
% under the same surge disturbance profile used in Task 3.
%
% Notes:
% - Uses full state feedback x(k) as the "measured state" (same assumption as Task 2).
% - Uses the same reference vector as Task 3 (T.x_ref_task3).
% - Disturbance is applied through C.d_surge each time step (no hard-coding).

% Prevent stale YALMIP optimiser state causing odd behaviour
yalmip('clear');

% Keep solver quiet in running checks
if isfield(T,'verbose')
    T.verbose = 0;
end

% Build linear model
[Ac, Bc, Cmeas, Dc] = auv_linearise(C);
[Ad, Bd, ~, ~]      = auv_discretise(Ac, Bc, Cmeas, Dc, C.Ts);

% References (Task 3)
x_ref = T.x_ref_task3(:);
u_ref = auv_equilibrium(x_ref, C);

% Build MPC optimiser
M = mpc_standard_build(Ad, Bd, x_ref, u_ref, C, T);

% Simulation setup
Tend = 80;
K    = round(Tend / C.Ts);

t = (0:K)' * C.Ts;

x = zeros(6,1);
x(C.idx.zp) = C.zp_min;

% Disturbance profile
d_true = local_build_disturbance_profile(C, K);

% Logs
logs.t = t;
logs.x = zeros(6, K+1); logs.x(:,1) = x;
logs.u = zeros(3, K);
logs.du = zeros(3, K);
logs.err = zeros(1, K);

for k = 1:K
    % Apply disturbance
    C.d_surge = d_true(k);

    % Solve MPC (silently)
    evalc('[u_cmd, du_cmd, err] = mpc_standard_solve(M, x, x_ref, u_ref);');

    % Saturate as safety net (constraints should already enforce)
    u_cmd = min(max(u_cmd, C.u_min), C.u_max);

    % Nonlinear step
    x = auv_step_nonlinear(x, u_cmd, C);

    % Log
    logs.u(:,k) = u_cmd;
    logs.du(:,k) = du_cmd;
    logs.err(k) = err;
    logs.x(:,k+1) = x;
end

end

% =====================================================================
function d_true = local_build_disturbance_profile(C, K)
% Returns K-by-1 disturbance profile (surge channel).

d_true = C.dist.surge_bias_N * ones(K,1);

if C.dist.enable
    k_step = round(C.dist.surge_step_time_s / C.Ts) + 1;
    k_step = max(1, min(K, k_step));
    d_true(k_step:end) = C.dist.surge_step_N;
end

end

% =====================================================================
function Ts_settle = local_settling_time(t, u_traj, k_step, u_ref, band)
% Settling time after k_step: first time index where u stays within band.

Ts_settle = inf;

if k_step < 1 || k_step > numel(u_traj)
    return
end

idx = k_step:numel(u_traj);

within = abs(u_traj(idx) - u_ref) <= band;

% Find first index after step such that all remaining samples are within band
for k = 1:numel(within)
    if all(within(k:end))
        Ts_settle = t(idx(k)) - t(k_step);
        return
    end
end

end
