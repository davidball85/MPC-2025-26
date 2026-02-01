function logs = task3_main()
% TASK3_MAIN - Main simulation for offset-free MPC (Task 3)
%
% Demonstrates offset-free MPC with augmented KF disturbance estimation.
% Optional: stop-at-wall behaviour (ramps speed reference to 0 at wall).
%
% OUTPUT: logs struct with fields:
%   .t, .x, .u, .xhat, .dhat, .x_ss, .u_ss, .y, .d_true

clc;

fprintf('\n');
fprintf('╔════════════════════════════════════════════════════════════╗\n');
fprintf('║                                                            ║\n');
fprintf('║            TASK 3: OFFSET-FREE MPC SIMULATION              ║\n');
fprintf('║                                                            ║\n');
fprintf('╚════════════════════════════════════════════════════════════╝\n');
fprintf('\n');

%% ====================================================================
%% STEP 1: LOAD CONFIGURATION
%% ====================================================================
fprintf('════════════════════════════════════════════════════════════\n');
fprintf('  STEP 1: CONFIGURATION LOADING\n');
fprintf('════════════════════════════════════════════════════════════\n');

Ccfg = config_constants();
Tcfg = config_tuning();

fprintf('[1.1] Physical Parameters:\n');
fprintf('      mx = %.1f kg, mz = %.1f kg, Iy = %.1f kg·m²\n', ...
    Ccfg.mx, Ccfg.mz, Ccfg.Iy);
fprintf('      Sampling time Ts = %.3f s\n', Ccfg.Ts);

fprintf('\n[1.2] Reference Setpoint:\n');
fprintf('      u_ref  = %.2f m/s  (surge velocity)\n', Tcfg.x_ref_task3(Ccfg.idx.u));
fprintf('      zp_ref = %.2f m    (depth)\n', Tcfg.x_ref_task3(Ccfg.idx.zp));

fprintf('\n[1.3] Disturbance Configuration:\n');
fprintf('      Enabled: %s\n', mat2str(Ccfg.dist.enable));
fprintf('      Step time:      t = %.1f s\n', Ccfg.dist.surge_step_time_s);
fprintf('      Step magnitude: d = %.1f N (resistive force)\n', Ccfg.dist.surge_step_N);
fprintf('      Initial bias:   d = %.1f N\n', Ccfg.dist.surge_bias_N);

fprintf('\n[1.4] MPC Settings:\n');
fprintf('      Prediction horizon: N = %d steps (%.1f s)\n', ...
    Tcfg.N, Tcfg.N * Ccfg.Ts);
fprintf('      State weights (diag(Q)): [%.0f, %.0f, %.0f, %.0f, %.0f, %.0f]\n', ...
    diag(Tcfg.Q)');
fprintf('      Input weights (diag(R)): [%.3f, %.3f, %.3f]\n', diag(Tcfg.R)');

%% ====================================================================
%% STEP 2: BUILD LINEAR MODEL
%% ====================================================================
fprintf('\n════════════════════════════════════════════════════════════\n');
fprintf('  STEP 2: LINEAR MODEL CONSTRUCTION\n');
fprintf('════════════════════════════════════════════════════════════\n');

fprintf('[2.1] Continuous-time linearisation...\n');
[Ac, Bc, ~, Dc] = auv_linearise(Ccfg);
fprintf('      ✓ Linearised around x_eq = [%.2f; %.2f; %.2f; %.2f; %.2f; %.2f]\n', ...
    Ccfg.x_eq');

fprintf('\n[2.2] Discretisation (Zero-Order Hold)...\n');
Cmeas = zeros(2, 6);
Cmeas(1, Ccfg.idx.xp) = 1;  % Measure xp
Cmeas(2, Ccfg.idx.zp) = 1;  % Measure zp
[A, B, Cy, ~] = auv_discretise(Ac, Bc, Cmeas, Dc, Ccfg.Ts);
fprintf('      ✓ Discrete-time model created\n');
fprintf('      ✓ Measurements: y = [xp; zp]\n');

nx = size(A, 1);
nu = size(B, 2);
ny = size(Cy, 1);

fprintf('\n[2.3] Model Dimensions:\n');
fprintf('      States:       nx = %d\n', nx);
fprintf('      Inputs:       nu = %d\n', nu);
fprintf('      Measurements: ny = %d\n', ny);

%% ====================================================================
%% STEP 3: BUILD AUGMENTED MODEL
%% ====================================================================
fprintf('\n════════════════════════════════════════════════════════════\n');
fprintf('  STEP 3: AUGMENTED MODEL FOR DISTURBANCE ESTIMATION\n');
fprintf('════════════════════════════════════════════════════════════\n');

[A_aug, B_aug, C_aug, Bw] = auv_build_augmented_model(A, B, Cy);

nx_aug = size(A_aug, 1);

fprintf('[3.1] Post-Construction Summary:\n');
fprintf('      Augmented states: nx_aug = %d (nx + 1 disturbance)\n', nx_aug);

% Infer Cd from C_aug if present (C_aug = [Cy, Cd])
Cd = [];
if size(C_aug,2) == (size(Cy,2) + 1)
    Cd = C_aug(:, end);
end

if ~isempty(Cd)
    fprintf('      Cd detected: [%.3f; %.3f]\n', Cd(1), Cd(2));
else
    fprintf('      Cd not detected (no extra column in C_aug)\n');
end

%% ====================================================================
%% STEP 4: INITIALISE KALMAN FILTER
%% ====================================================================
fprintf('\n════════════════════════════════════════════════════════════\n');
fprintf('  STEP 4: KALMAN FILTER INITIALISATION\n');
fprintf('════════════════════════════════════════════════════════════\n');

Maug = struct('A_aug', A_aug, 'B_aug', B_aug, 'C_aug', C_aug);
est = kalman_augmented_init(Maug, Ccfg, Tcfg);

fprintf('[4.1] Initial Estimate:\n');
fprintf('      xhat(0) = [%.2f; %.2f; %.2f; %.2f; %.2f; %.2f]\n', est.xhat(1:6)');
fprintf('      dhat(0) = %.2f N\n', est.xhat(end));

%% ====================================================================
%% STEP 5: SIMULATION SETUP
%% ====================================================================
fprintf('\n════════════════════════════════════════════════════════════\n');
fprintf('  STEP 5: SIMULATION INITIALISATION\n');
fprintf('════════════════════════════════════════════════════════════\n');

Tend = 80;                      % seconds
K = round(Tend / Ccfg.Ts);
t = (0:K)' * Ccfg.Ts;

fprintf('[5.1] Time Parameters:\n');
fprintf('      Simulation duration: %.1f s\n', Tend);
fprintf('      Number of steps:     K = %d\n', K);

% Initial state: single source of truth
x0 = Ccfg.Task3.initial_state;

fprintf('\n[5.2] Initial Plant State:\n');
fprintf('      x(0) = [u=%.2f, w=%.2f, q=%.2f, xp=%.2f, zp=%.2f, θ=%.2f]ᵀ\n', ...
    x0(Ccfg.idx.u), x0(Ccfg.idx.w), x0(Ccfg.idx.q), ...
    x0(Ccfg.idx.xp), x0(Ccfg.idx.zp), x0(Ccfg.idx.theta));

% Disturbance profile (length K, applied during each step k)
d_true = Ccfg.dist.surge_bias_N * ones(1, K);
if Ccfg.dist.enable
    k_step = round(Ccfg.dist.surge_step_time_s / Ccfg.Ts) + 1;
    k_step = max(1, min(K, k_step));
    d_true(k_step:end) = Ccfg.dist.surge_step_N;
else
    k_step = inf;
end

fprintf('\n[5.3] Disturbance Profile:\n');
fprintf('      d(t<%.1fs)  = %.1f N\n', Ccfg.dist.surge_step_time_s, Ccfg.dist.surge_bias_N);
fprintf('      d(t>=%.1fs) = %.1f N  (step at k=%d)\n', ...
    Ccfg.dist.surge_step_time_s, Ccfg.dist.surge_step_N, k_step);

% Allocate logs
x_log    = zeros(6, K+1);  x_log(:,1) = x0;
u_log    = zeros(3, K);
xhat_log = zeros(6, K);
dhat_log = zeros(1, K);
x_ss_log = zeros(6, K);
u_ss_log = zeros(3, K);
y_log    = zeros(2, K);

% Model package for MPC wrapper
model = struct('A', A, 'B', B, 'Bw', Bw, 'Cy', Cy);

% Reference setpoint (track u=1 m/s, zp=5 m)
x_ref = Tcfg.x_ref_task3;

% First-step equilibrium input
u_eq = auv_equilibrium(x_ref, Ccfg);

fprintf('\n[5.4] Equilibrium Input (no disturbance):\n');
fprintf('      u_eq = [Tsurge=%.2f, Theave=%.2f, τ=%.2f]ᵀ N/Nm\n', ...
    u_eq(1), u_eq(2), u_eq(3));

%% ====================================================================
%% Stop-at-wall (reference shaping) state
%% ====================================================================
u_ref_nom = x_ref(Ccfg.idx.u);    % nominal 1 m/s
stop_latched = false;

%% ====================================================================
%% STEP 6: MAIN SIMULATION LOOP
%% ====================================================================
fprintf('\n════════════════════════════════════════════════════════════\n');
fprintf('  STEP 6: RUNNING SIMULATION\n');
fprintf('════════════════════════════════════════════════════════════\n');

tic;

for k = 1:K

    x_curr = x_log(:, k);

    % Measurement: y = Cy*x + Cd*d_true (if Cd exists)
    y = Cy * x_curr;
    if ~isempty(Cd)
        y = y + Cd * d_true(k);
    end
    y_log(:, k) = y;

    % Previous input for KF
    if k == 1
        u_prev = u_eq;
    else
        u_prev = u_log(:, k-1);
    end

    % KF update
    [est, ~] = kalman_augmented_step(est, u_prev, y);

    x_hat = est.xhat(1:6);
    d_hat = est.xhat(end);

    xhat_log(:, k) = x_hat;
    dhat_log(k) = d_hat;

    % ------------------------------------------------------------
    % Offset-Free MPC Control (optional stop-at-wall behaviour)
    % ------------------------------------------------------------
    x_ref_k = x_ref;

    if isfield(Ccfg,'Task3') && isfield(Ccfg.Task3,'stop_at_wall_enable') && Ccfg.Task3.stop_at_wall_enable

        % Use the same wall the constraints enforce
        if isfield(Ccfg,'wall_stop_internal')
            wall_stop = Ccfg.wall_stop_internal;   % e.g. 48
        else
            wall_stop = Ccfg.wall_stop;
        end

        margin = Ccfg.Task3.wall_brake_margin_m;   % braking distance (m)
        margin = max(margin, 0.1);

        xp_now = x_curr(Ccfg.idx.xp);

        % Latch once we reach the wall (discrete-time safety)
        if xp_now >= wall_stop
            stop_latched = true;
        end

        if stop_latched
            u_ref_cmd = 0.0;
        else
            % Ramp u_ref down over the last 'margin' metres:
            %   dist_to_wall = margin -> u_ref = u_ref_nom
            %   dist_to_wall = 0      -> u_ref = 0
            dist_to_wall = wall_stop - xp_now;
            alpha = dist_to_wall / margin;
            alpha = max(0.0, min(1.0, alpha));
            u_ref_cmd = u_ref_nom * alpha;
        end

        % Do not command negative speed (no reverse)
        u_ref_cmd = max(0.0, u_ref_cmd);

        x_ref_k(Ccfg.idx.u) = u_ref_cmd;
    end

    OUT = mpc_offsetfree_wrapper(x_hat, d_hat, x_ref_k, model, Ccfg, Tcfg);

    u_cmd = OUT.u_cmd;
    u_ss  = OUT.u_ss;
    x_ss  = OUT.x_ss;

    u_log(:, k) = u_cmd;
    u_ss_log(:, k) = u_ss;
    x_ss_log(:, k) = x_ss;

    % Plant step (apply disturbance)
    Ccfg.d_surge = d_true(k);
    x_next = auv_step_nonlinear(x_curr, u_cmd, Ccfg);
    x_log(:, k+1) = x_next;

end

elapsed = toc;

fprintf('\n[6.1] Simulation Complete!\n');
fprintf('      Total time:   %.2f s\n', elapsed);
fprintf('      Avg per step: %.2f ms\n', 1000*elapsed/K);

%% ====================================================================
%% STEP 7: POST-SIM ANALYSIS
%% ====================================================================
fprintf('\n════════════════════════════════════════════════════════════\n');
fprintf('  STEP 7: PERFORMANCE ANALYSIS\n');
fprintf('════════════════════════════════════════════════════════════\n');

u_final  = x_log(Ccfg.idx.u, end);
zp_final = x_log(Ccfg.idx.zp, end);
xp_final = x_log(Ccfg.idx.xp, end);

u_ref_val  = Tcfg.x_ref_task3(Ccfg.idx.u);
zp_ref_val = Tcfg.x_ref_task3(Ccfg.idx.zp);

fprintf('[7.1] Final State:\n');
fprintf('      u(T)  = %.4f m/s  (ref: %.2f m/s, error: %.2e m/s)\n', ...
    u_final, u_ref_val, u_final - u_ref_val);
fprintf('      zp(T) = %.4f m    (ref: %.2f m, error: %.2e m)\n', ...
    zp_final, zp_ref_val, zp_final - zp_ref_val);
fprintf('      xp(T) = %.2f m\n', xp_final);

% Tail stats (last 10 seconds)
n_tail = min(round(10 / Ccfg.Ts), K);
u_tail  = x_log(Ccfg.idx.u, end-n_tail:end);
zp_tail = x_log(Ccfg.idx.zp, end-n_tail:end);

fprintf('\n[7.2] Steady-State Performance (last 10s):\n');
fprintf('      u:  mean = %.4f m/s, std = %.2e m/s\n', mean(u_tail), std(u_tail));
fprintf('      zp: mean = %.4f m,   std = %.2e m\n', mean(zp_tail), std(zp_tail));

d_tail = dhat_log(end-n_tail+1:end);
fprintf('\n[7.3] Disturbance Estimation (last 10s):\n');
fprintf('      d_true(T)  = %.2f N\n', d_true(end));
fprintf('      d_hat(T)   = %.2f N\n', dhat_log(end));
fprintf('      d_hat mean = %.2f N, std = %.2e N\n', mean(d_tail), std(d_tail));

%% ====================================================================
%% STEP 8: PLOTS
%% ====================================================================
fprintf('\n════════════════════════════════════════════════════════════\n');
fprintf('  STEP 8: GENERATING PLOTS\n');
fprintf('════════════════════════════════════════════════════════════\n');

fig = figure(350);
clf(fig);
fig.Name = 'Task 3: Offset-Free MPC Results';
fig.NumberTitle = 'off';
fig.Position = [100, 100, 1200, 900];

tiledlayout(fig, 3, 2);

% Plot 1: Speed tracking
nexttile;
plot(t, x_log(Ccfg.idx.u,:), 'b-', 'LineWidth', 2);
hold on;
yline(u_ref_val, 'r--', 'LineWidth', 1.5);
xline(Ccfg.dist.surge_step_time_s, 'g:', 'LineWidth', 1.5);
hold off;
grid on;
ylabel('u (m/s)');
title('Surge Velocity Tracking');
legend('u(t)', 'Reference', 'Disturbance Step', 'Location', 'best');

% Plot 2: Depth tracking
nexttile;
plot(t, x_log(Ccfg.idx.zp,:), 'b-', 'LineWidth', 2);
hold on;
yline(zp_ref_val, 'r--', 'LineWidth', 1.5);
xline(Ccfg.dist.surge_step_time_s, 'g:', 'LineWidth', 1.5);
hold off;
grid on;
ylabel('z_p (m)');
title('Depth Tracking');
legend('z_p(t)', 'Reference', 'Disturbance Step', 'Location', 'best');

% Plot 3: Disturbance estimate vs true
nexttile;
plot(t(1:end-1), dhat_log, 'b-', 'LineWidth', 2);
hold on;
plot(t(1:end-1), d_true, 'r--', 'LineWidth', 1.5);
xline(Ccfg.dist.surge_step_time_s, 'g:', 'LineWidth', 1.5);
hold off;
grid on;
ylabel('Disturbance (N)');
xlabel('Time (s)');
title('Disturbance Estimation');
legend('d_{hat}', 'd_{true}', 'Step Time', 'Location', 'best');

% Plot 4: Control effort
nexttile;
stairs(t(1:end-1), u_log(1,:), 'b-', 'LineWidth', 1.5);
hold on;
stairs(t(1:end-1), u_ss_log(1,:), 'r--', 'LineWidth', 1.5);
xline(Ccfg.dist.surge_step_time_s, 'g:', 'LineWidth', 1.5);
hold off;
grid on;
ylabel('T_{surge} (N)');
xlabel('Time (s)');
title('Surge Thrust');
legend('u_{cmd}', 'u_{ss}', 'Disturbance Step', 'Location', 'best');

% Plot 5: Position
nexttile;
plot(t, x_log(Ccfg.idx.xp,:), 'b-', 'LineWidth', 2);
hold on;
xline(Ccfg.dist.surge_step_time_s, 'g:', 'LineWidth', 1.5);
hold off;
grid on;
ylabel('x_p (m)');
xlabel('Time (s)');
title('Horizontal Position');

% Plot 6: Pitch angle
nexttile;
plot(t, rad2deg(x_log(Ccfg.idx.theta,:)), 'b-', 'LineWidth', 2);
hold on;
yline(rad2deg(Ccfg.theta_min), 'r--', 'LineWidth', 1);
yline(rad2deg(Ccfg.theta_max), 'r--', 'LineWidth', 1);
xline(Ccfg.dist.surge_step_time_s, 'g:', 'LineWidth', 1.5);
hold off;
grid on;
ylabel('\theta (deg)');
xlabel('Time (s)');
title('Pitch Angle');

out_eps = fullfile(fileparts(mfilename('fullpath')), 'task3_main_results.eps');
print(fig, out_eps, '-depsc', '-painters');
fprintf('[8.1] Saved plot: %s\n', out_eps);

%% ====================================================================
%% STEP 9: PACKAGE OUTPUT
%% ====================================================================
logs = struct();
logs.t      = t;
logs.x      = x_log;
logs.u      = u_log;
logs.xhat   = xhat_log;
logs.dhat   = dhat_log;
logs.x_ss   = x_ss_log;
logs.u_ss   = u_ss_log;
logs.y      = y_log;
logs.d_true = d_true;

fprintf('\nDone.\n');


%% ====================================================================
%% EXPORT ALL FIGURES AS EPS
%% ====================================================================
fprintf('\nExporting all figures as EPS...\n');

figHandles = findall(0, 'Type', 'figure');

outDir = fileparts(mfilename('fullpath'));

for k = 1:numel(figHandles)
    fig = figHandles(k);
    figNum = fig.Number;

    fname = sprintf('task3_figure_%d.eps', figNum);
    fpath = fullfile(outDir, fname);

    print(fig, fpath, '-depsc', '-painters');
    fprintf('  ✓ Exported Figure %d → %s\n', figNum, fname);
end

end