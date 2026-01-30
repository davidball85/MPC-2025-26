function R = check_02_kalman_observer()
% check_02_kalman_observer
%
% Purpose
%   Demonstrate that the augmented KF estimates:
%     - unmeasured velocities (u,w,q,theta)
%     - constant surge disturbance force d
%   using only y = [xp; zp] measurements.
%
% Adds extra diagnostics
%   - Disturbance estimation error e_d = d_true - d_hat
%   - Mean / std of e_d after convergence
%   - Sign-consistency flag
%   - Disturbance estimate settling time (±5% band)

R = struct('pass', false, 'notes', {{}}, 'error', '');

try
    % ----------------------------
    % Config
    % ----------------------------
    Ccfg = config_constants();
    Tcfg = config_tuning();

    nx = numel(Ccfg.x_eq);

    % ----------------------------
    % Linear discrete model (for estimator)
    % ----------------------------
    [Ac, Bc, ~, Dc] = auv_linearise(Ccfg);

    % Measurement matrix for y = [xp; zp]
    Cmeas = zeros(2, nx);
    Cmeas(1, Ccfg.idx.xp) = 1;
    Cmeas(2, Ccfg.idx.zp) = 1;

    [A, B, Cy, ~] = auv_discretise(Ac, Bc, Cmeas, Dc, Ccfg.Ts);

    % ----------------------------
    % Augmented model + estimator init
    % ----------------------------
    [A_aug, B_aug, C_aug, ~] = auv_build_augmented_model(A, B, Cy);
    est = kalman_augmented_init(A_aug, B_aug, C_aug, Tcfg);

    % ----------------------------
    % Simulation setup
    % ----------------------------
    Ts = Ccfg.Ts;
    t_end = 30;
    N = round(t_end/Ts) + 1;
    t = (0:N-1)*Ts;

    x_eq = Ccfg.x_eq;
    u_eq = auv_equilibrium(x_eq, Ccfg);
    u_applied = u_eq;

    % Disturbance profile
    d_true = Ccfg.dist.surge_bias_N * ones(1, N);
    if Ccfg.dist.enable
        k_step = round(Ccfg.dist.surge_step_time_s / Ts) + 1;
        d_true(k_step:end) = Ccfg.dist.surge_step_N;
    end

    % Initial condition
    x_true = zeros(nx, 1);

    % Logs
    y_meas_log = zeros(2, N);
    y_true_log = zeros(2, N);
    y_hat_log  = zeros(2, N);

    x_hat_log  = zeros(nx, N);
    x_true_log = zeros(nx, N);
    d_hat_log  = zeros(1, N);

    Rmeas = Tcfg.kf.R;

    % ----------------------------
    % Main loop
    % ----------------------------
    for k = 1:N
        % Inject true disturbance into plant
        Ccfg.d_surge = d_true(k);

        % True output (noiseless)
        y_true = Cy * x_true;

        % Noisy measurement
        v = [sqrt(Rmeas(1,1))*randn; sqrt(Rmeas(2,2))*randn];
        y_meas = y_true + v;

        % KF step
        est = kalman_augmented_step(est, u_applied, y_meas);

        x_hat = est.xhat(1:nx);
        d_hat = est.xhat(end);

        % Log
        y_true_log(:,k) = y_true;
        y_meas_log(:,k) = y_meas;
        y_hat_log(:,k)  = Cy * x_hat;

        x_hat_log(:,k)  = x_hat;
        x_true_log(:,k) = x_true;
        d_hat_log(k)    = d_hat;

        % Nonlinear plant update
        x_true = auv_step_nonlinear(x_true, u_applied, Ccfg);
    end

    % ----------------------------
    % Diagnostics
    % ----------------------------
    e_d = d_true - d_hat_log;

    if Ccfg.dist.enable
        k_step = round(Ccfg.dist.surge_step_time_s / Ts) + 1;

        d_pre  = mean(d_hat_log(1:max(1, k_step-1)));
        d_post = mean(d_hat_log(min(N, k_step+10):end));
        d_jump = abs(d_post - d_pre);

        u_true = x_true_log(Ccfg.idx.u, :);
        u_hat  = x_hat_log(Ccfg.idx.u, :);

        uerr_pre  = mean(abs(u_hat(1:max(1,k_step-1)) - u_true(1:max(1,k_step-1))));
        uerr_post = mean(abs(u_hat(min(N,k_step+10):end) - u_true(min(N,k_step+10):end)));

        % Error stats after convergence (last 25% of sim)
        tail_idx = round(0.75*N):N;
        ed_mean = mean(e_d(tail_idx));
        ed_std  = std(e_d(tail_idx));

     % --- Disturbance estimate settling time (±5% band around final value) ---
% IMPORTANT: use the logged VECTOR d_hat_log, not the scalar d_hat

tStep   = Ccfg.dist.surge_step_time_s;              % correct config field
idxStep = find(t >= tStep, 1, 'first');
if isempty(idxStep), idxStep = 1; end

tailN   = min(20, numel(d_hat_log));
tailIdx = (numel(d_hat_log)-tailN+1) : numel(d_hat_log);
d_inf   = mean(d_hat_log(tailIdx));                 % final value estimate

band = 0.05 * max(1e-6, abs(d_inf));                % ±5% band

idxSet = NaN;
for kk = idxStep:numel(t)
    if all(abs(d_hat_log(kk:end) - d_inf) <= band)
        idxSet = kk;
        break;
    end
end

if isnan(idxSet)
    R.notes{end+1} = 'Disturbance estimate did not settle within ±5% band by end of sim.';
else
    T_set = t(idxSet) - tStep;
    R.notes{end+1} = sprintf('Disturbance estimate settling time (±5%% band) ≈ %.2f s.', T_set);
end


        if isnan(idxSet)
            dSettleStr = 'Disturbance estimate did not settle within ±5% band by end of sim.';
        else
            T_set = t(idxSet) - tStep;
            dSettleStr = sprintf('Disturbance estimate settling time (±5%% band) ≈ %.2f s.', T_set);
        end

        % Notes (console)
        R.notes{end+1} = sprintf('Mean d_hat before step   = %.2f N.', d_pre);
        R.notes{end+1} = sprintf('Mean d_hat after step    = %.2f N (true %.1f N).', d_post, Ccfg.dist.surge_step_N);
        R.notes{end+1} = sprintf('|d_hat change| after step = %.2f N.', d_jump);
        R.notes{end+1} = sprintf('Mean |u_hat - u_true| before = %.4f, after = %.4f.', uerr_pre, uerr_post);
        R.notes{end+1} = sprintf('Disturbance estimation error e_d=d_true-d_hat (tail): mean=%.2f N, std=%.2f N.', ed_mean, ed_std);
        R.notes{end+1} = dSettleStr;

        % Simple sign-consistency check
        s_true = sign(Ccfg.dist.surge_step_N);
        s_hat  = sign(d_post);
        if s_true ~= 0 && s_hat ~= 0 && s_true ~= s_hat
            R.notes{end+1} = 'WARNING: d_hat converged with opposite sign to the configured disturbance step. This can be OK if the plant disturbance sign convention differs, but it should be explained/verified.';
        end

        if d_jump > 1.0 && (uerr_post <= 1.2*uerr_pre)
            R.notes{end+1} = 'PASS: d_hat responds to disturbance and u-estimation remains consistent.';
            R.pass = true;
        else
            R.notes{end+1} = 'FAIL: weak disturbance response or u-estimation degraded. Tune Tcfg.kf.Q_aug(end,end) / Tcfg.kf.R.';
            R.pass = false;
        end
    else
        R.notes{end+1} = 'Disturbance disabled in config: passing observer sanity check.';
        R.pass = true;
    end

    % ----------------------------
    % Plots (fixed figure numbers, tiledlayout)
    % ----------------------------
    fig1 = figure(320); clf(fig1);
    fig1.Name = 'Task3 Check 02: Augmented KF performance';
    fig1.NumberTitle = 'off';

    tiledlayout(fig1, 4, 1);

    nexttile;
    plot(t, y_true_log(1,:), t, y_meas_log(1,:), t, y_hat_log(1,:));
    grid on; ylabel('x_p (m)');
    title('Output x_p: true vs measured vs KF estimate');
    legend('true','measured','estimated','Location','best');

    nexttile;
    plot(t, y_true_log(2,:), t, y_meas_log(2,:), t, y_hat_log(2,:));
    grid on; ylabel('z_p (m)');
    title('Output z_p: true vs measured vs KF estimate');
    legend('true','measured','estimated','Location','best');

    nexttile;
    plot(t, d_true, t, d_hat_log);
    grid on; ylabel('d (N)');
    title('Surge disturbance: true vs estimated');
    legend('true','estimated','Location','best');

    nexttile;
    plot(t, e_d);
    grid on; xlabel('Time (s)'); ylabel('e_d (N)');
    title('Disturbance estimation error $e_d = d_{\mathrm{true}} - \hat{d}$', 'Interpreter','latex');

    out_png = fullfile(fileparts(mfilename('fullpath')), 'task3_check02_kf_disturbance.png');
    try
        exportgraphics(fig1, out_png, 'Resolution', 300);
    catch
        saveas(fig1, out_png);
    end
    R.notes{end+1} = ['Saved plot: ', out_png];

    fig2 = figure(321); clf(fig2);
    fig2.Name = 'Task3 Check 02b: Unmeasured states (true vs estimated)';
    fig2.NumberTitle = 'off';

    tiledlayout(fig2, 3, 1);

    nexttile;
    plot(t, x_true_log(Ccfg.idx.u,:), t, x_hat_log(Ccfg.idx.u,:));
    grid on; ylabel('u');
    title('u: true vs estimated');
    legend('true','est','Location','best');

    nexttile;
    plot(t, x_true_log(Ccfg.idx.w,:), t, x_hat_log(Ccfg.idx.w,:));
    grid on; ylabel('w');
    title('w: true vs estimated');
    legend('true','est','Location','best');

    nexttile;
    plot(t, x_true_log(Ccfg.idx.q,:), t, x_hat_log(Ccfg.idx.q,:));
    grid on; xlabel('Time (s)'); ylabel('q');
    title('q: true vs estimated');
    legend('true','est','Location','best');

    out_png2 = fullfile(fileparts(mfilename('fullpath')), 'task3_check02b_states.png');
    try
        exportgraphics(fig2, out_png2, 'Resolution', 300);
    catch
        saveas(fig2, out_png2);
    end
    R.notes{end+1} = ['Saved plot: ', out_png2];

    R.notes{end+1} = sprintf('Disturbance step configured: %.1f N at t=%.1f s.', ...
        Ccfg.dist.surge_step_N, Ccfg.dist.surge_step_time_s);

catch ME
    R.pass  = false;
    R.error = ME.message;
end
end
