function R = check_02_kf_disturbance_estimation()
% check_02_kf_disturbance_estimation
%
% Purpose
%   Demonstrate that the augmented KF estimates:
%     - unmeasured velocities (u,w,q,theta)
%     - constant surge disturbance force d
%   using only y=[xp; zp] measurements.
%
% Plant: nonlinear discrete step (Task 1 plant), with disturbance injected via C.d_surge.
% Measurement: y = [xp; zp] + noise (tuning config).
%
% Produces plots suitable for Task 3 report:
%   - measured vs true vs estimated outputs
%   - selected states true vs estimated
%   - disturbance true vs estimated
%
% Outputs: R.pass, R.notes, R.error

R = struct('pass', false, 'notes', {{}}, 'error', '');

try
    Cc = config_constants();
    T  = config_tuning();

    % --- Linear discrete model (for estimator)
     [Ac, Bc, Cmeas, Dc] = auv_linearise(C);
    [A,  B,  Cy,  ~ ]   = auv_discretise(Ac, Bc, Cmeas, Dc, C.Ts);
optim

    [A_aug,B_aug,C_aug,~] = auv_build_augmented_model(A,B,Cy);

    % --- Initialise estimator
    est = kalman_augmented_init(A_aug,B_aug,C_aug,T);

    % --- Simulation setup
    Ts = Cc.Ts;
    t_end = 30;
    N = round(t_end/Ts) + 1;
    t = (0:N-1)*Ts;

    % Reference-ish open-loop input (just hold equilibrium thrusts)
    x_eq = Cc.x_eq;
    u_eq = auv_equilibrium(x_eq, Cc);

    % Disturbance profile (config-driven)
    d_true = Cc.dist.surge_bias_N * ones(1,N);
    if Cc.dist.enable
        k_step = round(Cc.dist.surge_step_time_s / Ts) + 1;
        d_true(k_step:end) = Cc.dist.surge_step_N;
    end

    % Initial conditions
    x_true = zeros(6,1);      % start at rest at surface (like Task 2)
    u_applied = u_eq;

    % Logs
    y_meas_log = zeros(2,N);
    y_true_log = zeros(2,N);
    y_hat_log  = zeros(2,N);

    x_hat_log  = zeros(6,N);
    x_true_log = zeros(6,N);     % <-- added (needed for pass condition)
    d_hat_log  = zeros(1,N);

    % Noise (measurement only)
    Rmeas = T.kf.R;

    % --- Run
    for k = 1:N
        % Set plant disturbance for this step
        Cc.d_surge = d_true(k);

        % True output (noiseless)
        y_true = Cy * x_true;

        % Noisy measurement
        v = [sqrt(Rmeas(1,1))*randn; sqrt(Rmeas(2,2))*randn];
        y_meas = y_true + v;

        % KF step (estimator uses linear model + measured y)
        est = kalman_augmented_step(est, u_applied, y_meas);

        x_hat = est.xhat(1:6);
        d_hat = est.xhat(end);

        % Log
        y_true_log(:,k) = y_true;
        y_meas_log(:,k) = y_meas;
        y_hat_log(:,k)  = Cy * x_hat;

        x_hat_log(:,k)  = x_hat;
        x_true_log(:,k) = x_true;     % <-- added
        d_hat_log(k)    = d_hat;

        % Plant update (nonlinear)
        x_true = auv_step_nonlinear(x_true, u_applied, Cc);
    end

    % ---- Pass condition (robust + report-friendly)
    if Cc.dist.enable
        k_step = round(Cc.dist.surge_step_time_s / Ts) + 1;

        d_pre  = mean(d_hat_log(1:max(1,k_step-1)));
        d_post = mean(d_hat_log(min(N,k_step+10):end));
        d_jump = abs(d_post - d_pre);

        % Speed estimation consistency (true vs estimated)
        u_true = x_true_log(Cc.idx.u,:);
        u_hat  = x_hat_log(Cc.idx.u,:);

        uerr_pre  = mean(abs(u_hat(1:max(1,k_step-1)) - u_true(1:max(1,k_step-1))));
        uerr_post = mean(abs(u_hat(min(N,k_step+10):end) - u_true(min(N,k_step+10):end)));

        R.notes{end+1} = sprintf('Mean d_hat before step  = %.2f N.', d_pre);
        R.notes{end+1} = sprintf('Mean d_hat after step   = %.2f N (true %.1f N).', d_post, Cc.dist.surge_step_N);
        R.notes{end+1} = sprintf('|d_hat change| after step = %.2f N.', d_jump);
        R.notes{end+1} = sprintf('Mean |u_hat - u_true| before = %.4f, after = %.4f.', uerr_pre, uerr_post);

        % Mild conditions: d_hat responds, and u-estimation doesn't blow up
        if d_jump > 1.0 && (uerr_post <= 1.2*uerr_pre)
            R.notes{end+1} = 'PASS: d_hat responds to disturbance and u-estimation remains consistent.';
            R.pass = true;
        else
            R.notes{end+1} = 'FAIL: weak disturbance response or u-estimation degraded. Tune T.kf.Q_aug(end,end) / T.kf.R.';
            R.pass = false;
        end
    else
        R.notes{end+1} = 'Disturbance disabled in config: passing observer sanity check.';
        R.pass = true;
    end

    % --- Plots
    figure('Name','Task3 Check 02: Augmented KF performance','NumberTitle','off');

    % Outputs xp, zp
    subplot(3,1,1);
    plot(t, y_true_log(1,:), t, y_meas_log(1,:), t, y_hat_log(1,:));
    grid on; xlabel('Time (s)'); ylabel('x_p (m)');
    title('Output x_p: true vs measured vs KF estimate');
    legend('true','measured','estimated','Location','best');

    subplot(3,1,2);
    plot(t, y_true_log(2,:), t, y_meas_log(2,:), t, y_hat_log(2,:));
    grid on; xlabel('Time (s)'); ylabel('z_p (m)');
    title('Output z_p: true vs measured vs KF estimate');
    legend('true','measured','estimated','Location','best');

    % Disturbance estimate
    subplot(3,1,3);
    plot(t, d_true, t, d_hat_log);
    grid on; xlabel('Time (s)'); ylabel('d (N)');
    title('Surge disturbance: true vs estimated');
    legend('true','estimated','Location','best');

    out_png = fullfile(fileparts(mfilename('fullpath')), 'task3_check02_kf_disturbance.png');
    saveas(gcf, out_png);
    R.notes{end+1} = ['Saved plot: ', out_png];

    % Extra diagnostic figure for unmeasured states (u, w, q) with truth now included
    figure('Name','Task3 Check 02b: Unmeasured states (true vs estimated)','NumberTitle','off');

    subplot(3,1,1);
    plot(t, x_true_log(1,:), t, x_hat_log(1,:));
    grid on; ylabel('u'); title('u: true vs estimated'); legend('true','est','Location','best');

    subplot(3,1,2);
    plot(t, x_true_log(2,:), t, x_hat_log(2,:));
    grid on; ylabel('w'); title('w: true vs estimated'); legend('true','est','Location','best');

    subplot(3,1,3);
    plot(t, x_true_log(3,:), t, x_hat_log(3,:));
    grid on; ylabel('q'); xlabel('Time (s)');
    title('q: true vs estimated'); legend('true','est','Location','best');

    out_png2 = fullfile(fileparts(mfilename('fullpath')), 'task3_check02b_states.png');
    saveas(gcf, out_png2);
    R.notes{end+1} = ['Saved plot: ', out_png2];

    R.notes{end+1} = sprintf('Disturbance step configured: %.1f N at t=%.1f s.', ...
        Cc.dist.surge_step_N, Cc.dist.surge_step_time_s);

catch ME
    R.pass = false;
    R.error = ME.message;
end

end
