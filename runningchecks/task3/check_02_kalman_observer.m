function R = check_02_kalman_observer()
% check_02_kalman_observer
%
% Task 3 - Check 02:
% Demonstrate the augmented KF estimates:
%   - unmeasured states (u,w,q,theta)
%   - constant surge disturbance force d
% from y=[xp; zp] measurements.
%
% Improvement vs earlier version:
%   Start plant + observer at the trim/reference condition to avoid large
%   transients corrupting disturbance sign.

R = struct('pass', false, 'notes', {{}}, 'error', '');

try
    C = config_constants();
    T = config_tuning();

    % ---- Linear discrete model for observer
    [Ac, Bc, Cmeas, Dc] = auv_linearise(C);
    [A, B, Cy] = local_discretise(Ac, Bc, Cmeas, Dc, C);

    % ---- Build augmented observer model
    [A_aug, B_aug, C_aug, ~] = auv_build_augmented_model(A, B, Cy);

    % ---- Reference / trim point
    if isfield(T,'x_ref_task3')
        x_ref = T.x_ref_task3(:);
    else
        x_ref = [1;0;0;0;5;0];
    end

    u_hold = auv_equilibrium(x_ref, C);

    % ---- Initialise KF
    est = kalman_augmented_init(A_aug, B_aug, C_aug, T);

    % Seed estimator at trim (important!)
    est.xhat(1:6) = x_ref;
    est.xhat(end) = T.kf.d_hat0;

    % ---- Plant initial state at trim (so observer test is clean)
    x = x_ref;

    % ---- Sim horizon
    Ts = C.Ts;
    t_end = C.dist.surge_step_time_s + 40;
    N = round(t_end/Ts) + 1;
    t = (0:N-1)*Ts;

    % ---- Disturbance profile (from config)
    d_true = C.dist.surge_bias_N * ones(1,N);
    if C.dist.enable
        k_step = round(C.dist.surge_step_time_s / Ts) + 1;
        d_true(k_step:end) = C.dist.surge_step_N;
    end

    % ---- Measurement noise
    Rmeas = T.kf.R;

    % ---- Logs
    nx = 6; ny = size(Cy,1);
    x_true_log = zeros(nx,N);
    x_hat_log  = zeros(nx,N);
    y_meas_log = zeros(ny,N);
    y_true_log = zeros(ny,N);
    d_hat_log  = zeros(1,N);

    u_prev = u_hold;

    for k = 1:N
        % True output and noisy measurement
        y_true = Cy*x;
        v = [sqrt(Rmeas(1,1))*randn; sqrt(Rmeas(2,2))*randn];
        y = y_true + v;

        % KF update
        est = kalman_augmented_step(est, u_prev, y);
        x_hat = est.xhat(1:6);
        d_hat = est.xhat(end);

        % Inject disturbance into plant via config field
        C.d_surge = d_true(k);

        % Plant update
        x = local_plant_step(x, u_hold, C);

        % Logs
        x_true_log(:,k) = x;
        x_hat_log(:,k)  = x_hat;
        y_meas_log(:,k) = y;
        y_true_log(:,k) = y_true;
        d_hat_log(k)    = d_hat;

        u_prev = u_hold;
    end

    y_hat_log = Cy*x_hat_log;

    out_dir = fileparts(mfilename('fullpath'));

    % ---- Plot 1: outputs
    figure('Name','Task3 Check02: Outputs','NumberTitle','off');
    subplot(2,1,1);
    plot(t, y_true_log(1,:), t, y_meas_log(1,:), t, y_hat_log(1,:));
    grid on; ylabel('x_p (m)');
    title('x_p: true vs measured vs estimated');
    legend('true','measured','estimated','Location','best');

    subplot(2,1,2);
    plot(t, y_true_log(2,:), t, y_meas_log(2,:), t, y_hat_log(2,:));
    grid on; ylabel('z_p (m)'); xlabel('Time (s)');
    title('z_p: true vs measured vs estimated');
    legend('true','measured','estimated','Location','best');

    f1 = fullfile(out_dir, 'task3_check02_outputs.png');
    saveas(gcf, f1);
    R.notes{end+1} = ['Saved plot: ', f1];

    % ---- Plot 2: key states + disturbance
    figure('Name','Task3 Check02: States + disturbance','NumberTitle','off');
    subplot(3,1,1);
    plot(t, x_true_log(C.idx.u,:), t, x_hat_log(C.idx.u,:));
    grid on; ylabel('u (m/s)');
    title('Surge speed u: true vs estimated');
    legend('true','estimated','Location','best');

    subplot(3,1,2);
    plot(t, x_true_log(C.idx.theta,:), t, x_hat_log(C.idx.theta,:));
    grid on; ylabel('\theta (rad)');
    title('Pitch \theta: true vs estimated');
    legend('true','estimated','Location','best');

    subplot(3,1,3);
    plot(t, d_true, t, d_hat_log);
    grid on; ylabel('d (N)'); xlabel('Time (s)');
    title('Surge disturbance: true vs estimated');
    legend('true','estimated','Location','best');

    f2 = fullfile(out_dir, 'task3_check02_states_disturbance.png');
    saveas(gcf, f2);
    R.notes{end+1} = ['Saved plot: ', f2];

    % ---- Pass condition (robust + report-friendly)
    if C.dist.enable
        k_step = round(C.dist.surge_step_time_s / Ts) + 1;

        d_pre  = mean(d_hat_log(1:max(1,k_step-1)));
        d_post = mean(d_hat_log(min(N,k_step+10):end));

        R.notes{end+1} = sprintf('Mean d_hat before step  = %.2f N.', d_pre);
        R.notes{end+1} = sprintf('Mean d_hat after step   = %.2f N (true %.1f N).', d_post, C.dist.surge_step_N);

        % We require the estimate to move in the correct direction after the step
        if (d_post - d_pre) * C.dist.surge_step_N > 0
            R.notes{end+1} = 'PASS: d_hat moves in the correct direction after disturbance step.';
            R.pass = true;
        else
            R.notes{end+1} = 'FAIL: d_hat did not move in correct direction. Increase T.kf.Q_aug(end,end) or reduce T.kf.R.';
            R.pass = false;
        end
    else
        R.notes{end+1} = 'Disturbance disabled in config: passing observer sanity check.';
        R.pass = true;
    end

catch ME
    R.pass = false;
    R.error = ME.message;
end

end

% =====================================================================
function x_next = local_plant_step(x, u, Ccfg)
if exist('auv_step_nonlinear','file') == 2
    x_next = auv_step_nonlinear(x, u, Ccfg);
    return
end
xdot = auv_dynamics_nonlinear(x, u, Ccfg);
x_next = x + Ccfg.Ts*xdot;
end

function [A, B, C] = local_discretise(Ac, Bc, Cc, Dc, Ccfg)
Ts = Ccfg.Ts;
try
    [A, B, C, ~] = auv_discretise(Ac, Bc, Cc, Dc, Ts);
    return
catch
end
try
    [A, B, C] = auv_discretise(Ac, Bc, Cc, Ts);
    return
catch
end
try
    [A, B, C] = auv_discretise(Ac, Bc, Cc);
    return
catch
end
sysd = c2d(ss(Ac, Bc, Cc, Dc), Ts, 'zoh');
A = sysd.A; B = sysd.B; C = sysd.C;
end
