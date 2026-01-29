function logs = task3_main()
% task3_main
% Full Task 3 simulation (nonlinear plant, augmented KF, target calc, offset-free MPC).
% Disturbance: -20N step at t=10s in surge channel (from config).



C = config_constants();
T = config_tuning();

% Linear model
[Ac, Bc, Cmeas, Dc] = auv_linearise(C);
[Ad, Bd, ~, ~]      = auv_discretise(Ac, Bc, Cmeas, Dc, C.Ts);

Bw = Bd(:,1);       % surge disturbance channel (force)
model.A  = Ad;
model.B  = Bd;
model.Bw = Bw;
model.Cy = Cmeas;

% Augmented observer model
[A_aug, B_aug, C_aug, ~] = auv_build_augmented_model(Ad, Bd, Cmeas);
est = kalman_augmented_init(A_aug, B_aug, C_aug, T);

% References
x_ref = T.x_ref_task3(:);
u_eq  = auv_equilibrium(x_ref, C);

% Simulation setup
Tend = 80;
K    = round(Tend/C.Ts);
t    = (0:K)'*C.Ts;

x = zeros(6,1);
x(C.idx.zp) = C.zp_min;

% Disturbance profile (from config)
d_true = C.dist.surge_bias_N * ones(K,1);
if C.dist.enable
    k_step = round(C.dist.surge_step_time_s / C.Ts) + 1;
    d_true(k_step:end) = C.dist.surge_step_N;
end

% Logs
logs.t = t;
logs.x = zeros(6,K+1); logs.x(:,1) = x;
logs.u = zeros(3,K);
logs.y = zeros(2,K);
logs.xhat = zeros(6,K);
logs.dhat = zeros(1,K);
logs.uss  = zeros(3,K);

for k = 1:K
    % measurement y=[xp;zp] with noise
    y_true = Cmeas*x;
    v = [sqrt(T.kf.R(1,1))*randn; sqrt(T.kf.R(2,2))*randn];
    y = y_true + v;

    % KF update (use previous applied input if available)
    if k == 1
        u_prev = u_eq;
    else
        u_prev = logs.u(:,k-1);
    end
    est = kalman_augmented_step(est, u_prev, y);

    x_hat = est.xhat(1:6);
    d_hat = est.xhat(end);

    % controller (offset-free MPC wrapper)
    OUT = mpc_offsetfree_wrapper(x_hat, d_hat, x_ref, model, C, T);
    u_cmd = OUT.u_cmd;

    % apply disturbance to plant via config field (no hardcoding)
    C.d_surge = d_true(k);

    % nonlinear plant step
    x = auv_step_nonlinear(x, u_cmd, C);

    % log
    logs.u(:,k)     = u_cmd;
    logs.y(:,k)     = y;
    logs.x(:,k+1)   = x;
    logs.xhat(:,k)  = x_hat;
    logs.dhat(k)    = d_hat;
    logs.uss(:,k)   = OUT.u_ss;
end

% --- Plots (report-ready)
figure('Name','Task 3: Offset-Free MPC Results','NumberTitle','off');

subplot(3,1,1);
plot(logs.t, logs.x(C.idx.u,:), logs.t(1:end-1), logs.xhat(C.idx.u,:));
grid on;
ylabel('u (m/s)');
title('Speed: true vs estimated');
legend('true','estimated','Location','best');

subplot(3,1,2);
plot(logs.t, logs.x(C.idx.zp,:), logs.t(1:end-1), logs.xhat(C.idx.zp,:));
grid on;
ylabel('z_p (m)');
title('Depth: true vs estimated');
legend('true','estimated','Location','best');

subplot(3,1,3);
plot(logs.t(1:end-1), d_true, logs.t(1:end-1), logs.dhat);
grid on;
ylabel('d (N)'); xlabel('Time (s)');
title('Disturbance: true vs estimated');
legend('true','estimated','Location','best');

out_png = fullfile(fileparts(mfilename('fullpath')), 'task3_main_results.png');
saveas(gcf, out_png);
fprintf('Saved: %s\n', out_png);

end
