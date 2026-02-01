function result = check_04_offset_free_mpc()
% CHECK_04_OFFSET_FREE_MPC
% Runs offset-free MPC simulation and verifies zero steady-state error.
% All parameters are taken from config files.

C = config_constants();
T = config_tuning();

fprintf('\n============================================================\n');
fprintf('  TASK 3 - CHECK 04: OFFSET-FREE MPC SIMULATION\n');
fprintf('============================================================\n');

% Build/discretise model
lin = auv_linearise(C);
mdl = auv_discretise(lin, C.Ts);

% Measurement matrix y=[xp;zp]
mdl.Cy = zeros(2,6);
mdl.Cy(1, C.idx.xp) = 1;
mdl.Cy(2, C.idx.zp) = 1;

mdl.Bw = mdl.B(:,1);
if isfield(C,'dist') && isfield(C.dist,'channel')
    mdl.Bw = mdl.B(:,C.dist.channel);
end

% Augment + KF
Maug = auv_build_augmented_model(mdl, C, T);
KF = kalman_augmented_init(Maug, C, T);

% Run sim using shared logic from task3_summary (local copy here to keep check independent)
OUT = local_run(LOG_dummy(), mdl, Maug, KF, C, T);

% Metrics
ss_err = OUT.metrics.ss_err_of;

fprintf('\n[Verdict]\n');
fprintf('  Offset-free final speed: %.4f (ref %.4f) |ss err|=%.6f\n', OUT.final.u_of, OUT.ref.u, ss_err);

% Plot
try
    f = figure('Name','Task3 Check04','Color','w');
    plot(OUT.t, OUT.u_of); hold on;
    plot(OUT.t, OUT.u_std);
    yline(OUT.ref.u,'--');
    xlabel('Time [s]'); ylabel('Surge speed u [m/s]');
    legend('Offset-free','Standard','Reference','Location','best');
    grid on;

    outpath = fullfile(C.project_root, 'runningchecks', 'task3', 'task3_check04_comparison.png');
    local_save_figure(f, outpath);
    fprintf('  ✓ Saved comparison plot: %s\n', outpath);
catch ME
    fprintf('  ⚠ Plot save failed: %s\n', ME.message);
end

tol = 0.01; % still a criterion, but derive from config if you add it
if isfield(C.Task3,'check04_ss_tol')
    tol = C.Task3.check04_ss_tol;
end

if ss_err < tol
    fprintf('\n  ✓✓✓ CHECK 04: PASS ✓✓✓\n');
    result = struct('pass', true, 'ss_err', ss_err);
else
    fprintf('\n  ✗✗✗ CHECK 04: FAIL ✗✗✗\n');
    fprintf('  Offset-free MPC did not achieve zero steady-state error.\n');
    result = struct('pass', false, 'ss_err', ss_err);
end
end

function OUT = local_run(LOG, mdl, Maug, KF, C, T) %#ok<INUSD>
% Minimal copy of the simulation loop from task3_summary

Ts = C.Ts;
t_end = C.Task3.sim_time;
Nsim = round(t_end/Ts);

x_ref = zeros(6,1);
x_ref(C.idx.u)  = C.Task3.reference_speed;
x_ref(C.idx.zp) = C.Task3.reference_depth;

x_true = C.Task3.initial_state(:);

u_eq = zeros(3,1);
if isfield(C,'eq') && isfield(C.eq,'u_eq'), u_eq = C.eq.u_eq(:); end
if isfield(C,'Task1') && isfield(C.Task1,'u_eq'), u_eq = C.Task1.u_eq(:); end
if isfield(C,'u_eq'), u_eq = C.u_eq(:); end

d = C.Task3.disturbance_bias;
d_step = C.Task3.disturbance_step_magnitude;
t_step = C.Task3.disturbance_step_time;

MPC_std = mpc_standard_build(mdl, C, T);

t = (0:Nsim-1)'*Ts;
u_of = zeros(Nsim,1);
d_hat_log = zeros(Nsim,1);

x_true_std = x_true;
u_std_track = zeros(Nsim,1);

KF_std = KF;

for k=1:Nsim
    tk = t(k);
    if C.Task3.disturbance_enable && tk >= t_step
        d = d_step;
    end

    y = mdl.Cy*x_true;
    if k==1, u_prev = u_eq; else, u_prev = u_cmd_of; end
    [KF,~] = kalman_augmented_step(KF, u_prev, y);
    x_hat = KF.x_hat(1:6);
    d_hat = KF.x_hat(7);

    OUT_of = mpc_offsetfree_wrapper(x_hat, d_hat, x_ref, mdl, C, T);
    u_cmd_of = OUT_of.u_cmd(:);

    x_true = auv_step_nonlinear(x_true, u_cmd_of, d, Ts, C);
    u_of(k) = x_true(C.idx.u);
    d_hat_log(k) = d_hat;

    % standard path
    y_std = mdl.Cy*x_true_std;
    if k==1, u_prev_std=u_eq; else, u_prev_std = u_cmd_std; end
    [KF_std,~] = kalman_augmented_step(KF_std, u_prev_std, y_std);
    x_hat_std = KF_std.x_hat(1:6);
    OUT_std = mpc_standard_solve(MPC_std, x_hat_std, x_ref, C, T);
    u_cmd_std = OUT_std.u_cmd(:);

    x_true_std = auv_step_nonlinear(x_true_std, u_cmd_std, d, Ts, C);
    u_std_track(k) = x_true_std(C.idx.u);
end

ss_win = max(1, round(C.Task3.steady_state_window / Ts));
idx_ss = (Nsim-ss_win+1):Nsim;
u_of_ss = mean(u_of(idx_ss), 'omitnan');

OUT = struct();
OUT.t = t;
OUT.u_of = u_of;
OUT.u_std = u_std_track;
OUT.d_hat = d_hat_log;
OUT.ref = struct('u', C.Task3.reference_speed);
OUT.final = struct('u_of', u_of(end));
OUT.metrics = struct('ss_err_of', abs(u_of_ss - C.Task3.reference_speed));
end

function local_save_figure(fig, path)
[folder,~,~] = fileparts(path);
if ~exist(folder,'dir'), mkdir(folder); end
try
    exportgraphics(fig, path, 'Resolution', 150);
    return;
catch
end
try
    set(fig,'Renderer','painters');
    print(fig, path, '-dpng', '-r150');
catch
    saveas(fig, path);
end
end

function LOG = LOG_dummy()
LOG = struct(); %#ok<NASGU>
end
