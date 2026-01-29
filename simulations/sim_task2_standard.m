function [logs, sweep] = sim_task2_standard(mode)
% sim_task2_standard
%
% Task 2 simulation (nonlinear plant, linear MPC).
%
% Modes
%   logs = sim_task2_standard()
%       Runs a single simulation using config_constants + config_tuning.
%
%   [logs, sweep] = sim_task2_standard remember: with sweep mode
%       sim_task2_standard('sweep')
%       Runs a sweep of tuning values (no constraint hardcoding),
%       prints a table, picks the "best" run, exports plots, and returns
%       logs for the best candidate + the sweep table struct.
%
% Notes
% - Constraints are ALWAYS from config_constants() / mpc_constraints().
% - Tuning is modified only in a local copy of T (does not edit config files).
% - No clc here.

if nargin < 1 || isempty(mode)
    mode = 'single';
end

C = config_constants();
T0 = config_tuning();

% Pull wall settings from config
if ~isfield(C,'wall_xp') || ~isfield(C,'wall_stop')
    error('Missing wall settings in config_constants: expected C.wall_xp and C.wall_stop.');
end
wall_xp   = C.wall_xp;
wall_stop = C.wall_stop;

% Build linear model from Task 1 pipeline
[Ac, Bc, Cc, Dc] = auv_linearise(C);
[Ad, Bd, ~, ~]   = auv_discretise(Ac, Bc, Cc, Dc, C.Ts);

% References (inspection cruise)
x_ref = T0.x_ref_task2(:);   % [u; w; q; xp; zp; theta]
u_ref = auv_equilibrium(x_ref, C);

% -------------------------------------------------------------------------
% Helpers
% -------------------------------------------------------------------------
    function logs = run_once(T)
        % Build MPC optimiser
        M = mpc_standard_build(Ad, Bd, x_ref, u_ref, C, T);

        % Simulation setup
        Tend = 80;                 % seconds
        K    = round(Tend / C.Ts);

        x = zeros(6,1);
        x(5) = C.zp_min;

        logs.t   = (0:K)' * C.Ts;
        logs.x   = zeros(6, K+1);
        logs.u   = zeros(3, K);
        logs.du  = zeros(3, K);
        logs.err = zeros(1, K);

        logs.x(:,1) = x;

        for kk = 1:K
            [u_cmd, du_cmd, err] = mpc_standard_solve(M, x, x_ref, u_ref);

            % Safety saturation (primary saturation should already be in MPC constraints)
            u_cmd = min(max(u_cmd, C.u_min), C.u_max);

            % Nonlinear plant step (RK4)
            x = auv_step_nonlinear(x, u_cmd, C);

            % Log
            logs.x(:,kk+1) = x;
            logs.u(:,kk)   = u_cmd;
            logs.du(:,kk)  = du_cmd;
            logs.err(kk)   = err;

            % Stop if we hit the physical wall
            if x(4) >= wall_xp
                logs.t   = logs.t(1:kk+1);
                logs.x   = logs.x(:,1:kk+1);
                logs.u   = logs.u(:,1:kk);
                logs.du  = logs.du(:,1:kk);
                logs.err = logs.err(1:kk);
                break;
            end
        end
    end

    function met = metrics_from_logs(logs, T)
        xp = logs.x(4,:);
        zp = logs.x(5,:);
        uu = logs.x(1,:);
        U  = logs.u;

        met.N = T.N;
        met.R11 = T.R(1,1);
        met.Q_u = T.Q(1,1);

        met.xp_final = xp(end);
        met.zp_final = zp(end);
        met.u_final  = uu(end);

        met.xp_max   = max(xp);
        met.wall_margin_final = wall_stop - met.xp_final;
        met.wall_margin_min   = wall_stop - met.xp_max; % negative means overshoot (shouldn't happen)

        met.nSteps = numel(logs.err);
        met.nSolved = sum(logs.err == 0);
        met.nFailed = met.nSteps - met.nSolved;

        % effort proxy (integral of squared inputs)
        if ~isempty(U)
            Ts = C.Ts;
            met.int_u2 = sum(sum(U.^2,1))*Ts;
            met.int_Tsurge2 = sum(U(1,:).^2)*Ts;
            met.Tsurge_min = min(U(1,:));
            met.Tsurge_max = max(U(1,:));
        else
            met.int_u2 = NaN;
            met.int_Tsurge2 = NaN;
            met.Tsurge_min = NaN;
            met.Tsurge_max = NaN;
        end

        % braking onset (first time Tsurge < u_ref(1))
        if ~isempty(U)
            idx = find(U(1,:) < (u_ref(1) - 1e-6), 1, 'first');
            if isempty(idx)
                met.t_brake = NaN;
                met.xp_brake = NaN;
            else
                met.t_brake  = logs.t(idx);
                met.xp_brake = logs.x(4,idx);
            end
        else
            met.t_brake = NaN;
            met.xp_brake = NaN;
        end

        % feasibility flag for wall (with small tolerance)
        met.wall_ok = (met.xp_max <= wall_stop + 0.05);
    end

% -------------------------------------------------------------------------
% SINGLE RUN (default)
% -------------------------------------------------------------------------
if strcmpi(mode,'single')
    sweep = [];
    logs = run_once(T0);
    plot_and_export(logs, T0, wall_stop, wall_xp, '');
    return;
end

% -------------------------------------------------------------------------
% SWEEP MODE
% -------------------------------------------------------------------------
if ~strcmpi(mode,'sweep')
    error('Unknown mode "%s". Use "single" or "sweep".', mode);
end

% Ensure xp weight stays 0 (position not regulated)
T0.Q(4,4) = 0;

% Sweep definition: tune surge effort weight R(1,1)
% (Lower = pushes harder/longer, often gets closer before braking)
R11_vals = 0.005:0.005:0.08;   % adjust range if you want

% Optional: also try a couple horizons (kept small to avoid long runtimes)
N_vals = unique([T0.N, 100, 120]);

mets = [];
logs_all = cell(numel(R11_vals)*numel(N_vals),1);
idx = 0;

fprintf('\n[Tuning sweep] Sweeping R(1,1) (Tsurge effort) with Q_xp forced to 0.\n');
fprintf('  Candidates: %d (R11) x %d (N) = %d runs\n\n', numel(R11_vals), numel(N_vals), numel(R11_vals)*numel(N_vals));

nTotal = numel(R11_vals) * numel(N_vals);
nDone  = 0;


for iN = 1:numel(N_vals)
    for iR = 1:numel(R11_vals)
        idx = idx + 1;
        nDone = nDone + 1;
        fprintf('\r  Solving %d / %d', nDone, nTotal);


        T = T0;
        T.N = N_vals(iN);
        T.R(1,1) = R11_vals(iR);

        % Run quietly in case anything prints inside
        evalc('logs_i = run_once(T);');



        met_i = metrics_from_logs(logs_i, T);

        logs_all{idx} = logs_i;
        mets = [mets; met_i]; %#ok<AGROW>
    end

    fprintf('\n');

end

% Convert to table-like struct array (mets already is struct array)
sweep = mets;

% Selection rule:
%  1) must satisfy wall (xp_max <= wall_stop+0.05)
%  2) minimise final margin (wall_stop - xp_final), but keep it >= 0
%  3) minimise nFailed
%  4) minimise effort
ok = [sweep.wall_ok] & ([sweep.wall_margin_final] >= -1e-6);

if ~any(ok)
    warning('No sweep candidates satisfied the wall constraint. Returning the best (least violation) candidate.');
    % choose smallest xp_max overshoot (closest to wall without huge violation)
    [~, bestIdx] = min(abs([sweep.wall_margin_min]));
else
    cand = find(ok);

    wall_margin = [sweep(cand).wall_margin_final];
    nFailed     = [sweep(cand).nFailed];
    effort      = [sweep(cand).int_u2];

    % Lexicographic sort: margin (ascending), failures (ascending), effort (ascending)
    M = [wall_margin(:), nFailed(:), effort(:)];
    [~, ord] = sortrows(M, [1 2 3]);
    bestIdx = cand(ord(1));
end

logs = logs_all{bestIdx};

% Print summary table (compact)
fprintf('Sweep results (top 10 by closeness, then failures, then effort):\n');
fprintf('  %-6s %-7s %-10s %-10s %-10s %-8s %-8s\n', 'N','R11','xp_final','xp_max','margin','fail','effort');
fprintf('  %s\n', repmat('-',1,70));

% Build ranking the same way for display
if any(ok)
    cand = find(ok);
else
    cand = 1:numel(sweep);
end

wall_margin = [sweep(cand).wall_margin_final]';
nFailed     = [sweep(cand).nFailed]';
effort      = [sweep(cand).int_u2]';

M = [wall_margin, nFailed, effort];
[~, ord] = sortrows(M, [1 2 3]);

topK = min(10, numel(ord));
for r = 1:topK
    ii = cand(ord(r));
    fprintf('  %-6d %-7.4f %-10.3f %-10.3f %-10.3f %-8d %-8.2e\n', ...
        sweep(ii).N, sweep(ii).R11, sweep(ii).xp_final, sweep(ii).xp_max, sweep(ii).wall_margin_final, sweep(ii).nFailed, sweep(ii).int_u2);
end

% Print "best"
best = sweep(bestIdx);
fprintf('\nBest candidate selected:\n');
fprintf('  N   = %d\n', best.N);
fprintf('  R11 = %.5f\n', best.R11);
fprintf('  xp_final = %.3f m (margin = %.3f m)\n', best.xp_final, best.wall_margin_final);
fprintf('  xp_max   = %.3f m\n', best.xp_max);
fprintf('  failed MPC steps = %d / %d\n', best.nFailed, best.nSteps);
fprintf('  braking onset: t = %.2f s, xp = %.3f m\n', best.t_brake, best.xp_brake);
fprintf('  effort proxy int||u||^2 dt = %.3e\n\n', best.int_u2);

% Export plots for best
tag = sprintf('_best_N%d_R11_%g', best.N, best.R11);
plot_and_export(logs, struct('N',best.N,'R11',best.R11,'Q_u',best.Q_u), wall_stop, wall_xp, tag);

end

% -------------------------------------------------------------------------
% Local plotting + export helper
% -------------------------------------------------------------------------
function plot_and_export(logs, Tinfo, wall_stop, wall_xp, tag)
C = config_constants();

figDir = fullfile(pwd, 'figures');
if exist(figDir,'dir') ~= 7
    mkdir(figDir);
end

% Figure 1: u, zp, xp
f1 = figure(1); clf;
set(f1, 'Name', 'task2_summary');

subplot(3,1,1);
plot(logs.t, logs.x(1,:)); grid on;
ylabel('u (m/s)');

subplot(3,1,2);
plot(logs.t, logs.x(5,:)); grid on;
ylabel('z_p (m)');

subplot(3,1,3);
plot(logs.t, logs.x(4,:)); hold on; grid on;
yline(wall_stop,'--');
yline(wall_xp,'-');
ylabel('x_p (m)'); xlabel('t (s)');
legend('x_p','stop limit','wall','Location','best');

ttl = 'Task 2: Braking behaviour near the virtual wall';
if isfield(Tinfo,'R11')
    ttl = sprintf('%s (N=%d, R11=%.4g)', ttl, Tinfo.N, Tinfo.R11);
end
sgtitle(ttl);

% Figure 2: inputs
f2 = figure(2); clf;
plot(logs.t(1:end-1), logs.u'); grid on;
xlabel('t (s)'); ylabel('Inputs');
legend('Tsurge','Theave','taupitch','Location','best');
title('MPC inputs');

% Export
name1 = fullfile(figDir, ['task2_braking_wall_behaviour' tag '.png']);
name2 = fullfile(figDir, ['task2_inputs' tag '.png']);

exportgraphics(f1, name1, 'Resolution', 200);
exportgraphics(f2, name2, 'Resolution', 200);

fprintf('Exported plots:\n');
fprintf('  %s\n', name1);
fprintf('  %s\n', name2);
end