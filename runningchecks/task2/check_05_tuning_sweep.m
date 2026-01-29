function R = check_05_tuning_sweep()
% check_05_tuning_sweep
%
% Purpose
%   Sweep tuning parameters WITHOUT modifying sim_task2_standard.m.
%   Uses sim_task2_run_once(C,T) (check-only helper) to run each candidate.
%
% Sweep knobs (recommended, convex):
%   - Qxp = T.Q(4,4) >= 0   (position tracking weight)
%   - R11 = T.R(1,1) > 0    (surge effort penalty)
%
% Output
%   Prints a ranked table and exports:
%     - figures/task2_tuning_sweep.csv
%     - figures/task2_braking_wall_behaviour_best.png
%     - figures/task2_inputs_best.png

R = struct('pass', false, 'notes', {{}}, 'error', '');

try
    C  = config_constants();
    T0 = config_tuning();

    if ~isfield(C,'wall_stop')
        error('Missing C.wall_stop in config_constants.');
    end
    wall_stop = C.wall_stop;

    % --- Sweep definitions (EDIT THESE) ---
    % Keep these modest; each run is an 80 s sim with MPC inside.
    Qxp_vals = 0:0.5:5             % T.Q(4,4) (>=0)
    R11_vals = 0.005:0.005:0.06;     % T.R(1,1)
    % -------------------------------------

    nRuns = numel(Qxp_vals) * numel(R11_vals);

    fprintf('\n[Tuning sweep] Sweeping Qxp=T.Q(4,4) and R11=T.R(1,1)\n');
    fprintf('  Runs: %d (Qxp) x %d (R11) = %d simulations\n\n', numel(Qxp_vals), numel(R11_vals), nRuns);

    sweep = repmat(struct( ...
        'Qxp',[],'R11',[], ...
        'xp_final',[],'xp_max',[],'wall_margin_final',[],'wall_ok',[], ...
        'nSteps',[],'nSolved',[],'nFailed',[], ...
        'int_u2',[],'t_brake',[],'xp_brake',[]), nRuns, 1);

    logs_all = cell(nRuns,1);

    idx = 0;
    for iQ = 1:numel(Qxp_vals)
        for iR = 1:numel(R11_vals)
            idx = idx + 1;
            fprintf('\r  Solving %d/%d ...', idx, nRuns);

            T = T0;                    % local copy only
            T.Q(4,4) = Qxp_vals(iQ);    % xp weight
            T.R(1,1) = R11_vals(iR);    % surge effort weight

            logs_i = sim_task2_run_once(C, T);

            xp = logs_i.x(4,:);
            U  = logs_i.u;

            xp_final = xp(end);
            xp_max   = max(xp);

            met.Qxp = T.Q(4,4);
            met.R11 = T.R(1,1);

            met.xp_final = xp_final;
            met.xp_max   = xp_max;

            met.wall_margin_final = wall_stop - xp_final;
            met.wall_ok = (xp_max <= wall_stop + 0.05);

            met.nSteps  = numel(logs_i.err);
            met.nSolved = sum(logs_i.err == 0);
            met.nFailed = met.nSteps - met.nSolved;

            if ~isempty(U)
                met.int_u2 = sum(sum(U.^2,1)) * C.Ts;
                brakeIdx = find(U(1,:) < (mean(U(1,1:min(5,size(U,2)))) - 1e-6), 1, 'first');
                if isempty(brakeIdx)
                    met.t_brake  = NaN;
                    met.xp_brake = NaN;
                else
                    met.t_brake  = logs_i.t(brakeIdx);
                    met.xp_brake = logs_i.x(4,brakeIdx);
                end
            else
                met.int_u2   = NaN;
                met.t_brake  = NaN;
                met.xp_brake = NaN;
            end

            sweep(idx) = met;
            logs_all{idx} = logs_i;
        end
    end
    fprintf('\n');

    % --- Select best (feasible + closest) ---
    wall_ok = [sweep.wall_ok];
    margin  = [sweep.wall_margin_final];
    nFail   = [sweep.nFailed];
    effort  = [sweep.int_u2];

    valid = wall_ok & (margin >= -1e-6);

    if ~any(valid)
        error('No sweep candidate respected the wall constraint (xp_max <= wall_stop + 0.05).');
    end

    cand = find(valid);
    M = [margin(cand)', nFail(cand)', effort(cand)'];
    [~, ord] = sortrows(M, [1 2 3]);
    bestIdx = cand(ord(1));

    best = sweep(bestIdx);
    logs_best = logs_all{bestIdx};

    % --- Print table (top 10) ---
    fprintf('Sweep results (top 10 by closeness, then failures, then effort):\n');
    fprintf('  %-6s %-7s %-10s %-10s %-10s %-8s\n', 'Qxp','R11','xp_final','xp_max','margin','fail');
    fprintf('  %s\n', repmat('-',1,68));

    topK = min(10, numel(ord));
    for r = 1:topK
        ii = cand(ord(r));
        fprintf('  %-6.2f %-7.4f %-10.3f %-10.3f %-10.3f %-8d\n', ...
            sweep(ii).Qxp, sweep(ii).R11, sweep(ii).xp_final, sweep(ii).xp_max, sweep(ii).wall_margin_final, sweep(ii).nFailed);
    end

    fprintf('\nBest candidate selected:\n');
    fprintf('  Qxp = %.3f\n', best.Qxp);
    fprintf('  R11 = %.5f\n', best.R11);
    fprintf('  xp_final = %.3f m (margin = %.3f m)\n', best.xp_final, best.wall_margin_final);
    fprintf('  xp_max   = %.3f m\n', best.xp_max);
    fprintf('  failed MPC steps = %d / %d\n', best.nFailed, best.nSteps);
    fprintf('  effort proxy int||u||^2 dt = %.3e\n\n', best.int_u2);

    % --- Exports ---
    figDir = fullfile(pwd, 'figures');
    if exist(figDir,'dir') ~= 7
        mkdir(figDir);
    end

    % CSV table
    csvPath = fullfile(figDir, 'task2_tuning_sweep.csv');
    writetable(struct2table(sweep), csvPath);

    % Plots for best
    p1 = fullfile(figDir, 'task2_braking_wall_behaviour_best.png');
    p2 = fullfile(figDir, 'task2_inputs_best.png');

    f1 = figure(501); clf;
    subplot(3,1,1);
    plot(logs_best.t, logs_best.x(1,:)); grid on; ylabel('u (m/s)');
    subplot(3,1,2);
    plot(logs_best.t, logs_best.x(5,:)); grid on; ylabel('z_p (m)');
    subplot(3,1,3);
    plot(logs_best.t, logs_best.x(4,:)); hold on; grid on;
    yline(C.wall_stop,'--'); yline(C.wall_xp,'-');
    ylabel('x_p (m)'); xlabel('t (s)');
    legend('x_p','stop limit','wall','Location','best');
    sgtitle(sprintf('Best braking behaviour (Qxp=%.2f, R11=%.4g)', best.Qxp, best.R11));
    exportgraphics(f1, p1, 'Resolution', 200);

    f2 = figure(502); clf;
    plot(logs_best.t(1:end-1), logs_best.u'); grid on;
    xlabel('t (s)'); ylabel('Inputs');
    legend('Tsurge','Theave','taupitch','Location','best');
    title('Best MPC inputs');
    exportgraphics(f2, p2, 'Resolution', 200);

    R.notes{end+1} = sprintf('Sweep completed: %d candidates.', numel(sweep));
    R.notes{end+1} = sprintf('Best: Qxp=%.3f, R11=%.5f.', best.Qxp, best.R11);
    R.notes{end+1} = sprintf('Best stop: xp_final=%.3f m (margin=%.3f m).', best.xp_final, best.wall_margin_final);
    R.notes{end+1} = sprintf('Exported CSV: %s', csvPath);
    R.notes{end+1} = sprintf('Exported plots: %s', p1);
    R.notes{end+1} = sprintf('Exported plots: %s', p2);

    R.pass = true;

catch ME
    R.pass = false;
    R.error = ME.message;
end

end
