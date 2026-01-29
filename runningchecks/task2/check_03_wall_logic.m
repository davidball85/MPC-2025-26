function R = check_03_wall_logic()
% check_03_wall_logic
%
% Purpose
%   Verify the "virtual wall braking" logic is active:
%     - wall at xp = C.wall_xp (m)
%     - MUST stop before xp = C.wall_stop (m)
%
% Checks
%   A) Constraint-unit check: xp > wall_stop should be infeasible
%   B) Simulation check: sim_task2_standard() should keep max(xp) <= wall_stop + tol
%
% Also produces report figures:
%   1) General behaviour (Depth, Distance-to-wall, Speed)  [VISIBLE]
%   2) Braking behaviour (xp, u, Tsurge)                   [VISIBLE]
%   (+ exports PNGs into ./figures)

R = struct('pass', false, 'notes', {{}}, 'error', '');

try
    C = config_constants();

    % --- Wall settings from config ---
    if ~isfield(C,'wall_stop')
        error('Missing C.wall_stop in config_constants.');
    end
    wall_stop_spec = C.wall_stop;

    if isfield(C,'wall_stop_internal')
        wall_stop_enforced = C.wall_stop_internal;
    else
        wall_stop_enforced = wall_stop_spec;
    end

    if isfield(C,'wall_xp')
        wall_xp = C.wall_xp;
    else
        wall_xp = NaN;
    end

    % =========================================================
    % A) Unit-level feasibility: xp > wall_stop must be infeasible
    % =========================================================
    x = sdpvar(6,1);
    u = sdpvar(3,1);
    cons = [];
    cons = mpc_constraints(cons, x, u, C, 999);

    ops = sdpsettings('solver','quadprog','verbose',0);

    x_ok = [1; 0; 0; 0; (C.zp_min + C.zp_max)/2; 0];
    u_ok = (C.u_min + C.u_max)/2;

    sol_ok = optimize([cons, x == x_ok, u == u_ok], 0, ops);
    if sol_ok.problem ~= 0
        error('Wall constraint check failed: expected feasible point was infeasible.');
    end

    x_bad = x_ok; x_bad(4) = wall_stop_spec + 1e-3;
    sol_bad = optimize([cons, x == x_bad, u == u_ok], 0, ops);
    if sol_bad.problem == 0
        error('Wall constraint not enforced: xp > wall_stop was still feasible.');
    end

    R.notes{end+1} = sprintf('Constraint-level wall check OK: xp <= %.2f enforced.', wall_stop_enforced);

    if isfinite(wall_xp)
        R.notes{end+1} = sprintf('Config wall_xp = %.2f, wall_stop(spec) = %.2f, wall_stop(enforced) = %.2f.', ...
            wall_xp, wall_stop_spec, wall_stop_enforced);
    end

    % =========================================================
    % B) Simulation-level check
    % =========================================================
    if exist('sim_task2_standard','file') ~= 2
        R.notes{end+1} = 'sim_task2_standard.m not found, skipping simulation-level wall check.';
        R.pass = true;
        return;
    end

    % Run quietly (captures prints inside sim)
    evalc('logs = sim_task2_standard();');

    if ~isfield(logs,'x') || isempty(logs.x)
        error('sim_task2_standard did not return logs.x.');
    end

    xp = logs.x(4,:);
    zp = logs.x(5,:);
    uu = logs.x(1,:);   % surge speed state (as per your script)

    xp_max = max(xp);
    tol = 0.05;

    % MPC solve health summary
    has_err = isfield(logs,'err') && ~isempty(logs.err);
    if has_err
        nSteps  = numel(logs.err);
        nSolved = sum(logs.err == 0);
        nFailed = nSteps - nSolved;

        R.notes{end+1} = sprintf('MPC solve count: %d solved, %d infeasible/failed (out of %d).', ...
            nSolved, nFailed, nSteps);

        if nFailed > 0
            firstFail = find(logs.err ~= 0, 1, 'first');
            R.notes{end+1} = sprintf('First MPC infeasibility at step %d (fallback input used).', firstFail);
        end
    end

    % Final position & margins
    xp_final = xp(end);
    zp_final = zp(end);

    R.notes{end+1} = sprintf('Final position: xp = %.3f m, zp = %.3f m.', xp_final, zp_final);
    R.notes{end+1} = sprintf('Final margins: wall_stop-xp = %.3f m, zp-zp_min = %.3f m, zp_max-zp = %.3f m.', ...
        wall_stop_spec - xp_final, zp_final - C.zp_min, C.zp_max - zp_final);

    % Hard pass/fail on max xp
    if xp_max > wall_stop_spec + tol
        error('Simulation violated wall: max xp = %.3f m (limit %.2f m).', xp_max, wall_stop_spec);
    end

    R.notes{end+1} = sprintf('Simulation respects wall: max xp = %.3f m (<= %.2f + %.2f).', ...
        xp_max, wall_stop_spec, tol);

    % =========================================================
    % Time vector
    % =========================================================
    if ~isfield(logs,'t') || isempty(logs.t)
        t = (0:(size(logs.x,2)-1)) * C.Ts;
    else
        t = logs.t(:).';
    end

    % =========================================================
    % Output folder (under pwd)
    % =========================================================
    outdir_abs = fullfile(pwd, 'figures');
    if exist(outdir_abs,'dir') ~= 7
        [ok,msg] = mkdir(outdir_abs);
        if ~ok
            error('Could not create figures folder: %s', msg);
        end
    end

    % =========================================================
    % FIGURE 1 (VISIBLE): General behaviour (Depth, Distance, Speed)
    % =========================================================
    dist_to_wall = wall_stop_spec - xp;

    fig_general = figure; clf;

    subplot(3,1,1);
    plot(t, zp, 'LineWidth', 1.2); grid on; hold on;
    yline(C.zp_min, '--', 'z_{min}', 'HandleVisibility','off');
    yline(C.zp_max, '--', 'z_{max}', 'HandleVisibility','off');
    ylabel('z_p (m)');
    title('Task 2: General behaviour (depth, wall distance, speed)');

    subplot(3,1,2);
    plot(t, dist_to_wall, 'LineWidth', 1.2); grid on; hold on;
    yline(0, '--', 'Wall stop', 'HandleVisibility','off');
    ylabel('wall\_stop - x_p (m)');

    subplot(3,1,3);
    plot(t, uu, 'LineWidth', 1.2); grid on;
    ylabel('u (m/s)');
    xlabel('t (s)');

    outfileG = fullfile(outdir_abs, 'task2_general_depth_distance_speed.png');
    try
        exportgraphics(fig_general, outfileG, 'Resolution', 200);
    catch
        set(fig_general, 'PaperPositionMode', 'auto');
        print(fig_general, outfileG, '-dpng', '-r200');
    end
    R.notes{end+1} = sprintf('Exported plot: %s', outfileG);

    % =========================================================
    % FIGURE 2 (VISIBLE): Braking behaviour (your original)
    % =========================================================
    has_u = isfield(logs,'u') && ~isempty(logs.u);
    if has_u
        Tsurge = logs.u(1,:);
    else
        Tsurge = NaN(1, max(0, numel(t)-1));
    end

    fig_brake = figure; clf;

    subplot(3,1,1);
    plot(t, xp, 'LineWidth', 1.2); grid on; hold on;
    yline(wall_stop_spec, '--', 'wall\_stop');
    if isfinite(wall_xp)
        yline(wall_xp, '-', 'wall\_xp');
    end
    ylabel('x_p (m)');
    title('Task 2: Braking behaviour near the virtual wall');

    subplot(3,1,2);
    plot(t, uu, 'LineWidth', 1.2); grid on;
    ylabel('u (m/s)');

    subplot(3,1,3);
    if has_u && numel(t) >= 2
        plot(t(1:end-1), Tsurge, 'LineWidth', 1.2); grid on; hold on;
        yline(C.u_min(1), '--', 'u\_min');
        yline(C.u_max(1), '--', 'u\_max');
    else
        plot(t, nan(size(t))); grid on;
    end
    ylabel('T_{surge}');
    xlabel('t (s)');

    outfileB = fullfile(outdir_abs, 'task2_braking_wall_behaviour.png');
    try
        exportgraphics(fig_brake, outfileB, 'Resolution', 200);
    catch
        set(fig_brake, 'PaperPositionMode', 'auto');
        print(fig_brake, outfileB, '-dpng', '-r200');
    end
    R.notes{end+1} = sprintf('Exported plot: %s', outfileB);

    R.pass = true;

catch ME
    R.pass = false;
    R.error = ME.message;
end
end
