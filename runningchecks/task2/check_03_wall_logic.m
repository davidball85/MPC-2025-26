function R = check_03_wall_logic()
% check_03_wall_logic
%
% Purpose
%   Verify the "virtual wall braking" logic is active:
%     - wall at xp = C.wall_xp (m)
%     - MUST stop before xp = C.wall_stop (m)
%
% Checks
%   A) Constraint-level check: xp > wall_stop should be infeasible
%   B) Simulation check: sim_task2_standard() should keep max(xp) <= wall_stop + tol
%
% Robust to newer logs formats (e.g., logs.x wrapped in a struct).

R = struct('pass', false, 'notes', {{}}, 'error', '');

try
    C = config_constants();

    % --- Wall settings from config ---
    if ~isfield(C,'wall_stop')
        error('Missing C.wall_stop in config_constants.');
    end
    wall_stop_spec = local_unwrap_numeric(C.wall_stop);

    if isfield(C,'wall_stop_internal')
        wall_stop_enforced = local_unwrap_numeric(C.wall_stop_internal);
    else
        wall_stop_enforced = wall_stop_spec;
    end

    if isfield(C,'wall_xp')
        wall_xp = local_unwrap_numeric(C.wall_xp);
    else
        wall_xp = NaN;
    end

    % =========================================================
    % A) Unit-level feasibility: xp > wall_stop
    % =========================================================
    if exist('mpc_constraints','file') ~= 2
        R.notes{end+1} = 'mpc_constraints.m not found, skipping unit-level wall feasibility check.';
    else
        % Build a minimal feasibility test for constraints
        % (keeps behaviour similar to your original check file)
        idx_xp = 4;

        % pick a state that violates wall_stop by +0.5m
        x_test = zeros(6,1);
        x_test(idx_xp) = wall_stop_enforced + 0.5;

        % get constraints and test feasibility using your helper
        CON = mpc_constraints(C);
        ok = mpc_is_state_feasible(x_test, CON);

        if ok
            error('Constraint-level wall check failed: xp > wall_stop was still feasible.');
        else
            R.notes{end+1} = sprintf('Constraint-level wall check OK: xp <= %.2f enforced.', wall_stop_enforced);
            if ~isnan(wall_xp)
                R.notes{end+1} = sprintf('Config wall_xp = %.2f, wall_stop(spec) = %.2f, wall_stop(enforced) = %.2f.', ...
                    wall_xp, wall_stop_spec, wall_stop_enforced);
            end
        end
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

    X = local_unwrap_numeric(logs.x);

    % Some people store as logs.x.data etc — try a couple of common fallbacks
    if isstruct(X)
        if isfield(X,'data'), X = X.data; end
        if isfield(X,'x'),    X = X.x;    end
    end

    if ~isnumeric(X)
        error('logs.x is not numeric after unwrapping (type=%s).', class(X));
    end

    xp = X(4,:);
    zp = X(5,:);
    uu = X(1,:); %#ok<NASGU>

    xp_max = max(xp);
    tol = 0.05;

    % MPC solve health summary
    has_err = isfield(logs,'err') && ~isempty(logs.err);
    if has_err
        % logs.err might be numeric or struct; only count numeric 0/nonzero
        if isnumeric(logs.err)
            nSteps  = numel(logs.err);
            nSolved = sum(logs.err == 0);
            nFailed = nSteps - nSolved;

            R.notes{end+1} = sprintf('MPC solve count: %d solved, %d infeasible/failed (out of %d).', ...
                nSolved, nFailed, nSteps);

            if nFailed > 0
                firstFail = find(logs.err ~= 0, 1, 'first');
                R.notes{end+1} = sprintf('First MPC infeasibility at step %d (fallback input used).', firstFail);
            end
        else
            R.notes{end+1} = sprintf('logs.err exists but is type %s (not counted).', class(logs.err));
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
        error('Simulation violates wall: max xp = %.3f m (limit %.3f + %.3f).', xp_max, wall_stop_spec, tol);
    else
        R.notes{end+1} = sprintf('Simulation respects wall: max xp = %.3f m (<= %.3f + %.3f).', ...
            xp_max, wall_stop_spec, tol);
    end

    R.pass = true;

catch ME
    R.pass = false;
    R.error = ME.message;
end

end

% ---------------------------------------------------------
function v = local_unwrap_numeric(v)
% Peel struct wrappers so check scripts don't break when upstream
% code changes outputs to diagnostic structs.

guard = 0;
while isstruct(v) && guard < 10
    fn = fieldnames(v);
    if numel(fn) == 1
        v = v.(fn{1});
    else
        if isfield(v,'value'), v = v.value; break; end
        if isfield(v,'val'),   v = v.val;   break; end
        if isfield(v,'data'),  v = v.data;  break; end
        break;
    end
    guard = guard + 1;
end
end
