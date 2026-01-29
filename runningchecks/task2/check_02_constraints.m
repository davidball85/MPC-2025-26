function R = check_02_constraints()
% check_02_constraints
%
% Purpose
%   Verify that controllers/mpc_constraints.m correctly enforces:
%     - input saturation: C.u_min <= u <= C.u_max
%     - depth window:     C.zp_min <= zp <= C.zp_max
%     - pitch limit:      |theta| <= C.theta_max
%
% Method
%   Feasibility tests using optimise():
%     1) Pick a clearly-valid (x,u) and confirm feasible.
%     2) For each constraint, violate it slightly and confirm infeasible.
%
% Output
%   R.pass, R.notes, R.error

R = struct('pass', false, 'notes', {{}}, 'error', '');

try
    C = config_constants();

    % Basic checks on constants (helps catch typos)
    assert(all(C.u_min < C.u_max), 'Input bounds invalid: u_min must be strictly < u_max.');
    assert(C.zp_min < C.zp_max,    'Depth bounds invalid: zp_min must be < zp_max.');
    assert(C.theta_max > 0,        'theta_max must be > 0 rad.');

    % Decision variables
    x = sdpvar(6,1);   % [u; w; q; xp; zp; theta]
    u = sdpvar(3,1);   % [Tsurge; Theave; taupitch]

% Build constraints (wall comes from config; we just test generic constraints here)
cons = [];
cons = mpc_constraints(cons, x, u, C, 999);




    ops = sdpsettings('solver','quadprog','verbose',0);

    % Helper: feasibility check for fixed numeric x,u
    isFeasible = @(xv, uv) ( ...
        optimize([cons, x == xv, u == uv], 0, ops).problem == 0 );

    % --- 1) A clearly-valid point ---
    x_ok = [1; 0; 0; 0; (C.zp_min + C.zp_max)/2; 0];
    u_ok = (C.u_min + C.u_max)/2;

    if ~isFeasible(x_ok, u_ok)
        error('Expected feasible point was infeasible. Check constraint implementation and constants.');
    end
    R.notes{end+1} = 'Feasible point test passed.';

    % Small margin for violation
    epsv = 1e-3;

    % --- 2) Input lower bound violation ---
    u_bad = u_ok; u_bad(1) = C.u_min(1) - epsv;
    if isFeasible(x_ok, u_bad)
        error('Input lower bound not enforced: u(1) below u_min was still feasible.');
    end
    R.notes{end+1} = 'Input lower bound enforced (infeasible when violated).';

    % --- 3) Input upper bound violation ---
    u_bad = u_ok; u_bad(2) = C.u_max(2) + epsv;
    if isFeasible(x_ok, u_bad)
        error('Input upper bound not enforced: u(2) above u_max was still feasible.');
    end
    R.notes{end+1} = 'Input upper bound enforced (infeasible when violated).';

    % --- 4) Depth lower bound violation ---
    x_bad = x_ok; x_bad(5) = C.zp_min - epsv;
    if isFeasible(x_bad, u_ok)
        error('Depth lower bound not enforced: zp below zp_min was still feasible.');
    end
    R.notes{end+1} = 'Depth lower bound enforced (infeasible when violated).';

    % --- 5) Depth upper bound violation ---
    x_bad = x_ok; x_bad(5) = C.zp_max + epsv;
    if isFeasible(x_bad, u_ok)
        error('Depth upper bound not enforced: zp above zp_max was still feasible.');
    end
    R.notes{end+1} = 'Depth upper bound enforced (infeasible when violated).';

    % --- 6) Pitch limit violation (+theta) ---
    x_bad = x_ok; x_bad(6) = C.theta_max + epsv;
    if isFeasible(x_bad, u_ok)
        error('Pitch limit not enforced: theta > theta_max was still feasible.');
    end
    R.notes{end+1} = 'Pitch upper bound enforced (infeasible when violated).';

    % --- 7) Pitch limit violation (-theta) ---
    x_bad = x_ok; x_bad(6) = -(C.theta_max + epsv);
    if isFeasible(x_bad, u_ok)
        error('Pitch limit not enforced: theta < -theta_max was still feasible.');
    end
    R.notes{end+1} = 'Pitch lower bound enforced (infeasible when violated).';

    R.pass = true;

catch ME
    R.pass = false;
    R.error = ME.message;
end
end
