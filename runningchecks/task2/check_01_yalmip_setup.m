function R = check_01_yalmip_setup()
% check_01_yalmip_setup
%
% Purpose
%   Sanity-check that:
%   1) YALMIP is on the path
%   2) The chosen QP solver is available (e.g. quadprog)
%   3) A tiny QP solves to the expected answer
%   4) YALMIP 'optimizer' objects work (basic functionality)
%
% Output
%   R.pass   true/false
%   R.notes  cell array of strings (what was checked)
%   R.error  error message (if failed)

R = struct('pass', false, 'notes', {{}}, 'error', '');

try
    % --- 1) YALMIP on path ---
    if exist('yalmip', 'file') ~= 2
        error('YALMIP not found on MATLAB path (cannot find function "yalmip").');
    end
    R.notes{end+1} = 'YALMIP appears to be on the MATLAB path.';

    % --- 2) Solver availability ---
    if exist('quadprog','file') ~= 2
        error('quadprog not found. Install Optimisation Toolbox or choose a different QP solver.');
    end
    R.notes{end+1} = 'quadprog is available.';

    % --- 3) Solve a tiny QP (unique solution) ---
    % min (x-2)^2  s.t.  x >= 0
    x = sdpvar(1,1);
    obj = (x - 2)^2;
    cons = [x >= 0];
    ops = sdpsettings('solver','quadprog','verbose',0);
    sol = optimize(cons, obj, ops);

    if sol.problem ~= 0
        error('Tiny QP did not solve. YALMIP solver status: %d', sol.problem);
    end

    xstar = value(x);
    if abs(xstar - 2) > 1e-6
        error('Tiny QP solved, but solution unexpected: x* = %.6f (expected 2).', xstar);
    end
    R.notes{end+1} = sprintf('Tiny QP solved correctly (x* = %.6f).', xstar);

    % --- 4) Check optimizer object works ---
    p = sdpvar(1,1);
    u = sdpvar(1,1);
    % min (u-p)^2  s.t.  -1 <= u <= 1
    cons2 = [-1 <= u <= 1];
    obj2  = (u - p)^2;
    M = optimizer(cons2, obj2, ops, p, u);

    u_out = M{0.4};
    if isempty(u_out)
        error('optimizer returned empty output.');
    end
    if abs(u_out - 0.4) > 1e-6
        error('optimizer returned unexpected value: u = %.6f (expected 0.4).', u_out);
    end
    R.notes{end+1} = 'YALMIP optimizer object works (simple parametric QP).';

    R.pass = true;

catch ME
    R.pass = false;
    R.error = ME.message;
end
end
