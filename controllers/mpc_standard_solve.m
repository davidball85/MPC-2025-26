function [u_abs, du_opt, err] = mpc_standard_solve(M, x_abs, x_ref, u_ref)
% mpc_standard_solve
%
% Solve standard MPC and return absolute control u_abs.
%
% Inputs:
%   M     - YALMIP optimizer from mpc_standard_build
%   x_abs - current absolute state
%   x_ref - current absolute reference state (at k=0)
%   u_ref - reference (steady-state) input
%
% Outputs:
%   u_abs  - absolute control input to apply (u_ref + du_opt)
%   du_opt - optimal deviation input at current step
%   err    - diagnostic struct (no command-window prints)

err = struct();
err.ok = false;
err.yalmip_problem = [];
err.yalmip_info = '';
err.solveroutput = [];

du_opt = zeros(size(u_ref));
u_abs  = u_ref;

try
    x_tilde0 = x_abs - x_ref;

    % Solve using optimizer
    out = M{x_tilde0};

    % YALMIP optimizer returns a cell array; first entry is the output
    if iscell(out)
        du = out{1};
    else
        du = out;
    end

    if isempty(du)
        err.yalmip_problem = 1;
        err.yalmip_info = 'Empty solution from optimizer.';
        return;
    end

    du_opt = du;
    u_abs  = u_ref + du_opt;

    err.ok = true;

catch ME
    err.yalmip_problem = -1;
    err.yalmip_info = ME.message;
end

end
