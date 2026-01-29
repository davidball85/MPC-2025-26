function [u_abs, du_opt, err] = mpc_standard_solve(M, x_abs, x_ref, u_ref)
% mpc_standard_solve
%
% Solve MPC for current absolute state.
% If the QP is infeasible, return a safe fallback input (u_ref)
% and expose the failure via err.

x0_tilde = x_abs - x_ref;

[du_opt, err] = M{x0_tilde};

% Optional one-line decode (for debugging)
fprintf('MPC err=%d (%s)\n', err, yalmiperror(err));

if err ~= 0
    % MPC infeasible: apply safe fallback
    du_opt = zeros(size(u_ref));
    u_abs  = u_ref;
else
    u_abs = du_opt + u_ref;
end

end
