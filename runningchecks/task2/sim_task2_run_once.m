function logs = sim_task2_run_once(C, T)
% sim_task2_run_once
%
% Helper for running-checks ONLY.
% Runs the Task 2 nonlinear plant + linear MPC simulation using:
%   - constraints from config_constants (C)
%   - tuning passed in (T)
%
% Important:
% - Captures output from mpc_standard_solve to suppress "MPC err=..." spam.
% - Does NOT modify any config file.
% - Does NOT create plots (checks can plot/export separately).

% Wall settings from config
if ~isfield(C,'wall_xp') || ~isfield(C,'wall_stop')
    error('Missing wall settings in config_constants: expected C.wall_xp and C.wall_stop.');
end
wall_xp   = C.wall_xp;

% Build linear model from Task 1 pipeline
[Ac, Bc, Cc, Dc] = auv_linearise(C);
[Ad, Bd, ~, ~]   = auv_discretise(Ac, Bc, Cc, Dc, C.Ts);

% References (inspection cruise)
x_ref = T.x_ref_task2(:);     % [u; w; q; xp; zp; theta]
u_ref = auv_equilibrium(x_ref, C);

% Build MPC optimiser
M = mpc_standard_build(Ad, Bd, x_ref, u_ref, C, T);

% Simulation setup
Tend = 80;                 % seconds
K    = round(Tend / C.Ts);

x = zeros(6,1);
x(5) = C.zp_min;

% Logs
logs.t   = (0:K)' * C.Ts;
logs.x   = zeros(6, K+1);
logs.u   = zeros(3, K);
logs.du  = zeros(3, K);
logs.err = zeros(1, K);

logs.x(:,1) = x;

for k = 1:K
    % Solve MPC silently (suppresses spam)
    evalc('[u_cmd, du_cmd, err] = mpc_standard_solve(M, x, x_ref, u_ref);');

    % Safety saturation
    u_cmd = min(max(u_cmd, C.u_min), C.u_max);

    % Nonlinear plant step
    x = auv_step_nonlinear(x, u_cmd, C);

    % Log
    logs.x(:,k+1) = x;
    logs.u(:,k)   = u_cmd;
    logs.du(:,k)  = du_cmd;
    logs.err(k)   = err;

    % Stop if we hit physical wall (should not happen)
    if x(4) >= wall_xp
        logs.t   = logs.t(1:k+1);
        logs.x   = logs.x(:,1:k+1);
        logs.u   = logs.u(:,1:k);
        logs.du  = logs.du(:,1:k);
        logs.err = logs.err(1:k);
        break;
    end
end

end
