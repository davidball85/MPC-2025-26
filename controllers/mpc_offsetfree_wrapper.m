function OUT = mpc_offsetfree_wrapper(x_hat, d_hat, x_ref, model, C, T)
% mpc_offsetfree_wrapper
% Builds (persistent) target calculator + MPC optimiser and returns u_cmd.
%
% Inputs:
%   x_hat  (6x1) estimated state
%   d_hat  scalar disturbance estimate (N)
%   x_ref  (6x1) desired ref (u=1, zp=5, etc)
%   model  struct {A,B,Bw,Cy} discrete model (A,B from discretise; Bw=B(:,1))
%
% Output struct OUT:
%   OUT.u_cmd (3x1)
%   OUT.u_ss, OUT.x_ss
%   OUT.err_mpc, OUT.err_target

persistent MPC nx nu N

A  = model.A;
B  = model.B;
Bw = model.Bw;

nx = size(A,1);
nu = size(B,2);
N  = T.N;

% --- Target calc (soft)
data_tc.x_hat = x_hat;
data_tc.d_hat = d_hat;
data_tc.x_ref = x_ref;

[x_ss, u_ss, info_tc] = target_calc('qp', data_tc, model, C, T);

% --- Build MPC optimiser once
if isempty(MPC)
    dx = sdpvar(nx, N+1);
    du = sdpvar(nu, N);

    % slack for zp and theta only (per step)
    s_zp    = sdpvar(1, N);
    s_theta = sdpvar(1, N);

    % parameters
    xhat_p = sdpvar(nx,1);
    xss_p  = sdpvar(nx,1);
    uss_p  = sdpvar(nu,1);
    dhat_p = sdpvar(1,1);
    xp0_p  = sdpvar(1,1); % anchor xp steady-state to current xp estimate

    obj  = 0;
    cons = [dx(:,1) == xhat_p - xss_p];

    % terminal cost
    P = T.Q;
    if ~isfield(T,'useTerminalCost') || ~T.useTerminalCost
        P = zeros(nx);
    end

    for k = 1:N
        % Deviation dynamics with disturbance as constant parameter:
        % x_{k+1} = A x_k + B u_k + Bw d_hat
        % => dx_{k+1} = A dx_k + B du_k (disturbance cancels in deviation form)
        cons = [cons, dx(:,k+1) == A*dx(:,k) + B*du(:,k)];

        % cost
        obj = obj + dx(:,k)'*T.Q*dx(:,k) + du(:,k)'*T.R*du(:,k);

        % absolute variables
        x_abs = dx(:,k) + xss_p;
        u_abs = du(:,k) + uss_p;

        % hard input constraints
        cons = [cons, C.u_min <= u_abs <= C.u_max];

        % hard constraints that remain hard:
        % (wall logic stays in Task 2 checks; Task 3 focuses on disturbance rejection)

        % soft zp bounds
        if T.soft.use_zp
            cons = [cons, C.zp_min - s_zp(k) <= x_abs(C.idx.zp) <= C.zp_max + s_zp(k)];
            cons = [cons, s_zp(k) >= 0];
            obj  = obj + T.soft.rho_mpc * (s_zp(k)^2);
        else
            cons = [cons, C.zp_min <= x_abs(C.idx.zp) <= C.zp_max];
        end

        % soft theta bounds
        if T.soft.use_theta
            cons = [cons, C.theta_min - s_theta(k) <= x_abs(C.idx.theta) <= C.theta_max + s_theta(k)];
            cons = [cons, s_theta(k) >= 0];
            obj  = obj + T.soft.rho_mpc * (s_theta(k)^2);
        else
            cons = [cons, C.theta_min <= x_abs(C.idx.theta) <= C.theta_max];
        end
    end

    obj = obj + dx(:,N+1)'*P*dx(:,N+1);

    opts = sdpsettings('solver', T.solver, 'verbose', T.verbose);
    MPC = optimizer(cons, obj, opts, {xhat_p, xss_p, uss_p, dhat_p, xp0_p}, du(:,1));
end

% --- Solve MPC
[du0, err_mpc] = MPC{x_hat, x_ss, u_ss, d_hat, x_hat(C.idx.xp)};

if err_mpc ~= 0
    % fallback: apply u_ss
    u_cmd = u_ss;
else
    u_cmd = u_ss + du0;
end

OUT.u_cmd      = min(max(u_cmd, C.u_min), C.u_max);
OUT.u_ss       = u_ss;
OUT.x_ss       = x_ss;
OUT.err_mpc    = err_mpc;
OUT.err_target = info_tc;
end
