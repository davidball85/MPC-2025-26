function OUT = mpc_offsetfree_wrapper(x_hat, d_hat, x_ref, model, C, T)
% MPC_OFFSETFREE_WRAPPER
% Offset-free MPC architecture (lecture notes):
%   1) Target calculation: compute (x_ss, u_ss) for current d_hat and refs
%   2) Deviation MPC: regulate dx = x - x_ss, du = u - u_ss
%   3) Apply u = u_ss + du0
%
% All modes/tuning should live in config files.
%
% Inputs:
%   x_hat : (nx x 1) estimated state
%   d_hat : scalar disturbance estimate (N)
%   x_ref : (nx x 1) desired reference
%   model : struct with discrete A,B,Bw
%   C,T   : configs
%
% Output:
%   OUT.u_cmd, OUT.u_ss, OUT.x_ss, OUT.du0, OUT.err_mpc, OUT.target_info

persistent MPC nx nu N_cached

A  = model.A;
B  = model.B;
Bw = model.Bw;

nx = size(A,1);
nu = size(B,2);

N = T.N;

% --- Target calculation mode from config
tc_mode = 'qp';
if isfield(C,'Task3') && isfield(C.Task3,'target_calc_mode')
    tc_mode = C.Task3.target_calc_mode;
end

data_tc = struct();
data_tc.x_hat = x_hat(:);
data_tc.d_hat = d_hat;
data_tc.x_ref = x_ref(:);

[x_ss, u_ss, info_tc] = target_calc(tc_mode, data_tc, model, C, T);

% --- (Re)build MPC if needed
need_rebuild = isempty(MPC) || isempty(N_cached) || (N_cached ~= N);
if need_rebuild
    N_cached = N;

    dx = sdpvar(nx, N+1);
    du = sdpvar(nu, N);

    % Soft constraint slacks
    s_zp    = sdpvar(1, N);
    s_theta = sdpvar(1, N);

    % parameters
    xhat_p = sdpvar(nx,1);
    xss_p  = sdpvar(nx,1);
    uss_p  = sdpvar(nu,1);

    obj  = 0;
    cons = [dx(:,1) == xhat_p - xss_p];

    % terminal cost flag
    P = T.Q;
    if isfield(T,'useTerminalCost') && ~T.useTerminalCost
        P = zeros(nx);
    end

    for k = 1:N
        cons = [cons, dx(:,k+1) == A*dx(:,k) + B*du(:,k)];

        obj = obj + dx(:,k)'*T.Q*dx(:,k) + du(:,k)'*T.R*du(:,k);

        x_abs = dx(:,k) + xss_p;
        u_abs = du(:,k) + uss_p;

        % Virtual wall (optional)
        if isfield(T,'task3') && isfield(T.task3,'use_wall') && T.task3.use_wall
            if isfield(C,'wall_stop_internal')
                wall_stop = C.wall_stop_internal;
            else
                wall_stop = C.wall_stop;
            end
            cons = [cons, x_abs(C.idx.xp) <= wall_stop];
        end

        % Hard input constraints
        cons = [cons, C.u_min <= u_abs <= C.u_max];

        % Soft zp constraints
        if isfield(T,'soft') && isfield(T.soft,'use_zp') && T.soft.use_zp
            cons = [cons, C.zp_min - s_zp(k) <= x_abs(C.idx.zp) <= C.zp_max + s_zp(k)];
            cons = [cons, s_zp(k) >= 0];
            obj  = obj + T.soft.rho_mpc * (s_zp(k)^2);
        else
            cons = [cons, C.zp_min <= x_abs(C.idx.zp) <= C.zp_max];
        end

        % Soft theta constraints
        if isfield(T,'soft') && isfield(T.soft,'use_theta') && T.soft.use_theta
            cons = [cons, C.theta_min - s_theta(k) <= x_abs(C.idx.theta) <= C.theta_max + s_theta(k)];
            cons = [cons, s_theta(k) >= 0];
            obj  = obj + T.soft.rho_mpc * (s_theta(k)^2);
        else
            cons = [cons, C.theta_min <= x_abs(C.idx.theta) <= C.theta_max];
        end
    end

    obj = obj + dx(:,N+1)'*P*dx(:,N+1);

    opts = sdpsettings('solver', T.solver, 'verbose', T.verbose);
    MPC = optimizer(cons, obj, opts, {xhat_p, xss_p, uss_p}, du(:,1));
end

% --- Solve
[du0, err_mpc] = MPC{x_hat(:), x_ss(:), u_ss(:)};

if err_mpc ~= 0 || any(isnan(du0))
    du0 = zeros(nu,1);
    u_cmd = u_ss(:);
else
    u_cmd = u_ss(:) + du0;
end

u_cmd = min(max(u_cmd, C.u_min), C.u_max);

OUT = struct();
OUT.u_cmd = u_cmd;
OUT.u_ss  = u_ss(:);
OUT.x_ss  = x_ss(:);
OUT.du0   = du0(:);
OUT.err_mpc = err_mpc;
OUT.target_info = info_tc;
OUT.target_mode = tc_mode;
end
