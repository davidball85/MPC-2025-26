function M = mpc_standard_build(Ad, Bd, x_ref, u_ref, C, T)
% ... (your header)

nx = size(Ad,1);
nu = size(Bd,2);
N  = T.N;

x_tilde  = sdpvar(nx, N+1);
u_tilde  = sdpvar(nu, N);
x0_tilde = sdpvar(nx,1);

obj  = 0;
cons = [];

cons = [cons, x_tilde(:,1) == x0_tilde];

if T.useTerminalCost
    P = T.Q;
else
    P = zeros(nx);
end

for k = 1:N
    % deviation dynamics (pure LTI)
    x_next = Ad*x_tilde(:,k) + Bd*u_tilde(:,k);
    
    cons = [cons, x_tilde(:,k+1) == x_next];

    % absolute variables with time-varying xp reference
    x_ref_k = x_ref;
    x_ref_k(4) = x_ref(4) + k * C.Ts * x_ref(1);  % xp_ref grows over horizon
    
    x_ref_kp1 = x_ref;
    x_ref_kp1(4) = x_ref(4) + (k+1) * C.Ts * x_ref(1);
    
    x_abs      = x_tilde(:,k)   + x_ref_k;
    x_abs_next = x_tilde(:,k+1) + x_ref_kp1;
    u_abs      = u_tilde(:,k)   + u_ref;

    % cost
    obj = obj + x_tilde(:,k)'*T.Q*x_tilde(:,k) + u_tilde(:,k)'*T.R*u_tilde(:,k);

    % constraints at step k, and next state constraints at k+1
    cons = mpc_constraints(cons, x_abs,      u_abs, C, k);
    cons = mpc_constraints(cons, x_abs_next, [],    C, k+1);
end

obj = obj + x_tilde(:,N+1)'*P*x_tilde(:,N+1);

opts = sdpsettings('solver', T.solver, 'verbose', T.verbose);
M = optimizer(cons, obj, opts, x0_tilde, u_tilde(:,1));

end
