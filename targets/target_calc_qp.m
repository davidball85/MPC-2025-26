function [x_ss, u_ss, info] = target_calc_qp(data, model, C, T)
% target_calc_qp
% Constrained steady-state target calculator with soft constraints (slack).
% Matches Chapter 8 idea: solve for (x_ss, u_ss) given d_hat and reference.

persistent TC nx_red nu

if isempty(TC)
    % Reduced state for equilibrium target (ignore xp in steady-state)
    idx_red = [C.idx.u, C.idx.w, C.idx.q, C.idx.zp, C.idx.theta];
    nx_red = numel(idx_red);
    nu = size(model.B,2);

    Ared  = model.A(idx_red, idx_red);
    Bred  = model.B(idx_red, :);
    Bwred = model.Bw(idx_red, :);

    % Decision variables
    xss = sdpvar(nx_red,1);
    uss = sdpvar(nu,1);

    % Tracking slack (u and zp)
    s_tr = sdpvar(2,1);

    % Soft constraint slack for zp and theta bounds (optional)
    s_zp    = sdpvar(1,1);
    s_theta = sdpvar(1,1);

    % Parameters
    d_hat = sdpvar(1,1);
    r_u   = sdpvar(1,1);
    r_zp  = sdpvar(1,1);

    cons = [];

    % equilibrium with disturbance
    cons = [cons, xss == Ared*xss + Bred*uss + Bwred*d_hat];

    % input hard constraints
    cons = [cons, C.u_min <= uss <= C.u_max];

    % track u and zp with slack
    if T.soft.use_track
        cons = [cons, r_u  - s_tr(1) <= xss(1) <= r_u  + s_tr(1)];
        cons = [cons, r_zp - s_tr(2) <= xss(4) <= r_zp + s_tr(2)];
        cons = [cons, s_tr >= 0];
    else
        cons = [cons, xss(1) == r_u];
        cons = [cons, xss(4) == r_zp];
    end

    % soft state constraints (zp bounds)
    if T.soft.use_zp
        cons = [cons, C.zp_min - s_zp <= xss(4) <= C.zp_max + s_zp];
        cons = [cons, s_zp >= 0];
    else
        cons = [cons, C.zp_min <= xss(4) <= C.zp_max];
    end

    % soft theta bounds
    if T.soft.use_theta
        cons = [cons, C.theta_min - s_theta <= xss(5) <= C.theta_max + s_theta];
        cons = [cons, s_theta >= 0];
    else
        cons = [cons, C.theta_min <= xss(5) <= C.theta_max];
    end

    % Objective: small effort + heavy slack penalty
    obj = 1e-6*(uss'*uss);

    if T.soft.use_track
        obj = obj + T.soft.rho_target*(s_tr'*s_tr);
    end
    if T.soft.use_zp
        obj = obj + T.soft.rho_target*(s_zp^2);
    end
    if T.soft.use_theta
        obj = obj + T.soft.rho_target*(s_theta^2);
    end

    ops = sdpsettings('solver', T.solver, 'verbose', T.verbose);
    TC = optimizer(cons, obj, ops, {d_hat, r_u, r_zp}, {xss, uss});
end

% ----- Evaluate optimiser
x_hat = data.x_hat(:);
xp_ss = x_hat(C.idx.xp);

x_ref = data.x_ref(:);
r_u  = x_ref(C.idx.u);
r_zp = x_ref(C.idx.zp);

[dsol, err] = TC{data.d_hat, r_u, r_zp};

if err ~= 0
    % fallback: use linear method if QP fails
    [x_ss, u_ss, info] = target_calc_linear(data, model, C, T);
    info.method = 'qp_failed_fallback_linear';
    info.err = err;
    return
end

xss_red = dsol{1};
u_ss    = dsol{2};

idx_red = [C.idx.u, C.idx.w, C.idx.q, C.idx.zp, C.idx.theta];

x_ss = x_ref;
x_ss(idx_red) = xss_red;
x_ss(C.idx.xp) = xp_ss;

info.method = 'qp_soft_slack';
info.err = err;
end
