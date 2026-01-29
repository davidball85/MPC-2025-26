function [x_ss, u_ss, info] = target_calc_linear(data, model, C, T)
% target_calc_linear
% Chapter-8 style pseudo-inverse approach (fast, no constraints).
% Mainly useful as a baseline to compare against constrained QP target calc.

A  = model.A;
B  = model.B;
Bw = model.Bw;

x_hat = data.x_hat(:);
d_hat = data.d_hat;
x_ref = data.x_ref(:);

% Track u and zp (remove xp from target definition)
% We'll set xp_ss = current xp_hat (so deviation xp starts at 0).
xp_ss = x_hat(C.idx.xp);

% Reduced state: [u; w; q; zp; theta]
idx_red = [C.idx.u, C.idx.w, C.idx.q, C.idx.zp, C.idx.theta];

Ared  = A(idx_red, idx_red);
Bred  = B(idx_red, :);
Bwred = Bw(idx_red, :);

xred_ref = x_ref(idx_red);

% Solve equilibrium + tracking as least squares:
% [I-Ared, -Bred] [xss_red] = [Bwred*d_hat]
% [Ctrack,   0  ] [ uss   ]   [r_track]
Ctrack = [1 0 0 0 0;   % u
          0 0 0 1 0];  % zp
r_track = [x_ref(C.idx.u); x_ref(C.idx.zp)];

M = [eye(5)-Ared, -Bred;
     Ctrack,      zeros(2,3)];

rhs = [Bwred*d_hat;
       r_track];

sol = pinv(M)*rhs;

xss_red = sol(1:5);
u_ss    = sol(6:8);

x_ss = x_ref;
x_ss(idx_red) = xss_red;
x_ss(C.idx.xp) = xp_ss;  % keep position anchored to current estimate

info.method = 'linear_pinv';
info.residual = norm(M*sol - rhs);
end
