function est = kalman_augmented_init(Maug, C, T)
% KALMAN_AUGMENTED_INIT
% Initialise an augmented Kalman Filter for the (x,d) model:
%   x_aug(k+1) = A_aug x_aug(k) + B_aug u(k) + w(k)
%   y(k)       = C_aug x_aug(k) + v(k)
%
% All tuning MUST come from config_tuning.m (T.kf.*).
%
% Inputs:
%   Maug : struct with fields (A_aug, B_aug, C_aug)
%   C    : config_constants() struct
%   T    : config_tuning() struct
%
% Output:
%   est : struct with fields A,B,C,Q,R,P,xhat,nx,ny,nu

arguments
    Maug (1,1) struct
    C    (1,1) struct
    T    (1,1) struct
end

A  = Maug.A_aug;
B  = Maug.B_aug;
Ck = Maug.C_aug;

nx = size(A,1);
nu = size(B,2);
ny = size(Ck,1);

if ~isfield(T,'kf')
    error('Missing T.kf tuning struct in config_tuning.m');
end

% ---- Initial estimate ----
d0 = 0;
if isfield(T.kf,'d_hat0')
    d0 = T.kf.d_hat0;
end

x0 = zeros(nx-1,1);
if isfield(C,'Task3') && isfield(C.Task3,'initial_state')
    x0 = C.Task3.initial_state(:);
elseif isfield(C,'x0')
    x0 = C.x0(:);
end

x_aug0 = [x0; d0];

% ---- Covariances ----
P0 = eye(nx);
if isfield(T.kf,'P0_aug')
    P0 = T.kf.P0_aug;
end

Q = 1e-3*eye(nx);
if isfield(T.kf,'Q_aug')
    Q = T.kf.Q_aug;
end

R = eye(ny);
if isfield(T.kf,'R')
    R = T.kf.R;
end

% ---- Pack estimator struct ----
est = struct();
est.A = A;
est.B = B;
est.C = Ck;
est.Q = Q;
est.R = R;

est.P    = P0;
est.xhat = x_aug0;

est.nx = nx;
est.nu = nu;
est.ny = ny;

end
