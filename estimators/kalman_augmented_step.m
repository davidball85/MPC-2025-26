function [KF, info] = kalman_augmented_step(KF, u, y)
% KALMAN_AUGMENTED_STEP
% One discrete-time KF step for augmented model.
%
% Inputs:
%   KF : struct from kalman_augmented_init
%   u  : (nu x 1) input at time k
%   y  : (ny x 1) measurement at time k
%
% Outputs:
%   KF   : updated filter struct
%   info : diagnostics (innovation, K gain, etc.)

A = KF.A; B = KF.B; Ck = KF.C;
Q = KF.Q; R = KF.R;

x = KF.xhat;   % <-- changed
P = KF.P;

u = u(:);
y = y(:);

% Predict
x_pred = A*x + B*u;
P_pred = A*P*A' + Q;

% Update
S = Ck*P_pred*Ck' + R;

% Guard against numerical issues
if rcond(S) < 1e-12
    S = S + 1e-9*eye(size(S));
end

K = (P_pred*Ck') / S;
innov = y - Ck*x_pred;

x_upd = x_pred + K*innov;

I = eye(size(P_pred));
P_upd = (I - K*Ck)*P_pred*(I - K*Ck)' + K*R*K';  % Joseph form

KF.xhat = x_upd;   % <-- changed
KF.P = P_upd;

info = struct();
info.innovation = innov;
info.K = K;
info.S = S;
end
