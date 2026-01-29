function est = kalman_augmented_step(est, u, y)
% kalman_augmented_step
% One full KF predict + update step (time-varying gain).

% Predict
x_pred = est.A * est.xhat + est.B * u;
P_pred = est.A * est.P * est.A' + est.Q;

% Kalman gain
S = est.C * P_pred * est.C' + est.R;
K = P_pred * est.C' / S;

% Update
est.xhat = x_pred + K * (y - est.C * x_pred);
est.P    = (eye(size(P_pred)) - K * est.C) * P_pred;
end
