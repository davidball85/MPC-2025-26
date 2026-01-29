function est = kalman_augmented_step(est, u, y)
% kalman_augmented_step
% One KF update for the augmented observer.

% Predict
x_pred = est.A*est.xhat + est.B*u;
P_pred = est.A*est.P*est.A' + est.Q;

% Correct (steady-state gain)
est.xhat = x_pred + est.L*(y - est.C*x_pred);
est.P    = P_pred;  % steady-state KF: keep P propagated (simple + stable)
end
