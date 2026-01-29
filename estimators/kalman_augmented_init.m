function est = kalman_augmented_init(A_aug, B_aug, C_aug, T)
% kalman_augmented_init
% Linear augmented Kalman filter (Chapter 8 structure).
% Uses time-varying Kalman gain (computed each step from P).

nx_aug = size(A_aug,1);

est.A = A_aug;
est.B = B_aug;
est.C = C_aug;

est.xhat = zeros(nx_aug,1);
est.xhat(end) = T.kf.d_hat0;

est.P = T.kf.P0_aug;
est.Q = T.kf.Q_aug;
est.R = T.kf.R;
end
