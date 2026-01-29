% check_01_equilibrium.m

addpath(genpath(pwd))

C = config_constants();

x_eq = C.x_eq;
u_eq = auv_equilibrium(x_eq, C);

xdot_eq = auv_dynamics_nonlinear(x_eq, u_eq, C);

disp('Equilibrium input u_eq =');
disp(u_eq);

disp('State derivative at equilibrium xdot_eq =');
disp(xdot_eq);

fprintf('\nExpected behaviour:\n');
fprintf('  u_dot, w_dot, q_dot ≈ 0\n');
fprintf('  xp_dot = %.1f m/s\n', x_eq(1));
fprintf('  zp_dot, theta_dot ≈ 0\n');
