% Debug script to understand why MPC is infeasible
clear; clc;

% Add paths
addpath('config');
addpath('controllers');
addpath('models');

% Load configuration
C = config_constants();
T = config_tuning();

% Build linear model
[Ac, Bc, Cc, Dc] = auv_linearise(C);
[Ad, Bd, ~, ~] = auv_discretise(Ac, Bc, Cc, Dc, C.Ts);

% References
x_ref = [1; 0; 0; 0; 5; 0];
u_ref = auv_equilibrium(x_ref, C);

% Initial state (at rest at surface)
x0 = [0; 0; 0; 0; C.zp_min; 0];
x0_tilde = x0 - x_ref;

fprintf('Initial state (absolute): %s\n', mat2str(x0', 4));
fprintf('Initial state (deviation): %s\n', mat2str(x0_tilde', 4));
fprintf('Reference state: %s\n', mat2str(x_ref', 4));
fprintf('Wall stop: %.1f m\n', C.wall_stop);
fprintf('Prediction horizon: %d steps (%.1f seconds)\n', T.N, T.N * C.Ts);

% Manually simulate one step of MPC prediction to see what happens
fprintf('\n--- Manual prediction check ---\n');

% Assume zero control deviation
u_tilde_test = zeros(3,1);

% One step prediction
x_next_tilde = Ad * x0_tilde + Bd * u_tilde_test;
x_next_tilde(4) = x_next_tilde(4) + C.Ts * x_ref(1);  % xp ramp

x_next_abs = x_next_tilde + x_ref;

fprintf('After 1 step with zero control deviation:\n');
fprintf('  x_next (deviation): %s\n', mat2str(x_next_tilde', 4));
fprintf('  x_next (absolute): %s\n', mat2str(x_next_abs', 4));
fprintf('  xp after 1 step: %.4f m\n', x_next_abs(4));
fprintf('  zp after 1 step: %.4f m\n', x_next_abs(5));

% Check constraints
fprintf('\n--- Constraint check ---\n');
if x_next_abs(4) > C.wall_stop
    fprintf('  ERROR: xp = %.4f > wall_stop = %.1f\n', x_next_abs(4), C.wall_stop);
else
    fprintf('  OK: xp = %.4f <= wall_stop = %.1f\n', x_next_abs(4), C.wall_stop);
end

if x_next_abs(5) < C.zp_min || x_next_abs(5) > C.zp_max
    fprintf('  ERROR: zp = %.4f outside [%.1f, %.1f]\n', x_next_abs(5), C.zp_min, C.zp_max);
else
    fprintf('  OK: zp = %.4f in [%.1f, %.1f]\n', x_next_abs(5), C.zp_min, C.zp_max);
end

% Now try to build and solve MPC
fprintf('\n--- Trying to build and solve MPC ---\n');
try
    M = mpc_standard_build(Ad, Bd, x_ref, u_ref, C, T);
    fprintf('MPC built successfully.\n');
    
    [u_cmd, ~, err] = mpc_standard_solve(M, x0, x_ref, u_ref);
    
    if err == 0
        fprintf('MPC solved successfully!\n');
        fprintf('Commanded input: %s\n', mat2str(u_cmd', 4));
    else
        fprintf('MPC FAILED with err=%d\n', err);
    end
catch ME
    fprintf('Error building/solving MPC: %s\n', ME.message);
end

% Check if problem is feasible by examining the horizon
fprintf('\n--- Horizon analysis ---\n');
max_xp_reachable = 0;
x_sim = x0_tilde;
for k = 1:T.N
    % With maximum forward thrust
    u_max_dev = C.u_max - u_ref;
    x_sim = Ad * x_sim + Bd * u_max_dev;
    x_sim(4) = x_sim(4) + C.Ts * x_ref(1);
    max_xp_reachable = max(max_xp_reachable, x_sim(4) + x_ref(4));
end
fprintf('Maximum xp reachable in %d steps: %.2f m\n', T.N, max_xp_reachable);
fprintf('Wall stop limit: %.1f m\n', C.wall_stop);
if max_xp_reachable > C.wall_stop
    fprintf('WARNING: Can reach beyond wall within horizon!\n');
end
