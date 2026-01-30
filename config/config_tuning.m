function T = config_tuning()
% config_tuning
% Tuning parameters for Task 2 (and later Task 3).

%% Horizons
T.N = 80;                 % prediction horizon (samples)

%% Weights (state order: [u; w; q; xp; zp; theta])
% Tip: keep xp weight = 0 (we don't want to "regulate" position), but
% we will still CONSTRAIN xp for the wall.
T.Q = diag([ ...
    50, ...   % u (speed tracking)
    1,  ...   % w
    1,  ...   % q
    0,  ...   % xp
    200, ...  % zp (depth tracking)
    50  ...   % theta (keep pitch small)
]);

% Input order: [Tsurge; Theave; taupitch]
T.R = diag([0.005, 0.05, 0.2]);

%% Terminal cost option (simple: use same as Q for now)
% If you want, later you can set P = dare(Ad,Bd,Q,R) for a more "LQR-ish" tail.
T.useTerminalCost = true;

%% Solver
T.solver = 'quadprog';
T.verbose = 0;

T.x_ref_task2 = [1;0;0;0;5;0];


%% Task 3: references (track speed + depth, regulate others)
T.x_ref_task3 = [1; 0; 0; 0; 5; 0];

%% Task 3: augmented Kalman filter tuning (Chapter 8 style)
T.kf.d_hat0 = 0;   % initial disturbance estimate (N)

T.kf.P0_aug = diag([ ...
    1, 1, 1, 10, 10, 1, ...  % x states
    200 ...                  % disturbance d uncertainty
]);

T.kf.Q_aug = diag([ ...
    1e-3*ones(1,6), ...
    1e-1 ...
]);

T.kf.R = diag([1e-2, 1e-2]);  % for y=[xp; zp]

%% Task 3: soft constraint penalties (slack)
T.soft.rho_mpc    = 1e5;   % MPC slack penalty
T.soft.rho_target = 1e6;   % target-calc slack penalty (usually higher)

% Which constraints are softened (actuators remain hard)
T.soft.use_zp    = true;
T.soft.use_theta = true;

% For target tracking slack (speed/depth)
T.soft.use_track = true;

% Task 3/4 options
T.task3.use_wall = true;   % set false to disable wall constraint in offset-free MPC


end
