function T = config_tuning()
% CONFIG_TUNING - Tunable parameters for MPC design
%
% This file contains all "soft" parameters that can be adjusted
% without changing the fundamental problem structure.
%
% USAGE:
%   Tcfg = config_tuning();
%   N = Tcfg.N;           % Prediction horizon
%   Q = Tcfg.Q;           % State weights
%
% ORGANIZATION:
%   - MPC horizons and weights
%   - Task 2 settings
%   - Task 3: Kalman filter tuning
%   - Task 3: Soft constraint penalties

%% ====================================================================
%  MPC PREDICTION HORIZON
%  ====================================================================
T.N = 80;  % [samples] Prediction horizon
           % Rule of thumb: N * Ts ≈ 2-3 × settling time

%% ====================================================================
%  MPC STATE WEIGHTS (Q matrix)
%  State order: [u; w; q; xp; zp; theta]
%  ====================================================================
T.Q = diag([ ...
    50,  ... % u (surge velocity)     - Track speed setpoint
    1,   ... % w (heave velocity)     - Regulate to zero
    1,   ... % q (pitch rate)         - Regulate to zero
    0,   ... % xp (horizontal position) - Don't penalize (use constraints only)
    200, ... % zp (depth)             - Track depth setpoint strongly
    50   ... % theta (pitch angle)    - Keep pitch near zero
]);

%% ====================================================================
%  MPC INPUT WEIGHTS (R matrix)
%  Input order: [Tsurge; Theave; tau_pitch]
%  Higher values → smoother, less aggressive control
%  ====================================================================
T.R = diag([0.005, 0.05, 0.2]);

%% ====================================================================
%  TERMINAL COST
%  ====================================================================
T.useTerminalCost = true;
% If true, MPC calculates P = dare(A, B, Q, R) automatically
% This improves stability for finite horizons

%% ====================================================================
%  SOLVER SETTINGS
%  ====================================================================
T.solver = 'quadprog';  % MATLAB's built-in QP solver
T.verbose = 0;          % Solver output: 0=silent, 1=minimal, 2=detailed

%% ====================================================================
%  TASK 2: REFERENCE STATES
%  ====================================================================
T.x_ref_task2 = [1; 0; 0; 0; 5; 0];
% Target: u=1 m/s, depth=5m, everything else regulated to zero

%% ====================================================================
%  TASK 3: REFERENCE STATES (Offset-Free MPC)
%  ====================================================================
T.x_ref_task3 = [1; 0; 0; 0; 5; 0];
% Same as Task 2, but now we handle disturbances

%% ====================================================================
%  TASK 3: KALMAN FILTER TUNING (Augmented Observer)
%  ====================================================================
T.kf = struct();

% Initial Conditions
T.kf.d_hat0 = 0;  % [N] Initial disturbance estimate (unknown at start)

% Initial State Covariance
% P0(i,i) = initial uncertainty in state i
T.kf.P0_aug = diag([ ...
    1,   ... % u  - Velocity uncertainty
    1,   ... % w
    1,   ... % q
    10,  ... % xp - Position uncertainty
    10,  ... % zp
    1,   ... % theta
    200  ... % d  - HIGH uncertainty in disturbance (unknown force)
]);

T.kf.Q_aug = 1e-4*eye(7);
T.kf.Q_aug(end,end) = 5e-1;

% TUNING TIP for T.kf.Q_aug(end):
%   - Too low (1e-3):  Disturbance estimate changes very slowly (like low Ki in PID)
%   - Too high (1e1):  Disturbance estimate reacts to noise (chattering)
%   - Typical range: 1e-2 to 1e0

% Measurement Noise Covariance (R_kf)
% R(i,i) = sensor noise variance for output i
T.kf.R = diag([1e-2, 1e-2]);  % For y = [xp; zp] (position and depth sensors)

%% ====================================================================
%  TASK 3: SOFT CONSTRAINT PENALTIES
%  ====================================================================
T.soft = struct();

% Slack variable penalties (ρ)
% Higher values → constraints are "harder" (less violation allowed)
T.soft.rho_mpc = 1e5;     % MPC slack penalty
T.soft.rho_target = 1e6;  % Target calculator slack penalty (usually higher)

% Which constraints should be softened?
% (Input constraints are ALWAYS hard - actuators have physical limits)
T.soft.use_zp = true;     % Soften depth constraints
T.soft.use_theta = true;  % Soften pitch angle constraints
T.soft.use_track = true;  % Allow slack in reference tracking

%% ====================================================================
%  TASK 3/4: FEATURE FLAGS
%  ====================================================================
T.task3 = struct();
T.task3.use_wall = true;  % Enable virtual wall constraint in offset-free MPC

end
