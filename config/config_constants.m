function C = config_constants()
% Fixed constants from coursework brief (MMH626810)

%% Sampling
C.Ts = 0.1;   % s

C.depth_enforce_from_k = 20;   % gives ~2 seconds at Ts=0.1


%% Physical parameters
C.mx = 100;      % kg
C.mz = 120;      % kg
C.Iy = 15;       % kg*m^2

C.Du = 50;
C.Dw = 80;
C.Dq = 15;

C.Mrest = 30;    % Nm

%% Input constraints
C.u_min = [-200; -200; -30];
C.u_max = [ 200;  200;  30];

%% State constraints
C.zp_min = 0.5;
C.zp_max = 10.0;

C.theta_min = deg2rad(-20);
C.theta_max = deg2rad( 20);

%% Linearisation operating point
C.x_eq = [1; 0; 0; 0; 5; 0];


% Virtual wall (Task 2)
C.wall_xp   = 50;      % physical wall location (m)
C.wall_stop = 48;      % must stop before this (m)

% Optional internal safety margin for enforcement (set to 0 if not needed)
C.wall_stop_margin = 0;                 % metres
C.wall_stop_internal = C.wall_stop - C.wall_stop_margin;

C.depth_enforce_from_k = 2;   % enforce zp bounds from prediction step k>=2


%% Task 3: Disturbance scenario (surge force disturbance, N)
C.dist.enable = true;

% Step disturbance used in Task 3 brief: -20N at t=10s (surge channel)
C.dist.surge_step_time_s = 10;      % seconds
C.dist.surge_step_N      = +20;     % Newtons (opposes motion)

% --- Compatibility aliases for runningchecks scripts ---
C.dist.t_step_s = C.dist.surge_step_time_s;
C.dist.step_N   = C.dist.surge_step_N;


% Default (no disturbance)
C.dist.surge_bias_N = 0;            % Newtons

%% State indices (single source of truth)
C.idx.u     = 1;
C.idx.w     = 2;
C.idx.q     = 3;
C.idx.xp    = 4;
C.idx.zp    = 5;
C.idx.theta = 6;


end
