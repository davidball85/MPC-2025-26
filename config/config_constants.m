function C = config_constants()
% CONFIG_CONSTANTS - Fixed constants from coursework brief (MMH626810)
%
% This is the single source of truth for all physical parameters,
% constraints, and configuration settings.

%% ====================================================================
%%  SAMPLING TIME
%% ====================================================================
C.Ts = 0.1;   % [s] Discrete time step

%% ====================================================================
%%  PHYSICAL PARAMETERS (from coursework brief Section 2)
%% ====================================================================
C.mx = 100;      % [kg] Mass in surge direction (including added mass)
C.mz = 120;      % [kg] Mass in heave direction (including added mass)
C.Iy = 15;       % [kg·m²] Moment of inertia in pitch

C.Du = 50;       % [N·s²/m²] Drag coefficient in surge
C.Dw = 80;       % [N·s²/m²] Drag coefficient in heave
C.Dq = 15;       % [N·m·s²/rad²] Drag coefficient in pitch

C.Mrest = 30;    % [N·m] Metacentric restoring moment coefficient

%% ====================================================================
%%  INPUT CONSTRAINTS (Actuator Saturation) - Section 2
%% ====================================================================
C.u_min = [-200; -200; -30];  % [Tsurge; Theave; tau_pitch] minimum [N; N; N·m]
C.u_max = [ 200;  200;  30];  % [Tsurge; Theave; tau_pitch] maximum [N; N; N·m]

%% ====================================================================
%%  STATE CONSTRAINTS (Safety Limits) - Section 2
%% ====================================================================
C.zp_min = 0.5;               % [m] Minimum depth (surface boundary)
C.zp_max = 10.0;              % [m] Maximum depth (floor boundary)

C.theta_min = deg2rad(-20);   % [rad] Minimum pitch angle
C.theta_max = deg2rad( 20);   % [rad] Maximum pitch angle

%% ====================================================================
%%  LINEARISATION OPERATING POINT - Task 1
%%  "Inspection Cruise": u=1 m/s, z=5m, all else zero
%% ====================================================================
C.x_eq = [1; 0; 0; 0; 5; 0];  % [u; w; q; xp; zp; theta]

%% ====================================================================
%%  TASK 2: VIRTUAL WALL (Obstacle Avoidance)
%% ====================================================================
C.wall_xp   = 50;      % [m] Physical wall location
C.wall_stop = 48;      % [m] Must stop (u=0) before this point

% Optional internal safety margin for enforcement
C.wall_stop_margin = 0;                 % [m]
C.wall_stop_internal = C.wall_stop - C.wall_stop_margin;

% Depth constraint enforcement delay (allow settling from surface start)
C.depth_enforce_from_k = 2;   % Enforce zp bounds from prediction step k>=2

%% ====================================================================
%%  TASK 3: DISTURBANCE SCENARIO
%%  Constant surge force disturbance (opposing motion)
%% ====================================================================

% Disturbance Configuration
C.dist = struct();
C.dist.enable = true;

C.dist.surge_step_time_s = 10.0;      % [s] Time when disturbance hits
C.dist.surge_step_N      = -20.0;     % [N] Resistive surge force
C.dist.surge_bias_N      = 0.0;       % [N] Initial bias (before step)

% Compatibility aliases for runningchecks scripts
C.dist.t_step_s = C.dist.surge_step_time_s;
C.dist.step_N   = C.dist.surge_step_N;

% Task 3 Specific Parameters
C.Task3 = struct();
C.Task3.initial_depth = 0.0;           % [m] Starting depth (configurable)
C.Task3.initial_speed = 0;           % [m/s] Starting speed
C.Task3.sim_time = 80.0;               % [s] Simulation duration

% Build initial state vector from parameters
C.Task3.initial_state = zeros(6,1);
C.Task3.initial_state(1) = C.Task3.initial_speed;  % u
C.Task3.initial_state(5) = C.Task3.initial_depth;  % zp

C.Task3.stop_at_wall_enable = true;   % set true to brake to u_ref = 0 near wall
C.Task3.wall_brake_margin_m = 8.0;     % start braking when xp >= wall_stop - margin


%% ====================================================================
%%  STATE INDICES (Single Source of Truth)
%%  Use these everywhere: Ccfg.idx.u, Ccfg.idx.zp, etc.
%% ====================================================================
C.idx = struct();
C.idx.u     = 1;  % Surge velocity
C.idx.w     = 2;  % Heave velocity
C.idx.q     = 3;  % Pitch rate
C.idx.xp    = 4;  % Position (range)
C.idx.zp    = 5;  % Depth
C.idx.theta = 6;  % Pitch angle

end
