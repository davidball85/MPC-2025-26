function xdot = auv_dynamics_nonlinear(x, u_ctrl, C)
% auv_dynamics_nonlinear
% Implements the nonlinear AUV model from the coursework brief:
% x = [u; w; q; xp; zp; theta]
% u_ctrl = [Tsurge; Theave; taupitch]
%
% Equations (1)-(6) in the brief: :contentReference[oaicite:2]{index=2}

% --- Unpack states ---
u     = x(1);   % surge velocity (body) [m/s]
w     = x(2);   % heave velocity (body) [m/s]
q     = x(3);   % pitch rate [rad/s]
xp    = x(4);   %#ok<NASGU> % inertial x position [m]
zp    = x(5);   %#ok<NASGU> % depth [m]
theta = x(6);   % pitch angle [rad]

% --- Unpack inputs ---
Tsurge   = u_ctrl(1);   % [N]
Theave   = u_ctrl(2);   % [N]
taupitch = u_ctrl(3);   % [Nm]

% --- Shorthand params ---
mx    = C.mx;
mz    = C.mz;
Iy    = C.Iy;
Du    = C.Du;
Dw    = C.Dw;
Dq    = C.Dq;
Mrest = C.Mrest;

% --- Optional input disturbance (Task 3) ---
d_surge = 0;
if isfield(C,'d_surge')
    d_surge = C.d_surge;   % surge disturbance force [N]
end

% --- DEBUG: disturbance sign check (temporary) ---
%fprintf('d=%.1f  udot_contrib=%.3f\n', d_surge, d_surge/mx);

% Surge acceleration
udot = (1/mx) * (Tsurge - d_surge - Du*u*abs(u));



% --- Nonlinear dynamics (brief Eq. 1-6) ---
udot = (1/mx) * (Tsurge + d_surge - Du * u * abs(u));               % (1)
wdot = (1/mz) * (Theave - Dw * w * abs(w));                         % (2)
qdot = (1/Iy) * (taupitch - Dq * q * abs(q) - Mrest * sin(theta));  % (3)

xpdot = u * cos(theta) + w * sin(theta);                            % (4)
zpdot = -u * sin(theta) + w * cos(theta);                           % (5)
thetadot = q;                                                       % (6)

% --- Pack derivative ---
xdot = [udot; wdot; qdot; xpdot; zpdot; thetadot];
end
