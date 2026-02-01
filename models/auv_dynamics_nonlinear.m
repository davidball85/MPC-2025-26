function xdot = auv_dynamics_nonlinear(x, u_ctrl, C)
% auv_dynamics_nonlinear
%
% State:  x = [u; w; q; xp; zp; theta]
% Input:  u_ctrl = [Tsurge; Theave; taupitch]
%
% DISTURBANCE CONVENTION (Task 3/4):
%   C.d_surge [N] is an external surge force added to the surge force balance.
%   Therefore:
%       d_surge = -20 N  --> opposes forward motion (push-back)
%       d_surge = +20 N  --> assists forward motion (push-forward)

% --- Unpack states ---
u     = x(1);
w     = x(2);
q     = x(3);
theta = x(6);

% --- Unpack inputs ---
Tsurge   = u_ctrl(1);
Theave   = u_ctrl(2);
taupitch = u_ctrl(3);

% --- Params ---
mx    = C.mx;
mz    = C.mz;
Iy    = C.Iy;
Du    = C.Du;
Dw    = C.Dw;
Dq    = C.Dq;
Mrest = C.Mrest;

% --- Disturbance ---
d_surge = 0;
if isfield(C,'d_surge')
    d_surge = C.d_surge;
end

% --- Dynamics ---
udot = (1/mx) * (Tsurge + d_surge - Du*u*abs(u));
wdot = (1/mz) * (Theave          - Dw*w*abs(w));
qdot = (1/Iy) * (taupitch        - Dq*q*abs(q) - Mrest*sin(theta));

xpdot    = u*cos(theta) + w*sin(theta);
zpdot    = -u*sin(theta) + w*cos(theta);
thetadot = q;

xdot = [udot; wdot; qdot; xpdot; zpdot; thetadot];
end
