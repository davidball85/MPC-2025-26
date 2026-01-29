function u_eq = auv_equilibrium(x_eq, C)
% auv_equilibrium
% Computes equilibrium input u_eq = [Tsurge; Theave; taupitch]
% for a given equilibrium state x_eq, using the nonlinear model in the brief.
%
% Model equations (brief):
%   u_dot = (1/mx) (Tsurge - Du*u*|u|)
%   w_dot = (1/mz) (Theave - Dw*w*|w|)
%   q_dot = (1/Iy) (taupitch - Dq*q*|q| - Mrest*sin(theta))
%
% At equilibrium for inspection cruise, we set:
%   u_dot = 0, w_dot = 0, q_dot = 0
% and solve for the required inputs.

% Unpack equilibrium state
u     = x_eq(1);
w     = x_eq(2);
q     = x_eq(3);
theta = x_eq(6);

% Solve equilibrium conditions:
% 0 = Tsurge - Du*u*|u|
Tsurge_eq = C.Du * u * abs(u);

% 0 = Theave - Dw*w*|w|
Theave_eq = C.Dw * w * abs(w);

% 0 = taupitch - Dq*q*|q| - Mrest*sin(theta)
taupitch_eq = C.Dq * q * abs(q) + C.Mrest * sin(theta);

% Pack equilibrium input
u_eq = [Tsurge_eq; Theave_eq; taupitch_eq];
end
