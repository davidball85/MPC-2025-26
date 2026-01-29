function [Ac, Bc, Cc, Dc] = auv_linearise(C)
% auv_linearise
% Computes continuous-time linearised model:
%   x_dot = Ac x + Bc u
% about the inspection cruise equilibrium defined in the coursework brief.
%
% State:  x = [u; w; q; xp; zp; theta]
% Input:  u = [Tsurge; Theave; taupitch]

% Equilibrium state and input
x_eq = C.x_eq;
u_eq = auv_equilibrium(x_eq, C); %#ok<NASGU>

u     = x_eq(1);
w     = x_eq(2);
q     = x_eq(3);
theta = x_eq(6);

mx = C.mx;
mz = C.mz;
Iy = C.Iy;

Du = C.Du;
Dw = C.Dw;
Dq = C.Dq;
Mrest = C.Mrest;

%% Initialise Jacobians
Ac = zeros(6,6);
Bc = zeros(6,3);

%% --------- State Jacobian Ac ---------

% u_dot = (1/mx)(Tsurge - Du*u|u|)
Ac(1,1) = -(2*Du*abs(u))/mx;

% w_dot = (1/mz)(Theave - Dw*w|w|)
Ac(2,2) = -(2*Dw*abs(w))/mz;

% q_dot = (1/Iy)(taupitch - Dq*q|q| - Mrest*sin(theta))
Ac(3,3) = -(2*Dq*abs(q))/Iy;
Ac(3,6) = -(Mrest*cos(theta))/Iy;

% xp_dot = u*cos(theta) + w*sin(theta)
Ac(4,1) =  cos(theta);
Ac(4,2) =  sin(theta);
Ac(4,6) = -u*sin(theta) + w*cos(theta);

% zp_dot = -u*sin(theta) + w*cos(theta)
Ac(5,1) = -sin(theta);
Ac(5,2) =  cos(theta);
Ac(5,6) = -u*cos(theta) - w*sin(theta);

% theta_dot = q
Ac(6,3) = 1;

%% --------- Input Jacobian Bc ---------

% u_dot wrt Tsurge
Bc(1,1) = 1/mx;

% w_dot wrt Theave
Bc(2,2) = 1/mz;

% q_dot wrt taupitch
Bc(3,3) = 1/Iy;

%% --------- Output matrices (measurement model) ---------
% Measured outputs: y = [xp; zp]

Cc = zeros(2,6);
Cc(1,4) = 1;   % xp
Cc(2,5) = 1;   % zp

Dc = zeros(2,3);
end


function [A_aug, B_aug, C_aug, Bw] = auv_build_augmented_model(A, B, C)
% auv_build_augmented_model
% Chapter 8 style augmentation for constant input disturbance:
%   x_{k+1} = A x_k + B u_k + Bw d_k
%   d_{k+1} = d_k
%
% Here the disturbance is a surge FORCE (N), entering the same channel as Tsurge.
% Therefore Bw is the first column of B (discrete-time).

nx = size(A,1);

Bw = B(:,1);                 % disturbance enters surge channel only
A_aug = [A, Bw;
         zeros(1,nx), 1];

B_aug = [B;
         zeros(1,size(B,2))];

C_aug = [C, zeros(size(C,1),1)];
end
