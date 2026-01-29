function [Ad, Bd, Cd, Dd] = auv_discretise(Ac, Bc, Cc, Dc, Ts)
% auv_discretise
% Zero-order hold discretisation of continuous-time state-space model
%
%   x_dot = Ac x + Bc u
%   y     = Cc x + Dc u
%
% Returns:
%   x_{k+1} = Ad x_k + Bd u_k
%   y_k     = Cd x_k + Dd u_k

sys_c = ss(Ac, Bc, Cc, Dc);
sys_d = c2d(sys_c, Ts, 'zoh');

Ad = sys_d.A;
Bd = sys_d.B;
Cd = sys_d.C;
Dd = sys_d.D;
end
