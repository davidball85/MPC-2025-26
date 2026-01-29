% Check what Ad does to xp
% This script analyzes if the xp ramp term is needed

% At equilibrium: theta=0, so Ac(4,1) = cos(0) = 1
% This means: xp_dot = 1 * u + 0 * w + ...
% In continuous time: d(xp)/dt = u

% After discretization with Ts:
% xp(k+1) ≈ xp(k) + Ts * u(k)

% In matrix form with Ad, the (4,1) element should be:
% Ad(4,1) ≈ Ts (for small Ts)

% Let's think about what happens in deviation coordinates:
% xp_tilde = xp - xp_ref, where xp_ref = 0 (constant)
% u_tilde = u - u_ref, where u_ref = 1 m/s

% The dynamics: xp_dot = u
% In deviations: d(xp_tilde)/dt = d(xp)/dt - d(xp_ref)/dt = u - 0 = u
% But u = u_tilde + u_ref, so:
% d(xp_tilde)/dt = u_tilde + u_ref

% This means the TRUE deviation dynamics should be:
% xp_tilde(k+1) = xp_tilde(k) + Ts * (u_tilde(k) + u_ref)

% However, the linearization around (u=u_ref, xp=xp_ref) gives:
% delta(xp_dot) = (partial xp_dot / partial u) * delta(u) + (partial xp_dot / partial xp) * delta(xp)
% delta(xp_dot) = 1 * delta(u) + 0 * delta(xp)
% delta(xp_dot) = delta(u) = u_tilde

% So Ad only captures: xp_tilde(k+1) = xp_tilde(k) + Ts * u_tilde(k)
% But it's MISSING the u_ref term!

% The xp ramp term adds: x_next(4) = x_next(4) + Ts * x_ref(1)
% This adds back the missing u_ref term!

% So the total is:
% xp_tilde(k+1) = xp_tilde(k) + Ts * u_tilde(k)  [from Ad]
%                              + Ts * u_ref        [from ramp term]
%               = xp_tilde(k) + Ts * (u_tilde(k) + u_ref)

% This is CORRECT!

fprintf('Analysis: The xp ramp term is NEEDED and CORRECT.\n');
fprintf('It compensates for the fact that linearization misses the reference velocity.\n');
fprintf('\nBut this creates a mismatch with standard LTI MPC formulations...\n');
