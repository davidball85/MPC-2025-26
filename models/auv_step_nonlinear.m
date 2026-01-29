function x_next = auv_step_nonlinear(x, u_ctrl, C)
% auv_step_nonlinear
% One-step RK4 integrator for the nonlinear AUV plant.
% Uses sampling time C.Ts.

Ts = C.Ts;

f = @(xx, uu) auv_dynamics_nonlinear(xx, uu, C);

k1 = f(x, u_ctrl);
k2 = f(x + 0.5*Ts*k1, u_ctrl);
k3 = f(x + 0.5*Ts*k2, u_ctrl);
k4 = f(x + Ts*k3, u_ctrl);

x_next = x + (Ts/6)*(k1 + 2*k2 + 2*k3 + k4);
end
