function cons = mpc_constraints(cons, xk, uk, C, k)
% mpc_constraints
%
% Centralised hard constraints (config-driven).
% xk: absolute state at prediction step k
% uk: absolute input at prediction step k (pass [] to skip input constraints)
% k : prediction step index (1..N+1). If omitted, defaults to 1.

if nargin < 5 || isempty(k)
    k = 1;
end
if nargin < 3
    uk = [];
end

% --- Input saturation (only if uk provided) ---
if ~isempty(uk)
    cons = [cons, C.u_min <= uk <= C.u_max];
end

% --- Pitch constraint (always enforced) ---
cons = [cons, -C.theta_max <= xk(6) <= C.theta_max];

% --- Depth constraint (optionally delayed for initial recovery) ---
% Prefer config name depth_enforce_from_k, but allow aliases if present.
if isfield(C,'depth_enforce_from_k')
    kDepth = C.depth_enforce_from_k;
elseif isfield(C,'enforce_depth_from_step')
    kDepth = C.enforce_depth_from_step;
else
    kDepth = 1;
end

if k >= kDepth
    cons = [cons, C.zp_min <= xk(5) <= C.zp_max];
end

% --- Virtual wall braking constraint (config-driven) ---
if isfield(C,'wall_stop_internal')
    wall_stop = C.wall_stop_internal;
else
    wall_stop = C.wall_stop;
end
cons = [cons, xk(4) <= wall_stop];

end
