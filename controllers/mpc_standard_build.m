function M = mpc_standard_build(varargin)
% mpc_standard_build
%
% Builds a standard (non-offset-free) MPC controller as a YALMIP optimizer.
%
% Backward-compatible call patterns supported:
%
%   (A) Legacy (used in some Task 2/4 checks):
%       M = mpc_standard_build(Ad, Bd, x_ref, u_ref, C, T)
%
%   (B) Struct style (used by Task 3 logger-style scripts):
%       M = mpc_standard_build(mdl, x_ref, u_ref, C, T)
%       where mdl has fields A,B (and optionally Ts)
%
%   (C) Minimal struct style:
%       M = mpc_standard_build(mdl, C, T)
%       In this case x_ref/u_ref are taken from config if available:
%         x_ref = [u_ref; ... ; xp_ref; zp_ref; theta_ref] consistent with C.idx
%         u_ref = equilibrium input from auv_equilibrium(C)
%
% Notes:
% - All tuning/constraints must come from C and T (no hard-coded limits).
% - This function intentionally avoids printing to the command window.
%
% Requirements:
% - YALMIP on path
% - mpc_constraints(cons, x_abs, u_abs, C, k) available

% -----------------------------
% Parse inputs (backward compatible)
% -----------------------------
narginchk(3, 6);

Ad = []; Bd = []; x_ref = []; u_ref = []; C = []; T = [];

if nargin == 6
    % (Ad, Bd, x_ref, u_ref, C, T)
    Ad    = varargin{1};
    Bd    = varargin{2};
    x_ref = varargin{3};
    u_ref = varargin{4};
    C     = varargin{5};
    T     = varargin{6};

elseif nargin == 5 && isstruct(varargin{1})
    % (mdl, x_ref, u_ref, C, T)
    mdl   = varargin{1};
    Ad    = mdl.A;
    Bd    = mdl.B;
    x_ref = varargin{2};
    u_ref = varargin{3};
    C     = varargin{4};
    T     = varargin{5};

elseif nargin == 3 && isstruct(varargin{1})
    % (mdl, C, T) -> derive x_ref/u_ref from config if possible
    mdl = varargin{1};
    Ad  = mdl.A;
    Bd  = mdl.B;
    C   = varargin{2};
    T   = varargin{3};

    % Derive u_ref from equilibrium if possible
    if exist('auv_equilibrium','file') == 2
        try
            [~, u_eq] = auv_equilibrium(C);
            u_ref = u_eq;
        catch
            u_ref = zeros(size(Bd,2),1);
        end
    else
        u_ref = zeros(size(Bd,2),1);
    end

    % Derive x_ref from config Task3 refs if available
    nx = size(Ad,1);
    x_ref = zeros(nx,1);

    if isfield(C,'idx')
        % speed reference
        if isfield(C,'Task3') && isfield(C.Task3,'reference_speed') && isfield(C.idx,'u')
            x_ref(C.idx.u) = C.Task3.reference_speed;
        end
        % depth reference
        if isfield(C,'Task3') && isfield(C.Task3,'reference_depth') && isfield(C.idx,'zp')
            x_ref(C.idx.zp) = C.Task3.reference_depth;
        end
        % xp reference at current time assumed 0 unless provided by caller
        if isfield(C.idx,'xp')
            x_ref(C.idx.xp) = 0;
        end
        % theta reference 0 if available
        if isfield(C.idx,'theta')
            x_ref(C.idx.theta) = 0;
        end
    end

else
    error('mpc_standard_build:BadInputs', ...
        'Unsupported call signature. Expected (Ad,Bd,x_ref,u_ref,C,T) or (mdl,...)');
end

% -----------------------------
% Basic dimensions
% -----------------------------
nx = size(Ad,1);
nu = size(Bd,2);

% Horizon and options from tuning config
N = T.N;

% -----------------------------
% Decision variables (deviation form)
% -----------------------------
x_tilde  = sdpvar(nx, N+1);
u_tilde  = sdpvar(nu, N);
x0_tilde = sdpvar(nx, 1);

obj  = 0;
cons = [];

cons = [cons, x_tilde(:,1) == x0_tilde];

% Terminal cost option
if isfield(T,'useTerminalCost') && T.useTerminalCost
    P = T.Q;
else
    P = zeros(nx);
end

% Indices (avoid magic numbers if C.idx exists)
idx_u  = 1;
idx_xp = 4;
if isfield(C,'idx')
    if isfield(C.idx,'u');  idx_u  = C.idx.u;  end
    if isfield(C.idx,'xp'); idx_xp = C.idx.xp; end
end

% -----------------------------
% Build horizon dynamics + constraints
% -----------------------------
for k = 1:N
    % deviation dynamics
    x_next = Ad*x_tilde(:,k) + Bd*u_tilde(:,k);
    cons   = [cons, x_tilde(:,k+1) == x_next];

    % absolute variables with time-varying xp reference
    x_ref_k = x_ref;
    x_ref_k(idx_xp) = x_ref(idx_xp) + k * C.Ts * x_ref(idx_u);

    x_ref_kp1 = x_ref;
    x_ref_kp1(idx_xp) = x_ref(idx_xp) + (k+1) * C.Ts * x_ref(idx_u);

    x_abs      = x_tilde(:,k)   + x_ref_k;
    x_abs_next = x_tilde(:,k+1) + x_ref_kp1;
    u_abs      = u_tilde(:,k)   + u_ref;

    % quadratic cost
    obj = obj + x_tilde(:,k)'*T.Q*x_tilde(:,k) + u_tilde(:,k)'*T.R*u_tilde(:,k);

    % constraints at step k, and next state constraints at k+1
    cons = mpc_constraints(cons, x_abs,      u_abs, C, k);
    cons = mpc_constraints(cons, x_abs_next, [],    C, k+1);
end

obj = obj + x_tilde(:,N+1)'*P*x_tilde(:,N+1);

% Solver options
solverName = T.solver;
verboseLvl = 0;
if isfield(T,'verbose'); verboseLvl = T.verbose; end
opts = sdpsettings('solver', solverName, 'verbose', verboseLvl);

% Build optimizer returning first deviation control move
M = optimizer(cons, obj, opts, x0_tilde, u_tilde(:,1));

end
