function R = check_04_independent_qp_match()
% check_04_independent_qp_match
%
% Independent verification of MPC:
%   - Builds condensed QP matching mpc_standard_build
%   - Solves with quadprog
%   - Compares first control move against YALMIP optimizer
%
% This version is robust to newer APIs where mpc_standard_solve returns
% err as a struct (err.ok) rather than a numeric code.

R = struct('pass', false, 'notes', {{}}, 'error', '');

try
    C = config_constants();
    T = config_tuning();

    x_ref = T.x_ref_task2(:);

    wall_stop = C.wall_stop;
    wall_stop = local_unwrap_numeric(wall_stop);

    [Ac,Bc,Cc,Dc] = auv_linearise(C);
    [Ad,Bd,~,~]   = auv_discretise(Ac,Bc,Cc,Dc,C.Ts);

    nx = size(Ad,1);
    nu = size(Bd,2);
    N  = T.N;

    u_ref = auv_equilibrium(x_ref, C);

    x0 = zeros(nx,1);
    x0(5) = C.zp_min;
    x0_tilde = x0 - x_ref;

    R.notes{end+1} = sprintf('Independent QP check: nx=%d, nu=%d, N=%d.', nx, nu, N);
    R.notes{end+1} = sprintf('Sampling time Ts = %.3f s.', C.Ts);

    % -------------------------------------------------------------
    % Build condensed prediction matrices (same structure as before)
    % -------------------------------------------------------------
    nX = nx*(N+1);
    nU = nu*N;

    Abar = zeros(nX,nx);
    Bbar = zeros(nX,nU);

    Abar(1:nx,:) = eye(nx);

    for k = 1:N
        Ak = Ad^k;
        Abar(k*nx+(1:nx),:) = Ak;

        for j = 1:k
            A_kj = Ad^(k-j);
            Bbar(k*nx+(1:nx), (j-1)*nu+(1:nu)) = A_kj*Bd;
        end
    end

    % -------------------------------------------------------------
    % Quadratic cost
    % -------------------------------------------------------------
    Q = T.Q;
    Rw = T.R;

    Qbar = kron(eye(N+1), Q);
    Rbar = kron(eye(N),   Rw);

    % Cost in delta-u (du)
    H = Bbar.'*Qbar*Bbar + Rbar;
    f = Bbar.'*Qbar*(Abar*x0_tilde);

    H = 0.5*(H+H');  % symmetrise

    % -------------------------------------------------------------
    % Input constraints (in delta-u space)
    % -------------------------------------------------------------
    lb = repmat(C.u_min-u_ref, N, 1);
    ub = repmat(C.u_max-u_ref, N, 1);

    % -------------------------------------------------------------
    % State constraints (xp wall, depth bounds, pitch bounds)
    % -------------------------------------------------------------
    idx_xp = 4; idx_zp = 5; idx_th = 6;

    G=[]; h=[];

    for k = 1:N+1
        x_ref_k = x_ref;
        x_ref_k(idx_xp) = x_ref(idx_xp) + k*C.Ts*x_ref(1);

        row=zeros(1,nX); row((k-1)*nx+idx_xp)=1;
        G=[G;row]; h=[h; wall_stop - x_ref_k(idx_xp)];

        row=zeros(1,nX); row((k-1)*nx+idx_zp)=1;
        G=[G;row]; h=[h; C.zp_max - x_ref_k(idx_zp)];

        row=zeros(1,nX); row((k-1)*nx+idx_zp)=-1;
        G=[G;row]; h=[h; -(C.zp_min - x_ref_k(idx_zp))];

        row=zeros(1,nX); row((k-1)*nx+idx_th)=1;
        G=[G;row]; h=[h; C.theta_max];

        row=zeros(1,nX); row((k-1)*nx+idx_th)=-1;
        G=[G;row]; h=[h; -C.theta_min];   % <-- this was often wrong in older check files
    end

    Aineq = G*Bbar;
    bineq = h - G*(Abar*x0_tilde);

    % -------------------------------------------------------------
    % Solve QP
    % -------------------------------------------------------------
    [Ustar,~,exitflag] = quadprog(H,f,Aineq,bineq,[],[],lb,ub,[], ...
        optimoptions('quadprog','Display','off'));

    if exitflag <= 0
        error('Independent quadprog failed (exitflag=%d).', exitflag);
    end

    du0_qp = Ustar(1:nu);
    u0_qp  = du0_qp + u_ref;

    % -------------------------------------------------------------
    % YALMIP comparison (robust to err struct)
    % -------------------------------------------------------------
    M = mpc_standard_build(Ad,Bd,x_ref,u_ref,C,T);

    [u0_y,~,err] = mpc_standard_solve(M,x0,x_ref,u_ref);

    if isstruct(err)
        if ~isfield(err,'ok') || ~logical(err.ok)
            msg = 'YALMIP MPC failed (err.ok=false).';
            if isfield(err,'yalmip_info') && ~isempty(err.yalmip_info)
                msg = sprintf('%s Info: %s', msg, err.yalmip_info);
            end
            error(msg);
        end
    else
        % legacy behaviour: numeric err code
        if err ~= 0
            error('YALMIP MPC returned err=%d.', err);
        end
    end

    diff = norm(u0_qp - u0_y, 2);
    tol  = 1e-4;

    if diff > tol
        error('First move mismatch: ||u_qp-u_y|| = %.3e.', diff);
    end

    R.notes{end+1} = sprintf('First move match confirmed: ||u_qp-u_y|| = %.3e.', diff);
    R.notes{end+1} = sprintf('u0_qp = %s.', mat2str(u0_qp,6));
    R.notes{end+1} = sprintf('u0_y  = %s.', mat2str(u0_y,6));

    R.pass = true;

catch ME
    R.pass = false;
    R.error = ME.message;
end

end

% ---------------------------------------------------------
function v = local_unwrap_numeric(v)
% If something is a struct wrapper, peel it until numeric.
% This avoids breaking configs; we just adapt at the boundary.

guard = 0;
while isstruct(v) && guard < 10
    fn = fieldnames(v);
    if numel(fn) == 1
        v = v.(fn{1});
    else
        % common patterns
        if isfield(v,'value'), v = v.value; break; end
        if isfield(v,'val'),   v = v.val;   break; end
        % can't safely unwrap
        break;
    end
    guard = guard + 1;
end
end
