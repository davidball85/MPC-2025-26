function R = check_04_independent_qp_match()
% check_04_independent_qp_match
%
% Independent verification of MPC:
%   - Builds condensed QP matching mpc_standard_build
%   - Solves with quadprog
%   - Compares first control move against YALMIP optimizer
%
% Outputs detailed diagnostics suitable for reporting.

R = struct('pass', false, 'notes', {{}}, 'error', '');

try
    C = config_constants();
    T = config_tuning();

    x_ref = T.x_ref_task2(:);
    wall_stop = C.wall_stop;

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

    % --- Condensed prediction ---
    nX = nx*(N+1);
    nU = nu*N;

    Abar = zeros(nX,nx);
    Bbar = zeros(nX,nU);

    for k = 1:N+1
        Abar((k-1)*nx+1:k*nx,:) = Ad^(k-1);
        for j = 1:min(k-1,N)
            Bbar((k-1)*nx+1:k*nx,(j-1)*nu+1:j*nu) = Ad^(k-1-j)*Bd;
        end
    end

    if T.useTerminalCost
        P = T.Q;
    else
        P = zeros(nx);
    end

    QX = blkdiag(kron(eye(N),T.Q),P);
    RU = kron(eye(N),T.R);

    H = 2*(Bbar'*QX*Bbar + RU);
    f = 2*(Bbar'*QX*(Abar*x0_tilde));
    H = 0.5*(H+H');

    lb = repmat(C.u_min-u_ref,N,1);
    ub = repmat(C.u_max-u_ref,N,1);

    % --- State constraints ---
    idx_xp = 4; idx_zp = 5; idx_th = 6;
    G=[]; h=[];

    for k=1:N+1
        x_ref_k = x_ref;
        x_ref_k(idx_xp) = x_ref(idx_xp) + k*C.Ts*x_ref(1);

        row=zeros(1,nX); row((k-1)*nx+idx_xp)=1;
        G=[G;row]; h=[h;wall_stop-x_ref_k(idx_xp)];

        row=zeros(1,nX); row((k-1)*nx+idx_zp)=1;
        G=[G;row]; h=[h;C.zp_max-x_ref_k(idx_zp)];

        row=zeros(1,nX); row((k-1)*nx+idx_zp)=-1;
        G=[G;row]; h=[h;-(C.zp_min-x_ref_k(idx_zp))];

        row=zeros(1,nX); row((k-1)*nx+idx_th)=1;
        G=[G;row]; h=[h;C.theta_max];

        row=zeros(1,nX); row((k-1)*nx+idx_th)=-1;
        G=[G;row]; h=[h;C.theta_max];
    end

    Aineq = G*Bbar;
    bineq = h - G*(Abar*x0_tilde);

    % --- Solve QP ---
    [Ustar,~,exitflag] = quadprog(H,f,Aineq,bineq,[],[],lb,ub,[], ...
        optimoptions('quadprog','Display','off'));

    if exitflag <= 0
        error('Independent quadprog failed (exitflag=%d).', exitflag);
    end

    du0_qp = Ustar(1:nu);
    u0_qp  = du0_qp + u_ref;

    % --- YALMIP comparison ---
    M = mpc_standard_build(Ad,Bd,x_ref,u_ref,C,T);
    [u0_y,~,err] = mpc_standard_solve(M,x0,x_ref,u_ref);

    if err~=0
        error('YALMIP MPC returned err=%d.', err);
    end

    diff = norm(u0_qp-u0_y,2);
    tol  = 1e-4;

    if diff > tol
        error('First move mismatch: ||u_qp-u_y|| = %.3e.', diff);
    end

    R.notes{end+1} = sprintf('First move match confirmed: ||u_qp-u_y|| = %.3e.', diff);
    R.notes{end+1} = sprintf('u0_qp = %s.', mat2str(u0_qp,6));
    R.notes{end+1} = sprintf('u0_yalmip = %s.', mat2str(u0_y,6));

    R.pass = true;

catch ME
    R.pass = false;
    R.error = ME.message;
end
end