% check_05_observability.m
fprintf('\n[CHECK 05] Observability (discrete-time, outputs y=[xp; zp])\n');

C = config_constants();

% Continuous -> discrete
[Ac, Bc, Cc, Dc] = auv_linearise(C);
[Ad, Bd, Cd, Dd] = auv_discretise(Ac, Bc, Cc, Dc, C.Ts);

n = size(Ad,1);

% Build observability matrix
Obsv = [];
Ak = eye(n);
for i = 1:n
    Obsv = [Obsv; Cd*Ak]; %#ok<AGROW>
    Ak = Ad*Ak;
end

r = rank(Obsv);
fprintf('  rank(Obsv) = %d (n = %d)\n', r, n);

s = svd(Obsv);
fprintf('  sigma_max = %.3e, sigma_min = %.3e\n', max(s), min(s));

if r == n
    fprintf('[CHECK 05] PASS: System is observable from [xp, zp].\n');
else
    fprintf('[CHECK 05] NOTE: System is NOT fully observable from [xp, zp].\n');
    fprintf('  This motivates adding an observer/Kalman filter (Task 3).\n');
end
