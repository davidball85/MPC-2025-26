% check_04_controllability.m
fprintf('\n[CHECK 04] Controllability (discrete-time)\n');

C = config_constants();

% Continuous -> discrete
[Ac, Bc, Cc, Dc] = auv_linearise(C);
[Ad, Bd, Cd, Dd] = auv_discretise(Ac, Bc, Cc, Dc, C.Ts);

n = size(Ad,1);

% Build controllability matrix
Ctrb = [];
Ak = eye(n);
for i = 1:n
    Ctrb = [Ctrb, Ak*Bd]; %#ok<AGROW>
    Ak = Ad*Ak;
end

r = rank(Ctrb);
fprintf('  rank(Ctrb) = %d (n = %d)\n', r, n);

% A loose numeric conditioning indicator
s = svd(Ctrb);
fprintf('  sigma_max = %.3e, sigma_min = %.3e\n', max(s), min(s));

if r == n
    fprintf('[CHECK 04] PASS: System is controllable (full rank).\n');
else
    fprintf('[CHECK 04] FAIL: System is NOT fully controllable.\n');
    fprintf('  This would limit achievable tracking/constraint handling.\n');
end
