% check_03_discretisation.m
% Strong validation of ZOH discretisation

fprintf('\n[CHECK 03] Discretisation (ZOH) validation\n');

C = config_constants();
[Ac, Bc, Cc, Dc] = auv_linearise(C);
[Ad, Bd, Cd, Dd] = auv_discretise(Ac, Bc, Cc, Dc, C.Ts);

% ---- 1) Basic sanity checks ----
assert(all(size(Ad) == [6 6]), 'Ad is not 6x6');
assert(all(size(Bd) == [6 3]), 'Bd is not 6x3');
assert(all(isfinite(Ad(:))), 'Ad contains NaN/Inf');
assert(all(isfinite(Bd(:))), 'Bd contains NaN/Inf');

fprintf('  PASS: Dimensions and finite values OK.\n');

% ---- 2) Analytical spot-checks ----
% Because Ac(1,1) = -1, the exact ZOH discretisation gives Ad(1,1) = exp(-Ts)
Ad11_expected = exp(-C.Ts);
err_Ad11 = abs(Ad(1,1) - Ad11_expected);

fprintf('  Spot-check: Ad(1,1) = %.6f, expected exp(-Ts)=%.6f, error=%.2e\n', ...
    Ad(1,1), Ad11_expected, err_Ad11);

assert(err_Ad11 < 1e-6, 'Ad(1,1) does not match exp(-Ts) to tolerance');
fprintf('  PASS: Ad(1,1) matches exp(-Ts).\n');

% Input channel 1 affects u-state; under ZOH with scalar a=-1, b=1/mx:
% Bd(1,1) should be integral_0^Ts exp(a*t) b dt = b*(1-exp(a*Ts))/(-a)
b_u = 1/C.mx;
Bd11_expected = b_u * (1 - exp(-C.Ts));  % since a=-1
err_Bd11 = abs(Bd(1,1) - Bd11_expected);

fprintf('  Spot-check: Bd(1,1) = %.6f, expected %.6f, error=%.2e\n', ...
    Bd(1,1), Bd11_expected, err_Bd11);

assert(err_Bd11 < 1e-6, 'Bd(1,1) does not match ZOH integral to tolerance');
fprintf('  PASS: Bd(1,1) matches ZOH integral.\n');

% ---- 3) Behavioural check vs continuous integration (ZOH input) ----
% Compare one-step discrete prediction to continuous-time integration with constant input.
x0 = zeros(6,1);
u0 = [10; -5; 2];  % arbitrary constant input for 0.1s

% Discrete prediction
x1d = Ad*x0 + Bd*u0;

% Continuous-time integration with matrix exponential (exact under ZOH for linear system)
% x(Ts) = expm(Ac*Ts)*x0 + integral_0^Ts expm(Ac*t) Bc dt * u0
% Use augmented matrix trick to compute both in one expm:
n = size(Ac,1);
m = size(Bc,2);
M = [Ac, Bc; zeros(m,n+m)];
Md = expm(M*C.Ts);
Ad_exact = Md(1:n, 1:n);
Bd_exact = Md(1:n, n+1:n+m);

x1c = Ad_exact*x0 + Bd_exact*u0;

err_step = norm(x1d - x1c, 2);

fprintf('  One-step behaviour check: ||x1d - x1c||_2 = %.3e\n', err_step);
assert(err_step < 1e-10, 'Discrete step does not match exact ZOH result');
fprintf('  PASS: Discrete one-step matches exact ZOH (matrix exponential) result.\n');

% ---- 4) Eigenvalue report ----
eigAd = eig(Ad);
fprintf('  Eigenvalues(Ad):\n');
disp(eigAd);

fprintf('[CHECK 03] PASS: Discretisation validated.\n');
