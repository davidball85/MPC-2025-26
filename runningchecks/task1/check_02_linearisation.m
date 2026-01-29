% check_02_linearisation.m

addpath(genpath(pwd))

C = config_constants();

[Ac, Bc, Cc, Dc] = auv_linearise(C);

disp('Ac ='); disp(Ac)
disp('Bc ='); disp(Bc)

fprintf('\nKey expected properties:\n');
fprintf('  Ac(1,1) < 0   (surge damping)\n');
fprintf('  Ac(3,6) < 0   (pitch restoring)\n');
fprintf('  Ac(4,1) = 1   (xp_dot = u)\n');
fprintf('  Ac(5,2) = 1   (zp_dot = w)\n');
