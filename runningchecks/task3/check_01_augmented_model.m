function R = check_01_augmented_model()
% check_01_augmented_model
%
% Task 3 - Check 01
%   - Builds discrete linear model (A,B,C) from Task 1 pipeline
%   - Builds Chapter-8 augmented model with constant disturbance state
%   - Prints dimension checks + observability checks
%   - Saves report-ready plots

R = struct('pass', false, 'notes', {{}}, 'error', '');

try
    Ccfg = config_constants();
    Tcfg = config_tuning(); %#ok<NASGU>

    % ---- Continuous -> discrete
    [Ac, Bc, Cmeas, Dc] = auv_linearise(Ccfg);
    [A, B, Cy] = local_discretise(Ac, Bc, Cmeas, Dc, Ccfg);

    nx = size(A,1); nu = size(B,2); ny = size(Cy,1);

    % ---- Augment (Chapter 8)
    [A_aug, B_aug, C_aug, Bw] = auv_build_augmented_model(A, B, Cy);

    nx_aug = size(A_aug,1);

    % ---- Notes
    R.notes{end+1} = sprintf('Discrete model: nx=%d, nu=%d, ny=%d', nx, nu, ny);
    R.notes{end+1} = sprintf('Augmented model: nx_aug=%d (= nx + 1 disturbance state)', nx_aug);
    R.notes{end+1} = 'Disturbance model: constant input disturbance d(k+1)=d(k), entering surge channel via Bw = B(:,1).';

    % ---- Sanity checks
    assert(isequal(size(Bw), [nx,1]), 'Bw must be [nx x 1].');
    assert(isequal(size(B_aug), [nx_aug, nu]), 'B_aug has wrong dimensions.');
    assert(isequal(size(C_aug), [ny, nx_aug]), 'C_aug has wrong dimensions.');

    % ---- Observability
    Ob  = obsv(A, Cy);
    Oba = obsv(A_aug, C_aug);

    rOb  = rank(Ob);
    rOba = rank(Oba);

    R.notes{end+1} = sprintf('rank(obsv(A,C))         = %d (max %d)', rOb, nx);
    R.notes{end+1} = sprintf('rank(obsv(A_aug,C_aug))  = %d (max %d)', rOba, nx_aug);

    % ---- Plots
    out_dir = fileparts(mfilename('fullpath'));

    figure('Name','Task3 Check01: Eigenvalues','NumberTitle','off');
    subplot(1,2,1);
    eigA = eig(A);
    plot(real(eigA), imag(eigA), 'x'); grid on;
    xlabel('Real'); ylabel('Imag');
    title('Eigenvalues of A');

    subplot(1,2,2);
    eigAa = eig(A_aug);
    plot(real(eigAa), imag(eigAa), 'x'); grid on;
    xlabel('Real'); ylabel('Imag');
    title('Eigenvalues of A_{aug}');

    f1 = fullfile(out_dir, 'task3_check01_eigs.png');
    saveas(gcf, f1);
    R.notes{end+1} = ['Saved plot: ', f1];

    figure('Name','Task3 Check01: Observability singular values','NumberTitle','off');
    s1 = svd(Ob);
    s2 = svd(Oba);
    semilogy(s1, 'o-'); hold on;
    semilogy(s2, 's-'); grid on;
    xlabel('Index'); ylabel('Singular value (log scale)');
    title('Observability matrix singular values');
    legend('O(A,C)','O(A_{aug},C_{aug})','Location','best');

    f2 = fullfile(out_dir, 'task3_check01_obsv_svd.png');
    saveas(gcf, f2);
    R.notes{end+1} = ['Saved plot: ', f2];

    % ---- Pass criteria
    if rOba < nx_aug
        R.notes{end+1} = 'FAIL: Augmented observability is not full rank (KF may be weak/unreliable).';
        R.pass = false;
    else
        R.notes{end+1} = 'PASS: Augmented pair is full-rank observable.';
        R.pass = true;
    end

catch ME
    R.pass = false;
    R.error = ME.message;
end

end

% =====================================================================
function [A, B, C] = local_discretise(Ac, Bc, Cc, Dc, Ccfg)
Ts = Ccfg.Ts;

try
    [A, B, C, ~] = auv_discretise(Ac, Bc, Cc, Dc, Ts);
    return
catch
end

try
    [A, B, C] = auv_discretise(Ac, Bc, Cc, Ts);
    return
catch
end

try
    [A, B, C] = auv_discretise(Ac, Bc, Cc);
    return
catch
end

sysd = c2d(ss(Ac, Bc, Cc, Dc), Ts, 'zoh');
A = sysd.A; B = sysd.B; C = sysd.C;
end
