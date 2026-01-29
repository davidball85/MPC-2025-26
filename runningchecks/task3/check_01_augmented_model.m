function R = check_01_augmented_model()
% check_01_augmented_model
%
% Purpose
%   1) Build discrete linear model (Task 1 artefacts)
%   2) Build augmented model (Chapter 8) with constant disturbance state
%   3) Verify dimensions + observability of (A_aug, C_aug)
%   4) Report stability + conditioning diagnostics (useful for write-up)
%   5) Plot eigenvalues (A and A_aug) and observability singular values
%
% Outputs
%   R.pass, R.notes, R.error

R = struct('pass', false, 'notes', {{}}, 'error', '');

try
    Ccfg = config_constants();
    Tcfg = config_tuning(); %#ok<NASGU>  % (kept consistent)

    % --- Linear discrete model
    [Ac, Bc, Cmeas, Dc] = auv_linearise(Ccfg);
    [A,  B,  Cy,  ~ ]   = auv_discretise(Ac, Bc, Cmeas, Dc, Ccfg.Ts);

    % --- Augmentation (Chapter 8)
    [A_aug, ~, C_aug, Bw] = auv_build_augmented_model(A, B, Cy);

    % --- Dimensions
    nx = size(A,1);
    nu = size(B,2);
    ny = size(Cy,1);
    nx_aug = size(A_aug,1);

    R.notes{end+1} = sprintf('Discrete model dims: nx=%d, nu=%d, ny=%d', nx, nu, ny);
    R.notes{end+1} = sprintf('Augmented model dims: nx_aug=%d (nx + 1 disturbance)', nx_aug);

    % --- Disturbance channel check
    if any(size(Bw) ~= [nx,1])
        error('Bw has wrong size. Expected [%d x 1], got [%d x %d].', nx, size(Bw,1), size(Bw,2));
    end
    R.notes{end+1} = 'Bw set to first column of B (surge channel disturbance).';

    % --- Stability diagnostics (discrete-time)
    eigA    = eig(A);
    eigAaug = eig(A_aug);

    rhoA    = max(abs(eigA));
    rhoAaug = max(abs(eigAaug));

    R.notes{end+1} = sprintf('Spectral radius: max|eig(A)| = %.4g, max|eig(A_aug)| = %.4g', rhoA, rhoAaug);

    n_unstable_A    = sum(abs(eigA)    >= 1);
    n_unstable_Aaug = sum(abs(eigAaug) >= 1);
    R.notes{end+1} = sprintf('Eigenvalues outside unit circle: A=%d, A_aug=%d', n_unstable_A, n_unstable_Aaug);

    % --- Observability
    Ob  = obsv(A_aug, C_aug);
    rOb = rank(Ob);
    R.notes{end+1} = sprintf('rank(obsv(A_aug,C_aug)) = %d (max %d)', rOb, nx_aug);

    s = svd(Ob);
    condOb = s(1) / max(s(end), eps);
    R.notes{end+1} = sprintf('Obsv singular values: max=%.3g, min=%.3g, cond~=%.3g', s(1), s(end), condOb);

    % --- Plots (fixed figure numbers)
    fig1 = figure(310); clf(fig1);
    fig1.Name = 'Task3 Check 01: Eigenvalues';
    fig1.NumberTitle = 'off';

    tiledlayout(fig1,1,2);

    nexttile;
    plot(real(eigA), imag(eigA), 'x'); grid on;
    xlabel('Real'); ylabel('Imag');
    title('Eigenvalues of A');

    nexttile;
    plot(real(eigAaug), imag(eigAaug), 'x'); grid on;
    xlabel('Real'); ylabel('Imag');
    title('Eigenvalues of A_{aug}');

    out_png1 = fullfile(fileparts(mfilename('fullpath')), 'task3_check01_eigs.png');
    saveas(fig1, out_png1);
    R.notes{end+1} = ['Saved plot: ', out_png1];

    fig2 = figure(311); clf(fig2);
    fig2.Name = 'Task3 Check 01b: Observability singular values';
    fig2.NumberTitle = 'off';

    semilogy(1:numel(s), s, 'o-'); grid on;
    xlabel('Index'); ylabel('Singular value (log scale)');
    title('Observability matrix singular values');
    legend('O(A_{aug},C_{aug})','Location','best');

    out_png2 = fullfile(fileparts(mfilename('fullpath')), 'task3_check01_obsv_svd.png');
    saveas(fig2, out_png2);
    R.notes{end+1} = ['Saved plot: ', out_png2];

    % Pass condition: observable augmented model
    R.pass = (rOb == nx_aug);

catch ME
    R.pass  = false;
    R.error = ME.message;
end
end
