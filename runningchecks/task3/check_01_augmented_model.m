function R = check_01_augmented_model()
% check_01_augmented_model
%
% Purpose
%   1) Build discrete linear model (Task 1 artefacts)
%   2) Build augmented model (Chapter 8) with constant disturbance state
%   3) Verify dimensions + basic observability of (A_aug, C_aug)
%   4) Plot eigenvalues (A and A_aug) for report discussion
%
% Outputs
%   R.pass, R.notes, R.error

R = struct('pass', false, 'notes', {{}}, 'error', '');

try
    Cc = config_constants();
    T  = config_tuning();

    % --- Linear discrete model
    [Ac, Bc, Cmeas, Dc] = auv_linearise(Ccfg);
    [A,  B,  Cy,  ~ ]   = auv_discretise(Ac, Bc, Cmeas, Dc, Ccfg.Ts);


    % --- Augmentation (Chapter 8)
    [A_aug,B_aug,C_aug,Bw] = auv_build_augmented_model(A,B,Cy);

    % Checks
    nx = size(A,1); nu = size(B,2); ny = size(Cy,1);
    nx_aug = size(A_aug,1);

    R.notes{end+1} = sprintf('Discrete model dims: nx=%d, nu=%d, ny=%d', nx, nu, ny);
    R.notes{end+1} = sprintf('Augmented model dims: nx_aug=%d (nx + 1 disturbance)', nx_aug);

    if any(size(Bw) ~= [nx,1])
        error('Bw has wrong size. Expected [%d x 1], got [%d x %d].', nx, size(Bw,1), size(Bw,2));
    end
    R.notes{end+1} = 'Bw set to first column of B (surge channel disturbance).';

    % Observability of augmented pair
    Ob = obsv(A_aug, C_aug);
    rOb = rank(Ob);
    R.notes{end+1} = sprintf('rank(obsv(A_aug,C_aug)) = %d (max %d)', rOb, nx_aug);

    % Plot eigenvalues
    figure('Name','Task3 Check 01: Eigenvalues','NumberTitle','off');
    subplot(1,2,1);
    eigA = eig(A);
    plot(real(eigA), imag(eigA), 'x'); grid on;
    xlabel('Real'); ylabel('Imag');
    title('Eigenvalues of A');

    subplot(1,2,2);
    eigAaug = eig(A_aug);
    plot(real(eigAaug), imag(eigAaug), 'x'); grid on;
    xlabel('Real'); ylabel('Imag');
    title('Eigenvalues of A_{aug}');

    % Save figure for report reuse
    out_png = fullfile(fileparts(mfilename('fullpath')), 'task3_check01_eigs.png');
    saveas(gcf, out_png);

    R.notes{end+1} = ['Saved plot: ', out_png];
    R.pass = true;

catch ME
    R.pass = false;
    R.error = ME.message;
end
end