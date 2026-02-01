function R = check_01_augmented_model()
% CHECK_01_AUGMENTED_MODEL - Verify augmented model observability
%
% This is THE MOST IMPORTANT check for Task 3.
% If this fails, NOTHING else will work.
%
% CRITICAL TEST: rank(obsv(A_aug, C_aug)) must equal nx_aug (=7)

fprintf('\n');
fprintf('████████████████████████████████████████████████████████████\n');
fprintf('█                                                          █\n');
fprintf('█   TASK 3 - CHECK 01: AUGMENTED MODEL OBSERVABILITY      █\n');
fprintf('█                                                          █\n');
fprintf('████████████████████████████████████████████████████████████\n');
fprintf('\n');

R = struct('pass', false, 'notes', {{}}, 'error', '');

try
    %% ============================================================
    %%  STEP 1: LOAD CONFIGURATION
    %% ============================================================
    fprintf('════════════════════════════════════════════════════════\n');
    fprintf('  STEP 1: LOADING CONFIGURATION\n');
    fprintf('════════════════════════════════════════════════════════\n');
    
    Ccfg = config_constants();
    Tcfg = config_tuning();
    
    fprintf('[1.1] Configuration loaded successfully\n');
    fprintf('      Ts = %.3f s\n', Ccfg.Ts);
    fprintf('      Equilibrium: u=%.1f m/s, zp=%.1f m\n', ...
        Ccfg.x_eq(Ccfg.idx.u), Ccfg.x_eq(Ccfg.idx.zp));
    
    %% ============================================================
    %%  STEP 2: BUILD DISCRETE LINEAR MODEL
    %%  ============================================================
    fprintf('\n════════════════════════════════════════════════════════\n');
    fprintf('  STEP 2: BUILDING DISCRETE LINEAR MODEL\n');
    fprintf('════════════════════════════════════════════════════════\n');
    
    fprintf('[2.1] Continuous-time linearisation...\n');
    [Ac, Bc, Cmeas, Dc] = auv_linearise(Ccfg);
    fprintf('      ✓ Linearised around x_eq\n');
    fprintf('      ✓ Ac: [%d × %d], Bc: [%d × %d]\n', ...
        size(Ac,1), size(Ac,2), size(Bc,1), size(Bc,2));
    
    fprintf('\n[2.2] Discretising with ZOH...\n');
    [A, B, Cy, ~] = auv_discretise(Ac, Bc, Cmeas, Dc, Ccfg.Ts);
    fprintf('      ✓ Discretisation complete\n');
    fprintf('      ✓ A: [%d × %d], B: [%d × %d], Cy: [%d × %d]\n', ...
        size(A,1), size(A,2), size(B,1), size(B,2), size(Cy,1), size(Cy,2));
    
    nx = size(A,1);
    nu = size(B,2);
    ny = size(Cy,1);
    
    fprintf('\n[2.3] Model Dimensions:\n');
    fprintf('      States (nx):       %d\n', nx);
    fprintf('      Inputs (nu):       %d\n', nu);
    fprintf('      Measurements (ny): %d\n', ny);
    
    R.notes{end+1} = sprintf('Discrete model dims: nx=%d, nu=%d, ny=%d', nx, nu, ny);
    
    %% ============================================================
    %%  STEP 3: BUILD AUGMENTED MODEL (CRITICAL!)
    %%  ============================================================
    fprintf('\n════════════════════════════════════════════════════════\n');
    fprintf('  STEP 3: BUILDING AUGMENTED MODEL\n');
    fprintf('════════════════════════════════════════════════════════\n');
    fprintf('  *** THIS IS WHERE THE FIX IS APPLIED ***\n');
    fprintf('════════════════════════════════════════════════════════\n');
    
    % This function contains EXTENSIVE internal logging
    [A_aug, ~, C_aug, Bw] = auv_build_augmented_model(A, B, Cy);
    
    nx_aug = size(A_aug,1);
    
    fprintf('[3.1] Augmented model returned:\n');
    fprintf('      nx_aug = %d (should be nx + 1 = %d)\n', nx_aug, nx+1);
    R.notes{end+1} = sprintf('Augmented model dims: nx_aug=%d (nx + 1 disturbance)', nx_aug);
    
    %% Verify Bw size
    fprintf('\n[3.2] Disturbance channel check:\n');
    if any(size(Bw) ~= [nx,1])
        fprintf('      ✗ ERROR: Bw has wrong size!\n');
        fprintf('        Expected: [%d × 1]\n', nx);
        fprintf('        Got:      [%d × %d]\n', size(Bw,1), size(Bw,2));
        error('Bw has wrong size. Expected [%d x 1], got [%d x %d].', ...
            nx, size(Bw,1), size(Bw,2));
    else
        fprintf('      ✓ Bw size correct: [%d × 1]\n', nx);
    end
    R.notes{end+1} = 'Bw set to B(:,1) (surge channel disturbance)';
    
    %% Display C_aug structure
    fprintf('\n[3.3] CRITICAL: C_aug structure:\n');
    fprintf('      C_aug has %d rows (outputs) and %d columns (states)\n', ...
        size(C_aug,1), size(C_aug,2));
    fprintf('      Last column of C_aug (disturbance effect):\n');
    fprintf('        C_aug(:,end) = [%.1f; %.1f]\n', C_aug(1,end), C_aug(2,end));
    
    if all(C_aug(:,end) == 0)
        fprintf('      ✗✗✗ FAIL: Last column is ALL ZEROS!\n');
        fprintf('          This means disturbance does NOT affect output.\n');
        fprintf('          → System will be UNOBSERVABLE!\n');
        fprintf('          → You need to fix auv_build_augmented_model.m\n');
    else
        fprintf('      ✓✓✓ GOOD: Last column is NOT all zeros.\n');
        fprintf('          Disturbance affects output → may be observable.\n');
    end
    
    %% ============================================================
    %%  STEP 4: STABILITY ANALYSIS
    %%  ============================================================
    fprintf('\n════════════════════════════════════════════════════════\n');
    fprintf('  STEP 4: STABILITY ANALYSIS\n');
    fprintf('════════════════════════════════════════════════════════\n');
    
    eigA = eig(A);
    eigAaug = eig(A_aug);
    
    rhoA = max(abs(eigA));
    rhoAaug = max(abs(eigAaug));
    
    fprintf('[4.1] Spectral Radius (discrete-time):\n');
    fprintf('      ρ(A)     = max|λ(A)|     = %.6f\n', rhoA);
    fprintf('      ρ(A_aug) = max|λ(A_aug)| = %.6f\n', rhoAaug);
    
    if rhoA >= 1
        fprintf('      ⚠ Original system has eigenvalues on/outside unit circle\n');
    end
    if rhoAaug >= 1
        fprintf('      ⚠ Augmented system has eigenvalues on/outside unit circle\n');
        fprintf('        (Expected: disturbance integrator has λ=1)\n');
    end
    
    n_unstable_A = sum(abs(eigA) >= 1);
    n_unstable_Aaug = sum(abs(eigAaug) >= 1);
    
    fprintf('\n[4.2] Eigenvalues on/outside unit circle:\n');
    fprintf('      A:     %d eigenvalues with |λ| ≥ 1\n', n_unstable_A);
    fprintf('      A_aug: %d eigenvalues with |λ| ≥ 1\n', n_unstable_Aaug);
    
    R.notes{end+1} = sprintf('Spectral radius: max|eig(A)| = %.4g, max|eig(A_aug)| = %.4g', ...
        rhoA, rhoAaug);
    R.notes{end+1} = sprintf('Eigenvalues outside unit circle: A=%d, A_aug=%d', ...
        n_unstable_A, n_unstable_Aaug);
    
    %% ============================================================
    %%  STEP 5: OBSERVABILITY TEST (THE CRITICAL CHECK!)
    %%  ============================================================
    fprintf('\n════════════════════════════════════════════════════════\n');
    fprintf('  STEP 5: OBSERVABILITY TEST *** CRITICAL ***\n');
    fprintf('════════════════════════════════════════════════════════\n');
    fprintf('  THIS DETERMINES IF TASK 3 WILL WORK!\n');
    fprintf('════════════════════════════════════════════════════════\n');
    
    fprintf('\n[5.1] Computing observability matrix...\n');
    Ob = obsv(A_aug, C_aug);
    fprintf('      ✓ Observability matrix computed\n');
    fprintf('      ✓ Size: [%d × %d]\n', size(Ob,1), size(Ob,2));
    
    fprintf('\n[5.2] Computing rank...\n');
    rOb = rank(Ob);
    fprintf('      ✓ rank(obsv(A_aug, C_aug)) = %d\n', rOb);
    fprintf('      ✓ Required rank (nx_aug)   = %d\n', nx_aug);
    
    R.notes{end+1} = sprintf('rank(obsv(A_aug,C_aug)) = %d (max %d)', rOb, nx_aug);
    
    fprintf('\n[5.3] VERDICT:\n');
    if rOb < nx_aug
        fprintf('      ███████████████████████████████████████████████████\n');
        fprintf('      ██                                               ██\n');
        fprintf('      ██   ✗✗✗ OBSERVABILITY TEST: FAIL ✗✗✗          ██\n');
        fprintf('      ██                                               ██\n');
        fprintf('      ███████████████████████████████████████████████████\n\n');
        fprintf('      rank = %d, need rank = %d\n', rOb, nx_aug);
        fprintf('      Missing %d dimension(s) in observable subspace\n\n', nx_aug - rOb);
        fprintf('      DIAGNOSIS:\n');
        fprintf('      • The augmented system is NOT observable\n');
        fprintf('      • Kalman Filter CANNOT estimate disturbance\n');
        fprintf('      • Task 3 will FAIL\n\n');
        fprintf('      LIKELY CAUSE:\n');
        fprintf('      • C_aug = [C, zeros(ny,1)] in auv_build_augmented_model.m\n');
        fprintf('      • Disturbance does not affect measurement\n\n');
        fprintf('      FIX:\n');
        fprintf('      • Set Cd = [1; 0] (disturbance affects xp)\n');
        fprintf('      • Use: C_aug = [C, Cd]\n');
        fprintf('      • See OBSERVABILITY_FIX_EXPLAINED.txt\n\n');
        
        R.pass = false;
    else
        fprintf('      ███████████████████████████████████████████████████\n');
        fprintf('      ██                                               ██\n');
        fprintf('      ██   ✓✓✓ OBSERVABILITY TEST: PASS! ✓✓✓         ██\n');
        fprintf('      ██                                               ██\n');
        fprintf('      ███████████████████████████████████████████████████\n\n');
        fprintf('      rank = %d = nx_aug ✓\n', rOb);
        fprintf('      System is FULLY observable\n\n');
        fprintf('      IMPLICATIONS:\n');
        fprintf('      • Kalman Filter CAN estimate all states\n');
        fprintf('      • Disturbance estimation will work\n');
        fprintf('      • Offset-free MPC is feasible\n');
        fprintf('      • Task 3 should work!\n\n');
        
        R.pass = true;
    end
    
    %% ============================================================
    %%  STEP 6: CONDITIONING ANALYSIS
    %%  ============================================================
    fprintf('════════════════════════════════════════════════════════\n');
    fprintf('  STEP 6: NUMERICAL CONDITIONING\n');
    fprintf('════════════════════════════════════════════════════════\n');
    
    fprintf('\n[6.1] Computing singular values...\n');
    s = svd(Ob);
    condOb = s(1) / max(s(end), eps);
    
    fprintf('      σ_max (largest singular value)  = %.3e\n', s(1));
    fprintf('      σ_min (smallest singular value) = %.3e\n', s(end));
    fprintf('      κ(Ob) (condition number)        = %.3e\n', condOb);
    
    R.notes{end+1} = sprintf('Obsv singular values: max=%.3g, min=%.3g, cond~=%.3g', ...
        s(1), s(end), condOb);
    
    fprintf('\n[6.2] Conditioning Assessment:\n');
    if condOb > 1e12
        fprintf('      ✗ SEVERELY ILL-CONDITIONED (κ > 1e12)\n');
        fprintf('        → Major numerical issues expected\n');
    elseif condOb > 1e10
        fprintf('      ⚠ ILL-CONDITIONED (κ > 1e10)\n');
        fprintf('        → Kalman Filter may have numerical problems\n');
        fprintf('        → Increase process noise on disturbance\n');
    elseif condOb > 1e8
        fprintf('      ⚡ MODERATELY CONDITIONED (κ > 1e8)\n');
        fprintf('        → Acceptable, but monitor KF performance\n');
    else
        fprintf('      ✓ WELL-CONDITIONED (κ < 1e8)\n');
        fprintf('        → No numerical issues expected\n');
    end
    
    %% ============================================================
    %%  STEP 7: GENERATE DIAGNOSTIC PLOTS
    %%  ============================================================
    fprintf('\n════════════════════════════════════════════════════════\n');
    fprintf('  STEP 7: GENERATING DIAGNOSTIC PLOTS\n');
    fprintf('════════════════════════════════════════════════════════\n');
    
    % Plot 1: Eigenvalues
    fprintf('[7.1] Creating eigenvalue plot...\n');
    fig1 = figure(310);
    clf(fig1);
    fig1.Name = 'Task3 Check 01: Eigenvalues';
    fig1.NumberTitle = 'off';
    fig1.Position = [100, 100, 1000, 400];
    
    tiledlayout(fig1,1,2);
    
    % Original system
    nexttile;
    theta = linspace(0, 2*pi, 100);
    plot(cos(theta), sin(theta), 'k--', 'LineWidth', 1.5);  % Unit circle
    hold on;
    plot(real(eigA), imag(eigA), 'bx', 'MarkerSize', 12, 'LineWidth', 2);
    hold off;
    grid on;
    axis equal;
    xlabel('Real Part');
    ylabel('Imaginary Part');
    title(sprintf('Eigenvalues of A (nx=%d)', nx));
    legend('Unit Circle', 'Eigenvalues', 'Location', 'best');
    
    % Augmented system
    nexttile;
    plot(cos(theta), sin(theta), 'k--', 'LineWidth', 1.5);  % Unit circle
    hold on;
    plot(real(eigAaug), imag(eigAaug), 'rx', 'MarkerSize', 12, 'LineWidth', 2);
    hold off;
    grid on;
    axis equal;
    xlabel('Real Part');
    ylabel('Imaginary Part');
    title(sprintf('Eigenvalues of A_{aug} (nx=%d)', nx_aug));
    legend('Unit Circle', 'Eigenvalues', 'Location', 'best');
    
    out_png1 = fullfile(fileparts(mfilename('fullpath')), 'task3_check01_eigs.png');
    saveas(fig1, out_png1);
    fprintf('      ✓ Saved: %s\n', out_png1);
    R.notes{end+1} = ['Saved plot: ', out_png1];
    
    % Plot 2: Observability singular values
    fprintf('[7.2] Creating SVD plot...\n');
    fig2 = figure(311);
    clf(fig2);
    fig2.Name = 'Task3 Check 01b: Observability Singular Values';
    fig2.NumberTitle = 'off';
    fig2.Position = [150, 150, 600, 500];
    
    semilogy(1:numel(s), s, 'bo-', 'MarkerSize', 8, 'LineWidth', 2);
    hold on;
    yline(1e-10, 'r--', 'LineWidth', 1.5);
    text(numel(s)*0.7, 1e-9, 'Numerical zero threshold', 'Color', 'r');
    hold off;
    grid on;
    xlabel('Singular Value Index');
    ylabel('Singular Value (log scale)');
    title(sprintf('Observability Matrix Σ (κ = %.2e)', condOb));
    legend('σ_i', 'Location', 'best');
    
    out_png2 = fullfile(fileparts(mfilename('fullpath')), 'task3_check01_obsv_svd.png');
    saveas(fig2, out_png2);
    fprintf('      ✓ Saved: %s\n', out_png2);
    R.notes{end+1} = ['Saved plot: ', out_png2];
    
    %% ============================================================
    %%  FINAL SUMMARY
    %%  ============================================================
    fprintf('\n████████████████████████████████████████████████████████████\n');
    fprintf('█                                                          █\n');
    if R.pass
        fprintf('█          ✓✓✓ CHECK 01: PASS ✓✓✓                      █\n');
        fprintf('█                                                          █\n');
        fprintf('█  Augmented model is OBSERVABLE and ready for KF!        █\n');
    else
        fprintf('█          ✗✗✗ CHECK 01: FAIL ✗✗✗                      █\n');
        fprintf('█                                                          █\n');
        fprintf('█  Augmented model is NOT observable!                     █\n');
        fprintf('█  → Fix auv_build_augmented_model.m                      █\n');
    end
    fprintf('█                                                          █\n');
    fprintf('████████████████████████████████████████████████████████████\n\n');
    
catch ME
    fprintf('\n████████████████████████████████████████████████████████████\n');
    fprintf('█                                                          █\n');
    fprintf('█          ✗✗✗ CHECK 01: ERROR ✗✗✗                      █\n');
    fprintf('█                                                          █\n');
    fprintf('████████████████████████████████████████████████████████████\n\n');
    fprintf('Exception: %s\n', ME.message);
    fprintf('\nStack trace:\n');
    for k = 1:length(ME.stack)
        fprintf('  [%d] %s (line %d)\n', k, ME.stack(k).name, ME.stack(k).line);
    end
    fprintf('\n');
    
    R.pass = false;
    R.error = ME.message;
end

end
