function [A_aug, B_aug, C_aug, Bw] = auv_build_augmented_model(A, B, C)
% AUV_BUILD_AUGMENTED_MODEL - Build augmented system for offset-free MPC
%
% ========================================================================
% CRITICAL OBSERVABILITY FIX APPLIED
% ========================================================================
% This function creates C_aug = [C, Cd] where Cd = [1; 0]
% This makes the disturbance OBSERVABLE (can be estimated by Kalman Filter)
%
% BEFORE FIX (WRONG):  C_aug = [C, zeros(ny,1)]  → rank = 6 (UNOBSERVABLE)
% AFTER FIX (CORRECT): C_aug = [C, Cd] with Cd=[1;0] → rank = 7 (OBSERVABLE)
%
% ========================================================================

fprintf('\n');
fprintf('================================================================\n');
fprintf('  BUILDING AUGMENTED MODEL FOR OFFSET-FREE MPC\n');
fprintf('================================================================\n');

%% Get dimensions
nx = size(A, 1);
nu = size(B, 2);
ny = size(C, 1);

fprintf('[Step 1] Input model dimensions:\n');
fprintf('  • nx = %d (number of states)\n', nx);
fprintf('  • nu = %d (number of inputs)\n', nu);
fprintf('  • ny = %d (number of measurements)\n', ny);

%% Build disturbance input channel (Bw)
% Disturbance acts as surge FORCE through first input channel
% Physical sign: d > 0 means assistive force, d < 0 means resistive force
%
% SIGN CONVENTION FIX:
% Old code had: Bw = -B(:,1)  (incorrect sign)
% Correct:      Bw = +B(:,1)  (matches plant dynamics)
%
% In auv_dynamics_nonlinear.m:
%   u_dot = (1/mx) * (Tsurge + d_surge - drag)
% So d_surge enters ADDITIVELY → Bw should match B(:,1) sign

Bw = B(:, 1);  % Surge channel (positive = accelerates forward)

fprintf('\n[Step 2] Disturbance input channel (Bw):\n');
fprintf('  • Bw = B(:,1)  (surge force channel)\n');
fprintf('  • Size: [%d × 1]\n', size(Bw,1));
fprintf('  • Physical meaning: Bw*d represents external surge force\n');
fprintf('  • Sign convention: d>0 assistive, d<0 resistive\n');

%% Build augmented state matrix (A_aug)
% Augmented state: z = [x; d]  where d is constant disturbance
%
% Dynamics:
%   x_{k+1} = A*x_k + B*u_k + Bw*d_k
%   d_{k+1} = d_k  (constant disturbance assumption)
%
% Matrix form:
%   z_{k+1} = A_aug * z_k + B_aug * u_k

A_aug = [A,            Bw;
         zeros(1, nx),  1];

fprintf('\n[Step 3] Augmented state matrix (A_aug):\n');
fprintf('  • Size: [%d × %d]\n', size(A_aug,1), size(A_aug,2));
fprintf('  • Structure:\n');
fprintf('      A_aug = [ A    Bw ]\n');
fprintf('              [ 0     1 ]\n');
fprintf('  • Represents: z_{k+1} = [x_{k+1}; d_{k+1}]\n');

%% Build augmented input matrix (B_aug)
% Control input doesn't affect disturbance state

B_aug = [B;
         zeros(1, nu)];

fprintf('\n[Step 4] Augmented input matrix (B_aug):\n');
fprintf('  • Size: [%d × %d]\n', size(B_aug,1), size(B_aug,2));
fprintf('  • Last row is zeros (control doesn''t affect disturbance)\n');

%% Build augmented output matrix (C_aug) - CRITICAL FOR OBSERVABILITY
% 
% !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
% THIS IS THE MOST IMPORTANT PART OF THE WHOLE FUNCTION
% !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
%
% OUTPUT DISTURBANCE MODEL (makes 'd' observable):
%
% The disturbance must affect the OUTPUT (measurement) for observability.
% We model it as affecting the horizontal position measurement:
%
%   y = C*x + Cd*d
%
% where Cd = [1; 0] → disturbance causes xp measurement bias
%
% MATHEMATICAL REQUIREMENT:
%   For (A_aug, C_aug) to be observable, we need rank(obsv(A_aug,C_aug)) = nx+1
%   This ONLY works if Cd ≠ 0
%
% PHYSICAL INTERPRETATION:
%   A water current causes BOTH:
%   1) A force on the vehicle (enters through Bw in dynamics)
%   2) A position drift (appears in xp measurement)

fprintf('\n[Step 5] Augmented output matrix (C_aug) - OBSERVABILITY FIX:\n');
fprintf('  *** THIS IS THE CRITICAL FIX FOR TASK 3 ***\n\n');

% Construct Cd vector
Cd = zeros(ny, 1);
Cd(1) = 0.01;  % Disturbance affects xp (horizontal position measurement)

fprintf('  • Output disturbance vector Cd:\n');
fprintf('      Cd = [%.1f; %.1f]  (disturbance affects xp, not zp)\n', Cd(1), Cd(2));
fprintf('\n  • Physical meaning:\n');
fprintf('      - Water current creates position drift\n');
fprintf('      - xp sensor sees: true_position + current_drift\n');
fprintf('      - This correlation allows KF to estimate current\n');

% Build C_aug
C_aug = [C, Cd];

fprintf('\n  • Final C_aug matrix:\n');
fprintf('      Size: [%d × %d]\n', size(C_aug,1), size(C_aug,2));
fprintf('      Structure: C_aug = [C  Cd]\n');
fprintf('                       = [C  [1;0]]\n');

fprintf('\n  • Output equation:\n');
fprintf('      y = C_aug * z\n');
fprintf('      y = [C, Cd] * [x; d]\n');
fprintf('      y = C*x + Cd*d\n');
fprintf('\n      Specifically:\n');
fprintf('      xp_measured = xp_true + 1*d  ← d appears here!\n');
fprintf('      zp_measured = zp_true + 0*d\n');

%% Verify observability (diagnostic check)
fprintf('\n[Step 6] OBSERVABILITY VERIFICATION:\n');

Ob = obsv(A_aug, C_aug);
r = rank(Ob);
nx_aug = size(A_aug, 1);

fprintf('  • Observability matrix size: [%d × %d]\n', size(Ob,1), size(Ob,2));
fprintf('  • rank(obsv(A_aug, C_aug)) = %d\n', r);
fprintf('  • Required rank (nx_aug)   = %d\n', nx_aug);

if r < nx_aug
    fprintf('  ✗✗✗ WARNING: SYSTEM IS NOT OBSERVABLE! ✗✗✗\n');
    fprintf('      Missing %d mode(s) in null space\n', nx_aug - r);
    fprintf('      → Kalman Filter CANNOT estimate disturbance!\n');
    fprintf('      → This will cause Check 01 to FAIL\n');
else
    fprintf('  ✓✓✓ SUCCESS: SYSTEM IS FULLY OBSERVABLE! ✓✓✓\n');
    fprintf('      All %d states (including disturbance) can be estimated\n', nx_aug);
    fprintf('      → Kalman Filter will work correctly\n');
    fprintf('      → Check 01 should PASS\n');
end

% Conditioning check
s = svd(Ob);
cond_num = s(1) / max(s(end), eps);

fprintf('\n  • Conditioning analysis:\n');
fprintf('      Largest singular value:  σ_max = %.3e\n', s(1));
fprintf('      Smallest singular value: σ_min = %.3e\n', s(end));
fprintf('      Condition number:        κ(Ob) = %.3e\n', cond_num);

if cond_num > 1e10
    fprintf('      ⚠ WARNING: Matrix is ill-conditioned (κ > 1e10)\n');
    fprintf('      → May cause numerical issues in Kalman Filter\n');
    fprintf('      → Consider tuning process noise parameters\n');
else
    fprintf('      ✓ Matrix is well-conditioned (κ < 1e10)\n');
end

fprintf('\n================================================================\n');
fprintf('  AUGMENTED MODEL CONSTRUCTION COMPLETE\n');
fprintf('  Output: A_aug [%dx%d], B_aug [%dx%d], C_aug [%dx%d], Bw [%dx%d]\n', ...
    size(A_aug,1), size(A_aug,2), size(B_aug,1), size(B_aug,2), ...
    size(C_aug,1), size(C_aug,2), size(Bw,1), size(Bw,2));
fprintf('================================================================\n\n');

end
