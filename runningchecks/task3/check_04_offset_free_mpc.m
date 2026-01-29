function R = check_04_offset_free_mpc()
% check_04_offset_free_mpc
% Runs the full Task 3 sim and exports report-ready plots.
% Verifies recovery of speed after disturbance step.

R = struct('pass', false, 'notes', {{}}, 'error', '');

try
    logs = task3_main();

    C = config_constants();
    T = config_tuning();

    % Simple numeric check: final speed error small
    u_final = logs.x(C.idx.u,end);
    u_ref   = T.x_ref_task3(C.idx.u);
    ss_err  = abs(u_final - u_ref);

    R.notes{end+1} = sprintf('Final speed: %.4f m/s (ref %.2f), |ss err| = %.4g', u_final, u_ref, ss_err);
    R.notes{end+1} = sprintf('Disturbance step: %.1f N at t=%.1f s (from config).', C.dist.surge_step_N, C.dist.surge_step_time_s);

    % A "reasonable" tolerance for a stochastic noisy sim (KF noise)
    if ss_err < 0.02
        R.notes{end+1} = 'PASS criterion: speed steady-state error < 0.02 m/s.';
        R.pass = true;
    else
        R.notes{end+1} = 'FAIL criterion: speed steady-state error not small enough.';
        R.pass = false;
    end

catch ME
    R.pass = false;
    R.error = ME.message;
end
end
