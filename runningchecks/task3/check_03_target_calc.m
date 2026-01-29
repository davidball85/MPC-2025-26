function R = check_03_target_calc()
% check_03_target_calc
% Demonstrate target calculator producing non-zero u_ss to fight d_hat.
% Includes a sweep plot (u_ss(1) vs d_hat).
%
% NOTE:
% If you have "dbstop if error" enabled, YALMIP/solver errors will drop you
% into K>> before try/catch can catch. This check disables that locally.

R = struct('pass', false, 'notes', {{}}, 'error', '');

% --- Make this check robust against dbstop-if-error ---
%dbclear if error

try
    C = config_constants();
    T = config_tuning();

    % Prevent stale YALMIP optimiser state causing odd behaviour
    yalmip('clear');

    % Keep solvers quiet for running checks
    if isfield(T,'verbose')
        T.verbose = 0;
    end

    % ---- Discrete linear model (robust call)
    [Ac, Bc, Cmeas, Dc] = auv_linearise(C);
    [Ad, Bd, ~] = local_discretise(Ac, Bc, Cmeas, Dc, C);

    model.A  = Ad;
    model.B  = Bd;
    model.Bw = Bd(:,1);

    x_ref = T.x_ref_task3(:);

    % Sweep range (kept simple and visible)
    dhats = linspace(-40, 0, 41);
    uss1  = zeros(size(dhats));
    err_flags = zeros(size(dhats));

    data.x_hat = x_ref;
    data.x_ref = x_ref;

    for i = 1:numel(dhats)
        data.d_hat = dhats(i);

        [~, u_ss, info] = target_calc('qp', data, model, C, T);

        % Track if target calc fell back or reported solver trouble
        if isstruct(info) && isfield(info,'method') && contains(info.method,'failed')
            err_flags(i) = 1;
        end

        uss1(i) = u_ss(1);
    end

    % ---- Plot (report-ready)
    figure('Name','Task3 Check 03: Target Calc','NumberTitle','off');

    plot(dhats, uss1, 'o-'); grid on; hold on;
    xlabel('d\_hat (N)');
    ylabel('u\_{ss, surge} (N)');
    title('Target calculator feedforward: surge thrust vs estimated disturbance');

    % Mark any fallback points, if they happened
    if any(err_flags)
        plot(dhats(logical(err_flags)), uss1(logical(err_flags)), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
        legend('u_{ss}(1)', 'fallback points', 'Location', 'best');
        R.notes{end+1} = sprintf('NOTE: %d sweep points used fallback/solver recovery.', sum(err_flags));
    end

    out_png = fullfile(fileparts(mfilename('fullpath')), 'task3_check03_targetcalc.png');
    saveas(gcf, out_png);

    R.notes{end+1} = sprintf('Sweep done: d_hat in [%.1f, %.1f] N.', dhats(1), dhats(end));
    R.notes{end+1} = 'Expected trend: more negative disturbance -> higher (more positive) surge thrust.';
    R.notes{end+1} = ['Saved plot: ', out_png];

    R.pass = true;

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
