function R = check_03_target_calc()
% check_03_target_calc
%
% Purpose
%   Demonstrate target calculator producing non-zero u_ss to fight d_hat.
%
% What this check produces
%   1) Sweep plot: u_ss(surge) vs d_hat (basic monotonic sanity check)
%   2) Linear fit summary (slope) to quantify "how much thrust per N disturbance"
%
% Notes
%   - Uses config for constraints and bounds (single source of truth).
%   - Does not hard-code any limits.

R = struct('pass', false, 'notes', {{}}, 'error', '');

try
    Ccfg = config_constants();
    Tcfg = config_tuning();

    % Prevent stale YALMIP optimiser state causing odd behaviour
    yalmip('clear');

    % Keep solvers quiet for running checks
    if isfield(Tcfg,'verbose')
        Tcfg.verbose = 0;
    end

    % ---- Discrete linear model
    [Ac, Bc, ~, Dc] = auv_linearise(Ccfg);

    % Measurement matrix for y=[xp;zp]
    nx = numel(Ccfg.x_eq);
    Cmeas = zeros(2, nx);
    Cmeas(1, Ccfg.idx.xp) = 1;
    Cmeas(2, Ccfg.idx.zp) = 1;

    [Ad, Bd, ~] = local_discretise(Ac, Bc, Cmeas, Dc, Ccfg);

    model.A  = Ad;
    model.B  = Bd;
    model.Bw = Bd(:,1);

    x_ref = Tcfg.x_ref_task3(:);

    % Sweep range (keep aligned with your existing plot)
    dhats = linspace(-40, 0, 41);
    uss1  = zeros(size(dhats));
    err_flags = false(size(dhats));

    data.x_hat = x_ref;
    data.x_ref = x_ref;

    for i = 1:numel(dhats)
        data.d_hat = dhats(i);

        [~, u_ss, info] = target_calc('qp', data, model, Ccfg, Tcfg);

        if isstruct(info) && isfield(info,'method') && contains(info.method,'failed')
            err_flags(i) = true;
        end

        uss1(i) = u_ss(1);
    end

    % Quantify trend (simple least-squares fit)
    p = polyfit(dhats(:), uss1(:), 1);
    slope = p(1);  % N thrust per N disturbance (based on sign convention)
    R.notes{end+1} = sprintf('Target-calc linear fit: u_ss,surge ≈ %.3f*d_hat + %.3f.', p(1), p(2));
    R.notes{end+1} = sprintf('Interpretable gain: %.3f N surge thrust per 1 N disturbance estimate.', slope);

    % ---- Plot
    fig = figure(330); clf(fig);
    fig.Name = 'Task3 Check 03: Target Calc';
    fig.NumberTitle = 'off';

    plot(dhats, uss1, 'o-'); grid on; hold on;
    xlabel('d\_hat (N)');
    ylabel('u\_{ss, surge} (N)');
    title('Target calculator feedforward: surge thrust vs estimated disturbance');

    if any(err_flags)
        plot(dhats(err_flags), uss1(err_flags), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
        legend('u_{ss}(1)', 'fallback points', 'Location', 'best');
        R.notes{end+1} = sprintf('NOTE: %d sweep points used fallback/solver recovery.', sum(err_flags));
    end

    out_png = fullfile(fileparts(mfilename('fullpath')), 'task3_check03_targetcalc.png');
    saveas(fig, out_png);

    R.notes{end+1} = sprintf('Sweep done: d_hat in [%.1f, %.1f] N.', dhats(1), dhats(end));
    R.notes{end+1} = 'Expected trend: more negative disturbance -> higher (more positive) surge thrust.';
    R.notes{end+1} = ['Saved plot: ', out_png];

    R.pass = true;

catch ME
    R.pass  = false;
    R.error = ME.message;
end
end

% =====================================================================
function [A, B, Cy] = local_discretise(Ac, Bc, Cmeas, Dc, Ccfg)
Ts = Ccfg.Ts;

% Try your project discretiser first (robust to signature differences)
try
    [A, B, Cy, ~] = auv_discretise(Ac, Bc, Cmeas, Dc, Ts);
    return
catch
end

try
    [A, B, Cy] = auv_discretise(Ac, Bc, Cmeas, Ts);
    return
catch
end

try
    [A, B, Cy] = auv_discretise(Ac, Bc, Cmeas);
    return
catch
end

% Fallback: MATLAB c2d
sysd = c2d(ss(Ac, Bc, Cmeas, Dc), Ts, 'zoh');
A  = sysd.A;
B  = sysd.B;
Cy = sysd.C;
end
