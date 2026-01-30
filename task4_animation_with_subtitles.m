function task4_animation_task3_cleanwater_FIXEDSHAPE()
clc; close all;

%% ================= TOGGLES =================
OPT.showInsets = false;
OPT.makeVideo  = true;

%% ================= COLOURS =================
COL.waterFig = [0.05 0.12 0.22];
COL.waterAx  = [0.03 0.10 0.20];
COL.grid     = [0.75 0.85 0.95];
COL.white    = [1 1 1];

COL.safeLine = [0.70 0.85 1.00];
COL.wallLine = [1.00 0.85 0.40];
COL.stopLine = [1.00 0.55 0.40];

BOX.face = [0 0 0];
BOX.border = [0 0 0];
BOX.captionAlpha = 0.55;
BOX.telemetryAlpha = 0.45;
BOX.borderW = 1.5;

%% ================= VIDEO RESOLUTION =================
VIDEO.width  = 1920;
VIDEO.height = 1080;

%% ================= SETTINGS =================
OPT.videoName  = 'Task4_Task3_Animation_CleanWater_FixedShape.mp4';
OPT.fps       = 6;     % faster preview-friendly
OPT.frameHold = 1;     % set to 9 for ~3 min export
OPT.skip      = 1;

OPT.auvL = 2.6;        % longer capsule looks more submarine-like
OPT.auvH = 0.70;

OPT.auvFace = [0.85 0.85 0.88];
OPT.auvEdge = [0.10 0.10 0.15];
OPT.auvEdgeW = 2.0;

OPT.arrowScale_m_per_N = 0.03;
OPT.flipDistArrow = false;

u_ref = 1.0;
z_ref = 5.0;
DIST_STEP_TIME = 10.0;

THR.uRecover  = 0.02;
THR.dHatActive = 2.0;
THR.depthWarn = 0.20;

%% ================= LOAD CONSTANTS + LOGS =================
C = config_constants();

if ~evalin('base','exist(''logs'',''var'')')
    logs = task3_main();
else
    logs = evalin('base','logs');
end

t     = logs.t(:);
x     = logs.x;
dhat  = logs.dhat(:);
uss   = logs.uss;

xp    = x(C.idx.xp,:).';
zp    = x(C.idx.zp,:).';
theta = x(C.idx.theta,:).';
uSur  = x(C.idx.u,:).';
wHea  = x(C.idx.w,:).';

if numel(dhat) < numel(t), dhat_plot = [dhat; dhat(end)];
else, dhat_plot = dhat(1:numel(t)); end

if size(uss,2) < numel(t), uss_plot = [uss, uss(:,end)];
else, uss_plot = uss(:,1:numel(t)); end

%% ================= BUILD A STABLE CAPSULE (CLOCKWISE, CLOSED) =================
L = OPT.auvL;
H = OPT.auvH;
r = H/2;

% Straight section half-length (excluding round ends)
a = (L/2) - r;
if a <= 0
    error('AUV length must be greater than height so capsule has a straight section.');
end

nArc = 60;

% Nose (right) semicircle: top -> bottom (clockwise)
angNose = linspace(pi/2, -pi/2, nArc);
noseArc = [ a + r*cos(angNose(:)), 0 + r*sin(angNose(:)) ];

% Tail (left) semicircle: bottom -> top (clockwise)
angTail = linspace(-pi/2, pi/2, nArc);
tailArc = [ -a + r*cos(angTail(:)), 0 + r*sin(angTail(:)) ];

% Assemble clockwise:
% start at nose top, go down nose arc to nose bottom,
% then straight along bottom to tail bottom,
% then up tail arc to tail top,
% then straight along top back to nose top.
bottomLine = [ linspace(a, -a, 20).', -r*ones(20,1) ];
topLine    = [ linspace(-a, a, 20).',  r*ones(20,1) ];

capsuleLocal = [ noseArc; bottomLine; tailArc; topLine ];
capsuleLocal = [capsuleLocal; capsuleLocal(1,:)]; % close polygon

noseTipLocal = [a + r, 0];  % absolute nose tip (furthest +x)

%% ================= MAIN FIGURE =================
fig = figure( ...
    'Color', COL.waterFig, ...
    'Name', 'Task 4 Animation', ...
    'NumberTitle', 'off', ...
    'Units', 'pixels', ...
    'Position', [100 100 VIDEO.width VIDEO.height], ...
    'Resize', 'off' );

ax  = axes(fig);
hold(ax,'on');
set(ax,'Color', COL.waterAx);
grid(ax,'on');

ax.GridAlpha = 0.18;
ax.GridColor = COL.grid;
ax.XColor = COL.white;
ax.YColor = COL.white;

xlabel(ax,'x_p (m)','Color',COL.white);
ylabel(ax,'z_p (m) [depth]','Color',COL.white);
title(ax,'AUV Animation (Task 3)','Color',COL.white);
set(ax,'YDir','reverse');

padX = 8; padZ = 2;
xmin = min(xp) - padX;
xmax = max([max(xp), C.wall_xp]) + padX;
zmin = min([min(zp), C.zp_min]) - padZ;
zmax = max([max(zp), C.zp_max]) + padZ;
axis(ax, [xmin xmax zmin zmax]);

plot(ax, [xmin xmax], [C.zp_min C.zp_min], '--', 'LineWidth', 1.2, 'Color', COL.safeLine);
plot(ax, [xmin xmax], [C.zp_max C.zp_max], '--', 'LineWidth', 1.2, 'Color', COL.safeLine);
plot(ax, [C.wall_xp C.wall_xp], [zmin zmax], '-',  'LineWidth', 1.4, 'Color', COL.wallLine);
plot(ax, [C.wall_stop C.wall_stop], [zmin zmax], '--', 'LineWidth', 1.4, 'Color', COL.stopLine);

%% ================= AUV + ARROW =================
auvPatch = patch(ax, nan, nan, OPT.auvFace, ...
    'EdgeColor', OPT.auvEdge, 'LineWidth', OPT.auvEdgeW);

% Nose indicator (triangle) always at nose tip
noseTri = patch(ax, nan, nan, [1 0.6 0.2], 'EdgeColor','none');

distArrow = quiver(ax, nan, nan, nan, nan, ...
    'LineWidth', 2, 'MaxHeadSize', 3, 'Color', [0.95 0.95 0.95]);

%% ================= CAPTION + TELEMETRY =================
xRange = xmax - xmin; zRange = zmax - zmin;

capX = xmin + 0.02*xRange; capY = zmin + 0.02*zRange;
capW = 0.82*xRange;        capH = 0.12*zRange;
telX = capX;               telY = capY + capH + 0.012*zRange;
telW = 0.50*xRange;        telH = 0.18*zRange;

captionRect = rectangle(ax,'Position',[capX capY capW capH], ...
    'FaceColor',BOX.face,'EdgeColor',BOX.border,'LineWidth',BOX.borderW);
captionRect.FaceAlpha = BOX.captionAlpha;

telemetryRect = rectangle(ax,'Position',[telX telY telW telH], ...
    'FaceColor',BOX.face,'EdgeColor',BOX.border,'LineWidth',BOX.borderW);
telemetryRect.FaceAlpha = BOX.telemetryAlpha;

captionText = text(ax, capX + 0.012*xRange, capY + 0.012*zRange, '', ...
    'Color',COL.white,'FontSize',12,'FontWeight','bold', ...
    'VerticalAlignment','top','Interpreter','none');

telemetryText = text(ax, telX + 0.012*xRange, telY + 0.012*zRange, '', ...
    'Color',COL.white,'FontSize',10, ...
    'VerticalAlignment','top','Interpreter','none');

%% ================= VIDEO WRITER =================
if OPT.makeVideo
    vw = VideoWriter(OPT.videoName,'MPEG-4');
    vw.FrameRate = OPT.fps;
    open(vw);
end

%% ================= ANIMATE =================
K = numel(t);

for k = 1:OPT.skip:K
    tk = t(k);

    th = theta(k);
    R  = [cos(th) -sin(th); sin(th) cos(th)];

    pts = (R * capsuleLocal.').';
    Xp = pts(:,1) + xp(k);
    Zp = pts(:,2) + zp(k);
    set(auvPatch,'XData',Xp,'YData',Zp);

    % Nose triangle oriented with theta at nose tip
    noseW = (R * noseTipLocal.').';
    nx = noseW(1) + xp(k);
    nz = noseW(2) + zp(k);

    fwd = [cos(th), sin(th)];
    left = [-sin(th), cos(th)];

    triSize = 0.18;
    p1 = [nx, nz];
    p2 = p1 - triSize*fwd + 0.7*triSize*left;
    p3 = p1 - triSize*fwd - 0.7*triSize*left;

    set(noseTri,'XData',[p1(1) p2(1) p3(1)], 'YData',[p1(2) p2(2) p3(2)]);

    dx = OPT.arrowScale_m_per_N * dhat_plot(k);
    if OPT.flipDistArrow, dx = -dx; end
    set(distArrow,'XData',xp(k),'YData',zp(k),'UData',dx,'VData',0);

    disturbanceNowActive = (tk >= DIST_STEP_TIME);
    inStepWindow = abs(tk - DIST_STEP_TIME) <= 1.5;

    nearDepthConstraint = (zp(k) < C.zp_min + THR.depthWarn) || (zp(k) > C.zp_max - THR.depthWarn);
    speedRecovered = abs(uSur(k) - u_ref) < THR.uRecover;
    dhatActive = abs(dhat_plot(k)) > THR.dHatActive;

    if inStepWindow
        headline = 'Disturbance now active: unknown opposing surge force applied.';
    elseif disturbanceNowActive && dhatActive && ~speedRecovered
        headline = 'Observer estimating disturbance; offset-free MPC compensating.';
    elseif disturbanceNowActive && speedRecovered
        headline = 'Compensation successful: speed recovered with ~zero steady-state error.';
    else
        headline = 'Nominal tracking: MPC regulating speed and depth within constraints.';
    end

    if nearDepthConstraint
        conTxt = 'Depth constraint active: staying inside safe band.';
    else
        conTxt = 'Constraints satisfied: depth within safe band.';
    end

    if disturbanceNowActive
        distTxt = sprintf('dHat=%.1f N, uSS_surge=%.1f', dhat_plot(k), uss_plot(1,k));
    else
        distTxt = sprintf('Awaiting disturbance step at t=%.0f s', DIST_STEP_TIME);
    end

    captionText.String = sprintf('%s\n%s  %s', headline, conTxt, distTxt);

    telemetryText.String = sprintf([ ...
        't=%.1fs   x=%.2fm   z=%.2fm\n' ...
        'u=%.2fm/s   w=%.2fm/s\n' ...
        'pitch=%.1fdeg   dHat=%.1fN\n' ...
        'uSS_surge=%.1f'], ...
        tk, xp(k), zp(k), uSur(k), wHea(k), rad2deg(th), dhat_plot(k), uss_plot(1,k));

    drawnow;

    if OPT.makeVideo
        fr = getframe(fig);
        for rHold = 1:OPT.frameHold
            writeVideo(vw, fr);
        end
    end
end

if OPT.makeVideo
    close(vw);
    fprintf('Saved video: %s\n', fullfile(pwd, OPT.videoName));
end

end
