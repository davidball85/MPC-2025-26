function task3_make_submarine_video()
% TASK3_MAKE_SUBMARINE_VIDEO
%
% Produces a polished 2D side-view animation of the AUV for Task 3.
% Uses task3_main logs as the single source of truth.
%
% Features:
%  - Dark blue "water" background
%  - Wavy surface
%  - Rigid submarine silhouette (oval) + sail + tail
%  - Telemetry info box
%  - Narration subtitles (large box)
%  - MP4 output via VideoWriter
%
% ------------------------------------------------------------
% USER CONFIGURATION
% ------------------------------------------------------------

CFG.videoName     = 'task3_submarine_animation.mp4';
CFG.frameRate     = 30;        % video FPS
CFG.playbackRate  = 1.0;       % 1 = real time, 0.5 = slower
CFG.showTelemetry = true;
CFG.showSubtitles = true;

% Slightly darker water (you asked for "just a LITTLE darker")
CFG.waterColor    = [0.035 0.105 0.26];

CFG.subColor      = [0.85 0.85 0.90];
CFG.surfaceColor  = [0.70 0.85 1.00];

CFG.subScale      = 2;
CFG.subLength     = 2.0 * CFG.subScale;
CFG.subHeight     = 0.5 * CFG.subScale;

CFG.windowMarginX = 15;    % metres around vehicle
CFG.windowMarginZ = 6;

% Wall styling
CFG.wallColor     = [1.00 0.60 0.10]; % orange
CFG.wallLineW     = 2.5;
CFG.wallDashW     = 2.0;

% ------------------------------------------------------------
% RUN SIMULATION (ONCE)
% ------------------------------------------------------------
clc;
fprintf('Running task3_main...\n');
logs = task3_main();

t     = logs.t;
x     = logs.x;
u     = logs.u;
dhat  = logs.dhat;
dtrue = logs.d_true;

C = config_constants();
ixp = C.idx.xp;
izp = C.idx.zp;
iu  = C.idx.u;

xp   = x(ixp,:);
zp   = x(izp,:);
uvel = x(iu,:);

Ts = t(2)-t(1);
K  = numel(t);

stopAtWallEnabled = isfield(C,'Task3') && isfield(C.Task3,'stop_at_wall_enable') && C.Task3.stop_at_wall_enable;

% ------------------------------------------------------------
% SUBMARINE GEOMETRY (RIGID, body-frame)
% ------------------------------------------------------------
theta = linspace(0,2*pi,60);
ovalX = (CFG.subLength/2) * cos(theta);
ovalY = (CFG.subHeight/2) * sin(theta);

% Small sail/spout on top (body frame)
sailW = 0.18 * CFG.subLength;
sailH = 0.45 * CFG.subHeight;
sailX = [-sailW/2,  sailW/2,  sailW/2, -sailW/2];
sailY = [-CFG.subHeight/2, -CFG.subHeight/2, -CFG.subHeight/2 - sailH, -CFG.subHeight/2 - sailH];

% Tail geometry (shallow trapezoid, body frame)
tailBaseX = -CFG.subLength/2;      % rear of hull
tailLen   = 0.22 * CFG.subLength;  % tail length
tailH1    = 0.18 * CFG.subHeight;  % hull-side height
tailH2    = 0.05 * CFG.subHeight;  % tail tip height

tailX = [tailBaseX, tailBaseX - tailLen, tailBaseX - tailLen, tailBaseX];
tailY = [-tailH1,   -tailH2,          tailH2,          tailH1];

% ------------------------------------------------------------
% VIDEO SETUP (robust)
% ------------------------------------------------------------
vw = VideoWriter(CFG.videoName,'MPEG-4');
vw.FrameRate = CFG.frameRate;

try
    open(vw);
catch ME
    error(['Could not open VideoWriter. Common causes:\n' ...
           '  - No write permission in this folder\n' ...
           '  - videoName path invalid\n' ...
           '  - Codec not available\n\n' ...
           'MATLAB says: %s'], ME.message);
end

% GUARANTEE the writer closes even if an error happens mid-loop
cleanupVW = onCleanup(@() safeCloseVideoWriter(vw, CFG.videoName));

skip = max(1, round((1/CFG.frameRate)/Ts * CFG.playbackRate));

% ------------------------------------------------------------
% FIGURE SETUP (ONCE)
% ------------------------------------------------------------
fig = figure('Color',CFG.waterColor);
set(fig,'Position',[100 100 1200 500]);

ax = axes(fig);
hold(ax,'on');
axis(ax,'equal');
axis(ax,'manual');
set(ax,'Color',CFG.waterColor);
set(ax,'XColor','w','YColor','w');
xlabel(ax,'Horizontal position (m)');
ylabel(ax,'Depth (m)');
set(ax,'YDir','reverse');

xlim(ax,[min(xp)-CFG.windowMarginX, max(xp)+CFG.windowMarginX]);
ylim(ax,[min(zp)-CFG.windowMarginZ, max(zp)+CFG.windowMarginZ]);

% Turn on grid (your preference)
grid(ax,'on');
ax.GridAlpha = 0.18;
ax.MinorGridAlpha = 0.08;
ax.XMinorGrid = 'on';
ax.YMinorGrid = 'on';

% Cache y-limits for wall drawing
yL = ylim(ax);

% ------------------------------------------------------------
% WAVY SURFACE (closed polygon)
% ------------------------------------------------------------
xs = linspace(min(xp)-100, max(xp)+100, 400);
ys = 0.2*sin(0.1*xs) + 0.1*sin(0.35*xs);

zBottom = yL(2) + 5;                 % safely below view (remember YDir is reverse)
Xpoly   = [xs, fliplr(xs)];
Ypoly   = [ys, zBottom*ones(size(xs))];

hSurface = fill(ax, Xpoly, Ypoly, CFG.surfaceColor, 'EdgeColor','none');

% IMPORTANT: surface should be visible but not cover overlays
uistack(hSurface,'bottom');

% ------------------------------------------------------------
% SUBMARINE PATCHES
% ------------------------------------------------------------
hSub  = fill(ovalX + xp(1), ovalY + zp(1), CFG.subColor, 'EdgeColor','k','LineWidth',1.2);
hSail = fill(sailX + xp(1), sailY + zp(1), CFG.subColor, 'EdgeColor','k','LineWidth',1.0);
hTail = fill(tailX + xp(1), tailY + zp(1), CFG.subColor, 'EdgeColor','k','LineWidth',1.0);

uistack(hTail,'top');
uistack(hSub,'top');
uistack(hSail,'top');

% ------------------------------------------------------------
% VIRTUAL WALL LINES (draw once, on top of water)
% ------------------------------------------------------------
Ccfg = config_constants();

yL = ylim(ax);
ySpan = yL(2) - yL(1);

labelY = yL(1) + 0.10*ySpan;   % safely above seabed, below surface
labelDx = 0.6;                % horizontal text offset (m)

% ---- Physical wall (hard constraint) ----
if isfield(Ccfg,'wall_xp')
    hWallPhysical = plot(ax, ...
        [Ccfg.wall_xp Ccfg.wall_xp], yL, ...
        'LineStyle','-', ...
        'LineWidth',1.4, ...
        'Color',[0.85 0.15 0.15]);   % red

    uistack(hWallPhysical,'top');

    text(Ccfg.wall_xp + labelDx, labelY, ...
        sprintf('wall\\_xp = %.1f m', Ccfg.wall_xp), ...
        'Color',[0.95 0.25 0.25], ...
        'FontWeight','bold', ...
        'FontSize',11, ...
        'Parent',ax, ...
        'Clipping','on');
end

% ---- Stop / braking wall ----
if isfield(Ccfg,'wall_stop')
    hWallStop = plot(ax, ...
        [Ccfg.wall_stop Ccfg.wall_stop], yL, ...
        'LineStyle','--', ...
        'LineWidth',1.2, ...
        'Color',[0.15 0.75 0.30]);   % green

    uistack(hWallStop,'top');

    text(Ccfg.wall_stop - labelDx, labelY, ...
        sprintf('wall\\_stop = %.1f m', Ccfg.wall_stop), ...
        'Color',[0.20 0.85 0.35], ...
        'FontWeight','bold', ...
        'FontSize',11, ...
        'HorizontalAlignment','right', ...
        'Parent',ax, ...
        'Clipping','on');
end

% ------------------------------------------------------------
% TELEMETRY BOX
% ------------------------------------------------------------
if CFG.showTelemetry
    hTelem = annotation('textbox',[0.02 0.55 0.25 0.35], ...
        'Color','w','EdgeColor','w','FontSize',11, ...
        'Interpreter','none','BackgroundColor',[0 0 0 0.40]);
end

% ------------------------------------------------------------
% SUBTITLES (bigger box)
% ------------------------------------------------------------
if CFG.showSubtitles
    hSubText = annotation('textbox',[0.14 0.02 0.72 0.14], ...
        'Color','w','EdgeColor','none', ...
        'HorizontalAlignment','center', ...
        'VerticalAlignment','middle', ...
        'FontSize',16,'FontWeight','bold', ...
        'BackgroundColor',[0 0 0 0.45]);
end

% ------------------------------------------------------------
% MAIN ANIMATION LOOP
% ------------------------------------------------------------
for k = 1:skip:K

    % Move submarine
    set(hSub,  'XData', ovalX + xp(k), 'YData', ovalY + zp(k));
    set(hSail, 'XData', sailX + xp(k), 'YData', sailY + zp(k));
    set(hTail, 'XData', tailX + xp(k), 'YData', tailY + zp(k));

    % Update telemetry
    if CFG.showTelemetry
        telemStr = sprintf([ ...
            'Time: %.1f s\n' ...
            'u (surge speed): %.3f m/s\n' ...
            'Depth z_p: %.2f m\n' ...
            'Surge thrust: %.1f N\n' ...
            'Disturbance true: %.1f N\n' ...
            'Disturbance est: %.1f N'], ...
            t(k), uvel(k), zp(k), u(1,max(1,k-1)), dtrue(k), dhat(k));
        set(hTelem,'String',telemStr);
    end

    % Subtitles
    if CFG.showSubtitles

        if t(k) < C.dist.surge_step_time_s
            if stopAtWallEnabled
                subtitle = 'Tracking u=1 m/s and z_p=5 m. Stop-at-wall is enabled (vehicle will brake smoothly at the virtual wall).';
            else
                subtitle = 'Tracking u=1 m/s and z_p=5 m under nominal conditions.';
            end

        elseif abs(t(k) - C.dist.surge_step_time_s) < Ts
            subtitle = 'A sudden external surge disturbance is applied; the augmented observer begins estimating the bias.';

        else
            if stopAtWallEnabled && ~isempty(wall_stop)
                distToWall = wall_stop - xp(k);

                if distToWall <= 2.0
                    subtitle = 'Approaching the virtual wall: speed reference is ramped down to achieve a smooth stop at the constraint boundary.';
                else
                    subtitle = 'Offset-free MPC rejects the disturbance while maintaining depth; speed tracking remains regulated.';
                end
            else
                subtitle = 'Offset-free MPC rejects the disturbance while maintaining depth; speed tracking remains regulated.';
            end
        end

        set(hSubText,'String',subtitle);
    end

    drawnow limitrate;

    % Write frame (this is what makes the MP4)
    writeVideo(vw, getframe(fig));
end

% Explicit close happens via onCleanup too, but do it anyway
safeCloseVideoWriter(vw, CFG.videoName);

fprintf('\nVideo written to:\n  %s\n', CFG.videoName);
fprintf('Done.\n');

end

% =====================================================================
function safeCloseVideoWriter(vw, videoName)
% Close writer safely and tell user where it went.

try
    if strcmp(vw.VideoCompressionMethod,'None') %#ok<STISA>
        % no-op
    end
catch
    % ignore
end

try
    close(vw);
catch
    % ignore close errors
end

% Confirm file exists (helps debug the "played live but no file" case)
if exist(videoName,'file') == 2
    d = dir(videoName);
    fprintf('MP4 saved: %s (%.1f MB)\n', videoName, d.bytes/1e6);
else
    warning(['Video file was not created in the current working folder.\n' ...
             'Likely causes:\n' ...
             '  1) You ran from a folder you dont have write access to.\n' ...
             '  2) videoName path points somewhere unexpected.\n' ...
             '  3) VideoWriter failed before close().\n' ...
             'Try running:\n' ...
             '  fprintf("PWD: %s\\n", pwd);\n' ...
             'and set CFG.videoName to a full path (e.g., fullfile(pwd, ...)).']);
end
end
