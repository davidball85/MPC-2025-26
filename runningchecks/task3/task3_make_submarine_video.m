
function task3_make_submarine_video_from_logs_SURFACEWAVE_FIXED(logsIn)
% TASK3_MAKE_SUBMARINE_VIDEO_FROM_LOGS_SURFACEWAVE_FIXED
%
% Makes the Task 3 submarine animation WITHOUT re-running task3_main.
% This version draws a SINGLE moving sine-wave at the water surface (y=0 m),
% as requested, and fixes the gradient sizing + "no video frames" issue.
%
% Usage:
%   logs = task3_main();
%   save('task3_logs.mat','logs');
%   task3_make_submarine_video_from_logs_SURFACEWAVE_FIXED('task3_logs.mat')
%
% Or:
%   task3_make_submarine_video_from_logs_SURFACEWAVE_FIXED(logs)

% ------------------------------------------------------------
% USER CONFIGURATION
% ------------------------------------------------------------
CFG.videoName     = 'task3_submarine_animation.mp4';
CFG.frameRate     = 30;        % video FPS
CFG.playbackRate  = 0.5;       % 1 = real time, 0.5 = slower (more frames written)
CFG.showTelemetry = true;
CFG.showSubtitles = true;

% Water background (used for figure bg behind axes)
CFG.figureBG      = [0.15 0.15 0.15];

% Underwater vertical gradient (axes background)
CFG.waterTopColor    = [0.06 0.18 0.38];   % lighter near surface
CFG.waterBottomColor = [0.01 0.05 0.14];   % darker at depth

CFG.subColor      = [0.85 0.85 0.90];

CFG.subScale      = 2;
CFG.subLength     = 2.0 * CFG.subScale;
CFG.subHeight     = 0.5 * CFG.subScale;

CFG.windowMarginX = 15;    % metres around vehicle
CFG.windowMarginZ = 6;

% Wall styling
CFG.wallLineW     = 1.0;
CFG.wallDashW     = 0.9;
CFG.wallSolidCol  = [0.85 0.15 0.15];  % red
CFG.wallDashCol   = [0.15 0.75 0.30];  % green
CFG.wallLabelFS   = 12;
CFG.wallLabelDy   = 0.20;  % vertical offset factor (in y-span)
CFG.wallLabelDx   = 1.20;  % metres
CFG.wallLabelSep  = 0.07;  % extra y-span offset to separate labels

% Subtitles box
CFG.subBoxPos     = [0.14 0.02 0.72 0.14];
CFG.subBoxFS      = 16;

% Telemetry box
CFG.telemetryPos  = [0.02 0.68 0.18 0.17]; % [x y w h] (normalised figure units)
CFG.telemetryFS   = 9;

% Figure size in pixels (LOCKED to avoid frame-size mismatch)
CFG.figPosPixels  = [100 100 1800 750];

% Surface wave (SINGLE line at y=0)
WAV.amp   = 0.18;   % metres (make this bigger/smaller)
WAV.k     = 0.65;   % rad/m (more ripples -> bigger k)
WAV.w     = 0.90;   % rad/s (speed)
WAV.nPts  = 900;

% ------------------------------------------------------------
% LOAD LOGS (no re-sim)
% ------------------------------------------------------------
if nargin < 1 || isempty(logsIn)
    logsIn = 'task3_logs.mat';
end

if isstruct(logsIn)
    logs = logsIn;
else
    S = load(logsIn);
    if isfield(S,'logs')
        logs = S.logs;
    else
        error('MAT file did not contain a variable called "logs".');
    end
end

t     = logs.t(:);
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

% Wall values
wall_xp = [];
wall_stop = [];
if isfield(C,'wall_xp'),   wall_xp   = C.wall_xp;   end
if isfield(C,'wall_stop'), wall_stop = C.wall_stop; end
if isempty(wall_stop) && isfield(C,'wall_stop_internal'), wall_stop = C.wall_stop_internal; end

% ------------------------------------------------------------
% SUBMARINE GEOMETRY (body-frame)
% ------------------------------------------------------------
th = linspace(0,2*pi,80);
ovalX = (CFG.subLength/2) * cos(th);
ovalY = (CFG.subHeight/2) * sin(th);

% Sail/spout on top
sailW = 0.18 * CFG.subLength;
sailH = 0.45 * CFG.subHeight;
sailX = [-sailW/2,  sailW/2,  sailW/2, -sailW/2];
sailY = [-CFG.subHeight/2, -CFG.subHeight/2, -CFG.subHeight/2 - sailH, -CFG.subHeight/2 - sailH];

% Tail (simple triangle)
tailBaseX = -CFG.subLength/2;
tailLen   = 0.20 * CFG.subLength;
tailH     = 0.22 * CFG.subHeight;
tailX = [tailBaseX, tailBaseX - tailLen, tailBaseX];
tailY = [-tailH, 0, tailH];

% Nose offset: treat xp as the NOSE position rather than the centre
noseOffsetX = +CFG.subLength/2;

% ------------------------------------------------------------
% FIGURE SETUP (LOCKED SIZE)
% ------------------------------------------------------------
fig = figure('Color',CFG.figureBG, 'MenuBar','none', 'ToolBar','none');
set(fig,'Units','pixels');
set(fig,'Position',CFG.figPosPixels);
set(fig,'Resize','off');          % keep constant frame size
set(fig,'Renderer','opengl');

ax = axes(fig);
hold(ax,'on');
axis(ax,'equal');
axis(ax,'manual');
set(ax,'XColor','w','YColor','w');
xlabel(ax,'Horizontal position (m)');
ylabel(ax,'Depth (m)');
set(ax,'YDir','reverse');

xlim(ax,[min(xp)-CFG.windowMarginX, max(xp)+CFG.windowMarginX]);
ylim(ax,[min(zp)-CFG.windowMarginZ, max(zp)+CFG.windowMarginZ]);

% Grid (make it readable)
grid(ax,'on');
ax.XMinorGrid = 'on';
ax.YMinorGrid = 'on';
ax.GridColor      = [1 1 1];
ax.MinorGridColor = [1 1 1];
ax.GridAlpha      = 0.35;
ax.MinorGridAlpha = 0.18;
ax.GridLineStyle      = '-';
ax.MinorGridLineStyle = ':';
ax.Layer = 'top';

yL = ylim(ax);
ySpan = yL(2) - yL(1);
labelY = yL(1) + CFG.wallLabelDy*ySpan;

% ------------------------------------------------------------
% Underwater gradient background (draw as an image behind everything)
% ------------------------------------------------------------
% Build an HxW RGB image (no toolboxes, no implicit expansion assumptions)
Wbg = 1200;
Hbg = 500;
a = linspace(0,1,Hbg)';            % 0 at top, 1 at bottom (Hbg x 1)
A = repmat(a, 1, Wbg);             % Hbg x Wbg

img = zeros(Hbg, Wbg, 3);
for cch = 1:3
    img(:,:,cch) = (1-A)*CFG.waterTopColor(cch) + A*CFG.waterBottomColor(cch);
end

xL = xlim(ax);
yL = ylim(ax);

hBG = image(ax, [xL(1) xL(2)], [yL(1) yL(2)], img);
uistack(hBG,'bottom');

% ------------------------------------------------------------
% Surface wave line at y=0 (single moving sine wave)
% ------------------------------------------------------------
xWave = linspace(xL(1)-20, xL(2)+20, WAV.nPts);
yWave0 = zeros(size(xWave));

hWave = plot(ax, xWave, yWave0, 'Color',[0.80 0.92 1.00], 'LineWidth',2.0);
uistack(hWave,'top');

% ------------------------------------------------------------
% SUBMARINE PATCHES (opaque)
% ------------------------------------------------------------
xCentre0 = xp(1) - noseOffsetX;
hSub  = fill(ovalX + xCentre0, ovalY + zp(1), CFG.subColor, 'EdgeColor','k','LineWidth',1.2);
hSail = fill(sailX + xCentre0, sailY + zp(1), CFG.subColor, 'EdgeColor','k','LineWidth',1.0);
hTail = fill(tailX + xCentre0, tailY + zp(1), CFG.subColor, 'EdgeColor','k','LineWidth',1.0);
set([hSub, hSail, hTail], 'FaceAlpha', 1.0);

uistack(hTail,'top'); uistack(hSub,'top'); uistack(hSail,'top');

% ------------------------------------------------------------
% VIRTUAL WALLS (on top, spaced labels)
% ------------------------------------------------------------
if ~isempty(wall_xp)
    hWallXP = plot(ax, [wall_xp wall_xp], yL, '-', 'LineWidth',CFG.wallLineW, 'Color',CFG.wallSolidCol);
    uistack(hWallXP,'top');
    text(wall_xp + CFG.wallLabelDx, labelY, sprintf('wall\\_xp = %.1f m', wall_xp), ...
        'Color',CFG.wallSolidCol, 'FontWeight','bold', 'FontSize',CFG.wallLabelFS, ...
        'Parent',ax, 'Clipping','on', 'HorizontalAlignment','left');
end

if ~isempty(wall_stop)
    hWallStop = plot(ax, [wall_stop wall_stop], yL, '--', 'LineWidth',CFG.wallDashW, 'Color',CFG.wallDashCol);
    uistack(hWallStop,'top');
    text(wall_stop - CFG.wallLabelDx, labelY + CFG.wallLabelSep*ySpan, sprintf('wall\\_stop = %.1f m', wall_stop), ...
        'Color',CFG.wallDashCol, 'FontWeight','bold', 'FontSize',CFG.wallLabelFS, ...
        'Parent',ax, 'Clipping','on', 'HorizontalAlignment','right');
end

% ------------------------------------------------------------
% TELEMETRY BOX (styled, centred above plot)
% ------------------------------------------------------------
if CFG.showTelemetry
    hTelem = annotation('textbox', ...
        [0.37 0.75 0.30 0.22], ...   % [x y w h] – centred & larger
        'Color','w', ...
        'EdgeColor',[0.7 0.7 0.7], ...
        'LineWidth',1.2, ...
        'FontSize',11, ...
        'FontName','Helvetica', ...
        'Interpreter','none', ...
        'HorizontalAlignment','left', ...
        'VerticalAlignment','middle', ...
        'BackgroundColor',[0 0 0 0.55]);   % softer transparency
end


% ------------------------------------------------------------
% SUBTITLES
% ------------------------------------------------------------
if CFG.showSubtitles
    hSubText = annotation('textbox',CFG.subBoxPos, ...
        'Color','w','EdgeColor','none', ...
        'HorizontalAlignment','center', ...
        'VerticalAlignment','middle', ...
        'FontSize',CFG.subBoxFS,'FontWeight','bold', ...
        'BackgroundColor',[0 0 0 0.45]);
end

drawnow; % layout before video starts

% ------------------------------------------------------------
% VIDEO SETUP (open AFTER figure is stable)
% ------------------------------------------------------------
vw = VideoWriter(CFG.videoName,'MPEG-4');
vw.FrameRate = CFG.frameRate;
open(vw);
cleanupVW = onCleanup(@() safeCloseVideoWriter(vw, CFG.videoName)); %#ok<NASGU>

skip = max(1, round((1/CFG.frameRate)/Ts * CFG.playbackRate));

% First frame determines output resolution (write an IMAGE frame)
fr0 = getframe(fig);
img0 = fr0.cdata;
H0 = size(img0,1);
W0 = size(img0,2);
writeVideo(vw, img0);

% ------------------------------------------------------------
% MAIN LOOP
% ------------------------------------------------------------
refU = 1.0;
refZ = 5.0;

for k = 1+skip:skip:K

    % Wave motion (at y=0)
    tk = t(k);
    yWave = WAV.amp * sin(WAV.k*xWave + WAV.w*tk);
    set(hWave,'YData', yWave);

    % Sub centre from nose position
    xCentre = xp(k) - noseOffsetX;

    set(hSub,  'XData', ovalX + xCentre, 'YData', ovalY + zp(k));
    set(hSail, 'XData', sailX + xCentre, 'YData', sailY + zp(k));
    set(hTail, 'XData', tailX + xCentre, 'YData', tailY + zp(k));

    % Telemetry
    if CFG.showTelemetry
        kk = max(1, min(size(u,2), k-1));
        
        telemStr = sprintf([ ...
    '⏱  Time: %.1f s\n' ...
    '────────────────────────────\n' ...
    '📍  x_p (nose): %.2f m\n' ...
    '⚡  u (surge): %.3f m/s\n' ...
    '⬇️  z_p (depth): %.2f m\n\n' ...
    '🔧  Thrust: %.1f N\n' ...
    '🌊  Disturbance: %.1f N (true)\n' ...
    '🧠  Estimate: %.1f N'], ...
    t(k), xp(k), uvel(k), zp(k), ...
    u(1,kk), dtrue(min(k,end)), dhat(min(k,end)));


        set(hTelem,'String',telemStr);
    end

    % Subtitles
    if CFG.showSubtitles
        if t(k) < C.dist.surge_step_time_s
            if stopAtWallEnabled
                subtitle = sprintf('Tracking u=%.1f m/s and z_p=%.1f m. Stop-at-wall is enabled.', refU, refZ);
            else
                subtitle = sprintf('Tracking u=%.1f m/s and z_p=%.1f m under nominal conditions.', refU, refZ);
            end
        elseif abs(t(k) - C.dist.surge_step_time_s) < Ts
            subtitle = 'A surge disturbance is applied; the augmented observer begins estimating the bias.';
        else
            if stopAtWallEnabled && ~isempty(wall_stop) && (wall_stop - xp(k)) <= 2.0
                subtitle = 'Approaching the virtual wall: speed reference is ramped down to stop smoothly at the boundary.';
            else
                subtitle = 'Offset-free MPC rejects the disturbance while maintaining depth; speed tracking remains regulated.';
            end
        end
        set(hSubText,'String',subtitle);
    end

    drawnow limitrate;

    fr = getframe(fig);
    img = fr.cdata;

    % Enforce constant frame size
    if size(img,1) ~= H0 || size(img,2) ~= W0
        img = padCropFrame(img, H0, W0);
    end

    writeVideo(vw, img);
end

safeCloseVideoWriter(vw, CFG.videoName);

fprintf('\nVideo written to:\n  %s\n', CFG.videoName);
fprintf('Done.\n');

end

% =====================================================================
function imgOut = padCropFrame(imgIn, H, W)
% Pads/crops an RGB image to exactly HxW (no toolboxes).

imgOut = zeros(H, W, 3, 'uint8');

hin = size(imgIn,1);
win = size(imgIn,2);

row0 = 1; col0 = 1;
if hin > H, row0 = floor((hin - H)/2) + 1; end
if win > W, col0 = floor((win - W)/2) + 1; end

row1 = min(hin, row0 + H - 1);
col1 = min(win, col0 + W - 1);

crop = imgIn(row0:row1, col0:col1, :);

h = size(crop,1);
w = size(crop,2);
imgOut(1:h, 1:w, :) = crop;

end

% =====================================================================
function safeCloseVideoWriter(vw, videoName)
% Close writer safely and confirm output.

try
    close(vw);
catch
    % ignore close errors
end

if exist(videoName,'file') == 2
    d = dir(videoName);
    fprintf('MP4 saved: %s (%.1f MB)\n', videoName, d.bytes/1e6);
else
    warning(['Video file was not created in the current working folder.\n' ...
             'Try setting CFG.videoName to a full path (e.g., fullfile(pwd, ...)).']);
end
end
