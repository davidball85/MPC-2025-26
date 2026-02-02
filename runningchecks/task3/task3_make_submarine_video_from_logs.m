function task3_make_submarine_video_from_logs(logsIn)
% TASK3_MAKE_SUBMARINE_VIDEO_FROM_LOGS
%
% Makes the Task 3 submarine animation WITHOUT re-running task3_main.
%
% Usage:
%   % After you run task3_main once:
%   logs = task3_main();
%   save('task3_logs.mat','logs');
%
%   % Then generate the video from the saved logs:
%   task3_make_submarine_video_from_logs('task3_logs.mat')
%
%   % Or pass the logs struct directly:
%   task3_make_submarine_video_from_logs(logs)
%
% Notes:
%   - This script never calls task3_main.
%   - It animates the surface (gentle ripple) by phase-drifting the waves.
%   - It avoids frame-size errors by writing image frames (HxWx3) and padding/
%     cropping every frame to match the very first one.
%
% ------------------------------------------------------------
% USER CONFIGURATION
% ------------------------------------------------------------
CFG.videoName     = 'task3_submarine_animation.mp4';
CFG.frameRate     = 30;        % video FPS
CFG.playbackRate  = 0.5;       % 1 = real time, 0.5 = slower
CFG.showTelemetry = true;
CFG.showSubtitles = true;

% Slightly darker water
CFG.waterColor    = [0.035 0.105 0.26];
CFG.waterTopColor    = [0.06 0.18 0.38];   % lighter near surface
CFG.waterBottomColor = [0.01 0.05 0.14];   % darker at depth


CFG.subColor      = [0.85 0.85 0.90];
CFG.surfaceColor  = [0.70 0.85 1.00];

CFG.subScale      = 2;
CFG.subLength     = 2.0 * CFG.subScale;
CFG.subHeight     = 0.5 * CFG.subScale;

CFG.windowMarginX = 15;    % metres around vehicle
CFG.windowMarginZ = 6;

% Wall styling (requested)
CFG.wallLineW     = 1.0;
CFG.wallDashW     = 0.9;
CFG.wallSolidCol  = [0.85 0.15 0.15];  % red
CFG.wallDashCol   = [0.15 0.75 0.30];  % green
CFG.wallLabelFS   = 12;
CFG.wallLabelDy   = 0.20;  % vertical offset factor (in y-span)
CFG.wallLabelDx   = 1.20;  % metres
CFG.wallLabelSep  = 0.07;  % extra y-span offset to separate labels

% Subtitles (bigger box)
CFG.subBoxPos     = [0.14 0.02 0.72 0.14];
CFG.subBoxFS      = 16;

% Figure size in pixels (lock this to prevent frame-size mismatch)
CFG.figPosPixels  = [100 100 1800 750];

% Surface wave settings (bigger amplitude + visible ripples + motion)
WAV.A1 = 0.16;
WAV.A2 = 0.11;
WAV.A3 = 0.07;
WAV.k1 = 0.50;
WAV.k2 = 1.10;
WAV.k3 = 2.00;
WAV.p2 = 0.40;
WAV.p3 = 1.20;

% Phase drift speeds (rad/s) – keep gentle
WAV.w1 = 0.35;
WAV.w2 = 0.60;
WAV.w3 = 0.90;

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

% Wall values (used in subtitles too)
wall_xp = [];
wall_stop = [];
if isfield(C,'wall_xp'),   wall_xp   = C.wall_xp;   end
if isfield(C,'wall_stop'), wall_stop = C.wall_stop; end
if isempty(wall_stop) && isfield(C,'wall_stop_internal'), wall_stop = C.wall_stop_internal; end

% ------------------------------------------------------------
% SUBMARINE GEOMETRY (RIGID, body-frame)
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
tailBaseX = -CFG.subLength/2;      % rear of hull
tailLen   = 0.20 * CFG.subLength;  % tail length
tailH     = 0.22 * CFG.subHeight;  % half-height at base
tailX = [tailBaseX, tailBaseX - tailLen, tailBaseX];
tailY = [-tailH, 0, tailH];

% ------------------------------------------------------------
% VIDEO SETUP (robust close)
% ------------------------------------------------------------
vw = VideoWriter(CFG.videoName,'MPEG-4');
vw.FrameRate = CFG.frameRate;
open(vw);
cleanupVW = onCleanup(@() safeCloseVideoWriter(vw, CFG.videoName)); %#ok<NASGU>

skip = max(1, round((1/CFG.frameRate)/Ts * CFG.playbackRate));

% ------------------------------------------------------------
% FIGURE SETUP (LOCKED SIZE)
% ------------------------------------------------------------
fig = figure('Color',CFG.waterColor, 'MenuBar','none', 'ToolBar','none');
set(fig,'Units','pixels');
set(fig,'Position',CFG.figPosPixels);
set(fig,'Resize','off');   % keep constant frame size
set(fig,'Renderer','opengl');

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

grid(ax,'on');
ax.XMinorGrid = 'on';
ax.YMinorGrid = 'on';

ax.GridColor      = [1 1 1];     % bright grid
ax.MinorGridColor = [1 1 1];

ax.GridAlpha      = 0.35;        % stronger major grid
ax.MinorGridAlpha = 0.18;        % stronger minor grid

ax.GridLineStyle      = '-';
ax.MinorGridLineStyle = ':';
ax.Layer  = 'top'

yL = ylim(ax);
ySpan = yL(2) - yL(1);
labelY = yL(1) + CFG.wallLabelDy*ySpan;

% ------------------------------------------------------------
% WAVY SURFACE (behind overlays) – create patch once
% ------------------------------------------------------------
xs = linspace(min(xp)-100, max(xp)+100, 800);

% initial surface
ys = WAV.A1*sin(WAV.k1*xs) ...
   + WAV.A2*sin(WAV.k2*xs + WAV.p2) ...
   + WAV.A3*sin(WAV.k3*xs + WAV.p3);

zBottom = yL(2) + 5;
Xpoly   = [xs, fliplr(xs)];
Ypoly   = [ys, zBottom*ones(size(xs))];

hSurface = fill(ax, Xpoly, Ypoly, CFG.surfaceColor, 'EdgeColor','none');
set(hSurface, 'FaceAlpha', 0.88); 
uistack(hSurface,'bottom');

% ------------------------------------------------------------
% SUBMARINE PATCHES
% ------------------------------------------------------------
hSub  = fill(ovalX + xp(1), ovalY + zp(1), CFG.subColor, 'EdgeColor','k','LineWidth',1.2);
hSail = fill(sailX + xp(1), sailY + zp(1), CFG.subColor, 'EdgeColor','k','LineWidth',1.0);
hTail = fill(tailX + xp(1), tailY + zp(1), CFG.subColor, 'EdgeColor','k','LineWidth',1.0);

set([hSub, hSail, hTail], 'FaceAlpha', 1.0);   % fully opaque submarine

uistack(hSurface,'bottom');
uistack(hTail,'top');
uistack(hSub,'top');
uistack(hSail,'top');

% ------------------------------------------------------------
% VIRTUAL WALLS (on top, spaced labels)
% ------------------------------------------------------------
if ~isempty(wall_xp)
    hWallXP = plot(ax, [wall_xp wall_xp], yL, '-', ...
        'LineWidth',CFG.wallLineW, 'Color',CFG.wallSolidCol);
    uistack(hWallXP,'top');

    text(wall_xp + CFG.wallLabelDx, labelY, ...
        sprintf('wall\\_xp = %.1f m', wall_xp), ...
        'Color',CFG.wallSolidCol, 'FontWeight','bold', 'FontSize',CFG.wallLabelFS, ...
        'Parent',ax, 'Clipping','on', 'HorizontalAlignment','left');
end

if ~isempty(wall_stop)
    hWallStop = plot(ax, [wall_stop wall_stop], yL, '--', ...
        'LineWidth',CFG.wallDashW, 'Color',CFG.wallDashCol);
    uistack(hWallStop,'top');

    text(wall_stop - CFG.wallLabelDx, labelY + CFG.wallLabelSep*ySpan, ...
        sprintf('wall\\_stop = %.1f m', wall_stop), ...
        'Color',CFG.wallDashCol, 'FontWeight','bold', 'FontSize',CFG.wallLabelFS, ...
        'Parent',ax, 'Clipping','on', 'HorizontalAlignment','right');
end

% ------------------------------------------------------------
% TELEMETRY BOX
% ------------------------------------------------------------
if CFG.showTelemetry
    hTelem = annotation('textbox',[0.02 0.65 0.18 0.18], ...
        'Color','w','EdgeColor','w','FontSize',9, ...
        'Interpreter','none','BackgroundColor',[0 0 0 0.45]);
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

drawnow; % ensure everything is laid out before first frame

% ------------------------------------------------------------
% MAIN ANIMATION LOOP
% ------------------------------------------------------------
refU = 1.0;
refZ = 5.0;

% First frame determines output resolution
fr0 = getframe(fig);
img0 = fr0.cdata;
H0 = size(img0,1);
W0 = size(img0,2);
writeVideo(vw, img0);

for k = 1+skip:skip:K

    % -----------------------
    % Animate the water surface
    % -----------------------
    tk = t(k);
    ys = WAV.A1*sin(WAV.k1*xs + WAV.w1*tk) ...
       + WAV.A2*sin(WAV.k2*xs + WAV.w2*tk + WAV.p2) ...
       + WAV.A3*sin(WAV.k3*xs + WAV.w3*tk + WAV.p3);

    Ypoly = [ys, zBottom*ones(size(xs))];
    set(hSurface,'YData',Ypoly);

    % Move submarine
    set(hSub,  'XData', ovalX + xp(k), 'YData', ovalY + zp(k));
    set(hSail, 'XData', sailX + xp(k), 'YData', sailY + zp(k));
    set(hTail, 'XData', tailX + xp(k), 'YData', tailY + zp(k));

% Update telemetry
if CFG.showTelemetry
    kk = max(1, min(size(u,2), k-1));
    telemStr = sprintf([ ...
        'Time: %.1f s\n' ...
        'x_p (horizontal): %.2f m\n' ...
        'u (surge speed): %.3f m/s\n' ...
        'Depth z_p: %.2f m\n' ...
        'Surge thrust: %.1f N\n' ...
        'Disturbance true: %.1f N\n' ...
        'Disturbance est: %.1f N'], ...
        t(k), xp(k), uvel(k), zp(k), u(1,kk), ...
        dtrue(min(k,end)), dhat(min(k,end)));

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

    % Enforce constant frame size (no toolboxes)
    if size(img,1) ~= H0 || size(img,2) ~= W0
        img = padCropFrame(img, H0, W0);
    end

    % IMPORTANT: write image frames (not frame structs) to avoid size issues
    writeVideo(vw, img);
end

safeCloseVideoWriter(vw, CFG.videoName);

fprintf('\nVideo written to:\n  %s\n', CFG.videoName);
fprintf('Done.\n');

end

% =====================================================================
function imgOut = padCropFrame(imgIn, H, W)
% Pads/crops an RGB image to exactly HxW without toolboxes.
% - If imgIn is smaller: pads with black.
% - If larger: crops to centre (looks nicer than top-left).

imgOut = zeros(H, W, 3, 'uint8');

hin = size(imgIn,1);
win = size(imgIn,2);

% Determine crop region (centre crop if too big)
row0 = 1;
col0 = 1;
if hin > H
    row0 = floor((hin - H)/2) + 1;
end
if win > W
    col0 = floor((win - W)/2) + 1;
end

row1 = min(hin, row0 + H - 1);
col1 = min(win, col0 + W - 1);

crop = imgIn(row0:row1, col0:col1, :);

% Paste crop into output (top-left)
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
