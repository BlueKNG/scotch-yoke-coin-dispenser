%% Coin elevator: symbolic kinematics + animation (fanimator)
% 2103-322 Design of Mechanism, Term Project 2569 - Part II (Symbolic Math)
% Stand-alone: paste and run. Needs MATLAB + Symbolic Math Toolbox only.
% Numbers are the same as params.m of the project.
%
% Mechanism: the disc (link 2) is a crank and a face cam. Its crank pin drives a
% Scotch-yoke slider (link 3) whose stem pushes a column of coins up one coin
% per turn. A follower pin in the cam groove swings the rocker (link 4); its "7"
% hook is a pawl that holds the column while the stem goes back down. At bottom
% dead centre the next coin rolls down a 45 deg feed onto the stem.
% Frame: origin at the disc centre, x right, y up, angles CCW from +x. SI units.

%% 1. Parameters (from params.m)
p.N     = 30;                      % disc speed [rpm]
p.w     = 2*pi*p.N/60;             % [rad/s]
p.g     = 9.81;
p.D     = 0.020;                   % coin diameter (1 baht)
p.chanW = p.D + 0.001;             % coin channel width
p.Rd    = 0.060;                   % disc radius
p.e     = 0.018;                   % crank pin eccentricity
p.stemLen = 0.075;                 % stem top above the yoke slot centreline
p.stemW = 0.007;                   % stem width
p.a = 0.044;  p.h = 0.048;         % rocker pivot P (fixed)
p.L2 = 0.048;                      % pivot -> follower pin
p.rr = 0.002;                      % follower pin radius
p.rGate = 0.002;                   % pawl pin radius
p.sPin  = 0.008;                   % pawl pin offset from the column axis when IN
p.cPawl = 0.002;                   % pin-to-coin clearance at top dead centre
p.dFeed = 0.003;                   % feed floor above the stem top at bottom dead centre
p.phi0  = deg2rad(4.5);            % rocker swing +/- phi0
p.seg   = deg2rad([70 110 270 315]);   % rise start/end, return start/end (disc angle)
rc = p.D/2;
% "7" hook solved so the pawl pin is at (sPin, yIn) when phi = +phi0
yRest = sqrt((rc + p.rGate)^2 - p.sPin^2);
p.yIn = p.stemLen + p.e + rc - yRest - p.cPawl;
s0 = sin(p.phi0); c0 = cos(p.phi0);
x = [-s0 -c0; c0 -s0] \ [p.sPin - p.a; p.yIn - p.h];
p.L1 = x(1); p.b = x(2);

%% 2. Degrees of freedom (Kutzbach-Gruebler)
% Links: 1 ground, 2 disc, 3 slider, 4 rocker, 5 motor pinion
% Lower pairs: R 1-2, P 1-3, R 1-4, R 1-5 (j1 = 4)
% Higher pairs: crank pin in yoke slot 2-3, follower pin in groove 2-4, gear mesh 2-5 (j2 = 3)
n = 5; j1 = 4; j2 = 3;
F = 3*(n - 1) - 2*j1 - j2

%% 3. Symbolic kinematics in the disc angle th
syms th t real
syms e a h L1 L2 b phi0 w real positive
yS  = e*sin(th)                    % slider: Scotch yoke follows the pin height
vS  = w*diff(yS, th)               % slider velocity (th = w*t)
aS  = w^2*diff(yS, th, 2)          % slider acceleration
% Rocker motion law: cycloidal dwell-rise-dwell-return-dwell
cyc = @(u) u - sin(2*pi*u)/(2*pi);
sg = p.seg;
phi = piecewise(th < sg(1), -phi0, ...
    th < sg(2), -phi0 + 2*phi0*cyc((th - sg(1))/(sg(2) - sg(1))), ...
    th < sg(3),  phi0, ...
    th < sg(4),  phi0 - 2*phi0*cyc((th - sg(3))/(sg(4) - sg(3))), ...
    -phi0)
phid = w*diff(phi, th);            % rocker angular velocity
P = [a; h];                        % rocker pivot
syms ph real                       % rocker angle as a plain symbol, for readable output
T_ph = P + L2*[sin(ph); -cos(ph)]  % follower pin
H_ph = P + L1*[-sin(ph); cos(ph)]; % hook corner
G_ph = H_ph + b*[-cos(ph); -sin(ph)]   % pawl pin
T = subs(T_ph, ph, phi); H = subs(H_ph, ph, phi); G = subs(G_ph, ph, phi);
C_ph = [cos(th) sin(th); -sin(th) cos(th)]*T_ph   % cam groove pitch curve (inversion, disc frame)
C = subs(C_ph, ph, phi);

%% 4. Numbers on one turn
sv = [e a h L1 L2 b phi0 w];
nv = [p.e p.a p.h p.L1 p.L2 p.b p.phi0 p.w];
thv = linspace(0, 2*pi, 721);
num = @(ex) double(subs(subs(ex, sv, nv), th, thv));
K.yS = num(yS); K.vS = num(vS); K.phi = num(phi);
K.T = [num(T(1)); num(T(2))];  K.G = [num(G(1)); num(G(2))];
K.H = [num(H(1)); num(H(2))];  K.C = [num(C(1)); num(C(2))];
fprintf('Slider stroke %.1f mm, max speed %.0f mm/s\n', 1e3*(max(K.yS) - min(K.yS)), 1e3*max(abs(K.vS)));
fprintf('Rocker swing %.1f deg, hook L1 = %.1f mm, b = %.1f mm\n', rad2deg(max(K.phi) - min(K.phi)), 1e3*p.L1, 1e3*p.b);

%% 5. Coins: 1 on the stem + 3 waiting in the 45 deg feed, 3 disc turns
% The stem and pawl come from the symbolic results above. Coins rest on the
% highest support (stem, coin on the stem, or pawl pin) and fall freely when it
% leaves; the head coin of the feed rolls in when the stem is below the feed floor.
nTurn = 3; fps = 30;
tEnd = nTurn*2*pi/p.w;
CN = coinRun(p, K, thv, tEnd);
fprintf('Coins: start 1 on the stem + 3 in the feed; after %d turns %d coins in the column\n', ...
    nTurn, CN.nCol(end));

%% 6. Animation (fanimator): play it here, it is also saved as an AVI
fig = figure('Color', 'w', 'Position', [100 50 620 860]);
if exist('theme', 'file'), theme(fig, 'light'); end   % R2025a+: keep a white background
ax = axes(fig); hold(ax, 'on'); axis(ax, 'equal'); box(ax, 'on');
axis(ax, [-85 75 -70 180]); xlabel(ax, 'x [mm]'); ylabel(ax, 'y [mm]');
drawFrame(ax, p, K);
% fanimator takes one graphics object per call: each part piece gets its own call
for j = 1:3, fanimator(ax, @(tt) drawGroove(tt, p, K, j)); end
for j = 1:3, fanimator(ax, @(tt) drawSlider(tt, p, K, thv, j)); end
for j = 1:3, fanimator(ax, @(tt) drawRocker(tt, p, K, thv, j)); end
for j = 1:size(CN.xy, 2), fanimator(ax, @(tt) drawCoin(tt, p, CN, j)); end
fanimator(ax, @(tt) text(-80, 172, sprintf('t = %.2f s   \\theta = %3.0f deg   turn %d', tt, ...
    mod(rad2deg(p.w*tt), 360), floor(p.w*tt/(2*pi)) + 1), 'FontSize', 11));
set(findall(fig, 'Type', 'line'), 'Clipping', 'on');
playAnimation(fig, 'AnimationRange', [0 tEnd], 'FrameRate', fps);
here = fileparts(mfilename('fullpath')); if isempty(here), here = pwd; end
writeAnimation(fig, fullfile(here, 'Mechanism_symbolic.avi'), 'AnimationRange', [0 tEnd], 'FrameRate', fps);

%% Local functions
function Y = per(Yg, thv, thq)
% Periodic lookup of a one-turn table at disc angle thq
Y = interp1(thv, Yg.', mod(thq, 2*pi)).';
end

function drawFrame(ax, p, K)
% Fixed parts: disc rim, coin column walls, 45 deg feed, rocker pivot
tc = linspace(0, 2*pi, 200); hw = 1e3*p.chanW/2;
fill(ax, 1e3*p.Rd*cos(tc), 1e3*p.Rd*sin(tc), [0.72 0.86 0.97], 'EdgeColor', [0.3 0.5 0.7]);
plot(ax, 0, 0, 'k+', 'MarkerSize', 10);
yBot = 1e3*(p.stemLen - p.e) - 4; yC0 = 1e3*(p.stemLen - p.e + p.dFeed + p.D/2);
win = 1e3*[min(K.G(2,:)) - p.rGate - 0.001, max(K.G(2,:)) + p.rGate + 0.001];   % pawl window
plot(ax, [hw hw], [yBot win(1)], 'Color', [0.4 0.4 0.45], 'LineWidth', 2);       % right wall
plot(ax, [hw hw], [win(2) 170], 'Color', [0.4 0.4 0.45], 'LineWidth', 2);
uF = [1; -1]/sqrt(2); nF = [1; 1]/sqrt(2); s = [0 75];
c0 = [0; yC0] - uF*s;                                    % feed centreline, from the column up-left
lo = c0 - hw*nF; up = c0 + hw*nF;
lo(:,1) = lo(:,2) + (lo(:,1) - lo(:,2))*((-hw - lo(1,2))/(lo(1,1) - lo(1,2)));   % floor ends at the left wall
up(:,1) = up(:,2) + (up(:,1) - up(:,2))*((-hw - up(1,2))/(up(1,1) - up(1,2)));
plot(ax, lo(1,:), lo(2,:), 'Color', [0.4 0.4 0.45], 'LineWidth', 2);
plot(ax, up(1,:), up(2,:), 'Color', [0.4 0.4 0.45], 'LineWidth', 2);
plot(ax, [-hw -hw], [yBot lo(2,1)], 'Color', [0.4 0.4 0.45], 'LineWidth', 2);    % left wall below the feed
plot(ax, [-hw -hw], [up(2,1) 170], 'Color', [0.4 0.4 0.45], 'LineWidth', 2);     % left wall above the feed
plot(ax, 1e3*p.a, 1e3*p.h, 'ko', 'MarkerSize', 9, 'MarkerFaceColor', 'k');
end

function h = drawGroove(tt, p, K, j)
% Disc piece j: 1 cam groove pitch curve, 2 crank line, 3 crank pin; all turn with th = w*t
q = p.w*tt;
switch j
    case 1
        Cg = 1e3*[cos(q) -sin(q); sin(q) cos(q)]*K.C;
        h = plot(Cg(1,:), Cg(2,:), 'Color', [0.1 0.25 0.5], 'LineWidth', 2.5);
    case 2
        h = plot(1e3*[0 p.e*cos(q)], 1e3*[0 p.e*sin(q)], 'Color', [0.1 0.25 0.5], 'LineWidth', 1.5);
    otherwise
        h = plot(1e3*p.e*cos(q), 1e3*p.e*sin(q), 'o', 'MarkerSize', 7, 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'k');
end
end

function h = drawSlider(tt, p, K, thv, j)
% Slider piece j: 1 yoke, 2 stem (coin pusher), 3 yoke slot; y = e sin(th)
y = 1e3*per(K.yS, thv, p.w*tt);
W = 1e3*(p.e + 0.0047); sw = p.stemW*1e3/2;
switch j
    case 1
        h = fill([-W W W -W], [y-6 y-6 y+6 y+6], [0.85 0.15 0.15], 'EdgeColor', 'k');
    case 2
        h = fill([-sw sw sw -sw], [y+5 y+5 y+1e3*p.stemLen y+1e3*p.stemLen], [0.85 0.15 0.15], 'EdgeColor', 'k');
    otherwise
        h = plot(1e3*[-p.e p.e], [y y], 'w-', 'LineWidth', 3);
end
end

function h = drawRocker(tt, p, K, thv, j)
% Rocker piece j: 1 body T - P - H - G, 2 follower pin T, 3 pawl pin G
q = p.w*tt;
Tq = 1e3*per(K.T, thv, q); Hq = 1e3*per(K.H, thv, q); Gq = 1e3*per(K.G, thv, q);
switch j
    case 1
        L = [Tq 1e3*[p.a; p.h] Hq Gq];
        h = plot(L(1,:), L(2,:), '-', 'Color', [0.95 0.75 0.05], 'LineWidth', 5);
    case 2
        h = plot(Tq(1), Tq(2), 'o', 'MarkerSize', 6, 'MarkerFaceColor', [0.5 0.1 0.5], 'MarkerEdgeColor', 'k');
    otherwise
        h = plot(Gq(1), Gq(2), 'o', 'MarkerSize', 6, 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'k');
end
end

function h = drawCoin(tt, p, CN, j)
% Coin j at time tt (positions precomputed by coinRun)
[~, k] = min(abs(CN.t - tt));
tc = linspace(0, 2*pi, 40); r = 1e3*p.D/2;
c = 1e3*CN.xy(:, j, k);
h = fill(c(1) + r*cos(tc), c(2) + r*sin(tc), [0.5 0.2 0.65], 'EdgeColor', 'k', 'FaceAlpha', 0.9);
end

function CN = coinRun(p, K, thv, tEnd)
% Step the coins in time. Column = stack on the x = 0 axis, bottom centre yb.
% newc = the coin that rolled onto the stem and has not joined the stack yet.
rc = p.D/2; D = p.D; dt = 1/600;
yC0 = p.stemLen - p.e + p.dFeed + rc;          % coin centre on the feed floor at the column axis
uF = [1; -1]/sqrt(2);                          % feed direction (down-right, into the column)
dIn = (rc + p.stemW/2)*sqrt(2);                % head coin waits leaning on the stem side
aR = (2/3)*p.g*sin(pi/4);                      % rolling solid disc on 45 deg
nQ = 3; tv = 0:dt:tEnd; nT = numel(tv);
nCoin = 1 + nQ;
xy = nan(2, nCoin, nT); nColOut = zeros(1, nT);
col = 1; yb = per(K.yS, thv, 0) + p.stemLen + rc; vb = 0;   % coin 1 on the stem
queue = 2:nCoin; sq = zeros(1, nQ);           % distance of each queued coin back from the column axis
newc = []; rolling = false; sR = 0; vR = 0;
for k = 1:nT
    q = p.w*tv(k);
    stemC = per(K.yS, thv, q) + p.stemLen + rc;  % centre of a coin sitting on the stem
    Gq = per(K.G, thv, q);
    if Gq(1) < rc + p.rGate                    % pawl pin under the column
        pawlC = Gq(2) + sqrt((rc + p.rGate)^2 - Gq(1)^2);
    else
        pawlC = -inf;
    end
    % stack: rests on the higher of (stem or new coin) and the pawl; free fall otherwise
    sup = stemC + D*~isempty(newc);
    sup = max(sup, pawlC);
    if yb + vb*dt <= sup
        yb = sup; vb = 0;
    else
        vb = vb - p.g*dt; yb = max(sup, yb + vb*dt);
    end
    if ~isempty(newc) && abs(yb - (stemC + D)) < 1e-6
        col = [newc col]; newc = []; yb = stemC;   % stack sits on the new coin: one stack
    end
    % feed queue: head waits at dIn, or further back if a column coin is in the way
    yCol = [yb + (0:numel(col) - 1)*D, stemC*ones(1, ~isempty(newc))];
    tgt = max([dIn, arrayfun(@(y) feedTouch(yC0 - y, D), yCol)]);
    if rolling                                     % head coin rolls down to the column axis
        vR = vR + aR*dt; sR = sR - vR*dt;
        if sR <= 0
            newc = queue(1); queue(1) = []; sq(1) = []; rolling = false;
        else
            sq(1) = sR;
        end
    elseif ~isempty(queue) && isempty(newc) && stemC - rc <= yC0 - rc + 1e-9 ...
            && yb - stemC >= D && abs(sq(1) - dIn) < 1e-6
        rolling = true; sR = sq(1); vR = 0;        % stem below the feed floor: window open
    end
    for j = 1 + rolling:numel(queue)               % the rest follow one coin apart
        sq(j) = max(tgt + (j - 1)*D, sq(j) - 0.15*dt);
        if k == 1, sq(j) = tgt + (j - 1)*D; end
    end
    % positions
    for j = 1:numel(col),   xy(:, col(j), k) = [0; yb + (j - 1)*D]; end
    if ~isempty(newc),      xy(:, newc, k) = [0; stemC]; end
    for j = 1:numel(queue), xy(:, queue(j), k) = [0; yC0] - uF*sq(j); end
    nColOut(k) = numel(col) + ~isempty(newc);
end
CN.t = tv; CN.xy = xy; CN.nCol = nColOut;
end

function r = feedTouch(d, D)
% Distance back along the 45 deg feed at which a feed coin touches a column
% coin whose centre is d below the feed height (0 = path clear)
if d^2 >= 2*D^2, r = 0; return; end
u = (-d + sqrt(2*D^2 - d^2))/2;
r = max(0, sqrt(2)*u);
end
