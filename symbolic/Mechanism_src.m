%% Mechanism - Scotch yoke + face-cam pawl coin elevator
% 2103-322 Design of Mechanism, Term Project 2569
% Symbolic kinematics, cam synthesis, workspace and input torque.
% The slider stem pushes a column of coins up one coin per turn; the rocker's
% "7" hook is a pawl that holds the column while the slider returns.
% Frame: origin at disc centre, x right, y up, angles CCW from +x. SI units.
% Every number comes from params.m (one folder up).

here = fileparts(mfilename('fullpath'));
if isempty(here), here = pwd; end
root = fileparts(here);
addpath(root, fullfile(root, 'cad'));
p = params();
figdir = fullfile(root, 'figures');
if ~exist(figdir, 'dir'), mkdir(figdir); end

%% 1. Links, joints and degrees of freedom
% Links:  1 ground, 2 disc (crank + face cam + gear rim), 3 slider (yoke + pusher),
%         4 rocker (pawl), 5 motor pinion
% Lower pairs (remove 2 DOF each):  J1 R 1-2,  J2 P 1-3,  J3 R 1-4,  J6 R 1-5
% Higher pairs (remove 1 DOF each): J4 crank pin in yoke slot 2-3,
%                                   J5 rocker pin in cam groove 2-4,
%                                   J7 gear mesh 2-5
% The pinion only sets the disc speed; everything below is in the disc angle th.
n = 5; j1 = 4; j2 = 3;
F = 3*(n - 1) - 2*j1 - j2           % Kutzbach-Gruebler mobility

%% 2. Slider: Scotch yoke
% The crank pin sits at e*[cos th; sin th]. The yoke slot is horizontal, so
% the slider only follows the y-coordinate of the pin.
syms th real
syms e a h L1 L2 w phi0 real positive
yS = e*sin(th)                      % slider position
vS = w*diff(yS, th)                 % slider velocity  (w = d th/dt, constant)
aS = w^2*diff(yS, th, 2)            % slider acceleration

%% 3. Rocker: desired motion law (cycloidal dwell-rise-dwell-return-dwell)
% Cycloidal motion has zero velocity and zero acceleration at both ends, so
% the pawl starts and stops without a jerk spike.
% Pawl OUT while the column is pushed up, IN around top dead centre (90 deg),
% held IN while the stem goes down, OUT again after the new coin is on the stem.
assert(p.dwell1(1) == 0 && p.dwell1(2) == p.rise(1) && p.rise(2) == p.dwell2(1) ...
    && p.dwell2(2) == p.ret(1) && p.ret(2) == p.dwell3(1) ...
    && abs(p.dwell3(2) - 2*pi) < 1e-12, 'segments must tile 0..2pi');
cyc = @(x) x - sin(2*pi*x)/(2*pi);
r1 = p.rise; r2 = p.ret; d2 = p.dwell2;
phi = piecewise(th < r1(1), -phi0, ...
    th < r1(2), -phi0 + 2*phi0*cyc((th - r1(1))/(r1(2) - r1(1))), ...
    th < d2(2),  phi0, ...
    th < r2(2),  phi0 - 2*phi0*cyc((th - r2(1))/(r2(2) - r2(1))), ...
    -phi0)
phid  = w*diff(phi, th);            % rocker angular velocity
phidd = w^2*diff(phi, th, 2);       % rocker angular acceleration

%% 4. Positions of the key points
syms b real positive
P = [a; h] + 0*th;                  % rocker pivot (fixed to the frame)
T = P + L2*[sin(phi); -cos(phi)]    % follower pin centre
H = P + L1*[-sin(phi); cos(phi)];   % top corner of the "7" hook
G = H + b*[-cos(phi); -sin(phi)]    % pawl pin at the tip of the hook bar
vG = w*diff(G, th);                 % pawl pin velocity
aG = w^2*diff(G, th, 2);            % pawl pin acceleration

%% 5. Cam synthesis (inversion)
% Fix the disc and let the ground rotate by -th. The follower pin centre,
% seen from the disc, traces the pitch curve of the groove.
Rm = [cos(th) sin(th); -sin(th) cos(th)];   % ground -> disc frame
C  = Rm*T                                   % pitch curve on the disc
Cp = diff(C, th); Cpp = diff(C, th, 2);

%% 6. Numerical evaluation with the values in params.m
sv = [e a h L1 L2 w phi0 b];
nv = [p.e p.a p.h p.L1 p.L2 p.omega p.phi0 p.b];
gendir = tempname; mkdir(gendir); addpath(gendir);   % piecewise needs file output
num = @(expr) matlabFunction(subs(expr, sv, nv), 'Vars', th, ...
    'File', fullfile(gendir, uniqueName()));
thv = linspace(0, 2*pi, 721);
thv(end) = 2*pi - 1e-9;             % stay inside the last piecewise branch
ev = @(expr) evalPts(num(expr), thv, size(expr, 1));

R.th    = thv;
R.yS    = ev(yS);   R.vS = ev(vS);   R.aS = ev(aS);
R.phi   = ev(phi);  R.phid = ev(phid); R.phidd = ev(phidd);
R.P     = ev(P);    R.T = ev(T);     R.G = ev(G);   R.H = ev(H);
R.vG    = ev(vG);   R.aG = ev(aG);
R.C     = ev(C);    R.Cp = ev(Cp);   R.Cpp = ev(Cpp);

%% 7. Cam checks: pressure angle, curvature, fit on the disc
% Pressure angle: angle between the groove normal (force line on the pin)
% and the direction the pin can move as the rocker turns, u = [cos phi; sin phi].
nDisc = [-R.Cp(2,:); R.Cp(1,:)]./vecnorm(R.Cp);
nGnd  = [cos(thv).*nDisc(1,:) - sin(thv).*nDisc(2,:);
         sin(thv).*nDisc(1,:) + cos(thv).*nDisc(2,:)];
u     = [cos(R.phi); sin(R.phi)];
R.alpha = acos(min(1, abs(sum(nGnd.*u))));
% Radius of curvature of the pitch curve; must exceed the groove half-width
R.rho = vecnorm(R.Cp).^3 ./ abs(R.Cp(1,:).*R.Cpp(2,:) - R.Cp(2,:).*R.Cpp(1,:));
R.rC  = vecnorm(R.C);
halfW = p.rr + p.clear;

%% 8. Input torque (energy method, friction ignored)
% Power in = d/dt (kinetic + potential). The rocker turns about a fixed pivot
% with its mass lumped there, so only its rotary inertia takes power.
% Tin is the mechanism alone (checked against Simscape). TinLoad adds the coin
% column, which rides on the stem while the slider rises (pawl out) and rests
% on the pawl while it falls.
R.Tin = (p.ms*R.vS.*(R.aS + p.g) + p.Ir*R.phid.*R.phidd)/p.omega;
nLift = hopper_geometry(p).nLift;   % coins in the column, stem to the top bend
R.TinLoad = R.Tin + (R.vS > 0).*(nLift*p.mCoin*R.vS.*(R.aS + p.g))/p.omega;

%% 9. Verification
% (a) symbolic derivatives vs finite differences of the positions
fd = @(y) gradient(y, thv)*p.omega;
in = 2:numel(thv)-1;                % skip ends (one-sided differences)
vFD = fd(R.yS); pFD = fd(R.phi);
errV   = max(abs(vFD(in) - R.vS(in)))/max(abs(R.vS));
errPhi = max(abs(pFD(in) - R.phid(in)))/max(abs(R.phid));
% (b) closed groove, (c) fits between hub boss and rim, (d) pressure angle, (e) curvature
closeGap = norm(R.C(:,1) - R.C(:,end));
rRoot = p.mGear*p.Ng/2 - 1.25*p.mGear;   % disc gear root radius
checks = {
    'DOF = 1',                         F == 1,                        sprintf('%d', F)
    'slider velocity vs FD (rel err)', errV < 1e-3,                   sprintf('%.1e', errV)
    'rocker velocity vs FD (rel err)', errPhi < 1e-3,                 sprintf('%.1e', errPhi)
    'groove is closed [m]',            closeGap < 1e-6,               sprintf('%.1e', closeGap)
    'groove r_min > e + 6 mm [m]',     min(R.rC) - halfW > p.e + 0.006, sprintf('%.4f', min(R.rC) - halfW)
    'groove r_max < gear root - 2 mm [m]', max(R.rC) + halfW < rRoot - 0.002, sprintf('%.4f', max(R.rC) + halfW)
    'max pressure angle < 30 deg',     max(R.alpha) < p.alphaMax,     sprintf('%.1f deg', rad2deg(max(R.alpha)))
    'min curvature radius > half-width [m]', min(R.rho) > halfW,      sprintf('%.4f', min(R.rho))
    };
disp(cell2table(checks, 'VariableNames', {'Check', 'Pass', 'Value'}))

%% 10. Workspace and key results
fprintf('Slider stroke          : %.1f mm\n', 1e3*(max(R.yS) - min(R.yS)));
fprintf('Rocker swing           : %.1f deg\n', rad2deg(max(R.phi) - min(R.phi)));
fprintf('Hook size L1, b        : %.1f, %.1f mm\n', 1e3*p.L1, 1e3*p.b);
fprintf('Pawl pin range x       : %.1f .. %.1f mm\n', 1e3*min(R.G(1,:)), 1e3*max(R.G(1,:)));
fprintf('Pawl pin range y       : %.1f .. %.1f mm\n', 1e3*min(R.G(2,:)), 1e3*max(R.G(2,:)));
fprintf('Max slider speed       : %.1f mm/s\n', 1e3*max(abs(R.vS)));
fprintf('Max rocker speed       : %.2f rad/s\n', max(abs(R.phid)));
fprintf('Max input torque       : %.2f mN m (mechanism), %.2f mN m (with %d coins)\n', ...
    1e3*max(abs(R.Tin)), 1e3*max(abs(R.TinLoad)), nLift);
% Gear stage: omega_motor = omega*Ng/Np, T_motor = T_in*(Np/Ng)/eta
fprintf('Gear drive             : %d/%d teeth, m = %.1f mm, centre distance %.1f mm\n', ...
    p.Ng, p.Np, 1e3*p.mGear, 1e3*p.mGear*(p.Ng + p.Np)/2);
fprintf('Motor speed / torque   : %.0f rpm, %.2f mN m (with coins, eta = %.1f)\n', ...
    p.rpmMotor, 1e3*max(abs(R.TinLoad))*p.Np/p.Ng/p.etaGear, p.etaGear);

%% 11. Plots
deg = rad2deg(thv);
f1 = figure('Position', [100 100 900 700]); theme(f1, 'light');   % printed report
subplot(3,2,1); plot(deg, 1e3*R.yS); ylabel('y_S [mm]'); title('Slider'); grid on
subplot(3,2,3); plot(deg, 1e3*R.vS); ylabel('v_S [mm/s]'); grid on
subplot(3,2,5); plot(deg, 1e3*R.aS); ylabel('a_S [mm/s^2]'); xlabel('\theta [deg]'); grid on
subplot(3,2,2); plot(deg, rad2deg(R.phi)); ylabel('\phi [deg]'); title('Rocker'); grid on
subplot(3,2,4); plot(deg, R.phid); ylabel('d\phi/dt [rad/s]'); grid on
subplot(3,2,6); plot(deg, R.phidd); ylabel('d^2\phi/dt^2 [rad/s^2]'); xlabel('\theta [deg]'); grid on
exportgraphics(f1, fullfile(figdir, 'kinematics.png'), 'Resolution', 150);

f2 = figure('Position', [100 100 1000 450]); theme(f2, 'light');
subplot(1,2,1); hold on; axis equal; grid on
tc = linspace(0, 2*pi, 200);
plot(1e3*p.Rd*cos(tc), 1e3*p.Rd*sin(tc), 'k');
plot(1e3*R.C(1,:), 1e3*R.C(2,:), 'b', 'LineWidth', 1.5);
plot(1e3*p.e, 0, 'ro', 'MarkerFaceColor', 'r');
title('Cam pitch curve on the disc'); xlabel('x [mm]'); ylabel('y [mm]');
legend('disc rim', 'groove centreline', 'crank pin', 'Location', 'southoutside');
subplot(1,2,2); hold on; axis equal; grid on
plot(1e3*R.G(1,:), 1e3*R.G(2,:), 'm', 'LineWidth', 1.5);
plot(1e3*R.T(1,:), 1e3*R.T(2,:), 'b', 'LineWidth', 1.5);
plot(1e3*p.a, 1e3*p.h, 'ko', 'MarkerFaceColor', 'w', 'LineWidth', 2);
plot(1e3*[0 0], 1e3*[min(R.yS) max(R.yS)] + 1e3*p.stemLen, 'Color', [0.7 0.6 0.4], 'LineWidth', 3);
title('Workspace (ground frame)'); xlabel('x [mm]'); ylabel('y [mm]');
legend('pawl pin G', 'follower pin T', 'pivot P (fixed)', 'stem top', 'Location', 'southoutside');
exportgraphics(f2, fullfile(figdir, 'cam_workspace.png'), 'Resolution', 150);

f3 = figure('Position', [100 100 900 350]); theme(f3, 'light');
subplot(1,2,1); plot(deg, rad2deg(R.alpha)); yline(rad2deg(p.alphaMax), 'r--');
ylabel('pressure angle [deg]'); xlabel('\theta [deg]'); grid on
subplot(1,2,2); plot(deg, 1e3*R.Tin, deg, 1e3*R.TinLoad, '--'); grid on
ylabel('T_{in} [mN m]'); xlabel('\theta [deg]');
legend('mechanism', sprintf('with %d coins lifted', nLift), 'Location', 'best');
exportgraphics(f3, fullfile(figdir, 'pressure_torque.png'), 'Resolution', 150);

%% 12. Cam shape: radius vs angle and exaggerated profile
% The groove looks almost round because the pin moves only a few mm against a
% radius of ~44 mm. Plot the radius directly, and scale the deviation from the
% mean circle by 5 so the shape becomes visible.
r0 = mean(R.rC); k = 5;
fprintf('Pitch radius           : %.1f .. %.1f mm (mean %.1f)\n', 1e3*min(R.rC), 1e3*max(R.rC), 1e3*r0);
segs = {p.dwell1, 'dwell OUT'; p.rise, 'pawl in'; p.dwell2, 'dwell IN'; p.ret, 'pawl out'; ...
    p.dwell3, 'dwell OUT '};
cols = lines(5); cols(5,:) = cols(1,:);
f4 = figure('Position', [100 100 1100 480]); theme(f4, 'light');
subplot(1,2,1); hold on; grid on
for s = 1:size(segs, 1)
    in = thv >= segs{s,1}(1) & thv <= segs{s,1}(2);
    plot(deg(in), 1e3*R.rC(in), 'Color', cols(s,:), 'LineWidth', 2);
end
yline(1e3*r0, 'k--');
xlim([0 360]); xticks(0:45:360);
xlabel('\theta [deg]'); ylabel('pitch radius r [mm]');
title('Groove radius changes with disc angle');
legend([segs(1:4,2); {'mean'}], 'Location', 'best');
subplot(1,2,2); hold on; axis equal; grid on
tc = linspace(0, 2*pi, 200);
plot(1e3*r0*cos(tc), 1e3*r0*sin(tc), 'k--');
Cex = R.C.*((r0 + k*(R.rC - r0))./R.rC);
for s = 1:size(segs, 1)
    in = thv >= segs{s,1}(1) & thv <= segs{s,1}(2);
    plot(1e3*Cex(1,in), 1e3*Cex(2,in), 'Color', cols(s,:), 'LineWidth', 2);
end
plot(1e3*R.C(1,:), 1e3*R.C(2,:), 'Color', [0.6 0.6 0.6]);
plot(1e3*p.e, 0, 'ro', 'MarkerFaceColor', 'r');
xlabel('x [mm]'); ylabel('y [mm]');
title(sprintf('Pitch curve, deviation from mean circle x%d', k));
legend('mean circle', segs{:,2}, 'true scale', 'crank pin', 'Location', 'southoutside', 'NumColumns', 3);
exportgraphics(f4, fullfile(figdir, 'cam_shape.png'), 'Resolution', 150);

%% 13. Analytical model vs time and disc speed N
% Every quantity above is a function of the disc angle th only. With a motor
% at constant speed, th = omega*t and omega = 2*pi*N/60, so time and speed
% enter only through omega: positions do not depend on N, velocities scale
% with N and accelerations with N^2. Gravity does not scale, so the coin loop
% (gravity return, coin on the stem) sets the usable speed range.
syms t N real positive
syms stemL real positive
wN = 2*pi*N/60;
yTip = stemL + subs(yS, th, wN*t)          % stem (pusher) tip height vs time and speed
vTip = diff(yTip, t)
aTip = diff(yTip, t, 2)
Q    = N                                    % coins delivered per minute (one per turn)
vCol = p.Dcoin*N/60;                        % mean rise speed of the coin column [m/s]
G_tN = subs(G, th, wN*t);                   % rocker tip (pawl pin) [x; y] vs time and speed, first turn
vG_tN = diff(G_tN, t);                      % its velocity: a factor N in every term

% (a) Stem tip and pawl angle in time at three speeds
Ns = [15 30 60];
tt = linspace(0, 4, 2000);
f5 = figure('Position', [100 100 900 600]); theme(f5, 'light');
for k = 1:numel(Ns)
    thk = mod(2*pi*Ns(k)/60*tt, 2*pi);
    subplot(2,1,1); hold on
    plot(tt, 1e3*(p.stemLen + p.e*sin(thk)), 'LineWidth', 1.2);
    subplot(2,1,2); hold on
    plot(tt, rad2deg(interp1(thv, R.phi, thk)), 'LineWidth', 1.2);
end
subplot(2,1,1); grid on; ylabel('stem tip y [mm]');
title('Stem tip y(t) = stemLen + e sin(2\pi N t/60): same stroke, period 60/N');
legend(compose('N = %d rpm', Ns), 'Location', 'eastoutside');
subplot(2,1,2); grid on; ylabel('pawl angle \phi [deg]'); xlabel('t [s]');
legend(compose('N = %d rpm', Ns), 'Location', 'eastoutside');
exportgraphics(f5, fullfile(figdir, 'speed_time.png'), 'Resolution', 150);

% (b) Speed limits. Rolling coin on a channel floor at angle beta (solid disc,
% I = m r^2/2) with an effective drag coefficient c:
%   a = (2/3)*g*(sin(beta) - c*cos(beta))
% This is a lower bound: a coin that slips instead of rolling goes faster.
Hp = hopper_geometry(p);
rc = p.Dcoin/2;
aRoll = @(beta, c) (2/3)*p.g*(sin(beta) - c*cos(beta));
cs = linspace(0, 0.15, 31);

% Limit 1, coin lift-off: near top dead centre the stem decelerates at
% e*omega^2. Above g the column leaves the stem (it floats, then drops).
N_lift = 60/(2*pi)*sqrt(p.g/p.e);

% Limit 2, feed entry: the head coin waits on the 45 deg feed, leaning on the
% side of the stem at its own centre height yWait. From rest it must roll
% d_in = (rc + stemW/2)*sqrt(2) to the column axis, inside the window:
%   opens  when the stem top drops below yWait (the coin rolls over the stem corner
%          and rides the stem top down): stemLen + e*sin(th) = yWait, th in (90, 270)
%   closes when the stem top rises back past the feed floor: e*sin(th) = -e + dFeed
% A first, simpler model opened the window only when the stem top was below the
% feed floor (winSimple); the contact sim showed coins entering earlier, at
% 216-227 deg, so that model is too strict (kept for the report).
d_in  = (rc + p.stemW/2)*sqrt(2);
yWait = Hp.yC0 + d_in/sqrt(2);
thOpen  = pi - asin((yWait - p.stemLen)/p.e);
thClose = 3*pi/2 + acos(1 - p.dFeed/p.e);
win       = thClose - thOpen;
winSimple = 2*acos(1 - p.dFeed/p.e);
t_in = @(c) sqrt(2*d_in./aRoll(pi/4, c));
N_feed = @(c) win/(2*pi)*60./t_in(c);
N_feedSimple = winSimple/(2*pi)*60/t_in(p.cDrag);

% Limit 3, return flow: a coin leaves the hood peak at rest, rolls down the 12 deg
% ramp, falls down the drop and rolls down the 45 deg feed. Coins not in the
% column or the queue are in transit; at least one must wait in the queue, so
%   T_ret*N/60 <= nCoins - nLift - 1.
P1b = Hp.path(:,3); P2 = Hp.path(:,4); P3 = Hp.path(:,5); P0 = Hp.path(:,6);
bRamp = atan2(P1b(2) - P2(2), P1b(1) - P2(1));
N_ret = zeros(size(cs)); T_ret = zeros(size(cs));
for k = 1:numel(cs)
    [t1, v1] = rollTime(norm(P2 - P1b), 0, aRoll(bRamp, cs(k)));
    [t2, v2] = rollTime(norm(P3 - P2), v1, p.g);          % drop: free fall
    [t3, ~]  = rollTime(norm(P0 - P3) - d_in, v2*cos(pi/4), aRoll(pi/4, cs(k)));   % lands on 45 deg
    T_ret(k) = t1 + t2 + t3;
    N_ret(k) = (Hp.nCoins - Hp.nLift - 1)*60/T_ret(k);
end

% Limit 4, drive: the TT motor no-load speed over the gear ratio, for 3 .. 6 V supplies.
% The load torque is tiny next to the stall torque, so the speed stays near no-load.
N_motorV = p.ttNoLoad(:,2)*p.Np/p.Ng;      % disc rpm at each supply voltage
N_motor = N_motorV(1);                      % 3 V (2 x AA), closest to the 30 rpm design

% Input torque vs N: split Tin into the parts that scale with omega^2 and the
% gravity part. With y' = dyS/dth etc.: T = m*y'*(y''*omega^2 + g) + Ir*phi'*phi''*omega^2
yp = R.vS/p.omega; ypp = R.aS/p.omega^2;
fp = R.phid/p.omega; fpp = R.phidd/p.omega^2;
mLift = p.ms + (R.vS > 0)*Hp.nLift*p.mCoin;               % column rides up with the stem
Tof = @(n) mLift.*yp.*(ypp*(2*pi*n/60)^2 + p.g) + p.Ir*fp.*fpp*(2*pi*n/60)^2;
Nv = linspace(1, 350, 350);
Tmax = arrayfun(@(n) max(abs(Tof(n))), Nv);
Pavg = arrayfun(@(n) mean(max(Tof(n), 0))*2*pi*n/60, Nv);  % mean positive input power
TmotMax = Tmax*p.Np/p.Ng/p.etaGear;

lim = [N_lift, N_feed(p.cDrag), interp1(cs, N_ret, p.cDrag)];
fprintf('\nSpeed limits at drag c = %.2f (disc rpm):\n', p.cDrag);
fprintf('  coin lift-off at TDC   : %.0f rpm  (e*omega^2 = g)\n', lim(1));
fprintf('  feed entry window      : %.0f rpm  (window %.0f..%.0f deg = %.0f deg, roll-in %.0f ms)\n', ...
    lim(2), rad2deg(thOpen), rad2deg(thClose), rad2deg(win), 1e3*t_in(p.cDrag));
fprintf('    (simpler model, stem below the feed floor only: %.0f deg -> %.0f rpm, too strict)\n', ...
    rad2deg(winSimple), N_feedSimple);
fprintf('  return flow            : %.0f rpm  (return time %.0f ms, %d coins may be in transit)\n', ...
    lim(3), 1e3*interp1(cs, T_ret, p.cDrag), Hp.nCoins - Hp.nLift - 1);
fprintf('  usable range           : 0 .. %.0f rpm; design N = %d rpm -> margin x%.1f\n', ...
    min(lim), p.rpm, min(lim)/p.rpm);
fprintf('  at N = %d rpm          : %d coins/min, column rises %.1f mm/s, coin %d turns (%.0f s) bottom to top\n', ...
    p.rpm, p.rpm, 1e3*p.Dcoin*p.rpm/60, Hp.nLift, Hp.nLift*60/p.rpm);
for k = 1:size(p.ttNoLoad, 1)
    fprintf('  %s at %.1f V : %.0f rpm no-load -> %.0f rpm at the disc\n', ...
        p.motorName, p.ttNoLoad(k,1), p.ttNoLoad(k,2), N_motorV(k));
end
fprintf('  -> any supply from 3 to 6 V stays below every limit (highest %.0f rpm)\n', max(N_motorV));
fprintf('  torque at N = %d / %.0f : %.2f / %.2f mN m disc, %.2f / %.2f mN m motor\n', ...
    p.rpm, min(lim), 1e3*max(abs(Tof(p.rpm))), 1e3*max(abs(Tof(min(lim)))), ...
    1e3*max(abs(Tof(p.rpm)))*p.Np/p.Ng/p.etaGear, 1e3*max(abs(Tof(min(lim))))*p.Np/p.Ng/p.etaGear);
fprintf('  motor stall torque %.0f mN m at 3 V -> margin x%.0f over the smooth-model peak at %d rpm\n', ...
    1e3*p.ttStall3V, p.ttStall3V/(max(abs(Tof(p.rpm)))*p.Np/p.Ng/p.etaGear), p.rpm);
S = struct('N_lift', N_lift, 'N_feed', lim(2), 'N_ret', lim(3), 'N_motor', N_motor, 'N_motorV', N_motorV, ...
    'win', win, 'thOpen', thOpen, 'thClose', thClose, 'N_feedSimple', N_feedSimple, 't_in', t_in(p.cDrag), 'T_ret', interp1(cs, T_ret, p.cDrag), ...
    'cs', cs, 'N_feed_c', N_feed(cs), 'N_ret_c', N_ret, 'Nv', Nv, 'Tmax', Tmax, 'Pavg', Pavg);

f6 = figure('Position', [100 100 1100 420]); theme(f6, 'light');
subplot(1,3,1); hold on; grid on
plot(Nv, Nv, 'k', 'LineWidth', 1.5);
xline(lim(2), 'r--', 'feed entry'); xline(lim(1), 'm--', 'lift-off');
xline(p.rpm, 'g-', sprintf('design %d (TT 3 V: %.0f)', p.rpm, N_motor));
xline(max(N_motorV), 'g:', sprintf('TT 6 V: %.0f', max(N_motorV)));
xlabel('disc speed N [rpm]'); ylabel('coins per minute Q = N');
yyaxis right; plot(Nv, 1e3*p.Dcoin*Nv/60); ylabel('column rise speed [mm/s]');
title('Delivery rate');
subplot(1,3,2); hold on; grid on
plot(cs, N_feed(cs), 'r', cs, N_ret, 'c', 'LineWidth', 1.5);
yline(N_lift, 'm--'); yline(p.rpm, 'g-');
xlabel('drag coefficient c'); ylabel('max disc speed [rpm]');
legend('feed entry', 'return flow', 'lift-off', 'design', 'Location', 'best');
title('Speed limits vs coin drag');
subplot(1,3,3); hold on; grid on
plot(Nv, 1e3*Tmax, Nv, 1e3*TmotMax, '--', 'LineWidth', 1.5);
xline(min(lim), 'r--'); xline(p.rpm, 'g-');
xlabel('disc speed N [rpm]'); ylabel('peak torque [mN m]');
yyaxis right; plot(Nv, 1e3*Pavg); ylabel('mean input power [mW]');
legend('disc', 'motor (via gears)', 'Location', 'northwest');
title('Torque and power');
exportgraphics(f6, fullfile(figdir, 'speed_limits.png'), 'Resolution', 150);

save(fullfile(here, 'results.mat'), 'R', 'p', 'S');

%% 14. Animation from the symbolic solution (fanimator)
% Mechanism_symbolic.m (Part II, stand-alone so it can be pasted and run) repeats
% the symbolic kinematics and animates disc, slider, rocker and 4 coins with
% fanimator: 1 coin starts on the stem, 3 roll in from the feed, one per turn.
% It also writes symbolic/Mechanism_symbolic.avi. Its numbers are typed in, so
% they are checked against params.m here.
q = runPart2(fullfile(here, 'Mechanism_symbolic.m'));
pairs = {'e', 'e'; 'Rd', 'Rd'; 'Dcoin', 'D'; 'chanW', 'chanW'; 'stemLen', 'stemLen'; 'stemW', 'stemW';
    'a', 'a'; 'h', 'h'; 'L2', 'L2'; 'rr', 'rr'; 'rGate', 'rGate'; 'sPin', 'sPin'; 'cPawl', 'cPawl';
    'dFeed', 'dFeed'; 'phi0', 'phi0'; 'L1', 'L1'; 'b', 'b'; 'omega', 'w'};
same = cellfun(@(f, g) abs(p.(f) - q.(g)) < 1e-12, pairs(:,1), pairs(:,2));
same(end+1) = isequal(q.seg, [p.rise p.ret]);
assert(all(same), 'Mechanism_symbolic.m numbers differ from params.m: %s', strjoin(pairs(~same(1:end-1), 1), ', '));
fprintf('Mechanism_symbolic.m uses the same numbers as params.m (%d checked)\n', numel(same));

function q = runPart2(file)
% Run the stand-alone Part II script in its own workspace; return its parameters.
run(file);
q = p;
end

function nm = uniqueName()
% Valid, unused function name for a generated file (tempname gives 'tp...').
[~, nm] = fileparts(tempname);
end

function [tk, v1] = rollTime(L, v0, a)
% Time to cover L from speed v0 at constant acceleration a, and the end speed.
v1 = sqrt(v0^2 + 2*a*L);
tk = (v1 - v0)/a;
end

function Y = evalPts(f, thv, rows)
% Evaluate a matlabFunction point by point (piecewise output is not vectorised).
Y = zeros(rows, numel(thv));
for k = 1:numel(thv)
    Y(:,k) = f(thv(k));
end
end
