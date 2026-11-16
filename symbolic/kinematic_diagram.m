%% Kinematic diagram with link and joint labels (needs results.mat from Mechanism_src)
here = fileparts(mfilename('fullpath'));
if isempty(here), here = pwd; end
root = fileparts(here);
load(fullfile(here, 'results.mat'), 'R', 'p');

k  = find(R.th >= deg2rad(40), 1);           % draw at th = 40 deg
mm = 1e3;
th = R.th(k); yS = R.yS(k);
pin = p.e*[cos(th); sin(th)];
P = R.P(:,k); T = R.T(:,k); G = R.G(:,k); Hk = R.H(:,k);

f = figure('Position', [100 100 700 900]); theme(f, 'light'); hold on; axis equal off
tc = linspace(0, 2*pi, 200);
% Link 2: disc (crank + cam groove)
plot(mm*p.Rd*cos(tc), mm*p.Rd*sin(tc), 'Color', [0.6 0.6 0.6]);
Cg = [cos(th) -sin(th); sin(th) cos(th)]*R.C;
plot(mm*Cg(1,:), mm*Cg(2,:), '--', 'Color', [0.3 0.3 0.9]);
plot(mm*[0 pin(1)], mm*[0 pin(2)], 'b', 'LineWidth', 3);
% Link 3: slider with horizontal slot; its stem top pushes the coin column
plot(mm*[0 0], mm*[yS yS + p.stemLen], 'k', 'LineWidth', 4);
plot(mm*[-p.e - 0.004, p.e + 0.004], mm*[yS yS], 'k', 'LineWidth', 4);
% Link 4: rocker (pawl)
plot(mm*[T(1) P(1) Hk(1) G(1)], mm*[T(2) P(2) Hk(2) G(2)], 'm', 'LineWidth', 3);   % "7" hook
plot(mm*G(1), mm*G(2), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
text(mm*G(1), mm*G(2) + 7, 'pawl pin G', 'FontSize', 11, 'HorizontalAlignment', 'center');
% Ground: disc bearing, rocker pivot and slider guide
plot(0, 0, 'ko', 'MarkerSize', 12, 'MarkerFaceColor', 'w', 'LineWidth', 2);
groundSymbol(0, 0); groundSymbol(mm*P(1), mm*P(2));
gy = mm*mean(p.guideY); gx = mm*p.stemW/2 + 1;
plot(-[gx gx], [gy - 10, gy + 10], 'k', [gx gx], [gy - 10, gy + 10], 'k', 'LineWidth', 1.5);
for s = -5:4:5, plot(-gx - [4 0], [gy + s - 3, gy + s + 1], 'k'); plot(gx + [0 4], [gy + s + 1, gy + s - 3], 'k'); end
% Link 5: motor pinion, pitch circles touching at the mesh point
rg = mm*p.mGear*p.Ng/2; rp = mm*p.mGear*p.Np/2;
Q = (rg + rp)*[cos(p.gearAng); sin(p.gearAng)];
plot(rg*cos(tc), rg*sin(tc), ':', 'Color', [0.2 0.6 0.2]);
plot(Q(1) + rp*cos(tc), Q(2) + rp*sin(tc), 'Color', [0.2 0.6 0.2], 'LineWidth', 2.5);
plot(Q(1), Q(2), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'w', 'LineWidth', 2);
groundSymbol(Q(1), Q(2));
% Joints
plot(mm*pin(1), mm*pin(2), 'ro', 'MarkerSize', 9, 'MarkerFaceColor', 'r');
plot(mm*P(1), mm*P(2), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'w', 'LineWidth', 2);
plot(mm*T(1), mm*T(2), 'mo', 'MarkerSize', 9, 'MarkerFaceColor', 'm');
lab = @(x, y, s) text(x, y, s, 'FontSize', 12, 'FontWeight', 'bold');
lab(-22, -4, 'J1 (R 1-2)');
lab(-gx - 38, gy, 'J2 (P 1-3)');
lab(mm*P(1) + 5, mm*P(2) + 2, 'J3 (R 1-4)');
lab(mm*pin(1) + 3, mm*pin(2) - 6, 'J4 (pin-slot 2-3)');
lab(mm*T(1) + 3, mm*T(2) - 9, 'J5 (pin-groove 2-4)');
lab(-14, -22, 'Link 1 (ground)');
lab(-58, -45, 'Link 2 (disc + cam)');
lab(-60, mm*(yS + p.stemLen) - 4, 'Link 3 (slider / pusher)');
lab(mm*Hk(1) - 75, mm*Hk(2) + 16, 'Link 4 (rocker / pawl)');
lab(Q(1) + 8, Q(2) + 4, 'J6 (R 1-5)');
lab(Q(1) - 34, Q(2) - 27, 'Link 5 (motor pinion)');
M = rg*[cos(p.gearAng); sin(p.gearAng)];
lab(M(1) - 50, M(2) - 13, 'J7 (gear 2-5)');
axis([-70 100 -95 115]);
title(sprintf('Kinematic diagram at \\theta = %.0f deg   (n = 5, j1 = 4, j2 = 3, F = 1)', rad2deg(th)));
exportgraphics(f, fullfile(root, 'figures', 'kinematic_diagram.png'), 'Resolution', 150);

function groundSymbol(x0, y0)
% Hatched ground below a fixed joint at (x0, y0) [mm]
plot(x0 + [-12 12], y0 - [10 10], 'k', 'LineWidth', 1.5); plot([x0 x0], y0 - [10 0], 'k');
for s = -12:4:8, plot(x0 + [s s + 4], y0 - [14 10], 'k'); end
end
