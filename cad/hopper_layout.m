%% Coin elevator layout (2D) at four disc angles -> figures/hopper_layout.png
% Also runs the scripted coin flow for 3 turns and reports the timing checks.
here = fileparts(mfilename('fullpath'));
if isempty(here), here = pwd; end
root = fileparts(here);
addpath(root);
load(fullfile(root, 'symbolic', 'results.mat'), 'R', 'p');
Hp = hopper_geometry(p, R);
fprintf('Pawl pin to wall clearance: %.1f mm -> %s\n', 1e3*Hp.clearance, ...
    string(Hp.ok).replace('true', 'OK').replace('false', 'CLASH'));

% Coin flow over 3 turns, checks, and the states shown in the figure
Tp = 2*pi/p.omega;
t = (0:0.002:3*Tp);
[XY, chk] = coin_flow(p, R, Hp, t);
for f = fieldnames(chk)'
    v = chk.(f{1});
    if isempty(v), fprintf('%-16s: OK\n', f{1});
    else, fprintf('%-16s: FAIL %d times, first at theta = %.0f deg\n', f{1}, numel(v), ...
            rad2deg(mod(p.omega*v(1), 2*pi)));
    end
end
fprintf('Coins in loop: %d, lifted per stroke: %d\n', Hp.nCoins, Hp.nLift);

mm = 1e3; rc = mm*p.Dcoin/2; D = mm*p.Dcoin; tc = linspace(0, 2*pi, 100);
states = [30 100 200 290];
names = {'stem pushes the column up, pawl OUT', 'top dead centre, pawl moves IN', ...
         'column on the pawl, stem goes down', 'new coin on the stem, pawl OUT'};
f = figure('Position', [50 50 1500 700]); theme(f, 'light');
for s = 1:4
    subplot(1, 4, s); hold on; axis equal; axis([-85 130 -95 210]); box on
    tk = 2*Tp + deg2rad(states(s))/p.omega;
    [~, n] = min(abs(t - tk));
    k = find(R.th >= deg2rad(states(s)), 1);
    rectangle('Position', [-82 -92 210 297], 'EdgeColor', [0.6 0.6 0.6], 'LineStyle', '--');
    plot(mm*p.Rd*cos(tc), mm*p.Rd*sin(tc), 'Color', [0.7 0.7 0.7]);
    yS = mm*R.yS(k); T = mm*R.T(:,k); H = mm*R.H(:,k); G = mm*R.G(:,k);
    rectangle('Position', [-mm*p.stemW/2, yS + 5, mm*p.stemW, mm*p.stemLen - 5], ...
        'FaceColor', [0.85 0.75 0.55], 'EdgeColor', 'k');
    rectangle('Position', [-mm*p.e - 5, yS - 5, 2*mm*p.e + 10, 10], ...
        'FaceColor', [0.85 0.75 0.55], 'EdgeColor', 'k');
    for W = Hp.walls', plot(mm*W{1}(1,:), mm*W{1}(2,:), 'k', 'LineWidth', 2); end
    for c = XY{n}
        rectangle('Position', [mm*c(1) - rc, mm*c(2) - rc, D, D], 'Curvature', 1, ...
            'FaceColor', [0.75 0.78 0.85], 'EdgeColor', [0.3 0.3 0.4]);
    end
    plot([T(1) mm*p.a H(1) G(1)], [T(2) mm*p.h H(2) G(2)], 'm', 'LineWidth', 3);
    plot(mm*p.a, mm*p.h, 'ko', 'MarkerFaceColor', 'w', 'MarkerSize', 7);
    plot(G(1), G(2), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 6);
    title({sprintf('\\theta = %d deg', states(s)), names{s}}, 'FontSize', 9);
    xlabel('x [mm]'); if s == 1, ylabel('y [mm]'); end
end
exportgraphics(f, fullfile(root, 'figures', 'hopper_layout.png'), 'Resolution', 130);
