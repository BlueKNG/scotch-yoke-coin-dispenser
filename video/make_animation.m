%% Preview animation from the analytical solution -> Mechanism_preview.avi
% This is NOT the multibody simulation; it shows the designed motion. Coins are
% placed by the scripted rules in cad/coin_flow.m, not simulated with contact.
here = fileparts(mfilename('fullpath'));
if isempty(here), here = pwd; end
root = fileparts(here);
addpath(root, fullfile(root, 'cad'));
load(fullfile(root, 'symbolic', 'results.mat'), 'R', 'p');
Hp = hopper_geometry(p, R);

mm = 1e3; tc = linspace(0, 2*pi, 120); rc = mm*p.Dcoin/2; D = mm*p.Dcoin;
nRev = 3;
t = 0:1/30:nRev*2*pi/p.omega;              % real time at 30 frame/s
[XY, chk] = coin_flow(p, R, Hp, t);
per = @(y, th) interp1([R.th(1:end-1) 2*pi], [y(1:end-1) y(1)], th);
ptAt = @(Y, th) [per(Y(1,:), th); per(Y(2,:), th)];

vw = VideoWriter(fullfile(here, 'Mechanism_preview.avi'), 'Motion JPEG AVI');
vw.FrameRate = 30; vw.Quality = 95;
open(vw);
f = figure('Position', [100 100 600 800]); theme(f, 'light');
for n = 1:numel(t)
    th = mod(p.omega*t(n), 2*pi);
    clf; hold on; axis equal off
    axis([-85 130 -95 205]);
    yS = p.e*sin(th);
    Rg = [cos(th) -sin(th); sin(th) cos(th)];
    rectangle('Position', [-82 -92 210 297], 'LineStyle', '--', 'EdgeColor', [0.7 0.7 0.7]);
    % Disc with groove, crank pin and a reference mark
    fill(mm*p.Rd*cos(tc), mm*p.Rd*sin(tc), [0.92 0.92 0.95], 'EdgeColor', [0.5 0.5 0.5]);
    Cg = Rg*R.C;
    plot(mm*Cg(1,:), mm*Cg(2,:), 'Color', [0.3 0.3 0.8], 'LineWidth', 4);
    mk = Rg*[0 -p.Rd*0.9; 0 0];
    plot(mm*mk(1,:), mm*mk(2,:), 'Color', [0.6 0.6 0.6], 'LineWidth', 2);
    pin = p.e*[cos(th); sin(th)];
    % Slider: yoke with slot + stem (coin pusher); ground guides
    rectangle('Position', mm*[-p.e - 0.005, yS - 0.005, 2*p.e + 0.010, 0.010], ...
        'FaceColor', [0.85 0.75 0.55], 'EdgeColor', 'k');
    plot(mm*[-p.e - 0.0015, p.e + 0.0015], mm*[yS yS], 'k', 'LineWidth', 3);
    rectangle('Position', mm*[-p.stemW/2, yS + 0.005, p.stemW, p.stemLen - 0.005], ...
        'FaceColor', [0.85 0.75 0.55], 'EdgeColor', 'k');
    gx = mm*(p.stemW/2 + 0.001);
    plot([-gx -gx], mm*p.guideY, 'k', [gx gx], mm*p.guideY, 'k', 'LineWidth', 2);
    plot(mm*pin(1), mm*pin(2), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 7);
    % Coin loop walls and coins
    for W = Hp.walls'
        plot(mm*W{1}(1,:), mm*W{1}(2,:), 'k', 'LineWidth', 2.5);
    end
    for c = XY{n}
        rectangle('Position', [mm*c(1) - rc, mm*c(2) - rc, D, D], 'Curvature', 1, ...
            'FaceColor', [0.78 0.8 0.86], 'EdgeColor', [0.3 0.3 0.4]);
    end
    % Rocker (pawl) with "7" hook, fixed pivot (front layer)
    T = ptAt(R.T, th); H = ptAt(R.H, th); G = ptAt(R.G, th);
    plot(mm*[T(1) p.a H(1) G(1)], mm*[T(2) p.h H(2) G(2)], 'm', 'LineWidth', 4);
    plot(mm*p.a, mm*p.h, 'ko', 'MarkerFaceColor', 'w', 'MarkerSize', 8, 'LineWidth', 1.5);
    plot(mm*T(1), mm*T(2), 'mo', 'MarkerFaceColor', 'm', 'MarkerSize', 7);
    plot(mm*G(1), mm*G(2), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 7);
    plot(0, 0, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 6);
    text(-80, -80, sprintf('\\theta = %3.0f deg   y_S = %+5.1f mm   \\phi = %+4.1f deg', ...
        rad2deg(th), mm*yS, rad2deg(per(R.phi, th))), 'FontSize', 11);
    title('Coin elevator: Scotch yoke pusher + face cam pawl (analytical preview)');
    writeVideo(vw, getframe(f));
end
close(vw);
fprintf('Wrote %d frames; coin rule violations: %d\n', numel(t), ...
    sum(structfun(@numel, chk)));
