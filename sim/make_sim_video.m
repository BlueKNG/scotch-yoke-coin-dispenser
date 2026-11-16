%% Video of the Simscape coin simulation -> video/Mechanism.avi
% Plays back what Mechanism_coins.slx computed (run_coins saves it in coins_result.mat):
% disc angle, slider position and rocker angle from the joints, and every coin position
% from the contact simulation. Nothing here is scripted. Colours are the built model's.
%   make_sim_video            3 turns at the design speed (coins_result.mat)
%   make_sim_video(rpm)       the run saved by run_coins(nRev, rpm) -> video/Mechanism_<rpm>rpm.avi
function make_sim_video(rpm)
here = fileparts(mfilename('fullpath'));
root = fileparts(here);
addpath(root, fullfile(root, 'cad'));
tag = ''; if nargin > 0 && ~isempty(rpm), tag = sprintf('_%drpm', rpm); end
D = load(fullfile(here, ['coins_result' tag '.mat']));
S = load(fullfile(root, 'symbolic', 'results.mat'), 'R', 'p');
R = S.R; p = S.p; Hp = hopper_geometry(p, R);
T = D.simTs; mm = 1e3; rc = mm*p.Dcoin/2;
C = struct('base', [20 40 120]/255, 'disc', [110 180 235]/255, 'slider', [200 30 30]/255, ...
    'rocker', [240 200 20]/255, 'hopper', [70 70 75]/255, 'coin', [120 50 160]/255);

fps = 30; tEnd = D.t(end); tf = 0:1/fps:tEnd;
th = interp1(T.tth, T.th, tf, 'linear', 'extrap');
yS = interp1(T.ty, T.y, tf, 'linear', 'extrap');
ph = interp1(T.tp, T.phi, tf, 'linear', 'extrap');
X = interp1(D.t, D.X, tf, 'linear', 'extrap'); Y = interp1(D.t, D.Y, tf, 'linear', 'extrap');

% Rocker outline in its own frame (pivot at 0,0): follower arm, upright, hook bar to the pawl pin
rk = mm*[0 -p.L2; 0 p.L1; -p.b p.L1]';
tc = linspace(0, 2*pi, 100);
outName = ['Mechanism' tag '.avi'];
vw = VideoWriter(fullfile(root, 'video', outName), 'Motion JPEG AVI');
vw.FrameRate = fps; vw.Quality = 92; open(vw);
f = figure('Position', [100 100 620 840], 'Color', 'w'); theme(f, 'light');
for n = 1:numel(tf)
    clf; ax = axes('Position', [0 0 1 0.95]); hold(ax, 'on'); axis(ax, 'equal', 'off');
    axis(ax, [-85 130 -95 205]);
    fill(ax, [-82 128 128 -82], [-92 -92 205 205], C.base, 'EdgeColor', 'none');
    Rg = [cos(th(n)) -sin(th(n)); sin(th(n)) cos(th(n))];
    fill(ax, mm*p.Rd*cos(tc), mm*p.Rd*sin(tc), C.disc, 'EdgeColor', 'none');
    Cg = Rg*R.C; plot(ax, mm*Cg(1,:), mm*Cg(2,:), 'Color', [0.05 0.2 0.6], 'LineWidth', 3);
    mk = Rg*[0 -p.Rd*0.9; 0 0]; plot(ax, mm*mk(1,:), mm*mk(2,:), 'Color', [1 1 1], 'LineWidth', 2);
    for W = Hp.walls'
        plot(ax, mm*W{1}(1,:), mm*W{1}(2,:), 'Color', C.hopper, 'LineWidth', 4);
    end
    % slider: yoke bar + stem
    rectangle(ax, 'Position', mm*[-p.e - 0.006, yS(n) - 0.005, 2*p.e + 0.012, 0.010], ...
        'FaceColor', C.slider, 'EdgeColor', 'none');
    rectangle(ax, 'Position', mm*[-p.stemW/2, yS(n) + 0.005, p.stemW, p.stemLen - 0.005], ...
        'FaceColor', C.slider, 'EdgeColor', 'none');
    plot(ax, mm*p.e*cos(th(n)), mm*p.e*sin(th(n)), 'o', 'MarkerFaceColor', C.disc, ...
        'MarkerEdgeColor', 'k', 'MarkerSize', 7);
    for c = 1:size(X, 2)
        rectangle(ax, 'Position', [mm*X(n,c) - rc, mm*Y(n,c) - rc, 2*rc, 2*rc], 'Curvature', 1, ...
            'FaceColor', C.coin, 'EdgeColor', [0.3 0.1 0.4]);
    end
    Rp = [cos(ph(n)) -sin(ph(n)); sin(ph(n)) cos(ph(n))];
    Q = mm*[p.a; p.h] + Rp*rk;
    plot(ax, Q(1,:), Q(2,:), 'Color', C.rocker, 'LineWidth', 6);
    plot(ax, mm*p.a, mm*p.h, 'o', 'MarkerFaceColor', [0.1 0.1 0.1], 'MarkerEdgeColor', 'k', 'MarkerSize', 7);
    plot(ax, 0, 0, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 6);
    text(ax, -80, -86, sprintf('t = %.2f s   disc %.0f rpm   slider y = %+5.1f mm   rocker %+5.1f deg', ...
        tf(n), D.simTs.rpm, mm*yS(n), rad2deg(ph(n))), 'FontSize', 10, 'Color', 'w');
    title(ax, 'Simscape Multibody: coin contact simulation', 'FontSize', 12);
    writeVideo(vw, getframe(f));
end
close(vw);
fprintf('Wrote %d frames (%.1f s) to video/%s\n', numel(tf), tEnd, outName);
end
