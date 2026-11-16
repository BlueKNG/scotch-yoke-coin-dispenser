function run_coins(nRev, rpm)
%RUN_COINS Build Mechanism_coins.slx, simulate nRev turns, check the coin flow.
% Pass criteria: one coin goes over the hood per turn, no coin leaves the
% channel. Writes sim/coins_result.mat and figures/coins_sim.png.
% run_coins(nRev, rpm) tests another disc speed; it writes sim/coins_result_<rpm>rpm.mat
% and figures/coins_sim_<rpm>rpm.png and leaves Mechanism_coins.slx at that speed.
% sim/make_sim_video.m turns the saved result into video/Mechanism.avi.
if nargin < 1, nRev = 3; end
if nargin < 2, rpm = []; end
here = fileparts(mfilename('fullpath'));
root = fileparts(here);
addpath(here, root, fullfile(root, 'cad'));
p = params();
tag = '';
if ~isempty(rpm)
    p.rpm = rpm; p.omega = rpm*2*pi/60; p.rpmMotor = rpm*p.Ng/p.Np;
    tag = sprintf('_%drpm', rpm);
end
S = load(fullfile(root, 'symbolic', 'results.mat'), 'R');
Hp = hopper_geometry(p, S.R);
build_sim(true, [], rpm);
m = 'Mechanism_coins';
set_param(m, 'StopTime', num2str(nRev*2*pi/p.omega));
tic; o = sim(m); wall = toc;
fprintf('Simulated %d turns in %.0f s wall time\n', nRev, wall);

XY0 = coin_flow(p, S.R, Hp, 0); XY0 = XY0{1};
nC = size(XY0, 2);
t = o.get('coin01_x').Time;
X = zeros(numel(t), nC); Y = X;
for c = 1:nC
    X(:,c) = XY0(1,c) + squeeze(o.get(sprintf('coin%02d_x', c)).Data);
    Y(:,c) = XY0(2,c) + squeeze(o.get(sprintf('coin%02d_y', c)).Data);
end
% The motor drives the pinion; the disc-side torque is Tmotor*Ng/Np (no gear losses in the sim)
Tm = o.get('sim_Tmotor'); Tin = Tm; Tin.Data = Tm.Data*p.Ng/p.Np;
thD = o.get('sim_theta');
fprintf('Disc angle error through the gear mesh: %.2e rad\n', ...
    max(abs(squeeze(thD.Data) - p.omega*thD.Time)));
fprintf('Max motor torque (sim, no losses): %.2f mN m at %.0f rpm\n', 1e3*max(abs(Tm.Data)), p.rpmMotor);

% Coins delivered: crossings of the hood peak line (x = P1b) going left
xPeak = Hp.path(1, 3); yHood = Hp.path(2, 2);
over = 0;
for c = 1:nC
    s = X(:,c) < xPeak & Y(:,c) > yHood;
    over = over + sum(diff(s) == 1);
end
% Coins outside the channel (centre farther than half-width + 1 mm from the path)
off = 0;
for k = 1:20:numel(t)
    for c = 1:nC
        q = [X(k,c); Y(k,c)];
        if q(2) > Hp.yC0 - p.e - 0.004 && distPath(Hp.path, q) > p.chanW/2 + 0.001, off = off + 1; end
    end
end
% Coin-coin overlap over all pairs (the model only has contact between loop neighbours)
dMin = inf;
for c = 1:nC
    for d = c + 1:nC
        dMin = min(dMin, min(hypot(X(:,c) - X(:,d), Y(:,c) - Y(:,d))));
    end
end
fprintf('Coins over the hood: %d in %d turns (target %d)\n', over, nRev, nRev);
fprintf('Closest coin centres: %.2f mm (coin D %.0f mm, overlap if much less)\n', 1e3*dMin, 1e3*p.Dcoin);
fprintf('Coin samples outside the channel: %d\n', off);
fprintf('Max input torque with coins: %.2f mN m\n', 1e3*max(abs(Tin.Data)));
% Joint motion logged by the model (used by make_sim_video.m)
sy = o.get('sim_y'); sp = o.get('sim_phi');
simTs = struct('ty', sy.Time, 'y', squeeze(sy.Data), 'tp', sp.Time, 'phi', squeeze(sp.Data), ...
    'tth', thD.Time, 'th', squeeze(thD.Data), 'rpm', p.rpm);
save(fullfile(here, ['coins_result' tag '.mat']), 't', 'X', 'Y', 'Tin', 'over', 'off', 'wall', 'simTs');

f = figure('Position', [100 100 1100 500]); theme(f, 'light');
subplot(1, 2, 1); plot(t, 1e3*Y); grid on
xlabel('t [s]'); ylabel('coin y [mm]'); title('Coin heights (contact simulation)');
subplot(1, 2, 2); hold on; axis equal; grid on
for W = Hp.walls', plot(1e3*W{1}(1,:), 1e3*W{1}(2,:), 'k', 'LineWidth', 1.5); end
plot(1e3*X, 1e3*Y, '.', 'MarkerSize', 3);
title(sprintf('Coin centre paths, %d turns', nRev)); xlabel('x [mm]'); ylabel('y [mm]');
exportgraphics(f, fullfile(root, 'figures', ['coins_sim' tag '.png']), 'Resolution', 130);
end

function d = distPath(Pth, q)
d = inf;
for j = 1:size(Pth, 2) - 1
    a = Pth(:,j); s = Pth(:,j+1) - a;
    tt = max(0, min(1, (s'*(q - a))/(s'*s)));
    d = min(d, norm(q - a - s*tt));
end
end
