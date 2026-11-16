%% Run Mechanism_sim.slx and compare with the symbolic solution (results.mat)
here = fileparts(mfilename('fullpath'));
if isempty(here), here = pwd; end
root = fileparts(here);
addpath(here, root);
S = load(fullfile(root, 'symbolic', 'results.mat'), 'R', 'p');
R = S.R; p = S.p;

simOut = sim(fullfile(here, 'Mechanism_sim.slx'));
get = @(n) simOut.get(n);
t = get('sim_y').Time;
th = mod(p.omega*t, 2*pi);
% Analytical values at the simulated angles (periodic interpolation)
A = @(y) interp1([R.th(1:end-1) 2*pi], [y(1:end-1) y(1)], th, 'pchip');

pairs = {
    'slider y [m]',        get('sim_y').Data,      A(R.yS)
    'slider v [m/s]',      get('sim_v').Data,      A(R.vS)
    'slider a [m/s^2]',    get('sim_a').Data,      A(R.aS)
    'rocker phi [rad]',    get('sim_phi').Data,    A(R.phi)
    'rocker phid [rad/s]', get('sim_phid').Data,   A(R.phid)
    'rocker phidd [rad/s^2]', get('sim_phidd').Data, A(R.phidd)
    'input torque [N m]',  get('sim_Tin').Data,    A(R.Tin)
    };
rows = cell(size(pairs, 1), 4);
for k = 1:size(pairs, 1)
    s = squeeze(pairs{k, 2}); a = squeeze(pairs{k, 3});
    err = max(abs(s - a)); amp = max(abs(a));
    rows(k,:) = {pairs{k, 1}, amp, err, 100*err/amp};
end
T = cell2table(rows, 'VariableNames', {'Quantity', 'MaxAnalytical', 'MaxAbsError', 'ErrorPercent'});
disp(T)
writetable(T, fullfile(here, 'sim_vs_symbolic.csv'));

f = figure('Position', [100 100 1000 700]); theme(f, 'light');
titles = pairs(:, 1);
for k = 1:6
    subplot(3, 2, k); hold on; grid on
    [ths, i] = sort(rad2deg(th));
    a = squeeze(pairs{k, 3}); s = squeeze(pairs{k, 2});
    plot(ths, a(i), 'b-', 'LineWidth', 2);
    plot(ths, s(i), 'r--', 'LineWidth', 1.2);
    title(titles{k}); xlabel('\theta [deg]'); xlim([0 360]);
    if k == 1, legend('symbolic', 'Simscape', 'Location', 'best'); end
end
exportgraphics(f, fullfile(root, 'figures', 'sim_vs_symbolic.png'), 'Resolution', 150);
