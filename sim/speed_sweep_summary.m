%% Speed sweep: contact sim (run_coins(nRev, rpm)) vs the analytical speed limits
% Reads sim/coins_result*.mat and symbolic/results.mat (section 13 of Mechanism_src).
% Writes figures/speed_sweep.png and prints one line per speed.
here = fileparts(mfilename('fullpath'));
if isempty(here), here = pwd; end
root = fileparts(here);
A = load(fullfile(root, 'symbolic', 'results.mat'), 'S'); A = A.S;
files = [{fullfile(here, 'coins_result.mat')}, ...
    arrayfun(@(f) fullfile(f.folder, f.name), dir(fullfile(here, 'coins_result_*rpm.mat')), 'UniformOutput', false)'];
rows = [];
for k = 1:numel(files)
    D = load(files{k});
    tok = regexp(files{k}, '_(\d+)rpm', 'tokens');
    if isempty(tok), rpm = 30; else, rpm = str2double(tok{1}{1}); end
    nRev = round(D.t(end)*rpm/60);
    dmin = inf;
    for c = 1:size(D.X, 2)
        for d = c + 1:size(D.X, 2)
            dmin = min(dmin, min(hypot(D.X(:,c) - D.X(:,d), D.Y(:,c) - D.Y(:,d))));
        end
    end
    % Feed success: a coin centre arrives on the column axis below the feed level
    % (one per turn when the feed works). Hood crossings include the coin that is
    % already at the peak at t = 0, so the feed count is the cleaner measure.
    fed = 0;
    for c = 1:size(D.X, 2)
        col = abs(D.X(:,c)) < 0.002 & D.Y(:,c) < 0.075;
        fed = fed + sum(diff(col) == 1);
    end
    rows(end+1, :) = [rpm, nRev, fed, D.over, D.off, 1e3*dmin, 1e3*max(abs(D.Tin.Data)), D.wall]; %#ok<AGROW>
end
rows = sortrows(rows, 1);
fprintf('%6s %5s %5s %6s %5s %10s %10s %7s\n', 'rpm', 'turns', 'fed', 'over', 'off', 'dmin[mm]', 'Tmax[mNm]', 'wall[s]');
fprintf('%6d %5d %5d %6d %5d %10.2f %10.2f %7.0f\n', rows');
fprintf('Mean input power: %.2f mW at 30 rpm, %.2f mW at %.0f rpm\n', ...
    1e3*interp1(A.Nv, A.Pavg, 30), 1e3*interp1(A.Nv, A.Pavg, A.N_lift), A.N_lift);

f = figure('Position', [100 100 700 420]); theme(f, 'light'); hold on; grid on
xline(A.N_feedSimple, 'r:', 'feed, first model', 'LineWidth', 1.2);
xline(A.N_feed, 'r--', 'feed entry', 'LineWidth', 1.5);
xline(A.N_lift, 'm--', 'lift-off', 'LineWidth', 1.5);
plot(rows(:,1), rows(:,3)./rows(:,2), 'ko-', 'MarkerFaceColor', 'k', 'LineWidth', 1.5);
plot(rows(:,1), rows(:,4)./rows(:,2), 'bs--');
yline(1, 'g-');
ylim([0 1.6]); xlim([0 max(rows(:,1)) + 30]);
xlabel('disc speed N [rpm]'); ylabel('events per turn');
legend('', '', '', 'coins fed onto the stem', 'coins over the hood', 'target (1 per turn)', 'Location', 'southwest');
title('Contact simulation vs analytical speed limits');
exportgraphics(f, fullfile(root, 'figures', 'speed_sweep.png'), 'Resolution', 150);
