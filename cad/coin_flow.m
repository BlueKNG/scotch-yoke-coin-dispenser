function [XY, chk] = coin_flow(p, R, Hp, t)
%COIN_FLOW Scripted coin positions in the elevator loop at times t [s].
% Not a contact simulation: coins are placed by simple rules so the timing of
% stem, pawl and feed can be checked and animated.
%   - the column is one stack; its bottom rests on whichever is higher: the
%     stem (or the new coin on it) or the pawl pin (when the pin is under it)
%   - coins past the hood peak roll down the loop at vRoll and queue in the feed
%   - the head of the queue rolls onto the stem when the stem top is below the
%     feed floor, and waits behind the lowest column coin if that coin is in
%     the way (feedBlocked is flagged if it is still in the way at entry)
% XY{k} is 2 x nCoins (m). chk lists rule violations (should all be empty).
rc = p.Dcoin/2; D = p.Dcoin; rg = p.rGate;
vRoll = 0.15;                                   % coin rolling speed [m/s]
sHead = Hp.L - (rc + p.stemW/2)*sqrt(2);        % head coin resting on the stem side
per = @(y, th) interp1([R.th(1:end-1) 2*pi], [y(1:end-1) y(1)], th);

% Initial state at t(1): stack on the stem, the rest queued
th0 = mod(p.omega*t(1), 2*pi);
stemS = per(R.yS, th0) + p.stemLen + rc - Hp.yC0;
nS = floor((Hp.sPeak - stemS)/D) + 1;
sb = stemS;
% Head clearance along the 45 deg feed: distance back from the column axis at
% which a feed coin touches a column coin with centre height yb
rTouch = @(yb) feedTouch(Hp.yC0 - yb, D);
q = min(sHead, Hp.L - rTouch(Hp.yC0 + sb)) - (0:Hp.nCoins - nS - 1)*D;   % queue, head first
newc = NaN; entering = false;
chk = struct('pawlLifts', [], 'pawlHitsNewCoin', [], 'stackOnBareStem', [], 'queueFull', [], ...
    'feedBlocked', []);

XY = cell(1, numel(t));
for k = 1:numel(t)
    dt = 0; if k > 1, dt = t(k) - t(k-1); end
    th = mod(p.omega*t(k), 2*pi);
    stemS = per(R.yS, th) + p.stemLen + rc - Hp.yC0;
    G = [per(R.G(1,:), th); per(R.G(2,:), th)];
    if G(1) < rc + rg
        pawlS = G(2) + sqrt((rc + rg)^2 - G(1)^2) - Hp.yC0;
    else
        pawlS = -inf;
    end

    % Queue: coins roll toward their slot; the head enters when the stem is low
    q = sort(q, 'descend');
    yLow = Hp.yC0 + sb;                                % lowest column coin
    if ~isnan(newc), yLow = Hp.yC0 + newc; end
    head = min(sHead, Hp.L - rTouch(yLow));
    slotFree = yLow - Hp.yC0 >= D - 1e-4;             % no coin sitting in the column-bottom slot
    if ~entering && isnan(newc) && slotFree && stemS <= 0 && ~isempty(q) && q(1) >= head - 1e-6
        entering = true;
        if rTouch(yLow) > 5e-4, chk.feedBlocked(end+1) = t(k); end   % column coin in the way
    end
    for j = 1:numel(q)
        tgt = head - (j - 1)*D;
        if j == 1 && entering, tgt = Hp.L; end
        if j > 1 && entering, tgt = tgt + D; end
        q(j) = min(tgt, q(j) + vRoll*dt);
    end
    if entering && q(1) >= Hp.L - 1e-9
        q(1) = []; newc = stemS; entering = false;
    end
    if ~isnan(newc), newc = stemS; end

    % Stack bottom: highest support
    support = stemS;
    if ~isnan(newc), support = stemS + D; end
    sbNew = max(pawlS, support);
    if pawlS >= support && k > 1 && sbNew > sb + 5e-5
        chk.pawlLifts(end+1) = t(k);                    % pin pushes the stack up = jam
    end
    if isnan(newc) && pawlS < stemS && sbNew < sb - 1e-3 && stemS > -1e-3 - p.dFeed
        chk.stackOnBareStem(end+1) = t(k);             % fell onto the stem, no new coin
    end
    sb = sbNew;
    if ~isnan(newc) && support >= pawlS - 1e-9
        nS = nS + 1; sb = stemS; newc = NaN;           % new coin joins the stack
    end
    % Coins over the hood peak leave the stack and roll down
    while sb + (nS - 1)*D > Hp.sPeak + 1e-3
        q(end+1) = sb + (nS - 1)*D; nS = nS - 1; %#ok<AGROW>
    end
    if ~isnan(newc) && norm(G - colPt(Hp, newc)) < rc + rg - 5e-5
        chk.pawlHitsNewCoin(end+1) = t(k);
    end
    if sHead - (numel(q) - 1)*D < Hp.sPeak + D
        chk.queueFull(end+1) = t(k);                    % queue slots back up onto the hood
    end

    s = [sb + (0:nS - 1)*D, q];
    if ~isnan(newc), s = [newc, s]; end %#ok<AGROW>
    XY{k} = cell2mat(arrayfun(@(v) colPt(Hp, v), s, 'UniformOutput', false));
end
end

function r = feedTouch(d, D)
% Distance r back along the 45 deg feed (from the column axis at the feed
% height) where a feed coin touches a column coin whose centre is d below the
% feed height (d < 0: above). r = 0 means the feed path is clear.
if d^2 >= 2*D^2, r = 0; return; end
u = (-d + sqrt(2*D^2 - d^2))/2;
r = max(0, sqrt(2)*u);
end

function x = colPt(Hp, s)
% Point at arc length s on the loop (s < 0 continues down the column axis)
if s < 0, x = [0; Hp.yC0 + s]; return; end
s = mod(s, Hp.L);
j = find(Hp.s <= s, 1, 'last'); j = min(j, numel(Hp.s) - 1);
a = Hp.path(:, j); b = Hp.path(:, j + 1);
x = a + (b - a)*(s - Hp.s(j))/(Hp.s(j + 1) - Hp.s(j));
end
