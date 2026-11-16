function Hp = hopper_geometry(p, R)
%HOPPER_GEOMETRY Closed coin loop of the elevator, units m, ground frame.
% Coin centre path (counter-clockwise): up the column on x = 0 (pushed by the
% stem), up-left through a 45 deg hood, down a 12 deg ramp (gravity), down a
% vertical drop, then a 45 deg feed back to the stem at bottom dead centre.
% Walls are the path offset by half the channel width on each side. The right
% column wall has a window for the pawl pin. R (results) is optional; it gives
% the pin path for the window and the clearance check.
hw = p.chanW/2; rc = p.Dcoin/2; D = p.Dcoin;

% Coin centre path
yC0  = p.stemLen - p.e + p.dFeed + rc;    % feed coin centre on the column axis
yLip = 0.165;                             % column top: hood starts here
P0 = [0; yC0];
P1 = [0; yLip];
P1b = P1 + [-0.015; 0.015];               % hood peak; coins roll freely beyond it
xL = -0.060;
P2 = [xL; P1b(2) - (P1b(1) - xL)*tand(12)];
P3 = [xL; yC0 - xL];                      % 45 deg feed ends at P0
path = [P0 P1 P1b P2 P3 P0];
seg = vecnorm(diff(path, 1, 2));
Hp.path = path; Hp.s = [0 cumsum(seg)]; Hp.L = Hp.s(end);
Hp.sPeak = Hp.s(3);                       % arc length of the hood peak
Hp.yC0 = yC0; Hp.D = D;

% Walls: offset corners (outer = right of travel, inner = left)
Vo = offsetLoop(path(:, 1:end-1), hw);
Vi = offsetLoop(path(:, 1:end-1), -hw);
yBot = p.stemLen - p.e - 0.004;           % column walls reach below the stem top at BDC
% Feed floor (outer side of the 45 deg feed) ends at the column's left wall
uF = (P0 - P3)/norm(P0 - P3);
F0 = Vo(:, 5);                            % outer corner at P3
floorEnd = F0 + uF*((-hw - F0(1))/uF(1));
% Pawl window in the right column wall
if nargin > 1
    win = [min(R.G(2,:)) - p.rGate - 0.001, max(R.G(2,:)) + p.rGate + 0.001];
else
    win = p.yIn + [-p.rGate - 0.001, 0.008];
end
Hp.walls = {
    [hw hw; yBot win(1)]                                   % right wall below the window
    [[hw; win(2)], Vo(:, 2:5), floorEnd]                    % right wall, hood, ramp, drop, feed floor
    [[-hw; yBot], floorEnd]                                 % left wall under the feed floor
    [Vi, Vi(:, 1)]                                          % inner island
    };
Hp.names = {'Right_low', 'Outer', 'Left_low', 'Island'};
Hp.window = win; Hp.floorEnd = floorEnd;

% Coins: the column from the stem to the hood peak is lifted each turn
Hp.nLift = ceil((Hp.sPeak + yC0 - (p.stemLen - p.e + rc))/D);
Hp.nCoins = Hp.nLift + 4;                 % plus a short queue in the feed

% Check: pin path keeps clear of every wall (pin radius + 0.5 mm)
if nargin > 1
    dmin = inf;
    for k = 1:numel(Hp.walls)
        W = Hp.walls{k};
        for j = 1:size(W, 2) - 1
            a = W(:,j); s = W(:,j+1) - a;
            tt = max(0, min(1, (s'*(R.G - a))/(s'*s)));
            dmin = min(dmin, min(vecnorm(R.G - (a + s.*tt))));
        end
    end
    Hp.clearance = dmin - p.rGate;
    Hp.ok = Hp.clearance > 0.0005;
end
end

function V = offsetLoop(Q, d)
% Corners of a closed polyline Q (2xN) offset by d to the right of travel
N = size(Q, 2); V = zeros(2, N);
for j = 1:N
    a = Q(:, mod(j - 2, N) + 1); b = Q(:, j); c = Q(:, mod(j, N) + 1);
    u1 = (b - a)/norm(b - a); u2 = (c - b)/norm(c - b);
    n1 = [u1(2); -u1(1)]; n2 = [u2(2); -u2(1)];
    t = [u1, -u2] \ ((b + d*n2) - (b + d*n1));       % intersect the two offset lines
    V(:, j) = b + d*n1 + t(1)*u1;
end
end
