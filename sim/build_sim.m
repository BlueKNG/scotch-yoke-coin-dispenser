function build_sim(withCoins, coinSel, rpm)
%BUILD_SIM Build the Simscape Multibody model from params.m
%   build_sim()      -> Mechanism_sim.slx: mechanism only, checked against the
%                       symbolic solution (compare_sim.m)
%   build_sim(true)  -> Mechanism_coins.slx: same mechanism with the 3D-printed
%                       parts (cad/parts/*.step) as visuals and free coins that
%                       move by gravity and contact forces (walls, stem, pawl
%                       pin, each other) on planar joints
% Independent of the symbolic solution: only geometry, joints and the cam
% pitch curve (as a spline on the disc) go in. Outputs are logged to workspace.
%   build_sim(true, idx) keeps only coins idx (for diagnosing jams)
%   build_sim(true, [], rpm) runs the disc at rpm instead of params.m (speed tests)
if nargin < 1, withCoins = false; end
if nargin < 2, coinSel = []; end
here = fileparts(mfilename('fullpath'));
root = fileparts(here);
addpath(root, fullfile(root, 'cad'));
p = params();
C = asbuilt_colours();                            % print/cut colours of the built model (fabrication/COLOURS.md)
if nargin >= 3 && ~isempty(rpm)
    p.rpm = rpm; p.omega = rpm*2*pi/60; p.rpmMotor = rpm*p.Ng/p.Np;
end
S = load(fullfile(root, 'symbolic', 'results.mat'), 'R');
Hp = hopper_geometry(p, S.R);                   % hopper walls
pitch = S.R.C(:, 1:4:end-1)';                   % cam pitch curve on the disc [m], N x 2

m = 'Mechanism_sim';
if withCoins, m = 'Mechanism_coins'; end
if bdIsLoaded(m), close_system(m, 0); end
new_system(m); load_system('sm_lib'); load_system('nesl_utility');
lib = @(s) sprintf(s);                          % library paths contain newlines
pos = 0;
    function b = blk(src, name, varargin)
        pos = pos + 1;
        b = [m '/' name];
        add_block(lib(src), b, 'Position', [100 + 140*mod(pos, 8), 80*floor(pos/8) + 40, ...
            150 + 140*mod(pos, 8), 80*floor(pos/8) + 80], varargin{:});
    end
    function link(a, pa, b, pb)
        add_line(m, [a '/' pa], [b '/' pb], 'autorouting', 'on');
    end
    function n = rt(name, base, trans, axis, ang)
        % Rigid transform from frame "base" (block/port) with translation and optional rotation
        n = name;
        args = {'TranslationMethod', 'Cartesian', 'TranslationCartesianOffset', mat2str(trans), ...
            'TranslationCartesianOffsetUnits', 'm'};
        if nargin > 3
            args = [args, {'RotationMethod', 'StandardAxis', 'RotationStandardAxis', axis, ...
                'RotationAngle', num2str(ang), 'RotationAngleUnits', 'deg'}];
        end
        blk('sm_lib/Frames and\nTransforms/Rigid\nTransform', name, args{:});
        link(base{1}, base{2}, name, 'LConn1');
    end
    function brick(name, base, dims, centre, colour)
        % Graphic-only brick: inertia is carried by separate Inertia blocks
        t = rt([name '_T'], base, centre);
        blk('sm_lib/Body Elements/Brick Solid', name, 'BrickDimensions', mat2str(dims), ...
            'BrickDimensionsUnits', 'm', 'InertiaType', 'Custom', 'Mass', '0', ...
            'MomentsOfInertia', '[0 0 0]', 'ProductsOfInertia', '[0 0 0]', ...
            'GraphicDiffuseColor', mat2str(colour));
        link(t, 'RConn1', name, 'RConn1');
    end
    function wall(name, W, z, colour)
        % Graphic-only wall along segment W = [x1 x2; y1 y2] (ground frame)
        d = W(:,2) - W(:,1);
        t = rt([name '_T'], {'W', 'RConn1'}, [mean(W, 2)' z], '+Z', rad2deg(atan2(d(2), d(1))));
        blk('sm_lib/Body Elements/Brick Solid', name, 'BrickDimensions', mat2str([norm(d) 0.002 0.006]), ...
            'BrickDimensionsUnits', 'm', 'InertiaType', 'Custom', 'Mass', '0', ...
            'MomentsOfInertia', '[0 0 0]', 'ProductsOfInertia', '[0 0 0]', ...
            'GraphicDiffuseColor', mat2str(colour));
        link(t, 'RConn1', name, 'RConn1');
    end
    function cyl(name, base, r, len, centre, colour)
        % Graphic-only cylinder (axis = z), no mass
        t = rt([name '_T'], base, centre);
        blk('sm_lib/Body Elements/Cylindrical Solid', name, 'CylinderRadius', num2str(r), ...
            'CylinderRadiusUnits', 'm', 'CylinderLength', num2str(len), 'CylinderLengthUnits', 'm', ...
            'InertiaType', 'Custom', 'Mass', '0', 'MomentsOfInertia', '[0 0 0]', ...
            'ProductsOfInertia', '[0 0 0]', 'GraphicDiffuseColor', mat2str(colour));
        link(t, 'RConn1', name, 'RConn1');
    end
    function inertia(name, base, mass, Izz)
        blk('sm_lib/Body Elements/Inertia', name, 'InertiaType', 'Custom', ...
            'Mass', num2str(mass), 'MassUnits', 'kg', ...
            'CenterOfMass', '[0 0 0]', 'MomentsOfInertia', mat2str([1 1 1]*max(Izz, 1e-9)));
        link(base{1}, base{2}, name, 'RConn1');
    end
    function fsolid(name, base, part, colour, opacity)
        % Graphic-only STEP part from cad/parts (CAD frame = body frame, mm)
        blk('sm_lib/Body Elements/File Solid', name, 'ExtGeomFileName', ...
            fullfile(root, 'cad', 'parts', [part '.step']), 'UnitType', 'Custom', ...
            'ExtGeomFileUnits', 'mm', 'InertiaType', 'Custom', 'Mass', '0', ...
            'MomentsOfInertia', '[0 0 0]', 'ProductsOfInertia', '[0 0 0]', ...
            'GraphicVisPropType', 'AdvancedVisualProperties', 'GraphicDiffuseColor', mat2str(colour), ...
            'GraphicOpacity', num2str(opacity));
        link(base{1}, base{2}, name, 'RConn1');
    end
    function g = cbrick(name, base, dims, centre, angle)
        % Invisible contact brick; returns its geometry port
        t = rt([name '_T'], base, centre, '+Z', angle);
        blk('sm_lib/Body Elements/Brick Solid', name, 'BrickDimensions', mat2str(dims, 6), ...
            'BrickDimensionsUnits', 'm', 'ExportEntireGeometry', 'on', 'GraphicType', 'None', ...
            'InertiaType', 'Custom', 'Mass', '0', 'MomentsOfInertia', '[0 0 0]', ...
            'ProductsOfInertia', '[0 0 0]');
        link(t, 'RConn1', name, 'RConn1');
        g = {name, 'LConn1'};
    end
    function g = ccyl(name, base, r, len, centre, visible, colour)
        % Contact cylinder (axis z); returns its geometry port
        t = rt([name '_T'], base, centre);
        args = {'GraphicType', 'None'};
        if visible, args = {'GraphicDiffuseColor', mat2str(colour)}; end
        blk('sm_lib/Body Elements/Cylindrical Solid', name, 'CylinderRadius', num2str(r), ...
            'CylinderRadiusUnits', 'm', 'CylinderLength', num2str(len), 'CylinderLengthUnits', 'm', ...
            'ExportEntireGeometry', 'on', 'InertiaType', 'Custom', 'Mass', '0', ...
            'MomentsOfInertia', '[0 0 0]', 'ProductsOfInertia', '[0 0 0]', args{:});
        link(t, 'RConn1', name, 'RConn1');
        g = {name, 'LConn1'};
    end
    function contact(name, gB, gF)
        % Coin contact: soft spring-damper normal force + smooth stick-slip friction
        blk('sm_lib/Forces and Torques/Spatial Contact Force', name, 'NormalStiffness', '1e4', ...
            'NormalDamping', '5', 'NormalTransitionRegionWidth', '1e-4', ...
            'FrictionType', 'SmoothStickSlip', 'CoefficientOfStaticFriction', '0.4', ...
            'CoefficientOfDynamicFriction', '0.3', 'FrictionalCriticalVelocity', '5e-3');
        link(gB{1}, gB{2}, name, 'LConn1');
        link(gF{1}, gF{2}, name, 'RConn1');
    end
    function out(name, src, unit)
        cvt = [name '_ps2s'];   % not "c": nested functions share the parent's variables
        blk('nesl_utility/PS-Simulink\nConverter', cvt, 'Unit', unit);
        link(src{1}, src{2}, cvt, 'LConn1');
        blk('simulink/Sinks/To Workspace', [name '_ws'], 'VariableName', name, 'SaveFormat', 'Timeseries');
        add_line(m, [cvt '/1'], [name '_ws/1'], 'autorouting', 'on');
    end

% Environment
blk('sm_lib/Frames and\nTransforms/World Frame', 'W');
blk('sm_lib/Utilities/Mechanism\nConfiguration', 'MC', 'UniformGravity', 'Constant', ...
    'GravityVector', mat2str([0 -p.g 0]));
blk('nesl_utility/Solver\nConfiguration', 'SC');
link('W', 'RConn1', 'MC', 'RConn1');
link('W', 'RConn1', 'SC', 'RConn1');

% Link 1 graphics (ground): A4 back plate, centre shaft, slider guides
world = {'W', 'RConn1'};
if ~withCoins
    brick('Base_A4', world, [0.210 0.297 0.004], [0.023 0.0565 -0.012], C.base);
    cyl('Shaft', world, p.rHub - 0.0001, 0.020, [0 0 0], C.black);
    gl = diff(p.guideY); gc = mean(p.guideY); gx = p.stemW/2 + 0.0001 + 0.002;
    brick('Guide_L', world, [0.004 gl 0.008], [-gx gc 0.003], C.clear);
    brick('Guide_R', world, [0.004 gl 0.008], [gx gc 0.003], C.clear);
    % Coin loop walls in the slider layer (the stem pushes coins in this plane)
    for k = 1:numel(Hp.walls)
        W = Hp.walls{k};
        for j = 1:size(W, 2) - 1
            wall(sprintf('Hopper_%s_%d', Hp.names{k}, j), W(:, j:j+1), 0.003, C.hopper);
        end
    end
else
    % Printed parts as visuals (CAD layers: coin channel z = 9..13 mm)
    fsolid('Base_plate', world, 'base_plate', C.base, 1);
    fsolid('Axle_stub', world, 'axle_stub', C.black, 1);
    fsolid('Pivot_post', world, 'pivot_post', C.black, 1);
    fsolid('Hopper', world, 'hopper', C.hopper, 1);
    % No front cover here: even see-through it hid the coins in Mechanics Explorer.
    % It is graphic only (the coins touch the invisible wall bricks below).
    % Contact walls: thin bricks just behind each wall line, not extended past
    % the ends. Thick or extended bricks poke into the channel at the acute
    % (45 deg) island tip and jammed the feed in the first contact runs.
    zc = 0.011; tw = 0.0003; wallG = {};
    for k = 1:numel(Hp.walls)
        W = Hp.walls{k};
        for j = 1:size(W, 2) - 1
            a = W(:,j); d = W(:,j+1) - a; L = norm(d); u = d/L; n = [-u(2); u(1)];
            mid = a + d/2;
            if pathDist(Hp.path, mid + 1e-3*n) < pathDist(Hp.path, mid - 1e-3*n), n = -n; end
            c = mid + n*tw/2;
            wallG{end+1} = cbrick(sprintf('Wall_%s_%d', Hp.names{k}, j), world, ...
                [L, tw, 0.004], [c' zc], rad2deg(atan2(u(2), u(1)))); %#ok<AGROW>
        end
    end
end

% Drive angle with exact derivatives: theta = w*t for the disc
blk('simulink/Sources/Ramp', 'theta', 'slope', num2str(p.omega), 'start', '0', 'InitialOutput', '0');
blk('simulink/Sources/Constant', 'omega', 'Value', num2str(p.omega));
blk('simulink/Sources/Constant', 'alpha0', 'Value', '0');
if withCoins
    % Motor drives the pinion (J6); the disc follows through the gear mesh (J7).
    % Pinion angle = psi0 - (Ng/Np)*theta, psi0 = tooth phase from build_parts.py
    G6 = jsondecode(fileread(fullfile(root, 'cad', 'parts', 'pinion_phase.json')));
    kG = -p.Ng/p.Np;
    blk('simulink/Math Operations/Gain', 'k_theta', 'Gain', num2str(kG, 10));
    blk('simulink/Math Operations/Gain', 'k_omega', 'Gain', num2str(kG, 10));
    blk('simulink/Math Operations/Bias', 'psi0', 'Bias', num2str(G6.psi0, 10));
    add_line(m, 'theta/1', 'k_theta/1'); add_line(m, 'k_theta/1', 'psi0/1');
    add_line(m, 'omega/1', 'k_omega/1');
    drv = {'psi0/1', 'k_omega/1'};
else
    drv = {'theta/1', 'omega/1'};
end
blk('nesl_utility/Simulink-PS\nConverter', 'theta_s2ps', 'Unit', 'rad', ...
    'FilteringAndDerivatives', 'provide', 'UdotUserProvided', '2');
add_line(m, drv{1}, 'theta_s2ps/1'); add_line(m, drv{2}, 'theta_s2ps/2');
add_line(m, 'alpha0/1', 'theta_s2ps/3');
if ~withCoins
    % J1: disc revolute, driven directly
    blk('sm_lib/Joints/Revolute Joint', 'J1_Disc', 'MotionActuationMode', 'InputMotion', ...
        'TorqueActuationMode', 'ComputedTorque', 'SenseTorqueForce', 'on');
    link('W', 'RConn1', 'J1_Disc', 'LConn1');
    link('theta_s2ps', 'RConn1', 'J1_Disc', 'LConn2');
else
    % J1 free (starts at 0, sensed); J6 pinion revolute at Q, driven by the motor
    blk('sm_lib/Joints/Revolute Joint', 'J1_Disc', 'SensePosition', 'on', ...
        'PositionTargetSpecify', 'on', 'PositionTargetValue', '0', 'PositionTargetPriority', 'High');
    link('W', 'RConn1', 'J1_Disc', 'LConn1');
    qT = rt('Pinion_axis', {'W', 'RConn1'}, [G6.Q(:)'/1e3 0]);
    blk('sm_lib/Joints/Revolute Joint', 'J6_Pinion', 'MotionActuationMode', 'InputMotion', ...
        'TorqueActuationMode', 'ComputedTorque', 'SenseTorqueForce', 'on');
    link(qT, 'RConn1', 'J6_Pinion', 'LConn1');
    link('theta_s2ps', 'RConn1', 'J6_Pinion', 'LConn2');
    blk('sm_lib/Gears and\nCouplings/Gears/Common Gear\nConstraint', 'J7_GearMesh', ...
        'GearType', 'External', 'SpecificationType', 'CenterDistanceRatio', ...
        'CenterDistance', num2str(p.mGear*(p.Ng + p.Np)/2, 10), 'CenterDistanceUnits', 'm', ...
        'GearRatio', num2str(p.Np/p.Ng, 10));
    link('J1_Disc', 'RConn1', 'J7_GearMesh', 'LConn1');
    link('J6_Pinion', 'RConn1', 'J7_GearMesh', 'RConn1');
    blk('sm_lib/Body Elements/Inertia', 'Pinion_inertia', 'InertiaType', 'Custom', ...
        'Mass', '0.004', 'MassUnits', 'kg', 'CenterOfMass', '[0 0 0]', 'MomentsOfInertia', '[1 1 1]*5e-7');
    link('J6_Pinion', 'RConn1', 'Pinion_inertia', 'RConn1');
    fsolid('Pinion_part', {'J6_Pinion', 'RConn1'}, 'pinion', C.pinion, 1);
    fsolid('Motor_bracket', world, 'motor_bracket', C.black, 1);
    fsolid('Motor_TT', world, 'motor_TT', C.motor, 1);
end
disc = {'J1_Disc', 'RConn1'};
discT = rt('Disc_solid_T', disc, [0 0 -0.006]);
blk('sm_lib/Body Elements/Cylindrical Solid', 'Disc_solid', 'CylinderRadius', num2str(p.Rd), ...
    'CylinderRadiusUnits', 'm', 'CylinderLength', '0.006', 'CylinderLengthUnits', 'm', ...
    'GraphicDiffuseColor', mat2str(C.disc));
link(discT, 'RConn1', 'Disc_solid', 'RConn1');
pin = rt('Pin', disc, [p.e 0 0]);
if ~withCoins
    cyl('Crank_pin', {pin, 'RConn1'}, p.rPin, 0.010, [0 0 0.002], C.disc);
else
    set_param([m '/Disc_solid'], 'GraphicType', 'None');   % keeps its inertia, STEP part is the visual
    fsolid('Disc_part', disc, 'disc', C.disc, 1);
end
blk('sm_lib/Curves and Surfaces/Spline', 'Cam_groove', 'DataPoints', ...
    mat2str([pitch zeros(size(pitch, 1), 1)], 8), 'DataPointsUnits', 'm', 'EndConditionType', 'Closed', ...
    'GraphicDiffuseColor', '[0.05 0.2 0.6]');   % groove curve: darker than the light blue disc
link(disc{1}, disc{2}, 'Cam_groove', 'LConn1');

% J2: slider prismatic along world y (prismatic axis is z, so rotate in and out)
g = rt('Guide', {'W', 'RConn1'}, [0 0 0], '+X', -90);
blk('sm_lib/Joints/Prismatic\nJoint', 'J2_Slider', 'SensePosition', 'on', 'SenseVelocity', 'on', ...
    'SenseAcceleration', 'on');
link(g, 'RConn1', 'J2_Slider', 'LConn1');
sl = rt('Slider', {'J2_Slider', 'RConn1'}, [0 0 0], '+X', 90);
slider = {sl, 'RConn1'};
inertia('Slider_mass', slider, p.ms, 0);
if ~withCoins
    brick('Yoke', slider, [2*p.e + 0.012, 0.010, 0.004], [0 0 0.003], C.slider);
    brick('Stem', slider, [p.stemW, p.stemLen - 0.005, 0.004], [0 0.005 + (p.stemLen - 0.005)/2 0.003], C.slider);
else
    fsolid('Slider_part', slider, 'slider', C.slider, 1);
    stemG = cbrick('Stem_contact', slider, [p.stemW, p.stemLen - 0.005, 0.004], ...
        [0 0.005 + (p.stemLen - 0.005)/2 zc], 0);
end

% J4: crank pin in the yoke slot (slot along slider x)
blk('sm_lib/Joints/Pin Slot Joint', 'J4_PinSlot');
link(slider{1}, slider{2}, 'J4_PinSlot', 'LConn1');
link(pin, 'RConn1', 'J4_PinSlot', 'RConn1');

% J3: rocker revolute on the frame; phi = 0 means lower arm straight down
piv = rt('Pivot', world, [p.a p.h 0]);
blk('sm_lib/Joints/Revolute Joint', 'J3_Rocker', 'SensePosition', 'on', 'SenseVelocity', 'on', ...
    'SenseAcceleration', 'on', 'PositionTargetSpecify', 'on', 'PositionTargetPriority', 'Low', ...
    'PositionTargetValue', num2str(rad2deg(-p.phi0)), 'PositionTargetValueUnits', 'deg');
link(piv, 'RConn1', 'J3_Rocker', 'LConn1');
rocker = {'J3_Rocker', 'RConn1'};
inertia('Rocker_mass', rocker, p.mr, p.Ir);
fol = rt('Follower', rocker, [0 -p.L2 0]);
if ~withCoins
    brick('Gate', rocker, [0.006, p.L1 + p.L2 + 0.006, 0.004], [0 (p.L1 - p.L2)/2 0.008], C.rocker);
    brick('Hook_bar', rocker, [p.b + 0.006, 0.006, 0.004], [-p.b/2 p.L1 0.008], C.rocker);
    cyl('Pawl_pin', rocker, p.rGate, 0.010, [-p.b p.L1 0.005], C.rocker);
    cyl('Pivot_pin', rocker, 0.0015, 0.014, [0 0 0.006], C.black);
    cyl('Follower_pin', {fol, 'RConn1'}, p.rr, 0.012, [0 0 0.004], C.rocker);
else
    fsolid('Rocker_part', rocker, 'rocker', C.rocker, 1);
    pinG = ccyl('Pawl_contact', rocker, p.rGate, 0.004, [-p.b p.L1 zc], false, []);
end

% J5: follower pin centre stays on the groove pitch curve
blk('sm_lib/Constraints/Point on Curve\nConstraint', 'J5_PinGroove');
link('Cam_groove', 'RConn1', 'J5_PinGroove', 'LConn1');
link(fol, 'RConn1', 'J5_PinGroove', 'RConn1');

% Coins: free bodies on planar joints (x, y, spin) in the coin channel plane,
% starting where the scripted coin flow puts them at t = 0
if withCoins
    XY0 = coin_flow(p, S.R, Hp, 0); XY0 = XY0{1};
    if ~isempty(coinSel), XY0 = XY0(:, coinSel); end
    nC = size(XY0, 2); rc = p.Dcoin/2; tc = 0.0025;
    Ic = p.mCoin*[(3*rc^2 + tc^2)/12, (3*rc^2 + tc^2)/12, rc^2/2];
    coinG = cell(1, nC);
    for c = 1:nC
        nm = sprintf('Coin%02d', c);
        t0 = rt([nm '_start'], world, [XY0(:,c)' zc]);
        blk('sm_lib/Joints/Planar Joint', [nm '_J'], 'PxSensePosition', 'on', 'PySensePosition', 'on');
        link(t0, 'RConn1', [nm '_J'], 'LConn1');
        blk('sm_lib/Body Elements/Cylindrical Solid', nm, 'CylinderRadius', num2str(rc), ...
            'CylinderRadiusUnits', 'm', 'CylinderLength', num2str(tc), 'CylinderLengthUnits', 'm', ...
            'InertiaType', 'Custom', 'Mass', num2str(p.mCoin), ...
            'MomentsOfInertia', mat2str(Ic), 'ProductsOfInertia', '[0 0 0]', ...
            'GraphicDiffuseColor', mat2str(C.coin));
        link([nm '_J'], 'RConn1', nm, 'RConn1');
        % Contact shape: a sphere of the coin radius. Its section in the coin
        % plane is the coin itself, and sphere contact is far cheaper than
        % cylinder-cylinder (which made the first run ~1000x slower than real time).
        blk('sm_lib/Body Elements/Spherical Solid', [nm '_contact'], 'SphereRadius', num2str(rc), ...
            'SphereRadiusUnits', 'm', 'ExportEntireGeometry', 'on', 'GraphicType', 'None', ...
            'InertiaType', 'Custom', 'Mass', '0', 'MomentsOfInertia', '[0 0 0]', 'ProductsOfInertia', '[0 0 0]');
        link([nm '_J'], 'RConn1', [nm '_contact'], 'RConn1');
        coinG{c} = {[nm '_contact'], 'LConn1'};
        out(sprintf('coin%02d_x', c), {[nm '_J'], 'RConn2'}, 'm');
        out(sprintf('coin%02d_y', c), {[nm '_J'], 'RConn3'}, 'm');
    end
    % Coins run single file round the loop and cannot pass each other, so a
    % coin only meets its neighbours in loop order (up to 2 away, for margin).
    % run_coins checks that no other pair ever overlaps.
    [~, ord] = sort(arrayfun(@(c) pathS(Hp.path, XY0(:,c)), 1:nC));
    ringPos(ord) = 1:nC;
    nCC = 0;
    for c = 1:nC
        nm = sprintf('Coin%02d', c);
        for k = 1:numel(wallG), contact(sprintf('CF_%s_%s', nm, wallG{k}{1}), wallG{k}, coinG{c}); end
        contact(['CF_' nm '_stem'], stemG, coinG{c});
        contact(['CF_' nm '_pawl'], pinG, coinG{c});
        for d = c + 1:nC
            gap = abs(ringPos(c) - ringPos(d)); gap = min(gap, nC - gap);
            if gap <= 2
                contact(sprintf('CF_%s_Coin%02d', nm, d), coinG{c}, coinG{d}); nCC = nCC + 1;
            end
        end
    end
    fprintf('Coins: %d, contact pairs: %d\n', nC, nC*(numel(wallG) + 2) + nCC);
end

% Logged outputs (sensing port order: q, w, b; actuator torque on its own)
if ~withCoins
    out('sim_Tin', {'J1_Disc', 'RConn2'}, 'N*m');
else
    out('sim_Tmotor', {'J6_Pinion', 'RConn2'}, 'N*m');   % torque on the pinion shaft
    out('sim_theta', {'J1_Disc', 'RConn2'}, 'rad');      % disc angle through the gear mesh
end
out('sim_y', {'J2_Slider', 'RConn2'}, 'm');
out('sim_v', {'J2_Slider', 'RConn3'}, 'm/s');
out('sim_a', {'J2_Slider', 'RConn4'}, 'm/s^2');
out('sim_phi', {'J3_Rocker', 'RConn2'}, 'rad');
out('sim_phid', {'J3_Rocker', 'RConn3'}, 'rad/s');
out('sim_phidd', {'J3_Rocker', 'RConn4'}, 'rad/s^2');

if ~withCoins
    set_param(m, 'StopTime', num2str(2*2*pi/p.omega), 'Solver', 'ode15s', 'MaxStep', '1e-3', ...
        'RelTol', '1e-6', 'AbsTol', '1e-8', 'ReturnWorkspaceOutputs', 'on');
else
    % Contact is stiff and switches often: DAE solver, looser tolerances
    % 2 turns by default (enough for the video); run_coins sets its own length
    set_param(m, 'StopTime', num2str(2*2*pi/p.omega), 'Solver', 'daessc', 'MaxStep', '1e-3', ...
        'RelTol', '1e-4', 'AbsTol', '1e-6', 'ReturnWorkspaceOutputs', 'on');
end
colour_blocks(m, C);
Simulink.BlockDiagram.routeLine(find_system(m, 'FindAll', 'on', 'SearchDepth', 1, 'Type', 'line'));   % re-route lines after the blocks moved
save_system(m, fullfile(here, [m '.slx']));
fprintf('Saved %s\n', fullfile(here, [m '.slx']));
end

function C = asbuilt_colours()
% RGB (0..1) of the built model, same values as cad/fusion/fz_colours_asbuilt.py
C.base = [20 40 120]/255;     C.disc = [110 180 235]/255;  C.slider = [200 30 30]/255;
C.rocker = [240 200 20]/255;  C.pinion = [0 150 90]/255;   C.hopper = [70 70 75]/255;
C.black = [25 25 25]/255;     C.coin = [120 50 160]/255;   C.motor = [240 200 20]/255;
C.clear = [0.75 0.88 0.95];
end

function colour_blocks(m, C)
% Paint each block with its link's colour (the built model's colours) and place it in a labelled row band,
% so the diagram reads like the kinematic diagram: 1 ground, 2 disc, 3 slider, 4 rocker,
% 5 pinion, plus drive, hopper walls, coins, contacts and logged outputs.
tint = @(c) c;                          % full colour: same as the link in the 3D view
groups = {
  'Drive and solver (motor input, world frame, gravity)', [1 0.97 0.80], '^(W|MC|SC|theta|omega|alpha0|k_theta|k_omega|psi0|theta_s2ps)$';
  'Link 1: ground (base plate, shaft, guides, pivot, motor)', C.base, '^(Base.*|Shaft|Guide.*|Axle_stub|Pivot_post|Pivot|Motor_.*|Pinion_axis)$';
  'Hopper walls (ground)', C.hopper, '^(Hopper.*|Wall_.*)$';
  'Link 2: disc (crank pin + cam groove + gear rim)', tint(C.disc), '^(J1_Disc|Disc_.*|Pin|Crank_pin|Cam_groove)$';
  'Link 3: slider (Scotch yoke + stem)', tint(C.slider), '^(J2_Slider|Slider.*|Yoke|Stem.*)$';
  'Link 4: rocker (pawl)', tint(C.rocker), '^(J3_Rocker|Rocker_.*|Follower.*|Gate|Hook_bar|Pawl_.*|Pivot_pin)$';
  'Link 5: pinion and gear mesh', tint(C.pinion), '^(J6_Pinion|Pinion_.*|J7_GearMesh)$';
  'Higher pairs: pin in slot (2-3), pin in groove (2-4)', [1 1 1], '^(J4_PinSlot|J5_PinGroove)$';
  'Coins (free bodies)', tint(C.coin), '^Coin\d+.*$';
  'Coin contacts (walls, stem, pawl, other coins)', [0.90 0.86 0.95], '^CF_';
  'Logged outputs', [0.93 0.93 0.93], '_(ps2s|ws)$';
};
bl = find_system(m, 'SearchDepth', 1, 'Type', 'Block');
grp = zeros(size(bl));
for k = 1:numel(bl)
    nm = regexprep(get_param(bl{k}, 'Name'), '_T$', '');    % a transform follows its body
    for g = 1:size(groups, 1)
        if ~isempty(regexp(nm, groups{g, 3}, 'once'))
            grp(k) = g; set_param(bl{k}, 'BackgroundColor', mat2str(groups{g, 2}, 3)); break
        end
    end
end
% Layout: one band per group, 12 blocks per row
y = 40; perRow = 12; dx = 130; dy = 95;
for g = 1:size(groups, 1)
    idx = find(grp == g);
    if isempty(idx), continue, end
    [~, o] = sort(get_param(bl(idx), 'Name')); idx = idx(o);
    a = Simulink.Annotation(m, groups{g, 1}); a.Position = [40 y 700 y + 25];
    a.FontSize = 16; a.FontWeight = 'bold'; a.BackgroundColor = mat2str(groups{g, 2}, 3);
    if mean(groups{g, 2}) < 0.45, a.ForegroundColor = 'white'; end
    y = y + 45;
    for n = 1:numel(idx)
        r = floor((n - 1)/perRow); c = mod(n - 1, perRow);
        set_param(bl{idx(n)}, 'Position', [40 + dx*c, y + dy*r, 90 + dx*c, y + dy*r + 40]);
    end
    y = y + dy*ceil(numel(idx)/perRow) + 30;
end
end

function s = pathS(Pth, q)
% Arc length along the closed polyline Pth (2xN) of the point nearest to q
d = inf; s = 0; L = 0;
for j = 1:size(Pth, 2) - 1
    a = Pth(:,j); v = Pth(:,j+1) - a;
    tt = max(0, min(1, (v'*(q - a))/(v'*v)));
    dj = norm(q - a - v*tt);
    if dj < d, d = dj; s = L + tt*norm(v); end
    L = L + norm(v);
end
end

function d = pathDist(Pth, q)
% Distance from point q to the polyline Pth (2xN)
d = inf;
for j = 1:size(Pth, 2) - 1
    a = Pth(:,j); s = Pth(:,j+1) - a;
    t = max(0, min(1, (s'*(q - a))/(s'*s)));
    d = min(d, norm(q - a - s*t));
end
end
