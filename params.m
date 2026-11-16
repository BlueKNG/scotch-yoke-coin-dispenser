function p = params()
%PARAMS Single source of truth for every dimension and input of the mechanism.
% SI units (m, rad, s, kg). Frame: origin at disc centre, x right, y up,
% angles CCW from +x. Symbolic, simulation, CAD and video all read this file.
%
% Function (as in the reference clip): coin elevator. The slider stem pushes
% the bottom coin of a vertical column up by one coin per turn; the "7" hook of
% the rocker is a pawl that holds the column while the slider goes back down;
% a 45 deg feed channel rolls the next coin onto the stem at bottom dead centre.

% Input
p.rpm   = 30;                       % disc speed [rev/min]
p.omega = p.rpm*2*pi/60;            % [rad/s]
p.g     = 9.81;                     % [m/s^2]

% Workpiece (Thai 1 baht coin size)
p.Dcoin = 0.020;                    % coin diameter [m]
p.mCoin = 0.003;                    % coin mass [kg]
p.chanW = p.Dcoin + 0.001;          % coin channel width [m]
p.cDrag = 0.05;                     % effective rolling + wall drag coefficient of a coin in the
                                    % channel (estimate; speed limits are swept over 0..0.15)

% Link 2: disc = crank + face cam (groove)
p.Rd = 0.060;                       % disc radius [m]
p.e  = 0.018;                       % crank pin eccentricity [m]; stroke 2e = 1.8 coin. Needed so the
                                    % coin held by the pawl clears the 45 deg feed path (contact sim jammed at 1.4)

% Link 5: motor pinion driving teeth cut on the disc rim (spur gears, 20 deg)
% The disc tip circle stays Rd. The pinion sits below-right of the disc, where
% nothing else is on the plate, and its small gear motor mounts in front.
p.mGear    = 0.0015;                % module [m] (large enough for FDM teeth)
p.Ng       = 78;                    % disc teeth: pitch radius 58.5 mm, tip radius Rd
p.Np       = 20;                    % pinion teeth (>= 17: no undercut)
p.gearAng  = deg2rad(-60);          % pinion centre direction from the disc centre
p.etaGear  = 0.8;                   % printed spur pair efficiency (estimate)
p.rpmMotor = p.rpm*p.Ng/p.Np;       % motor speed for p.rpm at the disc [rev/min]
% Drive motor: yellow TT gear motor 1:48, single output shaft (bought). Typical seller
% values, not measured: no-load speed vs supply voltage, and stall torque at 3 V.
p.motorName = 'TT gear motor 1:48';
p.ttNoLoad  = [3 125; 4.5 170; 6 230];   % [V, rpm] (estimate)
p.ttStall3V = 0.08;                 % stall torque at 3 V [N m] (about 0.8 kgf cm, estimate)

% Link 3: slider (Scotch yoke), guided on x = 0; its stem is the coin pusher
p.ms      = 0.015;                  % slider mass [kg] (estimate)
p.stemLen = 0.075;                  % stem top above the slot centreline [m]
p.stemW   = 0.007;                  % stem width [m] (narrower than the column, pawl fits beside it)
p.guideY  = [0.030 0.056];          % guide extent in y [m], always covered by the stem

% Pawl target (where the selector pin must sit when it holds the column)
p.rGate = 0.002;                    % pawl pin radius [m]
p.sPin  = 0.008;                    % pin centre offset from the column axis when IN [m]
p.cPawl = 0.002;                    % pin-to-coin clearance at top dead centre [m]
p.dFeed = 0.003;                    % feed floor above the stem top at bottom dead centre [m]
rc = p.Dcoin/2;
p.yRest = sqrt((rc + p.rGate)^2 - p.sPin^2);   % coin centre above the pin when it rests on it
p.yIn   = p.stemLen + p.e + rc - p.yRest - p.cPawl;   % pin centre height when IN

% Link 4: rocker (pawl) on a FIXED frame pivot. The follower pin sits level
% with the disc centre and far out, so it moves almost radially on the cam.
p.a  = 0.044;                       % pivot x [m]
p.h  = 0.048;                       % pivot y [m] (|P| = 65 mm: the r 3.5 pivot post clears the disc)
p.L2 = 0.048;                       % pivot -> follower pin centre [m]
p.rr = 0.002;                       % follower pin radius [m]
p.mr = 0.005;                       % rocker mass [kg] (estimate)

% Rocker motion law (cam synthesis input), segments of one disc turn [rad]
% phi = angle of the lower arm from straight down, CCW positive.
% -phi0 = pawl OUT (clear of the column), +phi0 = pawl IN (holds the column).
% Slider top dead centre is th = 90 deg, bottom dead centre th = 270 deg.
p.phi0   = deg2rad(4.5);            % swing: phi in [-phi0, +phi0]
p.dwell1 = deg2rad([0 70]);         % OUT while the stem pushes the column up
p.rise   = deg2rad([70 110]);       % OUT -> IN around top dead centre (cycloidal)
p.dwell2 = deg2rad([110 270]);      % IN: column rests on the pin, stem goes down, new coin rolls in
p.ret    = deg2rad([270 315]);      % IN -> OUT after the new coin is on the stem (cycloidal)
p.dwell3 = deg2rad([315 360]);      % OUT

% "7" hook: arm L1 up from the pivot, bar b to the left, pin at the tip.
% Solved so the pin is exactly at (sPin, yIn) when phi = +phi0.
s = sin(p.phi0); c = cos(p.phi0);
x = [-s -c; c -s] \ [p.sPin - p.a; p.yIn - p.h];
p.L1 = x(1);                        % pivot -> top corner of the hook [m]
p.b  = x(2);                        % hook bar length to the left [m]
p.Ir = p.mr*(p.L1 + p.L2)^2/12;     % rocker inertia about pivot [kg m^2] (slender bar)

% Manufacturing (3D print) and design limits
p.clear    = 0.00015;               % groove clearance per side [m]
p.rHub     = 0.0025;                % disc centre hole radius [m] (M5 shaft)
p.rPin     = 0.0015;                % crank pin hole radius [m] (M3)
p.alphaMax = deg2rad(30);           % max allowed pressure angle
end
