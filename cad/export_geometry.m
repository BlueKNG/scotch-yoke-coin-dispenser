%% Export params + solved geometry to cad/geometry.json for build_parts.py
% Units in the JSON are mm and rad.
here = fileparts(mfilename('fullpath'));
if isempty(here), here = pwd; end
root = fileparts(here);
addpath(root);
load(fullfile(root, 'symbolic', 'results.mat'), 'R', 'p');
Hp = hopper_geometry(p, R);

mm = 1e3;
lengths = {'Dcoin', 'chanW', 'Rd', 'e', 'stemLen', 'stemW', 'guideY', 'rGate', 'sPin', ...
    'yIn', 'a', 'h', 'L2', 'L1', 'b', 'rr', 'clear', 'rPin'};
J.p = struct();
for k = 1:numel(lengths), J.p.(lengths{k}) = mm*p.(lengths{k}); end
J.p.phi0 = p.phi0;
J.p.mGear = mm*p.mGear; J.p.Ng = p.Ng; J.p.Np = p.Np; J.p.gearAng = p.gearAng;
J.R.th = R.th; J.R.yS = mm*R.yS; J.R.phi = R.phi;
J.R.C = mm*R.C; J.R.G = mm*R.G;
J.hopper.path = mm*Hp.path;
J.hopper.window = mm*Hp.window;
J.hopper.yBot = mm*(p.stemLen - p.e - 0.004);
J.hopper.yC0 = mm*Hp.yC0;
fid = fopen(fullfile(here, 'geometry.json'), 'w');
fprintf(fid, '%s', jsonencode(J, 'PrettyPrint', true));
fclose(fid);
fprintf('Wrote %s\n', fullfile(here, 'geometry.json'));
