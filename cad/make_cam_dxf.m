%% Cam disc outline -> cam_disc.dxf (mm) and pitch curve -> cam_pitch.csv
% Import the DXF into a Fusion 360 sketch, extrude the disc, cut the groove
% (between the two closed polylines) and the two holes.
here = fileparts(mfilename('fullpath'));
if isempty(here), here = pwd; end
root = fileparts(here);
load(fullfile(root, 'symbolic', 'results.mat'), 'R', 'p');

C = R.C(:, 1:end-1);                          % drop duplicate end point
Cp = R.Cp(:, 1:end-1);
n = [-Cp(2,:); Cp(1,:)]./vecnorm(Cp);
halfW = p.rr + p.clear;
wallA = 1e3*(C + halfW*n);
wallB = 1e3*(C - halfW*n);

fid = fopen(fullfile(here, 'cam_disc.dxf'), 'w');
fprintf(fid, '0\nSECTION\n2\nENTITIES\n');
circ(fid, 0, 0, 1e3*p.Rd);                    % disc rim
circ(fid, 0, 0, 1e3*p.rHub);                  % shaft hole
circ(fid, 1e3*p.e, 0, 1e3*p.rPin);            % crank pin hole (disc frame, th = 0)
poly(fid, wallA); poly(fid, wallB);           % groove walls
fprintf(fid, '0\nENDSEC\n0\nEOF\n');
fclose(fid);

writematrix([1e3*C' (0:size(C,2)-1)'*rad2deg(R.th(2))], fullfile(here, 'cam_pitch.csv'));
fprintf('Groove width %.2f mm, radius %.1f .. %.1f mm\n', 2e3*halfW, ...
    min(vecnorm(wallB)), max(vecnorm(wallA)));

function circ(fid, x, y, r)
fprintf(fid, '0\nCIRCLE\n8\n0\n10\n%.4f\n20\n%.4f\n30\n0\n40\n%.4f\n', x, y, r);
end

function poly(fid, P)
fprintf(fid, '0\nPOLYLINE\n8\n0\n66\n1\n70\n1\n');
fprintf(fid, '0\nVERTEX\n8\n0\n10\n%.4f\n20\n%.4f\n30\n0\n', P);
fprintf(fid, '0\nSEQEND\n');
end
