%% Rebuild every generated file after changing params.m
root = fileparts(mfilename('fullpath'));
run(fullfile(root, 'symbolic', 'Mechanism_src.m'));
run(fullfile(root, 'symbolic', 'kinematic_diagram.m'));
run(fullfile(root, 'cad', 'make_cam_dxf.m'));
run(fullfile(root, 'cad', 'hopper_layout.m'));    % coin loop figure + timing checks
run(fullfile(root, 'cad', 'export_geometry.m'));  % geometry.json for build_parts.py (3D parts)
run(fullfile(root, 'video', 'make_animation.m'));
addpath(fullfile(root, 'sim'));
build_sim();                                   % Simscape model from params.m
run(fullfile(root, 'sim', 'compare_sim.m'));   % sim vs symbolic table + figure
% 3D parts must be current first: uv run --python 3.12 --with cadquery --with shapely python cad/build_parts.py
run_coins(3);                                  % coin contact sim (Mechanism_coins.slx), ~1 min
close_system('Mechanism_coins', 0);
% Live script copy of the analysis for submission (Mechanism.mlx)
matlab.internal.liveeditor.openAndSave(fullfile(root, 'symbolic', 'Mechanism_src.m'), ...
    fullfile(root, 'symbolic', 'Mechanism.mlx'));
