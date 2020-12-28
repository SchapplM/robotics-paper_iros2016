% Plot one leg link of the Atlas model with STL files from drcsim
% 
% Input:
% j
%   Number of Robot Leg Segment
% lr
%   true = left, false = right
% version [1x1 uint8]
%   Atlas Model Version number
% T_b_urdf
%   Body frame in urdf coordinates
% 
% Output:
% hdl
%   handle to the created patch object
% 
% Sources:
% [1] Atlas Simulator STL files

% Moritz Schappler, schappler@irt.uni-hannover.de, 2014-11
% (c) Institut für Regelungstechnik, Universität Hannover


function hdl = atlas_plot_leg_link_stl(j, lr, version, T_b_urdf)

%% Init
STLpath = fileparts(which('atlas_stl_location'));

AS = atlas_const(version, lr);
%% Assemble Filename
if version == 3 % the same STL files for both versions
  STLpath = fullfile(STLpath, 'meshes_v3');
elseif version == 4 || version == 5
  STLpath = fullfile(STLpath, 'meshes_unplugged');
else
  error('Version %d not implemented yet', version);
end

filename = [AS.LN{AS.LI_Leg(j)}, '.stl'];

%% read STL
[v, f, n, c, ~] = stlread(fullfile(STLpath, filename), 0);

%% Move and rotate to link reference frame 
for i = 1:size(n,1)
    n(i,:) = (T_b_urdf(1:3,1:3)*n(i,:)')';
end
for i = 1:size(v,1)
    v(i,:) = (eye(3,4)*T_b_urdf*[v(i,:)';1])';
end

%% Plot
% Define colors
Arm_Colors = {'r', 'g', 'b', 'c', 'm', 'y', 'k'};

% draw STL
hdl = patch('Faces', f, 'Vertices', v,'FaceVertexCData',c, ...
    'FaceColor', 'k', 'FaceAlpha', 0.53, ...
    'EdgeColor', Arm_Colors{j}, 'EdgeAlpha', 0.54);