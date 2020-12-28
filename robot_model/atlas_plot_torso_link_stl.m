% Plot one torso link of the Atlas model with STL files from drcsim
% 
% Input:
% j
%   Number of Robot Body
% version
%   Atlas Model Version
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


function hdl = atlas_plot_torso_link_stl(j, version, T_b_urdf)

%% Init

STLpath = fileparts(which('atlas_stl_location'));
AS = atlas_const(version);

%% Assemble Filename
if version == 3 % the same STL files for both versions
  STLpath = fullfile(STLpath, 'meshes_v3');
elseif version == 4 || version == 5
  STLpath = fullfile(STLpath, 'meshes_unplugged');
else
  error('Version %d not implemented yet', version);
end
filename = [AS.LN{AS.LI_Torso(j)}, '.stl'];

%% read STL
[version, f, n, c, ~] = stlread(fullfile(STLpath, filename), 0);

%% Move and rotate to link reference frame 
for i = 1:size(n,1)
    n(i,:) = (T_b_urdf(1:3,1:3)*n(i,:)')';
end
for i = 1:size(version,1)
    version(i,:) = (eye(3,4)*T_b_urdf*[version(i,:)';1])';
end

% draw STL (grey color for torso)
hdl = patch('Faces', f, 'Vertices', version,'FaceVertexCData',c, ...
    'FaceColor', [0.5, 0.5, 0.5], 'FaceAlpha', 0.53, ...
    'EdgeColor', [0.2, 0.2, 0.2], 'EdgeAlpha', 0.54);