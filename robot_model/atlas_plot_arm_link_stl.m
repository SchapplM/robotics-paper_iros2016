% Plot one arm link of the Atlas model with STL files from drcsim
% 
% Input:
% j
%   Number of Robot Arm Segment
% lr [1x1 logical]
%   true for left, false for right
% version [1x1 uint8]
%   Atlas Model Version number
% T_b_urdf
%   Body frame in urdf coordinates

% Output:
% hdl
%   handle to the created patch object
% 
% Sources:
% [1] Atlas Simulator STL files
% [2] atlas_v4.urdf
% [3] atlas_v5.urdf
% 
% TODO: Fix Position of Atlas v4 Arm segments

% Moritz Schappler, schappler@irt.uni-hannover.de, 2014-11
% (c) Institut für Regelungstechnik, Universität Hannover


function hdl = atlas_plot_arm_link_stl(j, lr, version, T_b_urdf)

%% Init
assert(isa(lr,'logical') && all(size(lr) == [1 1]), ...
  'Left/Right flag has to be [1x1] logical');   
assert(isa(version,'uint8') && all(size(version) == [1 1]), ...
  'version number has to be [1x1] uint8'); 

STLpath = fileparts(which('atlas_stl_location'));
AS = atlas_const(version, lr);
%% Assemble Filename
if version == 3 % the same STL files for both versions
  STLpath = fullfile(STLpath, 'meshes_v3');
elseif version == 4
  STLpath = fullfile(STLpath, 'meshes_unplugged');
elseif version == 5
  if j <= 3
    STLpath = fullfile(STLpath, 'meshes_unplugged');
  end
  if j > 3
    % Der Unterarm sieht anders aus. Die STL-Dateien in meshes_unplugged
    % sind dafür geändert worden. Diese sind hier in den Ordner meshes_v5
    % umgezogen.
    STLpath = fullfile(STLpath, 'meshes_v5');
  end
else
  error('Version %d not implemented yet', version);
end

if version == 5
  AS = atlas_const(4, lr); %TODO KLÄREN OB HIER FÜNF STEHEN SOLL
  % See [3], <mesh filename="..."> for ufarm, lfarm, hand
  % number | name    | STL name
  % 1      | l_clav  | l_clav
  % 2      | l_scap  | l_scap
  % 3      | l_uarm  | l_uarm
  % 4      | l_larm  | l_larm.dae
  % 5      | l_ufarm | l_farm
  % 6      | l_lfarm | l_hand.stl
  % 7      | l_hand  | /
  if lr
    T_c_urdf_l_new_old = atlas5_simple_shapes_stf_plot_transformation();
    T_b_urdf = T_b_urdf * T_c_urdf_l_new_old(:,:,j);
  end
  if j == 7
    hdl = NaN;
    return; % take meshes from atlas v4
  end
end

filename = [AS.LN{AS.LI_Arm(j)}, '.stl'];
%% Move origin of STL
% if version == 4 && j == 1
%   % Atlas v4 has the same STL files but different position
%   if lr == true
%     T_b_urdf = T_b_urdf*transl([0; -0.048; -0.45]); % see [2], <link name="l_clav">
%   else
%     T_b_urdf = T_b_urdf*transl([0;  0.048; -0.45]); % see [2], <link name="l_clav">
%   end
% end

%% STL Operation
% read STL

[v, f, n, c, ~] = stlread(fullfile(STLpath, filename), 0);

% Move and rotate to link reference frame 
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