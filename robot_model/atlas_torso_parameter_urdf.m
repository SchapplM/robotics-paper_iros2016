% Return Parameter of the Atlas Robot (all versions) (hard coded)
% 
% Input:
% 
% Output:
% p_num [1x5]
%   Numeric Values of Kinematic Parameters
% rSges_num [5x3]
%   Center of Gravity of all independent bodies (torso only) in urdf frames
% m_num [5x1]
%   masses of all independent bodies (torso only)
% Iges_num  [5x6]
%   inertia of all independent bodies (torso only)
%     rows: bodies
%     columns: inertia tensor entries. Order: xx, yy, zz, xy, xz, yz
% 
% Sources:
% [1] atlas_v3.urdf

% Moritz Schappler, schappler@irt.uni-hannover.de, 2014-11
% (c) Institut für Regelungstechnik, Leibniz Universität Hannover

function [p_num, rSges_num, m_num, Iges_num] = atlas_torso_parameter_urdf(version)

assert(isa(version,'uint8') && all(size(version) == [1 1]), ...
  'version to be [1x1] uint8');  

%% URDF Parameter
if version == 3
  [p_num, rSges_num, m_num, Iges_num] = atlas3_torso_parameter_urdf();
elseif version == 4
  [p_num, rSges_num, m_num, Iges_num] = atlas4_torso_parameter_urdf();
elseif version == 5
  [p_num, rSges_num, m_num, Iges_num] = atlas5_torso_parameter_urdf();
else
  error('Version %d not implemented', version);
end