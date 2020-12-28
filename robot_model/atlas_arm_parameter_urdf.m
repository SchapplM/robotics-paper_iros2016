% Return Parameter of the Atlas Robot (all versions) (hard coded)
% 
% Input:
% lr
%   true for left, false for right
% 
% Output:
% p_num [15x1]
%   Numeric Values of Kinematic Parameters
% rSges_num [6x3]
%   Center of Gravity of all indepedent bodies (arm only) in urdf frames
% m_num [6x1]
%   masses of all indepedent bodies (arm only)
% Iges_num  [6x6]
%   inertia of all indepedent bodies (arm only)
%     rows: bodies
%     columns: inertia tensor entries. Order: xx, yy, zz, xy, xz, yz

% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-02
% (c) Institut für Regelungstechnik, Leibniz Universität Hannover

function [p_num, rSges_num, m_num, Iges_num] = atlas_arm_parameter_urdf(lr, version)

%% Init
% Coder Information
%#codegen

assert(isa(lr,'logical') && all(size(lr) == [1 1]));      
assert(isa(version,'uint8') && all(size(version) == [1 1]), ...
  'version number has to be [1x1] uint8'); 

if version == 3
  [p_num, rSges_num, m_num, Iges_num] = atlas3_arm_parameter_urdf(lr);
elseif version == 4
  [p_num, rSges_num, m_num, Iges_num] = atlas4_arm_parameter_urdf(lr);
elseif version == 5
  [p_num, rSges_num, m_num, Iges_num] = atlas5_arm_parameter_urdf(lr);
else
  error('Version %d not implemented yet', version);
end