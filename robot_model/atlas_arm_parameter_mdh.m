% Return MDH Parameter of the Atlas Robot (different versions)
% 
% Input:
% lr
%   true for left, false for right
% version
%   3, 4 or 5
% 
% Output: N joints
% a_mdh, d_mdh, alpha_mdh, q_offset_mdh
%   Numeric Values of Kinematic Parameters, MDH-Notation
% rSges_num [Nx3]
%   Center of Gravity of all indepedent bodies (arm only) in urdf frames
% m_num [Nx1]
%   masses of all indepedent bodies (arm only)
% Iges_num  [Nx6]
%   inertia of all indepedent bodies (arm only)
%     rows: bodies
%     columns: inertia tensor entries. Order: xx, yy, zz, xy, xz, yz
% 
% Sources:
% [1] [KhalilKle1986]

% Moritz Schappler, schappler@irt.uni-hannover.de, 2014-11
% (c) Institut für Regelungstechnik, Leibniz Universität Hannover

function [a_mdh, d_mdh, alpha_mdh, q_offset_mdh, ...
  rSges_num_mdh, m_num_mdh, Iges_num_mdh] = atlas_arm_parameter_mdh(lr, version)

assert(isa(lr,'logical') && all(size(lr) == [1 1]), ...
  'Left/Right flag has to be [1x1] logical');  
assert(isa(version,'uint8') && all(size(version) == [1 1]), ...
  'version to be [1x1] uint8');  
%% URDF Parameter
if version == 3
  [a_mdh, d_mdh, alpha_mdh, q_offset_mdh, ...
  rSges_num_mdh, m_num_mdh, Iges_num_mdh] = atlas3_arm_parameter_mdh(lr);
elseif version == 4
  [a_mdh, d_mdh, alpha_mdh, q_offset_mdh, ...
  rSges_num_mdh, m_num_mdh, Iges_num_mdh] = atlas4_arm_parameter_mdh(lr);
elseif version == 5
  [a_mdh, d_mdh, alpha_mdh, q_offset_mdh, ...
  rSges_num_mdh, m_num_mdh, Iges_num_mdh] = atlas5_arm_parameter_mdh(lr);
else
  error('Version %d not implemented', version);
end
