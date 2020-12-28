% Return MDH Parameter of the Atlas Robot (v4) (hard coded)
% 
% Input:
% lr
%   true for left, false for right
% 
% Output:
% a_mdh, d_mdh, alpha_mdh, q_offset_mdh [6x1]
%   Numeric Values of Kinematic Parameters, MDH-Notation
% rSges_num [6x3]
%   Center of Gravity of all indepedent bodies (arm only) in urdf frames
% m_num [6x1]
%   masses of all indepedent bodies (arm only)
% Iges_num  [6x6]
%   inertia of all indepedent bodies (arm only)
%     rows: bodies
%     columns: inertia tensor entries. Order: xx, yy, zz, xy, xz, yz
% 
% Sources:
% [1] atlas_v5_simple_shapes_with_head.urdf
% [2] [KhalilKle1986]

% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-03
% (c) Institut für Regelungstechnik, Universität Hannover

function [a_mdh, d_mdh, alpha_mdh, q_offset_mdh, ...
  rSges_num_mdh, m_num_mdh, Iges_num_mdh] = atlas5_arm_parameter_mdh(lr)

assert(isa(lr,'logical') && all(size(lr) == [1 1]), ...
  'Left/Right flag has to be [1x1] logical');  

%% URDF Parameter
[p_num_urdf, rSges_num_urdf, m_num_urdf, Iges_num_urdf] = atlas5_arm_parameter_urdf(lr);

%% Calculate MDH kinematic Parameter
alpha_mdh = zeros(7,1);
a_mdh = zeros(7,1);
q_offset_mdh = zeros(7,1);
d_mdh = zeros(7,1);

% set MDH parameters with values from urdf-file

d_mdh(1) = p_num_urdf(7);

alpha_mdh(2) = -pi/2;
if lr
  a_mdh(2) = p_num_urdf(6);
else
  a_mdh(2) = -p_num_urdf(6);
end
q_offset_mdh(2) = -pi/2;

alpha_mdh(3) = pi/2;
a_mdh(3) = p_num_urdf(9);
if lr
  d_mdh(3) = -p_num_urdf(8) - p_num_urdf(10);
else
  d_mdh(3) = p_num_urdf(8) + p_num_urdf(10);
end

alpha_mdh(4) = -pi/2;
a_mdh(4) = p_num_urdf(11);

% bis hierhin gleich

alpha_mdh(5) = pi/2;
a_mdh(5) = p_num_urdf(13);
if lr
  d_mdh(5) = -p_num_urdf(12);
else
  d_mdh(5) = p_num_urdf(12);
end

alpha_mdh(6) = -pi/2;

alpha_mdh(7) = pi/2;

%% Calculate MDH dynamic parameters

% Different frame position and orientation

m_num_mdh = m_num_urdf;
[rSges_num_mdh, Iges_num_mdh] = atlas5_arm_convert_urdf_mdh(lr, rSges_num_urdf, Iges_num_urdf);