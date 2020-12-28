% Return MDH Parameter of the Atlas Robot Leg (v4) (hard coded)
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
% [1] atlas_v4.urdf
% [2] [KhalilKle1986]

function [a_mdh, d_mdh, alpha_mdh, q_offset_mdh, ...
  rSges_num_mdh, m_num_mdh, Iges_num_mdh] = atlas4_leg_parameter_mdh(lr)

assert(isa(lr,'logical') && all(size(lr) == [1 1]), ...
  'Left/Right flag has to be [1x1] logical');  

[p_num_urdf, rSges_num_urdf, m_num_urdf, Iges_num_urdf] = atlas4_leg_parameter_urdf(lr);


%% Calculate MDH kinematic Parameter
alpha_mdh = zeros(6,1);
a_mdh = zeros(6,1);
q_offset_mdh = zeros(6,1);
d_mdh = zeros(6,1);

% set MDH parameters with values from urdf-file
% Um z-Achse so drehen, dass nächste Drehachse die y0-Achse wird
q_offset_mdh(1) = pi/2;

alpha_mdh(2) = pi/2;
q_offset_mdh(2) = pi/2;
d_mdh(2) = p_num_urdf(2);

alpha_mdh(3) = pi/2;
a_mdh(3) = p_num_urdf(4);
phi = -atan(p_num_urdf(5)/p_num_urdf(6));
q_offset_mdh(3) = -phi;
d_mdh(3) = p_num_urdf(3);

a_mdh(4) = -sqrt(p_num_urdf(5)^2 + p_num_urdf(6)^2);
q_offset_mdh(4) = phi;

a_mdh(5) = p_num_urdf(7);

alpha_mdh(6) = -pi/2;

%% Calculate MDH dynamic parameters

% Different frame position and orientation

m_num_mdh = m_num_urdf;

[rSges_num_mdh, Iges_num_mdh] = atlas4_leg_convert_urdf_mdh(lr, rSges_num_urdf, Iges_num_urdf);