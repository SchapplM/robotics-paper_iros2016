% Return Parameter of the Atlas Robot (v5) (hard coded)
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
% [1] atlas_v5_simple_shapes_with_head.urdf
% 
% Only checked the first two joints

% Moritz Schappler, schappler@irt.uni-hannover.de, 2014-11
% (c) Institut für Regelungstechnik, Leibniz Universität Hannover

function [p_num, rSges_num, m_num, Iges_num] = atlas5_torso_parameter_urdf()

%% Init
% Coder Information
%#codegen

p_num = NaN(1,5);
Iges_num = NaN(5,6);
%% Kinematic Parameters
% Source: [1]
% 1     back_bkz: pelvis -> ltorso
p_num(1) = -0.0125;
% 2     back_bky: ltorso -> mtorso
p_num(2) = 0.162;
% 3     back_bkx: mtorso -> utorso
p_num(3) = 0.05;
% 4     neck_ry:  utorso -> head
p_num(4:5) = [0.2546, 0.6215];

%% Output Parameters: Mass
m_num = [9.509;...  % (1) pelvis
         2.270;...  % (2) ltorso
         0.799;...  % (3) mtorso
         84.409;... % (4) utorso
         1];        % (5) head: homogene Kugel m=1kg, r=20cm
  
%% Output Parameters: Center of Mass
% center of mass coordinates in body reference frames. Source: [1]
rSges_num = [...
  [0.0111, 0, 0.0271];...                   % (1) pelvis
  [-0.0112984, -3.15366e-06, 0.0746835];... % (2) ltorso
  [-0.00816266 -0.0131245 0.0305974];...    % (3) mtorso
  [-0.0622, 0.0023, 0.3157];...             % (4) utorso
  [0,0,0]];                                 % (5) head
%% Output Parameters: Inertias
% Order: Ixx, Iyy, Izz, Ixy, Ixz, Iyz

% (1) pelvis
% <inertia ixx="0.1244" ixy="0.0008" ixz="-0.0007" iyy="0.0958" iyz="-0.0005" izz="0.1167"/>
Iges_num(1,:) = [0.1244, 0.0958, 0.1167, 0.0008, -0.0007, -0.0005];
% (2) ltorso
% <inertia ixx="0.0039092" ixy="-5.04491e-08" ixz="-0.000342157" iyy="0.00341694" iyz="4.87119e-07" izz="0.00174492"/>
Iges_num(2,:) = [0.0039092, 0.00341694, 0.00174492, -5.04491e-08, -0.000342157, 4.87119e-07];
% (3) mtorso
% <inertia ixx="0.000454181" ixy="-6.10764e-05" ixz="3.94009e-05" iyy="0.000483282" iyz="5.27463e-05" izz="0.000444215"/>
Iges_num(3,:) = [0.000454181, 0.000483282, 0.000444215, -6.10764e-05, 3.94009e-05, 5.27463e-05];
% (4) utorso
% <inertia ixx="1.577" ixy="-0.032" ixz="0.102" iyy="1.602" iyz="0.047" izz="0.565"/>
Iges_num(4,:) = [1.577, 1.602, 0.565, -0.032, 0.102, 0.047];
% (5) head
% <inertia ixx="0.016" ixy="0" ixz="0" iyy="0.016" iyz="0" izz="0.016"/>
Iges_num(5,:) = [0.016, 0.016, 0.016, 0, 0, 0]; % (5) head: homogene Kugel m=1kg, r=20cm