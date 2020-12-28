% Return Parameter of the Atlas Robot (v3) (hard coded)
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

function [p_num, rSges_num, m_num, Iges_num] = atlas3_torso_parameter_urdf()

%% Init
% Coder Information
%#codegen

p_num = NaN(1,5);
Iges_num = NaN(6,6);
%% Kinematic Parameters
% Source: [1]
% 1     back_bkz: pelvis -> ltorso
p_num(1) = -0.0125;

% 2     back_bky: ltorso -> mtorso
p_num(2) = 0.16197;

% 3     back_bkx: mtorso -> utorso
p_num(3) = 0.05;

% 4     neck_ry:  utorso -> head
p_num(4:5) = [0.21672, 0.53796];

%% Output Parameters: Mass
m_num = [17.882; ... % (1) pelvis
  2.409; ... % (2) ltorso
  0.69; ... % (3) mtorso
  52.007; ... % (4) utorso
  0];
  
%% Output Parameters: Center of Mass
% center of mass coordinates in body reference frames. Source: [1]
rSges_num = [ ...
  [0.0111, 0, 0.0271]; ... (1) pelvis
  [-0.0112984, -3.15366e-06, 0.0746835]; ... % (2) ltorso
  [-0.00816266 -0.0131245 0.0305974]; ...% (3) mtorso
  [-0.0923, 0, 0.3]; ... % (4) utorso
  [0,0,0]];
%% Output Parameters: Inertias
% Order: Ixx, Iyy, Izz, Ixy, Ixz, Iyz

% (1) pelvis
% <inertia ixx="0.1244" ixy="0.0008" ixz="-0.0007" iyy="0.0958" iyz="-0.0005" izz="0.1167" />
Iges_num(1,:) = [0.1244, 0.0958, 0.1167, 0.0008, -0.0007, -0.0005];

% (2) ltorso
% <inertia ixx="0.0039092" ixy="-5.04491e-08" ixz="-0.000342157" iyy="0.00341694" iyz="4.87119e-07" izz="0.00174492" />
Iges_num(2,:) = [0.0039092, 0.00341694, 0.00174492, -5.04491e-08, -0.000342157, 4.87119e-07];

% (3) mtorso
% <inertia ixx="0.000454181" ixy="-6.10764e-05" ixz="3.94009e-05" iyy="0.000483282" iyz="5.27463e-05" izz="0.000444215" />
Iges_num(3,:) = [0.000454181, 0.000483282, 0.000444215, 6.10764e-05, 3.94009e-05, 5.27463e-05];

% (4) utorso
% <inertia ixx="1.466" ixy="0.00362" ixz="0.336" iyy="1.51" iyz="0.001" izz="1.3" />
Iges_num(4,:) = [1.466, 1.51, 1.3, 0.00362, 0.336, 0.001];

Iges_num(5,:) = zeros(1,6);