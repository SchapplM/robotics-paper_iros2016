% Return Parameter of the Atlas Robot (v3) (hard coded)
% 
% Input:
% lr
%   true for left, false for right
% 
% Output:
% p_num [6x1]
%   Numeric Values of Kinematic Parameters
% rSges_num [6x3]
%   Center of Gravity of all independent bodies (leg only) in urdf frames
% m_num [6x1]
%   masses of all independent bodies (leg only)
% Iges_num  [6x6]
%   inertia of all independent bodies (leg only)
%     rows: bodies
%     columns: inertia tensor entries. Order: xx, yy, zz, xy, xz, yz
% 
% Sources:
% [1] atlas_v3.urdf

% Moritz Schappler, schappler@irt.uni-hannover.de, 2014-11
% (c) Institut für Regelungstechnik, Leibniz Universität Hannover

function [p_num, rSges_num, m_num, Iges_num] = atlas3_leg_parameter_urdf(lr)

%% Init
% Coder Information
%#codegen

assert(isa(lr,'logical') && all(size(lr) == [1 1]));      

p_num = NaN(1,6);
Iges_num = NaN(6,6);
rSges_num = NaN(6,3);
%% Kinematic Parameters
% Source: [1]

% 1     hpz: pelvis -> uglut
if lr == true % left
  p_num(1) = 0.089;
else
  p_num(1) = -0.089;
end

% 2     hpx: uglut -> lglut

% 3     hpy: lglut -> uleg
p_num(2:3) = [0.05, -0.05];

% 4     kny: uleg -> lleg
p_num(4:5) = [-0.05,-0.374];

% 5     aky: lleg -> talus
p_num(6) = -0.422;

% 6     akx: talus -> foot
  
%% Output Parameters: Mass
m_num = [0.648; ... % (1) uglut
  0.866; ... % (2) lglut
  9.209; ...% (3) uleg
  5.479; ... % (4) lleg
  0.125; ...% (5) talus
  2.05]; % (6) foot

%% Output Parameters: Center of Mass
% center of mass coordinates in body reference frames. Source: [1]
if lr == true % left
  rSges_num(1,:) = [0.00529262, -0.00344732, 0.00313046];  % (1) uglut
  rSges_num(2,:) = [0.0133341, 0.0170484, -0.0312052];    % (2) lglut
else
  rSges_num(1,:) = [0.00529262, 0.00344732, 0.00313046];   % (1) uglut
  rSges_num(2,:) = [0.0133341, -0.0170484, -0.0312052];    % (2) lglut
end
rSges_num(3,:) = [0, 0, -0.21];                          % (3) uleg
rSges_num(4,:) = [0.001, 0, -0.187];                     % (4) lleg
rSges_num(5,:) = [0, 0, 0];                              % (5) talus 
rSges_num(6,:) = [0.027, 0, -0.067];                     % (6) foot

%% Output Parameters: Inertias
% Order: Ixx, Iyy, Izz, Ixy, Ixz, Iyz

% (1) uglut
% <inertia ixx="0.00074276" ixy="-3.79607e-08" ixz="-2.79549e-05" iyy="0.000688179" iyz="-3.2735e-08" izz="0.00041242" />
Iges_num(1,:) = [0.00074276, 0.000688179, 0.00041242, -3.79607e-08, -2.79549e-05, -3.2735e-08];
% (2) lglut
% <inertia ixx="0.000691326" ixy="2.24344e-05" ixz="2.50508e-06" iyy="0.00126856" iyz="-0.000137862" izz="0.00106487" />
Iges_num(2,:) = [0.000691326, 0.00126856, 0.00106487, 2.24344e-05, 2.50508e-06, -0.000137862];

% (3) uleg
% <inertia ixx="0.09" ixy="0" ixz="0" iyy="0.09" iyz="0" izz="0.02" />
Iges_num(3,:) = [0.09, 0.09, 0.02, 0, 0, 0];

% (4) lleg
% <inertia ixx="0.077" ixy="0" ixz="-0.003" iyy="0.076" iyz="0" izz="0.01" />
Iges_num(4,:) = [0.077, 0.076, 0.01, 0, -0.003, 0];

% (5) talus
%  <inertia ixx="1.01674e-05" ixy="0" ixz="0" iyy="8.42775e-06" iyz="0" izz="1.30101e-05" />
Iges_num(5,:) = [1.01674e-05, 8.42775e-06, 1.30101e-05, 0, 0, 0];

% (6) foot
% <inertia ixx="0.002" ixy="0" ixz="0" iyy="0.007" iyz="0" izz="0.008" />
Iges_num(6,:) = [0.002, 0.007, 0.008, 0, 0, 0];