% Return URDF Parameter of the Atlas Robot (v4) leg (hard coded)
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
% [1] atlas_v5_simple_shapes_with_head.urdf

% Moritz Schappler, schappler@irt.uni-hannover.de, 2014-11
% (c) Institut für Regelungstechnik, Leibniz Universität Hannover

function [p_num, rSges_num, m_num, Iges_num] = atlas4_leg_parameter_urdf(lr)

%% Init
% Coder Information
%#codegen

assert(isa(lr,'logical') && all(size(lr) == [1 1]));      

p_num = NaN(1,7);
Iges_num = NaN(6,6);
%% Kinematic Parameters
% Source: [1]

% 1     hpz: pelvis -> uglut
if lr == true % left
  p_num(1) = 0.089;
else
  p_num(1) = -0.089;
end
% 2     hpx: uglut -> lglut
% no transformation
% 3     hpy: lglut -> uleg
if lr == true % left
  p_num(2:4) = [0.05, 0.0225, -0.066];
else
  p_num(2:4) = [0.05, -0.0225, -0.066];
end
% 4     kny: uleg -> lleg
p_num(5:6) = [-0.05,-0.374];
% 5     aky: lleg -> talus
p_num(7) = -0.422;
% 6     akx: talus -> foot
% no transformation
  
%% Output Parameters: Mass
m_num = [1.959; ... % (1) uglut
         0.898; ... % (2) lglut
         8.204; ... % (3) uleg
         4.515; ... % (4) lleg
         0.125; ... % (5) talus
         2.410];    % (6) foot

%% Output Parameters: Center of Mass
% center of mass coordinates in body reference frames. Source: [1]
rSges_num = [...
  [0.00529262, -0.00344732, 0.00313046];... % (1) uglut
  [0.0133341, 0.0170484, -0.0312052];...    % (2) lglut
  [0, 0, -0.21];...                         % (3) uleg
  [0.001, 0, -0.187];...                    % (4) lleg
  [0, 0, 0];...                             % (5) talus 
  [0.027, 0, -0.067]];                      % (6) foot
if lr == false
  rSges_num(2,2) = - rSges_num(2,2);
  rSges_num(1,2) = - rSges_num(1,2);
end
%% Output Parameters: Inertias
% Order: Ixx, Iyy, Izz, Ixy, Ixz, Iyz

% (1) uglut
% left
% <inertia ixx="0.00074276" ixy="-3.79607e-08" ixz="-2.79549e-05" iyy="0.000688179" iyz="-3.2735e-08" izz="0.00041242"/>
% right
% <inertia ixx="0.00074276" ixy="3.79607e-08" ixz="-2.79549e-05" iyy="0.000688179" iyz="3.2735e-08" izz="0.00041242"/>
if lr == true
  Iges_num(1,:) = [0.00074276, 0.000688179, 0.00041242, -3.79607e-08, -2.79549e-05, -3.2735e-08];
else
  Iges_num(1,:) = [0.00074276, 0.000688179, 0.00041242,  3.79607e-08, -2.79549e-05,  3.2735e-08];
end
% (2) lglut
% left
% <inertia ixx="0.000691326" ixy="-2.24344e-05" ixz="2.50508e-06" iyy="0.00126856" iyz="0.000137862" izz="0.00106487"/>
% right
% <inertia ixx="0.000691326" ixy="2.24344e-05" ixz="2.50508e-06" iyy="0.00126856" iyz="-0.000137862" izz="0.00106487"/>
if lr == true
  Iges_num(2,:) = [0.000691326, 0.00126856, 0.00106487, -2.24344e-05, 2.50508e-06,  0.000137862];
else
  Iges_num(2,:) = [0.000691326, 0.00126856, 0.00106487,  2.24344e-05, 2.50508e-06, -0.000137862];
end
% (3) uleg
% <inertia ixx="0.09" ixy="0" ixz="0" iyy="0.09" iyz="0" izz="0.02"/>
Iges_num(3,:) = [0.09, 0.09, 0.02, 0, 0, 0];
% (4) lleg
% <inertia ixx="0.077" ixy="0" ixz="-0.003" iyy="0.076" iyz="0" izz="0.01"/>
Iges_num(4,:) = [0.077, 0.076, 0.01, 0, -0.003, 0];
% (5) talus
%  <inertia ixx="1.01674e-05" ixy="0" ixz="0" iyy="8.42775e-06" iyz="0" izz="1.30101e-05"/>
Iges_num(5,:) = [1.01674e-05, 8.42775e-06, 1.30101e-05, 0, 0, 0];
% (6) foot
% <inertia ixx="0.002" ixy="0" ixz="0" iyy="0.007" iyz="0" izz="0.008"/>
Iges_num(6,:) = [0.002, 0.007, 0.008, 0, 0, 0];