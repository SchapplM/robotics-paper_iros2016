% Return Parameter of the Atlas Robot (v3) (hard coded)
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
% 
% Sources:
% [1] atlas_v3.urdf

% Moritz Schappler, schappler@irt.uni-hannover.de, 2014-11
% (c) Institut für Regelungstechnik, Leibniz Universität Hannover

function [p_num, rSges_num, m_num, Iges_num] = atlas3_arm_parameter_urdf(lr)

%% Init
% Coder Information
%#codegen

assert(isa(lr,'logical') && all(size(lr) == [1 1]));      

p_num = NaN(1,15);

Iges_num = NaN(6,6);
rSges_num = NaN(6,3);
%% Kinematic Parameters
% Source: [1]

% 1     shy: utorso -> clav
if lr == true % left
  p_num(1:3) = [0.06441,  0.13866, 0.10718];
  p_num(4:5) = [0.5,  0.866025]; % 60°
else
  p_num(1:3) = [0.06441,  -0.13866, 0.10718];
  p_num(4:5) = [0.5,  -0.866025];
end

% 2     shx: clav -> scap
if lr == true % left
  p_num(6:7) = [0.14035 0.19609];
else
  p_num(6:7) = [-0.14035 0.19609];
end

% 3     ely: scap -> uarm
if lr == true % left
  p_num(8:9) = [0.187, 0.016];
else
  p_num(8:9) = [-0.187, 0.016];
end

% 4     elx: uarm -> larm
if lr == true % left
  p_num(10:11) = [0.119, 0.00921];
else
  p_num(10:11) = [-0.119, 0.00921];
end

% 5     wry: larm -> farm
if lr == true % left
   p_num(12:13) = [0.187, -0.00921];
else
  p_num(12:13) = [-0.187, -0.00921];
end

% 6     wrx: farm -> hand
if lr == true % left
  p_num(14:15) = [0.119, 0.00921];
else
  p_num(14:15) = [-0.119, 0.00921];
end

%% Output Parameters: Center of Mass
% center of mass coordinates in body reference frames. Source: [1]

% (1) clav
if lr == true % left
  rSges_num(1,:) = [0,  0.048, 0.084];
  rSges_num(3,:) = [0, -0.065, 0];% (3) uarm
  rSges_num(5,:) = [0, -0.065, 0]; % (5) farm
else
  rSges_num(1,:) = [0, -0.048, 0.084];
  rSges_num(3,:) = [0, 0.065, 0];% (3) uarm
  rSges_num(5,:) = [0, 0.065, 0]; % (5) farm
end
rSges_num(2,:) = [0, 0, 0];% (2) scap
rSges_num(4,:) = [0, 0, 0]; % (4) larm
rSges_num(6,:) = [0, 0, 0];% (6) hand


%% Output Parameters: Mass
% Source: [2]
m_num = [3.45; ... % (1) clav
     3.012; ...% (2) scap
     3.388; ...% (3) uarm
     2.509; ...% (4) larm
     3.388; ...% (5) farm
     2.509];   % (6) hand
  
%% Output Parameters: Inertias
% Order: Ixx, Iyy, Izz, Ixy, Ixz, Iyz

% (1) clav
if lr == true % left
 % <inertia ixx="0.011" ixy="0" ixz="0" iyy="0.009" iyz="-0.004" izz="0.004" />
 Iges_num(1,:) = [0.011, 0.009, 0.004, 0, 0, -0.004];
else
  % <inertia ixx="0.011" ixy="0" ixz="0" iyy="0.009" iyz="0.004" izz="0.004" />
  Iges_num(1,:) = [0.011, 0.009, 0.004, 0, 0, 0.004];
end

% (2) scap
% <inertia ixx="0.00319" ixy="0" ixz="0" iyy="0.00583" iyz="0" izz="0.00583" />
Iges_num(2,:) = [0.00319, 0.00583, 0.00583, 0, 0, 0];

% (3) uarm
% <inertia ixx="0.00656" ixy="0" ixz="0" iyy="0.00358" iyz="0" izz="0.00656" />
Iges_num(3,:) = [0.00656, 0.00358, 0.00656, 0, 0, 0];

% (4) larm
% <inertia ixx="0.00265" ixy="0" ixz="0" iyy="0.00446" iyz="0" izz="0.00446" />
Iges_num(4,:) = [0.00265, 0.00446, 0.00446, 0, 0, 0];

% (5) farm
% <inertia ixx="0.00656" ixy="0" ixz="0" iyy="0.00358" iyz="0" izz="0.00656" />
Iges_num(5,:) = [0.00656, 0.00358, 0.00656, 0, 0, 0];

% (6) hand
% <inertia ixx="0.00265" ixy="0" ixz="0" iyy="0.00446" iyz="0" izz="0.00446" />
Iges_num(6,:) = [0.00265, 0.00446, 0.00446, 0, 0, 0];
