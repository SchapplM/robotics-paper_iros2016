% Return Parameter of the Atlas Robot (v5) (hard coded from urdf file)
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
% [1] data/atlas_v5_simple_shapes_with_head.urdf

% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-03
% (c) Institut für Regelungstechnik, Leibniz Universität Hannover

function [p_num, rSges_num, m_num, Iges_num] = atlas5_arm_parameter_urdf(lr)

%% Init
% Coder Information
%#codegen

assert(isa(lr,'logical') && all(size(lr) == [1 1]));      

p_num = NaN(1,13);

Iges_num = NaN(7,6);
rSges_num = NaN(7,3);
%% Kinematic Parameters
% Source: [1]

% 1     shz: utorso -> clav
if lr == true % left
  p_num(1:3) = [0.1406, 0.2256, 0.4776];
  p_num(4:5) = [0,  1]; % z-axis
else
  p_num(1:3) = [0.1406, -0.2256, 0.4776];
  p_num(4:5) = [0, 1];
end
% 2     shx: clav -> scap
p_num(6:7) = [-0.11 -0.245];
% 3     ely: scap -> uarm
p_num(8:9) = [-0.187, -0.016];
% 4     elx: uarm -> larm
p_num(10:11) = [-0.119, 0.0092];
% 5     wry: larm -> ufarm
p_num(12:13) = [-0.29955, -0.00921];
% 6     wrx: ufarm -> lfarm
% no transformation
% 7     wry2: lfarm -> hand
% no transformation

%% Output Parameters: Center of Mass
% center of mass coordinates in body reference frames. Source: [1]

rSges_num(1,:) = [0 0 -.084];             % (1) clav
rSges_num(2,:) = [0, 0, 0];               % (2) scap
rSges_num(3,:) = [0,  0.065, 0];          % (3) uarm
rSges_num(4,:) = [0, 0, 0];               % (4) larm
% (5) ufarm
if lr == true % left
  rSges_num(5,:) = [.00015  .08296 -.00037];
else
  rSges_num(5,:) = [.00015  .08296  .00037];
end
% (6) lfarm
if lr == true % left
  rSges_num(6,:) = [.00017 -.02515 -.00163];
else
  rSges_num(6,:) = [.00017 -.02515  .00163];
end
% (7) hand
if lr == true % left
  rSges_num(7,:) = [-.00016 -.08159 .00002];
else
  rSges_num(7,:) = [ .00016 -.08159 .00002];
end

%% Output Parameters: Mass
% Source: [1]
m_num = [4.466;...  % (1) clav
         3.899;...  % (2) scap
         4.386;...  % (3) uarm
         3.248;...  % (4) larm
         2.4798;... % (5) ufarm
         0.648;...  % (6) lfarm
         0.5839];   % (7) hand
%% Output Parameters: Inertias
% Order: Ixx, Iyy, Izz, Ixy, Ixz, Iyz

% (1) clav
% <inertia ixx="0.011" ixy="0" ixz="0" iyy="0.009" iyz="0.004" izz="0.004"/>
Iges_num(1,:) = [0.011, 0.009, 0.004, 0, 0, 0.004];
% (2) scap
% <inertia ixx="0.00319" ixy="0" ixz="0" iyy="0.00583" iyz="0" izz="0.00583"/>
Iges_num(2,:) = [0.00319, 0.00583, 0.00583, 0, 0, 0];
% (3) uarm
% <inertia ixx="0.00656" ixy="0" ixz="0" iyy="0.00358" iyz="0" izz="0.00656"/>
Iges_num(3,:) = [0.00656, 0.00358, 0.00656, 0, 0, 0];
% (4) larm
% <inertia ixx="0.00265" ixy="0" ixz="0" iyy="0.00446" iyz="0" izz="0.00446"/>
Iges_num(4,:) = [0.00265, 0.00446, 0.00446, 0, 0, 0];
% (5) ufarm
% <inertia ixx="0.012731" ixy="0" ixz="0" iyy="0.002857" iyz="0" izz="0.011948"/>
Iges_num(5,:) = [0.012731, 0.002857, 0.011948, 0, 0, 0];
% (6) lfarm
% <inertia ixx="0.000764" ixy="0" ixz="0" iyy="0.000429" iyz="0" izz="0.000825"/>
Iges_num(6,:) = [0.000764, 0.000429, 0.000825, 0, 0, 0];
% (7) hand
% <inertia ixx="0.000388" ixy="0" ixz="0" iyy="0.000477" iyz="0" izz="0.000379"/>
Iges_num(7,:) = [0.000388, 0.000477, 0.000379, 0, 0, 0];