% Plot Atlas arm CAD Models
% 
% Input:
% q [1x28]
%   joint angles [rad]
%   Order: See data/atlas_const.m
% version
%   Atlas Model Version
% fignr
%   Figure Handle to plot in
% T_pelvis
%   Transformation Matrix to Pelvis
% subplotInf 
%   information for subplot [row x col x number], 
% Source:
% [1] Atlas Message Format from DRCSim
% https://bitbucket.org/osrf/drcsim/src/c028afcf24728fb5bfbe57a9f3648c23ab711b39/atlas_msgs/msg/AtlasState.msg?at=default

% Moritz Schappler, schappler@irt.uni-hannover.de, 2014-11
% (c) Institut für Regelungstechnik, Universität Hannover

function hdl = atlas_plot_robot(q, version, fignr, T_pelvis,subplotInf)

AS = atlas_const(version);
hdl = NaN(AS.NJ+1,1);
%% Set Default Input
if nargin == 0
  q = zeros(1,AS.NJ);
end

if nargin < 2
  version = 3;
end
if nargin < 3
  figure(1);set(1, 'Name', '5:Frame', 'NumberTitle', 'off');
  hold on; grid on;axis equal;view(3);set(1, 'Renderer','OpenGL')
else
  figure(fignr);
end

if nargin < 4
  T_pelvis = eye(4);
end

if nargin < 5
  % no subplot
else  
  subplot(subplotInf);
end

%% Calculate Kinematics
T_torso_c_urdf = atlas_torso_fkine(q(AS.JI_Torso), version);

T_legL_c_urdf = atlas_leg_fkine(q(AS.JI_lLeg), true, version);
T_legR_c_urdf = atlas_leg_fkine(q(AS.JI_rLeg), false, version);

T_armL_c_urdf = atlas_arm_fkine(q(AS.JI_lArm), true, version);
T_armR_c_urdf = atlas_arm_fkine(q(AS.JI_rArm), false, version);

%% Assemble Kinematics for full Humanoid
% Consider given T_pelvis
for i = 1:size(T_torso_c_urdf,3)
  T_torso_c_urdf(:,:,i) = T_pelvis * T_torso_c_urdf(:,:,i);
end

% Leg Transformations start at pelvis
for i = 1:size(T_legL_c_urdf,3)
  T_legL_c_urdf(:,:,i) = T_pelvis * T_legL_c_urdf(:,:,i);
  T_legR_c_urdf(:,:,i) = T_pelvis * T_legR_c_urdf(:,:,i);
end

% Arm Transformations start at utorso
% pelvis-> utorso
for i = 1:size(T_armL_c_urdf,3)
  T_armL_c_urdf(:,:,i) = T_torso_c_urdf(:,:,4) * T_armL_c_urdf(:,:,i);
  T_armR_c_urdf(:,:,i) = T_torso_c_urdf(:,:,4) * T_armR_c_urdf(:,:,i);
end
  
%% Plot
hdl(AS.LI_Torso)=atlas_plot_torso_stl(T_torso_c_urdf, version);

hdl(AS.LI_lLeg)=atlas_plot_leg_stl(T_legL_c_urdf, true, version);
hdl(AS.LI_rLeg)=atlas_plot_leg_stl(T_legR_c_urdf, false, version);

hdl(AS.LI_lArm)=atlas_plot_arm_stl(T_armL_c_urdf, true, version);
hdl(AS.LI_rArm)=atlas_plot_arm_stl(T_armR_c_urdf, false, version);
