% Plot Atlas arm with all frames
% 
% Input:
% q
%   joint angles
% LI
%   Indices of the body frames to plot (see atlas_const)
% version [1x1 uint8]
%   Atlas Model Version number
% T_pelvis [4x4]
%   Transformation Matrix to Pelvis

% Moritz Schappler, schappler@irt.uni-hannover.de, 2014-11
% (c) Institut für Regelungstechnik, Universität Hannover

function atlas_plot_frames(q, LI, version, T_pelvis)

AS = atlas_const(version);

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
for i = 1:7
  T_legL_c_urdf(:,:,i) = T_pelvis * T_legL_c_urdf(:,:,i);
  T_legR_c_urdf(:,:,i) = T_pelvis * T_legR_c_urdf(:,:,i);
end

% Arm Transformations start at utorso
% pelvis-> utorso
for i = 1:size(T_armL_c_urdf,3)
  T_armL_c_urdf(:,:,i) = T_torso_c_urdf(:,:,4) * T_armL_c_urdf(:,:,i);
  T_armR_c_urdf(:,:,i) = T_torso_c_urdf(:,:,4) * T_armR_c_urdf(:,:,i);
end

%% Plot Frames

for i = LI
  % left Arm
  name = strrep(AS.LN{i}, '_', '\\_');
  if any(i==AS.LI_lArm)
    SegmentNr = i-AS.LI_lArm(1)+1;
    trplot(T_armL_c_urdf(:,:,SegmentNr+1), 'frame', name, 'arrow', 'rgb', 'length', 0.3)
  end
  % right Arm
  if any(i==AS.LI_rArm)
    SegmentNr = i-AS.LI_rArm(1)+1;
    trplot(T_armR_c_urdf(:,:,SegmentNr+1), 'frame', name, 'arrow', 'rgb', 'length', 0.3)
  end
  % left Leg
  if any(i==AS.LI_lLeg)
    SegmentNr = i-AS.LI_lLeg(1)+1;
    trplot(T_legL_c_urdf(:,:,SegmentNr+1), 'frame', name, 'arrow', 'rgb', 'length', 0.3)
  end
  % right Leg
  if any(i==AS.LI_rLeg)
    SegmentNr = i-AS.LI_rLeg(1)+1;
    trplot(T_legR_c_urdf(:,:,SegmentNr+1), 'frame', name, 'arrow', 'rgb', 'length', 0.3)
  end
  if any(i==AS.LI_Torso)
    SegmentNr = i-AS.LI_Torso(1)+1;
    trplot(T_torso_c_urdf(:,:,SegmentNr), 'frame', name, 'arrow',   'rgb', 'length', 0.3)
  end
  % text_opts', {'interpreter', 'none'}
end
