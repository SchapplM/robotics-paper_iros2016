% Constants for the Atlas Robot
% 
% Input:
% version [1x1 uint8]
%   Atlas Model Version number
% lr
%   true for left, false for right
% 
% Output:
% AtlasStruct
%   Structure with several helpful constants
%   .JI_Torso
%     Indices to access torso joints
%   .JI_lLeg
%     Indices to access left leg joints
%   .JI_rLeg
%     Indices to access right leg joints
%   .JI_lArm
%     Indices to access left arm joints
%   .JI_rArm
%     Indices to access right arm joints
%   .JN
%     Joint Names (Cell Array)
%   .LI_Torso
%     Indices to access torso links
%   .LI_lLeg
%     Indices to access left leg links
%   .LI_rLeg
%     Indices to access right leg links
%   .LI_lArm
%     Indices to access left arm links
%   .LI_rArm
%     Indices to access right arm links
%   .LN
%     Link Names (Cell Array
%   .qmin
%     lower joint position limit
%   .qmax
%     upper joint position limit
%   .NJ
%     Number of all Joints
%   .NJA
%     Number of Joints of the Arm
%   .taumax
%     joint effor limit
%   .hydr_r
%     moment arm of the hydraulic force
%   .hydr_Apos
%   .hydr_Aneg
%   .k_motor_elec
%     Electric motor constant [Nm/A]
%   .gear_ratio_elec
%     gear ratio of electric joints
% 
% Sources:
% [1] atlas_v3.urdf
% [2] atlas_v4.urdf
% [3] atlas_v5.urdf
% [4] Atlas v4 ActuatorData.xls
%     (Material/BDI_atlas4_20150114_SupplementalRobotData)

% Moritz Schappler, schappler@irt.uni-hannover.de, 2014-12
% (c) Institut für Regelungstechnik, Universität Hannover

function AtlasStruct = atlas_const(version, lr)
% assert(isa(lr,'logical') && all(size(lr) == [1 1]));   

AtlasStruct = struct('version', version);

%% Joint and Link Indexes and Names

if version == 3 || version == 4 % Both versions nearly identical
  % Joint Indices
  AtlasStruct.JI_Torso = 1:4;
  AtlasStruct.JI_lLeg = 5:10;
  AtlasStruct.JI_rLeg = 11:16;
  AtlasStruct.JI_lArm = 17:22;
  AtlasStruct.JI_rArm = 23:28;
  
  % Link Indices
  AtlasStruct.LI_Torso = 1:5; % pelvis, ltorso, mtorso, utorso, head
  AtlasStruct.LI_lLeg = 6:11; % uglut, lglut, uleg, lleg, talus, foot
  AtlasStruct.LI_rLeg = 12:17;
  AtlasStruct.LI_lArm = 18:23; % clav, scap, uarm, larm, farm, hand
  AtlasStruct.LI_rArm = 24:29;
  AtlasStruct.NL = 29;
  
  % joint names
  JointNameCell = {'back_bkz'; 'back_bky'; 'back_bkx'; 'neck_ry'; ...
   'l_leg_hpz'; 'l_leg_hpx'; 'l_leg_hpy'; 'l_leg_kny'; 'l_leg_aky'; 'l_leg_akx'; ...
   'r_leg_hpz'; 'r_leg_hpx'; 'r_leg_hpy'; 'r_leg_kny'; 'r_leg_aky'; 'r_leg_akx'; ...
   'l_arm_shy'; 'l_arm_shx'; 'l_arm_ely'; 'l_arm_elx'; 'l_arm_wry'; 'l_arm_wrx'; ...
   'r_arm_shy'; 'r_arm_shx'; 'r_arm_ely'; 'r_arm_elx'; 'r_arm_wry'; 'r_arm_wrx'};
  AtlasStruct.JN = JointNameCell;
  if version == 4
    AtlasStruct.JN{AtlasStruct.JI_lArm(1)} = 'l_arm_shz';
    AtlasStruct.JN{AtlasStruct.JI_rArm(1)} = 'r_arm_shz';
  end
  
  % Link Names
  LinkNameCell = {'pelvis'; 'ltorso'; 'mtorso'; 'utorso'; 'head'; ...
   'l_uglut'; 'l_lglut'; 'l_uleg'; 'l_lleg'; 'l_talus'; 'l_foot'; ...
   'r_uglut'; 'r_lglut'; 'r_uleg'; 'r_lleg'; 'r_talus'; 'r_foot'; ...
   'l_clav'; 'l_scap'; 'l_uarm'; 'l_larm'; 'l_farm'; 'l_hand'; ...
   'r_clav'; 'r_scap'; 'r_uarm'; 'r_larm'; 'r_farm'; 'r_hand'};
  AtlasStruct.LN = LinkNameCell;
  
  % Number of Joints
  AtlasStruct.NJ = 28;
  AtlasStruct.NJA = 6;
end

if version == 5
  % Joint Indices
  AtlasStruct.JI_Torso = 1:4;
  AtlasStruct.JI_lLeg = 5:10;
  AtlasStruct.JI_rLeg = 11:16;
  AtlasStruct.JI_lArm = 17:23;
  AtlasStruct.JI_rArm = 24:30;
  
  % Link Indices
  AtlasStruct.LI_Torso = 1:5; % pelvis, ltorso, mtorso, utorso, head
  AtlasStruct.LI_lLeg = 6:11; % uglut, lglut, uleg, lleg, talus, foot
  AtlasStruct.LI_rLeg = 12:17;
  AtlasStruct.LI_lArm = 18:24; % clav, scap, uarm, larm, ufarm, lfarm, hand
  AtlasStruct.LI_rArm = 25:31;
  AtlasStruct.NL = 31;
  
  % joint names
  JointNameCell = {'back_bkz'; 'back_bky'; 'back_bkx'; 'neck_ry'; ...
   'l_leg_hpz'; 'l_leg_hpx'; 'l_leg_hpy'; 'l_leg_kny'; 'l_leg_aky'; 'l_leg_akx'; ...
   'r_leg_hpz'; 'r_leg_hpx'; 'r_leg_hpy'; 'r_leg_kny'; 'r_leg_aky'; 'r_leg_akx'; ...
   'l_arm_shz'; 'l_arm_shx'; 'l_arm_ely'; 'l_arm_elx'; 'l_arm_wry'; 'l_arm_wrx'; 'l_arm_wry2'; ...
   'r_arm_shz'; 'r_arm_shx'; 'r_arm_ely'; 'r_arm_elx'; 'r_arm_wry'; 'r_arm_wrx'; 'r_arm_wry2'};
  AtlasStruct.JN = JointNameCell;
 
  % Link Names
  LinkNameCell = {'pelvis'; 'ltorso'; 'mtorso'; 'utorso'; 'head'; ...
   'l_uglut'; 'l_lglut'; 'l_uleg'; 'l_lleg'; 'l_talus'; 'l_foot'; ...
   'r_uglut'; 'r_lglut'; 'r_uleg'; 'r_lleg'; 'r_talus'; 'r_foot'; ...
   'l_clav'; 'l_scap'; 'l_uarm'; 'l_larm'; 'l_ufarm'; 'l_lfarm'; 'l_hand'; ...
   'r_clav'; 'r_scap'; 'r_uarm'; 'r_larm'; 'r_ufarm'; 'r_lfarm'; 'r_hand'};
  
  AtlasStruct.LN = LinkNameCell;
	
  AtlasStruct.tau_max = NaN(28,1); % not set yet
  AtlasStruct.q_min = NaN(28,1); % not set yet
  AtlasStruct.q_max = NaN(28,1); % not set yet
  
  % Number of Joints
  AtlasStruct.NJ = 30;
  AtlasStruct.NJA = 7;

end

% Transmission Ratio [m] and Areas [m²] of the hydraulic joints [Source: 4]
if version == 4 || version == 5
  % Insert Data for Atlas v4
  AtlasStruct.hydr_r = [zeros(16,1);... % torso, legs
    0.006998;ones(5,1)*0.005123;... % left arm
    0.006998;ones(5,1)*0.005123]; % right arm
  AtlasStruct.hydr_Apos = [zeros(16,1);... % torso, legs
    514.6*1e-6; 1007.4*1e-6; 641.3*1e-6; 1134.1*1e-6; 641.3*1e-6; 641.3*1e-6;... % left arm
    641.3*1e-6; 1134.1*1e-6; 514.6*1e-6; 1007.4*1e-6; 514.6*1e-6; 514.6*1e-6]; % right arm
  AtlasStruct.hydr_Aneg = [zeros(16,1);... % torso, legs
    641.3*1e-6; 1134.1*1e-6; 514.6*1e-6; 1007.4*1e-6; 514.6*1e-6; 514.6*1e-6;... % left arm
    514.6*1e-6; 1007.4*1e-6; 641.3*1e-6; 1134.1*1e-6; 641.3*1e-6; 641.3*1e-6]; % right arm
  
  % change indexes and fill with NaN for electric forearms of arm upgrade
  if version == 5
    AtlasStruct.hydr_r = [AtlasStruct.hydr_r(1:16); ... % torso, legs
      AtlasStruct.hydr_r(17:20); NaN(3,1); % left arm v4 with electric forearm
      AtlasStruct.hydr_r(23:26); NaN(3,1)]; % right arm v4 with electric forearm
    AtlasStruct.hydr_Apos = [AtlasStruct.hydr_Apos(1:16); ... % torso, legs
      AtlasStruct.hydr_Apos(17:20); NaN(3,1); % left arm v4 with electric forearm
      AtlasStruct.hydr_Apos(23:26); NaN(3,1)]; % right arm v4 with electric forearm
    AtlasStruct.hydr_Aneg = [AtlasStruct.hydr_Aneg(1:16); ... % torso, legs
      AtlasStruct.hydr_Aneg(17:20); NaN(3,1); % left arm v4 with electric forearm
      AtlasStruct.hydr_Aneg(23:26); NaN(3,1)]; % right arm v4 with electric forearm
  end
end

AtlasStruct.NJL = 6; % Number of Links in Leg
%% Limits
if version == 3
  % effort limit in urdf tbd
  
  % lower [position] limit in urdf
  q_min = [ ...
    NaN(4,1); ...% torso tbd
    NaN(6,1); NaN(6,1); ... % legs tbd
    -1.5708; -1.5708; 0; 0; 0; -1.1781; ... % l_arm_shz, l_arm_shx, l_arm_ely, l_arm_elx, l_arm_wry, l_arm_wrx
    -1.5708; -1.5708; 0; -2.35619; 0; -1.1781];% r_arm_shz, r_arm_shx, r_arm_ely, r_arm_elx, r_arm_wry, r_arm_wrx
    
    
  % upper [position] limit in urdf
  q_max = [ ...
    NaN(4,1); ...% torso tbd
    NaN(6,1); NaN(6,1); ... % legs tbd
    0.785398; 1.5708; 3.14159; 2.35619; 3.14159; 1.1781; ... % l_arm_shz, l_arm_shx, l_arm_ely, l_arm_elx, l_arm_wry, l_arm_wrx
    0.785398; 1.5708; 3.14159; 0; 3.14159; 1.1781];% r_arm_shz, r_arm_shx, r_arm_ely, r_arm_elx, r_arm_wry, r_arm_wrx
  
  % leave unset
  qD_max = NaN(28,1);
  
  % leave unset
  tau_max = NaN(28,1);
  
  AtlasStruct.q_min = q_min;
  AtlasStruct.q_max = q_max;
  AtlasStruct.tau_max = tau_max;
  AtlasStruct.qD_max = qD_max;
end

if version == 4
  AtlasStruct.JN{AtlasStruct.JI_lArm(1)} = 'l_arm_shz';
  AtlasStruct.JN{AtlasStruct.JI_rArm(1)} = 'r_arm_shz';

  % effort limit in urdf
  tau_max = [ ...
    62; 245; 163; 5; ... % 'back_bkz'; 'back_bky'; 'back_bkx'; 'neck_ry'
    NaN(6,1); NaN(6,1); ... % legs tbd
    198; 159; 106; 159; 106; 56; ...% l_arm_shz, l_arm_shx, l_arm_ely, l_arm_elx, l_arm_wry
    198; 159; 106; 159; 106; 56];
  
  % velocity limit in urdf set to 12 rad/s for the arms.
  % set the limits lower to assure safety
  qD_max = [ ...
    NaN(4,1); ... % 'back_bkz'; 'back_bky'; 'back_bkx'; 'neck_ry'
    NaN(6,1); NaN(6,1); ... % legs tbd
    4; 4; 3; 3; 2; 2; ...% l_arm_shz, l_arm_shx, l_arm_ely, l_arm_elx, l_arm_wry
    4; 4; 3; 3; 2; 2];
  
  % lower [position] limit in urdf
  q_min = [ ...
    NaN(4,1); ...% torso tbd
    NaN(6,1); NaN(6,1); ... % legs tbd
    -1.5708; -1.5708; 0; 0; 0; -1.1781; ... % l_arm_shz, l_arm_shx, l_arm_ely, l_arm_elx, l_arm_wry, l_arm_wrx
    -0.785398; -1.5708; 0; -2.35619; 0; -1.1781];% r_arm_shz, r_arm_shx, r_arm_ely, r_arm_elx, r_arm_wry, r_arm_wrx
    
    
  % upper [position] limit in urdf
  q_max = [ ...
    NaN(4,1); ...% torso tbd
    NaN(6,1); NaN(6,1); ... % legs tbd
    0.785398; 1.5708; 3.14159; 2.35619; 3.14159; 1.1781; ... % l_arm_shz, l_arm_shx, l_arm_ely, l_arm_elx, l_arm_wry, l_arm_wrx
    1.5708; 1.5708; 3.14159; 0; 3.14159; 1.1781];% r_arm_shz, r_arm_shx, r_arm_ely, r_arm_elx, r_arm_wry, r_arm_wrx
  
  AtlasStruct.tau_max = tau_max;
  AtlasStruct.qD_max = qD_max;
  AtlasStruct.tau_min = -tau_max;
  AtlasStruct.qD_min = -qD_max;
  AtlasStruct.q_min = q_min;
  AtlasStruct.q_max = q_max;
end

% Sign Vector: Show which signs of joint positions have to be changed for
% symmetric pose:
if version == 3 || version == 4
  AtlasStruct.q_Arm_mirror = [-1, -1, 1, -1, 1, -1];
elseif version == 5
  AtlasStruct.q_Arm_mirror = [-1, -1, 1, -1, 1, -1, 1];
end
AtlasStruct.q_Leg_mirror = [-1, -1, 1, 1, 1, -1];

if version == 5
    
  % lower [position] limit in urdf
  q_min = [ ...
    -0.663225; -0.219388; -0.523599; -0.602139; ...% 'back_bkz'; 'back_bky'; 'back_bkx'; 'neck_ry'
    -0.174358; -0.523599; -1.61234; 0; -1; -0.8; ... % l_leg_hpz, l_leg_hpx, l_leg_hpy, l_leg_kny, l_leg_aky, l_leg_akx
    -0.786794; -0.523599; -1.61234; 0; -1; -0.8; ... % r_leg_hpz, r_leg_hpx, r_leg_hpy, r_leg_kny, r_leg_aky, r_leg_akx
    -1.5708; -1.5708; 0; 0; -3.011; -1.7628; -2.9671; ... % l_arm_shz, l_arm_shx, l_arm_ely, l_arm_elx, l_arm_wry, l_arm_wrx, l_arm_wry2
    -0.785398; -1.5708; 0; -2.35619; -3.011; -1.7628; -2.9671];% r_arm_shz, r_arm_shx, r_arm_ely, r_arm_elx, r_arm_wry, r_arm_wrx, r_arm_wry2
    
    
  % upper [position] limit in urdf
  q_max = [ ...
    0.663225; 0.538783; 0.523599; 1.14319; ...% 'back_bkz'; 'back_bky'; 'back_bkx'; 'neck_ry'
    0.786794; 0.523599; 0.65764; 2.35637; 0.7; 0.8; ... % l_leg_hpz, l_leg_hpx, l_leg_hpy, l_leg_kny, l_leg_aky, l_leg_akx
    0.174358; 0.523599; 0.65764; 2.35637; 0.7; 0.8; ... % r_leg_hpz, r_leg_hpx, r_leg_hpy, r_leg_kny, r_leg_aky, r_leg_akx
    0.785398; 1.5708; 3.14159; 2.35619; 3.011; 1.7628; 2.9671; ... % l_arm_shz, l_arm_shx, l_arm_ely, l_arm_elx, l_arm_wry, l_arm_wrx, l_arm_wry2
    1.5708; 1.5708; 3.14159; 0; 3.011; 1.7628; 2.9671];% r_arm_shz, r_arm_shx, r_arm_ely, r_arm_elx, r_arm_wry, r_arm_wrx, r_arm_wry2
  	
  % velocity limit in urdf set to 12 rad/s for the arms.
  % set the limits lower to assure safety
  % set limits of electrical motors very low so they do not overheat
  qD_max = [ ...
    12; 9; 12; 6.28; ... % 'back_bkz'; 'back_bky'; 'back_bkx'; 'neck_ry'
    12*ones(6,1); ... % l_leg_hpz, l_leg_hpx, l_leg_hpy, l_leg_kny, l_leg_aky, l_leg_akx
    12*ones(6,1); ... % r_leg_hpz, r_leg_hpx, r_leg_hpy, r_leg_kny, r_leg_aky, r_leg_akx
    4; 4; 3; 3; 2.5; 2.5; 2.5; ...% l_arm_shz, l_arm_shx, l_arm_ely, l_arm_elx, l_arm_wry, l_arm_wrx, l_arm_wry2
    4; 4; 3; 3; 2.5; 2.5; 2.5];
  
  % effort limit in urdf (aus v4)
  tau_max = [ ...
    106; 445; 300; 25; ... % 'back_bkz'; 'back_bky'; 'back_bkx'; 'neck_ry'
    275; 530; 840; 890; 740; 360; ... % l_leg_hpz, l_leg_hpx, l_leg_hpy, l_leg_kny, l_leg_aky, l_leg_akx
    275; 530; 840; 890; 740; 360;  ... % r_leg_hpz, r_leg_hpx, r_leg_hpy, r_leg_kny, r_leg_aky, r_leg_akx
    87; 99; 63; 112; 25; 25; 25; ...% l_arm_shz, l_arm_shx, l_arm_ely, l_arm_elx, l_arm_wry, l_arm_wrx, l_arm_wry2
    87; 99; 63; 112; 25; 25; 25];

  
  AtlasStruct.tau_max = tau_max;
  AtlasStruct.qD_max = qD_max;
  AtlasStruct.tau_min = -tau_max;
  AtlasStruct.qD_min = -qD_max;
  
  AtlasStruct.q_min = q_min;
  AtlasStruct.q_max = q_max;
  
  AtlasStruct.k_motor_elec = 0.12679; %[Nm/A]
  AtlasStruct.gear_ratio_elec = [100; 100; 99; 100; 100; 99];
  % for all joints, for easier indexing
  AtlasStruct.k_motor_elec_all = 0.12679*ones(30,1); %[Nm/A]
  AtlasStruct.gear_ratio_elec_all = [NaN(20,1); 100; 100; 99; NaN(4,1); 100; 100; 99];
end

%% Fuß-Geometrie
% Bestimmt mit data/atlas_footsole_generation.m
% Eckpunkte des äußeren Vierecks aller Punkte der Fußsohle. Im URDF-KS foot
if version == 5
  AtlasStruct.footsole_left = [...
     -0.0877   -0.0626   -0.0765; ...
     -0.0877    0.0661   -0.0765; ...
      0.1728   -0.0626   -0.0765; ...
      0.1728    0.0661   -0.0765];
  AtlasStruct.footsole_right = [...
     -0.0877   -0.0661   -0.0765; ...
     -0.0877    0.0626   -0.0765; ...
      0.1728   -0.0661   -0.0765; ...
      0.1728    0.0626   -0.0765];
else
  AtlasStruct.footsole_left = NaN(4,3);
  AtlasStruct.footsole_right = NaN(4,3);
end

%% Daten für eine Körperseite
% Export Joint and Link Indices for the requested arm
if nargin > 1
  if lr == true % left
    AtlasStruct.LI_Leg = AtlasStruct.LI_lLeg;
    AtlasStruct.LI_Arm = AtlasStruct.LI_lArm;
    AtlasStruct.JI_Leg = AtlasStruct.JI_lLeg;
    AtlasStruct.JI_Arm = AtlasStruct.JI_lArm;
    AtlasStruct.footsole = AtlasStruct.footsole_left;
  else % right
    AtlasStruct.LI_Leg = AtlasStruct.LI_rLeg;
    AtlasStruct.LI_Arm = AtlasStruct.LI_rArm;
    AtlasStruct.JI_Leg = AtlasStruct.JI_rLeg;
    AtlasStruct.JI_Arm = AtlasStruct.JI_rArm;
    AtlasStruct.footsole = AtlasStruct.footsole_right;
  end
end
