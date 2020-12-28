% Calculate Forward Kinematics for Atlas Robot Torso (all versions)
% 
% Input:
% q [1x4]
%   Joint Angles [rad]
% version [1x1 uint8]
%   Version of the robot (e.g. 3 or 4)
% 
% Output:
% T_c_urdf [4x4x5]
%   homogenious transformation matrices for each body frame
%   1: pelvis -> pelvis
%   2: pelvis -> ltorso
%   3: pelvis -> mtorso
%   4: pelvis -> utorso
%   5: pelvis -> head
% 
% Sources:
% [1] atlas_v3.urdf

% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-01
% (c) Institut für Regelungstechnik, Leibniz Universität Hannover


function T_c_urdf = atlas_torso_fkine(q, version)
%% Init
%#codegen
% Coder Information
assert(isa(q,'double') && isreal(q) && all(size(q) == [1 4]), ...
  'Joint angles q have to be [1x4] double');
assert(isa(version,'uint8') && all(size(version) == [1 1]), ...
  'version number has to be [1x1] uint8'); 

%% Parameters
p_num = atlas_torso_parameter_urdf(version);

%% Frame to Frame Transformations
T_urdf = NaN(4,4,4);

% T_urdf: Overview over the indices
% 1     back_bkz: pelvis -> ltorso
% 2     back_bky: ltorso -> mtorso
% 3     back_bkx: mtorso -> utorso
% 4     neck_ry:  utorso -> head


% 1     back_bkz: pelvis -> ltorso
T_urdf(:,:,1) = transl([p_num(1), 0, 0]) * trotz(q(1));

% 2     back_bky: ltorso -> mtorso
T_urdf(:,:,2) = transl([0, 0, p_num(2)]) * troty(q(2));

% 3     back_bkx: mtorso -> utorso
T_urdf(:,:,3) = transl([0, 0, p_num(3)]) * trotx(q(3));

% 4     neck_ry:  utorso -> head
T_urdf(:,:,4) = transl([p_num(4), 0, p_num(5)]) * troty(q(4));


%% Kumulierte Transformationsmatrizen
T_c_urdf = NaN(4,4,5);
T_c_urdf(:,:,1) = eye(4);

for j = 2:5
    T_c_urdf(:,:,j) = T_c_urdf(:,:,j-1) * T_urdf(:,:,j-1);
end

end
