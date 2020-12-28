% Calculate Forward Kinematics for Atlas Robot (all versions)
% 
% Input:
% q [1x(N)]
%   Joint Angles [rad]
% lr [1x1 logical]
%   true for left, false for right
% version [1x1 uint8]
%   Version of the robot (e.g. 3, 4, 5)
% 
% Output:
% T_c_urdf [4x4x(N+2)]
%   homogenious transformation matrices for each body frame
%   1: utorso -> utorso
%   2: utorso -> clav
%   3: utorso -> scap
%   4: utorso -> uarm
%   5: utorso -> larm
%   ...


% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-01
% (c) Institut für Regelungstechnik, Leibniz Universität Hannover


function T_c_urdf = atlas_arm_fkine(q, lr, version)
%% Init
%#codegen
% Coder Information
assert(isa(q,'double') && isreal(q) && all(size(q) <= [1 7]), ...
  'Joint angles q have to be [1xN] double');
assert(isa(lr,'logical') && all(size(lr) == [1 1]), ...
  'Left/Right flag has to be [1x1] logical');      
assert(isa(version,'uint8') && all(size(version) == [1 1]), ...
  'version number has to be [1x1] uint8'); 

%% Calculation
if version == 3
  T_c_urdf = atlas3_arm_fkine_num(q, lr);
elseif version == 4
  T_c_urdf = atlas4_arm_fkine_num(q, lr);
elseif version == 5
  T_c_urdf = atlas5_arm_fkine_num(q, lr);
else
  error('Version %d not implemented yet', version);
end
