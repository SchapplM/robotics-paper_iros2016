% Calculate Forward Kinematics for Atlas Robot Leg (all versions)
% 
% Input:
% q [1x6]
%   Joint Angles [rad]
% lr [1x1 logical]
%   true for left, false for right
% version [1x1 uint8]
%   Version of the robot (e.g. 3 or 4)
% 
% Output:
% T_c_urdf [4x4x7]
%   homogenious transformation matrices for each body frame
%   1: pelvis -> pelvis
%   2: pelvis -> uglut
%   3: pelvis -> lglut
%   4: pelvis -> uleg
%   5: pelvis -> lleg
%   6: pelvis -> talus
%   7: pelvis -> foot
% 
% Sources:
% [1] atlas_v4.urdf

% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-01
% (c) Institut für Regelungstechnik, Leibniz Universität Hannover


function T_c_urdf = atlas_leg_fkine(q, lr, version)
%% Init
%#codegen
% Coder Information
assert(isa(q,'double') && isreal(q) && all(size(q) == [1 6]), ...
  'Joint angles q have to be [1x6] double');
assert(isa(lr,'logical') && all(size(lr) == [1 1]), ...
  'Left/Right flag has to be [1x1] logical');      
assert(isa(version,'uint8') && all(size(version) == [1 1]), ...
  'version number has to be [1x1] uint8'); 

%% Calculation
if version == 3
  T_c_urdf = atlas3_leg_fkine_num(q, lr);
elseif version == 4 || version == 5
  T_c_urdf = atlas4_leg_fkine_num(q, lr);
else
  error('Version %d not implemented yet', version);
end