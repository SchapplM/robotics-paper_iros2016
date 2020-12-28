% Calculate Forward Kinematics for Atlas Robot (v5)
% 
% Input:
% q [1x7]
%   Joint Angles [rad]
% lr [1x1 logical]
%   true for left, false for right
% 
% Output:
% T_c_urdf [4x4x9]
%   homogenious transformation matrices for each body frame
%   1: utorso -> utorso
%   2: utorso -> clav
%   3: utorso -> scap
%   4: utorso -> uarm
%   5: utorso -> larm
%   6: utorso -> ufarm
%   7: utorso -> lfarm
%   8: utorso -> hand
%   9: utorso -> endpoint
% 
% Sources:
% [1] atlas_v5.urdf
% 

% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-01
% (c) Institut für Regelungstechnik, Leibniz Universität Hannover


function T_c_urdf = atlas5_arm_fkine(q, lr)
%% Init
%#codegen
% Coder Information
assert(isa(q,'double') && isreal(q) && all(size(q) == [1 7]), ...
  'Joint angles q have to be [1x7] double');
assert(isa(lr,'logical') && all(size(lr) == [1 1]), ...
  'Left/Right flag has to be [1x1] logical');      

%% Calculation
T_c_urdf = atlas5_arm_fkine_num(q, lr);