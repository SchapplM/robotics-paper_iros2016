% Calculate Forward Kinematics for Atlas Robot (v4)
% 
% Input:
% q [1x6]
%   Joint Angles [rad]
% lr [1x1 logical]
%   true for left, false for right
% 
% Output:
% T_c_urdf [4x4x8]
%   homogenious transformation matrices for each body frame
%   1: utorso -> utorso
%   2: utorso -> clav (depending on q1)
%   3: utorso -> scap (depending on q1, q2)
%   4: utorso -> uarm
%   5: utorso -> larm
%   6: utorso -> farm
%   7: utorso -> hand
%   8: utorso -> endpoint
% 
% Sources:
% [1] atlas_v4.urdf
% 

% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-01
% (c) Institut für Regelungstechnik, Leibniz Universität Hannover


function T_c_urdf = atlas4_arm_fkine_num(q, lr)
%% Init
%#codegen
% Coder Information
assert(isa(q,'double') && isreal(q) && all(size(q) == [1 6]), ...
  'Joint angles q have to be [1x6] double');
assert(isa(lr,'logical') && all(size(lr) == [1 1]), ...
  'Left/Right flag has to be [1x1] logical');      


p_num = atlas4_arm_parameter_urdf(lr);

%% Frame to Frame Transformations
T_urdf = NaN(4,4,7);

% T_urdf: Overview over the indices
% 1     shz: utorso -> clav
% 2     shx: clav -> scap
% 3     ely: scap -> uarm
% 4     elx: uarm -> larm
% 5     wry: larm -> farm
% 6     wrx: farm -> hand
% 7          hand -> endpoint

% 1     shz: utorso -> clav
T_urdf(:,:,1) = transl(p_num(1:3)) * trotz(q(1));

% 2     shx: clav -> scap
T_urdf(:,:,2) = transl([0, p_num(6:7)]) * trotx(q(2));

% 3     ely: scap -> uarm
T_urdf(:,:,3) = transl([0,  p_num(8:9)]) * troty(q(3));

% 4     elx: uarm -> larm
T_urdf(:,:,4) = transl([0,  p_num(10:11)]) * trotx(q(4));

% 5     wry: larm -> farm
T_urdf(:,:,5) = transl([0,  p_num(12:13)]) * troty(q(5));

% 6     wrx: farm -> hand
T_urdf(:,:,6) = transl([0,  p_num(14:15)]) * trotx(q(6));

% 7          hand -> endpoint
T_urdf(:,:,7) = eye(4);

%% Kumulierte Transformationsmatrizen
T_c_urdf = NaN(4,4,8);
T_c_urdf(:,:,1) = eye(4);

for j = 2:8
    T_c_urdf(:,:,j) = T_c_urdf(:,:,j-1) * T_urdf(:,:,j-1);
end