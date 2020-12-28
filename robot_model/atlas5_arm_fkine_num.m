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
%   2: utorso -> clav (depending on q1)
%   3: utorso -> scap (depending on q1, q2)
%   4: utorso -> uarm
%   5: utorso -> larm
%   6: utorso -> ufarm
%   7: utorso -> lfarm
%   8: utorso -> hand
%   9: utorso -> endpoint
% 
% Sources:
% [1] atlas_v5_simple_shapes_with_head.urdf

% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-01
% (c) Institut für Regelungstechnik, Leibniz Universität Hannover


function T_c_urdf = atlas5_arm_fkine_num(q, lr)
%% Init
%#codegen
% Coder Information
assert(isa(q,'double') && isreal(q) && all(size(q) == [1 7]), ...
  'Joint angles q have to be [1x7] double');
assert(isa(lr,'logical') && all(size(lr) == [1 1]), ...
  'Left/Right flag has to be [1x1] logical');      

p_num = atlas5_arm_parameter_urdf(lr);

%% Frame to Frame Transformations
T_urdf = NaN(4,4,8);

% T_urdf: Overview over the indices
% 1     shz: utorso -> clav
% 2     shx: clav -> scap
% 3     ely: scap -> uarm
% 4     elx: uarm -> larm
% 5     wry: larm -> ufarm
% 6     wrx: ufarm -> lfarm
% 7     wry2:lfarm -> hand
% 8          hand -> endpoint

% 1     shz: utorso -> clav
if lr
  T_urdf(:,:,1) = transl(p_num(1:3)) * trotz(3.14159265359) * trotz(q(1));
else
  T_urdf(:,:,1) = transl(p_num(1:3)) * trotz(q(1));
end
% 2     shx: clav -> scap
if lr
  T_urdf(:,:,2) = transl([0, p_num(6:7)]) * trotx(-q(2));
else
  T_urdf(:,:,2) = transl([0, p_num(6:7)]) * trotx(q(2));
end
% 3     ely: scap -> uarm
if lr
  T_urdf(:,:,3) = transl([0,  p_num(8:9)]) * troty(-q(3));
else
  T_urdf(:,:,3) = transl([0,  p_num(8:9)]) * troty(q(3));
end
% 4     elx: uarm -> larm
if lr
  T_urdf(:,:,4) = transl([0,  p_num(10:11)]) * trotx(-q(4));
else
  T_urdf(:,:,4) = transl([0,  p_num(10:11)]) * trotx(q(4));
end
% 5     wry: larm -> ufarm
if lr
  T_urdf(:,:,5) = transl([0,  p_num(12:13)]) * troty(3.14159265359) * troty(-q(5));
else
  T_urdf(:,:,5) = transl([0,  p_num(12:13)]) * troty(q(5));
end
% 6     wrx: ufarm -> lfarm
T_urdf(:,:,6) = trotx(q(6));
% 7     wry2: lfarm -> hand
if lr
  T_urdf(:,:,7) = troty(3.14159265359) * troty(-q(7));
else
  T_urdf(:,:,7) = troty(q(7));
end
% 8          hand -> endpoint
if lr
  T_urdf(:,:,8) = trotz(pi);
else
  T_urdf(:,:,8) = eye(4);
end

%% Kumulierte Transformationsmatrizen
T_c_urdf = NaN(4,4,9);
T_c_urdf(:,:,1) = eye(4);
for j = 2:9
    T_c_urdf(:,:,j) = T_c_urdf(:,:,j-1) * T_urdf(:,:,j-1);
end