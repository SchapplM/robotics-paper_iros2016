% Calculate Forward Kinematics for Atlas Robot Leg (v3)
% 
% Input:
% q [1x6]
%   Joint Angles [rad]
% lr [1x1 logical]
%   true for left, false for right
% 
% Output:
% T_c_urdf [4x4x7]
%   homogenious transformation matrices for each body frame
%   1: pelvis -> pelvis
%   2: pelvis -> glut
%   3: pelvis -> lglut
%   4: pelvis -> uleg
%   5: pelvis -> lleg
%   6: pelvis -> talus
%   7: pelvis -> foot
% 
% Sources:
% [1] atlas_v3.urdf

% Moritz Schappler, schappler@irt.uni-hannover.de, 2014-11
% (c) Institut für Regelungstechnik, Leibniz Universität Hannover


function T_c_urdf = atlas3_leg_fkine_num(q, lr)
%% Init
%#codegen
% Coder Information
assert(isa(q,'double') && isreal(q) && all(size(q) == [1 6]), ...
  'Joint angles q have to be [1x6] double');
assert(isa(lr,'logical') && all(size(lr) == [1 1]), ...
  'Left/Right flag has to be [1x1] logical');      


p_num = atlas3_leg_parameter_urdf(lr);

%% Frame to Frame Transformations
T_urdf = NaN(4,4,7);

% T_urdf: Overview over the indices
% 1     hpz: pelvis -> uglut
% 2     hpx: uglut -> lglut
% 3     hpy: lglut -> uleg
% 4     kny: uleg -> lleg
% 5     aky: lleg -> talus
% 6     akx: talus -> foot


% 1     hpz: pelvis -> uglut
T_urdf(:,:,1) = transl([0, p_num(1), 0]) * trotz(q(1));

% 2     hpx: uglut -> lglut
T_urdf(:,:,2) = trotx(q(2));
% 
% 3     hpy: lglut -> uleg
T_urdf(:,:,3) = transl([p_num(2), 0, p_num(3)]) * troty(q(3));
% 
% 4     kny: uleg -> lleg
T_urdf(:,:,4) = transl([p_num(4), 0, p_num(5)]) * troty(q(4));
% 
% 5     aky: lleg -> talus
T_urdf(:,:,5) = transl([0,0,p_num(6)]) * troty(q(5));
% 
% 6     akx: talus -> foot
T_urdf(:,:,6) = trotx(q(6));

%% Kumulierte Transformationsmatrizen
T_c_urdf = NaN(4,4,7);
T_c_urdf(:,:,1) = eye(4);

for j = 2:7
    T_c_urdf(:,:,j) = T_c_urdf(:,:,j-1) * T_urdf(:,:,j-1);
end