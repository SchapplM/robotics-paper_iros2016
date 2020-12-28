% Calculate geometric jacobian for Atlas robot (v4)
% 
% Input:
% q
%   Joint Angles [rad]
% lr
%   true for left, false for right
% 
% Output:
% JG [6x7] double
%   Geometrische Jacobi-Matrix

% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-03
% (c) Institut für Regelungstechnik, Universität Hannover
 
function JG = atlas5_arm_jacobig_num(q, lr) %#codegen
%% Init
% Coder Information
%#codegen
assert(isa(q,'double') && isreal(q) && all(size(q) == [1 7]), ...
  'q has to be [1x7] double');
assert(isa(lr,'logical') && all(size(lr) == [1 1]), ...
  'lr has to be [1x1] logical');   

JG = NaN(6, 7);

%% Forward Kinematics
T_c_urdf = atlas5_arm_fkine(q, lr);

% T_c_urdf(:,:,3) is the body frame of the first body (upper shoulder)
% q1 turns around the z-axis of this frame

%% Geometric Calculations

for i = 1:7
  % Get rotation axis of the joint. See fkine
  if i == 1
    ax = T_c_urdf(1:3,3, i+1); % z-Axis
  elseif i == 2
    if lr
      ax = -T_c_urdf(1:3,1, i+1); % neg. x-Axis
    else
      ax = T_c_urdf(1:3,1, i+1); % x-Axis
    end
  elseif i ==  3
    if lr
      ax = -T_c_urdf(1:3,2, i+1); % neg. y-Axis
    else
      ax = T_c_urdf(1:3,2, i+1); % y-Axis
    end
  elseif i ==  4
    if lr
      ax = -T_c_urdf(1:3,1, i+1); % neg. x-Axis
    else
      ax = T_c_urdf(1:3,1, i+1); % x-Axis
    end
  elseif i ==  5
    if lr
      ax = -T_c_urdf(1:3,2, i+1); % neg. y-Axis
    else
      ax = T_c_urdf(1:3,2, i+1); % y-Axis
    end
  elseif i ==  6
    ax = T_c_urdf(1:3,1, i+1); % x-Axis
  else % i ==  7
    if lr
      ax = -T_c_urdf(1:3,2, i+1); % neg. y-Axis
    else
      ax = T_c_urdf(1:3,2, i+1); % y-Axis
    end
  end
  
  % vectors from joints to endeffector
  r = T_c_urdf(1:3,4, 8) - T_c_urdf(1:3,4, i+1);
  
  % lever arm from joint to endeffector on joint axis
  ar = cross(ax, r);
  
  % Compose geometric Jacobian
  JG(:,i) = [ar; ax];
end

