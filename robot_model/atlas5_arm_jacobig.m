% Calculate geometric jacobian for Atlas robot (v5)
% 
% Input:
% q [1x7]
%   Joint Angles [rad]
% lr [1x1]
%   true for left, false for right
% 
% 
% Source:
% [1] Robotik I Skript

% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-01
% (c) Institut für Regelungstechnik, Universität Hannover
 
function JG = atlas5_arm_jacobig(q, lr)
%% Init

%#codegen
% Coder Information
assert(isa(q,'double') && isreal(q) && all(size(q) == [1 7]), ...
  'atlas5_arm_jacobig: Joint angles q have to be [1x7] double');
assert(isa(lr,'logical') && all(size(lr) == [1 1]), ...
  'atlas5_arm_jacobig: Left/Right flag has to be [1x1] logical');      

%% Calculation

JG = atlas5_arm_jacobig_num(q, lr);