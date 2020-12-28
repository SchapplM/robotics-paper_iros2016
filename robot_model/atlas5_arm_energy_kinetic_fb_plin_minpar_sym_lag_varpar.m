% Calculate Potential Energy for Atlas robot Arm (v5)
% Use matlab function from symbolic calculations
% Berechnung mit Regressorform (Parameterlineare Form)
% Minimal-Parametersatz (neu gruppierte Parameter)
% 
% Input:
% q
%   Joint Angles [rad]
% MPV [45x1]
%   minimum parameter vector (regrouped dynamic parameters)
% lr
%   true for left, false for right
% 
% Output:
% T [1x1]
%   Kinetic Energy

% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-03
% (c) Institut für Regelungstechnik, Universität Hannover
 
function T = atlas5_arm_energy_kinetic_fb_plin_minpar_sym_lag_varpar( ...
  q, qD, MPV, lr)
%% Init
% Coder Information
%#codegen
assert(isa(q,'double') && isreal(q) && all(size(q) == [1 7]), ...
  'Joint angles q have to be [1x7] double');
assert(isa(qD,'double') && isreal(qD) && all(size(qD) == [1 7]));
assert(isa(MPV,'double') && isreal(MPV) && all(size(MPV) == [45 1]), ...
  'Minimum Parameter vector has to be [45x1] double');
assert(isa(lr,'logical') && all(size(lr) == [1 1]));    

%% Kinematik-Parameter
[a_mdh, d_mdh, ~, q_offset_mdh] = atlas5_arm_parameter_mdh(lr);

%% Regressor-Matrix
T_regressor = atlas5_arm_e_kinetic_fb_reg_minpar_sym_lag_varpar( ...
q, qD, a_mdh, d_mdh, q_offset_mdh);

%% Energie
T = (T_regressor*MPV)';