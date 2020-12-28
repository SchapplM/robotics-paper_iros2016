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
% U [1x1]
%   Potential Energy

% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-03
% (c) Institut für Regelungstechnik, Universität Hannover
 
function U = atlas5_arm_energy_potential_plin_minpar_sym_lag_varpar( ...
  q, g0_utorso, MPV, lr)
%% Init
% Coder Information
%#codegen
assert(isa(q,'double') && isreal(q) && all(size(q) == [1 7]), ...
  'Joint angles qs have to be [1x7] double');
assert(isa(g0_utorso,'double') && isreal(g0_utorso) && all(size(g0_utorso) == [1 3]), ...
  'Gravitation vector has to be [1x3] double');
assert(isa(MPV,'double') && isreal(MPV) && all(size(MPV) == [45 1]), ...
  'Minimum Parameter vector has to be [45x1] double');
assert(isa(lr,'logical') && all(size(lr) == [1 1]));    

%% Kinematik-Parameter
p_num_urdf = atlas5_arm_parameter_urdf(lr);
[a_mdh, d_mdh, ~, q_offset_mdh] = atlas5_arm_parameter_mdh(lr);

%% Regressor-Matrix
U_regressor = atlas5_arm_energy_potential_regressor_minpar_sym_lag_varpar( ...
  q, g0_utorso, p_num_urdf, a_mdh, d_mdh, q_offset_mdh, lr);

%% Gelenkmoment
U = (U_regressor*MPV)';