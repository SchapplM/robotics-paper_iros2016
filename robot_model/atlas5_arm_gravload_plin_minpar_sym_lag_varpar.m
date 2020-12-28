% Calculate Gravitation Load for Atlas Robot Arms (v5)
% Use Code from Maple symbolic Code Generation
% Berechnung mit Regressorform (Parameterlineare Form)
% 
% Input:
% q [1x7]
%   Joint Angles [rad]
% g0_utorso [1x3]
%   gravitation vector in utorso frame [m/s^2]
% MPV [45x1]
%   minimum parameter vector (regrouped dynamic parameters)
% lr
%   true for left, false for right
% 
% Output:
% taug [1x7]
%   joint torques required to compensate gravitation load

% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-02
% (c) Institut für Regelungstechnik, Universität Hannover
 
function taug = atlas5_arm_gravload_plin_minpar_sym_lag_varpar( ...
  q, g0_utorso, MPV, lr)
%% Init
% Coder Information
%#codegen
assert(isa(q,'double') && isreal(q) && all(size(q) == [1 7]), ...
  'atlas5_arm_gravload_plin_minpar_sym_lag_varpar:Joint angles q have to be [1x7] double');
assert(isa(g0_utorso,'double') && isreal(g0_utorso) && all(size(g0_utorso) == [1 3]), ...
  'atlas5_arm_gravload_plin_minpar_sym_lag_varpar:Gravitation vector has to be [1x3] double');
assert(isa(MPV,'double') && isreal(MPV) && all(size(MPV) == [45 1]), ...
  'atlas5_arm_gravload_plin_minpar_sym_lag_varpar:Minimum Parameter vector has to be [45x1] double');
assert(isa(lr,'logical') && all(size(lr) == [1 1]), ...
  'atlas5_arm_gravload_plin_minpar_sym_lag_varpar:Left/Right Flag has to be [1x1] logical');      

%% Kinematik-Parameter
p_num_urdf = atlas5_arm_parameter_urdf(lr);
[~, ~, ~, q_offset_mdh] = atlas5_arm_parameter_mdh(lr);

%% Regressor-Matrix
taug_regressor = atlas5_arm_gravload_regressor_minpar_sym_lag_varpar( ...
  q, g0_utorso, p_num_urdf, q_offset_mdh);

%% Gravitationslast
taug = (taug_regressor*MPV)';
