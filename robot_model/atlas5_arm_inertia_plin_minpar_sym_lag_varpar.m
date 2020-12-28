% Calculate joint inertia matrix for Atlas robot Arm (v5)
% Use matlab function from symbolic Maple calculations
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
% Mq [7x7]
%   Joint Mass-Matrix

% Source:
% [1] Robotik I Skript

% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-02
% (c) Institut für Regelungstechnik, Universität Hannover


function MM = atlas5_arm_inertia_plin_minpar_sym_lag_varpar(q, MPV, lr)

%% Init
%#codegen
% Coder Information
assert(isa(q,'double') && isreal(q) && all(size(q) == [1 7]), ...
  'atlas5_arm_inertia_plin_minpar_sym_lag_varpar: Joint angles q have to be [1x7] double');
assert(isa(MPV,'double') && isreal(MPV) && all(size(MPV) == [45 1]), ...
  'atlas5_arm_inertia_plin_minpar_sym_lag_varpar: Minimum Parameter vector has to be [45x1] double');
assert(isa(lr,'logical') && all(size(lr) == [1 1]), ...
  'atlas5_arm_inertia_plin_minpar_sym_lag_varpar: Left/Right flag has to be [1x1] logical'); 

%% Kinematik-Parameter
[a_mdh, d_mdh, ~, q_offset_mdh] = atlas5_arm_parameter_mdh(lr);

%% Regressor-Matrix
MM_regressor = atlas5_arm_inertia_plin_minpar_regressor_sym_lag_varpar(q, a_mdh, d_mdh, q_offset_mdh, lr);

%% Massenmatrix
MM = zeros(7,7);
% TODO: Symmetrie der Massenmatrix berücksichtigen
for i = 1:7
  for j = 1:7
    for k = 1:45
      MM(i,j) = MM(i,j) + MM_regressor(7*(i-1)+j,k)*MPV(k);
    end
  end
end