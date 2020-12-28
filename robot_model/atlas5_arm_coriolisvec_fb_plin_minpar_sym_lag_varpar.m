% Calculate joint torque vector of coriolis and centripetal forces for
% Atlas Robot (v5)
% from Maple symbolic code generation in MDH frames
% Berechnung mit Regressorform (Parameterlineare Form)
% Minimal-Parametersatz (neu gruppierte Parameter)
% 
% Input:
% q [1x7]
%   Joint Angles [rad]
% qD [1x7]
%   Joint Velocities [rad/s]
% MPV [45x1]
%   minimum parameter vector (regrouped dynamic parameters)
% lr [1x1 logical]
%   true for left, false for right
% 
% Output:
% cq [1x7]
%   joint torque vector of coriolis and centripetal forces [Nm]
% 

% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-02
% (c) Institut für Regelungstechnik, Leibniz Universität Hannover

function tauc = atlas5_arm_coriolisvec_fb_plin_minpar_sym_lag_varpar(q, qD, MPV, lr)

%% Init
%#codegen
% Coder Information
assert(isa(q,'double') && isreal(q) && all(size(q) == [1 7]), ...
  'atlas5_arm_coriolisvec_fb_plin_minpar_sym_lag_varpar: Joint angles q have to be [1x7] double');
assert(isa(qD,'double') && isreal(qD) && all(size(qD) == [1 7]), ...
  'atlas5_arm_coriolisvec_fb_plin_minpar_sym_lag_varpar: Joint Velocities qD have to be [1x7] double');
assert(isa(MPV,'double') && isreal(MPV) && all(size(MPV) == [45 1]), ...
  'atlas5_arm_coriolisvec_fb_plin_minpar_sym_lag_varpar: Minimum Parameter vector has to be [45x1] double');
assert(isa(lr,'logical') && all(size(lr) == [1 1]), ...
  'atlas5_arm_coriolisvec_fb_plin_minpar_sym_lag_varpar: Left/Right flag has to be [1x1] logical'); 

%% Kinematik-Parameter
[a_mdh, d_mdh, ~, q_offset_mdh] = atlas5_arm_parameter_mdh(lr);

%% Regressor-Matrix
tauc_regressor = atlas5_arm_coriolisvec_fb_regressor_minpar_sym_lag_varpar(q, qD, a_mdh, d_mdh, q_offset_mdh, lr);

%% Gelenkmoment
tauc = (tauc_regressor*MPV)';
