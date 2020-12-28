% Berechnung der Regressormatrix (parameterlineare Form) für die
% Gravitationslast (atlas v5)
% 
% Parametersatz 2 (Trägheit (zweites Moment), Masse*Schwerpunkt (erstes
% Moment), Masse)
% 
% Input:
% q [1x7]
%   Joint Angles [rad]
% g0_utorso [1x3]
%   gravitation vector in utorso frame [m/s^2]
% lr
%   true for left, false for right
% 
% Output:
% taug_regressor [6x60]
%   Regressormatrix zur Multiplikation mit dem (vollen) Parametervektor

% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-03
% (c) Institut für Regelungstechnik, Universität Hannover
 
function taug_regressor = atlas5_arm_gravload_regressor_minpar_sym_lag_varpar( ...
  q, g0_utorso, p_num_urdf, q_offset_mdh)
%% Init
% Coder Information
%#codegen
assert(isa(q,'double') && isreal(q) && all(size(q) == [1 7]), ...
  'Joint angles q have to be [1x7] double');
assert(isa(g0_utorso,'double') && isreal(g0_utorso) && all(size(g0_utorso) == [1 3]), ...
  'Gravitation vector has to be [1x3] double');
assert(isa(p_num_urdf,'double') && isreal(p_num_urdf) && all(size(p_num_urdf) == [1 13]), ...
  'p_num_urdf has to be [1x13] double'); 
assert(isa(q_offset_mdh,'double') && isreal(q_offset_mdh) && all(size(q_offset_mdh) == [7 1]), ...
  'q_offset_mdh has to be [7x1] double'); 

qs1 = q(1);
qs2 = q(2);
qs3 = q(3);
qs4 = q(4);
qs5 = q(5);
qs6 = q(6);
qs7 = q(7);

%% Get MDH Parameters
p4 = p_num_urdf(4);
p5 = p_num_urdf(5);

qoffset2 = q_offset_mdh(2);

% Gravitation Vector in base frame (utorso)
g1 = g0_utorso(1);
g2 = g0_utorso(2);
g3 = g0_utorso(3);

%% Maple Generated Calculation 
% from codeexport/atlas5_arm_gravload_regressor_minpar_mapleexport.m

t261 = sin(qs1);
t235 = g2 * p5 - g3 * p4;
t250 = cos(qs1);
t232 = -t250 * g1 - t261 * t235;
t239 = qs2 + qoffset2;
t236 = sin(t239);
t260 = t236 * t232;
t233 = -t261 * g1 + t250 * t235;
t234 = -g2 * p4 - g3 * p5;
t237 = cos(t239);
t231 = t237 * t233 - t236 * t234;
t249 = cos(qs3);
t244 = sin(qs3);
t256 = t244 * t232;
t227 = t249 * t231 + t256;
t230 = t236 * t233 + t237 * t234;
t243 = sin(qs4);
t248 = cos(qs4);
t220 = t243 * t227 + t248 * t230;
t242 = sin(qs5);
t259 = t242 * t220;
t251 = t249 * t232;
t226 = -t244 * t231 + t251;
t258 = t243 * t226;
t257 = t244 * t230;
t221 = t248 * t227 - t243 * t230;
t247 = cos(qs5);
t212 = -t242 * t221 + t247 * t226;
t246 = cos(qs6);
t255 = t246 * t212;
t254 = t247 * t220;
t253 = t248 * t226;
t252 = t249 * t230;
t245 = cos(qs7);
t241 = sin(qs6);
t240 = sin(qs7);
t229 = -t244 * t233 + t237 * t251;
t228 = -t249 * t233 - t237 * t256;
t225 = t248 * t229 - t243 * t260;
t224 = t243 * t229 + t248 * t260;
t223 = -t243 * t231 - t248 * t252;
t222 = t248 * t231 - t243 * t252;
t219 = t247 * t225 + t242 * t228;
t218 = -t242 * t225 + t247 * t228;
t217 = t247 * t223 + t242 * t257;
t216 = -t242 * t223 + t247 * t257;
t215 = -t242 * t227 + t247 * t253;
t214 = -t247 * t227 - t242 * t253;
t213 = t247 * t221 + t242 * t226;
t211 = t246 * t215 - t241 * t258;
t210 = t246 * t219 - t241 * t224;
t209 = -t241 * t221 - t246 * t254;
t208 = t246 * t217 - t241 * t222;
t207 = t246 * t213 - t241 * t220;
t206 = -t241 * t213 - t246 * t220;
taug_regressor = [0 t232 -t233 0 0 0 0 0 t237 * t232 -t260 0 0 0 0 0 t229 t228 0 0 0 0 0 t225 -t224 0 0 0 0 0 t219 t218 0 0 0 0 0 t210 -t241 * t219 - t246 * t224 0 0 0 0 0 t245 * t210 + t240 * t218 -t240 * t210 + t245 * t218; 0 0 0 0 0 0 0 0 -t230 -t231 0 0 0 0 0 -t252 t257 0 0 0 0 0 t223 -t222 0 0 0 0 0 t217 t216 0 0 0 0 0 t208 -t241 * t217 - t246 * t222 0 0 0 0 0 t245 * t208 + t240 * t216 -t240 * t208 + t245 * t216; 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 t226 -t227 0 0 0 0 0 t253 -t258 0 0 0 0 0 t215 t214 0 0 0 0 0 t211 -t241 * t215 - t246 * t258 0 0 0 0 0 t245 * t211 + t240 * t214 -t240 * t211 + t245 * t214; 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -t220 -t221 0 0 0 0 0 -t254 t259 0 0 0 0 0 t209 -t246 * t221 + t241 * t254 0 0 0 0 0 t245 * t209 + t240 * t259 -t240 * t209 + t245 * t259; 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 t212 -t213 0 0 0 0 0 t255 -t241 * t212 0 0 0 0 0 -t240 * t213 + t245 * t255 -t245 * t213 - t240 * t255; 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 t206 -t207 0 0 0 0 0 t245 * t206 -t240 * t206; 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -t240 * t207 + t245 * t212 -t245 * t207 - t240 * t212;];