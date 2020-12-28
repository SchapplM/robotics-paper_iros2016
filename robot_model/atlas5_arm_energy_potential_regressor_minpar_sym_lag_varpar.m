% Calculate Potential Energy for Atlas robot Arm (v5)
% Use matlab function from symbolic calculations
% Berechnung mit Regressorform (Parameterlineare Form)
% Minimal-Parametersatz (neu gruppierte Parameter)
% 
% Input:
% q
%   Joint Angles [rad]
% lr
%   true for left, false for right
% 
% Output:
% U_regressor [1x45]
%   Regressorform of Potential Energy (Gravitation)

% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-02
% (c) Institut für Regelungstechnik, Universität Hannover
 
function U_regressor = atlas5_arm_energy_potential_regressor_minpar_sym_lag_varpar( ...
  q, g0_utorso, p_num_urdf, a_mdh, d_mdh, q_offset_mdh, lr)
%% Init
% Coder Information
%#codegen
assert(isa(q,'double') && isreal(q) && all(size(q) == [1 7]));
assert(isa(g0_utorso,'double') && isreal(g0_utorso) && all(size(g0_utorso) == [1 3]), ...
  'Gravitation vector has to be [1x3] double');
assert(isa(p_num_urdf,'double') && isreal(p_num_urdf) && all(size(p_num_urdf) == [1 13]), ...
  'p_num_urdf has to be [1x13] double'); 
assert(isa(a_mdh,'double') && isreal(a_mdh) && all(size(a_mdh) == [7 1]), ...
  'a_mdh has to be [7x1] double'); 
assert(isa(d_mdh,'double') && isreal(d_mdh) && all(size(d_mdh) == [7 1]), ...
  'd_mdh has to be [7x1] double'); 
assert(isa(q_offset_mdh,'double') && isreal(q_offset_mdh) && all(size(q_offset_mdh) == [7 1]), ...
  'q_offset_mdh has to be [7x1] double'); 
assert(isa(lr,'logical') && all(size(lr) == [1 1]));    

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

d3 = d_mdh(3);
d5 = d_mdh(5);

a2 = a_mdh(2);
a3 = a_mdh(3);
a4 = a_mdh(4);
a5 = a_mdh(5);
a6 = a_mdh(6);

qoffset2 = q_offset_mdh(2);

% Gravitation Vector in utorso frame
g1 = g0_utorso(1);
g2 = g0_utorso(2);
g3 = g0_utorso(3);
%% Maple Generated Calculation 
% from codeexport/atlas5_arm_energy_potential_regressor_minpar_mapleexport.m
constant = 0;

t192 = (a5 ^ 2);
t205 = 3 * t192;
t204 = 3 * constant;
t203 = 4 * constant;
t172 = g2 * p5 - g3 * p4;
t183 = sin(qs1);
t189 = cos(qs1);
t169 = -t183 * g1 + t189 * t172;
t171 = -g2 * p4 - g3 * p5;
t177 = qs2 + qoffset2;
t173 = sin(t177);
t174 = cos(t177);
t164 = t174 * t169 - t173 * t171;
t168 = -t189 * g1 - t183 * t172;
t182 = sin(qs3);
t188 = cos(qs3);
t161 = t188 * t164 + t182 * t168;
t191 = d3 ^ 2;
t194 = (a3 ^ 2);
t202 = (t191 - t194);
t201 = (t191 + t194);
t200 = constant * a3;
t199 = constant * a4;
t198 = constant * a5;
t197 = constant * d3;
t196 = constant * d5;
t163 = t173 * t169 + t174 * t171;
t181 = sin(qs4);
t187 = cos(qs4);
t157 = t187 * t161 - t181 * t163;
t160 = -t182 * t164 + t188 * t168;
t180 = sin(qs5);
t186 = cos(qs5);
t154 = -t180 * t157 + t186 * t160;
t156 = t181 * t161 + t187 * t163;
t193 = a4 ^ 2;
t190 = d5 ^ 2;
t185 = cos(qs6);
t184 = cos(qs7);
t179 = sin(qs6);
t178 = sin(qs7);
t176 = 2 * t197;
t175 = 2 * t196;
t155 = t186 * t157 + t180 * t160;
t153 = t185 * t155 - t179 * t156;
U_regressor = [(6 * a2 ^ 2 + 5 * t194 + 1) * constant (6 * constant * a2) + t169 t168 + constant t176 + constant * t202 + (t193 + t202) * t203 (5 * d3 + 1) * t200 0 0 constant + t176 + constant * t201 + (t193 + t201) * t203 t164 + (5 * t200) -t163 - constant - (5 * t197) (-4 * t193 + t205) * constant -t199 0 0 (1 + 4 * t193 + t205) * constant (4 * t199) + t161 t160 + constant t175 + (t190 - t192) * t204 (3 * d5 + 1) * t198 0 0 constant + t175 + (t190 + t192) * t204 t157 + (3 * t198) -t156 - constant - (3 * t196) 0 0 0 0 constant t155 t154 + constant 0 0 0 0 constant t153 -t179 * t155 - t185 * t156 - constant -constant 0 0 0 0 t184 * t153 + t178 * t154 -t178 * t153 + t184 * t154;];