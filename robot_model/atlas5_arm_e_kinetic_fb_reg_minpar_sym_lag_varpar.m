% Calculate Kinetic Energy for Atlas Robot Arm (v5)
% Use matlab function from symbolic calculations
% Berechnung mit Regressorform (Parameterlineare Form)
% Minimal-Parametersatz (neu gruppierte Parameter)
% 
% Input:
% q [1x7]
%   Joint Angles [rad]
% lr
%   true for left, false for right
% 
% Output:
% T [1x45]
%   Geometry Vector of Kinetic Energy

% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-02
% (c) Institut für Regelungstechnik, Universität Hannover
 
function T_regressor = atlas5_arm_e_kinetic_fb_reg_minpar_sym_lag_varpar( ...
  q, qD, a_mdh, d_mdh, q_offset_mdh)
%% Init
% Coder Information
%#codegen
assert(isa(q,'double') && isreal(q) && all(size(q) == [1 7]));
assert(isa(qD,'double') && isreal(qD) && all(size(qD) == [1 7]));
assert(isa(a_mdh,'double') && isreal(a_mdh) && all(size(a_mdh) == [7 1]), ...
  'a_mdh has to be [7x1] double'); 
assert(isa(d_mdh,'double') && isreal(d_mdh) && all(size(d_mdh) == [7 1]), ...
  'd_mdh has to be [7x1] double'); 
assert(isa(q_offset_mdh,'double') && isreal(q_offset_mdh) && all(size(q_offset_mdh) == [7 1]), ...
  'q_offset_mdh has to be [7x1] double'); 

qs1 = q(1);
qs2 = q(2);
qs3 = q(3);
qs4 = q(4);
qs5 = q(5);
qs6 = q(6);
qs7 = q(7);

qsD1 = qD(1);
qsD2 = qD(2);
qsD3 = qD(3);
qsD4 = qD(4);
qsD5 = qD(5);
qsD6 = qD(6);
qsD7 = qD(7);
%% Get MDH Parameters

d3 = d_mdh(3);
d5 = d_mdh(5);

a2 = a_mdh(2);
a3 = a_mdh(3);
a4 = a_mdh(4);
a5 = a_mdh(5);
a6 = a_mdh(6);

qoffset2 = q_offset_mdh(2);


%% Maple Generated Calculation 
% from codeexport/atlas5_arm_energy_kinetic_regressor_minpar_mapleexport.m

t105 = sin(qs3);
t106 = sin(qs2);
t112 = cos(qs2);
t114 = qsD1 ^ 2;
t137 = t112 * t114;
t128 = t106 * t137;
t111 = cos(qs3);
t87 = t111 * qsD2;
t129 = qsD1 * t87;
t88 = qs2 + qoffset2;
t86 = cos(t88);
t152 = t105 * t128 - t86 * t129;
t104 = sin(qs4);
t110 = cos(qs4);
t85 = sin(t88);
t69 = (a3 * t86 + d3 * t85 + a2) * qsD1;
t62 = d3 * t87 + t105 * t69;
t146 = qsD1 * t85;
t74 = t105 * t146 + t87;
t68 = -qsD2 * a3 - t74 * a4;
t118 = -t104 * t62 - t110 * t68;
t100 = t112 ^ 2;
t142 = t100 * t114;
t126 = t142 / 0.2e1;
t113 = qsD2 ^ 2;
t150 = t113 / 0.2e1;
t123 = t85 * t129;
t76 = t105 * t123;
t93 = t105 ^ 2;
t99 = t111 ^ 2;
t63 = t93 * t126 + t99 * t150 + t76;
t94 = t106 ^ 2;
t144 = t94 * t114;
t83 = t86 * qsD1;
t72 = t144 / 0.2e1 + qsD3 * (t83 + qsD3 / 0.2e1);
t148 = t63 + t72;
t124 = 0.2e1 * t148;
t138 = t105 * qsD2;
t120 = -d3 * t138 + t111 * t69;
t130 = qsD1 * t138;
t46 = qsD3 * t120 + (a3 * t85 - d3 * t86) * t130 + (a3 * t113 + (a3 * t94 + (-d3 * t112 + a2) * t106) * t114) * t111;
t75 = -t111 * t146 + t138;
t55 = qsD3 * t75 + t111 * t128 + t86 * t130;
t66 = d3 * t113 + (d3 * t100 + (-a3 * t106 - a2) * t112) * t114;
t32 = t110 * t46 - t104 * t66 + qsD4 * t118 + (t104 * t55 + t110 * t124) * a4;
t135 = 0.2e1 * t104;
t64 = t99 * t126 + t93 * t150 - t76;
t92 = t104 ^ 2;
t98 = t110 ^ 2;
t42 = (-t98 + t92) * t55 + (-t64 + t72) * t110 * t135;
t145 = t110 * t55;
t53 = t104 * t145;
t43 = t92 * t64 + t98 * t72 + t53;
t52 = qsD4 * (qsD4 / 0.2e1 + t74) + t63;
t151 = 0.2e1 * (t43 + t52) * a5 + d5 * t42 + t32;
t81 = t83 + qsD3;
t147 = t104 * t75 + t110 * t81;
t143 = qsD1 * qsD2;
t101 = sin(qs7);
t107 = cos(qs7);
t141 = t101 * t107;
t102 = sin(qs6);
t108 = cos(qs6);
t140 = t102 * t108;
t103 = sin(qs5);
t109 = cos(qs5);
t139 = t103 * t109;
t134 = a2 * t106 * t114;
t133 = t85 * t143;
t54 = qsD3 * t74 - t152;
t56 = -(t99 - t93) * t133 + (t113 - t142) * t111 * t105;
t39 = -qsD4 * t147 - t104 * t56 - t110 * t54;
t59 = -t104 * t81 + t110 * t75;
t73 = qsD4 + t74;
t51 = t103 * t73 + t109 * t59;
t30 = qsD5 * t51 - t103 * t39 - t109 * t42;
t132 = t30 * t140;
t40 = qsD4 * t59 - t104 * t54 + t110 * t56;
t131 = t40 * t139;
t125 = -t103 * t59 + t109 * t73;
t57 = qsD5 + t147;
t119 = -t102 * t51 - t108 * t57;
t49 = t104 * t68 - t110 * t62;
t115 = -qsD3 * t62 + a3 * t123 + (-t134 + (-t113 - t144) * a3) * t105 + t152 * d3;
t117 = -a4 * t56 - a5 * t40 + d5 * t39 + t115;
t116 = qsD4 * t49 - t104 * t46 - t110 * t66;
t97 = t109 ^ 2;
t96 = t108 ^ 2;
t95 = t107 ^ 2;
t91 = t103 ^ 2;
t90 = t102 ^ 2;
t89 = t101 ^ 2;
t50 = qsD6 + t125;
t48 = t73 * d5 - t49;
t47 = -t73 * a5 - t118;
t45 = -t102 * t57 + t108 * t51;
t44 = t98 * t64 + t92 * t72 - t53;
t41 = t81 * a4 + a5 * t147 - t59 * d5 + t120;
t37 = qsD5 * (qsD5 / 0.2e1 + t147) + t43;
t36 = t103 * t41 + t109 * t48;
t35 = -t103 * t48 + t109 * t41;
t34 = t97 * t44 + t91 * t52 + t131;
t33 = -t102 * t47 + t108 * t36;
t31 = (t97 - t91) * t40 + 0.2e1 * (-t44 + t52) * t139;
t29 = qsD5 * t125 + t103 * t42 - t109 * t39;
t28 = t91 * t44 - t131 + t97 * t52 + qsD6 * (qsD6 / 0.2e1 + t125);
t27 = a5 * t42 + 0.2e1 * (t44 + t52) * d5 + (t104 * t124 - t145) * a4 - t116;
t26 = qsD6 * t45 - t102 * t29 + t108 * t31;
t25 = qsD6 * t119 - t102 * t31 - t108 * t29;
t24 = t96 * t34 + t90 * t37 - t132;
t23 = (-t96 + t90) * t30 + 0.2e1 * (-t34 + t37) * t140;
t22 = qsD5 * t35 + t117 * t103 + t151 * t109;
t21 = -qsD5 * t36 - t151 * t103 + t117 * t109;
t20 = t108 * t22 - t102 * t27 + qsD6 * (-t102 * t36 - t108 * t47);
T_regressor = [t114 / 0.2e1 0 0 t126 -t128 -t133 -t86 * t143 t150 t134 a2 * t137 t64 t56 t55 t54 t72 t46 t115 t44 t42 t40 t39 t52 t32 (-t148 * t135 + t145) * a4 + t116 t34 t31 t30 t29 t37 t22 t21 t24 t23 t26 t25 t28 t20 -qsD6 * t33 - t102 * t22 - t108 * t27 t26 * t141 + t95 * t24 + t89 * t28 (t95 - t89) * t26 + 0.2e1 * (-t24 + t28) * t141 -t107 * t23 - t101 * t25 + qsD7 * (t101 * t50 + t107 * t45) t101 * t23 - t107 * t25 + qsD7 * (-t101 * t45 + t107 * t50) t90 * t34 + t132 + t96 * t37 + qsD7 * (qsD7 / 0.2e1 - t119) t107 * t20 + t101 * t21 + qsD7 * (-t101 * t33 + t107 * t35) -t101 * t20 + t107 * t21 + qsD7 * (-t101 * t35 - t107 * t33);];