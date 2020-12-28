% Return the minimum parameter vector for Atlas Legs

% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-03
% (c) Institut für Regelungstechnik, Universität Hannover

function MPV = atlas_leg_convert_par1_MPV(rSges_num_mdh, m_num_mdh, Iges_num_mdh, a_mdh, d_mdh, version)

%% Init
assert(isa(version,'uint8') && all(size(version) == [1 1]), ...
  'version number has to be [1x1] uint8'); 

d2 = d_mdh(2);
d3 = d_mdh(3);

a2 = a_mdh(2);
a3 = a_mdh(3);
a4 = a_mdh(4);
a5 = a_mdh(5);
a6 = a_mdh(6);

% Convert to Parameterset 2
% Gleicher Funktionsaufruf wie für den Arm
[mrSges_num_mdh, Ifges_num_mdh] = atlas_arm_convert_mdh_par1_par2(rSges_num_mdh, Iges_num_mdh, m_num_mdh);


% Mass
M1 = m_num_mdh(1);
M2 = m_num_mdh(2);
M3 = m_num_mdh(3);
M4 = m_num_mdh(4);
M5 = m_num_mdh(5);
M6 = m_num_mdh(6);

% Erstes Moment
MX1 = mrSges_num_mdh(1,1);
MY1 = mrSges_num_mdh(1,2);
MZ1 = mrSges_num_mdh(1,3);
MX2 = mrSges_num_mdh(2,1);
MY2 = mrSges_num_mdh(2,2);
MZ2 = mrSges_num_mdh(2,3);
MX3 = mrSges_num_mdh(3,1);
MY3 = mrSges_num_mdh(3,2);
MZ3 = mrSges_num_mdh(3,3);
MX4 = mrSges_num_mdh(4,1);
MY4 = mrSges_num_mdh(4,2);
MZ4 = mrSges_num_mdh(4,3);
MX5 = mrSges_num_mdh(5,1);
MY5 = mrSges_num_mdh(5,2);
MZ5 = mrSges_num_mdh(5,3);
MX6 = mrSges_num_mdh(6,1);
MY6 = mrSges_num_mdh(6,2);
MZ6 = mrSges_num_mdh(6,3);

% Inertia
XX1 = Ifges_num_mdh(1,1);
XY1 = Ifges_num_mdh(1,4);
XZ1 = Ifges_num_mdh(1,5);
YY1 = Ifges_num_mdh(1,2);
YZ1 = Ifges_num_mdh(1,6);
ZZ1 = Ifges_num_mdh(1,3);
XX2 = Ifges_num_mdh(2,1);
XY2 = Ifges_num_mdh(2,4);
XZ2 = Ifges_num_mdh(2,5);
YY2 = Ifges_num_mdh(2,2);
YZ2 = Ifges_num_mdh(2,6);
ZZ2 = Ifges_num_mdh(2,3);
XX3 = Ifges_num_mdh(3,1);
XY3 = Ifges_num_mdh(3,4);
XZ3 = Ifges_num_mdh(3,5);
YY3 = Ifges_num_mdh(3,2);
YZ3 = Ifges_num_mdh(3,6);
ZZ3 = Ifges_num_mdh(3,3);
XX4 = Ifges_num_mdh(4,1);
XY4 = Ifges_num_mdh(4,4);
XZ4 = Ifges_num_mdh(4,5);
YY4 = Ifges_num_mdh(4,2);
YZ4 = Ifges_num_mdh(4,6);
ZZ4 = Ifges_num_mdh(4,3);
XX5 = Ifges_num_mdh(5,1);
XY5 = Ifges_num_mdh(5,4);
XZ5 = Ifges_num_mdh(5,5);
YY5 = Ifges_num_mdh(5,2);
YZ5 = Ifges_num_mdh(5,6);
ZZ5 = Ifges_num_mdh(5,3);
XX6 = Ifges_num_mdh(6,1);
XY6 = Ifges_num_mdh(6,4);
XZ6 = Ifges_num_mdh(6,5);
YY6 = Ifges_num_mdh(6,2);
YZ6 = Ifges_num_mdh(6,6);
ZZ6 = Ifges_num_mdh(6,3);

%% Parameter
if version == 3 || version == 4 || version == 5
  % Quelle: codeexport/atlas4_leg_minimal_parameter_vector.m
  t8 = M5 + M6;
  t7 = M4 + t8;
  t4 = (M3 + t7);
  t14 = MZ4 + MZ5;
  t6 = (MZ3 + t14);
  t19 = d3 * t4 + t6;
  t18 = (d2 * (M2 + t4));
  t3 = a4 ^ 2 * t7;
  t16 = (-YY3 - t3);
  t5 = a5 ^ 2 * t8;
  t15 = (-YY4 - t5);
  t13 = 2 * d3 * t6 + YY5 - t15 - t16;
  t12 = (a3 ^ 2);
  t9 = d3 ^ 2;
  MPV = [t12 * t4 + YY2 + ZZ1 + (2 * MZ2 + t18) * d2; MX1; MY1 - MZ2 - t18; XX2 - YY2 + (-t12 + t9) * t4 + t13; t19 * a3 + XY2; XZ2; YZ2; ZZ2 + (t9 + t12) * t4 + t13; a3 * t4 + MX2; MY2 - t19; XX3 + t16; XY3; -a4 * t14 + XZ3; YZ3; ZZ3 + t3; a4 * t7 + MX3; MY3; XX4 + t15; XY4; -MZ5 * a5 + XZ4; YZ4; ZZ4 + t5; a5 * t8 + MX4; MY4; XX5 + YY6 - YY5; XY5; XZ5; YZ5; ZZ5 + YY6; MX5; MY5 + MZ6; XX6 - YY6; XY6; XZ6; YZ6; ZZ6; MX6; MY6;];
else
  error('Version %d not implemented yet', version);
end