% Return the minimum parameter vector

% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-02
% (c) Institut für Regelungstechnik, Universität Hannover

function MPV = atlas_arm_convert_par1_MPV(rSges_num_mdh, m_num_mdh, Iges_num_mdh, a_mdh, d_mdh, version)

%% Init

assert(isa(version,'uint8') && all(size(version) == [1 1]), ...
  'version number has to be [1x1] uint8'); 
assert(isa(rSges_num_mdh,'double') && isreal(rSges_num_mdh) && all(size(rSges_num_mdh) <= [7,3]), ...
  'rSges_num_mdh has to be [Nx3] double'); 
assert(isa(m_num_mdh,'double') && isreal(m_num_mdh) && all(size(m_num_mdh) <= [7 1]), ...
  'm_num_mdh has to be [1x1] double'); 
assert(isa(Iges_num_mdh,'double') && isreal(Iges_num_mdh) && all(size(Iges_num_mdh) <= [7 6]), ...
  'Iges_num_mdh has to be [Nx6] double'); 
assert(isa(a_mdh,'double') && isreal(a_mdh) && all(size(a_mdh) <= [7 1]), ...
  'a_mdh has to be [Nx1] double'); 
assert(isa(d_mdh,'double') && isreal(d_mdh) && all(size(d_mdh) <= [7 1]), ...
  'd_mdh has to be [Nx1] double'); 


d3 = d_mdh(3);
d5 = d_mdh(5);

a2 = a_mdh(2);
a3 = a_mdh(3);
a4 = a_mdh(4);
a5 = a_mdh(5);
a6 = a_mdh(6);

% Convert to Parameterset 2
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
if version == 3 || version == 4
  % Quelle: codeexport/atlas3_arm_minimal_parameter_vector.m
  MPV = [ZZ1 + YY2 + (a3 ^ 2 * (M3 + M4 + M5 + M6)) + (a2 ^ 2 * (M2 + M3 + M4 + M5 + M6)); (a2 * (M2 + M3 + M4 + M5 + M6)) + MX1; MY1 + MZ2; (d3 ^ 2 * (M3 + M4 + M5 + M6)) + (2 * d3 * MZ3) + XX2 + YY3 + (a4 ^ 2 * (M4 + M5 + M6)) - YY2 - (a3 ^ 2 * (M3 + M4 + M5 + M6)); XY2 + (a3 * MZ3) + (a3 * d3 * (M3 + M4 + M5 + M6)); XZ2; YZ2; ZZ2 + YY3 + (a4 ^ 2 * (M4 + M5 + M6)) + (2 * d3 * MZ3) + ((a3 ^ 2 + d3 ^ 2) * (M3 + M4 + M5 + M6)); (a3 * (M3 + M4 + M5 + M6)) + MX2; MY2 - MZ3 - (d3 * (M3 + M4 + M5 + M6)); XX3 + YY4 + (a5 ^ 2 * (M5 + M6)) - YY3 - (a4 ^ 2 * (M4 + M5 + M6)); -(MZ4 * a4) + XY3; XZ3; YZ3; ZZ3 + YY4 + (a5 ^ 2 * (M5 + M6)) + (a4 ^ 2 * (M4 + M5 + M6)); (a4 * (M4 + M5 + M6)) + MX3; MY3 + MZ4; (d5 ^ 2 * (M5 + M6)) + (2 * d5 * MZ5) + XX4 + (a6 ^ 2 * M6) + YY5 - YY4 - (a5 ^ 2 * (M5 + M6)); XY4 + (a5 * MZ5) + (a5 * d5 * (M5 + M6)); XZ4; YZ4; ZZ4 + (a6 ^ 2 * M6) + YY5 + (2 * d5 * MZ5) + ((a5 ^ 2 + d5 ^ 2) * (M5 + M6)); (a5 * (M5 + M6)) + MX4; MY4 - MZ5 - (d5 * (M5 + M6)); -(a6 ^ 2 * M6) + XX5 - YY5 + YY6; -(MZ6 * a6) + XY5; XZ5; YZ5; (a6 ^ 2 * M6) + YY6 + ZZ5; (M6 * a6) + MX5; MY5 + MZ6; XX6 - YY6; XY6; XZ6; YZ6; ZZ6; MX6; MY6;];
elseif version == 5
  
  M7 = m_num_mdh(7);
  MX7 = mrSges_num_mdh(7,1);
  MY7 = mrSges_num_mdh(7,2);
  MZ7 = mrSges_num_mdh(7,3);  
  XX7 = Ifges_num_mdh(7,1);
  XY7 = Ifges_num_mdh(7,4);
  XZ7 = Ifges_num_mdh(7,5);
  YY7 = Ifges_num_mdh(7,2);
  YZ7 = Ifges_num_mdh(7,6);
  ZZ7 = Ifges_num_mdh(7,3);
  
  % Quelle: codeexport/atlas5_arm_minimal_parameter_vector_matlab.m
  t6 = (M5 + M6 + M7);
  t5 = M4 + t6;
  t2 = (a4 ^ 2 * t5);
  t19 = (-YY3 - t2);
  t11 = (a5 ^ 2);
  t18 = (t11 * t6 + YY4);
  t17 = 2 * d5 * MZ5 + YY5;
  t16 = 2 * d3 * MZ3 - t19;
  t3 = (M3 + t5);
  t15 = d3 * t3 + MZ3;
  t14 = d5 * t6 + MZ5;
  t13 = (a3 ^ 2);
  t10 = d3 ^ 2;
  t9 = d5 ^ 2;
  t1 = M2 + t3;
  MPV = [a2 ^ 2 * t1 + t13 * t3 + YY2 + ZZ1; a2 * t1 + MX1; MY1 + MZ2; XX2 - YY2 + (t10 - t13) * t3 + t16; t15 * a3 + XY2; XZ2; YZ2; ZZ2 + (t10 + t13) * t3 + t16; a3 * t3 + MX2; MY2 - t15; XX3 + t18 + t19; -MZ4 * a4 + XY3; XZ3; YZ3; ZZ3 + t2 + t18; a4 * t5 + MX3; MY3 + MZ4; t9 * t6 + XX4 + t17 - t18; t14 * a5 + XY4; XZ4; YZ4; ZZ4 + (t9 + t11) * t6 + t17; a5 * t6 + MX4; MY4 - t14; XX5 + YY6 - YY5; XY5; XZ5; YZ5; ZZ5 + YY6; MX5; MY5 + MZ6; XX6 + YY7 - YY6; XY6; XZ6; YZ6; ZZ6 + YY7; MX6; MY6 - MZ7; XX7 - YY7; XY7; XZ7; YZ7; ZZ7; MX7; MY7;];
else
  error('Version %d not implemented yet', version);
end