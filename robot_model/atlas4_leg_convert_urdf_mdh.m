% Convert dynamic parameters from urdf to mdh for atlas v4 leg
% 
% Input:
% lr [1x1 logical]
%   true for left, false for right
% rSges_num_urdf [6x3 double]
%   center of mass of all atlas arm bodies in urdf frames
% Iges_num_urdf [6x6 double]
%   inertia of all atlas arm bodies in urdf frames
%   order: xx, yy, zz, xy, xz, yz
% 
% Output:
% rSges_num_mdh [6x3 double]
%   center of mass of all atlas arm bodies in MDH frames
% Iges_num_mdh [6x6 double]
%   inertia of all atlas arm bodies in MDH frames
%   order: xx, yy, zz, xy, xz, yz

% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-03
% (c) Institut für Regelungstechnik, Universität Hannover


function [rSges_num_mdh, Iges_num_mdh] = atlas4_leg_convert_urdf_mdh(lr, rSges_num_urdf, Iges_num_urdf)

%% Init
% Coder Information
%#codegen

assert(isa(lr,'logical') && all(size(lr) == [1 1]));      
assert(isa(rSges_num_urdf,'double') && all(size(rSges_num_urdf) == [6 3]));  
assert(isa(Iges_num_urdf,'double') && all(size(Iges_num_urdf) == [6 6]));  

%% get frames
p_num = atlas4_leg_parameter_urdf(lr);
T_c_mdh_urdf_hc = NaN(4,4,8); % Transformation from urdf to mdh frame (hard coded)
for i = 1:8
  T_c_mdh_urdf_hc(:,:,i) = eye(4);
end
% T_c_mdh(:,:,j+1)\T_c_urdf(:,:,j+1)
T_c_mdh_urdf_hc(:,:,1) = transl([0;0;p_num(1)]);

T_c_mdh_urdf_hc(:,:,2) = trotz(-pi/2);

T_c_mdh_urdf_hc(:,:,3) = troty(pi/2)*trotz(pi)*transl([-p_num(2);0;0]);

phi = -atan(p_num(5)/p_num(6));
T_c_mdh_urdf_hc(:,:,4) = trotz(phi)*troty(pi/2)*trotz(pi/2); 

T_c_mdh_urdf_hc(:,:,5) = troty(pi/2)*trotz(pi/2);

T_c_mdh_urdf_hc(:,:,6) = troty(pi/2)*trotz(pi/2);

T_c_mdh_urdf_hc(:,:,7) = troty(pi/2)*trotz(pi);
%% Convert center of mass
rSges_num_mdh = NaN(6,3);
for i = 1:6
  rSges_num_mdh(i,:) = (eye(3,4)*T_c_mdh_urdf_hc(:,:,i+1) * [rSges_num_urdf(i,:)';1])';
end
%% Convert Inertia
Iges_num_mdh = NaN(6,6);
for i = 1:6
  I_i_urdf = inertiavector2matrix(Iges_num_urdf(i,:));
  I_i_mdh = T_c_mdh_urdf_hc(1:3,1:3,i+1) * I_i_urdf * T_c_mdh_urdf_hc(1:3,1:3,i+1)';
  Iges_num_mdh(i,:) = inertiamatrix2vector(I_i_mdh);
end