% Convert dynamic parameters from urdf to mdh (v5)
% 
% Input:
% lr [1x1 logical]
%   true for left, false for right
% rSges_num_urdf [7x3 double]
%   center of mass of all atlas arm bodies in urdf frames
% Iges_num_urdf [7x6 double]
%   inertia of all atlas arm bodies in urdf frames
%   order: xx, yy, zz, xy, xz, yz
% 
% Output:
% rSges_num_mdh [7x3 double]
%   center of mass of all atlas arm bodies in MDH frames
% Iges_num_mdh [7x6 double]
%   inertia of all atlas arm bodies in MDH frames
%   order: xx, yy, zz, xy, xz, yz

% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-01
% (c) Institut für Regelungstechnik, Universität Hannover


function [rSges_num_mdh, Iges_num_mdh] = ...
             atlas5_arm_convert_urdf_mdh(lr, rSges_num_urdf, Iges_num_urdf)

%% Init
% Coder Information
%#codegen

assert(isa(lr,'logical') && all(size(lr) == [1 1]));      
assert(isa(rSges_num_urdf,'double') && all(size(rSges_num_urdf) == [7 3]));  
assert(isa(Iges_num_urdf,'double') && all(size(Iges_num_urdf) == [7 6]));  


%% get frames
% hard coded transformations calculated in testfunctions/atlas_transformation_urdf_mdh_test
p_num = atlas5_arm_parameter_urdf(lr);
T_c_mdh_urdf_hc = NaN(4,4,8); % Transformation from urdf to mdh frame (hard coded)
for i = 1:8
  T_c_mdh_urdf_hc(:,:,i) = eye(4);
end

    T_c_mdh_urdf_hc(:,:,1) =     [
                                  0.0000    -1.0000         0    p_num(2)
                                  1.0000     0.0000         0    -p_num(1)
                                       0          0    1.0000    -p_num(3)
                                       0          0         0      1.0000];

    T_c_mdh_urdf_hc(:,:,2) =     [
                                  0.0000   -1.0000         0           0
                                  1.0000    0.0000         0           0
                                  0         0         1.0000   -p_num(7)
                                  0         0         0           1.0000];
    if lr == false
      T_c_mdh_urdf_hc(2,4,2) = -T_c_mdh_urdf_hc(2,4,2);
    end

    T_c_mdh_urdf_hc(:,:,3) =     [
                                  -0.0000         0    1.0000   -0.0000
                                   0.0000   -1.0000    0.0000         0
                                   1.0000    0.0000    0.0000         0
                                        0         0         0    1.0000];

    T_c_mdh_urdf_hc(:,:,4) =     [
                                  -0.0000    0.0000   1.0000     -0.0000
                                   1.0000         0   0.0000           0
                                   0.0000    1.0000  -0.0000  -p_num(10)
                                        0         0        0      1.0000];

    if lr % correct signchange from last version to this
      T_c_mdh_urdf_hc(3,4,4) = -T_c_mdh_urdf_hc(3,4,4);
    end
                                      
    T_c_mdh_urdf_hc(:,:,5) =     [
                                  -0.0000    0.0000    1.0000   -0.0000
                                   0.0000   -1.0000    0.0000    0.0000
                                   1.0000    0.0000    0.0000         0
                                        0         0         0    1.0000];
    % bis hierhin identisch mit v4  
    T_c_mdh_urdf_hc(:,:,6) = [[[0,0,1;1,0,0;0,1,0], [0;0;0]];[0,0,0,1]];
    T_c_mdh_urdf_hc(:,:,7) = [[[0,0,1;0,-1,0;1,0,0], [0;0;0]];[0,0,0,1]];
    T_c_mdh_urdf_hc(:,:,8) = [[[0,0,1;1,0,0;0,1,0], [0;0;0]];[0,0,0,1]];
    
    if lr % add additional transformations in new kinematic of simple shapes urdf
      T_c_mdh_urdf_hc(:,:,2) = T_c_mdh_urdf_hc(:,:,2) * trotz(3.14159265359); % shz->shx rotz pi
      T_c_mdh_urdf_hc(:,:,3) = T_c_mdh_urdf_hc(:,:,3) * trotz(3.14159265359); % shx->ely rotz pi (from before)
      T_c_mdh_urdf_hc(:,:,4) = T_c_mdh_urdf_hc(:,:,4) * trotz(3.14159265359); % ely->elx rotz pi (from before)
      T_c_mdh_urdf_hc(:,:,5) = T_c_mdh_urdf_hc(:,:,5) * trotz(3.14159265359); % elx->wry rotz pi (from before)
      T_c_mdh_urdf_hc(:,:,6) = T_c_mdh_urdf_hc(:,:,6) * trotz(3.14159265359) * troty(3.14159265359); % wry->wrx rotz pi (from before) and roty pi
      T_c_mdh_urdf_hc(:,:,7) = T_c_mdh_urdf_hc(:,:,7) * trotz(3.14159265359) * troty(3.14159265359); % wrx->wry2 rotz pi (from before) and roty pi (from before)
      T_c_mdh_urdf_hc(:,:,8) = T_c_mdh_urdf_hc(:,:,8) * trotz(3.14159265359); % wry2->ee rotz pi (from before), roty pi from before compensated by additional roty pi
    end
    
%% Convert center of mass
rSges_num_mdh = NaN(7,3);
for i = 1:7
  rSges_num_mdh(i,:) = (T_c_mdh_urdf_hc(1:3,1:4,i+1) * [rSges_num_urdf(i,:)';1])';
end
%% Convert Inertia
Iges_num_mdh = NaN(7,6);
for i = 1:7
  I_i_urdf = inertiavector2matrix(Iges_num_urdf(i,:));
  I_i_mdh = T_c_mdh_urdf_hc(1:3,1:3,i+1) * I_i_urdf * T_c_mdh_urdf_hc(1:3,1:3,i+1)';
  Iges_num_mdh(i,:) = inertiamatrix2vector(I_i_mdh);
end     