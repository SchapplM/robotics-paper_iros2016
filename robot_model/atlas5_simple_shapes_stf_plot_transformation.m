% Hard coded transformations between urdf kinematics from
% b1fd87405f181721888af26eb3646d55f8ddb8ab to the urdf kinematics from
% 2ef658d873147440ee853191c8382e125e952a09 in order to be able to plot the robot
% with the new kinematics from data/atlas_v5_simple_shapes_with_head.urdf.
% 
% Input: none
% 
% Output:
% T_c_urdf_l_neu_alt [4x4x9]
%   homogenious transformation matrices for each body frame from T_c_urdf_l_new
%   to T_c_urdf_l_old.
% 
% Sources:
% [1] data/atlas_v5_simple_shapes_with_head.urdf

% Jonathan Vorndamme, vorndamme@irt.uni-hannover.de, 2017-08
% (c) Institut für Regelungstechnik, Leibniz Universität Hannover

function T_c_urdf_l_new_old = atlas5_simple_shapes_stf_plot_transformation()

T_c_urdf_l_new_old(:,:,1) = [
                             -1     0     0     0
                              0    -1     0     0
                              0     0     1     0
                              0     0     0     1];

T_c_urdf_l_new_old(:,:,2) = [
                             -1     0     0     0
                              0    -1     0     0
                              0     0     1     0
                              0     0     0     1];

T_c_urdf_l_new_old(:,:,3) = [
                             -1     0     0     0
                              0    -1     0     0
                              0     0     1     0
                              0     0     0     1];

T_c_urdf_l_new_old(:,:,4) = [
                             -1     0     0     0
                              0    -1     0     0
                              0     0     1     0
                              0     0     0     1];

T_c_urdf_l_new_old(:,:,5) = [
                             -1     0     0     0
                              0    -1     0     0
                              0     0     1     0
                              0     0     0     1];

T_c_urdf_l_new_old(:,:,6) = [
                             1     0     0     0
                             0    -1     0     0
                             0     0    -1     0
                             0     0     0     1];

T_c_urdf_l_new_old(:,:,7) = [
                             1     0     0     0
                             0    -1     0     0
                             0     0    -1     0
                             0     0     0     1];

T_c_urdf_l_new_old(:,:,8) = [
                             -1     0     0     0
                              0    -1     0     0
                              0     0     1     0
                              0     0     0     1];

T_c_urdf_l_new_old(:,:,9) = [
                             1     0     0     0
                             0     1     0     0
                             0     0     1     0
                             0     0     0     1];