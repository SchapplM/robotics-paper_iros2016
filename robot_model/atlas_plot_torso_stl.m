% Plot the complete Atlas arm model with STL files from drcsim
% 
% Input:
% T_c_urdf [4x4x11]
%   Body frames in urdf coordinates (from atlas_dirkin_urdf function)
% version [1x1 uint8]
%   Atlas Model Version number
% 
% Output:
% hdl
%   handle to the created patch object
% 
% Sources:
% [1] Atlas Simulator STL files

% Moritz Schappler, schappler@irt.uni-hannover.de, 2014-11
% (c) Institut für Regelungstechnik, Universität Hannover

function hdl = atlas_plot_torso_stl(T_c_urdf, version)

hdl = NaN(5,1);
for j = 1:5
  hdl(j) = atlas_plot_torso_link_stl(j, version, T_c_urdf(:,:,j));
end