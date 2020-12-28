% Plot the complete Atlas leg model with STL files from drcsim
% 
% Input:
% T_c_urdf [4x4x11]
%   Body frames in urdf coordinates (from atlas_dirkin_urdf function)
% lr
%   true = left, false = right
% version [1x1 uint8]
%   Atlas Model Version number
% 
% Output:
% hdl
%   handle to the created patch object
% 
% Sources:
% [1] Baxter Simulator STL files

% Moritz Schappler, schappler@irt.uni-hannover.de, 2014-11
% (c) Institut für Regelungstechnik, Universität Hannover

function hdl = atlas_plot_leg_stl(T_c_urdf, lr, version)

hdl = NaN(6,1);
for j = 1:6
  hdl(j) = atlas_plot_leg_link_stl(j, lr, version, T_c_urdf(:,:,j+1));
end