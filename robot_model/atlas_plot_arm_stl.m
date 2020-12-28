% Plot the complete Atlas arm model with STL files from drcsim
% 
% Input:
% T_c_urdf [4x4x11]
%   Body frames in urdf coordinates (from atlas_dirkin_urdf function)
% lr [1x1 logical]
%   true for left, false for right
% version [1x1 uint8]
%   Atlas Model Version number

% Output:
% hdl
%   handle to the created patch object
% 
% Sources:
% [1] Atlas Simulator STL files

% Moritz Schappler, schappler@irt.uni-hannover.de, 2014-11
% (c) Institut für Regelungstechnik, Universität Hannover

function hdl = atlas_plot_arm_stl(T_c_urdf, lr, version)

assert(isa(lr,'logical') && all(size(lr) == [1 1]), ...
  'Left/Right flag has to be [1x1] logical'); 
assert(isa(version,'uint8') && all(size(version) == [1 1]), ...
  'version number has to be [1x1] uint8'); 

hdl = NaN(7,1);
for j = 1:6
  hdl(j) = atlas_plot_arm_link_stl(j, lr, version, T_c_urdf(:,:,j+1));
end
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');