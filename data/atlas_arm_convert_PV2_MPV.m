% Convert Parameter Vector 2 to Minimum Parameter Vector (MPV)
% 
% This conversion is unique, since MPV is a linear combination of PV2
% entries
% 
% Input:
% PV2
%   Parameter Vector 2
% lr [1x1 logical]
%   true for left, false for right
% version
%   Atlas Version
% 
% Output:
% MPV
%   Minimum Parameter Vector

% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-04
% (c) Institut für Regelungstechnik, Universität Hannover

function MPV = atlas_arm_convert_PV2_MPV(PV2, lr,version)

%% Init
assert(isa(lr,'logical') && all(size(lr) == [1 1]));      
assert(isa(version,'uint8') && all(size(version) == [1 1]));   

%% Load Default Values
% Standard-Werte
[a_mdh, d_mdh] = atlas_arm_parameter_mdh(lr, version);

if version == 3 || version == 4
  dMPVdPV2 = atlas3_arm_diff_MPV_wrt_PV2(a_mdh, d_mdh);
elseif version == 5
  dMPVdPV2 = atlas5_arm_diff_MPV_wrt_PV2(a_mdh, d_mdh);
else
  error('version %d not implemented yet', version);
end
MPV = dMPVdPV2*PV2;