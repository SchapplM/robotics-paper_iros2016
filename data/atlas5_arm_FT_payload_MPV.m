% Return MPV for payload behind F/T sensor
% 
% Only use static parameters. Ignore inertia of the payload.
% Assume payload CoM data in hand frame (not: FT sensor frame).
% 
% Input:
%   m_pl
%     mass of the payload
%   r_pl_EE
%     center of mass of the payload (in hand-frame)
%   I_pl_EE
%     inertia of the payload (in hand frame)
%     order: XX, XY, XZ, YY, YZ, ZZ
%   lr
%     left/right flag
% Output:
%   MPV_payload
% Testfunction:
%   calibration/hand_force_torque_calibration_test

% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-04
% (c) Institut für Regelungstechnik, Universität Hannover



function MPV_payload = atlas5_arm_FT_payload_MPV(m_pl, r_pl_EE, I_pl_EE, lr)

%% Init
assert(isa(m_pl,'double') && isreal(m_pl) && all(size(m_pl) == [1 1]));
assert(isa(r_pl_EE,'double') && isreal(r_pl_EE) && all(size(r_pl_EE) == [3 1]));
assert(isa(I_pl_EE,'double') && isreal(I_pl_EE) && all(size(I_pl_EE) == [6 1]));
assert(isa(lr,'logical') && all(size(lr) == [1 1])); 
NJA = 7;

%% Convert Payload data from sensor frame to hand frame
% TODO.
% Assume data in hand frame for now.

%% convert payload data to whole arm PV2 data

PV2 = zeros(NJA*10,1);
% Leave inertia at zero

% set mass of the last link
PV2(end) = m_pl;
% First moment
PV2(end-3:end-1) = m_pl*r_pl_EE;
% Inertia
PV2(end-9:end-4) = I_pl_EE;

%% convert to MPV
MPV_payload = atlas_arm_convert_PV2_MPV(PV2, lr,uint8(5));
