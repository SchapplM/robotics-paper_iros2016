% Plot different measured values from robot experiments
% This script is called by a settings file creating the structure
% SettingsStruct in the workspace
% 
% Creates the following figures:
% the figures for each limb are ordered left arm, right arm, left leg,
% right leg.
%    1    Joint positions (all joints)
%    2    Joint velocities (all joints)
%    3    Joint torques (all joints)
%    4    FT sensors (all joints)
%    8    IMU
%   11-14 limb joint positions
%   15-18 limb joint velocities
%   21-24 limb joint accelerations
%   25-28 limb joint torques
%   31-34 limb cartesian error
%   35-38 limb collision detection
%   40    electric motor data
%   41-42 hydraulics data
%   51-54 impedance controller gains
%   61-62 impedance controller output
%   65-66 impedance controller output (observer)
%   71-72 impedance controller output
%   73-74 stiffness comparison (cartesian)
%   75-76 stiffness comparison (joint space)
%   80-81 robot plots
%   91-94 limb cartesian endpoint velocity
% 
% Sources:
% [1] Robotik I Skript (WS_2014_15)

% TODO: Move appending of files to own function
% TODO: Experimentnamen als Titel oben in jedes Bild

% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-02
% (c) Institut für Regelungstechnik, Universität Hannover

% oft aufgerufene Funktionen kompilieren
Mex_Erstellen({'atlas_arm_invdyn_fb_plin_minpar_sym_lag_varpar', ...
'atlas_leg_invdyn_fb_plin_minpar_sym_lag_varpar', ...
'atlas_arm_gravload_plin_minpar_sym_lag_varpar', ...
'atlas_leg_gravload_plin_minpar_sym_lag_varpar', ...
'atlas_arm_fkine', ...
'atlas_arm_jacobig', ...
'quat2r', ...
'apply_FT_calibration', ...
'atlas_T_pelvis_utorso'});

% suppress warnings
%#ok<*CCAT>
%% User Settings

% Alle Einstellungen hier NAN setzen
mode = SettingsStruct.mode;
file_number_range = SettingsStruct.file_number_range;
atlas_version = SettingsStruct.atlas_version;
N_calc = SettingsStruct.N_calc;
if N_calc > 1e5
  try %#ok<TRYNC>
    h = parpool; % use parallel computation only when needed.
    h.IdleTimeout = 60*24;  % one day
  end
end
plot_robot = SettingsStruct.plot_robot;
data_path = SettingsStruct.data_path;
eval_path = SettingsStruct.eval_path;
plot_q_all = SettingsStruct.plot_q_all;
plot_qD_all = SettingsStruct.plot_qD_all;
plot_tau_all = SettingsStruct.plot_tau_all;
plot_tau_ext = SettingsStruct.plot_tau_ext;
plot_FT = SettingsStruct.plot_FT;
if ~isfield(SettingsStruct, 'calib_Fext')
  calib_Fext = 'first';
else
  calib_Fext = SettingsStruct.calib_Fext;
end
plot_IMU = SettingsStruct.plot_IMU;
plot_q_limb = SettingsStruct.plot_q_limb;
plot_qD_limb = SettingsStruct.plot_qD_limb;
plot_qDD_limb = SettingsStruct.plot_qDD_limb;
plot_tau_limb = SettingsStruct.plot_tau_limb;
plot_elecmot = SettingsStruct.plot_elecmot;
plot_hydraulics = SettingsStruct.plot_hydraulics;
plot_actuator_state = SettingsStruct.plot_actuator_state;
plot_actuator_raw = SettingsStruct.plot_actuator_raw;
plot_arm_error = SettingsStruct.plot_arm_error;
if isfield(SettingsStruct, 'plot_EE_velo')
  plot_EE_velo = SettingsStruct.plot_EE_velo;
else
  plot_EE_velo = false(1,4);
end
plot_impctrl = SettingsStruct.plot_impctrl;
plot_impctrl_sl = SettingsStruct.plot_impctrl_sl;
plot_colldet = SettingsStruct.plot_colldet;
plot_stiffness = SettingsStruct.plot_stiffness;
save_plot = SettingsStruct.save_plot;

MPV_lArm_damp = SettingsStruct.MPV_lArm_damp;
MPV_lArm_model = SettingsStruct.MPV_lArm_model;
MPV_rArm_damp = SettingsStruct.MPV_rArm_damp;
MPV_rArm_model = SettingsStruct.MPV_rArm_model;
% Einstellungen Laden


% TODO: Alle Einstellungen hier anzeigen und prüfen


%% Dateien laden
for file_number = file_number_range
%% Init

% if lr == true % left
%   data_path = fullfile(tb_path, 'trajexport','l_arm','out');
% else
%   data_path = fullfile(tb_path, 'trajexport','r_arm','out');
% end
files = dir(fullfile(data_path, '*.mat'));

% Print File List
for i = 1:length(files)
  fprintf('[%d/%d] %s\n', i, length(files), files(i).name);
end

if strcmp(mode, 'single')
  if ~exist(data_path, 'file')
    error('Folder %s does not exist.', data_path);
  end
  if length(files) < file_number
    error('Folder %s has no mat file number %d', data_path, file_number);
  end
  filename = files(file_number).name;
  % load arbitrary data file
  Data = load_experiment_data(fullfile(data_path, filename));
  figtitle = filename;
elseif strcmp(mode, 'all')
  % Append the data
  empty_struct = struct('t', [], 'q', [], 'qD', [], 'qDD', [], 'tau', [], ...
    'F_lH', [], 'F_lF', [], 'F_rH', [], 'F_rF', [], ...
    'pelvis_x_r', [], 'pelvis_xD_r', [], 'pelvis_xDD_t', []);
  Data = struct('FromRobot', empty_struct, 'ToRobot', empty_struct);
  t_End = 0;
  if isempty(files)
    error('no files found in %s', data_path);
  end
  for i = 1:length(files)
    filename = files(i).name;
    DataTmp = load_experiment_data(fullfile(data_path, filename));
    Data.FromRobot.t = [Data.FromRobot.t; t_End+DataTmp.FromRobot.t];
    Data.FromRobot.q = [Data.FromRobot.q; DataTmp.FromRobot.q];
    Data.FromRobot.qD = [Data.FromRobot.qD; DataTmp.FromRobot.qD];
    Data.FromRobot.qDD = [Data.FromRobot.qDD; DataTmp.FromRobot.qDD];
    Data.FromRobot.tau = [Data.FromRobot.tau; DataTmp.FromRobot.tau];

    Data.FromRobot.pelvis_x_r = [Data.FromRobot.pelvis_x_r; DataTmp.FromRobot.pelvis_x_r];
    Data.FromRobot.pelvis_xD_r = [Data.FromRobot.pelvis_xD_r; DataTmp.FromRobot.pelvis_xD_r];
    Data.FromRobot.pelvis_xDD_t = [Data.FromRobot.pelvis_xDD_t; DataTmp.FromRobot.pelvis_xDD_t];

    Data.FromRobot.F_lH = [Data.FromRobot.F_lH; DataTmp.FromRobot.F_lH];
    Data.FromRobot.F_lF = [Data.FromRobot.F_lF; DataTmp.FromRobot.F_lF];
    Data.FromRobot.F_rH = [Data.FromRobot.F_rH; DataTmp.FromRobot.F_rH];
    Data.FromRobot.F_rF = [Data.FromRobot.F_rF; DataTmp.FromRobot.F_rF];
    
    Data.ToRobot.t = [Data.ToRobot.t; t_End+DataTmp.ToRobot.t];
    Data.ToRobot.q = [Data.ToRobot.q; DataTmp.ToRobot.q];
    Data.ToRobot.qD = [Data.ToRobot.qD; DataTmp.ToRobot.qD];
    Data.ToRobot.qDD = [Data.ToRobot.qDD; DataTmp.ToRobot.qDD];
    Data.ToRobot.tau = [Data.ToRobot.tau; DataTmp.ToRobot.tau];  

    % nächste Startzeit
    t_End = max(Data.FromRobot.t(end), Data.ToRobot.t(end))+0.15;
    
    % Mit NaN beenden (damit nicht durchgezeichnet wird zwischen einzelnen
    % Messwert-Dateien
    Data.FromRobot.t = [Data.FromRobot.t; Data.FromRobot.t(end)+0.1];
    Data.FromRobot.q = [Data.FromRobot.q; NaN(1,28)];
    Data.FromRobot.qD = [Data.FromRobot.qD; NaN(1,28)];
    Data.FromRobot.qDD = [Data.FromRobot.qDD; NaN(1,28)];
    Data.FromRobot.tau = [Data.FromRobot.tau; NaN(1,28)];

    Data.FromRobot.F_lH = [Data.FromRobot.F_lH; NaN(1,6)];
    Data.FromRobot.F_lF = [Data.FromRobot.F_lF; NaN(1,6)];
    Data.FromRobot.F_rH = [Data.FromRobot.F_rH; NaN(1,6)];
    Data.FromRobot.F_rF = [Data.FromRobot.F_rF; NaN(1,6)];  
    
    Data.FromRobot.pelvis_x_r = [Data.FromRobot.pelvis_x_r; NaN(1,4)];
    Data.FromRobot.pelvis_xD_r = [Data.FromRobot.pelvis_xD_r; NaN(1,3)];
    Data.FromRobot.pelvis_xDD_t = [Data.FromRobot.pelvis_xDD_t; NaN(1,3)];  

    Data.ToRobot.t = [Data.ToRobot.t; Data.ToRobot.t(end)+0.1];
    Data.ToRobot.q = [Data.ToRobot.q; NaN(1,AS.NJ)];
    Data.ToRobot.qD = [Data.ToRobot.qD; NaN(1,AS.NJ)];
    Data.ToRobot.qDD = [Data.ToRobot.qDD; NaN(1,AS.NJ)];
    Data.ToRobot.tau = [Data.ToRobot.tau; NaN(1,AS.NJ)];  
  end
  figtitle = 'all Files';
else
  error('mode %s not defined!', mode);
end


AS = atlas_const(atlas_version);
FT_Names = {'Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz'};
chains = {'left_arm', 'right_arm', 'left_leg', 'right_leg'};
% parallel pool for dynamic calculation
% try
%   parpool('local')
% end
% Mex_Erstellen({'atlas_arm_gravload_vps', 'atlas_arm_invdyn_fb_vps'});

if ~isempty(Data.ToRobot.t)
  TLim = minmax2(Data.ToRobot.t');
else
  TLim = minmax2(Data.FromRobot.t');
end

% if ~isempty(Data.ToRobot.t)
%   Data.ToRobot.t = NaN;
%   Data.ToRobot.q = [Data.ToRobot.q; NaN(1,AS.NJ)];
%   Data.ToRobot.qD = [Data.ToRobot.qD; NaN(1,AS.NJ)];
%   Data.ToRobot.qDD = [Data.ToRobot.qDD; NaN(1,AS.NJ)];
%   Data.ToRobot.tau = [Data.ToRobot.tau; NaN(1,AS.NJ)];  
% end

%% Add Fields
Data.FromRobot.tau_ext = NaN(length(Data.FromRobot.t), AS.NJ);
Data.FromRobot.Dq = NaN(length(Data.FromRobot.t), AS.NJ);

%% F/T Calibration
if strcmp(calib_Fext, 'ImpCtrl')
  settings_fields = {'ImpCtrlGains_left_arm', 'ImpCtrlGains_right_arm'};
  for lr = [true, false]
    AS = atlas_const(atlas_version, lr);
    
    % use impedance controller F/T calibration as reference
    if isfield(Data.ToRobot, settings_fields{2-lr})
      ImpCtrlStrct = Data.ToRobot.(settings_fields{2-lr}); % temporary variable for changing

      % load FT data
      if lr == true
        F_Mess = Data.FromRobot.F_lH_raw;
      else
        F_Mess = Data.FromRobot.F_rH_raw;
      end

      % calibrate sensor data
      F_mess_calib = NaN(size(F_Mess));
      calib_data = [ImpCtrlStrct.mass(1); ImpCtrlStrct.COM_x(1); ImpCtrlStrct.COM_y(1); ImpCtrlStrct.COM_z(1); ...
          ImpCtrlStrct.offset_F_x(1); ImpCtrlStrct.offset_F_y(1); ImpCtrlStrct.offset_F_z(1); ...
          ImpCtrlStrct.offset_T_x(1); ImpCtrlStrct.offset_T_y(1); ImpCtrlStrct.offset_T_z(1)];
      for i = 1:size(F_mess_calib,1)
        F_mess_calib(i,:) = apply_FT_calibration_mex(F_Mess(i,1:6), ...
          Data.FromRobot.q(i,:), Data.FromRobot.q(i,AS.JI_Arm), ...
          Data.FromRobot.pelvis_x_r(i,:), ...
          lr, Data.FromRobot.atlas_version, calib_data(1), ...
          calib_data(2:4), calib_data(5:10));
      end
      % rotation has to happen after the calibration, because calibration
      % offset is recorded with rotated sensor.
      if lr == true
        Data.FromRobot.F_lH_raw_calib = F_mess_calib;
        Data.FromRobot.F_lH_calib = F_mess_calib;
        % Data.FromRobot.F_lH_calib(:,[1,2,4,5]) = -F_mess_calib(:,[1,2,4,5]);
      else
        Data.FromRobot.F_rH_raw_calib = F_mess_calib;
        Data.FromRobot.F_rH_calib = F_mess_calib;
      end
    end
  end
elseif strcmp(calib_Fext, 'first') % assume first measured force data point as reference
  % calibration of raw data (without correct orientation)
  Data.FromRobot.F_lH_raw_calib = Data.FromRobot.F_lH_raw - ...
    repmat(Data.FromRobot.F_lH_raw(1,:), length(Data.FromRobot.t), 1);
  Data.FromRobot.F_rH_raw_calib = Data.FromRobot.F_rH_raw - ...
    repmat(Data.FromRobot.F_rH_raw(1,:), length(Data.FromRobot.t), 1);
  
  % calibration of rotated data
  Data.FromRobot.F_lH_calib = Data.FromRobot.F_lH - ...
    repmat(Data.FromRobot.F_lH(1,:), length(Data.FromRobot.t), 1);
  Data.FromRobot.F_rH_calib = Data.FromRobot.F_rH - ...
    repmat(Data.FromRobot.F_rH(1,:), length(Data.FromRobot.t), 1);
else
  error('unknown calibration mode for external forces!');
end

%% Output Gains
if isfield(Data.ToRobot, 'ConstGains')
Gains = {'k_q_p', 'k_q_i', 'k_qd_p', 'k_f_p', 'ff_qd', 'ff_qd_d', 'ff_f_d'};
% print gains to post them in the wiki
fprintf('| | ');
for i = [AS.JI_lArm, AS.JI_rArm]
  fprintf('%s |', AS.JN{i});
end
fprintf('\n');
for IG = 1:length(Gains)
  if ~isfield(Data.ToRobot.ConstGains, Gains{IG})
    continue; % Der Gain verändert sich
  end
  GainData = Data.ToRobot.ConstGains.(Gains{IG});
  fprintf('| %s | ', Gains{IG});
  for i = 1:size(GainData,2)
    fprintf('%1.3f | ', GainData(1,i));
  end
  fprintf('\n');
end

end
%% Plot joint angles
if plot_q_all
fprintf('printing all joint positions in one plot\n');
figure(1);set(1, 'Name', 'q', 'NumberTitle', 'off');clf;

subplot(2,3,1);hold on;grid on;
hdl=plot(Data.FromRobot.t, Data.FromRobot.q(:,AS.JI_lArm)); set(gca, 'ColorOrderIndex', 1)
plot(Data.ToRobot.t, Data.ToRobot.q(:,AS.JI_lArm), '-');
legend(hdl, AS.JN(AS.JI_lArm), 'interpreter', 'none');
ylabel('q [rad]');
xlabel('t [s]');
title('Left Arm'); 

subplot(2,3,2);hold on;grid on;
hdl=plot(Data.FromRobot.t, Data.FromRobot.q(:,AS.JI_rArm)); set(gca, 'ColorOrderIndex', 1)
plot(Data.ToRobot.t, Data.ToRobot.q(:,AS.JI_rArm), '-');
legend(hdl, AS.JN(AS.JI_rArm), 'interpreter', 'none');
ylabel('q [rad]');
xlabel('t [s]');
title('Right Arm'); 

subplot(2,3,4);hold on;grid on;
hdl=plot(Data.FromRobot.t, Data.FromRobot.q(:,AS.JI_lLeg)); set(gca, 'ColorOrderIndex', 1)
plot(Data.ToRobot.t, Data.ToRobot.q(:,AS.JI_lLeg), '-');
legend(hdl, AS.JN(AS.JI_lLeg), 'interpreter', 'none');
ylabel('q [rad]');
xlabel('t [s]');
title('Left Leg'); 

subplot(2,3,5);hold on;grid on;
hdl=plot(Data.FromRobot.t, Data.FromRobot.q(:,AS.JI_rLeg)); set(gca, 'ColorOrderIndex', 1)
plot(Data.ToRobot.t, Data.ToRobot.q(:,AS.JI_rLeg), '-');
legend(hdl, AS.JN(AS.JI_rLeg), 'interpreter', 'none');
ylabel('q [rad]');
xlabel('t [s]');
title('Right Leg'); 

subplot(2,3,3);hold on;grid on;
hdl=plot(Data.FromRobot.t, Data.FromRobot.q(:,AS.JI_Torso)); set(gca, 'ColorOrderIndex', 1)
plot(Data.ToRobot.t, Data.ToRobot.q(:,AS.JI_Torso), '-');
legend(hdl, AS.JN(AS.JI_Torso), 'interpreter', 'none');
ylabel('q [rad]');
xlabel('t [s]');
title('Back'); 
linkxaxes;
subplot_expand(1,2,3);
xlim(TLim)
end
%% Plot joint velocities
if plot_qD_all
fprintf('printing all joint velocities in one plot\n');
figure(2);set(2, 'Name', 'qD', 'NumberTitle', 'off'); clf;
subplot(2,3,1);hold on;grid on;
hdl=plot(Data.FromRobot.t, Data.FromRobot.qD(:,AS.JI_lArm)); set(gca, 'ColorOrderIndex', 1)
plot(Data.ToRobot.t, Data.ToRobot.qD(:,AS.JI_lArm), '-');
legend(hdl, AS.JN(AS.JI_lArm), 'interpreter', 'none');
ylabel('qD [rad/s]');
xlabel('t [s]');
title('Left Arm'); 

subplot(2,3,2);hold on;grid on;
hdl=plot(Data.FromRobot.t, Data.FromRobot.qD(:,AS.JI_rArm)); set(gca, 'ColorOrderIndex', 1)
plot(Data.ToRobot.t, Data.ToRobot.qD(:,AS.JI_rArm), '-');
legend(hdl, AS.JN(AS.JI_rArm), 'interpreter', 'none');
ylabel('qD [rad/s]');
xlabel('t [s]');
title('Right Arm'); 

subplot(2,3,4);hold on;grid on;
hdl=plot(Data.FromRobot.t, Data.FromRobot.qD(:,AS.JI_lLeg)); set(gca, 'ColorOrderIndex', 1)
plot(Data.ToRobot.t, Data.ToRobot.qD(:,AS.JI_lLeg), '-');
legend(hdl, AS.JN(AS.JI_lLeg), 'interpreter', 'none');
ylabel('qD [rad/s]');
xlabel('t [s]');
title('Left Leg'); 

subplot(2,3,5);hold on;grid on;
hdl=plot(Data.FromRobot.t, Data.FromRobot.qD(:,AS.JI_rLeg)); set(gca, 'ColorOrderIndex', 1)
plot(Data.ToRobot.t, Data.ToRobot.qD(:,AS.JI_rLeg), '-');
legend(hdl, AS.JN(AS.JI_rLeg), 'interpreter', 'none');
ylabel('qD [rad/s]');
xlabel('t [s]');
title('Right Leg'); 

subplot(2,3,3);hold on;grid on;
hdl=plot(Data.FromRobot.t, Data.FromRobot.qD(:,AS.JI_Torso)); set(gca, 'ColorOrderIndex', 1)
plot(Data.ToRobot.t, Data.ToRobot.qD(:,AS.JI_Torso), '-');
legend(hdl, AS.JN(AS.JI_Torso), 'interpreter', 'none');
ylabel('qD [rad/s]');
xlabel('t [s]');
title('Back'); 
linkxaxes;
subplot_expand(2,2,3);
xlim(TLim)
end
%% Plot joint torque
if plot_tau_all
fprintf('printing all joint torques in one plot\n');
figure(3);clf;set(3, 'Name', 'tau', 'NumberTitle', 'off'); clf;
subplot(2,3,1);hold on;grid on;
hdl=plot(Data.FromRobot.t, Data.FromRobot.tau(:,AS.JI_lArm));
plot(Data.ToRobot.t, Data.ToRobot.tau(:,AS.JI_lArm), '-');
legend(hdl, AS.JN(AS.JI_lArm), 'interpreter', 'none');
ylabel('\tau [Nm]');
xlabel('t [s]');
title('Left Arm'); 

subplot(2,3,2);hold on;grid on;
hdl=plot(Data.FromRobot.t, Data.FromRobot.tau(:,AS.JI_rArm));
plot(Data.ToRobot.t, Data.ToRobot.tau(:,AS.JI_rArm), '-');
legend(hdl, AS.JN(AS.JI_rArm), 'interpreter', 'none');
ylabel('\tau [Nm]');
xlabel('t [s]');
title('Right Arm'); 

subplot(2,3,4);hold on;grid on;
hdl=plot(Data.FromRobot.t, Data.FromRobot.tau(:,AS.JI_lLeg));
plot(Data.ToRobot.t, Data.ToRobot.tau(:,AS.JI_lLeg), '-');
legend(hdl, AS.JN(AS.JI_lLeg), 'interpreter', 'none');
ylabel('\tau [N ?]');
xlabel('t [s]');
title('Left Leg'); 

subplot(2,3,5);hold on;grid on;
hdl=plot(Data.FromRobot.t, Data.FromRobot.tau(:,AS.JI_rLeg));
plot(Data.ToRobot.t, Data.ToRobot.tau(:,AS.JI_rLeg), '-');
legend(hdl, AS.JN(AS.JI_rLeg), 'interpreter', 'none');
ylabel('\tau [N ?]');
xlabel('t [s]');
title('Right Leg'); 

subplot(2,3,3);hold on;grid on;
hdl=plot(Data.FromRobot.t, Data.FromRobot.tau(:,AS.JI_Torso));
plot(Data.ToRobot.t, Data.ToRobot.tau(:,AS.JI_Torso), '-');
legend(hdl, AS.JN(AS.JI_Torso), 'interpreter', 'none');
ylabel('\tau [N ?]');
xlabel('t [s]');
title('Back');  
linkxaxes;
subplot_expand(3,2,3);
xlim(TLim)
end
%% Force/Torque Sensor Data
if plot_FT && isfield(Data.FromRobot, 'F_lH')
  % data with correct orientation (hand frame)
  fprintf('plotting F/T sensor data\n');
  figure(4);clf;set(4, 'Name', 'F_T_Sensors', 'NumberTitle', 'off'); clf;
  subplot(2,2,1);hold on;grid on;
  hdl=plot(Data.FromRobot.t, Data.FromRobot.F_lH);
  legend(hdl, FT_Names, 'interpreter', 'tex');
  title('left Hand F/T'); 
  ylabel('F [N], M [Nm]');

  subplot(2,2,2);hold on;grid on;
  hdl=plot(Data.FromRobot.t, Data.FromRobot.F_rH);
  legend(hdl, FT_Names, 'interpreter', 'tex');
  title('right Hand F/T');  
  ylabel('F [N], M [Nm]');

  subplot(2,2,3);hold on;grid on;
  hdl=plot(Data.FromRobot.t, Data.FromRobot.F_lF);
  legend(hdl, FT_Names, 'interpreter', 'tex');
  title('left Foot F/T');  
  ylabel('F [N], M [Nm]');

  subplot(2,2,4);hold on;grid on;
  hdl=plot(Data.FromRobot.t, Data.FromRobot.F_rF);
  legend(hdl, FT_Names, 'interpreter', 'tex');
  title('right Foot F/T');  
  ylabel('F [N], M [Nm]');
  linkxaxes;
  subplot_expand(4,2,2);
  xlim(TLim)
  
  % calibrated raw data
  figure(5);clf;set(5, 'Name', 'F_T_calib', 'NumberTitle', 'off'); clf;
  % raw measured force (wrong orientation) after calibration
  subplot(2,2,1);hold on;grid on;
  hdl=plot(Data.FromRobot.t, Data.FromRobot.F_lH_raw_calib(:,1:3));
  legend(hdl, FT_Names(1:3), 'interpreter', 'tex');
  title('left Hand F raw calib'); 
  ylabel('F [N]');

  subplot(2,2,2);hold on;grid on;
  hdl=plot(Data.FromRobot.t, Data.FromRobot.F_lH_raw_calib(:,4:6));
  legend(hdl, FT_Names(4:6), 'interpreter', 'tex');
  title('left Hand M raw calib'); 
  ylabel('M [Nm]');

  subplot(2,2,3);hold on;grid on;
  hdl=plot(Data.FromRobot.t, Data.FromRobot.F_rH_raw_calib(:,1:3));
  legend(hdl, FT_Names(1:3), 'interpreter', 'tex');
  title('right Hand F raw calib'); 
  ylabel('F [N]');

  subplot(2,2,4);hold on;grid on;
  hdl=plot(Data.FromRobot.t, Data.FromRobot.F_rH_raw_calib(:,4:6));
  legend(hdl, FT_Names(4:6), 'interpreter', 'tex');
  title('right Hand M raw calib'); 
  ylabel('M [Nm]');
  linkxaxes;
  subplot_expand(5,2,2);
  xlim(TLim)
end
%% IMU Data
if plot_IMU
fprintf('plotting IMU sensor data\n');
figure(8);clf;set(8, 'Name', 'IMU_Sensors', 'NumberTitle', 'off'); clf;
subplot(2,2,1);hold on;grid on;
hdl=plot(Data.FromRobot.t, Data.FromRobot.pelvis_x_r);
legend(hdl, {'quat_x', 'quat_y', 'quat_z', 'quat_w'}, 'interpreter', 'tex');
title('Pelvis Quaternion Orientation');  
ylabel('[1]');

subplot(2,2,2);hold on;grid on;
ori_utorso_angle = NaN(length(Data.FromRobot.t),1);
% Calculate utorso orientation
for i = 1:length(Data.FromRobot.t)
  % pelvis quaternion orientation (correct precision errors)
  quat_pelvis_corr = Data.FromRobot.pelvis_x_r(i,:)/ norm(Data.FromRobot.pelvis_x_r(i,:));
  R_pelvis = quat2r_mex(quat_pelvis_corr);
  T_pelvis_utorso = atlas_T_pelvis_utorso_mex(Data.FromRobot.q(i,:), atlas_version);
  R_utorso = R_pelvis * T_pelvis_utorso(1:3,1:3);

  % calculate angle between utorso z-axis and global z-axis
  ori_utorso_angle(i) = acos(R_utorso(1:3,3)'*[0;0;1] / (1*1));
end
hdl=plot(Data.FromRobot.t, ori_utorso_angle*180/pi);
legend(hdl, {'angle_utorso'}, 'interpreter', 'none');
title('Utorso Angle Orientation');  
ylabel('angle betw. z_{utorso} and z_{world} [deg]');

subplot(2,2,3);hold on;grid on;
hdl=plot(Data.FromRobot.t, Data.FromRobot.pelvis_xD_r);
legend(hdl, {'\omega_x', '\omega_y', '\omega_z'}, 'interpreter', 'tex');
title('Pelvis Angular Velocity'); 
ylabel('\omega [rad/s]');

subplot(2,2,4);hold on;grid on;
hdl=plot(Data.FromRobot.t, Data.FromRobot.pelvis_xDD_t);
legend(hdl, {'a_x', 'a_y', 'a_z'}, 'interpreter', 'tex');
title('Pelvis Linear Acceleration'); 
ylabel('xDD [m/s^2]');
linkxaxes;
subplot_expand(8,2,2);
xlim(TLim)
end
%% Plot position, velocity, acceleration and torque for all joint chains
for limb = [1, 2]
for lr = [true, false]
  

  %% Init  
  figoffset = 0;
  lrString = 'left';
  ImpCtrlStrct = Data.ToRobot.ImpCtrlGains_left_arm;
  if lr == false
    figoffset = figoffset + 1;
    lrString = 'right';
    ImpCtrlStrct = Data.ToRobot.ImpCtrlGains_right_arm;
  end
  AS = atlas_const(atlas_version, lr);
  if limb == 1
    NJL = AS.NJA;
    JI = AS.JI_Arm;
    limbString = 'Arm';
  else
    NJL = 6;
    figoffset = figoffset + 2;
    JI = AS.JI_Leg;
    limbString = 'Leg';
  end
  
  %% Calculations
  
  % calculate torque from external forces
  if (plot_tau_ext || plot_stiffness(2-lr)) && limb == 1
    F_0 = NaN(6,1);
    tau_ext = NaN(length(Data.FromRobot.t),NJL);
    if limb == 1
      if lr == true, F_Mess = Data.FromRobot.F_lH_calib(:, :);
      else F_Mess = Data.FromRobot.F_rH_calib(:, :); end
    else
      if lr == true, F_Mess = Data.FromRobot.F_lF(:, :);
      else F_Mess = Data.FromRobot.F_rF(:, :); end
    end
    for i = 1:length(Data.FromRobot.t)
      if limb == 1
        T_0_EE = atlas_arm_fkine_mex(Data.FromRobot.q(i, JI), lr, atlas_version);
      else
        T_0_EE = atlas_leg_fkine_mex(Data.FromRobot.q(i, JI), lr, atlas_version);
      end
      F_0(1:3) = t2r(T_0_EE(:,:,end))*(F_Mess(i,1:3))';
      F_0(4:6) = t2r(T_0_EE(:,:,end))*(F_Mess(i,4:6))';
      if limb == 1
        tau_ext(i,:) = atlas_arm_jacobig_mex(Data.FromRobot.q(i, JI), lr, atlas_version)'*F_0;
      else
        tau_ext(i,:) = atlas_leg_jacobig_mex(Data.FromRobot.q(i, JI), lr, atlas_version)'*F_0;
      end
    end
    Data.FromRobot.tau_ext(:,AS.JI_Arm) = tau_ext;
  end
  
  % Position Error

  % sample data to robot with data from robot times
  q_Des_time_meas = interp1(Data.ToRobot.t,Data.ToRobot.q(:,AS.JI_Arm),Data.FromRobot.t,'previous');
  Data.FromRobot.Dq(:,AS.JI_Arm) = Data.FromRobot.q(:,AS.JI_Arm) - q_Des_time_meas;  
  
  %% Joint Position (Arm/Leg)
  if plot_q_limb(2-lr + 2*(limb-1))
  fprintf('plotting %s %s position data\n', lrString, limbString);
  figure(11+figoffset);set(11+figoffset, 'Name', sprintf('q_%s_%s', lrString, limbString), 'NumberTitle', 'off');clf;
  for i = 1:NJL
    if NJL == 6, nr = 2; nc = 3;
    else            nr = 3; nc = 3; end
    subplot(nr,nc,i);hold on;grid on;
    plot(Data.FromRobot.t, Data.FromRobot.q(:,JI(i)));
    plot(Data.ToRobot.t, Data.ToRobot.q(:,JI(i)), '-');
    plot(minmax2(Data.FromRobot.t'), [1,1]*AS.q_min(JI(i)), 'r--');
    plot(minmax2(Data.FromRobot.t'), [1,1]*AS.q_max(JI(i)), 'r--');
    xlim(TLim)
    legend({'Ist', 'Soll', 'lim'}, 'interpreter', 'none');
    title(sprintf('%s (q%d) [rad]',  AS.JN{JI(i)}, i), 'interpreter', 'none'); 
    % ylabel(sprintf('q_{%d} [rad]', i));
    xlabel('t [s]');
  end
  linkxaxes;
  subplot_expand(11+figoffset,nr,nc);
  end
  %% Joint Velocity (Arm/Leg)
  if plot_qD_limb(2-lr + 2*(limb-1)) 
  fprintf('plotting %s %s velocity data\n', lrString, limbString);
  figure(15+figoffset);set(15+figoffset, 'Name', sprintf('qD_%s_%s', lrString, limbString), 'NumberTitle', 'off');clf;
  for i = 1:NJL
    if NJL == 6, nr = 2; nc = 3;
    else            nr = 3; nc = 3; end
    subplot(nr,nc,i);hold on;grid on;
    plot(Data.FromRobot.t, Data.FromRobot.qD(:,JI(i)));
    plot(Data.ToRobot.t, Data.ToRobot.qD(:,JI(i)), '-');
    legend({'Ist', 'Soll'}, 'interpreter', 'none');
    title(AS.JN{JI(i)}, 'interpreter', 'none'); 
    xlim(TLim)
    ylabel(sprintf('qD_{%d} [rad/s]', i));
    xlabel('t [s]');
  end
  linkxaxes;
  subplot_expand(15+figoffset,nr,nc);
  end

  %% Joint Acceleration (Arm/Leg)
  if plot_qDD_limb(2-lr + 2*(limb-1)) && isfield(Data.FromRobot, 'qDD')
  fprintf('plotting %s %s acceleration data\n', lrString, limbString);
  figure(21+figoffset);set(21+figoffset, 'Name', sprintf('qDD_%s_%s', lrString, limbString), 'NumberTitle', 'off');clf;
  for i = 1:NJL
    if NJL == 6, nr = 2; nc = 3;
    else            nr = 3; nc = 3; end
    subplot(nr,nc,i);hold on;grid on;
    plot(Data.FromRobot.t, Data.FromRobot.qDD(:,JI(i)));
    plot(Data.ToRobot.t, Data.ToRobot.qDD(:,JI(i)), '-');
    legend({'Ist', 'Soll'}, 'interpreter', 'none');
    title(AS.JN{JI(i)}, 'interpreter', 'none'); 
    xlim(TLim)
    ylabel(sprintf('qDD_{%d} [rad/s^2]', i));
    xlabel('t [s]');
  end
  linkxaxes;
  subplot_expand(21+figoffset,nr, nc);
  end
  %% Joint Torque (Arm/Leg)
  if plot_tau_limb(2-lr + 2*(limb-1))
  fprintf('plotting %s %s effort data\n', lrString, limbString);
  % calculate gravity and inverse dynamics
  N = length(Data.FromRobot.t);

  % load dynamic base parameters
  if limb == 1
    if lr == true
      MPV_arm = MPV_lArm_model;
    else
      MPV_arm = MPV_rArm_model;
    end
  else
    % default values for dynamic parameters
    [~, rSges_urdf_ORIG, ~, Iges_urdf_ORIG] = ...
      atlas_arm_parameter_urdf(lr, uint8(atlas_version));
    [a_mdh, d_mdh, ~, ~, rSges_mdh_ORIG, m_ORIG,Iges_mdh_ORIG] = atlas_leg_parameter_mdh(lr, atlas_version);
    MPV_leg = atlas_leg_convert_par1_MPV(rSges_mdh_ORIG, m_ORIG, Iges_mdh_ORIG, a_mdh, d_mdh, atlas_version);
  end
  tau_invdyn = NaN(N,NJL);
  tau_grav = NaN(N,NJL);
  if isfield(Data.FromRobot, 'pelvis_x_r')
  Q = Data.FromRobot.q;
  QD = Data.FromRobot.qD;
  QDD = Data.FromRobot.qDD;
  X_r = Data.FromRobot.pelvis_x_r;
  fprintf('Calculate inverse dynamics for %d joint angles...', min(N_calc, size(Q,1)));
  t1 = tic;
  parfor i = 1:min(N_calc, size(Q,1))
    q_Rob = Q(i,:);
    qD_Rob = QD(i,:);
    qDD_Rob = QDD(i,:);
    R_pelvis = quat2r_mex(X_r(i,:));
    if limb == 1
      T_pelvis_utorso = atlas_T_pelvis_utorso_mex(q_Rob, atlas_version);
      R_utorso = R_pelvis * T_pelvis_utorso(1:3,1:3);
      g0_utorso = R_utorso'*[0;0;-9.81];

      tau_invdyn(i,:) = atlas_arm_invdyn_fb_plin_minpar_sym_lag_varpar_mex( ...
                    q_Rob(JI), qD_Rob(JI), qDD_Rob(JI), ...
                    g0_utorso', MPV_arm, lr, atlas_version);
      tau_grav(i,:) = atlas_arm_gravload_plin_minpar_sym_lag_varpar_mex( ...
                    q_Rob(JI), ...
                    g0_utorso', MPV_arm, lr, atlas_version);
    else
      g0_pelvis = R_pelvis'*[0;0;-9.81];

      tau_invdyn(i,:) = atlas_leg_invdyn_fb_plin_minpar_sym_lag_varpar_mex( ...
                    q_Rob(JI), qD_Rob(JI), qDD_Rob(JI), ...
                    g0_pelvis', MPV_leg, lr, atlas_version);
      tau_grav(i,:) = atlas_leg_gravload_plin_minpar_sym_lag_varpar_mex( ...
                    q_Rob(JI), ...
                    g0_pelvis', MPV_leg, lr, atlas_version);
    end
  end
  fprintf('... finished. %1.1fs; Average %1.3fms\n', toc(t1), 1e3*toc(t1)/N);
  end
 
  figure(25+figoffset);set(25+figoffset, 'Name', sprintf('tau_%s_%s', lrString, limbString), 'NumberTitle', 'off');clf;
  for i = 1:NJL
    if NJL == 6, nr = 2; nc = 3;
    else            nr = 3; nc = 3; end
    subplot(nr,nc,i);hold on;grid on;
    legend_hdl = plot(Data.FromRobot.t, Data.FromRobot.tau(:,JI(i)));
    legend_content = {'Ist'};
    if ~isempty(Data.ToRobot.tau)
      legend_hdl(end+1) = plot(Data.ToRobot.t, Data.ToRobot.tau(:,JI(i)), '-'); %#ok<SAGROW>
      legend_content = {legend_content{:}, 'Soll'}; 
    end
    if any(~isnan(tau_invdyn(:,i)))
      legend_hdl(end+1) = plot(Data.FromRobot.t, tau_invdyn(:,i)); %#ok<SAGROW>
      legend_content = {legend_content{:}, 'ID'}; 
    end
    if any(~isnan(tau_grav(:,i)))
      legend_hdl(end+1) = plot(Data.FromRobot.t, tau_grav(:,i)); %#ok<SAGROW>
      legend_content = {legend_content{:}, 'Grav'}; 
    end

    % observed joint torque
    if plot_impctrl(2-lr + 2*(limb-1)) && isfield(Data.FromRobot, 'tau_obs')
      legend_hdl(end+1) = plot(Data.FromRobot.t, Data.FromRobot.tau_obs(:,JI(i)), ':'); %#ok<SAGROW>
      legend_content = {legend_content{:}, 'obs'}; 
    end
    
    % external forces joint torque
    if plot_tau_ext
      legend_hdl(end+1) = plot(Data.FromRobot.t, Data.FromRobot.tau_ext(:,AS.JI_Arm(i)), '.-'); %#ok<SAGROW>
      legend_content = {legend_content{:}, 'ext'}; 
    end
    
    % actuator state joint torque
    if plot_actuator_state && isfield(Data.FromRobot, 'tau_as')
      legend_hdl(end+1) = plot(Data.FromRobot.t, Data.FromRobot.tau_as(:,JI(i)),':'); %#ok<SAGROW>
      legend_content = {legend_content{:}, 'as'}; 
    end
    
    % raw actuator joint torque
    if plot_actuator_raw && isfield(Data.FromRobot, 'tau_ar')
      legend_hdl(end+1) = plot(Data.FromRobot.t, Data.FromRobot.tau_ar(:,JI(i)),'--'); %#ok<SAGROW>
      legend_content = {legend_content{:}, 'ar'}; 
    end
    
    % actuator command before the integrator is added
    if plot_impctrl(2-lr + 2*(limb-1)) && isfield(Data.ToRobot, 'tau_cmd')
      legend_hdl(end+1) = plot(Data.ToRobot.t, Data.ToRobot.tau_cmd(:,JI(i)),'--'); %#ok<SAGROW>
      legend_content = {legend_content{:}, 'wo_int'}; 
    end
    
    % calculated torque from electric motors
    if i >=5 && limb == 1 && isfield(Data.FromRobot, 'i_el') && ...
        any(~isnan(Data.FromRobot.i_el(:,AS.JI_Arm(i))))
      tau_elec = Data.FromRobot.i_el(:,AS.JI_Arm(i)).* ...
        AS.k_motor_elec_all(AS.JI_Arm(i)).*...
        AS.gear_ratio_elec_all(AS.JI_Arm(i));
      legend_hdl(end+1) = plot(Data.FromRobot.t, tau_elec); %#ok<SAGROW>
      legend_content = {legend_content{:}, 'elec'}; 
    end
    
    if i == 1 || i == NJL
      legend(legend_hdl, legend_content, 'interpreter', 'none');
    end
    title(AS.JN{JI(i)}, 'interpreter', 'none'); 
    ylabel(sprintf('\\tau_{%d} [Nm]', i));
    xlabel('t [s]');
    xlim(TLim)
  end 
  linkxaxes;
  subplot_expand(25+figoffset,nr, nc);
  end
  
  %% Plot cartesian and joint angle plot_error
  if plot_arm_error(2-lr) && limb == 1
    fprintf('plotting %s %s endpoint error\n', lrString, limbString);
    tic;
    figure(31+figoffset);clf;set(31+figoffset,'Name',sprintf('Error_%s_Arm', lrString), 'NumberTitle', 'off');
    if ~isempty(Data.ToRobot.t)
    % sample data to robot with data from robot times
    q_Des_time_meas = interp1(Data.ToRobot.t,Data.ToRobot.q(:,AS.JI_Arm),Data.FromRobot.t,'previous');
    q_e = Data.FromRobot.q(:,AS.JI_Arm) - q_Des_time_meas;
    % Plot joint position offset
    subplot(2,1,1);
    plot(Data.FromRobot.t, q_e);
    legend(AS.JN(AS.JI_Arm), 'interpreter', 'none');grid on;
    ylabel('\Deltaq [rad]');
    % calculate and plot cartesian error
    x_EE_Soll = NaN(length(Data.FromRobot.t), 7);
    x_EE_Ist = NaN(length(Data.FromRobot.t), 7);
    x_EE_Diff = NaN(length(Data.FromRobot.t), 7);
    for i = 1:length(Data.FromRobot.t)
      T_c_Soll = atlas_arm_fkine_mex(q_Des_time_meas(i,:), lr, atlas_version);
      T_c_Ist = atlas_arm_fkine_mex(Data.FromRobot.q(i,AS.JI_Arm), lr, atlas_version);
      % T_EE_Diff = tr2delta(T_c_Soll(:,:,end), T_c_Ist(:,:,end));
      x_EE_Soll(i,:) = tr2quat(T_c_Soll(:,:,end));
      x_EE_Ist(i,:) = tr2quat(T_c_Ist(:,:,end));
      x_EE_Diff(i,1:3) = T_c_Ist(1:3,4,end) - T_c_Soll(1:3,4,end);
      % tr2quat(T_c_Soll(:,:,end) / T_c_Ist(:,:,end));
    end
    subplot(2,1,2);
    plot(Data.FromRobot.t, [1e3*x_EE_Diff(:,1:3), 1e3*sqrt(sum(abs(x_EE_Diff(:,1:3)).^2,2))]);
    legend({'x', 'y', 'z', 'norm'});
    ylabel('\Delta x trans [mm]');grid on;
    linkxaxes
    subplot_expand(31+figoffset, 2, 1);
    fprintf('Calculated forward kinematics for %d positions for %s %s. %1.4fs\n', ...
      length(Data.FromRobot.t), lrString, limbString,toc);
    end
  end
  
  
  %% Plot End-Effector velocities
  if plot_EE_velo(1+figoffset)
    % EE-Geschw. berechnen (Annahme: Basis fest)
    XD = NaN(length(Data.FromRobot.t),6);
    for i = 1:length(Data.FromRobot.t)
      if limb == 1,   JG = atlas_arm_jacobig_mex(Data.FromRobot.q(i,JI), lr, atlas_version);
      else            JG = atlas_leg_jacobig_mex(Data.FromRobot.q(i,JI), lr, atlas_version); end
      XD(i,:) = JG * (Data.FromRobot.qD(i,JI)');
    end
    XD_Norm = [sqrt(XD(:,1).^2+XD(:,2).^2+XD(:,3).^2), ... % Norm der Geschw.
               sqrt(XD(:,4).^2+XD(:,5).^2+XD(:,6).^2)];    % Norm der Winkelgeschw.
             
    figure(91+figoffset);clf;
    subplot(2,2,1);
    plot(Data.FromRobot.t, XD(:,1:3));
    ylabel('v_{EE}');grid on;
    legend({'v_x', 'v_y', 'v_z'});
    subplot(2,2,2);
    plot(Data.FromRobot.t, XD_Norm(:,1));
    ylabel('||v_{EE}||');grid on;
    subplot(2,2,3);
    plot(Data.FromRobot.t, XD(:,4:6));   
    ylabel('\omega_{EE}');grid on;
    subplot(2,2,4);
    plot(Data.FromRobot.t, XD_Norm(:,2));   
    ylabel('||\omega_{EE}||');grid on;
    linkxaxes
  end
  
  
  %% Plot Collision Detection features
  if plot_colldet(2-lr) && limb == 1 && isfield(Data.FromRobot, 'tau_obs')
    fprintf('plotting %s %s endpoint error\n', lrString, limbString);
    tic;
    figure(35+figoffset);clf;set(35+figoffset,'Name',sprintf('CollDet_%s_Arm', lrString), 'NumberTitle', 'off');
    
    for i = 1:AS.NJA
      if NJL == 6, nr = 2; nc = 3;
      else            nr = 3; nc = 3; end
      subplot(nr,nc,i);hold on;grid on;
      
      % observer joint torque
      legend_hdl(end+1) = plot(Data.FromRobot.t, Data.FromRobot.tau_obs(:,JI(i))); %#ok<SAGROW>
      legend_content = {legend_content{:}, 'obs'}; 
    
      % threshold for collision detection
      stairs(ImpCtrlStrct.t, -10./ImpCtrlStrct.K_cd_o, 'r--');   
      legend_hdl(end+1) = stairs(ImpCtrlStrct.t, 10./ImpCtrlStrct.K_cd_o, 'r--'); %#ok<SAGROW>
      legend_content = {legend_content{:}, 'threshold'};
      
      ylabel('tau_{obs} [Nm]');
      xlabel('t [s]');
      title(AS.JN{JI(i)}, 'interpreter', 'none'); 
      linkxaxes
    end

  end
end
end

%% Plot Electric Motor Characteristics
if plot_elecmot && isfield(Data.FromRobot, 'T_el')
  fprintf('plotting electric motor data\n');
  figure(40);clf;set(40,'Name','Elec', 'NumberTitle', 'off');
  for lr = [true, false]
    lrString = 'left';
    if lr == false
      lrString = 'right';
    end
    AS = atlas_const(atlas_version, lr);

    subplot(3,2,sprc2no(3,2,1,2-lr));hold on;grid on;
    plot(Data.FromRobot.t, Data.FromRobot.T_el(:,AS.JI_Arm(5:7)) );
    ylabel('T [deg C]');
    legend(AS.JN(AS.JI_Arm(5:7)), 'interpreter', 'none');
    if lr == true
      title('left');
    else
      title('right');
    end
    
    subplot(3,2,sprc2no(3,2,2,2-lr));hold on;grid on;
    plot(Data.FromRobot.t, Data.FromRobot.i_el(:,AS.JI_Arm(5:7)) );
    ylabel('i [A]');
    
    subplot(3,2,sprc2no(3,2,3,2-lr));hold on;grid on;
    plot(Data.FromRobot.t, Data.FromRobot.tau(:,AS.JI_Arm(5:7)) );
    ylabel('tau [Nm]');
    xlim(TLim)
  end
  linkxaxes;
  subplot_expand(40,3,2);
end

%% Plot Hydraulics data
% convert psi to Pa
psi2Pa = 6894.757293168; % Source: http://en.wikipedia.org/wiki/Pounds_per_square_inch
if plot_hydraulics && isfield(Data.FromRobot, 'p_p')
  fprintf('plotting hydraulic actuator data\n');
  for lr = [true, false]
    figoffset = 0;
    lrString = 'left';
    if lr == false
      figoffset = figoffset + 1;
      lrString = 'right';
    end
    figure(41+figoffset);clf;set(41+figoffset,'Name',sprintf('Hydr_%s_Arm',lrString) , 'NumberTitle', 'off');
    AS = atlas_const(atlas_version, lr);
    
    for i = 1:4
      % Pressure sensors
      subplot(2,4,sprc2no(2,4,1,i));hold on;grid on;
      title(AS.JN{AS.JI_Arm(i)}, 'interpreter', 'none');
      plot(Data.FromRobot.t, Data.FromRobot.p_p(:, AS.JI_Arm(i))/psi2Pa );
      plot(Data.FromRobot.t, Data.FromRobot.p_n(:, AS.JI_Arm(i))/psi2Pa );
      if i == 1
        ylabel('p [psi]');
      end
      % Inlet and supply pressure
      if i == 4 && isfield(Data.FromRobot, 'p_I')
        plot(Data.FromRobot.t, Data.FromRobot.p_I/psi2Pa );
        plot(Data.FromRobot.t, Data.FromRobot.p_S/psi2Pa );
        legend({'p_{pos}', 'p_{neg}', 'p_{I}', 'p_{S}'});
      end
    
      % calculated joint torque
      r = AS.hydr_r(AS.JI_Arm(i));
      A_1 = AS.hydr_Apos(AS.JI_Arm(i));
      A_2 = AS.hydr_Aneg(AS.JI_Arm(i));
      p_1 = Data.FromRobot.p_p(:, AS.JI_Arm(i));
      p_2 = Data.FromRobot.p_n(:, AS.JI_Arm(i));
      tau_calc = r*(A_1*p_1 - A_2*p_2);
      subplot(2,4,sprc2no(2,4,2,i));hold on;grid on;
      plot(Data.FromRobot.t, tau_calc );
      plot(Data.FromRobot.t, Data.FromRobot.tau(:,AS.JI_Arm(i)), '--' );
      if i == 1
        ylabel('\tau [Nm]');
      end
      if i == 4
        legend({'\tau_{calc}'});
      end
      
    end
  end
  linkxaxes;
end

%% Plot change of impedance controller gains

firstelement = @(x)(x(1));
  
if any(plot_impctrl)
  fprintf('plotting impedance controller data\n');
  for ich = 1:length(chains)
    if ich == 1 || ich == 3
      lr = true;
    else
      lr = false;
    end
    if ~plot_impctrl(ich)
        continue
    end
    if ~isfield(Data.ToRobot, sprintf('ImpCtrlGains_%s', chains{ich}))
      continue;
    end
    AS = atlas_const(atlas_version, lr);
    if ich == 1 || ich == 2
      names = AS.JN(AS.JI_Arm);
    else      
      names = AS.JN(AS.JI_Leg);
    end
    % remove the chain name from the joint name to access stored parameters
    names_red = names;
    for i = 1:length(names)
      tmp = names{i};
      names_red{i} = tmp(7:end);
    end
    Strct = Data.ToRobot.(sprintf('ImpCtrlGains_%s', chains{ich}));
    if isempty(Strct.t)
      continue % no data available
    end
    figure(51+ich);clf;
    set(51+ich, 'Name', sprintf('ImpCtrl_%s', chains{ich}), 'NumberTitle', 'off');
    
    % plot stiffness
    subplot(2,3,1);hold on;grid on
    for i = 1:length(names)
      stairs(Strct.t, Strct.(sprintf('K_d_%s', names_red{i})) );
    end
    ylabel('Stiffness [Nm/rad]');
    legend(names_red)
    title(sprintf('ImpCtrl_%s', chains{ich}), 'interpreter', 'none');

    % plot damping
    subplot(2,3,2);hold on;grid on
    for i = 1:length(names)
      stairs(Strct.t, Strct.(sprintf('D_%s', names_red{i})));
    end
    ylabel('Damping [1]');
    
    subplot(2,3,3);hold on;grid on
    plot_list = {'K_tau_K', 'K_tau_D', 'K_tau_g', 'K_tau_e'};
    legend_content = {};
    for i = 1:length(plot_list)
      if ~isfield(Strct, plot_list{i}), continue; end
      stairs(Strct.t, Strct.(plot_list{i}));
      legend_content = {legend_content{:}, sprintf('%s (starts %1.1f)', plot_list{i}, firstelement(Strct.(plot_list{i})))};
    end
    legend(legend_content, 'interpreter', 'none');
    
    % plot feedforward terms
    subplot(2,3,4);hold on;grid on
    plot_list = {'K_tau_c', 'K_tau_a', 'K_qDD_d', 'K_qD_d', 'K_tau_f'};
    legend_content = {};
    for i = 1:length(plot_list)
      stairs(Strct.t, Strct.(plot_list{i}));
      legend_content = {legend_content{:}, sprintf('%s (starts %1.1f)', plot_list{i}, firstelement(Strct.(plot_list{i})))};
    end
    legend(legend_content, 'interpreter', 'none');
    
    % plot observer settings
    subplot(2,3,5);hold on;grid on
    plot_list = {'K_obs_c', 'K_ext_c', 'K_fri_o', 'K_ext_o', 'K_tau_o', 'K_O'};
    legend_content = {};
    for i = 1:length(plot_list)
      stairs(Strct.t, Strct.(plot_list{i}));
      legend_content = {legend_content{:}, sprintf('%s (starts %1.1f)', plot_list{i}, firstelement(Strct.(plot_list{i})))};
    end
    legend(legend_content, 'interpreter', 'none');
    
    % integral gain
    subplot(2,3,6);hold on;grid on
    legend_content = {};
    for i = 1:length(names)
      if isfield(Strct, sprintf('KI_%s', names_red{i}))
        stairs(Strct.t, Strct.(sprintf('KI_%s', names_red{i})) );
        legend_content = {legend_content{:}, names_red{i}};
      end
    end
    legend(legend_content, 'interpreter', 'none');
    ylabel('Integral Gain K_I');
  end
  linkxaxes
end

%% Plot Impedance Controller output
% Calculate Impedance Controller Output with Simulink from Controller Input
for lr = [true, false]
  if plot_impctrl_sl(2-lr)
    AS = atlas_const(atlas_version, lr);
    figoffset = 0;
    lrString = 'left';
    if lr == false
      figoffset = figoffset + 1;
      lrString = 'right';
    end
    % Simulink-Modell mit Regler aus Eingangsdaten berechnen
    atlas_calc_impctrl_output
    
    figure(61+figoffset);clf;set(61+figoffset,'Name',sprintf('SL_ImpCtrl_%s_Arm',lrString) , 'NumberTitle', 'off');
    
    qD_Des_time_meas = interp1(Data.ToRobot.t,Data.ToRobot.qD(:,AS.JI_Arm),Data.FromRobot.t,'previous');
    dqD = Data.FromRobot.qD(:,AS.JI_Arm) - qD_Des_time_meas;
    
    % Ergebnisse plotten
    for i = 1:AS.NJA
      subplot(3,AS.NJA,sprc2no(3,AS.NJA,1,i));hold on;grid on;
      % Positionsfehler, 
      plot(Data.FromRobot.t, Data.FromRobot.Dq(:,AS.JI_Arm(i)));
      if i == 1, ylabel('\Delta q [rad]'); end
      title( AS.JN{AS.JI_Arm(i)}, 'interpreter', 'none' );
      % Geschwindigkeitsfehler
      subplot(3,AS.NJA,sprc2no(3,AS.NJA,2,i));hold on;grid on;
      plot(Data.FromRobot.t, dqD(:,i));
      if i == 1, ylabel('\Delta qD [rad/s]'); end
      % Drehmoment
      subplot(3,AS.NJA,sprc2no(3,AS.NJA,3,i));hold on;grid on;
      plot(out_t, out_tau_Soll(:,i));
      plot(Data.ToRobot.t, Data.ToRobot.tau(:,AS.JI_Arm(i)));
      if i == 1, 
        ylabel('\tau [Nm]');
        legend({'Simulink', 'ToRobot'})
      end
    end
    linkxaxes
    
    % compare observer data with saved observer data
    figure(65+figoffset);clf;set(65+figoffset,'Name',sprintf('SL_ImpCtrlObs_%s_Arm',lrString) , 'NumberTitle', 'off');
    for i = 1:AS.NJA
      subplot(3,3,i);hold on;grid on;
      title( AS.JN{AS.JI_Arm(i)}, 'interpreter', 'none' );
      ylabel('\tau_{obs} [Nm] (observer)');
      
      % observer data from experiment
      plot(Data.FromRobot.t, Data.FromRobot.tau_obs(:,AS.JI_Arm(:,i)));
      
      % observer data from simulink based on other measured data
      plot(out_t, out_tau_obs(:,i), '--');
      
      legend({'experiment', 'simulink'});
    end
    linkxaxes
    
    % only compare impedance controller torque without observer
    figure(71+figoffset);clf;set(71+figoffset,'Name',sprintf('SL_without_obs_%s_Arm',lrString) , 'NumberTitle', 'off');
    for i = 1:AS.NJA
      subplot(3,AS.NJA,sprc2no(3,AS.NJA,1,i));hold on;grid on;
      % Positionsfehler, 
      plot(Data.FromRobot.t, Data.FromRobot.Dq(:,AS.JI_Arm(i)));
      if i == 1, ylabel('\Delta q [rad]'); end
      title( AS.JN{AS.JI_Arm(i)}, 'interpreter', 'none' );
      % Geschwindigkeitsfehler
      subplot(3,AS.NJA,sprc2no(3,AS.NJA,2,i));hold on;grid on;
      plot(Data.FromRobot.t, dqD(:,i));
      if i == 1, ylabel('\Delta qD [rad/s]'); end
      % Drehmoment
      subplot(3,AS.NJA,sprc2no(3,AS.NJA,3,i));hold on;grid on;
      plot(out_t, out_tau_Soll_wo_obs(:,i));
      plot(Data.ToRobot.t, Data.ToRobot.tau(:,AS.JI_Arm(i)));
      if i == 1, 
        ylabel('\tau [Nm] (w/o observer)');
        legend({'Simulink', 'ToRobot'})
      end
    end
    linkxaxes
  end
end

%% Plot Stiffness
% Calculate cartesian stiffness
for lr = [true, false]
  if lr == true
    F_mess = Data.FromRobot.F_lH;
  else
    F_mess = Data.FromRobot.F_rH;
  end
  % Sensoren driften ziemlich stark. Ersten Datenwert als Referenzmessung
  F_mess = F_mess - repmat(F_mess(1,:), size(F_mess,1),1);
  
  
  AS = atlas_const(atlas_version, lr);
  if plot_stiffness(2-lr)
    % calculate desired cartesian stiffness
    Strct = Data.ToRobot.(sprintf('ImpCtrlGains_%s', chains{2-lr}));
    
    K_d = diag([Strct.K_d_shz(1); Strct.K_d_shx(1); Strct.K_d_ely(1); Strct.K_d_elx(1); Strct.K_d_wry(1); Strct.K_d_wrx(1); Strct.K_d_wry2(1)]);
    Kx_theo = NaN(length(Data.FromRobot.t), 3);
    for i = 1:length(Data.FromRobot.t)
      J = atlas_arm_jacobig_mex(Data.FromRobot.q(i, AS.JI_Arm), ...
        lr, atlas_version);
      % cartesian stiffness
      % [1], S. 59
      Kx = inv(J * (K_d \ (J')));
      Kx_theo(i,:) = Kx(sub2ind(size(Kx), [1 2 3], [1 2 3]));
    end
    
    % calculate experimental stiffness
    Kx_exp = NaN(length(Data.FromRobot.t), 3);
    q_Des_time_meas = interp1(Data.ToRobot.t,Data.ToRobot.q(:,AS.JI_Arm),Data.FromRobot.t,'previous');
    F_0 = NaN(length(Data.FromRobot.t), 3);
    Dx = NaN(length(Data.FromRobot.t), 3);
    for i = 1:length(Data.FromRobot.t)
      T_0_EE = atlas_arm_fkine_mex(Data.FromRobot.q(i, AS.JI_Arm), ...
        lr, atlas_version);
      x_EE = tr2quat_mex(T_0_EE(:,:,end));

      T_0_EE_d = atlas_arm_fkine_mex(q_Des_time_meas(i, :), ...
        lr, atlas_version);
      x_EE_d = tr2quat_mex(T_0_EE_d(:,:,end));
      Dx(i,:) = x_EE(1:3) - x_EE_d(1:3);
      F_0(i,:) = t2r(T_0_EE(:,:,end))*(F_mess(i,1:3))';
      % TODO: Hier wurden keine Kopplungen zwischen den Achsen
      % berücksichtigt.
      % Die somit experimentell bestimmte Steifigkeit ist nicht der
      % korrekte Eintrag in der Steifigkeitsmatrix!
      Kx_exp(i,:) = F_0(i,1:3) ./ (Dx(i,:));
    end
    
    xyz = {'x', 'y', 'z'};
    figure(73+figoffset);clf;set(73+figoffset,'Name',sprintf('CartStiff_%s_Arm',lrString) , 'NumberTitle', 'off');
    for i = 1:3
      subplot(3,3,sprc2no(3,3,i,1));hold all; grid on;
      plot(Data.FromRobot.t, 1e3*Dx(:,i));
      ylabel(sprintf('\\Delta %s [mm]', xyz{i}));
      
      subplot(3,3,sprc2no(3,3,i,2));hold all; grid on;
      plot(Data.FromRobot.t, F_0(:,i));
      ylabel(sprintf('F_{%s} [N]', xyz{i}));
      
      subplot(3,3,sprc2no(3,3,i,3));hold all; grid on;
      plot(Data.FromRobot.t, Kx_exp(:,i)/1e3);
      plot(Data.FromRobot.t, Kx_theo(:,i)/1e3);
      legend({'Mess', 'Soll.'});

      ylabel(sprintf('K_{%s%s} [N/mm]', xyz{i}, xyz{i}));
      % ylim(minmax2(Kx_theo(:,i)'));
    end
    linkxaxes
    
    %% Joint Stiffness
    figure(75+figoffset);clf;set(75+figoffset,'Name',sprintf('JointStiff_%s_Arm',lrString) , 'NumberTitle', 'off');
    for i = 1:AS.NJA
      subplot(3,7,sprc2no(3,7,1,i));hold all; grid on;
      plot(Data.FromRobot.t, Data.FromRobot.Dq(:,AS.JI_Arm(i)));
      ylabel(sprintf('\\Delta q_%d [rad]', i));
      
      subplot(3,7,sprc2no(3,7,2,i));hold all; grid on;
      plot(Data.FromRobot.t, Data.FromRobot.tau_ext(:,AS.JI_Arm(i)));
      ylabel(sprintf('\\tau_{ext,%d} [Nm]', i));
      
      subplot(3,7,sprc2no(3,7,3,i));hold all; grid on;
      Kq_i_exp = Data.FromRobot.tau_ext(:,AS.JI_Arm(i)) ./ Data.FromRobot.Dq(:,AS.JI_Arm(i));
      plot(Data.FromRobot.t, Kq_i_exp);
      plot(minmax2(Data.FromRobot.t'), K_d(i,i)*[1,1]);
      legend({'Mess', 'Soll.'});

      ylabel(sprintf('K_{q%d} [Nm/rad]',i));
      % ylim(minmax2(Kx_theo(:,i)'));
    end
    linkxaxes
    
    
  end
end
%% Plot Robot
if plot_robot
figure(81);clf;set(81, 'Name', 'Rob_t_end', 'NumberTitle', 'off');
hold on; grid on;axis equal;view(3);set(81, 'Renderer','OpenGL')
I = 1;
q = Data.FromRobot.q(I,:);
T_pelvis = quat2tr([0,0,0,Data.FromRobot.pelvis_x_r(I,:)]);
atlas_plot_robot(q, atlas_version, 81, T_pelvis)
atlas_plot_frames(q, [AS.LI_rArm, AS.LI_lArm], atlas_version, T_pelvis);
%subtitle(filename);

figure(80);clf;set(80, 'Name', 'Rob_t0', 'NumberTitle', 'off');
hold on; grid on;axis equal;view(3);set(80, 'Renderer','OpenGL')
I = size(Data.FromRobot.t,1)-1;
q = Data.FromRobot.q(I,:);
T_pelvis = quat2tr([0,0,0,Data.FromRobot.pelvis_x_r(I,:)]);
atlas_plot_robot(q, atlas_version, 80, T_pelvis)
atlas_plot_frames(q, [AS.LI_rArm, AS.LI_lArm], atlas_version, T_pelvis);
%subtitle(filename);


for ff = 5:6
  figure(ff);
  % Insert Floor  
  p0 = [0 0 0];
  v1 = [1 0 0];
  v2 = [0 1 0];
  plane = [p0 v1 v2];

  drawPlane3d(plane, 'FaceColor', [0.6,0.6,0.6], 'FaceAlpha', 0.3)
  
  % change view
  %view([90 0]) % YZ-View
  %view([0 90]) % XY-View
  view([0 0]) % XZ-View
end
end
%% Save Figures
dockall
if save_plot
eval_path = [eval_path, sprintf('_eval_%s', datestr(now, 'yyyymmdd_HHMM'))]; %#ok<AGROW>
if ~exist(eval_path, 'file')
  mkdir(eval_path);
end
if strcmp(mode, 'single')
  [~,basename] = fileparts(filename);
else
  basename = 'evaluation_compilation';
end
tic;
Bilder_Speichern(eval_path, {'fig', 'png'}, basename);
fprintf('Saved Figures to %s. Time: %1.1fs.\n', eval_path, toc);
end
end