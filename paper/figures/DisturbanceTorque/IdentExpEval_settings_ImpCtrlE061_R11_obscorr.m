% Evaluate E061_R11
% Check Forces, observed joint torque and other values
% Beobachter einmal mit und einmal ohne Berücksichtigung der externen
% Kräfte berechnen.
% Dazu werden die Messwerte in einem temporären Ordner kopiert und die
% Reglerparameter verändert.


% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-08
% (c) Institut für Regelungstechnik, Universität Hannover

%% Init

clear
close all

% Version of the Atlas Robot in this experiment
SettingsStruct = struct('atlas_version', uint8(5));

% directory which contains the measurement data. 
tb_path = fileparts(which('drc_paper_path_init.m')); 

% Mode:
% 'single' mode: Only plot data for one file
% 'all' mode: Append all the data of the selected files and plot them all together (too much compuatational effort!)
SettingsStruct.mode = 'single';

% Number of files to append in mode "all"
SettingsStruct.file_number_range = 1;

% Number of data points for which the joint torque shall be calculated (Inf means for all points, 1000 will calculate for the first second of data (@1kHz))
SettingsStruct.N_calc = 0; % calculate all torques

% Plot all joint positions, velocities, efforts
SettingsStruct.plot_q_all = false;
SettingsStruct.plot_qD_all = false;
SettingsStruct.plot_tau_all = false;
% plot sensor data
SettingsStruct.plot_FT = true;
SettingsStruct.plot_IMU = false;

% Plot joint positions, velocities, accelerations, efforts for the arms and legs
% order: left, right arm, left/right leg
SettingsStruct.plot_q_limb = [false, false, false, false];
SettingsStruct.plot_qD_limb = [false, false, false, false];
SettingsStruct.plot_qDD_limb = [false, false, false, false];
SettingsStruct.plot_tau_limb = [true, false, false, false];

% Plot the electric motor data
SettingsStruct.plot_elecmot = false;

% Plot the hydraulics data
SettingsStruct.plot_hydraulics = false;

% Plot raw actuator data
SettingsStruct.plot_actuator_raw = false;

% Plot actuator state data
SettingsStruct.plot_actuator_state = false;

% Plot the Robot in Start and End configuration
SettingsStruct.plot_robot = false;

% Plot impedance controller settings
SettingsStruct.plot_impctrl = [false, false, false, false];

% Plot impedance controller output comparison using simulink
SettingsStruct.plot_impctrl_sl = [true, false];

% plot joint error and plot cartesian error
SettingsStruct.plot_arm_error = [false, false];

SettingsStruct.plot_tau_ext = true;
SettingsStruct.calib_Fext = 'ImpCtrl';

% collision detection
SettingsStruct.plot_colldet = [false, false];

% save the figures to the directory in exp_subfolder
SettingsStruct.save_plot = false;

SettingsStruct.plot_stiffness = [false, false];

% Set used MPV
MPV_Liste = MPV_list();
SettingsStruct.MPV_lArm_damp = MPV_Liste(9).MPV;
fprintf('Using %s for left Arm damping calculations.\n', MPV_Liste(9).Name);
SettingsStruct.MPV_lArm_model = MPV_Liste(5).MPV;
fprintf('Using %s for left Arm dynamics calculations.\n', MPV_Liste(5).Name);
SettingsStruct.MPV_rArm_damp = MPV_Liste(10).MPV;
fprintf('Using %s for right Arm damping calculations.\n', MPV_Liste(10).Name);
SettingsStruct.MPV_rArm_model = MPV_Liste(18).MPV;
fprintf('Using %s for right Arm dynamics calculations.\n', MPV_Liste(18).Name);

SettingsStruct.eval_path = fullfile(tb_path, 'experiments', 'eval_atlas5', ...
  'ImpCtrlv5_E061_R11_BeoVgl');


%% Daten modifizieren

% originale Messdaten
source_file = fullfile(tb_path, 'experiments', 'atlas5_ImpCtrl', ...
  'ImpCtrlv5_E061_R11_20150730_extforce_test', 'ImpCtrlv5_E061_R11_log_0.mat');

% Kraftsensor-Offset entfernen
target_file_1 = fullfile(tb_path, 'experiments', 'atlas5_test', ...
'ImpCtrlv5_E061_R11_ChangeFT', 'ImpCtrlv5_E061_R11_log_0_bearb_FT.mat');

if ~exist(target_file_1, 'file')
  mkdirs(fileparts(target_file_1));
  copyfile(source_file, target_file_1);
  tmp = load_experiment_data(target_file_1);
  FromRobot = tmp.FromRobot;
  ToRobot = tmp.ToRobot;
  % Im Experiment war K_ext_o ohne Auswirkungen auf -1. Für Berechnung des
  % Beobachters ohne Kraftberücksichtigung muss hier eine 0 stehen!
  ToRobot.ImpCtrlGains_left_arm.K_ext_o = zeros(3,1);
  % graphisch ermittelten Offset abziehen
  FromRobot.F_lH = FromRobot.F_lH_raw - repmat([0.6, 27, -0.6, 0.13, -0.7, 0.13], length(FromRobot.t), 1);
  save(target_file_1, 'FromRobot', 'ToRobot')
end

% zusätzlich Parameter ändern
target_file_2 = fullfile(tb_path, 'experiments', 'atlas5_test', ...
'ImpCtrlv5_E061_R11_ChangeImpSetting', 'ImpCtrlv5_E061_R11_log_0_bearb_FT_ImpSett.mat');
if ~exist(target_file_2, 'file')
  mkdirs(fileparts(target_file_2));
  copyfile(target_file_1, target_file_2);
  tmp = load_experiment_data(target_file_2);
  FromRobot = tmp.FromRobot;
  ToRobot = tmp.ToRobot;
  ToRobot.ImpCtrlGains_left_arm.K_ext_o = ones(4,1);
  save(target_file_2, 'FromRobot', 'ToRobot')
end

%% Auswertung 1: Simulink mit voreingestellten Parametern
SettingsStruct.data_path = fileparts(target_file_1);

% Start the Evaluation
atlas_identification_experiment_test

% save figures: Keine Berücksichtigung der externen Kraft im Beobachter
eval_path = fullfile(tb_path, 'experiments', 'eval_atlas5', 'ImpCtrlv5_E061');
mkdirs(eval_path);

saveas(65, fullfile(eval_path, 'R11_Obs_ohne_ext_Beo.fig'));
saveas(25, fullfile(eval_path, 'R11_torque_Zeroed_FT.fig'));
%% Auswertung 2: Simulink mit geänderten Parametern
SettingsStruct.data_path = fileparts(target_file_2);

% Start the Evaluation
atlas_identification_experiment_test
eval_path = fullfile(tb_path, 'experiments', 'eval_atlas5', 'ImpCtrlv5_E061');
saveas(65, fullfile(eval_path, 'R11_Obs_mit_ext_Beo.fig'));