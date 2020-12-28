% Start the Atlas Arm (v5) joint impctrl
% Test the Model holding a position and exerting external forces

% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-09
% (c) Institut für Regelungstechnik, Universität Hannover

clear
clc
close all
tb_path = fileparts(which('drc_paper_path_init.m')); 

%% Init
atlas_version = uint8(5);
lr = true;
atlas_limb = 1;
atlas_joint_impctrl_sqrt_damping_test_settings_default

%% Startpose laden
% siehe trajectories/ImpCtrlExperiments_poses/ImpCtrlExperiments_poses.m
q0 = [0, 0, pi/2, pi/2, 0, 0, 0];
qD0 = zeros(1,AS.NJA);

%% Trajektorie
qsollDaten.q = q0;
qsollDaten.qD = zeros(1,AS.NJA);
qsollDaten.qDD = zeros(1,AS.NJA);
qsollDaten.t = 0;

%% Externe Kraft

% stetiger Kraftverlauf (nicht gewünscht)
% F_start = 0;
% F_end = 100;
% % C3-stetige Trapez-Trajektorie fahren
% zmax = [ F_end ; F_end/1e-1; F_end/1e-3; F_end/1e-4];
% [ew_t, ew_z, w_z, w_t] = Trapez_nAbl([F_start;0;0], [F_end;0;0], 0, zmax, 1e-3, false);
% 
% t_Fext = [w_t;          w_t(end)+3;   w_t(end)+3+w_t];
% F_ext =  [w_z(1:end,1); w_z(end,1);   flipud(w_z(1:end,1))];

t_Fext = [0;1;4];
F_ext =  [[0;50;0], zeros(3,5)];
%% Simulink-Eingangsdaten
simin_q_soll = struct('time', qsollDaten.t, ...
    'signals', struct('values', qsollDaten.q, 'dimensions', NJL), ...
    'name', 'q_soll');

simin_qD_soll = struct('time', qsollDaten.t, ...
    'signals', struct('values', qsollDaten.qD, 'dimensions', NJL), ...
    'name', 'qD_soll');
  
simin_qDD_soll = struct('time', qsollDaten.t, ...
    'signals', struct('values', qsollDaten.qDD, 'dimensions', NJL), ...
    'name', 'qD_soll');
  
simin_F0_ext = struct('time', t_Fext, ...
    'signals', struct('values', F_ext, 'dimensions', 6), ...
    'name', 'F0_ext');
rng(0); % seed for reproducable results
mkdirs(fullfile(tb_path, 'simulink', 'results', 'MdlNoise'));
mkdirs(fullfile(tb_path, 'simulink', 'results', 'MesNoise'));

%% generate reference data
[a_mdh, d_mdh, ~, ~, rSges_mdh_ORIG, m_ORIG,Iges_mdh_ORIG] = atlas_arm_parameter_mdh(atlas_arm_lr, atlas_version);
MPV_ORIG = atlas_arm_convert_par1_MPV(rSges_mdh_ORIG, m_ORIG, Iges_mdh_ORIG, a_mdh, d_mdh, atlas_version);
% MPV_arm_plant=MPV_ORIG; % MPV for simulated plant
MPV_arm_plant=MPV_ORIG; % MPV for calculating the gravity model
% apply noise to identified parameters
model_noise_ratio = 0;
rSges_mdh_NOISE = rSges_mdh_ORIG.*(1-0.5*model_noise_ratio+model_noise_ratio*rand(size(rSges_mdh_ORIG)));
m_NOISE = m_ORIG.*(1-0.5*model_noise_ratio+model_noise_ratio*rand(size(m_ORIG)));
Iges_mdh_NOISE = Iges_mdh_ORIG.*(1-0.5*model_noise_ratio+model_noise_ratio*rand(size(Iges_mdh_ORIG)));
% calculate MPV with noisy data
MPV_arm_model=atlas_arm_convert_par1_MPV(rSges_mdh_NOISE, m_NOISE, Iges_mdh_NOISE, a_mdh, d_mdh, atlas_version); % MPV for calculating the gravity model
MPV_arm_damp=MPV_arm_model; % MPV for calculating the inertia matrix (positive definite)
% set noise levels for simulated measurements
q_noise     = 3.4e-4;
qD_noise    = 5.5e-2;
tau_noise   = 7.3e-2;
F_ext_noise = 2.3;
g_noise     = 2e-4;
% set noise seeds for measurement noises
q_noise_seed     = 0;
qD_noise_seed    = 0;
tau_noise_seed   = 0;
F_ext_noise_seed = 0;
g_noise_seed     = 0;
% set noise switches for measurement noises (1=on, 0=off)
q_noise_switch     = 0;
qD_noise_switch    = 0;
tau_noise_switch   = 0;
F_ext_noise_switch = 0;
g_noise_switch     = 0;

%friction parameters from identification
d   = [1.30 0.90 1.60 2.70 0.50 0.20 0.20]'; % identified values 0.2*ones(NJL,1);
muC = [2.00 6.70 10.3 6.10 0.10 0.10 3.10]'; % identified values 0.1*ones(NJL,1);

% controller parameters and gains
K_d     = 150*eye(NJL);
D       = 0.3*eye(NJL);
K_O     = 10*eye(NJL);
T       = 0.001;
T1      = 0;
K_tau_K = 1;
K_tau_g = 1;
K_tau_c = 1;
K_tau_D = 1;
K_tau_a = 1;
K_tau_f = 1;
K_tau_e = 0;
K_tau_o = 0; % Beobachter-Kompensation aus
K_ext_c = 0;
K_obs_c = 0;
K_fri_o = K_tau_f;
K_ext_o = 1;
K_qD_d  = 1;
K_qDD_d = 1;
tau_c   = muC.*(1-0.5*model_noise_ratio+model_noise_ratio*rand(size(muC)));% - 4*ones(NJL,1);
B       = d.*(1-0.5*model_noise_ratio+model_noise_ratio*rand(size(d)));% - 0.1*ones(NJL,1);
qD_th   = 0.05*ones(NJL,1);
tau_th  = 0.02*ones(NJL,1);

%% Configure Model

load_system(sl_Modellname)
configSet = getActiveConfigSet(sl_Modellname);
set_param(configSet, 'Solver', 'ode4');
set_param(configSet, 'FixedStep', '1e-4');

%% Define Inputs
simin_tau_ext = struct('time', 0, ...
    'signals', struct('values', zeros(1,NJL), 'dimensions', NJL), ...
    'name', 'tau_ext');
simin_tau_m = struct('time', 0, ...
    'signals', struct('values', zeros(1,NJL), 'dimensions', NJL), ...
    'name', 'tau_m');

%% Start Simulation with different settings
t1 = tic;
simOut = sim(sl_Modellname, 'StopTime', '10', ...
  'SimulationMode', 'rapid'); % normal
clear sl
sl = get_simulink_outputs(simOut, sl_Modellname);
fprintf('Simulink-Modell berechnet. Rechenzeit: %1.1fs\n', toc(t1));
filename = fullfile(tb_path, 'simulink', 'results', 'MdlNoise', sprintf('atlas5_impctrl_obs_FextO_MdlNoise_ref.mat'));
save(filename, 'sl');
fprintf('Simulationsergebnisse nach %s gespeichert.\n', filename);

%% noise level comparison
for noise_level=0:0.01:0.3
  for j=1:10
    [a_mdh, d_mdh, ~, ~, rSges_mdh_ORIG, m_ORIG,Iges_mdh_ORIG] = atlas_arm_parameter_mdh(atlas_arm_lr, atlas_version);
    MPV_ORIG = atlas_arm_convert_par1_MPV(rSges_mdh_ORIG, m_ORIG, Iges_mdh_ORIG, a_mdh, d_mdh, atlas_version);
    % MPV_arm_plant=MPV_ORIG; % MPV for simulated plant
    MPV_arm_plant=MPV_ORIG; % MPV for calculating the gravity model
    % apply noise to identified parameters
    model_noise_ratio = noise_level;
    rSges_mdh_NOISE = rSges_mdh_ORIG.*(1-0.5*model_noise_ratio+model_noise_ratio*rand(size(rSges_mdh_ORIG)));
    m_NOISE = m_ORIG.*(1-0.5*model_noise_ratio+model_noise_ratio*rand(size(m_ORIG)));
    Iges_mdh_NOISE = Iges_mdh_ORIG.*(1-0.5*model_noise_ratio+model_noise_ratio*rand(size(Iges_mdh_ORIG)));
    % calculate MPV with noisy data
    MPV_arm_model=atlas_arm_convert_par1_MPV(rSges_mdh_NOISE, m_NOISE, Iges_mdh_NOISE, a_mdh, d_mdh, atlas_version); % MPV for calculating the gravity model
    MPV_arm_damp=MPV_arm_model; % MPV for calculating the inertia matrix (positive definite)
    % set noise levels for simulated measurements
    q_noise     = 3.4e-4;
    qD_noise    = 5.5e-2;
    tau_noise   = 7.3e-2;
    F_ext_noise = 2.3;
    g_noise     = 2e-4;
    % set noise seeds for measurement noises
    q_noise_seed     = 0;
    qD_noise_seed    = 0;
    tau_noise_seed   = 0;
    F_ext_noise_seed = 0;
    g_noise_seed     = 0;
    % set noise switches for measurement noises (1=on, 0=off)
    q_noise_switch     = 1;
    qD_noise_switch    = 1;
    tau_noise_switch   = 1;
    F_ext_noise_switch = 1;
    g_noise_switch     = 1;

    %friction parameters from identification
    d   = [1.30 0.90 1.60 2.70 0.50 0.20 0.20]'; % identified values 0.2*ones(NJL,1);
    muC = [2.00 6.70 10.3 6.10 0.10 0.10 3.10]'; % identified values 0.1*ones(NJL,1);

    % controller parameters and gains
    K_d     = 150*eye(NJL);
    D       = 0.3*eye(NJL);
    K_O     = 10*eye(NJL);
    T       = 0.001;
    T1      = 0;
    K_tau_K = 1;
    K_tau_g = 1;
    K_tau_c = 1;
    K_tau_D = 1;
    K_tau_a = 1;
    K_tau_f = 1;
    K_tau_e = 0;
    K_tau_o = 1; % Beobachter-Kompensation an
    K_ext_c = 0;
    K_obs_c = 0;
    K_fri_o = K_tau_f;
%     K_ext_o = 1;
    K_qD_d  = 1;
    K_qDD_d = 1;
    tau_c   = muC.*(1-0.5*model_noise_ratio+model_noise_ratio*rand(size(muC)));% - 4*ones(NJL,1);
    B       = d.*(1-0.5*model_noise_ratio+model_noise_ratio*rand(size(d)));% - 0.1*ones(NJL,1);
    qD_th   = 0.05*ones(NJL,1);
    tau_th  = 0.02*ones(NJL,1);
    
    for K_ext_o = 0:1
      %% Configure Model

      load_system(sl_Modellname)
      configSet = getActiveConfigSet(sl_Modellname);
      set_param(configSet, 'Solver', 'ode4');
      set_param(configSet, 'FixedStep', '1e-4');

      %% Define Inputs
      simin_tau_ext = struct('time', 0, ...
          'signals', struct('values', zeros(1,NJL), 'dimensions', NJL), ...
          'name', 'tau_ext');
      simin_tau_m = struct('time', 0, ...
          'signals', struct('values', zeros(1,NJL), 'dimensions', NJL), ...
          'name', 'tau_m');

      %% Start Simulation with different settings
      t1 = tic;
      simOut = sim(sl_Modellname, 'StopTime', '10', ...
        'SimulationMode', 'rapid'); % normal
      clear sl
      sl = get_simulink_outputs(simOut, sl_Modellname);
      fprintf('Simulink-Modell berechnet. Rechenzeit: %1.1fs\n', toc(t1));
      filename = fullfile(tb_path, 'simulink', 'results', 'MdlNoise', sprintf('atlas5_impctrl_obs_FextO_MdlNoise%.2f_Kexto%d_%d.mat',noise_level, K_ext_o, j));
      save(filename, 'sl');
      fprintf('Simulationsergebnisse nach %s gespeichert.\n', filename);
    end
  end
end

%% measurement noise comparison 
[a_mdh, d_mdh, ~, ~, rSges_mdh_ORIG, m_ORIG,Iges_mdh_ORIG] = atlas_arm_parameter_mdh(atlas_arm_lr, atlas_version);
MPV_ORIG = atlas_arm_convert_par1_MPV(rSges_mdh_ORIG, m_ORIG, Iges_mdh_ORIG, a_mdh, d_mdh, atlas_version);
% MPV_arm_plant=MPV_ORIG; % MPV for simulated plant
MPV_arm_plant=MPV_ORIG; % MPV for calculating the gravity model
% apply noise to identified parameters
model_noise_ratio = 0.1;
rSges_mdh_NOISE = rSges_mdh_ORIG.*(1-0.5*model_noise_ratio+model_noise_ratio*rand(size(rSges_mdh_ORIG)));
m_NOISE = m_ORIG.*(1-0.5*model_noise_ratio+model_noise_ratio*rand(size(m_ORIG)));
Iges_mdh_NOISE = Iges_mdh_ORIG.*(1-0.5*model_noise_ratio+model_noise_ratio*rand(size(Iges_mdh_ORIG)));
% calculate MPV with noisy data
MPV_arm_model=atlas_arm_convert_par1_MPV(rSges_mdh_NOISE, m_NOISE, Iges_mdh_NOISE, a_mdh, d_mdh, atlas_version); % MPV for calculating the gravity model
MPV_arm_damp=MPV_arm_model; % MPV for calculating the inertia matrix (positive definite)
for noise_level=1:100
  % set noise levels for simulated measurements
  q_noise     = 3.4e-4*noise_level;
  qD_noise    = 5.5e-2*noise_level;
  tau_noise   = 7.3e-2*noise_level;
  F_ext_noise = 2.3*noise_level;
  g_noise     = 2e-4*noise_level;
  % set noise seeds for measurement noises
  q_noise_seed     = 0;
  qD_noise_seed    = 0;
  tau_noise_seed   = 0;
  F_ext_noise_seed = 0;
  g_noise_seed     = 0;
  % set noise switches for measurement noises (1=on, 0=off)
  q_noise_switch     = 1;
  qD_noise_switch    = 1;
  tau_noise_switch   = 1;
  F_ext_noise_switch = 1;
  g_noise_switch     = 1;

  %friction parameters from identification
  d   = [1.30 0.90 1.60 2.70 0.50 0.20 0.20]'; % identified values 0.2*ones(NJL,1);
  muC = [2.00 6.70 10.3 6.10 0.10 0.10 3.10]'; % identified values 0.1*ones(NJL,1);

  % controller parameters and gains
  K_d     = 150*eye(NJL);
  D       = 0.3*eye(NJL);
  K_O     = 10*eye(NJL);
  T       = 0.001;
  T1      = 0;
  K_tau_K = 1;
  K_tau_g = 1;
  K_tau_c = 1;
  K_tau_D = 1;
  K_tau_a = 1;
  K_tau_f = 1;
  K_tau_e = 0;
  K_tau_o = 1; % Beobachter-Kompensation an
  K_ext_c = 0;
  K_obs_c = 0;
  K_fri_o = K_tau_f;
%   K_ext_o = 1;
  K_qD_d  = 1;
  K_qDD_d = 1;
  tau_c   = muC.*(1-0.5*model_noise_ratio+model_noise_ratio*rand(size(muC)));% - 4*ones(NJL,1);
  B       = d.*(1-0.5*model_noise_ratio+model_noise_ratio*rand(size(d)));% - 0.1*ones(NJL,1);
  qD_th   = 0.05*ones(NJL,1);
  tau_th  = 0.02*ones(NJL,1);

  for K_ext_o=0:1
    %% Configure Model

    load_system(sl_Modellname)
    configSet = getActiveConfigSet(sl_Modellname);
    set_param(configSet, 'Solver', 'ode4');
    set_param(configSet, 'FixedStep', '1e-4');

    %% Define Inputs
    simin_tau_ext = struct('time', 0, ...
        'signals', struct('values', zeros(1,NJL), 'dimensions', NJL), ...
        'name', 'tau_ext');
    simin_tau_m = struct('time', 0, ...
        'signals', struct('values', zeros(1,NJL), 'dimensions', NJL), ...
        'name', 'tau_m');

    %% Start Simulation with different settings
    t1 = tic;
    simOut = sim(sl_Modellname, 'StopTime', '10', ...
      'SimulationMode', 'rapid'); % normal
    clear sl
    sl = get_simulink_outputs(simOut, sl_Modellname);
    fprintf('Simulink-Modell berechnet. Rechenzeit: %1.1fs\n', toc(t1));
    filename = fullfile(tb_path, 'simulink', 'results', 'MesNoise', sprintf('atlas5_impctrl_obs_FextO_MesNoise_Kexto%d_%d.mat', K_ext_o, noise_level));
    save(filename, 'sl');
    fprintf('Simulationsergebnisse nach %s gespeichert.\n', filename);
  end
end

atlas_joint_impctrl_sqrt_damping_test_extforce_noise_plot