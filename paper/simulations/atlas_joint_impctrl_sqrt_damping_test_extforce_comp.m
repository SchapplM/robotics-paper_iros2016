% Start the Atlas Arm (v5) joint impctrl
% Test the Model holding a position and exerting external forces
% 
% Erstellt Bild 12 aus [VorndammeSchToeHad2016]

% Quellen
% [VorndammeSchToeHad2016] Vorndamme, Schappler, Tödtheide,  Haddadin: 
% Soft Robotics for the Hydraulic {A}tlas Arms: Joint Impedance Control
% with Collision Detection and Disturbance Compensation (IROS 2016)

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

noise_level=1;  
  
[a_mdh, d_mdh, ~, ~, rSges_mdh_ORIG, m_ORIG,Iges_mdh_ORIG] = atlas_arm_parameter_mdh(atlas_arm_lr, atlas_version);
MPV_ORIG = atlas_arm_convert_par1_MPV(rSges_mdh_ORIG, m_ORIG, Iges_mdh_ORIG, a_mdh, d_mdh, atlas_version);
% MPV_arm_plant=MPV_ORIG; % MPV for simulated plant
MPV_arm_plant=MPV_ORIG; % MPV for calculating the gravity model
% apply noise to identified parameters
model_noise_ratio = 0.1;
rng(0); % seed for reproducable results
rSges_mdh_NOISE = rSges_mdh_ORIG.*(1-0.5*model_noise_ratio+model_noise_ratio*rand(size(rSges_mdh_ORIG)));
m_NOISE = m_ORIG.*(1-0.5*model_noise_ratio+model_noise_ratio*rand(size(m_ORIG)));
Iges_mdh_NOISE = Iges_mdh_ORIG.*(1-0.5*model_noise_ratio+model_noise_ratio*rand(size(Iges_mdh_ORIG)));
% calculate MPV with noisy data
MPV_arm_model=atlas_arm_convert_par1_MPV(rSges_mdh_NOISE, m_NOISE, Iges_mdh_NOISE, a_mdh, d_mdh, atlas_version); % MPV for calculating the gravity model
MPV_arm_damp=MPV_arm_model; % MPV for calculating the inertia matrix (positive definite)
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
K_tau_f = 0;
K_tau_e = 0;
K_tau_o = 1; % Beobachter-Kompensation an
K_ext_c = 0;
K_obs_c = 0;
K_fri_o = K_tau_f;
K_ext_o = 0;
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
FileList = {};
K_ext_o_Settings = [0, 1, 0];
K_tau_o_Settings = [1, 1, 0];
DescList = {};
for ik = 1:length(K_ext_o_Settings)
  K_ext_o = K_ext_o_Settings(ik);
  K_tau_o = K_tau_o_Settings(ik);
  t1 = tic;
  simOut = sim(sl_Modellname, 'StopTime', '10', ...
    'SimulationMode', 'rapid'); % normal
  clear sl
  sl = get_simulink_outputs(simOut, sl_Modellname);
  fprintf('Simulink-Modell berechnet. Rechenzeit: %1.1fs\n', toc(t1));
  FileList{ik} = fullfile(tb_path, 'simulink', 'results', sprintf('atlas5_impctrl_obs_FextO_varobs_%d.mat', ik));
  save(FileList{ik}, 'sl');
  fprintf('Simulationsergebnisse nach %s gespeichert.\n', FileList{ik});
  DescList{ik} = sprintf('Kexto=%d, Ktauo=%d', K_ext_o_Settings(ik), K_tau_o_Settings(ik));
end

%% Plot comparison
PlotSettingsStruct = struct('atlas_version', atlas_version, 'I', AS.JI_lArm);
PlotSettingsStruct.Files = FileList;
PlotSettingsStruct.Names = DescList;
atlas_plot_sl_compare

%% Format plots
dockall

%% Neues Bild generieren
Erg1 = load(FileList{1}, 'sl');
sl1 = Erg1.sl;
Erg2 = load(FileList{2}, 'sl');
sl2 = Erg2.sl;
Erg3 = load(FileList{3}, 'sl');
sl3 = Erg3.sl;

figure(99);clf;
axhdl = NaN(3,1);
axhdl(1) = subplot(3,1,1);hold on;grid on;
linhdl = NaN(3,1);
linhdl(1) = plot(sl1.t-0.5, sl1.q(:,1));
linhdl(2) = plot(sl2.t-0.5, sl2.q(:,1));
linhdl(3) = plot(sl3.t-0.5, sl3.q(:,1));
leghdl = line_format_publication(linhdl, {}, {'ext0', 'ext1', 'tauobs0'});
ylabel('$q_1$ [rad]', 'interpreter', 'latex');

axhdl(2) = subplot(3,1,2);hold on;grid on;
linhdl = NaN(3,1);
linhdl(1) = plot(sl1.t-0.5, sl1.tau_ext(:,1));
linhdl(2) = plot(sl2.t-0.5, sl2.tau_ext(:,1));
linhdl(3) = plot(sl3.t-0.5, sl3.tau_ext(:,1));
leghdl = line_format_publication(linhdl, {}, {'ext0', 'ext1', 'tauobs0'});
ylabel('$\tau_{\mathrm{ext},1}$ [Nm]', 'interpreter', 'latex');

axhdl(3) = subplot(3,1,3);hold on;grid on;
linhdl = NaN(3,1);
linhdl(1) = plot(sl1.t-0.5, -sl1.tau_obs(:,1));
linhdl(2) = plot(sl2.t-0.5, -sl2.tau_obs(:,1));
linhdl(3) = plot(sl3.t-0.5, -sl3.tau_obs(:,1));
leghdl = line_format_publication(linhdl, {}, {'ext0', 'ext1', 'tauobs0'});
ylabel('$\hat{\tau}_{\varepsilon,1}$ [Nm]', 'interpreter', 'latex');
xlabel('$t$ [s]', 'interpreter', 'latex');

figure(99);
linkxaxes;
set(axhdl,'xlim',[0 4.5]);

set_y_autoscale(99,0.05)

subplot_expand(99, 3, 1);

% Gesamt-Formatierung
figure_format_publication(axhdl)
set_size_plot_subplot(99, ...
  8.67, 6, ...
  [3 1], ...
  0.13, 0.02, 0.1, 0.15, ... % l r u d
  0.075, 0.04) % x y

% Gemeinsame Legende
h = legend(leghdl, {'$\kappa_{\mathrm{ext}}=0$', ...
  '$\kappa_{\mathrm{ext}}=1$', '$\kappa_\varepsilon=0$'}, ...
  'interpreter', 'latex', 'orientation', 'horizontal');
set(h, 'Position', [0.42, 0.92, .25, .08])

%% Bild speichern
% save figures: Keine Berücksichtigung der externen Kraft im Beobachter
res_path = fullfile(tb_path, 'simulink', 'results', 'atlas_JIC_obs_extforce_cmp');

mkdirs(res_path);
Filebasename_res = 'SimExp_ObsExtForce';
saveas(99, fullfile(res_path, [Filebasename_res, '.fig']));
export_fig(fullfile(res_path, [Filebasename_res, '.pdf']));
export_fig(fullfile(res_path, [Filebasename_res, '.eps']));
export_fig(fullfile(res_path, [Filebasename_res, '.png']));

fprintf('Nach %s gespeichert.\n', res_path);