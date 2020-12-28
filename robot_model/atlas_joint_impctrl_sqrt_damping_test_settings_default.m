% Settings to start the Atlas Arm joint impedance controller
% called by atlas_joint_impctrl_sqrt_damping_test_start.m

% Jonathan Vorndamme, vorndamme@irt.uni-hannover.de, 2015-01
% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-09
% (c) Institut für Regelungstechnik, Universität Hannover

%% Init

lrStrings = {'left', 'right'};

AS=atlas_const(atlas_version, lr);
if atlas_limb == 1
  q_r=0.5*(AS.q_min(AS.JI_rArm)+AS.q_max(AS.JI_rArm));
  q_l=0.5*(AS.q_min(AS.JI_lArm)+AS.q_max(AS.JI_lArm));
  NJL = AS.NJA;
  JI = AS.JI_Arm;
  % TODO: Simulink-Modell an Arm v5 umbenennen
  sl_Modellname = 'atlas_joint_impctrl_sqrt_damping_test';
  atlas_arm_lr = lr;
else
  q_r=0.5*(AS.q_min(AS.JI_rLeg)+AS.q_max(AS.JI_rLeg));
  q_l=0.5*(AS.q_min(AS.JI_lLeg)+AS.q_max(AS.JI_lLeg));
  NJL = AS.NJL;
  JI = AS.JI_Leg;
  sl_Modellname = 'atlas5_leg_joint_impctrl_sqrt_damping_test';
  atlas_leg_lr = lr; 
end

%% Soll-Trajektorie
q0=q_l;
qD0 = zeros(1,NJL)';

q_start = q0';
q_end = q_start;
q_end(2) = q_start(2)+pi/2; %Drehe q2 um 90° (seitlich ausgestreckt)
% C3-stetige Trapez-Trajektorie fahren
vmax = 2*ones(1,NJL);
amax = 10*ones(1,NJL);
jmax = 100*ones(1,NJL);
zmax = [ (q0(2)+pi/2) ; 2; 10; 100];

[~, ~, w_t, w_Q] = traj_trapezN( [q_start; zeros(2,NJL)], ...
  [q_end; zeros(2,NJL)], [q_end; vmax; amax; jmax], 1e-3, false);
qsollDaten.q = NaN(length(w_t), NJL);
qsollDaten.qD = NaN(length(w_t), NJL);
qsollDaten.qDD = NaN(length(w_t), NJL);
for i = 1:NJL
  qsollDaten.q(:,i) = w_Q(:,1,i);
  qsollDaten.qD(:,i) = w_Q(:,2,i);
  qsollDaten.qDD(:,i) = w_Q(:,3,i);
end

qsollDaten.t = w_t; % Zeitbasis ist für alle Achsen gleich. Daher hier keine Anpassung notwendig

%% Externe Kraft
t_Fext = [0;1;4];
F_ext =  [[0;100;0], zeros(3,5)];

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
  
%% model parameters
if atlas_limb == 1
  [a_mdh, d_mdh, ~, ~, rSges_mdh_ORIG, m_ORIG,Iges_mdh_ORIG] = atlas_arm_parameter_mdh(atlas_arm_lr, atlas_version);
  MPV_ORIG = atlas_arm_convert_par1_MPV(rSges_mdh_ORIG, m_ORIG, Iges_mdh_ORIG, a_mdh, d_mdh, atlas_version);
  % Zusatzlast: 1kg mit 100mm Hebelarm, damit Trägheit im Dynamikmodell
  % höher wird.
  MPV_payload = atlas5_arm_FT_payload_MPV(1, [.1,.1,.1]', [0.1, 0, 0, 0.1, 0, 0.1]', true);
  MPV_arm_plant=MPV_ORIG+MPV_payload; % MPV for simulated plant
  MPV_arm_model=MPV_ORIG+MPV_payload;% MPV for calculating the gravity model
  MPV_arm_damp=MPV_ORIG+MPV_payload; % MPV for calculating the inertia matrix (positive definite)
  % set noise levels for simulated measurements
  q_noise     = 0.01;
  qD_noise    = 0.01;
  tau_noise   = 0.5;
  F_ext_noise = 3;
  g_noise     = 0.001;
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
else
  [a_mdh, d_mdh, ~, ~, rSges_mdh_ORIG, m_ORIG,Iges_mdh_ORIG] = atlas_leg_parameter_mdh(atlas_leg_lr, atlas_version);
  MPV_ORIG = atlas_leg_convert_par1_MPV(rSges_mdh_ORIG, m_ORIG, Iges_mdh_ORIG, a_mdh, d_mdh, atlas_version);
  MPV_leg_plant=MPV_ORIG; % MPV for simulated plant
  MPV_leg_model=MPV_ORIG;% MPV for calculating the gravity model
  MPV_leg_damp=MPV_ORIG; % MPV for calculating the inertia matrix (positive definite)
end

d   = 0.2*ones(NJL,1);
muC = 8*ones(NJL,1);

% controller parameters and gains
K_d     = 150*eye(NJL);
D       = 0.7*eye(NJL);
K_O     = 5*eye(NJL);
T       = 0.001;
T1      = 0;
K_tau_g = 1;
K_tau_c = 1;
K_tau_K = 1;
K_tau_D = 1;
K_tau_a = 1;
K_tau_f = 0;
K_tau_e = 0;
K_tau_o = 0;
K_ext_c = 0;
K_obs_c = 0;
K_fri_o = 1;
K_ext_o = 1;
K_qD_d  = 1;
K_qDD_d = 1;
tau_c   = muC - 4*ones(NJL,1);
B       = d - 0.1*ones(NJL,1);
qD_th   = 0.05*ones(NJL,1);
tau_th  = 0.02*ones(NJL,1);

% Filter für Ein- und Ausgänge des Reglers
T1_filter_input = 0;
T1_filter_output = 0;
%% Configure Model

load_system(sl_Modellname)
configSet = getActiveConfigSet(sl_Modellname);
set_param(configSet, 'Solver', 'ode4');
set_param(configSet, 'FixedStep', '1e-6');

%% Define Inputs
simin_tau_ext = struct('time', 0, ...
    'signals', struct('values', zeros(1,NJL), 'dimensions', NJL), ...
    'name', 'tau_ext');
simin_tau_m = struct('time', 0, ...
    'signals', struct('values', zeros(1,NJL), 'dimensions', NJL), ...
    'name', 'tau_m');
