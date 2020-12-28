% Prüfe Ausgaben des Gelenkimpedanzreglers mit Eingangsdaten aus Roboterversuchen 
% Notwendig zum laufen: Assert-Befehl in sqrt-damping-block in Impedanzregler muss entfernt werden.
% TODO: Änderung der Impedanzregler-Parameter berücksichtigen
% TODO: SL mit tuneable parameters, um schneller neu zu berechnen.

% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-04
% (c) Institut für Regelungstechnik, Universität Hannover

% function [out_tau_Soll, out_tau_obs, out_tau_ext_mes] = atlas_calc_impctrl_output(Data, atlas_version, lr)

out_t = [];

% gestelltes Soll-Drehmoment
out_tau_Soll = [];
% Beobachter-Störmoment aus Simulink-Modell
out_tau_obs = [];
% Soll-Drehmoment ohne Beobachter-Störmoment
out_tau_Soll_wo_obs = [];
out_tau_ext_mes = [];

chains = {'left_arm', 'right_arm'};
AS = atlas_const(atlas_version, lr);
if lr == true
  ich = 1;
else
  ich = 2;
end
atlas_arm_lr = lr;
Strct = Data.ToRobot.(sprintf('ImpCtrlGains_%s', chains{ich}));


% Loop over all different parameter changes of the controller. Execute the
% simulink-model again each time.
for i_change = 1:length(Strct.t)-1

  %% Set Gains of Impedance Controller
  K_d = diag([Strct.K_d_shz(i_change); Strct.K_d_shx(i_change); Strct.K_d_ely(i_change); Strct.K_d_elx(i_change); Strct.K_d_wry(i_change); Strct.K_d_wrx(i_change); Strct.K_d_wry2(i_change)]);
  D = diag([Strct.D_shz(i_change); Strct.D_shx(i_change); Strct.D_ely(i_change); Strct.D_elx(i_change); Strct.D_wry(i_change); Strct.D_wrx(i_change); Strct.D_wry2(i_change)]);

  K_tau_K = Strct.K_tau_K(i_change);
  K_tau_D = Strct.K_tau_D(i_change);

  K_tau_g = Strct.K_tau_g(i_change);
  K_tau_c = Strct.K_tau_c(i_change);
  K_tau_a = Strct.K_tau_a(i_change);
  K_tau_f = Strct.K_tau_f(i_change);
  K_tau_e = Strct.K_tau_e(i_change);
  K_tau_o = Strct.K_tau_o(i_change);
  K_ext_c = Strct.K_ext_c(i_change);
  K_obs_c = Strct.K_obs_c(i_change);
  K_fri_o = Strct.K_fri_o(i_change);
  K_ext_o = Strct.K_ext_o(i_change);

  K_qD_d  = Strct.K_qD_d(i_change);
  K_qDD_d = Strct.K_qDD_d(i_change);
  T1 = Strct.T1(i_change);
  K_O = Strct.K_O(i_change) * eye(AS.NJA);
  T = 1e-3;

  qD_th = [Strct.qD_th_shz(i_change); Strct.qD_th_shx(i_change); Strct.qD_th_ely(i_change); Strct.qD_th_elx(i_change); Strct.qD_th_wry(i_change); Strct.qD_th_wrx(i_change); Strct.qD_th_wry2(i_change)];
  tau_th = [Strct.tau_th_shz(i_change); Strct.tau_th_shx(i_change); Strct.tau_th_ely(i_change); Strct.tau_th_elx(i_change); Strct.tau_th_wry(i_change); Strct.tau_th_wrx(i_change); Strct.tau_th_wry2(i_change)];

  tau_c   = [Strct.tau_c_shz(i_change); Strct.tau_c_shx(i_change); Strct.tau_c_ely(i_change); Strct.tau_c_elx(i_change); Strct.tau_c_wry(i_change); Strct.tau_c_wrx(i_change); Strct.tau_c_wry2(i_change)];
  B       = [Strct.B_shz(i_change); Strct.B_shx(i_change); Strct.B_ely(i_change); Strct.B_elx(i_change); Strct.B_wry(i_change); Strct.B_wrx(i_change); Strct.B_wry2(i_change)];

  if lr == true
    MPV_arm_damp = MPV_lArm_damp;
    MPV_arm_model = MPV_lArm_model;
  else
    MPV_arm_damp =   MPV_rArm_damp;
    MPV_arm_model = MPV_rArm_model; 
  end


  %% Set Inputs
  simin_q_mess = struct('time', Data.FromRobot.t, ...
      'signals', struct('values', Data.FromRobot.q(:,AS.JI_Arm), 'dimensions', AS.NJA), ...
      'name', 'q_mess');
  simin_qD_mess = struct('time', Data.FromRobot.t, ...
      'signals', struct('values', Data.FromRobot.qD(:,AS.JI_Arm), 'dimensions', AS.NJA), ...
      'name', 'qD_mess');
  simin_q_soll = struct('time', Data.ToRobot.t, ...
      'signals', struct('values', Data.ToRobot.q(:,AS.JI_Arm), 'dimensions', AS.NJA), ...
      'name', 'q_soll');
  simin_qD_soll = struct('time', Data.ToRobot.t, ...
      'signals', struct('values', Data.ToRobot.qD(:,AS.JI_Arm), 'dimensions', AS.NJA), ...
      'name', 'qD_soll');  
  simin_qDD_soll = struct('time', Data.ToRobot.t, ...
      'signals', struct('values', Data.ToRobot.qDD(:,AS.JI_Arm), 'dimensions', AS.NJA), ...
      'name', 'qDD_soll'); 
  % Calculate Gravity in utorso  
  G_utorso = NaN(length(Data.FromRobot.t), 3);
  for i = 1:length(Data.FromRobot.t)
    R_pelvis = quat2r_mex(Data.FromRobot.pelvis_x_r(i,:));
    T_pelvis_utorso = atlas_T_pelvis_utorso_mex(Data.FromRobot.q(i,:), atlas_version);
    R_utorso = R_pelvis * T_pelvis_utorso(1:3,1:3);
    G_utorso(i,:) = R_utorso'*[0;0;-9.81]; 
  end
  simin_g_mess= struct('time', Data.FromRobot.t, ...
      'signals', struct('values', G_utorso, 'dimensions', 3), ...
      'name', 'g_mess'); 
  simin_tauJ_mess = struct('time', Data.FromRobot.t, ...
      'signals', struct('values', Data.FromRobot.tau(:,AS.JI_Arm), 'dimensions', AS.NJA), ...
      'name', 'tauJ_mess'); 
  % Die ROS-Implementierung nutzte F_lH_raw (von Mai bis Juli). Damit war
  % die linke Hand im falschen KS.
  % Eigentlich hätte F_lH_calib genutzt werden sollen (Kalibrierung auf die
  % korrekt gedrehten Daten angewendet). 
  if lr == true
    F_ext_hand = Data.FromRobot.F_lH_raw_calib;
  else
    F_ext_hand = Data.FromRobot.F_rH_raw_calib;
  end
  F_ext_utorso = NaN(size(F_ext_hand));
  % rotate external force into utorso base frame
  for i = 1:size(F_ext_hand,1)
    T_0_EE = atlas_arm_fkine_mex(Data.FromRobot.q(i,AS.JI_Arm), lr, atlas_version);
    F_ext_utorso(i,1:3) = t2r(T_0_EE(:,:,end))*(F_ext_hand(i,1:3))';
    F_ext_utorso(i,4:6) = t2r(T_0_EE(:,:,end))*(F_ext_hand(i,4:6))';
  end
  
  simin_F_ext_mess = struct('time', Data.FromRobot.t, ...
      'signals', struct('values', F_ext_utorso, 'dimensions', 6), ...
      'name', 'F_ext_mess'); 

  %% Calculate Simulink Model
  sl_Modellname = 'atlas5_arm_impctrl_iotest';
  starttime = Strct.t(i_change);
  if i_change == 1
    stoptime = Strct.t(i_change+1);
  else
    stoptime = Data.FromRobot.t(end);
  end
  if stoptime <= starttime + 1e-3
    continue % protect against wrong time settings in input data
  end
  tic;
  fprintf('Berechne Reglerausgang mit Simulink-Modell für Zeit [%1.4f, %1.4f]\n', ...
    starttime, stoptime);
  simOut = sim(sl_Modellname, 'StopTime', num2str(stoptime), 'StartTime', num2str(starttime)); % num2str(Data.FromRobot.t(end))
  fprintf('\tRechenzeit: %1.1fs\n', toc);
  sl_logsout = simOut.get('logsout');
  sl_y = simOut.get('yout');
  sl_t = simOut.get('tout');

  %% Get Signals from Logsout
  sl = get_simulink_outputs(simOut, sl_Modellname);

  out_tau_ext_mes = [out_tau_ext_mes; sl.tau_ext_mes]; %#ok<AGROW>
  out_tau_obs = [out_tau_obs; sl.tau_obs]; %#ok<AGROW>
  out_tau_Soll = [out_tau_Soll; sl.tau_Soll]; %#ok<AGROW>
  out_tau_Soll_wo_obs = [out_tau_Soll_wo_obs; ...
    sl.tau_Soll - K_ext_o * sl.tau_obs]; %#ok<AGROW>
  out_t = [out_t; sl.t]; %#ok<AGROW>

end