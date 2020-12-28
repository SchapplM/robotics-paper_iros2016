% In order to get the data follow the following procedure:
% 
% * Identifikationsergebnis vom DRC-Netzlauf ins DRC-Repo kopieren oder
% verlinken: /irtfs_drc/Experiments/atlas5_SysId/ident_results/00_chosen
%         -> atlas_matlab/ident_results/00_chosen
% * Ausführen von identification/misc/plot_mpv_12_20150825_left_ICRA_2.m
% 
% Beim Generieren der Plots und Berechnung der Fehler wurden folgende
% Messdaten berücksichtigt: 
% 
%     'SI_E048_OptimTraj_Both_Arms_00'        'R03_both_20150728'    'Ident_Batch_20150728_SI_E048_BagOptTrajSpeed_10_2015-07-28-19_59_1.mat'    
%     'SI_E049_OptimTraj_Both_Arms_01'        'R02_both_20150728'    'Ident_Batch_20150728_SI_E049_BagOptTrajSpeed_10_2015-07-28-19_59_1.mat'    
%     'SI_E050_OptimTraj_Both_Arms_02'        'R02_both_20150728'    'Ident_Batch_20150728_SI_E050_BagOptTrajSpeed_10_2015-07-28-19_59_1.mat'    
%     'SI_E051_OptimTraj_Both_Arms_03'        'R02_both_20150728'    'Ident_Batch_20150728_SI_E051_BagOptTrajSpeed_10_2015-07-28-19_59_1.mat'    
%     'SI_E052_OptimTraj_Both_Arms_04'        'R02_both_20150728'    'Ident_Batch_20150728_SI_E052_BagOptTrajSpeed_10_2015-07-28-19_59_1.mat'    
%     'SI_E054_Opt_OptimTraj_Both_Arms_17'    'R05_both_20150728'    'Ident_Batch_20150728_SI_E054_BagOptTrajSpeed_10_2015-07-28-19_59_1.mat'    
%     'SI_E055_Opt_OptimTraj_Both_Arms_18'    'R04_both_20150728'    'Ident_Batch_20150728_SI_E055_BagOptTrajSpeed_10_x10_2015-07-28-19_59_1.mat'
%     'SI_E056_Opt_OptimTraj_Both_Arms_19'    'R03_both_20150728'    'Ident_Batch_20150728_SI_E056_BagOptTrajSpeed_10_2015-07-28-19_59_1.mat'    
%     'SI_E057_Opt_OptimTraj_Both_Arms_20'    'R03_both_20150728'    'Ident_Batch_20150728_SI_E057_BagOptTrajSpeed_10_2015-07-28-19_59_1.mat'    
%     'SI_E058_Opt_OptimTraj_Both_Arms_21'    'R03_both_20150728'    'Ident_Batch_20150728_SI_E058_BagOptTrajSpeed_10_2015-07-28-19_59_1.mat'    
%     'SI_E059_Opt_OptimTraj_Both_Arms_22'    'R02_both_20150728'    'Ident_Batch_20150728_SI_E059_BagOptTrajSpeed_10_2015-07-28-19_59_1.mat'   

% Alexander Tödtheide, toedtheide@irt.uni-hannover.de, 2015-08
% (c) Institut für Regelungstechnik, Universität Hannover

clear ; close all; clc;
tb_path = fileparts(which('drc_paper_path_init.m'));
path_output_figure = fullfile(tb_path, 'experiments', ...
  'eval_atlas5', 'atlas5_plot_torque_ident_MPV_left_IROS');
name_output_figure = 'atlas5_plot_torque_ident_MPV_left_IROS';

% Versuchsergebnisse für das Zweischrittverfahren (getrennte
% Identifikation von Dynamik und Reibung)
tmp=load(fullfile(tb_path, 'tmp', '2015.08.25-13.19.39_left_dyn_assume_frct_more_novar_6.mat'));
dat_mes = tmp.dat_mes_out;
dat_gen = tmp.dat_gen_out;
% Versuchsergebnisse für das Einschrittverfahren (kombinierte
% Identifikation von Dynamik und Reibung)
tmp=load(fullfile(tb_path, 'tmp', '2015.08.25-13.19.39_left_dyn_assume_frct_more_novar_6MPV_004_compare.mat'));
dat_mes_MPV004 = tmp.dat_mes_MPV004;
dat_gen_MPV004 = tmp.dat_gen_MPV004;

% Cell joint names
cell_joint_names = {'shz', 'shx', 'ely', 'elx', 'wry', 'wrx', 'wry2'};

lr = true;
AS = atlas_const(uint8(5), lr);
joint_indices = AS.JI_Arm;

xlim_1 = 10;
xlim_2 = 20;

% Cut indices by find function
indices_time = find(dat_gen.t > xlim_1 & dat_gen.t < xlim_2);
indices_time_MPV4 = find(dat_gen_MPV004.t > xlim_1 & dat_gen_MPV004.t < xlim_2);

% Run through measured data for all joints and plot 
fig_handle = figure;
error_tau_gen = NaN(7,1);
error_tau_gen_MPV4 = NaN(7,1);
detcoeff_tau_gen = NaN(7,1);
detcoeff_tau_gen_MPV4 = NaN(7,1);
var_tau_gen = NaN(7,1);
var_tau_gen_MPV4 = NaN(7,1);
indices_position = 1:2:13;
indices_torque = 2:2:14;
axhdl = NaN(7,1);
IJ_plot = [1 2 3 4]; % Indizes der zu plottenden Gelenke (nur hydraulische)
for i = 1:7
  
  %% Fehlerberechnung
  % shift time starting from t = 0
  t_time = dat_gen.t(indices_time,1) - xlim_1;
  t_timeMPV4 = dat_gen.t(indices_time,1) - xlim_1;
  % Gemessenes Moment
  tau_mes =  dat_mes.TAU_Rob(indices_time,joint_indices(i));
  % Generiertes Moment für Zweischrittverfahren
  tau_gen = dat_gen.tau(indices_time,i);
  % Generiertes Moment für Einschrittverfahren
  tau_gen_MPV4 = dat_gen_MPV004.tau(indices_time_MPV4,i);

  % do error calculation here because it fits in the loop
  error_tau_gen(i) = sum((tau_mes - tau_gen).^2/length(tau_mes));
  error_tau_gen_MPV4(i) = sum((tau_mes - tau_gen_MPV4).^2/length(tau_mes));

  % Berechne das Bestimmtheitsmaß zwischen den gemessenen Daten und der
  % Regression
  % siehe https://de.wikipedia.org/wiki/Bestimmtheitsma%C3%9F#Konstruktion
%   covar = cov(tau_mes, tau_gen);
%   detcoeff_tau_gen(i) = sum( (tau_gen-mean(tau_mes)).^2 ) / sum((tau_mes-mean(tau_mes)).^2);
  var_tau_gen(i) = cov(tau_mes-tau_gen);
%   covar = cov(tau_mes, tau_gen_MPV4);
%   detcoeff_tau_gen_MPV4(i) = covar(2,2) / covar(1,1);
  var_tau_gen_MPV4(i) = cov(tau_mes-tau_gen_MPV4);
  %% Zeichnen
  if ~any(i == IJ_plot)
    continue % Berechne Fehler für alle Daten, aber plotte nur Hydraulik
  end
  axhdl(i) = subplot(length(IJ_plot),1,(i));
  format = {'r',  '', '-', 0; ...
            'k', 'd', '-', 8; ...
            'b', 's', '-', 8};  
  linhdl = NaN(3,1);
  linhdl(1) = plot(t_time, tau_mes,'r'); hold on;
  linhdl(2) = plot(t_timeMPV4, tau_gen_MPV4,'k--'); hold on;
  linhdl(3) = plot(t_time, tau_gen,'b-.'); hold on; 
  leghdl = line_format_publication(linhdl, format, {'m', 'I', 'II'});

  grid on;
  if i == 1
    l1 = legend(leghdl,{'$\tau_{\mathrm{m},i}$','$\tau_{\mathrm{I},i}$','$\tau_{\mathrm{II},i}$'}, ...
      'interpreter', 'latex');
     set(l1,'Orientation','horizontal'); set(l1, 'Location', 'Southeast');
  end

  xlabel('$t$ [s]', 'interpreter', 'latex');  
  ylabel(sprintf('\\tau_{%d} [Nm]', i));
  xlim([0, xlim_2 - xlim_1]);


end

%% Output

shape = [length(IJ_plot),1];  
n_cols = shape(2);
nc_rows = shape(1);

set_FontFontsize(fig_handle,'Times',8)
set_y_autoscale(fig_handle,0.05)
% set_NumberOfTickElements(fig_handle,5,5,nc_rows,n_cols);
remove_InnerLabels(fig_handle,nc_rows,n_cols,4);
set_size_plot_subplot(gcf, ...
  8.7, 9.5, ...
  axhdl(IJ_plot), ...
  0.12, 0.02, 0.07, 0.08, ... % l, r, u, d
  0.07, 0.02) % dx, dy

% Legende nach oben
set(l1, 'Position', [0.35 0.95, .3, .03])

% Rand vollständig zeichnen
set(gca, 'Box', 'on')
% Weißer Hintergrund
set(gcf,'color','w');

% Alle ylabel auf eine Höhe
for i = IJ_plot
  yil = get(axhdl(i), 'YLABEL');
  set(yil, 'POSITION', get(yil, 'POSITION').*[0 1 1]+[-0.7 0 0]);
end
% x-Achsenbeschriftung etwas höher
x4l = get(axhdl(4), 'XLABEL');
set(x4l, 'POSITION', [5, -26, 1]);


mkdirs(path_output_figure);

saveaspdf(fig_handle,fullfile(path_output_figure, [name_output_figure]));
savefig(fig_handle, fullfile(path_output_figure, [name_output_figure,'.fig']));
saveas(fig_handle, fullfile(path_output_figure, [name_output_figure,'.png']),'png');
fprintf('Bild nach %s gespeichert\n', path_output_figure);
%% Error calculation
% Vergleich der MSE-Werte für beide Identifikationsmethoden
fprintf('Error MPV4 (combined identification):\n');
disp(error_tau_gen_MPV4');
fprintf('Error MPV_frct (two-step identification):\n');
disp(error_tau_gen');

% fprintf('Coefficient of Determination MPV4 (combined identification):\n');
% disp(detcoeff_tau_gen_MPV4');
% fprintf('Coefficient of Determination MPV_frct (two-step identification):\n');
% disp(detcoeff_tau_gen');

fprintf('Variance MPV4 (combined identification):\n');
disp(var_tau_gen_MPV4');
fprintf('Variance MPV_frct (two-step identification):\n');
disp(var_tau_gen');