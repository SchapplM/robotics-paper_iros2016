% Create assembly of constant velocity plots
% 
% before, execute:
% experiments/evaluation_settings/atlas_experiment_comparison_settings_constvel

% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-08
% (c) Institut für Regelungstechnik, Universität Hannover

clear
close all
clc

tb_path = fileparts(which('drc_paper_path_init.m')); 
eval_path = fullfile(tb_path, 'experiments', 'eval_atlas5', 'SI_E036_Joint4_ConstVel');

%% look for figure files with different speeds
speed =  [1, 3, 5]; % different speeds, for which the input figures have been created

figin_filepath_velo = cell(3,1);
figin_filepath_tau = cell(3,1);
figin_filepath = cell(6,1); % filepaths of the original figures for the comparison plot
figin_filepath2 = cell(6,1); % paths to figures with only one subplot
for i = 1:3
  % get figure file paths
  figin_filepath_velo{i} = fullfile(eval_path, sprintf('SI_E036_Joint4_ConstVel_speed%d_vel', speed(i)));
  figin_filepath_tau{i} = fullfile(eval_path, sprintf('SI_E036_Joint4_ConstVel_speed%d_tau', speed(i)));
  figin_filepath{2*i-1} = figin_filepath_velo{i};
  figin_filepath{2*i-0} = figin_filepath_tau{i};
end

%% Delete unneeded subplots
for i = 1:length(figin_filepath)

  [~, Filebasename] = fileparts(figin_filepath{i});
  figinputpath = fullfile(eval_path, [Filebasename, '.fig']);
  uiopen(figinputpath,1);
 
  ch = get(gcf, 'children');
  delete(ch([1:4, 6:8])); % keep elx
  subplot_expand(gcf,1,1,ch(5));
  
  figin_filepath2{i} = fullfile(eval_path, [Filebasename, '_elx.fig']);
  saveas(gcf, figin_filepath2{i});
end

%% Gesamtbild
close all
fighdl = figure(1);clf;
axhdl = set_fig2subfig(fighdl, figin_filepath2([3 4])); % [1,3,5,2,4,6]

%% Farb- und Linienkorrektur
axes(axhdl(1))
linhdl1 = get(axhdl(1), 'children');
% doppelte Soll-Linie löschen
delete(linhdl1(3))
format = {'r', 'd', '-', 25; ... % ImpCtrl
          'b', '', '-', 25; ... % PD-Regler
          'k',  '', '--', 25}; ... % Sollwerte; 
leghdl1 = line_format_publication(linhdl1([4,2,1]), format, {'IC', 'PD','Soll'});    

axes(axhdl(2))
linhdl2 = get(axhdl(2), 'children');
leghdl2 = line_format_publication(linhdl2([2 1]), format, {'IC', 'PD'});    

%% Achsenbeschriftung
for i = 1:2
  for j = 1:1
    subplot(2,1,sprc2no(2,1,i,j));
    if i == 2
      xlabel('$t$ [s]', 'interpreter', 'latex');
    end
    if i == 1 && j == 1
      ylabel('$\dot{q}_4$ [rad/s]', 'interpreter', 'latex');
      % title(sprintf('qD = %1.1f rad/s', 0.2*speed(j)));
    elseif i == 2 && j == 1
      ylabel('$\tau_4$ [Nm]', 'interpreter', 'latex');
    end
    if i == 1
      set(gca, 'Xticklabel', {});
    end
    
      xlim([0,3.5]);
      grid on;
  end
end

%%

figure_format_publication(axhdl);

% Achsen anpassen
linkxaxes
change_x_data(fighdl, -0.65);
xlim([0,2.75]);
% Grenzen automatisch ändern
set_y_autoscale(fighdl,0.1, 0)

% Größe des Plots maximieren
set_size_plot_subplot(gcf, ...
  8.7, 5.6, ...
  axhdl, ...
  0.14, 0.01, 0.12, 0.13, ... % l, r, u, d
  0.05, 0.05) % dx, dy
x2l = get(axhdl(2), 'XLABEL');
set(x2l, 'POSITION', get(x2l, 'POSITION').*[0 0 1]+[1.2 -28 0])

%%
% Legende erstellen
h = legend(leghdl1, ...
  {'ImpCtrl','PD','desired'});
set(h, 'location', 'northoutside', 'Orientation', 'horizontal')
rect = [0.5 0.87, .05, .15]; % x, y, h, b
set(h, 'Position', rect)

% speichern
mkdirs(eval_path);
export_fig(fighdl, fullfile(eval_path, 'SI_E036_Joint4_ConstVel_Summary_medium_speed.fig'));
export_fig(fighdl, fullfile(eval_path, 'SI_E036_Joint4_ConstVel_Summary_medium_speed.eps'));
export_fig(fighdl, fullfile(eval_path, 'SI_E036_Joint4_ConstVel_Summary_medium_speed.pdf'));
export_fig(fighdl, fullfile(eval_path, 'SI_E036_Joint4_ConstVel_Summary_medium_speed.png'));
fprintf('Bilder nach %s gespeichert\n', eval_path);