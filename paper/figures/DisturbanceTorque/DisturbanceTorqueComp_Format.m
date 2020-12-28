% Automatisch erstellte Bilder mit Beobachter-Daten aus Versuch E061
% (Gegendrücken) öffnen und neu formatieren
% 
% Vorher folgendes Skript ausführen:
% IdentExpEval_settings_ImpCtrlE061_R11_obscorr.m

% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-08
% (c) Institut für Regelungstechnik, Universität Hannover
%% Init
tb_path = fileparts(which('drc_paper_path_init.m')); 
close all

% Import-Pfad für vorher generierte Bilder
eval_path = fullfile(tb_path, 'experiments', 'eval_atlas5', ...
  'ImpCtrlv5_E061');

%% Teilbild 1: Achse 1
% Speichern der xy-Daten
X = cell(3,1);
Y = cell(3,1);

%% tau_eps
% die beiden Versuche durchgehen und die Daten laden
Basenames = {'R11_Obs_ohne_ext_Beo', 'R11_Obs_mit_ext_Beo'};
for i = 1:2
  Filebasename = Basenames{i};
  figinputpath = fullfile(eval_path, [Filebasename, '.fig']);
  uiopen(figinputpath,1);
  fig_i = gcf;
  ch_f = get(fig_i, 'Children');
  % alle subplots bis auf tau_e_1 löschen
  delete(ch_f(1:13))
  
  % Datenreihen speichern (1=Simulink)
  % tau_eps wird in aktueller Implementierung negativ geschätzt.
  % Daher hier Vorzeichen umkehren
  ch = get(gca, 'Children');
  X{i} = get(ch(1), 'XData');
  Y{i} = -get(ch(1), 'YData');
end

%% tau_ext
% die beiden Versuche durchgehen und die Daten laden
Basenames = {'R11_torque_Zeroed_FT'};
for i = 1
  Filebasename = Basenames{i};
  figinputpath = fullfile(eval_path, [Filebasename, '.fig']);
  uiopen(figinputpath,1);
  fig_i = gcf;
  ch_f = get(fig_i, 'Children');
  % alle subplots bis auf shz löschen
  delete(ch_f(1:10))

  % Datenreihen speichern (1=tau_ext)
  ch = get(gca, 'Children');
  X{3} = get(ch(1), 'XData');
  Y{3} = get(ch(1), 'YData');
end

% zusammengesuchte Daten neu plotten
fig_ges = figure();
hold on;
linhdl = NaN(3,1);

% Gelenkmomente aus externen Kräften
linhdl(1) = plot(X{3}, Y{3});

% Beobachter-Störmoment ohne ExtKraft-Kompensation
linhdl(2) = plot(X{1}, Y{1});

% Beobachter-Störmoment mit ExtKraft-Kompensation
linhdl(3) = plot(X{2}, Y{2});

leghdl = line_format_publication(linhdl, {'Ext', 'OhneKomp', 'MitKomp'});
%% Formatieren

xlim([3, 14]);

set_y_autoscale(fig_ges,0.1)

grid on;

ylabel('$\hat{\tau}_{\varepsilon,1}, \tau_{\mathrm{ext,1}}$  [Nm]', ...
  'interpreter', 'latex');
xlabel('$t$ [s]', 'interpreter', 'latex');

% Vor-Formatierung
figure_format_publication();
set_sizePositionPlotSubplot(gcf, ...
  8.67, 5.5, ...
  [1 1], ...
  0.11, 0.02, 0.15, 0.17, ... % l r u d
  0.075, 0.05) % x y

% Legende
ch = get(gca, 'children');
h = legend(leghdl, {'$\tau_{\mathrm{ext}}$', ...
  '$\hat{\tau}_{\varepsilon}, K_{\mathrm{ext,o}}=0$', ...
  '$\hat{\tau}_{\varepsilon}, K_{\mathrm{ext,o}}=1$'}, ...
  'interpreter', 'latex', 'orientation', 'horizontal');
set(h, 'Position', [0.42, 0.88, .25, .08])

% Speichern


