% Automatisch erstellte Bilder mit Ergebnissen aus Versuch E061
% (Gegendrücken) öffnen und neu formatieren
% 
% Vorher folgende Skripte ausführen:
% Ordner: irt_drc_atlas_repo_matlab/experiments/evaluation_settings/
% atlas_identification_experiment_test_settings_ImpCtrlE061_R11.m
% atlas_identification_experiment_test_settings_ImpCtrlE061_R13.m

% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-08
% (c) Institut für Regelungstechnik, Universität Hannover
%% Init
tb_path = fileparts(which('drc_paper_path_init.m')); 
close all

% Import-Pfad für vorher generierte Bilder
eval_path = fullfile(tb_path, 'experiments', 'eval_atlas5', ...
  'ImpCtrlv5_E061');

% Cell für Dateinamen der einzelnen Bilder für das Gesamtbild
GesFig = cell(1,2);

% Offset zwischen den Versuchen
T_Off = [0,-2];

% Beginn und Ende des Vergleichszeitraumes für die Steifigkeit
% (Originalmessdaten)
T_1 = 7.5;
T_2 = 8;
%% Teilbild 1: Position error
% Speichern der xy-Daten
X = cell(2,1);
Y = cell(2,1);

% die beiden Versuche durchgehen und die Daten laden
Basenames = {'R11_cartesian_stiffness', 'R13_cartesian_stiffness'};
for i = 1:2
  Filebasename = Basenames{i};
  figinputpath = fullfile(eval_path, [Filebasename, '.fig']);
  uiopen(figinputpath,1);
  fig_i = gcf;
  ch_f = get(fig_i, 'Children');
  % alle subplots bis auf delta x löschen
  delete(ch_f([1:11]))

  % Datenreihen speichern
  ch = get(gca, 'Children');
  X{i} = get(ch(1), 'XData');
  Y{i} = get(ch(1), 'YData');
end

% zusammengesuchte Daten neu plotten
fig_tmp = figure();
hold on;
for i = 1:2
  plot(X{i}+T_Off(i), Y{i});
end

% Speichern
GesFig{1,1} = fullfile(eval_path, 'StiffObs_icra_deltax.fig');
saveas(fig_tmp, GesFig{1,1});

% Textausgabe für Auswertung:
[~,I1_1] = min(abs(X{1}-T_1+T_Off(1)));
[~,I2_1] = min(abs(X{2}-T_1+T_Off(2)));
[~,I1_2] = min(abs(X{1}-T_2+T_Off(1)));
[~,I2_2] = min(abs(X{2}-T_2+T_Off(2)));
dp1 = mean(Y{1}(I1_1:I1_2));
dp2 = mean(Y{2}(I2_1:I2_2));
fprintf('Steigerung der Positionsabweichung bei t=%1.1f...%1.1f: %1.2f%% (%1.3f -> %1.3f\n', ...
  T_1, T_2, 100*(dp2 - dp1) / dp1, dp1, dp2 );
%% Teilbild 2: Externe Kraft
% Speichern der xy-Daten
X = cell(2,1);
Y = cell(2,1);

% die beiden Versuche durchgehen und die Daten laden
Basenames = {'R11_cartesian_stiffness', 'R13_cartesian_stiffness'};
for i = 1:2
  Filebasename = Basenames{i};
  figinputpath = fullfile(eval_path, [Filebasename, '.fig']);
  uiopen(figinputpath,1);
  fig_i = gcf;
  ch_f = get(fig_i, 'Children');
  % alle subplots bis auf Fx löschen
  delete(ch_f([1:10,12]))

  % Datenreihen speichern
  ch = get(gca, 'Children');
  X{i} = get(ch(1), 'XData');
  Y{i} = get(ch(1), 'YData');
end

% zusammengesuchte Daten neu plotten
fig_tmp = figure();
hold on;
for i = 1:2
  plot(X{i}+T_Off(i), Y{i});
end

% Speichern
GesFig{1,2} = fullfile(eval_path, 'StiffObs_icra_Fx.fig');
saveas(fig_tmp, GesFig{1,2});

% Textausgabe für Auswertung:
[~,I1_1] = min(abs(X{1}-T_1+T_Off(1)));
[~,I2_1] = min(abs(X{2}-T_1+T_Off(2)));
[~,I1_2] = min(abs(X{1}-T_2+T_Off(1)));
[~,I2_2] = min(abs(X{2}-T_2+T_Off(2)));
f1 = mean(Y{1}(I1_1:I1_2));
f2 = mean(Y{2}(I2_1:I2_2));
fprintf('Steigerung der externen Kraft bei t=%1.1f...%1.1f: %1.2f%%. %1.1f -> %1.1f\n', ...
  T_1, T_2, 100*(f2 - f1) / f1, f1, f2 );
%% Teilbild 3: Kartesische Steifigkeit
% Speichern der xy-Daten
X = cell(2,1);
Y = cell(2,1);

% die beiden Versuche durchgehen und die Daten laden
Basenames = {'R11_cartesian_stiffness', 'R13_cartesian_stiffness'};
for i = 1:2
  Filebasename = Basenames{i};
  figinputpath = fullfile(eval_path, [Filebasename, '.fig']);
  uiopen(figinputpath,1);
  fig_i = gcf;
  ch_f = get(fig_i, 'Children');
  % alle subplots bis auf Kxx löschen
  delete(ch_f([1:9, 11,12]))

  % Datenreihen speichern
  ch = get(gca, 'Children');
  X{i} = get(ch(2), 'XData'); % Messwerte
  Y{i} = get(ch(2), 'YData');
end

% zusammengesuchte Daten neu plotten
fig_tmp = figure();
hold on;
for i = 1:2
  plot(X{i}+T_Off(i), Y{i});
end

% Speichern
% Nicht in Gesamtbild aufnehmen
% GesFig{1,2} = fullfile(eval_path, 'StiffObs_icra_stiffnessx.fig');
% saveas(fig_tmp, GesFig{1,2});

% Textausgabe für Auswertung:
[~,I1_1] = min(abs(X{1}-T_1+T_Off(1)));
[~,I2_1] = min(abs(X{2}-T_1+T_Off(2)));
[~,I1_2] = min(abs(X{1}-T_2+T_Off(1)));
[~,I2_2] = min(abs(X{2}-T_2+T_Off(2)));
k1 = mean(Y{1}(I1_1:I1_2));
k2 = mean(Y{2}(I2_1:I2_2));
fprintf('Steigerung der kartesischen Steifigkeit bei t=%1.1f...%1.1f: %1.2f%%. %1.2f -> %1.2f\n', ...
  T_1, T_2, 100*(k2 - k1) / k1, k1, k2 );
%% Teilbild 4: Beobachter-Störmoment
% Speichern der xy-Daten
X = cell(2,1);
Y = cell(2,1);

% die beiden Versuche durchgehen und die Daten laden
Basenames = {'R11_torque', 'R13_torque'};
for i = 1:2
  Filebasename = Basenames{i};
  figinputpath = fullfile(eval_path, [Filebasename, '.fig']);
  uiopen(figinputpath,1);
  fig_i = gcf;
  ch_f = get(fig_i, 'Children');
  % alle subplots bis auf shz löschen
  delete(ch_f(1:10))

  % Datenreihen speichern (3=observer)
  ch = get(gca, 'Children');
  X{i} = get(ch(3), 'XData'); % obs
  Y{i} = get(ch(3), 'YData');
end

% zusammengesuchte Daten neu plotten
fig_tmp = figure();
hold on;
for i = 1:2
  plot(X{i}+T_Off(i), Y{i});
end

% Speichern
% Nicht in Gesamtbild aufnehmen
% GesFig{2,2} = fullfile(eval_path, 'StiffObs_icra_observer1.fig');
% saveas(fig_tmp, GesFig{2,2});



%% Zwei Einzelbilder in einen großen Kachel-Plot
close all
% Beide Einzelbilder laden
figHandle = figure(1);

n_cols = 2;
n_rows = 1;

axhdl = set_fig2subfig(figHandle,GesFig);

linkxaxes

change_x_data(figHandle, -4.5);
xlim([0, 3.5]);

set_y_autoscale(figHandle,0.1)
for i = 1:n_cols*n_rows
  % Grenzen automatisch ändern
  axes(axhdl(i)) %#ok<LAXES>
  grid on;hold on;
  ch = get(axhdl(i), 'Children');
  leghdl = line_format_publication(ch, [], {'without obs', 'with obs'});
end

% links:
axes(axhdl(1))
xl1=xlabel('$t$ [s]', 'interpreter', 'latex');
yl1=ylabel('$\Delta x$ [mm]', 'interpreter', 'latex');

% rechts:
axes(axhdl(2))
xl2=xlabel('$t$ [s]', 'interpreter', 'latex');
yl2 = ylabel('$f_{x}$ [N]', 'interpreter', 'latex');

% Vor-Formatierung
figure_format_publication(axhdl)
set_size_plot_subplot(figHandle, ...
  8.55, 3, ...
  axhdl, ...
  0.14, 0.01, 0.18, 0.17, ... % l r u d
  0.14, 0.1) % x y

% Legende
axes(axhdl(1));
h = legend(leghdl, {'$\kappa_{\varepsilon}=1$', '$\kappa_{\varepsilon}=0$'}, 'interpreter', 'latex');
set(h, 'Position', [0.4, 0.90, .25, .05], ... % x y b h
'Orientation', 'Horizontal') 

% a,b,c,d einblenden
annotation('textbox', [ 0.02 0.18 0 0], 'string', '(a)','FontSize',10,'FontName','Times')
annotation('textbox', [ 0.50 0.18 0 0], 'string', '(b)','FontSize',10,'FontName','Times')

% Position der Achsenbeschriftungen nachjustieren
set(xl1, 'position', get(xl1, 'position').*[0 0 1]+[1,-175,0]);
set(xl2, 'position', get(xl2, 'position').*[0 0 1]+[1,-115,0]);
set(yl1, 'position', get(yl1, 'position').*[0 1 1]+[-0.9,0,0]);
set(yl2, 'position', get(yl2, 'position').*[0 1 1]+[-0.8,0,0]);

% Speichern
figure(figHandle)

Filebasename_res = 'ObsFeedthroughStiff_compare';
saveas(figHandle, fullfile(eval_path, [Filebasename_res, '.fig']));
export_fig(fullfile(eval_path, [Filebasename_res, '.pdf']));
export_fig(fullfile(eval_path, [Filebasename_res, '.eps']));
export_fig(fullfile(eval_path, [Filebasename_res, '.png']));

fprintf('Bild nach %s gespeichert.\n', eval_path);