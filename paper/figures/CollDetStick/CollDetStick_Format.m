% Automatisch erstellte Bilder mit Beobachter-Daten aus Versuch E054
% (Gegendrücken mit Stab an Struktur) öffnen und neu formatieren
% 
% Vorher folgendes Skript ausführen:
% experiments/evaluation_settings/
% atlas_identification_experiment_test_settings_ImpCtrlE055_R04.m

% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-08
% (c) Institut für Regelungstechnik, Universität Hannover
%% Init
tb_path = fileparts(which('drc_paper_path_init.m')); 
close all

% Import-Pfad für vorher generierte Bilder
eval_path = fullfile(tb_path, 'experiments', 'eval_atlas5', ...
  'ImpCtrlv5_E055');

%% Teilbild 1: Achse 1 Drehmomente
% Speichern der xy-Daten
X = cell(2,1);
Y = cell(2,1);

%% tau_mess, tau_modell laden
figinputpath = fullfile(eval_path, ['R04_torque', '.fig']);
uiopen(figinputpath,1);
fig_i = gcf;
ch_f = get(fig_i, 'Children');
% alle subplots bis auf shz löschen
delete(ch_f(1:end-1))

% Datenreihen speichern
ch = get(gca, 'Children');
% ID
X{1} = get(ch(5), 'XData');
Y{1} = get(ch(5), 'YData');
% ext
X{2} = get(ch(2), 'XData');
Y{2} = get(ch(2), 'YData');
% obs
X{3} = get(ch(3), 'XData');
Y{3} = get(ch(3), 'YData');
% ist
X{4} = get(ch(7), 'XData');
Y{4} = get(ch(7), 'YData');

%% positionsdaten laden
figinputpath = fullfile(eval_path, ['R04_position', '.fig']);
uiopen(figinputpath,1);
fig_i = gcf;
ch_f = get(fig_i, 'Children');
% alle subplots bis auf shz löschen
delete(ch_f(1:end-1))

% Datenreihen speichern
ch = get(gca, 'Children');
% ist
X{5} = get(ch(4), 'XData');
Y{5} = get(ch(4), 'YData');
% soll
X{6} = get(ch(3), 'XData');
Y{6} = get(ch(3), 'YData');
%% zusammengesuchte Daten neu plotten
fig_ges = figure();
linhdl_pos = NaN(2,1);

% Positions-Plot
ax1=subplot(2,1,1);hold on;grid on;
plot(X{5}, Y{5}, 'r-');
I_Ausw = round(linspace(1, length(X{5}), 25)); % Indizes zur Auswahl: Alle 25 Datenpunkte ein Marker
plot(X{5}(I_Ausw), Y{5}(I_Ausw), 'rs');
linhdl_pos(1) = plot(NaN, NaN, 'r-s'); % Für Legende

linhdl_pos(2) = plot(X{6}, Y{6}, 'k--');

l1 = legend(linhdl_pos(1:2), {'$q_1$', '$q_{\mathrm{d},1}$'}, 'interpreter', 'latex');

ylabel('$q_{1}$ [rad]', 'interpreter', 'latex');

% Drehmoment-Plot
linhdl_tau = NaN(3,1);
ax2=subplot(2,1,2);hold on;grid on;
% ID
linhdl_tau(1)=plot(X{1}, Y{1});

% % Ext
% plot(X{2}, Y{2}, 'r-');

% obs
linhdl_tau(2)=plot(X{3}, Y{3});

% ist
linhdl_tau(3)=plot(X{4}, Y{4});

leghdl = line_format_publication(linhdl_tau, [], {'ID', 'obs', 'ist'});

l2 = legend(leghdl, {'$\tau_{\mathrm{mdl},1}$', '$-\hat{\tau}_{\varepsilon,1}$', ...
  '$\tau_{\mathrm{m},1}$'}, 'interpreter', 'latex');

ylabel('$\tau_{1}$ [Nm]', 'interpreter', 'latex');
xl = xlabel('$t$ [s]', 'interpreter', 'latex');


%% Formatieren
linkxaxes

change_x_data(fig_ges, -8);
xlim([0, 7]);
subplot_expand(fig_ges, 2, 1);
set_y_autoscale(fig_ges,0.1)

% Vor-Formatierung
figure_format_publication([ax1,ax2])
set_sizePositionPlotSubplot(gcf, ...
  8.67, 5.5, ...
  [2 1], ...
  0.15, 0.02, 0.02, 0.17, ... % l r u d
  0.075, 0.05) % x y

% Größe nochmals anpassen
ax1pos = get(ax1, 'position'); 
ax2pos = get(ax2, 'position'); 
set(fig_ges, 'papersize',[8.67 5.0], 'pos', [0 0 8.67 4.5])
set(ax1, 'position', [ax1pos(1), 0.68, ax1pos(3), 0.3]);
set(ax2, 'position', [ax2pos(1:3), 0.45]);

% Legende
% , 'orientation', 'horizontal'
set(l1, 'Position', [0.44, 0.78, .25, .15])
set(l2, 'Position', [0.44, 0.43, .25, .07], 'orientation', 'vertical')

set(xl, 'Position', [3.5, -13])

% Speichern
Filebasename_res = 'CollDetStickIROS';
saveas(fig_ges, fullfile(eval_path, [Filebasename_res, '.fig']));
export_fig(fullfile(eval_path, [Filebasename_res, '.pdf']));
export_fig(fullfile(eval_path, [Filebasename_res, '.eps']));
export_fig(fullfile(eval_path, [Filebasename_res, '.png']));

fprintf('Nach %s gespeichert.\n', eval_path);
