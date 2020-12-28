% Plot results from collision experiment E057 (Cinderblock) in one figure
% See https://external.torcrobotics.com/redmine-drc/projects/drc/wiki/List_of_ImpCtrl_Experiments_Atlas5#ImpCtrlv5_E057
% 
% first, execute:
% experiments/evaluation_settings/atlas_experiment_comparison_settings_ImpCtrlE057_coll.m 

% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-08
% (c) Institut für Regelungstechnik, Universität Hannover

clear
clc
close all

tb_path = fileparts(which('drc_paper_path_init.m')); 
eval_path = fullfile(tb_path, 'experiments', 'eval_atlas5', 'ImpCtrlv5_E057');

%% Collision: Forces

Filebasename = 'ImpCtrl_E057_coll_FT';
figinputpath = fullfile(eval_path, [Filebasename, '.fig']);
uiopen(figinputpath,1);

xlim([6,10]);
% get subplot for |F|
ch = get(gcf, 'children');
delete(ch([1, 3:end]));

Fnorm_path = fullfile(eval_path, [Filebasename, '_Fnorm.fig']);
saveas(gcf, Fnorm_path);


%% Collision: Observer shz
Filebasename = 'ImpCtrl_E057_coll_tau_obs';
figinputpath = fullfile(eval_path, [Filebasename, '.fig']);
uiopen(figinputpath,1);
xlim([6,10]);
ch = get(gcf, 'children');
delete(ch(1:7));
obs_shz_path = fullfile(eval_path, [Filebasename, '_shz.fig']);
saveas(gcf, obs_shz_path);

%% Collision: Position xyz
Filebasename = 'ImpCtrl_E057_coll_pos';
figinputpath = fullfile(eval_path, [Filebasename, '.fig']);
uiopen(figinputpath,1);
xlim([6,10]);
ch = get(gcf, 'children');
delete(ch(2:7));
pos_shz_path = fullfile(eval_path, [Filebasename, '_shz.fig']);
saveas(gcf, pos_shz_path);

%% Gesamtbild
close all

fighdl = figure(1);clf;
axhdl = set_fig2subfig(fighdl, {Fnorm_path; obs_shz_path; pos_shz_path});

linkxaxes
subplot_expand(fighdl, 3, 1, axhdl);
figure_format_publication(axhdl);

% Formatieren: Subplot 1
axes(axhdl(1));grid on;hold on;
ylabel(sprintf('$|f_{\\mathrm{ext,EE}}|$ [N]'), 'interpreter', 'latex');
ylim([0,100])
linhdl = get(axhdl(1), 'children');
delete(linhdl(1)); % BDI-Versuch
leghdl = line_format_publication(linhdl(2:4), [], {'Stiff', 'CollDet.', 'Comp.'});

h = legend(leghdl, ...
  {'Stiff', 'Coll. Det.', 'Compliant'});
set(h, 'Position', [0.50, 0.87, .10, .15], 'orientation', 'horizontal') % x, y, h, b

% Formatieren: Subplot 2
axes(axhdl(2));grid on;hold on;
ylabel(sprintf('$\\tau_{\\varepsilon,1}$ [Nm]'), 'interpreter', 'latex');
linhdl = get(axhdl(2), 'children');
delete(linhdl([1 2 4 5 7 8 10 11])); % Treppen-Plots für Schwellwert
delete(linhdl(3)) % BDI-Versuch, sowieso keine Daten
linhdl_rest = get(axhdl(2), 'children');

line_format_publication(linhdl_rest, [], {'Stiff', 'CollDet.', 'Comp.'});

% Schwellwert einzeichnen
leghdl = plot([0;100], [-11.1;-11.1], 'k--');
h=legend(leghdl, {'$\zeta$'}, 'interpreter', 'latex');
set(h, 'Position', [0.21, 0.38, .10, .06]) % x, y, b, h

axes(axhdl(3));grid on;hold on;
ylabel(sprintf('$q_{1}$ [rad]'), 'interpreter', 'latex');
% delete superfluous lines
linhdl = get(axhdl(3), 'children');
delete(linhdl([2 3 5 7])) % Überflüssige Linien löschen (u.a. BDI-Versuch)
line_format_publication(linhdl([4,6,8]), [], {'Stiff', 'CollDet.', 'Comp.'});
h=legend(linhdl(1), {'$q_{\mathrm{d}}$'}, 'interpreter', 'latex');
set(h, 'Position', [0.21, 0.13, .10, .06]) % x, y, b, h

xlabel('$t$ [s]', 'interpreter', 'latex');

xl = get(axhdl(3), 'XLABEL');
set(xl, 'Position', [1.25, -1.15, -1]);

% Alle x-Werte verschieben
change_x_data(fighdl, -7.5)
xlim([0, 2.5]);

% Größe des Plots maximieren
set_sizePositionPlotSubplot(gcf, ...
  8.7, 6, ...
  [3 1], ...
  0.15, 0.03, 0.13, 0.10, ... % l, r, u, d
  0.065, 0.03) % dx, dy

export_fig(gcf, fullfile(eval_path, 'Collision_Summary_Cinderblock.fig'));
export_fig(gcf, fullfile(eval_path, 'Collision_Summary_Cinderblock.eps'));
export_fig(gcf, fullfile(eval_path, 'Collision_Summary_Cinderblock.pdf'));
export_fig(gcf, fullfile(eval_path, 'Collision_Summary_Cinderblock.png'));

fprintf('Bild nach %s gespeichert.\n', eval_path);