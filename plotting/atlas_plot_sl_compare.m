% Unterschiedliche Durchläufe der Simulink-Simulation plotten
% 
% Starten aufgerufen von anderem Skript
% Übergabe:
% PlotSettingsStruct
%   Struktur mit Einstellungen zum Plotten
% 
% Erstellt Bilder:
% 1 q
% 2 qD
% 3 qDD
% 4 E
% 5 tau_obs
% 6 tau_ext
% 7 F_ext
% 8 tau_m

% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-08
% (c) Institut für Regelungstechnik, Universität Hannover

files = PlotSettingsStruct.Files;
names = PlotSettingsStruct.Names;
I = PlotSettingsStruct.I;
atlas_version = PlotSettingsStruct.atlas_version;
AS = atlas_const(atlas_version);
%% Init
xyz = {'x', 'y', 'z'};
FM = {'Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz'};
%% 
for i = [1 2 3 4 5 6 7 8]
  figure(i);clf;
end
for i_File = 1:length(files)
  
  %% Daten laden
  load(files{i_File});
  
  %% Gelenkwinkel zeichnen
  change_current_figure(1);
  set(1, 'Name', 'q', 'NumberTitle', 'off');
  if i_File == 1, clf; end
  for i = 1:size(sl.q,2)
    subplot(3,3,i);hold all;
    plot(sl.t, 180/pi*sl.q(:,i));
%     ylabel(sprintf('$q_%d \\text{[deg](%s)}$', i, AS.JN{I(i)}), 'interpreter', 'latex');
    ylabel(sprintf('$q_%d$ [deg]', i), 'interpreter', 'latex');
    grid on;
  end
  linkxaxes
  if i_File==length(files), legend(names, 'interpreter', 'none'); end  %% Gelenkwinkel zeichnen
  
  %% Winkelgeschwindigkeit
  change_current_figure(2);
  set(2, 'Name', 'qD', 'NumberTitle', 'off');
  if i_File == 1, clf; end
  for i = 1:size(sl.q,2)
    subplot(3,3,i);hold all;
    plot(sl.t, sl.qD(:,i));
%     ylabel(sprintf('$q_%d \\text{[deg](%s)}$', i, AS.JN{I(i)}), 'interpreter', 'latex');
    ylabel(sprintf('qD_%d [rad/s]', i), 'interpreter', 'tex');
    grid on;
  end
  linkxaxes
  if i_File==length(files), legend(names, 'interpreter', 'none'); end
  
  %% Beschleunigung
  change_current_figure(3);
  set(3, 'Name', 'qDD', 'NumberTitle', 'off');
  if i_File == 1, clf; end
  for i = 1:size(sl.q,2)
    subplot(3,3,i);hold all;
    plot(sl.t, sl.qDD(:,i));
%     ylabel(sprintf('$q_%d \\text{[deg](%s)}$', i, AS.JN{I(i)}), 'interpreter', 'latex');
    ylabel(sprintf('qDD_%d [rad/s²]', i), 'interpreter', 'tex');
    grid on;
  end
  linkxaxes
  if i_File==length(files), legend(names, 'interpreter', 'none'); end
  
  %% Energie
  change_current_figure(4);
  set(4, 'Name', 'E', 'NumberTitle', 'off');
  Label = {'T', 'U', 'E_ges'};
  if i_File == 1, clf; end
  for i = 1:3
    subplot(3,1,i);hold all;
    plot(sl.t, sl.E(:,i));
    grid on;
    ylabel(sprintf('%s [J]', Label{i}), 'interpreter', 'tex');
  end
  linkxaxes
  if i_File==length(files), legend(names, 'interpreter', 'none'); end

  %% Beobachter-Drehmoment
  change_current_figure(5);
  set(5, 'Name', 'tau_obs', 'NumberTitle', 'off');
  if i_File == 1, clf; end
  for i = 1:size(sl.tau_obs,2)
    subplot(3,3,i);hold all;
    plot(sl.t, sl.tau_obs(:,i));
    ylabel(sprintf('\\tau_{obs,%d} [Nm]', i), 'interpreter', 'tex');
    grid on;
  end
  linkxaxes
  if i_File==length(files), legend(names, 'interpreter', 'none'); end
  
  %% Drehmoment aus externen Kräften
  change_current_figure(6);
  set(6, 'Name', 'tau_ext', 'NumberTitle', 'off');
  if i_File == 1, clf; end
  for i = 1:size(sl.tau_ext,2)
    subplot(3,3,i);hold all;
    plot(sl.t, sl.tau_ext(:,i));
    ylabel(sprintf('\\tau_{ext,%d} [Nm]', i), 'interpreter', 'tex');
    grid on;
  end
  linkxaxes
  if i_File==length(files), legend(names, 'interpreter', 'none'); end
  
  %% Externe Kräfte
  change_current_figure(7);
  set(7, 'Name', 'F_ext', 'NumberTitle', 'off');
  if i_File == 1, clf; end
  for i = 1:size(sl.out_F0_ext,2)
    subplot(3,2,i);hold all;
    plot(sl.t, sl.out_F0_ext(:,i));
    ylabel(sprintf('%s [N]/[Nm]', FM{i}), 'interpreter', 'tex');
    grid on;
  end
  linkxaxes
  if i_File==length(files), legend(names, 'interpreter', 'none'); end

  %% Motor-Drehmoment
  change_current_figure(8);
  set(8, 'Name', 'tau_m', 'NumberTitle', 'off');
  if i_File == 1, clf; end
  for i = 1:size(sl.tau_m,2)
    subplot(3,3,i);hold all;
    plot(sl.t, sl.tau_m(:,i));
    ylabel(sprintf('\\tau_{m,%d} [Nm]', i), 'interpreter', 'tex');
    grid on;
  end
  linkxaxes
  if i_File==length(files), legend(names, 'interpreter', 'none'); end
end

dockall