% Create pretty figures from friction characteristics
% 
% % Before: Execute in identification/friction_settings:
%   identification_friction_single_jointsettings_Joint1_dyn.m
%   identification_friction_single_jointsettings_Joint2_dyn.m
%   identification_friction_single_jointsettings_Joint3_dyn.m
%   identification_friction_single_jointsettings_Joint4_dyn.m
%   identification_friction_single_jointsettings_Joint5_dyn.m
%   identification_friction_single_jointsettings_Joint6_dyn.m
%   identification_friction_single_jointsettings_Joint7_dyn.m
% 
% Reads Input:
% experiments/eval_atlas5/results_arm_ident_friction/...
% 
% Creates Output:
% experiments/eval_atlas5/FrictionCharacteristics/...
% 
% Siehe auch (alte Version dieses Skripts):
% vigir_final_report/FrictionCharacteristics_resultfigures.m

% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-08
% (c) Institut für Regelungstechnik, Universität Hannover

tb_path = fileparts(which('drc_paper_path_init.m')); 
figin_path = fullfile(tb_path, 'experiments', 'eval_atlas5','results_arm_ident_friction');
figout_path = fullfile(tb_path, 'experiments', 'eval_atlas5', 'FrictionCharacteristics');

% find newest friction characteristic for each joint
atlas_version = uint8(5);

input_figures = cell(2,2); % Anordnung der Subplots. Transponiert zu späterer Darstellung, da
% Spalten- und Zeilenindizes der einzelnen Gelenke in den Subplot-Handles
% Die Gelenke sollen Zeilenweise ausgegeben werden (erst Zeile 1 vollst., ...)
[IC,IR] = ind2sub([2,2],1:4); 

% Matrix mit Reibungsparametern aus Identifikation
PI_viskos = NaN(7,1);
PI_Coulomb = NaN(7,1);
IJ_plot = [1 2 3 4]; % Indizes der zu plottenden Gelenke (nur hydraulische)
for lr = [true, false]
  %% init
  if lr == true
    lrString = 'left';
  else
    lrString = 'right';
  end
  AS = atlas_const(atlas_version, lr);
  %% look for figure files from friction identification script
  for ij = IJ_plot
    % joint name
    jname = AS.JN{AS.JI_Arm(ij)};
    % all friction characteristics plots. Korrigierte Kennlinie: Abzug des
    % Gravitationsmomentes
    res1 = dir(fullfile(figin_path, sprintf('*%s_*Kennlinie_Korr.fig', jname)));
    if isempty(res1)
      res1 = dir(fullfile(figin_path, jname, sprintf('*%s_*Kennlinie_Korr.fig', jname)));
      figin_filepath = fullfile(figin_path, jname, res1(end).name);
    else
      % Nehme die letzte Auswertungsdatei
      figin_filepath = fullfile(figin_path, res1(end).name);      
    end
    if isempty(res1)
      error('no friction characteristics found for %s', jname);
    end
    input_figures{IR(ij),IC(ij)} = figin_filepath;
    fprintf('[%d,%d] %s\n', IC(ij),IR(ij),figin_filepath);
    
    % Lade die Identifikationsergebnisse zu diesem Bild
    k = strfind(figin_filepath, 'Kennlinie_Korr.fig');
    resmatfile = [figin_filepath(1:k-1), 'results.mat'];
    tmp = load(resmatfile);
    PI_viskos(ij) = tmp.Identification_Results.PV_viscous_PI_curve_corr;
    PI_Coulomb(ij) = tmp.Identification_Results.PV_Coulomb_PI_curve_corr;
  end

  %% Gesamtbild
  close all
  fighdl = figure(1);clf;
  axhdl = set_fig2subfig(fighdl, input_figures);
  for ij = IJ_plot
    axes( axhdl( IR(ij), IC(ij) ) );
    grid on;
    jn = AS.JN{AS.JI_Arm(ij)}; % Gelenkname als string

    ylabel(sprintf('$\\tau_{%d}$ (%s) [Nm]', ij, jn(7:end) ), 'interpreter', 'latex');
    xlabel(sprintf('$\\dot{q}_{%d}$ [rad/s]', ij), 'interpreter', 'latex');
    % remove identification line from all data points (contains too much low
    % velocity data)
    linhdl = get(axhdl( IR(ij), IC(ij) ), 'children');
    delete(linhdl(2));
    xlim([-2.5, 2.5]);
  end
   
  
  % change color of lines
  objs = findobj(axhdl, 'Type', 'line');
  for iii = 1:length(objs)
      if strcmp(get(objs(iii), 'LineStyle'), '-')
          set(objs(iii), 'Color', 'r')
      end
  end
  
  %% Figure adjustments
  figure_format_publication(axhdl);
  
  % Grenzen automatisch ändern
  set_y_autoscale(fighdl,0.1)

  % Größe des Plots maximieren
  set_size_plot_subplot(fighdl, ...
    8.5, 6.5, ...
    axhdl, ...
    0.12, 0.008, 0.11, 0.10, ... % l, r, u, d
    0.15, 0.1) % dx, dy

  h = legend(linhdl([1,3]), {'model','measurement'});
  rect = [0.5 0.92 0.03 0.06]; % x, y, h, b
  set(h, 'Position', rect, 'Orientation', 'horizontal')
  
  % x-Position der y-Achsenbeschriftungen ändern
  YL = NaN(size(axhdl));
  for i = 1:length(axhdl(:));
    [X_off, X_slope] = get_relative_position_in_axes(axhdl(i), 'x');
    [Y_off, Y_slope] = get_relative_position_in_axes(axhdl(i), 'y');
    YL(i) = get(axhdl(i), 'YLABEL');
    set(YL(i), 'POSITION', [X_off+X_slope*(-1.4), Y_off+Y_slope*(0) 1]);
  end
  
  % Mittleren Xticklabel ("0") entfernen
  for i = 1:length(axhdl(:))
    xtl = get(axhdl(i), 'XTICKLABEL');
    xtl{2} = '';
    set(axhdl(i), 'XTICKLABEL', xtl);
  end
  % Xlabel nach oben verschieben (wo vorher die mittlere Achsenbeschriftung
  % war
  XL = NaN(size(axhdl));
  for i = 1:length(axhdl(:));
    % Umwandlung von Achseneinheiten ([Nm], für jeden Plot anders) zu
    % normierten Einheiten (-1 bis 1)
    [Y_off, Y_slope] = get_relative_position_in_axes(axhdl(i), 'y');
    XL(i) = get(axhdl(i), 'XLABEL');
    set(XL(i), 'POSITION', [0, Y_off+Y_slope*(-1.2), 1]);
  end
  %% Export

  % speichern
  mkdirs(figout_path);
  export_fig(fighdl, fullfile(figout_path, sprintf('IROS_FrictionCharacteristics_%s.fig', lrString)));
  export_fig(fighdl, fullfile(figout_path, sprintf('IROS_FrictionCharacteristics_%s.eps', lrString)));
  export_fig(fighdl, fullfile(figout_path, sprintf('IROS_FrictionCharacteristics_%s.pdf', lrString)));
  export_fig(fighdl, fullfile(figout_path, sprintf('IROS_FrictionCharacteristics_%s.png', lrString)));
  fprintf('Nach %s gespeichert\n', figout_path);
  
  return
end