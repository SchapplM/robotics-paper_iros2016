% Start the Atlas Arm (v5) joint impctrl
% Test the Model holding a position and exerting external forces
% Run `atlas_joint_impctrl_sqrt_damping_test_extforce_noise.m` first.

% Jonathan Vorndamme, vorndamme@irt.uni-hannover.de, 2016-02
% (c) Institut für Regelungstechnik, Universität Hannover

clear
clc
close all
tb_path = fileparts(which('drc_paper_path_init.m'));


%% load data and store everything needed for plotting
filename = fullfile(tb_path, 'simulink', 'results', 'MdlNoise', sprintf('atlas5_impctrl_obs_FextO_MdlNoise_ref.mat'));
load(filename);
q_ref = sl.q;
noise_level=0:0.01:0.3;
plotdata.MdlNoise0(length(noise_level),10)=0;
plotdata.NoiseLevel0(length(noise_level),10)=0;
plotdata.MdlNoise1(length(noise_level),10)=0;
plotdata.NoiseLevel1(length(noise_level),10)=0;
for i=1:length(noise_level)
  for j=1:10
    for K_ext_o=0:1
      filename = fullfile(tb_path, 'simulink', 'results', 'MdlNoise', sprintf('atlas5_impctrl_obs_FextO_MdlNoise%.2f_Kexto%d_%d.mat',noise_level(i), K_ext_o, j));
      load(filename);
      plotdata.(['MdlNoise' num2str(K_ext_o)])(i,j) = sum(trapz(sl.t,(sl.q-q_ref).^2))/(7*(sl.t(end)-sl.t(1)));
      plotdata.(['NoiseLevel' num2str(K_ext_o)])(i,j) = noise_level(i);
    end
    if (mod(j,10)==0)
      fprintf('\b\b\b\b\b\b%2d,%3d',i,j);
    end
  end
end
fprintf('\b\b\b');

plotdata.MesNoise0(100)=0;
plotdata.MesNoise1(100)=0;
for i=1:100
  for K_ext_o=0:1
    filename = fullfile(tb_path, 'simulink', 'results', 'MesNoise', sprintf('atlas5_impctrl_obs_FextO_MesNoise_Kexto%d_%d.mat', K_ext_o, i));
    load(filename);
    plotdata.(['MesNoise' num2str(K_ext_o)])(i) = sum(trapz(sl.t,(sl.q-q_ref).^2))/(7*(sl.t(end)-sl.t(1)));
  end
  if (mod(i,10)==0)
    fprintf('\b\b\b%3d',i);
  end
end
fprintf('\b\b\b>> ');
filename = fullfile(tb_path, 'simulink', 'results', sprintf('atlas5_impctrl_obs_FextO_Noise.mat'));
save(filename,'plotdata');

%% Plot comparison
filename = fullfile(tb_path, 'simulink', 'results', sprintf('atlas5_impctrl_obs_FextO_Noise.mat'));
load(filename);

figure(1);

axhdl = NaN(2,1);
axhdl(1) = subplot(2,1,1);hold on;grid on;
plot(plotdata.NoiseLevel0(11:end)*100,plotdata.MdlNoise0(11:end)/max([plotdata.MdlNoise0(11:end), plotdata.MdlNoise1(11:end)]),'xb');
ylabel('$\|\mathrm{MSIE}\|$','interpreter','latex');

axhdl(2) = subplot(2,1,2);hold on;grid on;
plot(plotdata.NoiseLevel1(11:end)*100,plotdata.MdlNoise1(11:end)/max([plotdata.MdlNoise0(11:end), plotdata.MdlNoise1(11:end)]),'xb');
ylabel('$\|\mathrm{MSIE}\|$','interpreter','latex');
xlabel('Maximum relative model parameter noise [%]');

linkxaxes(1);
set_y_autoscale(1,0.05,false)
subplot_expand(1, 2, 1);

% Gesamt-Formatierung
figure_format_publication(axhdl)
set_size_plot_subplot(1, ...
  8.67, 6, ...
  axhdl, ...
  0.14, 0.02, 0.01, 0.15, ... % l r u d
  0.075, 0.04) % x y

figure(2);

plot(plotdata.MesNoise0(1:30)/max([plotdata.MesNoise0(1:30) plotdata.MesNoise1(1:30)]),'xb');hold on;grid on;
plot(plotdata.MesNoise1(1:30)/max([plotdata.MesNoise0(1:30) plotdata.MesNoise1(1:30)]),'sr');
axhdl = findall(2,'type','axes');
ylabel('$\|\mathrm{MSIE}\|$','interpreter','latex');
xlabel('Measurement noise amplification');
legend({'$\kappa_\mathrm{ext}=0$','$\kappa_\mathrm{ext}=1$'}, 'interpreter', 'latex'); 

set_y_autoscale(2,0.05,false)

% Gesamt-Formatierung
figure_format_publication(axhdl)
set_size_plot_subplot(2, ...
  8.67, 3.3, ...
  axhdl, ...
  0.11, 0.02, 0.01, 0.28, ... % l r u d
  0.075, 0.04) % x y

%% Bilder speichern
% save figures: Keine Berücksichtigung der externen Kraft im Beobachter
res_path = fullfile(tb_path, 'simulink', 'results', 'atlas_JIC_noise_extforce_cmp');
fig_path = fullfile(tb_path, 'paper', 'simulations');
mkdirs(res_path);
Filebasename_res = 'SimExp_MdlNoiseExtForce';
figure(1);
saveas(1, fullfile(res_path, [Filebasename_res, '.fig']));
export_fig(fullfile(res_path, [Filebasename_res, '.pdf']));
export_fig(fullfile(fig_path, [Filebasename_res, '.pdf']));
export_fig(fullfile(res_path, [Filebasename_res, '.eps']));
export_fig(fullfile(res_path, [Filebasename_res, '.png']));

fprintf('Nach %s gespeichert.\n', res_path);

Filebasename_res = 'SimExp_MesNoiseExtForce';
figure(2);
saveas(2, fullfile(res_path, [Filebasename_res, '.fig']));
export_fig(fullfile(res_path, [Filebasename_res, '.pdf']));
export_fig(fullfile(fig_path, [Filebasename_res, '.pdf']));
export_fig(fullfile(res_path, [Filebasename_res, '.eps']));
export_fig(fullfile(res_path, [Filebasename_res, '.png']));

fprintf('Nach %s gespeichert.\n', res_path);
