% Run all scripts which reproduce the simulation figures in this repository

% Moritz Schappler, schappler@irt.uni-hannover.de, 2020-12
% (C) Institut für Regelungstechnik, Leibniz Universität Hannover

clear
clc
close all

tb_path = fileparts(which('drc_paper_path_init.m'));
sim_path = fullfile(tb_path, 'paper', 'simulations');
run(fullfile(sim_path, 'atlas_joint_impctrl_sqrt_damping_test_extforce_comp.m'));
tb_path = fileparts(which('drc_paper_path_init.m'));
sim_path = fullfile(tb_path, 'paper', 'simulations');
run(fullfile(sim_path, 'atlas_joint_impctrl_sqrt_damping_test_extforce_noise.m'));
tb_path = fileparts(which('drc_paper_path_init.m'));
sim_path = fullfile(tb_path, 'paper', 'simulations');
run(fullfile(sim_path, 'atlas_joint_impctrl_sqrt_damping_test_extforce_noise_plot.m'));