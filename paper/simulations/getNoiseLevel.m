% Helper script to determine noise levels for measured entities
clear all; close all; %#ok<CLSCR>
AC=atlas_const(5);
expdata = load_experiment_data('E:\DRC\matlab\experiments\atlas5_ImpCtrl\ImpCtrlv5_E061_R11_20150730_extforce_test\ImpCtrlv5_E061_R11_log_0.mat');
noise_list={'q','qD','tau'};
for i=1:length(noise_list)
  diff_max = 0;
  for j=1:4
    diff_max = max(diff_max, max(expdata.FromRobot.(noise_list{i})(1:4000,AC.JI_lArm(j)))-...
                             min(expdata.FromRobot.(noise_list{i})(1:4000,AC.JI_lArm(j))));
    noise_level.(noise_list{i})(j)=diff_max;
  end
end

diff_max = 0;
for i=1:3
  diff_max = max(diff_max, max(expdata.FromRobot.F_lH(1:4000,i))-...
                           min(expdata.FromRobot.F_lH(1:4000,i)));
  noise_level.F_lH(i)=diff_max;
end

g(3,4000)=0;
for i=1:4000
  R=quat2r(expdata.FromRobot.pelvis_x_r(i,:));
  g(:,i)=R'*[0;0;-9.81];
end

diff_max = 0;
for i=1:3
  diff_max = max(diff_max, max(g(i,:))-min(g(i,:)));
  noise_level.g(i)=diff_max;
end

noise_list={'q','qD','tau','F_lH', 'g'};
for i=1:length(noise_list)
  fprintf('Rauschamplitude fuer %4s: %7.2e.\n', noise_list{i}, 0.5*max(noise_level.(noise_list{i})));
end

