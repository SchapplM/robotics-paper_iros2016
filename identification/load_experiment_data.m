% Extract Data for Experiment with given filename
%
% Input:
% matfilepath [string]
%   Path to the experiment data. The mat file is converted from a bag file
% Indexes_To [Nx2 double]
%   Indexes for the data to the robot, which is chosen for extraction
%   Each row contains a start and end index
% Indexes_From_struct [Nx2 double]
%   Indexes for the data from the robot, which is chosen for extraction
%   Each row contains a start and end index
%
% Output:
% exp_struct
%   Structure with experiment data
%   .FromRobot
%     Structure with measurement data from the robot
%       .q        Joint Positions
%       .qD       Joint Velocities
%       .tau      Joint Efforts
%       .t        Timestamps
%   .ToRobot
%       same values as in .FromRobot
%

% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-02
% (c) Institut für Regelungstechnik, Universität Hannover


function [exp_struct, Load_struct] = load_experiment_data(matfilepath, Indexes_To, Indexes_From)

% load mat file
Load_struct = load(fullfile(matfilepath));

% calculate acceleration

Load_struct.FromRobot = filt_velocity_elec(Load_struct.FromRobot);
Load_struct.ToRobot = filt_velocity_elec(Load_struct.ToRobot);


Load_struct.FromRobot = calc_acceleration(Load_struct.FromRobot);
Load_struct.ToRobot = calc_acceleration(Load_struct.ToRobot);

% Filter Velocity
Load_struct.FromRobot = calc_filt_velocity(Load_struct.FromRobot);

% calculate electric motor torque 
% status = 1;
% try
% if isfield(Load_struct.FromRobot,'i_el') == 1 && status == 1
%   for joint = 1:6;
%     Load_struct.FromRobot = calc_torque_elec(Load_struct.FromRobot,Load_struct.ToRobot,joint,lr);
%   end
% end
% catch
%   warning('No electric motor currents available');
% end

% extend impedance controller settings to last timestamp
% change stored values for these structures:
settings_fields = {'ImpCtrlGains_left_arm', 'ImpCtrlGains_right_arm'};
for i = 1:length(settings_fields)
  if ~isfield(Load_struct.ToRobot, settings_fields{i})
    continue % old data does not contain this field
  end
  Strct = Load_struct.ToRobot.(settings_fields{i}); % temporary variable for changing
  % get fields of structure
  fields = fieldnames(Load_struct.ToRobot.ImpCtrlGains_left_arm);
  for j = 1:numel(fields)
    Tmp = Strct.(fields{j});
    % check if field is time dependant
    if length(Tmp) ~= length(Strct.t)
      continue
    end
    if strcmp(fields{j}, 't')
      continue % do not change the times here
    end
    if ~(isa(Strct.(fields{j}), 'double') || isa(Strct.(fields{j}), 'logical'))
      continue
    end
    % add last entry again
    Strct.(fields{j}) = [Tmp;Tmp(end)];
  end
  % add time stamp of last measured or commanded value
  Strct.t = [Strct.t; max([Load_struct.FromRobot.t(end), Load_struct.ToRobot.t(end)])];
  % write changed structure back
  Load_struct.ToRobot.(settings_fields{i}) = Strct;
end

% convert force torque sensor measurements to hand frames
% https://external.torcrobotics.com/redmine-drc/issues/2588
% rotate from sensor frame into hand frame
% assume rotation about z-axis
Load_struct.FromRobot.F_lH_raw = Load_struct.FromRobot.F_lH;
Load_struct.FromRobot.F_rH_raw = Load_struct.FromRobot.F_rH;

% Load_struct.FromRobot.F_lH(:,[1,2,4,5]) = -Load_struct.FromRobot.F_lH(:,[1,2,4,5]);

% if no indexes specified: return whole file
if nargin == 1
  exp_struct = Load_struct;
  return
end

% if indexes specified: Return only data in the intervals given by indexes
exp_struct.FromRobot = ExtractInterval(Load_struct.FromRobot, Indexes_From);
exp_struct.ToRobot = ExtractInterval(Load_struct.ToRobot, Indexes_To);


end

function EditStruct = filt_velocity_elec(EditStruct)
% Return Acceleration. Calculate if needed
% TODO: Filter Acceleration with kinematic mode kalman filter

global FilterElecVel
if isempty(FilterElecVel)
  return
end
% Differentation by four different methods
% Filter first and derivate afterwards or derivate first and filter
% afterwards. Filters can be done by smoothing or butterworth filtering
if isfield(EditStruct, 'qD')
  
  tic
  omega = FilterElecVel*2*pi;
  step_time = 1e-3;
  order = 2;
  switch order
    case 1
      constants = tf(omega,[1 omega]);
    case 2
      constants = tf(omega^2,[1 2*1*omega omega^2]);
  end
  constants_z=c2d(constants,step_time);
  [num_z,den_z]=tfdata(constants_z);
  b_filt = num_z{1};
  d_filt = den_z{1};
  
  
  AS = atlas_const(uint8(5), true);
  for i = [AS.JI_lArm((end-2):end),AS.JI_rArm((end-2):end)]
    qD_filt = filtfilt(b_filt,d_filt,EditStruct.qD(:,i));
    EditStruct.qD(:,i) = qD_filt;
  end
end
end


function EditStruct = calc_acceleration(EditStruct)
% Return Acceleration. Calculate if needed
% TODO: Filter Acceleration with kinematic mode kalman filter

global FilterAcc
if isempty(FilterAcc) % TODO: Globale Variable besser abfangen. Standard-Einstellung ermöglichen, falls nicht gesetzt
  FilterAcc = 20;
end
% Differentation by four different methods
% Filter first and derivate afterwards or derivate first and filter
% afterwards. Filters can be done by smoothing or butterworth filtering
if ~isfield(EditStruct, 'qDD') && isfield(EditStruct, 't')
  
  tic
  omega = FilterAcc*2*pi;
  step_time = 1e-3;
  order = 2;
  switch order
    case 1
      constants = tf(omega,[1 omega]);
    case 2
      constants = tf(omega^2,[1 2*1*omega omega^2]);
  end
  constants_z=c2d(constants,step_time);
  [num_z,den_z]=tfdata(constants_z);
  b_filt = num_z{1};
  d_filt = den_z{1};
  
  dt = diff(EditStruct.t);
  % remove dt=0-values (e.g. from Gazebo when two time stamps are equal)
  dt(abs(dt)<1e-6) = 1e-6;
  
  filter_befAft = 1; % filtfilt-error-msg. Take this as default.
  
  switch filter_befAft
    case 1
      % acceleration from numerical differentiation
      QDD_1 = [zeros(1,size(EditStruct.qD,2)); diff(EditStruct.qD)./repmat(dt,1,size(EditStruct.qD,2))];
      % Filter Acceleration
      for i = 1:size(QDD_1,2)
        QDD_1_sm(:,i) = smooth(QDD_1(:,i), 2000);
      end
      QDD = QDD_1_sm;
    case 2
      for i = 1:size(QDD_1,2)
        QDD_1_tp(:,i) = filtfilt(b_filt,d_filt,QDD_1(:,i));
      end
      QDD = QDD_1_tp;
    case 3
      for i = 1:size(EditStruct.qD,2)
        QD_2_sm(:,i) = smooth(EditStruct.qD(:,i), 3000);
      end
      
      QDD_2_sm = [zeros(1,size(QD_2_sm,2)); diff(QD_2_sm)./repmat(dt,1,size(QD_2_sm,2))];
      QDD = QDD_2_sm;
    case 4
      for i = 1:size(EditStruct.qD,2)
        QD_2_tp(:,i) = filtfilt(b_filt,d_filt,EditStruct.qD(:,i));
      end
      % acceleration from numerical differentiation
      QDD_2_tp = [zeros(1,size(QD_2_tp,2)); diff(QD_2_tp)./repmat(dt,1,size(QD_2_tp,2))];
      QDD = QDD_2_tp;
  end
  
  % old plotting information
  %         figure; plot(EditStruct.qD(:,17)); hold on; plot(QD_2_sm(:,17),'r'); hold on; plot(QD_2_tp(:,17),'g'); legend('speed_{orig}','speed_{sm}','speed_{tp}')
  %             figure;
  %             plot(QDD_1(:,17)); hold on;
  %             plot(QDD_1_sm(:,17),'m'); hold on;
  %             plot(QDD_1_tp(:,17),'k');
  %             plot(QDD_2_sm(:,17),'r');  hold on;
  %             plot(QDD_2_tp(:,17),'g'); hold on;
  %             legend('direct','sm_{after}','tp_{after}','sm_{bef}','tp_{bef}')
  %         QDD = QDD_2_tp;
  
  if size(QDD,1) > 400
    QDD(1:100,:) = 0;
    QDD(end-400:end,:) = 0;
  end
  EditStruct.qDD = QDD;
end

end

function EditStruct = calc_filt_velocity(EditStruct)
% Return Acceleration. Calculate if needed
% TODO: Filter Acceleration with kinematic mode kalman filter
QD = EditStruct.qD;
QD_filt = NaN(size(QD));
% Filter Velocity
for i = 1:size(QD,2)
  QD_filt(:,i) = smooth(QD(:,i), 100);
end
EditStruct.qD_filt = QD_filt;
end

function OutStruct = ExtractInterval(EditStruct, Ind)

% count number of rows
N = 0;
for j = 1:size(Ind,1)
  N = N + Ind(j,2) - Ind(j,1) + 1;
end

% Create structure with the same fields as the Input Structure
% The dimensions are already pre-assigned from the intervals above
fields = fieldnames(EditStruct);
OutStruct = struct('t', NaN(N,1));
for fnr=1:numel(fields)
  tmp = EditStruct.(fields{fnr});
  if size(tmp,1) == length(EditStruct.t)
    OutStruct.(fields{fnr}) = NaN(N,size(tmp,2));
  end
end

% Loop Through all Intervals given in the Indexes Matrix
% Extract Data for given Interval
ii = 1; % starting index
time_last = 0;
for i = 1:size(Ind,1)
  I1 = Ind(i,1); % Starting Index
  I2 = Ind(i,2); % end Index
  NN = I2-I1+1; % Number of entries
  
  % Loop over all fields and extract data, if field is a time series
  for fnr=1:numel(fields)
    tmp = EditStruct.(fields{fnr});
    if size(tmp,1) == length(EditStruct.t)
      
      if strcmp(fields{fnr},'t') == 1
        % avoid time to be discontinous
        
        difference = diff(tmp(I1:I2,:));
        difference(isnan(difference)) = [];
        t_sample = mean(difference);
        tmp_ = tmp(I1:I2,:) - tmp(I1);
        OutStruct.(fields{fnr})(ii:ii+NN-1,:) = tmp_+time_last+t_sample;
        time_last = tmp_(end)+time_last;
      else
        OutStruct.(fields{fnr})(ii:ii+NN-1,:) = tmp(I1:I2,:);
      end
    end
  end
  
  % Return Interval
  OutStruct.I(ii:ii+NN-1) = (I1:I2)';
  
  ii = ii + NN; % Starting Index in output struct for next iteration
end
end

% function FromRobot = calc_torque_elec(FromRobot,ToRobot,joint,lr)
% 
% % This function requires an impedance controlled identification where
% % desired torques are given to the robot. Based on the desired torque, the
% % sign of the measured torque is determined.
% 
% % Alexander Tödtheide, toedtheide@irt.uni-hannover.de, 2015-07
% % (c) Institut für Regelungstechnik, Universität Hannover
% 
% plotting = 0;
% 
% % load atlas constants
% AS = atlas_const(uint8(5), lr);
% 
% k_motor = AS.k_motor_elec;
% gear_ratio = AS.gear_ratio_elec;
% 
% % iterate over 
% % since there are two arms with three electric motors
%  
%   
%   indices = [AS.JI_lArm(end-2:end),AS.JI_rArm(end-2:end)];
%   
%   t_to = ToRobot.t;
%   t_from = FromRobot.t;
%   tau_est = k_motor*gear_ratio(joint)*FromRobot.i_el(:,indices(joint));
%   tau_des = ToRobot.tau(:,indices(joint));
%   
%   Intervals_des = (tau_des > 0);
%   
%   %% find time intervals where the indices changes
%   k = 2;
%   % start first interval edge with zero
%   t_switch(1) = 0;
%   first_interval_sign = 3;
%   for i = 2:size(tau_des,1)
%     % towards positive
%     if tau_des(i-1) < 0 && tau_des(i) > 0
%       t_switch(k) = t_to(i);
%       k = k + 1;
%       
%       % determine sign of first interval to known sign of other intervals
%       if first_interval_sign == 3
%         first_interval_sign = -1; % first interval is negative
%       end
%     end
%     
%     % determine sign of first interval to known sign of other intervals
%     if tau_des(i-1) > 0 && tau_des(i) < 0
%       t_switch(k) = t_to(i);
%       k = k + 1;
%       if first_interval_sign == 3
%         first_interval_sign = 1; % first interval is positive
%       end
%     end
%   end
%   % finish final interval edge with the last time value
%   t_switch(k) = t_to(end);
%   
%   % determine start/stop and sign of intervals
%   intervals_infos  = zeros(length(t_switch)-1,3);
%   for i = 1:length(t_switch)-1
%     intervals_infos(i,1) = t_switch(i);
%     intervals_infos(i,2) = t_switch(i+1);
%     % create alternating function by cosinus
%     intervals_infos(i,3) = round(cos((i-1)*pi)*first_interval_sign);
%   end
%   
%   %% Apply signs to measurement
%   k_last = 1;
%   for i = 1:length(t_from)
%     % find out interval
%     for k = k_last:length(intervals_infos)
%       if k < length(intervals_infos)
%         if t_from(i) >= intervals_infos(k,1) && t_from(i) <= intervals_infos(k+1,1)
%           sign = intervals_infos(k,3);
%           k_last = k;
%           break
%         end
%       else
%         if t_from(i) >= intervals_infos(k,1)
%           sign = intervals_infos(k,3);
%           k_last = k;
%           break
%         end
%       end
%     end  
%     tau_est_corr(i,1) = tau_est(i,1)*sign;
%   end
%   
%   %% determine final shift
%   tau_est_corr_out = (tau_est_corr);
%   FromRobot.tau(:,indices(joint)) = tau_est_corr_out;
% 
%   if plotting == 1
%     figure;
%     subplot(3,1,1);
%     plot(t_from,tau_est); xlabel('Time [s]'); ylabel('Torque [Nm]');
%     subplot(3,1,2);
%     p1 = plot(t_from,tau_est); hold on;
%     p2 = plot(t_to,tau_des); hold on;
%     p3 = plot(t_to(Intervals_des),tau_des(Intervals_des)); hold on; xlabel('Time [s]'); ylabel('Torque ');
%     p4 = plot(t_switch,zeros(length(t_switch),1),'or');
%     legend([p1,p2,p3,p4],{'k_{motor}*i_{gear}*I','t_{des}','Indexbased','Interval edges'})
%     subplot(3,1,3);
%     p1 = plot(t_from, tau_est_corr_out); hold on;
%     p2 = plot(t_to,tau_des); hold on; legend([p1, p2],'tau_{est}','tau_{des}'); xlabel('Time [s]'); ylabel('Torque ');
%   end
% 
% end
