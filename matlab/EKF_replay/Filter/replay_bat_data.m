clear all;
close all;

% add required paths
addpath('../Common');

% loop through files in directory
dataDirectory = '/Users/paul/Downloads/RFDesign Inertial Data/20170620/';
fileName = 'D.csv';
[VarName1,VarName2,VarName3,VarName4,VarName5,VarName6,VarName7,VarName8,VarName9,VarName10] = importBatData(strcat(dataDirectory,fileName), 1, 1e6);

% convert csv data
convertBatData;

clearvars -except imu_data;

% covariance initialisation parameters
param.alignment.quatErr = 0.1; % Initial 1SD uncertainty in quaternion.
param.alignment.velErrNE = 5.0; % Initial 1SD velocity error when aligning without GPS. (m/sec)
param.alignment.velErrD = 1.0; % Initial 1SD vertical velocity error when aligning without GPS. (m/sec)
param.alignment.delAngBiasErr = 0.05*pi/180; % Initial 1SD rate gyro bias uncertainty. (rad/sec)
param.alignment.delVelBiasErr = 0.07; % Initial 1SD accelerometer bias uncertainty. (m/sec^2)

% process noise parameters
param.prediction.dAngBiasPnoise = 0.001; % IMU gyro bias 1SD rate of change (rad/sec^2)
param.prediction.dVelBiasPnoise = 0.03; % IMU accel bias 1SD rate of change (m/sec^3)
param.prediction.angRateNoise = 0.015; % IMU gyro 1SD rate process noise (rad/sec)
param.prediction.accelNoise = 0.35; % IMU accelerometer 1SD error noise including switch on bias uncertainty. (m/sec^2)

% velocity fusion parameters
param.fusion.gate = 3.0; % Size of the velocity innovation consistency check gate in SD
param.fusion.noise = 5.0; % Observation noise 1SD for the velocity observation (m/sec)
param.fusion.g_max = 4.0; % Maximum acceleration for which velocity fusion will be used to correct tilt errors (g)
param.fusion.g_max_time = 1.0; % time required to pass g check before velocity fusion will resume (sec)
param.fusion.reset_time = 2.0; % maximum time without fusion and g checks passed before the filter is reset (sec)

% state alignment parameters
param.alignment.reset_g_max = 0.1; % maximum accel variation from 1g permitted for a filter reset

% run the filter replay
output = RunFilter(param,imu_data);

plotDimensions = [0 0 210*3 297*3];
figure('Units','Pixels','Position',plotDimensions,'PaperOrientation','portrait');
h=gcf;
set(h,'PaperOrientation','portrait');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
margin = 5;
% plot bat speed and g load
title('Bat speed and specific force');
xlabel('time (sec');
[AX,H1,H2] = plotyy(output.time_lapsed,output.speed,output.time_lapsed,output.g_load);
ylabel(AX(1),'speed (m/s)');
ylabel(AX(2),'acceleration (g)');
grid on;

% heuristic to find peak bat speed using peak g event

% find index for peak g event
peak_g_index = find(output.g_load == max(output.g_load));

% search 0.1 seconds either side for peak speed
min_index = max((peak_g_index-25),1);
max_index = min((peak_g_index+25),length(output.g_load));
peak_speed_index = find(output.speed == max(output.speed(min_index:max_index)));
peak_bat_speed = output.speed(peak_speed_index);
fprintf('peak bat speed = %6.1f m/s\n',peak_bat_speed);
