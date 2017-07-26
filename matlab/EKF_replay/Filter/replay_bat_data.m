clear all;
close all;

% add required paths
addpath('../Common');

% loop through files in directory
dataDirectory = '/Users/paul/Downloads/RFDesign Inertial Data/20170620/';
fileName = 'J.csv';
[VarName1,VarName2,VarName3,VarName4,VarName5,VarName6,VarName7,VarName8,VarName9,VarName10] = importBatData(strcat(dataDirectory,fileName), 1, 1e6);

% convert csv data
convertBatData;

clearvars -except imu_data;

% load default parameters
run('SetParameters.m');

% run the filter replay
output = RunFilter(param,imu_data);

% plot bat speed and g load
title('Bat speed and specific force');
xlabel('time (sec');
[AX,H1,H2] = plotyy(output.time_lapsed,output.speed,output.time_lapsed,output.g_load);
ylabel(AX(1),'speed (m/s)');
ylabel(AX(2),'acceleration (g)');
grid on;
