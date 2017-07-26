clear all;
close all;

% add required paths
addpath('../Common');

% loop through files in directory
dataDirectory = '/Users/paul/Downloads/RFDesign Inertial Data/20170620/';
fileName = 'I.csv';
[VarName1,VarName2,VarName3,VarName4,VarName5,VarName6,VarName7,VarName8,VarName9,VarName10] = importBatData(strcat(dataDirectory,fileName), 1, 1e6);

% convert csv data
convertBatData;

clearvars -except imu_data;

% load default parameters
run('SetParameters.m');

% run the filter replay
output = RunFilter(param,imu_data);

plot(output.time_lapsed,output.speed);
grid on;
ylabel('speed m/s');
xlabel('time (sec');