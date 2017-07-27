function [states, imu_start_index] = InitStates(imu_data)

% initialise the state vector and quaternion
states = zeros(13,1);
quat = [1;0;0;0];

imu_start_index = 1;

% average first 100 accel readings to reduce effect of vibration
initAccel(1) = mean(imu_data.accel(imu_start_index:imu_start_index+99,1));
initAccel(2) = mean(imu_data.accel(imu_start_index:imu_start_index+99,2));
initAccel(3) = mean(imu_data.accel(imu_start_index:imu_start_index+99,3));

% align tilt using gravity vector (If the velocity is changing this will
% induce errors)
quat = AlignTilt(quat,initAccel);
states(1:4) = quat;

end