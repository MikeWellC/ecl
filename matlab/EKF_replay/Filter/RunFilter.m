function output = RunFilter(param,imu_data)

% compulsory inputs

% param : parameters defined  by SetParameterDefaults.m
% imu_data : IMU delta angle and velocity data in body frame

%% Set initial conditions

% constants
gravity = 9.80665; % initial value of gravity - will be updated when WGS-84 position is known

% initialise the state vector
[states, imu_start_index] = InitStates(imu_data);

dt_imu_avg = 0.5 * (median(imu_data.gyro_dt) + median(imu_data.accel_dt));
indexStop = length(imu_data.time_us) - imu_start_index;
indexStart = 1;

% create static structs for output data
output = struct('time_lapsed',[]',...
    'euler_angles',[],...
    'velocity_NED',[],...
    'gyro_bias',[],...
    'accel_bias',[],...
    'dt',0,...
    'state_variance',[],...
    'innovations',[]);

% initialise the state covariance matrix
covariance = InitCovariance(param,dt_imu_avg);

%% Main Loop

% array access index variables
imuIndex = imu_start_index;
innovIndex = 1;
for index = indexStart:indexStop
    
    % read IMU measurements
    local_time=imu_data.time_us(imuIndex)*1e-6;
    delta_angle(:,1) = imu_data.del_ang(imuIndex,:);
    delta_velocity(:,1) = imu_data.del_vel(imuIndex,:);
    dt_imu = 0.5 * (imu_data.accel_dt(imuIndex) + imu_data.gyro_dt(imuIndex));
    imuIndex = imuIndex+1;
    
    % predict states
    [states, delAngCorrected, delVelCorrected]  = PredictStates(states,delta_angle,delta_velocity,imu_data.accel_dt(imuIndex),gravity);
    
    % constrain states
    [states]  = ConstrainStates(states,dt_imu_avg);
    
    % predict covariance
    covariance  = PredictCovariance(delAngCorrected,delVelCorrected,states,covariance,dt_imu,param);
    
    % output state data
    output.time_lapsed(index) = local_time;
    output.euler_angles(index,:) = QuatToEul(states(1:4)')';
    output.velocity_NED(index,:) = states(5:7)';
    output.speed(index) = sqrt(dot(states(5:7),states(5:7)));
    output.gyro_bias(index,:) = states(8:10)';
    output.accel_bias(index,:) = states(11:13)';
    
    % output covariance data
    for i=1:13
        output.state_variances(index,i) = covariance(i,i);
    end
    
    % output equivalent euler angle variances
    error_transfer_matrix = quat_to_euler_error_transfer_matrix(states(1),states(2),states(3),states(4));
    euler_covariance_matrix = error_transfer_matrix * covariance(1:4,1:4) * transpose(error_transfer_matrix);
    for i=1:3
        output.euler_variances(index,i) = euler_covariance_matrix(i,i);
    end
    
    % fuse in a zero velocity to constrain attitude drift if the g level is
    % below 2
    accel_vec = delVelCorrected / dt_imu;
    if (dot(accel_vec,accel_vec) < (2*gravity)^2)
        gate = 5;
        vel_noise = 10;
        [states,covariance,velInnov,velInnovVar] = FuseVelocity(states,covariance,[0 0 0],gate,vel_noise);

        % data logging
        output.innovations.vel_time_lapsed(innovIndex) = local_time;
        output.innovations.vel_innov(innovIndex,:) = velInnov';
        output.innovations.vel_innov_var(innovIndex,:) = velInnovVar';
        innovIndex = innovIndex+1;
    end
    
        
end

end