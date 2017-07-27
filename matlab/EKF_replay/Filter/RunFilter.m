function output = RunFilter(param,imu_data)

% param : parameters defined  by SetParameterDefaults.m
% imu_data : IMU delta angle and velocity data in body frame

%% Set initial conditions

% constants
gravity = 9.80665; % initial value of gravity - will be updated when WGS-84 position is known

% initialise the state vector
[states, imu_start_index] = InitStates(imu_data);

dt_imu_avg = 0.004;
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

imuIndex = imu_start_index;
innovIndex = 1;
lastVelFuseTime = imu_data.time_us(imu_start_index)*1e-6;
lastHighAccelTime = imu_data.time_us(imu_start_index)*1e-6 - 2;
local_time_prev = imu_data.time_us(imu_start_index)*1e-6;
for index = indexStart:indexStop
    
    % read IMU measurements
    local_time = imu_data.time_us(imuIndex)*1e-6;
    dt_imu = local_time - local_time_prev;
    local_time_prev = local_time;
    imuIndex = imuIndex+1;
    
    if (dt_imu > 0)
        
        % predict states
        [states, delAngCorrected, delVelCorrected] = PredictStates(states,imu_data.ang_rate(imuIndex,:)',imu_data.accel(imuIndex,:)',dt_imu, gravity);
        
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
        g_load = (sqrt(dot(imu_data.accel(imuIndex,:),imu_data.accel(imuIndex,:))) / gravity);
        output.g_load(index) = g_load;
        
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
        % below 4
        if (g_load > param.fusion.g_max)
            lastHighAccelTime = local_time;
        end
        if ((local_time - lastHighAccelTime) > param.fusion.g_max_time)
            [states,covariance,velInnov,velInnovVar,innovTestPass] = FuseVelocity(states,covariance,[0 0 0],param.fusion.gate,param.fusion.noise);
            if (innovTestPass == 1)
                lastVelFuseTime = local_time;
            end
            
            % data logging
            output.innovations.vel_time_lapsed(innovIndex) = local_time;
            output.innovations.vel_innov(innovIndex,:) = velInnov';
            output.innovations.vel_innov_var(innovIndex,:) = velInnovVar';
            innovIndex = innovIndex+1;
        end
        
        % reset states if too long without aiding
        if ((local_time - lastVelFuseTime > param.fusion.reset_time) ...
                && (g_load <= 1 + param.alignment.reset_g_max) ...
                && (g_load >= 1 - param.alignment.reset_g_max) ...
                && ((local_time - lastHighAccelTime) > param.fusion.g_max_time))
            quat = AlignTilt([1;0;0;0],imu_data.accel(imuIndex,:));
            states(1:4) = quat;
            states(5) = 0;
            states(6) = 0;
            states(7) = 0;
            covariance = InitCovariance(param,dt_imu_avg);
        end
        
    end
    
end

end