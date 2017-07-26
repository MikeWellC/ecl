clear imu_data;

n_samples = length(VarName1);
time_sec = (VarName1 - VarName1(1))*0.004;

index = 0;
for i=2:n_samples
    index = index + 1;
    gravity = 9.80665;
    deg2rad = pi/180;
    
    % convert time stamp data
    imu_data.time_us(index,1) = time_sec(i) * 1e6;
    dt = (time_sec(i) - time_sec(i-1));
    imu_data.gyro_dt(index,1) = dt;
    imu_data.accel_dt(index,1) = dt;
    
    % calculate delta angle data
    delta_angle_x = 0.5*(VarName5(i) + VarName5(i-1)) * dt * deg2rad;
    delta_angle_y = 0.5*(VarName6(i) + VarName6(i-1)) * dt * deg2rad;
    delta_angle_z = 0.5*(VarName7(i) + VarName7(i-1)) * dt * deg2rad;
    imu_data.del_ang(index,:) = [delta_angle_x, delta_angle_y, delta_angle_z];
    
    % calculate delta velocity data
    % check for saturation for each accel axis and take data from high range
    % sensor if above 15g
    if ((abs(VarName2(i)) > 15) || (abs(VarName2(i-1)) > 15))
        delta_vel_x = 0.5 * (VarName8(i) + VarName8(i)) * dt * gravity;
    else
        delta_vel_x = 0.5 * (VarName2(i) + VarName2(i)) * dt * gravity;
    end
    if ((abs(VarName3(i)) > 15) || (abs(VarName3(i-1)) > 15))
        delta_vel_y = 0.5 * (VarName9(i) + VarName9(i)) * dt * gravity;
    else
        delta_vel_y = 0.5 * (VarName3(i) + VarName3(i)) * dt * gravity;
    end
    if ((abs(VarName4(i)) > 15) || (abs(VarName4(i-1)) > 15))
        delta_vel_z = 0.5 * (VarName10(i) + VarName10(i)) * dt * gravity;
    else
        delta_vel_z = 0.5 * (VarName4(i) + VarName4(i)) * dt * gravity;
    end
    imu_data.del_vel(index,:) = [delta_vel_x, delta_vel_y, delta_vel_z];
    
end
