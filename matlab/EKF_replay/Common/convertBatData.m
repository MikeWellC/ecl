clear imu_data;

n_samples = length(VarName1);
time_sec = (VarName1 - VarName1(1))*0.004;

gravity = 9.80665;
deg2rad = pi/180;

for index=1:n_samples
    % convert time stamp data
    imu_data.time_us(index,1) = time_sec(index) * 1e6;
    
    % convert angular rate data
    imu_data.ang_rate(index,:) = [VarName5(index), VarName6(index), VarName7(index)] * deg2rad;
    
    % calculate delta velocity data
    % check for saturation for each accel axis and take data from high range
    % sensor if above 15g
    if (abs(VarName2(index)) > 15)
        accel_x = VarName8(index) * gravity;
    else
        accel_x = VarName2(index) * gravity;
    end
    if (abs(VarName3(index)) > 15)
        accel_y = VarName9(index) * gravity;
    else
        accel_y = VarName3(index) * gravity;
    end
    if (abs(VarName4(index)) > 15)
        accel_z = VarName10(index) * gravity;
    else
        accel_z = VarName4(index) * gravity;
    end
    imu_data.accel(index,:) = [accel_x, accel_y, accel_z];
    
end
