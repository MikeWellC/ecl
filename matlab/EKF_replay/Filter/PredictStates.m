function [states, correctedDelAng, correctedDelVel]  = PredictStates( ...
    states, ... % previous state vector (4x1 quaternion, 3x1 velocity, 3x1 position, 3x1 delAng bias, 3x1 delVel bias)
    newAngRate, ... % IMU delta angle measurements, 3x1 (rad)
    newAccel, ... % IMU delta velocity measurements 3x1 (m/s)
    dt, ... % time delta of the IMU measurement (sec)
    gravity) % acceleration due to gravity (m/s/s)

sample_ratio = 10;

% define persistent variables for previous imu data which
% are required for delta angle and velocity conversions and upsampling
persistent prevAngRate;
if isempty(prevAngRate)
    prevAngRate = newAngRate;
end

persistent prevAccel;
if isempty(prevAccel)
    prevAccel = newAccel;
end

% calculate delta angles and velocities
correctedDelAng = (0.5 * dt * (newAngRate + prevAngRate) - states(8:10));
correctedDelVel = (0.5 * dt * (newAccel + prevAccel) - states(11:13));

angRateBias = states(8:10) / dt;
accelBias = states(11:13) / dt;

oldAngRateHF = prevAngRate - angRateBias;
oldAccelHF = prevAccel - accelBias;
% divide the IMU data up into increments to improve integration accuracy
for index = 1:sample_ratio
    % calculate high frequency delta angle using linear interpolation and
    % trapezoidal integration
    newAngRateHF = (index/sample_ratio) * (newAngRate - prevAngRate) + prevAngRate - angRateBias;
    delAngHF = 0.5 * (dt / sample_ratio) * (newAngRateHF + oldAngRateHF);
    oldAngRateHF = newAngRateHF;
    
    % calculate high frequency delta velocity using linear interpolation and
    % trapezoidal integration
    newAccelHF = (index/sample_ratio) * (newAccel - prevAccel) + prevAccel - accelBias;
    delVelHF = 0.5 * (dt / sample_ratio) * (newAccelHF + oldAccelHF);
    oldAccelHF = newAccelHF;
    
    % Convert the rotation vector to its equivalent quaternion
    deltaQuat = RotToQuat(delAngHF);
    
    % Update the quaternions by rotating from the previous attitude through
    % the delta angle rotation quaternion
    states(1:4) = QuatMult(states(1:4),deltaQuat);
    
    % Normalise the quaternions
    states(1:4) = NormQuat(states(1:4));
    
    % Calculate the body to nav cosine matrix
    Tbn = Quat2Tbn(states(1:4));
    
    % transform body delta velocities to delta velocities in the nav frame
    delVelNav = Tbn * delVelHF + [0;0;gravity]*dt/sample_ratio;
    
    % Sum delta velocities to get the velocity
    states(5:7) = states(5:7) + delVelNav(1:3);
end

end