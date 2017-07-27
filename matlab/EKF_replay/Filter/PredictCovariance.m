function P  = PredictCovariance(deltaAngle, ...
    deltaVelocity, ...
    states,...
    P, ...  % Previous state covariance matrix
    dt, ... % IMU and prediction time step
    param) % tuning parameters
    
% Set filter state process noise other than IMU errors, which are already
% built into the derived covariance predition equations.
% This process noise determines the rate of estimation of IMU bias errors
dAngBiasSigma = dt*dt*param.prediction.dAngBiasPnoise;
dVelBiasSigma = dt*dt*param.prediction.dVelBiasPnoise;
processNoiseVariance = [zeros(1,7), dAngBiasSigma*[1 1 1], dVelBiasSigma*[1 1 1]].^2;

% Specify the noise variance on the IMU delta angles and delta velocities
daxVar = (dt*param.prediction.angRateNoise)^2;
dayVar = daxVar;
dazVar = daxVar;
dvxVar = (dt*param.prediction.accelNoise)^2;
dvyVar = dvxVar;
dvzVar = dvxVar;

dvx = deltaVelocity(1);
dvy = deltaVelocity(2);
dvz = deltaVelocity(3);
dax = deltaAngle(1);
day = deltaAngle(2);
daz = deltaAngle(3);

q0 = states(1);
q1 = states(2);
q2 = states(3);
q3 = states(4);

dax_b = states(8);
day_b = states(9);
daz_b = states(10);

dvx_b = states(11);
dvy_b = states(12);
dvz_b = states(13);

vn = states(5);
ve = states(6);
vd = states(7);

% Predicted covariance
F = calcF13(dax,dax_b,day,day_b,daz,daz_b,dvx,dvx_b,dvy,dvy_b,dvz,dvz_b,q0,q1,q2,q3);
Q = calcQ13(daxVar,dayVar,dazVar,dvxVar,dvyVar,dvzVar,q0,q1,q2,q3);
P = F*P*transpose(F) + Q;

% Add the general process noise variance
for i = 1:13
    P(i,i) = P(i,i) + processNoiseVariance(i);
end

% Force symmetry on the covariance matrix to prevent ill-conditioning
% of the matrix which would cause the filter to blow-up
P = 0.5*(P + transpose(P));

% ensure diagonals are positive
for i=1:13
    if P(i,i) < 0
        P(i,i) = 0;
    end
end

end