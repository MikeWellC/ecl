function covariance = InitCovariance(param,dt)

% Define quaternion state errors
Sigma_quat = param.alignment.quatErr * [1;1;1;1];

% Define velocity state errors
Sigma_velocity = [param.alignment.velErrNE;param.alignment.velErrNE;param.alignment.velErrD];

% Define delta angle bias state errors
Sigma_dAngBias = param.alignment.delAngBiasErr*dt*[1;1;1];

% Define delta velocity bias state errors
Sigma_dVelBias = param.alignment.delVelBiasErr*dt*[1;1;1];

% Convert to variances and write to covariance matrix diagonals
covariance = diag([Sigma_quat;Sigma_velocity;Sigma_dAngBias;Sigma_dVelBias;].^2);
end

