%% define symbolic variables and constants
clear all;
reset(symengine);
syms dax day daz real % IMU delta angle measurements in body axes - rad
syms dvx dvy dvz real % IMU delta velocity measurements in body axes - m/sec
syms q0 q1 q2 q3 real % quaternions defining attitude of body axes relative to local NED
syms vn ve vd real % NED velocity - m/sec
syms dax_b day_b daz_b real % delta angle bias - rad
syms dvx_b dvy_b dvz_b real % delta velocity bias - m/sec
syms dt real % IMU time step - sec
syms gravity real % gravity  - m/sec^2
syms daxVar dayVar dazVar dvxVar dvyVar dvzVar real; % IMU delta angle and delta velocity measurement variances

%% define the state prediction equations

% define the measured Delta angle and delta velocity vectors
dAngMeas = [dax; day; daz];
dVelMeas = [dvx; dvy; dvz];

% define the IMU bias errors and scale factor
dAngBias = [dax_b; day_b; daz_b];
dVelBias = [dvx_b; dvy_b; dvz_b];

% define the quaternion rotation vector for the state estimate
quat = [q0;q1;q2;q3];
% derive the truth body to nav direction cosine matrix
Tbn = Quat2Tbn(quat);

% define the truth delta angle
% ignore coning compensation as these effects are negligible in terms of 
% covariance growth for our application and grade of sensor
dAngTruth = dAngMeas - dAngBias;

% Define the truth delta velocity -ignore sculling and transport rate
% corrections as these negligible are in terms of covariance growth for our
% application and grade of sensor
dVelTruth = dVelMeas - dVelBias;

% define the attitude update equations
% use a first order expansion of rotation to calculate the quaternion increment
% acceptable for propagation of covariances
deltaQuat = [1;
    0.5*dAngTruth(1);
    0.5*dAngTruth(2);
    0.5*dAngTruth(3);
    ];
quatNew = QuatMult(quat,deltaQuat);

% define the velocity update equations
% ignore coriolis terms for linearisation purposes
vNew = [vn;ve;vd] + [0;0;gravity]*dt + Tbn*dVelTruth;

% define the IMU error update equations
dAngBiasNew = dAngBias;
dVelBiasNew = dVelBias;

% Define the state vector & number of states
stateVector = [quat;vn;ve;vd;dAngBias;dVelBias];
nStates=numel(stateVector);

% Define vector of process equations
stateVectorNew = [quatNew;vNew;dAngBiasNew;dVelBiasNew];

%% derive the state transition and state error matrix

% Define the control (disturbance) vector. Error growth in the inertial
% solution is assumed to be driven by 'noise' in the delta angles and
% velocities, after bias effects have been removed. This is OK becasue we
% have sensor bias accounted for in the state equations.
distVector = [daxVar;dayVar;dazVar;dvxVar;dvyVar;dvzVar];

% derive the control(disturbance) influence matrix
G = jacobian(stateVectorNew, [dAngMeas;dVelMeas]);

% derive the state error matrix
distMatrix = diag(distVector);
Q = G*distMatrix*transpose(G);
f = matlabFunction(Q,'file','calcQ13.m');

% derive the state transition matrix
F = jacobian(stateVectorNew, stateVector);
f = matlabFunction(F,'file','calcF13.m');

%% calculate Quaternion to Euler angle error transfer matrix
% quat = [q0;q1;q2;q3];
% syms roll pitch yaw 'real';
roll = atan2(2*(quat(3)*quat(4)+quat(1)*quat(2)) , (quat(1)*quat(1) - quat(2)*quat(2) - quat(3)*quat(3) + quat(4)*quat(4)));
pitch = -asin(2*(quat(2)*quat(4)-quat(1)*quat(3)));
yaw = atan2(2*(quat(2)*quat(3)+quat(1)*quat(4)) , (quat(1)*quat(1) + quat(2)*quat(2) - quat(3)*quat(3) - quat(4)*quat(4)));
euler = [roll;pitch;yaw];
error_transfer_matrix = jacobian(euler,quat);
matlabFunction(error_transfer_matrix,'file','quat_to_euler_error_transfer_matrix.m');
