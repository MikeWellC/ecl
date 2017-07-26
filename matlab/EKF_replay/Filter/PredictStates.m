function [states, correctedDelAng, correctedDelVel]  = PredictStates( ...
    states, ... % previous state vector (4x1 quaternion, 3x1 velocity, 3x1 position, 3x1 delAng bias, 3x1 delVel bias)
    delAng, ... % IMU delta angle measurements, 3x1 (rad)
    delVel, ... % IMU delta velocity measurements 3x1 (m/s)
    dt, ... % accumulation time of the IMU measurement (sec)
    gravity) % acceleration due to gravity (m/s/s)

% define persistent variables for previous delta angle and velocity which
% are required for sculling and coning error corrections
persistent prevDelAng;
if isempty(prevDelAng)
    prevDelAng = delAng;
end

persistent prevDelVel;
if isempty(prevDelVel)
    prevDelVel = delVel;
end

% Remove sensor bias errors
delAng = delAng - states(8:10);
delVel = delVel - states(11:13);

% Correct delta velocity for rotation and skulling
% Derived from Eqn 25 of:
% "Computational Elements For Strapdown Systems"
% Savage, P.G.
% Strapdown Associates
% 2015, WBN-14010
correctedDelVel= delVel + ...
0.5*cross(prevDelAng + delAng , prevDelVel + delVel) + 1/6*cross(prevDelAng + delAng , cross(prevDelAng + delAng , prevDelVel + delVel)) +  1/12*(cross(prevDelAng , delVel) + cross(prevDelVel , delAng));

% Apply corrections for coning errors
% Coning correction from :
% "A new strapdown attitude algorithm", 
% R. B. MILLER, 
% Journal of Guidance, Control, and Dynamics
% July, Vol. 6, No. 4, pp. 287-291, Eqn 11 
correctedDelAng   = delAng - 1/12*cross(prevDelAng , delAng);

% Save current measurements
prevDelAng = delAng;
prevDelVel = delVel;

% Convert the rotation vector to its equivalent quaternion
deltaQuat = RotToQuat(correctedDelAng);

% Update the quaternions by rotating from the previous attitude through
% the delta angle rotation quaternion
states(1:4) = QuatMult(states(1:4),deltaQuat);

% Normalise the quaternions
states(1:4) = NormQuat(states(1:4));

% Calculate the body to nav cosine matrix
Tbn = Quat2Tbn(states(1:4));

% transform body delta velocities to delta velocities in the nav frame
delVelNav = Tbn * correctedDelVel + [0;0;gravity]*dt;

% Sum delta velocities to get the velocity
states(5:7) = states(5:7) + delVelNav(1:3);

end