%% Discretetise model
h = 0.01;
KFparam.h = h;  % The kinect sample rate

UseAcceleration = 1; % Option, set to one if both acceleration and positional
                     % measurements should be used (default) and 0 if only
                     % positional measurements should be included.

KFparam.Ad = eye(3) + diag([h,h],1) + diag(h^2/2,2);
KFparam.Bd = zeros(3,1); % Dummy, can't be set [] in the current KF implementation
KFparam.Cd = eye(3);
KFparam.Cd(2,:) = []; % Remove velocities (always)
KFparam.Dd = [];

nStates = size(KFparam.Ad,1);
nControlsignals = 1; % Dummy, can't be set to 0 in th current KF implementation
if UseAcceleration
    nMeasuredStates = 2; % 1 if only using positions from openni, 2 if including acceleration
    std_R = [0.1, 0.5]; % std noise using both acceleration and position
else
    nMeasuredStates = 1;
    KFparam.Cd(2,:) = []; % Remove acceleration (if only measuring position)
    std_R = [0.1]; % std noise using only position
end

% Parameters for use in all Kalman filters
KFparam.P0 = eye(nStates);
KFparam.Q = 0.1*eye(nStates);
KFparam.R = diag(std_R);

if length(KFparam.R) ~= length(KFparam.Cd(:,1))
    disp('The number of measured states in C does not match R')
end

% State description f?r the S-function
KFparam.nInputs = nControlsignals + nMeasuredStates;
KFparam.nMeasuredStates = nMeasuredStates;
KFparam.nOutputs = nStates;
KFparam.nDiscreteStates = nStates;
KFparam.nControlsignals = nControlsignals;

% Initial x
KFparam.x0 = zeros(KFparam.nDiscreteStates,1);
KFparam.x0(3) = 1;

% MA filter order
filterOrder = 10;

% Delays in bluetooth transmission
KFparam.sendDelay = 0.3;
KFparam.recDelay = 0.3;

% Reference trajectory
freq = 0.4;
trajectory.pos = @(t) (1-cos(freq*pi*t))/(freq*pi)^2;
trajectory.acc = @(t) cos(freq*pi*t);