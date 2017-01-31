Ts = 0.1;
sys_disc = c2d(ss([0,1;0,0],[0;1],[1,0],[]), Ts);
KFparam.Ad = sys_disc.A;
KFparam.Bd = sys_disc.B;
KFparam.Cd = sys_disc.C;

nStates = size(KFparam.Ad,1);
nMeasuredStates = 1;
nControlsignals = 1;

% Parameters for use in all Kalman filters
KFparam.P0 = eye(nStates);  % Initial covariance matrix, tbd
KFparam.std_Q = 0;   % Standard deviation on process noise (no noise added)
KFparam.std_R = 0.1; % Standard deviation of measurement noise
KFparam.Q = (KFparam.std_Q^2)*eye(nStates);
KFparam.R = (KFparam.std_R^2)*eye(nMeasuredStates);

% State description for the S-function
KFparam.nInputs = nControlsignals + nMeasuredStates;
KFparam.nMeasuredStates = nMeasuredStates;
KFparam.nControlsignals = nControlsignals;
KFparam.nOutputs = nStates;
KFparam.nDiscreteStates = nStates;
KFparam.h = Ts;

% Initial x
KFparam.x0 = zeros(nStates,1);