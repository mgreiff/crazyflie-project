%% Sets up the necessary parameters for all filters, any of which can be run
% by swithcing models in the filter block in the simulink model. The
% available choices are located in /kalman_filters/* and are
%
% * discrete_linear_kalman    (fucntional)
% * discrete_extended_kalman  (dysfunctional)
% * discrete_unscented_kalman (dysfucntional)

%% Number of states
nStates = 12;
nControlsignals = 4;
nMeasuredStates = 9;

%% Continuous time model
Ixx = I(1); Iyy=I(2); Izz = I(3);

Ac = zeros(12);
Ac(1:3,4:6) = eye(3);
Ac(4:6,4:6) = -diag(A)/m;
Ac(7:9,10:12) = eye(3);

Bc = zeros(12, 4);
Bc(6, :) = k;
Bc(10:12,:) = [0,-k*l/Ixx,0,k*l/Ixx;
              -k*l/Iyy,0,k*l/Iyy,0;
              -b/Izz,b/Izz,-b/Izz,b/Izz];
Bc = sqrt(m*g/k).*Bc;

Cc = eye(nStates);
Cc(10:12,:) = []; % remove angular speeds

Dc = zeros(length(Cc(:,1)), 4);

%% Discretetise model
KFparam.h = 0.1;
contsys = ss(Ac,Bc,Cc,Dc);
discsys = c2d(contsys, KFparam.h);

% Linearized model for use in the liinearized kalman filter
[KF_A, KF_B, KF_C, KF_D] = ssdata(discsys);
KFparam.Ad = KF_A;
KFparam.Bd = KF_B;
KFparam.Cd = KF_C;
KFparam.Dd = KF_D;

% Parameters for use in all Kalman filters
KFparam.P0 = eye(nStates); % Initial covariance matrix, tbd
KFparam.Q = eye(nStates);
KFparam.Q(12,12) = 0.1;

KF_R = diag([0.1,0.1,0.1,0.1,0.1,0.1,0.01,0.01,0.01]);
KFparam.R = KF_R;

if length(KFparam.R) ~= length(KFparam.Cd(:,1))
    disp('The number of measured states in C does not match R')
end

% State description f?r the S-function
KFparam.nInputs = nControlsignals + nMeasuredStates;
KFparam.nOutputs = nStates;
KFparam.nDiscreteStates = nStates;

% Initial x
KFparam.x0 = zeros(KFparam.nDiscreteStates,1);

% Additional parameters for the EKF
KFparam.g = g;
KFparam.m = m;
KFparam.k = k;
KFparam.A = A;
KFparam.I = I;
KFparam.l = l;
KFparam.b = b;