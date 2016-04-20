function [sys,x0,str,ts] = UKF_sfunc(t,x,u,flag,param)

switch flag,
    case 0 % Initialization
        [sys,x0,str,ts] = mdlInitializeSizes(param);
	case 2 % Update of discrete states
        sys = mdlUpdates(t,x,u,param);
	case 3 % Calculation of outputs
        sys = mdlOutputs(t,x,u,param);
    case {1, 4, 9}
        % 1 - Calculation of derivatives (not needed).
        % 4 - Calculation of next sample hit (variable sample time block only).
        % 9 - End of simulation tasks (not needed).
        sys = [];
    otherwise
        % No other flags are defined in simulink, throws error
        error(['Unhandled flag = ',num2str(flag)]);
end

function [sys,x0,str,ts] = mdlInitializeSizes(param)

% Initialize simsizes
sizes = simsizes;

% No continuous states
sizes.NumContStates  = 0;

% AAll discrite states, both reshaped Rref and x 10 + 10*m in total
sizes.NumDiscStates  = param.nDiscreteStates;

% Number of outputs (3)
sizes.NumOutputs     = param.nOutputs;

% Number of inputs (1)
sizes.NumInputs      = param.nInputs;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes); 

x0 = param.x0;
str = [];                % Set str to an empty matrix.
ts  = [param.h 0];       % sample time: [period, offset]
		      
%==============================================================
% Update the discrete states
%==============================================================
function sys = mdlUpdates(t,x,u,param)
% System update with the UKF following nomenclature and
% algorithm description in E. Wan, chapter 7

Nc = param.nControlsignals;

uk = u(1:Nc); % Control signal
zk = u(Nc+1:end); % Measurements

Ad = param.Ad;
Bd = param.Bd;
Cd = param.Cd;

Q = param.Q;
R = param.R;

persistent P
if isempty(P) || t == 0 % Assuming the simulaiton starts at t=0
    P = param.P0;
end

% ~~~ UKF Parameters ~~~
L = size(x,1);                     % Number of discrete states
beta = 2;                           % Optimal for gaussian distributions
alpha = 5e-1; %1e-3 in DI           % Tuning parameter (0 < alpha < 1e-4)
keta = 0;                           % Tuning parameter (set to 0 or 3-L)
lambda =  alpha^2 * (L + keta) - L; % Scaling factor (see E. Wan for details)

% ~~~ UKF Weights ~~~
% Weight for mean
Wm = [lambda/(L + lambda);...                      % i = 0
      1/(2 * (L + lambda))*ones(2*L,1)];           % i \neq 0
      
% Weight for covariances
Wc = [lambda/(L + lambda)+(1 - alpha^2 + beta);... % i = 0
      1/(2 * (L + lambda))*ones(2*L,1)];           % i \neq 0

% ~~~ Computes covariances and sigma points (predictive step) ~~~
%Psqrt = sqrtm((L+lambda)*Pa);
Psqrt = sqrt(L + lambda) * chol(P)';

X = [x, x(:,ones(1,length(x)))+Psqrt, x(:,ones(1,length(x)))-Psqrt];

Xf = zeros(size(X));
for ii = 1:length(X(1,:))
    %Xf(:,ii) = discrete_nonlinear_dynamics(uk, X(:,ii), param.g, param.m, param.k,...
    %                                       param.A, param.I,param.l, param.b,...
    %                                       param.h);
    Xf(:,ii) = Ad*X(:,ii) + Bd*uk;
end

% ~~~ UT-transform of the dynamics (time update) ~~~
xMean = zeros(L,1);
for ii = 1:2*L+1
    xMean = xMean + Wm(ii)*Xf(:,ii);                 % Transformed mean (x)
end
xDev = Xf - xMean(:,ones(1,length(Xf(1,:)))); % Transformed deviations (x)

% ~~~ UT-transform of the measurements (measurement update) ~~~
Yf = zeros(size(Cd,1), 2*L+1);
for ii = 1:2*L+1
    Yf(:,ii) = Cd*Xf(:,ii);
end

yMean = zeros(L,1);
for ii = 1:2*L+1
    yMean =+ Wm(ii)*Yf(:,ii);                 % Transformed mean (x)
end
yDev = Yf - yMean(:,ones(1,length(Yf(1,:)))); % Transformed deviations (y)

% ~~~ Compute transformed covariance matrices ~~~
Pxx = xDev*diag(Wc)*xDev' + Q;  % transformed state covariance
Pxy = xDev*diag(Wc)*yDev';    % transformed cross covariance
Pyy = yDev*diag(Wc)*yDev' + R;  % transformed measurement covariance

% Correction update
K = Pxy / Pyy;
xhat = xMean + K * (zk - yMean);    % State estimation
P = Pxx - K * Pyy * K';            % Computes new covariance

sys = xhat;

%==============================================================
% Calculate outputs
%==============================================================
function sys = mdlOutputs(t,x,u,param)

sys = x;
