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

uk = u(1:4);  % Control signal
zk = u(5:end);   % Measurements
Q = param.Q;
R = param.R;
Cd = param.Cd;

persistent P
if isempty(P)
    %P = blkdiag(param.P0, param.Q, param.R);
    P = param.P0;
elseif t == 0
	%P = blkdiag(param.P0, param.Q, param.R);
    P = param.P0;
end

% ~~~ UKF Parameters ~~~
L = param.nDiscreteStates;          % Number of discrete states
beta = 2;                           % Optimal for gaussian distributions
alpha = 1e-4;                       % Tuning parameter (0 < alpha < 1e-4)
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
Ps = sqrtm((L+lambda)*P);
%Ps = sqrt(L + lambda) * chol(P)';

X = [x, x(:,ones(1,length(x)))+Ps, x(:,ones(1,length(x)))-Ps];
Xf = X;

for ii = 1:length(X(1,:))
    Xf(:,ii) = discrete_nonlinear_dynamics(uk, X(:,ii), param.g, param.m, param.k,...
                                           param.A, param.I,param.l, param.b,...
                                           param.h);
end

% ~~~ UT-transform of the dynamics (time update) ~~~
xMean = zeros(L,1);
for ii = 1:2*L+1
    xMean =+ Wm(ii)*Xf(:,ii);               % Transformed mean (x)
end
xDev = Xf - xMean(:,ones(1,length(Xf(1,:)))); % Transformed deviations (x)

% ~~~ UT-transform of the measurements (measurement update) ~~~
Yf = zeros(length(Cd(:,1)), 2*L+1);
for ii = 1:2*L+1
    Yf(:,ii) = Cd*Xf(:,ii);
end
yMean = Cd * xMean; % Transformed mean (y) using the fact that C is linear
yDev = Yf - yMean(:,ones(1,length(Yf(1,:)))); % Transformed deviations (y)

% ~~~ Compute transformed covariance matrices ~~~
Pxx = xDev*diag(Wc)*xDev'+Q;  % transformed state covariance
Pxy = xDev*diag(Wc)*yDev';    % transformed cross covariance
Pyy = yDev*diag(Wc)*yDev'+R;  % transformed measurement covariance

% Correction update
K = Pxy / Pyy;
xhat = xMean + K * (zk - yMean);
P = Pxx - K * Pyy * K';
sys = xhat;

%==============================================================
% Calculate outputs
%==============================================================
function sys = mdlOutputs(t,x,u,param)

sys = x;
