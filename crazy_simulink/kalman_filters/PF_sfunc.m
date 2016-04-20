function [sys,x0,str,ts] = PF_sfunc(t,x,u,flag,param)

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
global P;
P = param.P0;

str = [];                % Set str to an empty matrix.
ts  = [param.h 0];       % sample time: [period, offset]
		      
%==============================================================
% Update the discrete states
%==============================================================
function sys = mdlUpdates(t,x,u,param)
Ad = param.Ad;
Bd = param.Bd;
Cd = param.Cd;

Nc = param.nControlsignals;
Nx = param.nDiscreteStates;

uk = u(1:Nc);
zk = u(Nc+1:end);

Np = 5000;                   % Number of particles

% Generates a persistent variable of normally distributed states at t=0
persistent particles
persistent weights
if t == 0 || isempty(particles) || isempty(weights)
    variance = 0.2;                                   % Variance of initial states
    particles = normrnd(zeros(Nx,Np),sqrt(variance)); % Initial states
    weights = log((1 / Np)) * ones(1,Np);             % Use logarithm for numerical stability
end

% Puts every paticle through the non-linearity and computes weight
for ii = 1:Np
    %xf = discrete_nonlinear_dynamics(uk, particles(:,ii), param.g, param.m,...
    %                                 param.k, param.A, param.I,param.l,...
    %                                 param.b, param.h);

    % OBS! The discrete_nonlinear_dynamics must be uncommented in order
    % to update using the non-linear model (if defined)
    xf = Ad*particles(:,ii) + Bd*uk;
    wf = weights(ii) * (zk - Cd*xf)^2;

    % Updates variables
    particles(:,ii) = xf;
    weights(ii) = wf;
end

weights = log(exp(weights)/sum(exp(weights))); % Normalize weight vector - exponentiera

Neff = 1/sum(exp(weights).^2);       % Effective sample size

% Resample using the systematic resampling method by looking at the
% cumulative distribution function and removing points in bins that are
% less represented and expanding bins that have greater weights.
if Neff < 0.5*Np % Condition to combat degeneracy
    disp(['Degeneracy at time t = ',num2str(t)])
    edges = min([0 cumsum(weights)],1);
    edges(end) = 1;
    u1 = rand/Np;
    [~, index] = histc(u1:1/Np:1, edges);
    particles = particles(:,index);
end

if 0 % Plots distributions
    figure(1);
    subplot(3,1,1);
    hist(particles(1,:), 100)
    subplot(3,1,2);
    hist(particles(2,:), 100)
    subplot(3,1,3);
    hold on;
    plot(cumsum(exp(weights)))
end

% State estimation
xmean = zeros(Nx,1);
for ii = 1:Np;
   xmean =+ exp(weights(ii))*particles(:,ii);
end
particles = xmean(:,ones(1,Np));

sys = xmean;

function sys = mdlOutputs(t,x,u,param)
% Returns the current estimation
sys = x;
