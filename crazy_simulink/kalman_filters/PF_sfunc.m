function [sys,x0,str,ts] = KF_sfunc(t,x,u,flag,param)

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
Cd = param.Cd; % Measurement map
uk = u(1:4);   % Control signal
zk = u(5:end); % Measurements

Np = 50;                   % Number of particles
Nx = param.nDiscreteStates; % Number of states

% Generates a persistent variable of normally distributed states at t=0
persistent particles
persistent weights
if t == 0 || isempty(particles) || isempty(weights)
    variance = 0.2;                                   % Variance of initial states
    particles = normrnd(zeros(Nx,Np),sqrt(variance)); % Initial states
    weights = (1 / Np) * ones(1,Np);                  % Initial weights
end

% Puts every paticle through the non-linearity and computes weight
for ii = 1:Np
    xf = discrete_nonlinear_dynamics(uk, particles(:,ii), param.g, param.m,...
                                     param.k, param.A, param.I,param.l,...
                                     param.b, param.h);

    wf = weights(ii) * sum(zk - Cd*xf); % Observer to find the additive noise

    % Updates variables
    particles(:,ii) = xf;
    weights(ii) = wf;
end

weights = weights/sum(weights); % Normalize weight vector
Neff = 1/sum(weights.^2);       % Effective sample size

% Resample using the systematic resampling method
if Neff < 0.5*Np % Condition to combat degeneracy
    disp('Degenerate')
    edges = min([0 cumsum(wk)'],1);
    edges(end) = 1;
    u1 = rand/Ns;
    [~, index] = histc(u1:1/Ns:1, edges);
    particles = particles(:,index);
    weights = weights(:,index)/sum(weights(:,index)); % assigns and normalises weights
end

% State estimation
xmean = zeros(Nx,1);
for ii = 1:Np;
   xmean =+ weights(ii)*particles(:,ii);
end

particles = xmean(:,ones(1,Np)); % Store mean particle state
sys = xmean;

function sys = mdlOutputs(t,x,u,param)
% Returns the current estimation
sys = x;
