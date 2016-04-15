function [sys,x0,str,ts] = trajectory_sfunc(t,x,u,flag,splines)

switch flag,
    case 0 % Initialization
        [sys,x0,str,ts] = mdlInitializeSizes(splines);
	case 2 % Update of discrete states
        sys = mdlUpdates(t,x,u,splines);
	case 3 % Calculation of outputs
        sys = mdlOutputs(t,x,u,splines);
    case {1, 4, 9}
        % 1 - Calculation of derivatives (not needed).
        % 4 - Calculation of next sample hit (variable sample time block only).
        % 9 - End of simulation tasks (not needed).
        sys = [];
    otherwise
        % No other flags are defined in simulink, throws error
        error(['Unhandled flag = ',num2str(flag)]);
end

function [sys,x0,str,ts] = mdlInitializeSizes(splines)

% Initialize simsizes
sizes = simsizes;

% No continuous states
sizes.NumContStates  = 0;

% All discrite states, both reshaped Rref and x 10 + 10*m in total
sizes.NumDiscStates  = 1;

% Number of outputs (3)
sizes.NumOutputs     = 24*6;

% Number of inputs (1)
sizes.NumInputs      = 0;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes); 

x0 = [0];

str = [];                % Set str to an empty matrix.
ts  = [splines.h 0];       % sample time: [period, offset]
		      
%==============================================================
% Update the discrete states
%==============================================================
function sys = mdlUpdates(t,x,u,splines)

sys = x;

function sys = mdlOutputs(t,x,u,splines)
% Returns the current estimation

currentTime = t;  % Starting time
Ts = 0.5;         % Time between samples
predHorizon = (20-8)/0.5; % Number of points on the prediction horizon

times = currentTime + Ts.* (0:predHorizon-1);
referenceTrajectory = zeros(6,predHorizon);

for ii = 1:predHorizon
    % Finds the spline to evaluate
    T = times(ii);
    splineNumber = sum(splines.times <=  T);

    % Treats the case when the time is outside the defined interval [t_0,t_s]
    if splineNumber == 0
        disp('Tried to evaluate spline before t_0, setting reference to the initial spline point.')
        T = splines.times(1);
        splineNumber = 1;
    elseif T >= splines.times(end)
        disp('Tried to evaluate spline after t_s, keeping position in terinal spline point')
        T = splines.times(end);
        splineNumber = length(splines.coeff)/(splines.N + 1);
    end
    % Evaluates splines and stores data in referenceTrajectory
    for dim = 1:3
        startIndex = (splines.N + 1) * (splineNumber - 1) + 1;
        endIndex = (splines.N + 1) * splineNumber;
        poscoeff = splines.coeff(startIndex:endIndex,dim);
        poscoeff = poscoeff(end:-1:1); % Reverses for polyval
        tt = T - splines.times(splineNumber);

        % Position
        referenceTrajectory(dim,ii) = polyval(poscoeff,tt);

        % Velocity
        poscoeff(end) = [];
        velcoeff = (length(poscoeff):-1:1)'.*poscoeff; 
        referenceTrajectory(dim + 1,ii) = polyval(velcoeff,tt);
    end
end
disp(referenceTrajectory)
sys = reshape(referenceTrajectory, [numel(referenceTrajectory),1]);
