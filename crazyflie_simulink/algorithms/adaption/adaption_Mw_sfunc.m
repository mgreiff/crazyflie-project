function [sys,x0,str,ts] = adaption_Mw_sfunc(t,x,u,flag)

switch flag,
    case 0 % Initialization
        [sys,x0,str,ts] = mdlInitializeSizes();
	case 2 % Update of discrete states
        sys = mdlUpdates(t,x,u);
	case 3 % Calculation of outputs
        sys = mdlOutputs(x);
    case {1, 4, 9}
        % 1 - Calculation of derivatives (not needed).
        % 4 - Calculation of next sample hit (variable sample time block only).
        % 9 - End of simulation tasks (not needed).
        sys = [];
    otherwise
        % No other flags are defined in simulink, throws error
        error(['Unhandled flag = ',num2str(flag)]);
end

function [sys,x0,str,ts] = mdlInitializeSizes()
h = 0.002;

% Initialize simsizes
sizes = simsizes;

% No continuous states
sizes.NumContStates  = 0;

% All discrite states, both reshaped Rref and x, 10 + (10+1)*m in total
sizes.NumDiscStates  = 2;

% Number of outputs (3)
sizes.NumOutputs     = 2;

% Number of inputs (1)
sizes.NumInputs      = 8;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes); 

% Sets the initial condition in the reference trajectory to 0
x0 = [1.1*10^-8;1*10^-8];% D_k
 
% Initialize the discrete states.
str = [];                        % Set str to an empty matrix.
ts  = [h 0];       % sample time: [period, offset]
		      
function sys = mdlUpdates(t,x,u)
if 0
    %% Use a LS linear fitting algorithm
    M = 500;
    h = 0.002;

    time = (t-h*(M-1)):h:t;
    persistent karray
    if isempty(karray)
        karray = ones(1,M).*x(1);
    end
    Tref = u(1:4);
    what = u(5:8);
    wmin = 500;
    wmax = 1800;
    what(what > wmax) = wmax;
    what(what < wmin) = wmin;
    
    kbar = Tref(1)/(sum(what.^2));

    % Update karray
    karray(1:end-1) = karray(2:end);
    karray(end) = kbar;

    % Set up and solve LS problem
    A = [M, sum(time);
         sum(time), sum(time.^2)];
    b = [sum(karray);
         sum(karray.*time)];
    alpha = A\b;

    x(1) = alpha(1) + alpha(2)*(t + h);
    x(2) = 10^-8;
elseif 1
    %% Use a simple LP filter
    persistent kprev
    if isempty(kprev)
        kprev = x(1);
    end
    Tref = u(1:4);
    what = u(5:8);
    wmin = 500;
    wmax = 1800;
    what(what > wmax) = wmax;
    what(what < wmin) = wmin;
    kin = Tref(1)/(sum(what.^2));
    khat = 0.99*kprev + 0.01*kin;
    kprev = khat;
    
    x(1) = khat;
    x(2) = 10^-8;
else
    %% Use an RLS scheme
    persistent P
    persistent y

    if isempty(P) || t == 0
        P = 1;
        y = 10^-8;
    end
    lambda = 0.99;

    Tref = u(1:4);
    what = u(5:8);
    kin = Tref(1)/(sum(what.^2));
    limits = [1.2*10^-8,8*10^-9];
    if kin > limits(1)
        kin = limits(1);
    elseif kin < limits(2)
        kin = limits(2);
    end
    
    estPar = x(1);
    phi = -y;

    epsilon = kin - phi'*estPar;
    P = ((1/lambda).*(P - P*(phi*phi')*P))/(lambda + phi'*P*phi);
    K = P*phi;
    estPar = estPar + K*epsilon;

    y = kin;
    x(1) = estPar;
end
sys =  x;

%=========================================================================
% Calculate outputs - called on every simulation step, so the
% optimization whould be done in the update discrete states
%=========================================================================
function sys = mdlOutputs(x)
sys = x;
