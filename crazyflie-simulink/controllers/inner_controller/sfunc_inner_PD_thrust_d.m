function [sys,x0,str,ts] = sfunc_inner_PD_thrust_d(t,x,u,flag,param)

switch flag,
    case 0
        [sys,x0,str,ts] = mdlInitializeSizes(param);
	case 2
        sys = mdlUpdates(t,x,u,param);
	case 3
        sys = mdlOutputs(x);
    case {1, 4, 9}
        sys = [];
    otherwise
        error(['Unhandled flag = ',num2str(flag)]);
end

function [sys,x0,str,ts] = mdlInitializeSizes(param)

sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 2;
sizes.NumOutputs     = 2;
sizes.NumInputs      = 24;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes); 

x0 = [0;0]; % T (control signal) and saturation limit
 
str = [];
ts  = [param.h 0];
		      
function sys = mdlUpdates(t,x,u,param)
ref = u(1:12);
x_hat = u(13:24);
z_ref = ref(3);   % z-state
z_hat = x_hat(3); % z-state
dz_ref = ref(6);   % dz-state
dz_hat = x_hat(6); % dz-state
phi = x_hat(7);
theta = x_hat(8);

Kp = param.Kp;
Kd = param.Kd;
g = param.g;
m = param.m;
numlim = param.numlim;
maxlim = param.maxlim;
minlim = param.minlim;

den = cos(phi)*cos(theta);

%% Functional method
% if (mod(phi+pi/2, 2*pi)-pi) > 0
%     T = minlim;
% elseif (mod(theta+pi/2, 2*pi)-pi) > 0
%     T = minlim;
% elseif abs(den) < numlim
%     T = maxlim;
% else
%     T = (m/den) * (g + Kd * (dz_ref - dz_hat) + Kp * (z_ref - z_hat));
%     if T > maxlim
%         T = maxlim;
%     elseif T < minlim
%         T = minlim;
%     end
% end
%% Sinusodial dynamical saturations
n = 2;
if den > 0
    Tmax = maxlim;
	Tmin = minlim;
else
    Tmax = minlim + (maxlim - minlim) * sin(acos(den)).^(2*n);
    Tmin = minlim;
end
T = (m/den) * (g + Kd * (dz_ref - dz_hat) + Kp * (z_ref - z_hat));
if T > Tmax
    T = Tmax;
elseif T < Tmin
    T = Tmin;
end
x(1) = T;
x(2) = Tmax;
sys = x;

%=========================================================================
% Calculate outputs - called on every simulation step, so the
% optimization whould be done in the update discrete states
%=========================================================================
function sys = mdlOutputs(x)
sys = x;
