function [sys,x0,str,ts] = sfunc_inner_PD_angle_d(t,x,u,flag,param)

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
sizes.NumDiscStates  = 3;
sizes.NumOutputs     = 3;
sizes.NumInputs      = 24;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes); 

x0 = [0;0;0]; % tau (control signal)
 
str = [];
ts  = [param.h 0];
		      
function sys = mdlUpdates(t,x,u,param)
ref = u(1:12);
x_hat = u(13:24);
eta_ref = ref(7:9);   % z-state
eta_hat = x_hat(7:9); % z-state
deta_ref = ref(10:12);   % dz-state
deta_hat = x_hat(10:12); % dz-state

Kp = param.Kp;
Kd = param.Kd;
I = param.I;
maxlim = param.maxlim;
minlim = param.minlim;

tau = diag(I) * (Kd .* (deta_ref - deta_hat) + Kp .* (eta_ref - eta_hat));
tau(tau > maxlim) = maxlim;
tau(tau < minlim) = minlim;
x(1:3) = tau;
sys = x;

function sys = mdlOutputs(x)
sys = x;
