function [sys,x0,str,ts] = eval_traj_sfunc2(t,x,u,flag,param)

switch flag,
    case 0
        [sys,x0,str,ts] = mdlInitializeSizes(param);
	case 2
        sys = mdlUpdates(u,param);
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
sizes.NumDiscStates  = 20; % Four flat outputs with five derivetives each
sizes.NumOutputs     = 20; % Four flat outputs with five derivetives each
sizes.NumInputs      = 1;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes); 

x0 = zeros(1,20);          % Initializes trajectory to 0
str = [];
ts  = [param.h 0];

function sys = mdlUpdates(u, param)
T = u;
P = param.Pmat;
N = param.N;
times = param.times;

values = zeros(5,4);
for ii = 1:4
    values(:,ii) = eval_traj(P(:,ii), times, N, T);
end
sys = reshape(values', [20,1]);

function sys = mdlOutputs(x)
sys = x;
