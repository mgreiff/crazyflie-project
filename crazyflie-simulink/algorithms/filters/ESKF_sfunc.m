function [sys,x0,str,ts] = ESKF_sfunc(t,x,u,flag,param)

switch flag,
    case 0
        [sys,x0,str,ts] = mdlInitializeSizes(param);
	case 2
        sys = mdlUpdates(t,x,u,param);
	case 3
        sys = mdlOutputs(t,x,u,param);
    case {1, 4, 9}
        sys = [];
    otherwise
        error(['Unhandled flag = ',num2str(flag)]);
end

function [sys,x0,str,ts] = mdlInitializeSizes(par)

% Initialize simsizes
sizes = simsizes;

% Continuous time states
sizes.NumContStates  = 0;

% Discrete states
% (i) number of system states (nX)
% (ii) counter for variable looprates, 1 in total
sizes.NumDiscStates = par.nX + 1;

% Output
% (i) number of system states (nX)
sizes.NumOutputs = par.nX;

% Input
% (i) number of measured states (nM)
% (ii) number of control signals (nC)
sizes.NumInputs = par.nM + par.nC;

% Enable feedthrough and sample times
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;

% Initialize program sizes (for memory)
sys = simsizes(sizes); 

% Initial state
% (i) IC for of system states (initX)
% (ii) clock counter for variable looprates, start at 0
x0 = [par.initX; 0];

% Strings and time steps, run with fastest looprate (IMU)
str = [];         
ts  = [par.hImu 0];

function sys = mdlUpdates(t,x,u,par)
% Loop rate difference
hImu = par.hImu;
hNominal = par.hNominal;
rateMultiple = par.rateMultiple;
count = x(end);

% Extract states
p = x(1:3);
v = x(4:6);
q = x(7:10);
ab = x(11:13);
wb = x(14:16);
g = x(17:19);

% Extract measurements
um = u(1:par.nC);
zm = u(par.nC+1:par.nZ);
am = u(par.nZ+1:par.nZ+3);
wm = u(par.nZ+4:par.nZ+6);

% Update error state
if count == 0
    
    disp('Full kalman prediction and update')
    
    % Jacobian wrt nominal state
    Fx = getJacobian(q, am, wm, ab, wb, par);

    % Jacobian wrt stochastic impulses
    Fi = [zeros(zeros(3, 12));
          eye(12);
          zeros(zeros(3, 12))];

    % Covariance of stochastic impulses
    Qi = blkdiag(par.Vi, par.Thetai, par.Ai, par.Omegai);
    
    %% --------------- ESKF IMU prediction  --------------- 
    % ESKF prediction equation
    %x = predictImu(x, );
    P = F*P*Fx' + Fi*Qi*Fi';
    
    % Update rate counter
    x(end) = rateMultiple;
else
    disp('Error dynamics prediction')
end

% Update counter
x(end) = x(end) - 1;
sys = x;

% Help functions
function sys = mdlOutputs(t,x,u,par)
out = zeros(12,1);
sys = out;

function Xx = get

function Fx = getJacobian(q, am, wm, ab, wb, par)
%% Computes the pertubation impulse jacobian using a zero order integration
% Parameters and dimensions
dt = par.hImu;

R = rotMatQ(q);
Rv = -R * skewS(am - ab);
angle = (wm - wb)*dt;
qangle = angle2Q(angle);
Rang = rotMatQ(qangle)';

I = eye(3);
O = zeros(3);

% Positional terms
Fp = cat(2, I, I*delta_t, O, O, O, O);

% Velocital terms
Fv = cat(2, O, I, Rv*delta_t, -Rv*delta_t, O, I*delta_t);

% Angular terms
Fw = cat(2, O, O, Rang, O, -I*delta_t, O);

% Acceleration drift terms
Fab = cat(2, O, O, O, I, O, O);

% Angular speed drift terms
Fwb = cat(2, O, O, O, O, I, O);

% Gravity estimation terms
Fg = cat(2, O, O, O, O, O, I);

% Concatenates terms
Fx = cat(1, Fp, Fv, Fw, Fab, Fwb, Fg);

function eta = Q2angle(q)
%% Translates a rotational quaternion into euler angles in the Z-Y-X form
phi = atan2(2*(q(1)*q(2) + q(3)*q(4)), q(1)^2 - q(2)^2 - q(3)^2 + q(4)^2);
theta = asin(2*(q(1)*q(3) - q(4)*q(2)));
psi = atan2(2*(q(1)*q(4) + q(2)*q(3)), q(1)^2 + q(2)^2 - q(3)^2 - q(4)^2);
eta = [phi;theta;psi];

function q = angle2Q(eta)
%% Translates euler angles in the Z-Y-X form to a rotational quaternion
phi=  eta(1);
theta = eta(2);
psi = eta(3);

cphi2 = cos(phi/2);
ctheta2 = cos(theta/2);
cpsi2 = cos(psi/2);
sphi2 = sin(phi/2);
stheta2 = sin(theta/2);
spsi2 = sin(psi/2);

q = [cphi2*ctheta2*cpsi2 + sphi2*stheta2*spsi2;
     sphi2*ctheta2*cpsi2 - cphi2*stheta2*spsi2;
     cphi2*stheta2*cpsi2 + sphi2*ctheta2*spsi2;
     cphi2*ctheta2*spsi2 - sphi2*stheta2*cpsi2];

function R = rotMatQ(q)
qw = q(1); qv = q(2:4);
qvx = skewS(qv);
R = (qw^2 - qv' * qv)*eye(3) + 2.*(qv * qv') + 2 .* qw .* qvx;

function out = productQ(p, q)
pw = p(1); px = p(2); py = p(3); pz = p(4); 
qw = q(1); qx = q(2); qy = q(3); qz = q(4); 
out = [pw*qw - px*qx - py*qy - pz*qz;
       pw*qx - px*qw - py*qz - pz*qy;
       pw*qy - px*qz - py*qw - pz*qx;
       pw*qz - px*qy - py*qx - pz*qw];

function S = skewS(v)
vx = v(1); vy = v(2); vz = v(3);
S = [  0, -vz, vy;
      vz,   0, vx;
     -vy,  vx,  0];


