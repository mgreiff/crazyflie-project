% Simulation of a nonlinear quadruple tank lab process controlled by
% an MPC controller using Simulink.
%
% Copyright Johan Åkesson 2006
%


clear all
path(path,'../')
% Model for the four tank process

% Process parameters
A1=28; % cm
A2=32;
A3=28;
A4=32;

a1=0.071; % cm
a2=0.057;
a3=0.071;
a4=0.057;

kc=0.5; % V/cm
g=981;  % cm/s^2

v10_nmp=3.00; % V
v20_nmp=3.00;

k1_nmp=3.33; % cm^3/Vs
k2_nmp=3.35; % cm^3/Vs

g1_nmp=0.25; % ratio of allocated punmp capacity between lower and
            % upper tank
g2_nmp=0.35;

% If symbolic toolbox is installed, this code could be used to
% calculate the stationary values
% Stationary values, non-minimum phase
%S = solve('-a1/A1*sqrt(2*g*h10_nmp)+a3/A1*sqrt(2*g*h30_nmp)+g1_nmp*k1_nmp/A1*v10_nmp',...
%	  '-a2/A2*sqrt(2*g*h20_nmp)+a4/A2*sqrt(2*g*h40_nmp)+g2_nmp*k2_nmp/A2*v20_nmp',...
%	  '-a3/A3*sqrt(2*g*h30_nmp)+(1-g2_nmp)*k2_nmp/A3*v20_nmp',...
%  '-a4/A4*sqrt(2*g*h40_nmp)+(1-g1_nmp)*k1_nmp/A4*v10_nmp',...
%  'h10_nmp,h20_nmp,h30_nmp,h40_nmp');

%h10_nmp = eval(S.h10_nmp);
%h20_nmp = eval(S.h20_nmp);
%h30_nmp = eval(S.h30_nmp);
%h40_nmp = eval(S.h40_nmp);

h10_nmp = 8.24441415257276;
h20_nmp = 19.01629576919927;
h30_nmp = 4.31462580236556;
h40_nmp = 8.80652939083585;

% Build state space model, minimum phase
T1_nmp=A1/a1*sqrt(2*h10_nmp/g);
T2_nmp=A2/a2*sqrt(2*h20_nmp/g);
T3_nmp=A3/a3*sqrt(2*h30_nmp/g);
T4_nmp=A4/a4*sqrt(2*h40_nmp/g);

A_nmp=[-1/T1_nmp 0 A3/(A1*T3_nmp) 0;
      0 -1/T2_nmp 0 A4/(A2*T4_nmp);
      0 0 -1/T3_nmp 0;
      0 0 0 -1/T4_nmp];

B_nmp=[g1_nmp*k1_nmp/A1 0;
      0 g2_nmp*k2_nmp/A2;
      0 (1-g2_nmp)*k2_nmp/A3;
      (1-g1_nmp)*k1_nmp/A4 0];

C_nmp=[kc 0 0 0; % Notice the measured signals are given in Volts!
      0 kc 0 0];

D_nmp=zeros(2,2);

h = 3;

% Constraints
% No constraints on du
% Pump capacities [0 10]V
% Level 1 [0 20]cm = [0 10]V
% Level 2 [0 20]cm
% Level 3 [0 h30_nmp+8]cm
% Level 4 [0 20]cm
du_max = [inf inf]'; % limit on delta u; slew rate
du_min = [-inf -inf]';
u_max = [10-v10_nmp 10-v20_nmp]'; % limit absolute value of u
u_min = [-v10_nmp -v20_nmp]';
z_max = [10-h10_nmp*kc-0.1 10-h20_nmp*kc-0.1 10-h30_nmp*kc-0.1 10-h40_nmp*kc-0.1]'; % Limits on controlled outputs
z_min = [-h10_nmp*kc -h20_nmp*kc -h30_nmp*kc -h40_nmp*kc]'; 

% Set point trajectory including setpoints for u:s
s = [zeros(round(60/h),1); 3*ones(1400/h,1)];
s = [s zeros(length(s),1)];

% Input disturbance trajectory
d = [zeros(round(600/h),1); -1*ones(860/h,1)];
d = [zeros(length(d),1) d];

% MPC parameters
Hp = 30; % Prediction horizon
Hu = 10; % Horizon for varying input signal
Hw = 1; % First penalty sample
zblk=2;
ublk=2;

Q = diag([4 1]);
R = 0.01*diag([1 1]);

W = diag([1 1 1 1]);
V = diag(0.01*ones(1,2));

[Ad, Bd, Cyd, Dzd]=ssdata(c2d(ss(A_nmp,B_nmp,C_nmp,D_nmp),h));

Czd = Cyd;
Dzd = zeros(2,2);


Ccd = 0.5*eye(4);
Dcd = zeros(4,2);

md = MPCInit(Ad,Bd,Cyd,Czd,Dzd,Ccd,Dcd,Hp,Hw,zblk,Hu,ublk, ...
	    du_max,du_min,u_max,u_min,z_max, ...
	    z_min,Q,R,W,V,h,2,'qp_as');

t = 0:h:((size(s,1)-1)*h);

sim_u = [t' s d];

sim_opts = simset;

[tt,x_internal,xx] = sim('QuadTank_MPC',[0 t(end)],sim_opts,sim_u);

x = xx(:,1:4);
u = xx(:,5:6);

u(:,1) = u(:,1) + v10_nmp;
u(:,2) = u(:,2) + v20_nmp;

figure(1);
clf
subplot(3,2,1)
hold on
stairs(tt,x(:,3),'b--')
%stairs(tt,kc*x_est(3,:),'--r')
ylabel('h_3 [cm]')
%title('h3')
subplot(3,2,2)
hold on
stairs(tt,x(:,4),'b--')
%stairs(tt,kc*x_est(4,:),'--r')
ylabel('h_4 [cm]')
%title('h4')
subplot(3,2,3)
hold on
stairs(tt,x(:,1),'b--')
%stairs(tt,kc*x_est(1,:),'--r')
%plot(tt,s(1:max(tt)/h+1,1)','--');
ylabel('h_1 [cm]')
%title('h1')
subplot(3,2,4)
hold on
stairs(tt,x(:,2),'b--')
%stairs(tt,s(1:max(tt)/h+1,2)','--');
%stairs(tt,kc*x_est(2,:),'--r')
ylabel('h_2 [cm]')
%title('h2')
subplot(3,2,5)
hold on
stairs(tt,u(:,1),'b--')
%title('u1')
ylabel('u_1 [V]')
xlabel('t [s]')
subplot(3,2,6)
hold on
stairs(tt,u(:,2),'b--')
%title('u2')
ylabel('u_2 [V]')
xlabel('t [s]')
zoom on

W = diag([1 1 1 1 1 1]);

md = MPCInit(Ad,Bd,Cyd,Czd,Dzd,Ccd,Dcd,Hp,Hw,zblk,Hu,ublk, ...
	    du_max,du_min,u_max,u_min,z_max, ...
	    z_min,Q,R,W,V,h,4,'qp_as');

[tt,x_internal,xx] = sim('QuadTank_MPC',[0 t(end)],sim_opts,sim_u);

x = xx(:,1:4);
u = xx(:,5:6);

u(:,1) = u(:,1) + v10_nmp;
u(:,2) = u(:,2) + v20_nmp;

s = s/kc;
s(:,1) = s(:,1)+h10_nmp;
s(:,2) = s(:,2)+h20_nmp;

figure(1);
subplot(3,2,1)
hold on
stairs(tt,x(:,3),'b')
%stairs(tt,kc*x_est(3,:),'--r')
ylabel('h_3 [cm]')
%title('h3')
grid
axis([0 1200 2 14])
subplot(3,2,2)
hold on
stairs(tt,x(:,4),'b')
%stairs(tt,kc*x_est(4,:),'--r')
ylabel('h_4 [cm]')
%title('h4')
grid
axis([0 1200 2 10])
subplot(3,2,3)
hold on
stairs(tt,x(:,1),'b')
%stairs(tt,kc*x_est(1,:),'--r')
plot(t,s(1:max(tt)/h+1,1)','-.');
ylabel('h_1 [cm]')
%title('h1')
grid
axis([0 1200 6 16])
subplot(3,2,4)
hold on
stairs(tt,x(:,2),'b')
stairs(t,s(1:max(tt)/h+1,2)','-.');
%stairs(tt,kc*x_est(2,:),'--r')
ylabel('h_2 [cm]')
%title('h2')
grid
axis([0 1200 17 20])
subplot(3,2,5)
hold on
stairs(tt,u(:,1),'b')
%title('u1')
ylabel('u_1 [V]')
xlabel('t [s]')
grid
axis([0 1200 0 4])
subplot(3,2,6)
hold on
stairs(tt,u(:,2),'b')
grid
%title('u2')
ylabel('u_2 [V]')
xlabel('t [s]')
axis([0 1200 2 8])
zoom on
  
