% Simulation of a nonlinear helicopter lab process controlled by an
% MPC controller using Simulink.
%
% Copyright Johan Åkesson 2006
%

path(path,'../')
% Parameters:
Je = 0.91;   % kgm^2, Moment of inertia about elevation axis 
la = 0.66;   % m, Arm length from elevation axis to helicopter body
Kf = 0.5;    % Motor Force Constant
Fg = 0.5;    % N, Differential force due to gravity and counter
             % weight
Tg = la*Fg;  % Nm, Differential torque
Jp = 0.0364; % kgm^2, Moment of inertia about pitch axis
lh = 0.177;  % m, Distance from pitch axis to either motor
Jt = Je;     % Moment of inertia about travel axis

u1_max = 3;
u1_min = -2;
u2_max = 3;
u2_min = -2;

elev_max = 0.6;
elev_min = -0.5;
pitch_max = 1;
pitch_min = -1;

% Linearize around elevation = 0, pitch = 0, position = 0

x_stat = [0 0 0 0 0 0]';
u_stat = [Tg/(2*Kf*la) Tg/(2*Kf*la)]';

% States: 
% x1 = elevation 
% x2 = elevation vel
% x3 = position
% x4 = position vel
% x5 = pitch
% x6 = pitch vel

A = [0 1 0 0 0 0;
     0 0 0 0 0 0;
     0 0 0 1 0 0;
     0 0 0 0 -Fg*la/Jt 0;     
     0 0 0 0 0 1;
     0 0 0 0 0 0;
     ];

B = [0 0;
     Kf*la/Je Kf*la/Je;
     0 0;
     0 0;
     0 0;
     Kf*lh/Jp -Kf*lh/Jp];

Ct = eye(6);

Cy = [1 0 0 0 0 0;
      0 0 1 0 0 0;
      0 0 0 0 1 0];

Dt = zeros(6,2);

Dy = zeros(3,2);

x0 = x_stat;

h = 0.12;

sysc = ss(A,B,Ct,Dt);

[Ad,Bd,Cd,Dd] = ssdata(c2d(sysc,h));

Cz = [1 0 0 0 0 0;
      0 0 1 0 0 0];

Dz = zeros(2,2);

Cc = [1 0 0 0 0 0;
      0 0 0 0 1 0];
Dc = zeros(2,2);

z_max = [0.6 1];
z_min = [-0.5 -1];
u_max = [3 3];
u_min = [-2 -2];
du_max = [inf inf];
du_min = [-inf -inf];

Q = diag([1 1]);
R = diag([0.1 0.1]);

cmode = 0;

Hp = 30;
Hu = 10;
Hw = 1;

u_blk = 2;
z_blk = 2;

md = MPCInit(Ad,Bd,Cd,Cz,Dz,Cc,Dc,Hp,Hw,z_blk,Hu,u_blk,...
	     du_max,du_min,u_max,u_min,z_max,z_min,Q,R,[],[],h,cmode, ...
	     'qp_ip');

[tt,x_internal,xx] = sim('helimodel_mpc',[0 50]);

figure(1)
clf
subplot(2,2,1)
stairs(tt,xx(:,1))
hold on
stairs(tt,xx(:,9),'--')
axis([0 30 -0.1 0.4])
grid
ylabel('Elevation [rad]')
subplot(2,2,3)
stairs(tt,xx(:,5))
axis([0 30 -1.1 1.1])
grid
xlabel('t [s]');
ylabel('Pitch [rad]')
subplot(2,2,2)
stairs(tt,xx(:,3))
hold on
stairs(tt,xx(:,10),'--')
axis([0 30 -1 4])
grid
ylabel('Rotation [rad]')
subplot(2,2,4)
stairs(tt,xx(:,7));
hold on
stairs(tt,xx(:,8));
axis([0 30 -2 4])
grid
ylabel('V_f, V_b [V]')
xlabel('t [s]');

