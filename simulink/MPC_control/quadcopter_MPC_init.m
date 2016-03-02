%% ~~~ Creates a linearized system for MPC control ~~~
% Discretization time
Ts = 0.025;

% Thrust mapping
At = [1, Ts;0, 1];
Bt = [Ts.^2/2;Ts];
Ct = [1, 0];

% Angular mapping - constants in the mapping of the rotor angles are TBD
n0 = 40;
d1 = 8;
d2 = 80;
Acont = [-d1,1;
         -d2,0];
Bcont = [0;
         n0];
Ccont = [1,0];

% Combining thrust and angular mapping
syscont = ss(Acont,Bcont,Ccont,[]);
sysdisc = c2d(syscont,Ts);

Atheta = sysdisc.A;
Btheta = sysdisc.B;
Ctheta = sysdisc.C;

A4 = [At, Bt*Ctheta;
      zeros(2), Atheta];
B4 = [zeros(2,1);
      Btheta];

A8 = [A4, zeros(4);
      zeros(4), A4];
B8 = [B4, zeros(4,1);
      zeros(4,1), B4];

%%Elevation mapping
Kt = 0.91; % Thrust ratio TBD empirically
m = 1.3;   % Mass
g = 9.81;

Az = [1,Ts;0,1];
Bz = (-Kt/m).*[Ts.^2/2;Ts];

% Complete ss-model
A = [A8,zeros(8,2);
     zeros(2,8),Az];
B = [B8,zeros(8,1);
     zeros(2,2),Bz];

C = kron(eye(5),[1,0]);

k = zeros(10,1); % How does k enter into the ss-model?
k(9) = (g/m)*Ts.^2/2;
k(10) = (g/m)*Ts;

syscomplete = ss(A,B,C,[],0.025);

%% ~~~ Creates a linearized system for MPC control ~~~
% **Sets up MPC parameters**

% System matrices for the plant
Ad = syscomplete.A;
Bd = syscomplete.B;
Cyd = syscomplete.C;

% Matrices defining the controlled outputs
Czd = Cyd;
Dzd = syscomplete.D;

% Matrices defining the constrained outputs
Ccd = Cyd;
Dcd = syscomplete.D;

% Horizons
Hw = 10;
Hp = 10;
Hu = 10;

% Blocking factors
zblk = 1;
ublk = 1;

% Control variable limits TBD
u_min = [10,10,10]';
u_max = [-10,-10,-10]';

% Control increment limits TBD
du_min = [-inf,-inf,-inf]';
du_max = [inf,inf,inf]';

% Controlled variable limits TBD
z_min = [10,10,10,10,10]';
z_max = [-10,-10,-10,-10,-10]';

% Weighting matrices
Q = diag([1,1,1,1,1]);      % 5 measured states in total
R = diag([10,1,1]);         % 3 control signals in total
W = diag(zeros(10,1));      % 10 states in total
V = 0.01.*diag(zeros(5,1)); % 5 rows in C.

% ** Creates MPC controller**
md = MPCInit(Ad,Bd,Cyd,Czd,Dzd,Ccd,Dcd,Hp,Hw,zblk,Hu,ublk, ...
	    du_max,du_min,u_max,u_min,z_max, ...
	    z_min,Q,R,W,V,Ts,2,'qp_as');