% ~~~ Creates a linearized system for MPC control ~~~

%% Discretization time
Ts = 0.025;

%% Thrust mapping
At = [1, Ts;0, 1];
Bt = [Ts.^2/2;Ts];
Ct = [1, 0];

%% Angular mapping
% Constants in the mapping of the rotor angles, TBD empirically
n0 = 40;
d1 = 8;
d2 = 80;
Acont = [-d1,1;
         -d2,0];
Bcont = [0;
         n0];
Ccont = [1,0];

%% Combining thrust and angular mapping
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

%% Elevation mapping
% TBD empirically
Kt = 0.91; % Thrust ratio
m = 1.3;   % Mass
g = 9.81;

Az = [1,Ts;0,1];
Bz = (-Kt/m).*[Ts.^2/2;Ts];

%% Complete ss-model
A = [A8,zeros(8,2);
     zeros(2,8),Az];
B = [B8,zeros(8,1);
     zeros(2,2),Bz];

C = kron(eye(5),[1,0]);

k = zeros(10,1); % How does k enter into the ss-model?
k(9) = (g/m)*Ts.^2/2;
k(10) = (g/m)*Ts;

syscomplete = ss(A,B,C,[],0.025);

%% TODO set up MPC parameters

%% TODO create MPC controller