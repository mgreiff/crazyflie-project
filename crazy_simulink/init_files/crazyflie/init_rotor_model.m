%% DC motor parameters (simulink type model)
Jp = 0.03;
Jm = 0.13;
b = 0.1;
Kt = 580;
Ke = 0.001;
R = 2.3;
L = 0.1;

%% DC motor parameters (continuous statespace type model)
Ap = [ -b/Jp, Kt/Jp; -Ke/L, -R/L];
Am = [ -b/Jm, Kt/Jm; -Ke/L, -R/L];
B = [0; 1/L];
C = [0,1];
syscp = ss(Ap,B,C,[]);
syscm = ss(Am,B,C,[]);

%% DC motor parameters (discrete statespace type model)
relay_lim = 1e4;
timestep = 0.002;
sysdp = c2d(syscp, timestep, 'zoh');
sysdm = c2d(syscm, timestep, 'zoh');
Adp = sysdp.A;
Adm = sysdm.A;
Bd = sysdm.B;
Cd = sysdm.C;
Dd = sysdm.D;

%% DC motor initial conditions
x0 = [0;0];