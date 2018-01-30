function omega = controller_avoidobs(r,rdot,rddot,eta,etadot,etaddot, g, m, k, A, I, l, b,obs,obs_dot,obs_ddot)

%%% Desired Position
r_des = [2;2;3];
rdot_des = [0;0;0];
rddot_des = [0;0;0];
%%% Desired configuration;
eta_des = [deg2rad(0);deg2rad(0);deg2rad(10)];
etadot_des = zeros(3,1);
etaddot_des = zeros(3,1);

%%% Define a Lyapunov Function
x_des = [(r-r_des);(rdot-rdot_des);(eta-eta_des);(etadot-etadot_des)];
P = 10*eye(12);
V = x'*P*x;
%%%%%  Need f and g
LfV = x'*P*f; 
LgV = x'*P*g;

%%% Define a Barrier Function
x_obs = [(r-r_obs);(rdot-rdot_obs)] %;(eta-eta_obs);(etadot-etadot_obs)];
d = 
B = log(1/x_obs);   
   
%%%%% Create the Optimization Problem 
%% Define the varible
%% omega0 = rand(4,1);
%% omega_lb = [];
%% omega_ub = []

x0 = fmincon(@fun, omega0, omega_lb,omega_ub, )

end   
   
    