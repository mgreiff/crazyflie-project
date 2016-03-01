g = 9.81;       % m/s^2
m = 0.468;      % kg
l = 0.225;      % m
k = 2.980e-6;   % unitless
b = 1.140e-7;   % unitless
Im = 3.357e-5;  % kg*m^2
I = [4.856e-3;  % kg*m^2
     4.856e-3;  % kg*m^2
     8.801e-3]; % kg*m^2
A = [0.25;      % kg/s
	 0.25;      % kg/s
	 0.25];     % kg/s
 
% Thrust per rotor required to hover steadily
hover_omega = sqrt(g*m/(4*k));

generate_example_omega