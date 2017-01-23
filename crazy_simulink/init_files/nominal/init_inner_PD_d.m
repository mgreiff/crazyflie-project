PD_thrust_param.Kp = 1.5;
PD_thrust_param.Kd = 2.5;
PD_thrust_param.h = 0.002;      % [s]
PD_thrust_param.g = 9.81;       % m/s^2
PD_thrust_param.m = 0.468;      % kg
PD_thrust_param.numlim = 1e-8;
PD_thrust_param.maxlim = 20;
PD_thrust_param.minlim = 0.5;

PD_angle_param.Kp = 8;
PD_angle_param.Kd = 4.5;
PD_angle_param.h = 0.002;      % [s]
PD_angle_param.I = [4.856e-3;4.856e-3;8.801e-3]; % kg*m^2
PD_angle_param.maxlim = 0.1;
PD_angle_param.minlim = -0.1;