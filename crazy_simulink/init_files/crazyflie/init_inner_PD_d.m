PD_thrust_param.Kp = 10;
PD_thrust_param.Kd = 5;
PD_thrust_param.h = 0.002;      % s
PD_thrust_param.g = 9.81;       % m/s^2
PD_thrust_param.m = 0.027;      % kg
PD_thrust_param.numlim = 1e-8;

% C0mputes the saturation bounds for the thrust PD controller
sat_lim = 0.1;        % difference between thrust and omega saturation
omega_min_lim = 0;    % rad/s
omega_max_lim = 2500; % rad/s
Tm = 4.*k.^2.*omega_min_lim;
Tp = 4.*k.*omega_max_lim.^2;
T_min_lim = Tm + (Tp - Tm) * sat_lim;
T_max_lim = Tp - (Tp - Tm) * sat_lim;
PD_thrust_param.maxlim = T_max_lim;
PD_thrust_param.minlim = T_min_lim;

PD_angle_param.Kp = 13;
PD_angle_param.Kd = 7;
PD_angle_param.h = 0.002;      % [s]
PD_angle_param.I = [1.660e-5; 1.660e-5; 2.930e-5]; % kg*m^2
PD_angle_param.maxlim = 0.1;
PD_angle_param.minlim = -0.1;