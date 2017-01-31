% Demonstrates the differential flatness property by
% the system and recreating the response using the flatness
% generator.

%% Initializes controllers
run('init_quadcopter_model')
run('init_inner_controller');
run('init_inner_PD_c')

% Computes the saturation bounds for the thrust PD controller
sat_lim = 0.1;        % difference between thrust and omega saturation
omega_min_lim = 0;    % rad/s
omega_max_lim = 2500; % rad/s
Tm = 4.*k.^2.*omega_min_lim;
Tp = 4.*k.*omega_max_lim.^2;
T_min_lim = Tm + (Tp - Tm) * sat_lim;
T_max_lim = Tp - (Tp - Tm) * sat_lim;
tau_max_lim = 1e-5;
tau_min_lim = -1e-5;