%% Initialize rotor model
run('init_rotor_model')

%% Initialize discretization step and load/measurement disturbances
measurement_variance = 15;
measurement_mean = 0;

% Load disturbance
load_amp = 1;
load_start = 10;
load_finish = 22;

% Reference signals
ref_mean = 2000;
ref_amp = 200;
pulse_period = 8; % [s]
sinusoid_frequency = 1; %[rad/s]

% Discretization step
rotor_h = 0.002;

dir = input(['Choose one of the following examples by writing a corresponding\n',...
             'string ARG and then press enter.\n\n',...
             '    * "PID" - Simulates SISO rotor control using a discrete\n',...
             '        time PD controller.\n',...
             '    * "MRAC" - Simulates control using model reference adaptive\n',...
             '        control.\n',...
             '    * "MOPC" - Simulates control with minimal order polynomial\n',...
             '        design.\n',...
             '    * "AMOPC" - Simulates control using an active minimal\n',...
             '        order polynomial design using a constant trace RLS\n',...
             '        algorithm (still a bit buggy).\n\nInput: ']);
if sum(strcmp({'PID','MRAC','MOPC','AMOPC'}, dir)) == 0
    fprintf(['The option "', dir, '" is not supported. \nRe-run init.m to try something else.\n'])
else
    try
        cd(['example ', dir,' (discrete)'])
        fprintf('\nInitilizing...\n')
        run('init')
    catch
        fprintf('Warning. Failed to initialize the directory.\n')
    end
    try
        fprintf('Launching model...\n')
        open(['example_rotor_', dir])
        sim(['example_rotor_', dir])
    catch
        fprintf('Warning. Failed to open Simulink model.\n')
    end
end