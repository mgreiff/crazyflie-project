function init_project( ARG1, ARG2, ARG3 )
    %% Status
    % algorithm [Complete]
    %     * RLS - Demonstrates the on-line identification of parameters
    %         in a 3x3 pulse transfer function corresonding to the voltage
    %         to current function in the rotor dynamics. Demonstrates both
    %         the constant trace and reglar RLS formulations. [Complete]
    %     * map - Demonstrates the problem of conversion from Tait-Bryan
    %         angles to quaternions and back, and allow for experimentation
    %         with the conversion maps.                       [Complete]   
    %     * flatness - Demonstrates the differential flatness property by
    %         the system and recreating the response using the flatness
    %         generator.                                      [Complete]
    %     * trajectory - Example of the discrete time trajectory 
    %         evaluation of polynomial trajectories.          [Complete]
    %     * GPF - Example of the discrete generic particle filter and
    %         how the state filters are implemented with a simple double
    %         integrator example.                             [Complete]
    %
    % rotor [Complete]
    %     * identification - Scripts and example for system identificaiton
    %         of a linear rotor model with variable inertia.     [Complete]
    %     * response - Models for demonstrating the rotor system
    %         response in continuous and discrete time.          [Complete]
    %     * control - Example showing the system response of the discrete
    %         time SISO PID controller and it's disturbance rejecting
    %         properties when tun on the continuous time rotor.  [Complete]
    %     * loop - Example of the full MIMO system with four coupled SISO
    %         rotors and adaption of the k parameter.            [Complete]
    %     * estimation - Example of a dual kalman filter used for
    %         estimating the rotor speed. Complete but needs more work to
    %         deal with the limit cycle arising in the discrete time model
    %         in the kalman filter.                              [Complete]
    %
    % quadcopter [Complete]
    %     * identification - Scripts and example for system identificaiton
    %         of a linearized quadcopter model by using the open loop
    %         non-linear response.                               [Complete]
    %     * response - Models for demonstrating the quadcopter system
    %         response in continuous and discrete time excluding rotor
    %         dynamics, comparable to the work of Lukkonen.      [Complete]
    %     * aggressive - Uses the augmented dynamics to show how
    %         singularities in the model can be handled.         [Complete]
    %     * control - Demonstrates the inner loop controller, mapping
    %         references to thrusts and torques by means of PD, LQR, LQRi
    %         SE(3), APPC controllers.                           [Complete]
    %     * estimation

    %% Initiates the Simulink project
    root = pwd;
    options.algorithm = {'RLS', 'flatness', 'map', 'trajectory', 'GPF'};
    options.rotor = {'identification', 'response', 'loop', 'estimation', 'control'};
    options.quadcopter = {'identification', 'response','aggressive','estimation', 'control'};
    dirs = fieldnames(options);
    
    % Help printout
    if strcmp(ARG1, '-h') || strcmp(ARG1, '--h') || strcmp(ARG1, 'h')
        helpstr = ['Launches the simulink project by setting up paths,\n',...
                   'running necessary init files and cding to the\n',...
                   'example [ARG1] with an subexample [ARG2] in the\n',...
                   'configuration [ARG3], where the configuration is a\n',...
                   'string containing "nominal" och "crazyflie".\n\n',...
                   'Example usage: init_project("inner","control","nominal")\n\n',...
                   'Valid combinations of ARG1 and ARG2 are\n'];
        for ii = 1:length(dirs)
            helpstr = strcat(helpstr, ['    ', dirs{ii}, '\n']);
            opt = getfield(options, dirs{ii});
            for jj = 1:length(opt)
                helpstr = strcat(helpstr,['        * ', opt{jj}, '\n']);
            end
        end
        fprintf(helpstr)
        return
    end
    
    % Add directories
    basedirs = {'models',...
                'controllers/rotor_controller',...
                'controllers/inner_controller',...
                'algorithms/filters',...
                'algorithms/adaption',...
                'algorithms/RLS',...
                'algorithms/madgwick',...
                'algorithms/flatness',...
                'trajectory_files',...
                'analysis'};
    try
        for ii = 1:length(basedirs)
            addpath([root, '/', basedirs{ii}])
        end
    catch
        disp(['Could not initiate all directories'])
        return
    end
    
    % Add init files
    configurations = {'nominal', 'crazyflie'};
    newconfig =  [root, '/', 'init_files/', ARG3];
    pathCell = regexp(path, pathsep, 'split');
    for ii = 1:length(configurations)
        oldconfig =  [root, '/', 'init_files/', configurations{ii}];
        if any(strcmp(oldconfig, pathCell))
            rmpath(oldconfig)
        end
    end
    if any(strcmp(configurations, ARG3))
        addpath(newconfig)
    else
        fprintf(['Could not locate init files, check that "/init_files/', ARG3, '" exists.\n'])
        return
    end

    % Sanity check and cd
    if sum(strcmp(dirs, ARG1)) == 0
        fprintf(['The argument ARG1 = "', ARG1,'" is not supported.\n',...
                 'Type -h for help.\n'])
        return
    end
    if sum(strcmp(getfield(options, ARG1), ARG2)) == 0
        fprintf(['The combination ARG1 = "', ARG1,'" with ARG2 = "', ARG2,...
                 '" is not\nsupported. Type -h for help.\n'])
        return
    end
    try
        cd([root, '/examples/', ARG1, '_', ARG2])
        run('init');
        save('configuration')
        evalin('caller', 'load(''configuration'')')
    catch
        fprintf('Failed to initialize directory')
        return
    end
    return
 end