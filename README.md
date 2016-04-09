## Directories

#### /crazy_ros/*
Currently contains (1) the master node, enabling interation from the terminal (2) the reference generator node, which sents a signal to the \reference_signal topic. The quadcopterModel node (3) listens to this topic and computes the
system response (in terms of the measured states) and sends the data to a \measured_states topic. The computation is done by first evaluating the nonlinear continuous time system matrices, and then computing the ZOH discrete equivalent to condut the time stepping. The purpose of this is to simulate the system with ROS to test different methods of control before connecting to the crazyflie.

An issue that needs to be met is the nondeterministic behaviour of the ROS publishers (see eg. \reference_executiontime), which cannot be used in critical parts of the control system. Instead, tests should be made with ZMQ and the realtime_tools C++ layer.
###### Contents
* /scripts/* - Contains the code for generating nodes.
* /msg/* - Contains the message definitions used.
* /config/* - Contains a JSON configuration file for all ROS nodes.
* /launch/* - Contains the launch XML file.
* /example/* - The raw python file for the quadcopter dynamics, example of what a system response in harder realtime will look like.

###### Useful ROS commands
* ``roslaunch crazy_ros crazy.launch`` - Launches the entire project in crazy_ros (package) using the crazy.launch XML file.
* ``rosrun rqt_graph rqt_graph`` - Plots a graph of the ROS structure.
* ``rosrun rqt_plot rqt_plot <topic>``  - Plots the values of the topic ``\measurements\data[0]`` displays the x-measurements in time.
* ``rostopic echo <topic>`` - Prints the data sent to the topic in the terminal window.

#### /crazy_documentation/*
The project documentation, will eventually include a report discussion both the theory and implementation of the project. 

###### Contents
* **Report.tex** - A report containing a discussion on the dynamics, filtering, motion planning, PD, MPC, L1 control and TODO's.

#### /crazy_trajectory/*
Contains the scripts for motion planning (see crazy_documentation).

###### Status
Everything works as expected, but the boundary conditions (enforced by A0 and AT) for preserving continuity across underfined spline points (referred to ass free EC's in the code) currently does not work. It remains to figure out why, and the problem seems to be in how these constraints are defined.

###### Contents
* **get_A.m** - Computes the constraint matrix of a polynomial spline.
* **get_Q.m** - Computes the hessian of a signle polynomial spline (squared).
* **compute_splines.m** - Assembles the global Q and A matrices and minimizes the const function using quadprog.
* **plot_splines.m** - Plots the position-, velocity-, acceleation- and jerkprofiles of the complete trajectory taking the output of quadprog and the time array as input.
* **splines_2_1D_example.m** - An example which generates the minimum snap trajectory when constraining positional endpoints using 1-3 polynomials of degree 3-5.
* **reftraj_compute_example.m** - Computes a 3D reference trajectory of splines parameterized in time, where the polynomial coefficients (Nx3), breakpoint times (1xN) and polynomial order (integer) are store in a struct and save to a .mat file.
* **reftraj_eval_example.m** - Evaluates the trajectory in the points on the predictions horizion and plots the result. When times before t_0 or after t_f are requested, the reference is set to the corresponding endpoint on the entire trajetory. This function effectively computes Rref and was implemented in the /Examples/mpc_cvx/mpc_cvx.slx file.

#### /crazy_simulink/*
Contains the files used in modelling of the quadcopter. Many of the files are works in progress and some cannot be run at all. The necessary paths and parameters are set up by tunning the ``simulink_init.m`` file, after which the examples in the /Examples/* directory can be run. Some of these may need additional parameters, in which case an ``init_.m`` file is located in the same directory as the example subdirectory. The mpc_cvx_test and LQR_control_test can be run but are far from complete just yet, everything else should be OK. 

###### Contents - Models (.mdl and .slx)
* /quadcopter_model/**quadcopter_model.mdl** - A process model of the quadcopter based on the Newton-Lagrange equations.
* /quadcopter_model/**linearization.m** - A script for generating a symbolic expression for the jacobian of the system, the thought was here to derive such an expression which became infeasibly complex (takes ~0.07 s to evaluate with a complete linearization).
* /PD_controller/**quadcopter_PD_controller.mdl** - A model for mapping control errors in x, y and z to control signals in phi, theta and z to rotor angular velocities squared.
* /PD_controller/**quadcopter_PD_controller_with_thrust.mdl** - Uses thrust and angular references as control signals, used in the MPC controllers.
* /L1_controller/**adaptationlaw_with_projection_op.slx** - A simple adaptation law with the projection operator validated by comparison to the work of Chengyu Cao and Naira Hovakimyan (see /crazy_documentation).
* /kalman_filters/**discrete_kalman_filter.slx** - Implementation of a simple kalman filter for the linearized system (currently dysfuntional in Matlab, works in Python)
* /kalman_filters/**extended_discrete_kalman_filter.slx** - Implementation of an extended kalman filter for the quadcopter process (currently dysfuntional in Matlab, works in Python).

###### Contents - Examples (/examples/pd_*/* and /examples/quadcopter_model_test/*)
* /quadcopter_model_test/**quadcopter_process_test.slx** - An example of the dynamics responding to rotor speeds, replicating the results in th work of Lukkonens (requires init_process_test.m to run).
* /pd_control_test/**quadcopter_pd_test.slx** - Demonstrates the system response with only the inner stabilising PD controller.
* /pd_position_control_test/**quadcopter_pd_position_test.slx** - Demonstrates the system response with an inner PD and outer PID position controller.

###### Contents - Examples (/examples/mpc_control_test/*)
* /mpc_control_test/**quadcopter_mpc_position_test.slx** - Demonstrates the system response with an outer inner PD and outer MPC controller using MPC_tools (requires init_MPC_test.m to run). Functional and works well, but is very slow.

###### Contents - Examples (/examples/mpc_cvx_test/*)
* /mpc_cvx_test/**csolve.mexmaci64** - CVXgen .mex-solver used in the S-function calls (see /crazy_documentation for solver definition).
* /mpc_cvx_test/**mpc_trajectory_data_callback.m** - Callback for loading the trajectory data properly into Simulink.
* * /mpc_cvx_test/**example_mpc_trajectory.slx** - Shows how the trajectory data is imported into and evaluated in Simulink. As can be seen, there is a slight difference between the result generated in simulink (incorrect) and the result generated outside of the simulink environment (see /crazy_trajectory/reftraj_eval_example.m). This needs to be fixed somehow.
* /mpc_cvx_test/**example_mpc_cycle.m** - Computes and plots the result one MPC cycle optimization to show the solver syntax.
* /mpc_cvx_test/**mpc_cvx_sfunc.m** - An M S-function implementing the csolve.m handle for solving the optimization problem on every iteration in Simulink. We need to think through how we want to define the control signal and how this is updated, currently, u_0 = [0,0,0]' on every iteration, and there is no state feedback. The file simply demonstrates that we can evaluate the trajectory everey Ts [s] and update the control signal using the S-file!

###### Contents - Examples (/examples/LQR_control_test/*)
*	/LQR_control_test/**quadcopter_LQR_test.slx** - Uses the model without a stabilising PD controller and therefore requires a truncated model to run (removing the x and y states, as these are not controllable). The model behaves as expected, with a stationary error in the z-position and very nice angular response. Should be improved by including feedforward/integrators in the LQR scheme to avoid the stationary errors.
