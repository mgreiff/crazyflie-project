## Directories

#### /crazy_ros/*
The project currently contains (1) the master node, enabling interation from the terminal (2) the reference generator node, which sents a signal to the \reference_signal topic. The quadcopterModel node (3) listens to this topic and computes the
system response (in terms of the measured states) and sends the data to a \measured_states topic. The computation is done by first evaluating the nonlinear continuous time system matrices, and then computing the ZOH discrete equivalent to condut the time stepping. The purpose of this is to simulate the system with ROS to test different methods of control before connecting to the crazyflie.

Currently, an issue that needs to be met is the nondeterministic behavious of the ROS publishers (see eg. \reference_executiontime), which cannot be used in critical parts of the control system. Instead, tests should be made with ZMQ and the realtime_tools C++ layer.
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

#### /VM_scripts/*
Contains the scripts that run on the Bitcraze VM. This part of the project is not necessary for running any other script, and will be removed in the future.
###### Contents
* RampMotorExample.py - An example for connecting and controlling thrust to the motors from the Bitcraze VM using ZMQ.

#### /crazy_documentation/*
The project documentation, will eventually include a report discussion both the theory and implementation of the project. 

###### Contents
* **Report.tex** - A report containing a discussion on the dynamics, filtering, motion planning, PD, MPC, L1 control and TODO's.

#### /crazy_trajectory/*
Contains the scripts for motion planning (see crazy_documentation).

###### Contents
* **get_A.m** - Computes the constraint matrix of a polynomial spline.
* **get_Q.m** - Computes the hessian hessian of a polynomial spline.
* **compute_splines.m** - Assembles the global Q and A matrices and minimizes the const function usin quadprog.
* **plot_splines.m** - Plots the position-, velocity-, acceleation- and jerkprofiles of the complete trajectory taking the output of quadprog and the time array as input.
* **splines_2_1D_example.m** - An example which generates the minimum snap trajectory when constraining positional endpoints using 1-3 polynomials of degree 3-5.

#### /simulink/*
Contains the files used in modelling of the quadcopter. Many of the files are works in progress and some cannot be run at all. The necessary paths and parameters are set up by tunning the ``simulink_init.m`` file, after which the examples in the /examples/* directory can be run. Some of these may need additional parameters, in which case an ``init_.m`` file is located in the same directory as the example subdirectory. The MPC_cvx_test and LQR_control_test can be run but are far from complete just yet, everything else should be OK. 

###### Contents - Models
* /quadcopter_model/**quadcopter_init.m** - Constants and initial conditions of thenon-linear model.
* /quadcopter_model/**quadcopter_model.mdl** - A process model of the quadcopter based on the Newton-Lagrange equations.
* /quadcopter_model/**linearization.m** - A script for generating a symbolic expression for the jacobian of the system, the thought was here to derive such an expression which became infeasibly complex (takes ~0.07 s to evaluate with a complete linearization).
* /PD_controller/**init_PD_controller.m** -  Initalises the constants in the inner PD controller.
* /PD_controller/**init_PD_position_controller.m** - Initalises the constants in the outer PD controller.
* /PD_controller/**quadcopter_PD_controller.mdl** - A model for mapping control errors in x, y and z to control signals in phi, theta and z to rotor angular velocities squared.
* /MPC_controller/**init_MPC.m** - Initializes the MPC controller, setting up the linearized discrete system matrices and declaring weights.
* /L1_controller/**adaptationlaw_with_projection_op.slx** - A simple adaptation law with the projection operator validated by comparison to the work of Chengyu Cao and Naira Hovakimyan (see /documentation).
* /kalman_filters/**discrete_kalman_filter.slx** - Implementation of a simple kalman filter for the linearized system (functional)
* /kalman_filters/**extended_discrete_kalman_filter.slx** - Implementation of an extended kalman filter for the quadcopter process (currently dysfuntional).

###### Contents - Examples (/examples/*)
* /quadcopter_model_test/**quadcopter_process_test.slx** - An example of the dynamics responding to rotor speeds, replicating the results in th work of Lukkonens (requires init_process_test.m to run).
* /pd_control_test/**quadcopter_pd_test.slx** - Demonstrates the system response with only the innen PD controller.
* /pd_position_control_test/**quadcopter_pd_position_test.slx** - Demonstrates the system response with an inner PD and outer PD position controller.
* /mpc_control_test/**quadcopter_mpc_position_test.slx** - Demonstrates the system response with an outer inner PD and outer MPC controller (requires init_MPC_test.m to run).
* /kalman_filter_test/**dicrete_kalman_filter_test.slx** - An example of the state esimation using Kalman filtering.
