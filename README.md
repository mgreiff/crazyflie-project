## Directories

#### /crazy_ros/*
The ROS-project currently contains (1) the /master node which can be interacted with from the terminal, (2) the /kinect node node which calibrates reads data from a kinect using the openni driver [[1]], (3) various controller nodes (MPC/PID/RefGen) and (4) the /quadcopterModel node which computes the system response (in terms of the measured states) and for simulation purposes. The computation is done by first evaluating the nonlinear continuous time system matrices, and then computing the ZOH discrete equivalent to condut the time stepping. The purpose of this is to simulate the system with ROS to test different methods of control before connecting to the crazyflie. For simplicity, the control signals are published to the same topics that are used to communicate with the crazyflie using Wolfgang's crazyros driver [[2]].

![alt tag](https://github.com/mgreiff/crazyflie_project/blob/master/crazy_documentation/figures/ROSstruct.png)

Currently, an issue that needs to be met is the nondeterministic behavious of the ROS publishers (see eg. \reference_executiontime), which cannot be used in critical parts of the control system. Instead, tests should be made OROCOS or the realtime_tools C++ layer.

###### Contents
* /scripts/* - Contains the code for generating nodes.
* /msg/* - Contains the message definitions used.
* /config/* - Contains a JSON configuration file for all ROS nodes.
* /launch/* - Contains the launch XML files.
* /modules/* - Contains the crazylib module in which various filters and c-wrappers are defined. See /modules/examples/* for examples on how to use the functions. When running the examples, make sure to include the path to the dynamic libraries (.so files) in the environment variable ``LD_LIBRARY_PATH``. This can for instance be done by executing ``export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:<path>``, where the the path is your path to the /modules directory.

###### Useful commands
* ``roslaunch crazy_ros crazy.launch useDumy:=1`` - Launches the entire project in crazy_ros (package) using the crazy.launch XML file. If no additional arguments are given, then by default, a dummy node is launched instead of openni to test and debug calibration. This can be disabled by setting ``usedDummy:=0'' when launching the project.
* ``rosrun rqt_graph rqt_graph`` - Plots a graph of the ROS structure.
* ``rosrun rqt_plot rqt_plot <topic>``  - Plots the values of the topic ``\system\controlsignal\data[0]`` displays the pitch reference in time.
* ``cc -fPIC -shared -o foo.so foo.c`` - creates a dynamic shared library ``foo.so`` from the foo c-file.

[1]: http://wiki.ros.org/openni_kinect
[2]: http://wiki.ros.org/crazyflie

#### /crazy_documentation/*
The project documentation, will eventually include a report discussion both the theory and implementation of the project. 

###### Contents
* **Report.tex** - A report in progress containing a discussion on the dynamics, filtering, motion planning, PD, MPC, L1 control and TODO's.

#### /crazy_python/*
Contains python code which has since been mergen with the ROS structure, serves little purpose and will be removed shortly.

#### /crazy_trajectory/*
Contains the scripts for motion planning (see crazy_documentation).

###### Contents
* trajectory_functions/**get_A.m** - Computes the constraint matrix of a polynomial spline.
* trajectory_functions/**get_Q.m** - Computes the hessian hessian of a polynomial spline squared to form the objective function.
* trajectory_functions/**compute_splines.m** - Assembles the global Q and A matrices and minimizes the const function usin quadprog.
* trajectory_functions/**plot_splines.m** - Plots the position-, velocity-, acceleation- and jerkprofiles of the complete
* trajectory_functions/**reftraj_compute_example.m** Generates coefficients for 3D-trajectories and stores data in a .mat file. 
* trajectory_functions/**trajectory_sfunc.m** - S fucntion used to evaluate 3D-trajectories in simulink.
* trajectory taking the output of quadprog and the time array as input.
* Various examples showing how the trajectory can be computed and evaluated in a .m format when constraining positional endpoints using 1-3 polynomials of degree 3-5 with minimum snap. Also contains examples on how it can be implemented in simulink, using either model callbacks or S-fucntions (due to the fact that structs cannot be imported directly), and how the evaluation can be set up to copute points on the predition horizon in MPC based soley on polynomial coefficients.

#### /crazy_simulink/*
Contains the files used in modelling of the quadcopter. Many of the files are works in progress and some cannot be run at all. The necessary paths and parameters are set up by tunning the ``simulink_init.m`` file, after which the examples in the /examples/* directory can be run. Some of these may need additional parameters, in which case an ``init_.m`` file is located in the same directory as the example subdirectory.

###### Contents - Models
* /quadcopter_model/**quadcopter_init.m** - Constants and initial conditions of the non-linear model and subdirectories.
* /quadcopter_model/**quadcopter_model.mdl** - A process model of the quadcopter based on the Newton-Lagrange equations.

* /PD_controller/**quadcopter_PD_controller.mdl** - A model for mapping control errors in x, y and z to control signals in phi, theta and z to rotor angular velocities.
* /PD_controller/**quadcopter_PD_controller_with_thrust.mdl** - A model for mapping control errors in x, y and z to control signals in phi, theta and z to deviation in thrust and torque vectors in the global frame.

* /L1_controller/**adaptationlaw_with_projection_op.slx** - A simple adaptation law with the projection operator validated by comparison to the work of Chengyu Cao and Naira Hovakimyan (see /documentation).

* /kalman_filters/**discrete_linear_kalman.slx** - Implementation of a simple kalman filter for the linearized system (functional).
* /kalman_filters/**discrete_extended_kalman.slx** - Implementation of an extended kalman filter (functional).
* /kalman_filters/**discrete_particle_filter.slx** - Implementation of an unscented kalman filter (dysfunctional).
* /kalman_filters/**discrete_particle_filter.slx** - Implementation of a generic particle filter (dysfunctional).
* /kalman_filters/**_sfunc.m** - The S-fucntion algorithms implemented in the filter models.

###### Contents - Examples (/examples/*)
**Dynamics**
* /dynamics_test/**quadcopter_process_test.slx** - An example of the dynamics responding to rotor speeds, replicating the results in the work of Lukkonens.

**Stabilising inner controllers**
* /inner_loop_PD_test/**quadcopter_pd_test.slx** - Demonstrates the system response with only the inner PD controller.
* /inner_loop_LQR_test/**quadcopter_LQR_2_test.slx** - Demonstrates the system response with only the inner LQR controller.

**Outer positional controllers**
* /outer_loop_pd_test/**quadcopter_pd_position_test.slx** - Demonstrates the system response with an inner PD and outer PD position controller.
* /outer_loop_mpc_test/**quadcopter_mpc_position_test.slx** - Demonstrates the system response with an outer inner PD and outer MPC controller.
* /outer_loop_mpc_cvx_test/**mpc_cvxgen.slx** - Simulates the same problem as in the /mpc_control_test/ directory, but uses the CVXgen mex solver (currently dysfunctional) 
* /outer_loop_mpc_cvx_test/**example_mpc_trajectory.slx** - Example showing exactly how the trajectory is evaluated in the model.
* /outer_loop_mpc_cvx_test/**example_mpc_cycle.m** - Example showing how the optimization works and what it achieves.

**State estimation**
* /double_integrator_filter_test/**integrator_filter_test.slx** - An example of the state esimation of a double integrator using KF, UKF and GPF simultaneously.
* /inner_loop_filter_test/**filters_example.slx** - An example of the state esimation using varoius filters that can be switched between by clicking the state estimation model block and selecting any .slx file in the /kalman_filter/* directory. This uses the full nonlinear dynamics of the quadcopter, and an example estimation is shown below.

<img src="https://github.com/mgreiff/crazyflie_project/blob/master/crazy_documentation/figures/KalmanFilterComparison.png">

