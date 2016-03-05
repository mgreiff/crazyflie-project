## Directories

#### /ROS_project/*
The project currently contains (1) the master node, enabling interation from the terminal (2) the reference generator node, which sents a signal to the \reference_signal topic. The quadcopterModel node (3) listens to this topic and computes the
system response (in terms of the measured states) and sends the data to a \measured_states topic. The computation is done by first evaluating the nonlinear continuous time system matrices, and then computing the ZOH discrete equivalent to condut the time stepping. The purpose of this is to simulate the system with ROS to test different methods of control before connecting to the crazyflie.

Currently, an issue that needs to be met is the nondeterministic behavious of the ROS publishers (see eg. \reference_executiontime), which cannot be used in critical parts of the control system. Instead, tests should be made with ZMQ and the realtime_tools C++ layer.
###### Contents
* /scripts - Contains the code for generating nodes.
* /msg - Contains the message definitions used.
* /config - Contains a JSON configuration file for all ROS nodes.
* /launch - Contains the launch XML file.
* /example - The raw python file for the quadcopter dynamics, example of what a system response in harder realtime will look like.

###### Useful ROS commands
* ``rosrun rqt_graph rqt_graph`` - Plots a graph of the ROS structure.
* ``rosrun rqt_plot rqt_plot <topic>``  - Plots the values of the topic ``\measurements\data[0]`` displays the x-measurements in time.
* ``rosmsg echo <topic>`` - Prints the data sent to the topic in the terminal window.

#### /VM_scripts/*
Contains the scripts that run on the Bitcraze VM. This part of the project is not necessary for running any other script, and will be removed in the future.
###### Contents
* RampMotorExample.py - An example for connecting and controlling thrust
  to the motors from the Bitcraze VM using ZMQ.

#### /documentation/*
The project documentation, will eventually include a report discussion both the theory and implementation of the project. 
###### Contents
* Report.tex - A report containing a short discussion on the dynamics, MPC, L1 control and PD control.

#### /simulink_model/*
Contains the files used in modelling of the quadcopter.

* model/quadcopter_init - Constants and initial conditions of the
  non-linear model (see Luukkonen's work).
* model/quadcopter_model - A process model based, currently operational
  but incomplete  (see Luukkonen's work).

* MPC_control/quadcopter_MPC_init.m
* MPC_control/quadcopter_MPC_simulate.m

#### /trajectory_planning/*
Contains the scripts trajectory planning, possibly suing IRIS and CVXGEN -
currently empty.

