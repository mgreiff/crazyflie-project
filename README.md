~~~~ Directories ~~~~

/ROS_project/*
Contains the ROS code - currently empty as no progress has been made
from the work of Wolfgang, which can be cloned from github.

/VM_scripts/*
Contains all scripts run on the Bitcraze VM.

    * RampMotorExample.py - An example for connecting and controlling thrust
        to the motors from the Bitcraze VM.

/figures/*
Contains the figures used in the project reports.

/simulink_model/*
Contains the files used in modelling of the quadcopter.

    * model/quadcopter_init - Constants and initial conditions of the
        non-linear model (see Luukkonen's work).
    * model/quadcopter_model - A process model based, currently operational
        but incomplete  (see Luukkonen's work).

    * MPC_control/quadcopter_MPC_init.m
    * MPC_control/quadcopter_MPC_simulate.m

/trajectory_planning/*
Contains the scripts trajectory planning, possibly suing IRIS and CVXGEN -
currently empty.

~~~~ Documents ~~~~

Notes.tex - Brief notes on the project with references to works and software
might be worth while looking into.

Report.tex - A more comprehensive document which I started writing while
reading about the L1 control. Contains the TODOs of the project.
