# The Crazyflie Project
This repository contains the necessary code to run the simulations presented in the thesis. The complete project will be uploaded shortly, but due to interest, the relevant Matlab/Simulink codes are provided.

## Directories

#### /crazy_documentation/*
To be included shortly when the M.Sc. thesis is published, and unofficial report can be provided at request.

#### /crazy_simulink/*
This directory contains enough code to explore the general quad-rotor dynamics in a Simulink/Matlab environment. A special case study of the Crazyflie is included, and the project can be navigated by running `init_project -h`.

All controllers and estimators outlined in the report can be simulated in this directory, and new methods of control can be explored in well tested dynamics. Discrete time implementations of the differential flatness equations are included, as well as RLS/ctRLS parameter estimators, KF/UKF/EKF/GPF/AHRS state-estimators, PID/MRAC/MOPC/LQR/SE(3) controllers. The details of all the mentioned algorithms with references can be found in the /crazy_documentation/*.

#### /crazy_trajectory/*
This directory contains a stripped down version of the standard polynomial motion planning technique used in the real time application (see for instance Mellinger and Landry). Initialise by running the `init.m` file and proceed with any given example.

#### /crazy_ros/*
This code is deprecated and will be committed as soon as it is in a good state.

#### /crazy_firmware/*
This code is deprecated and will be committed as soon as it is in a good state.

#### /crazy_driver/*
This code is deprecated and will be committed as soon as it is in a good state.

## Installation
The real-time implementation can currently be run on a Ubuntu 14.04 (Trusy) with a ROS Indigo installation. In addition, the following stacks need to be installed

* Openni driver - For publishing raw data from the kinect [[1]].
* Ros_numpy - For data conversion related to the disparity images of the kinect [[2]].
* Crazyflie driver - For communicating with the crazyflie [[3]].

[1]: http://wiki.ros.org/openni_kinect
[2]: http://wiki.ros.org/ros_numpy
[3]: http://wiki.ros.org/crazyflie

