# The Crazyflie Project
This repository contains the necessary code to run the simulations and experiments presented in the thesis. This includes relevant Matlab/Simulink models as well as modifications to the existing firmware and drivers. The system is best run with a MOCCAP like VICON or Optitrack, but can also be run using Bitcrazes UWB LPS system as demonstrated in this [video](https://www.youtube.com/watch?v=lYQgRQeGyPA).

## Dependencies
The real-time implementation can currently be run on a Ubuntu 14.04 (Trusy) with a ROS Indigo installation. In addition, the following stacks need to be installed

* Openni driver - For publishing raw data from the kinect (if applicable) [[1]].
* Ros_numpy - For data conversion related to the disparity images of the kinect (if applicable) [[2]].
* Crazyflie driver - For communicating with the crazyflie, modified from the original by Wolfgang [[3]].
* Crazyflie firmware - Modified from the original to support SE(3) control [[4]].

## Getting started
First of all, this project requires a machine with a Ubuntu 14.04 installation, which preferrably should be installed natively. Simply follow the [tutorials](http://wiki.ros.org/indigo/Installation/Ubuntu) on installing ROS Indigo and setting up a catkin environment. Once done, clone the [openni_driver](http://wiki.ros.org/openni_kinect) and [ROS_numpy](http://wiki.ros.org/ros_numpy) stacks if using the kinect camera. If so, basic image processing and kalman filtering can be accomplished by cloning the [kinect_vision](https://github.com/mgreiff/kinect_vision) project. For all the ROS stacks, simply place them in the ``./catkin_ws/src/`` directory. Once complete, clone the modified [ROS driver](https://github.com/mgreiff/crazyflie_ros) which has been re-written to support the loading of trajectories.

At this point, you should have at the very least a Ubuntu 14.04 installation with the crazyflie_ros driver. We then need to switch to the full_control branch of the driver, simply done through

    cd ~/catkin_ws/src/crazyflie_ros/
    git checkout full_control

Next, clone the [crazyflie firmware](https://github.com/mgreiff/crazyflie-firmware) from and follow the instructions on how to set up and configure the system in a linux environment. Note especially that in order to use the USB radio without root priveliges, new [udev rules](https://github.com/bitcraze/crazyflie-lib-python#setting-udev-permissions) need to be written. Oncce this has been done, checkout the se3_control branch, make and flash the firmware using the following commands

    cd ~/.../crazyflie-firmware
    it submodule init
    git submodule update
    git ckeckout se3_control
    make clean && make all -j4
    make cload

Now that all the dependencies have been installed and the firmware image has been flashed, simply install the [crazyflie_trajectory] stack. By sourcing the environment and running the launch file

    cd ~/catkin_ws
    source devel/setup.bash
    roslaunch crazyflie_trajecotry example.launch

the ros program is run. Refer to the documentation in ``/crazy_ros/docs/`` for documentation on the python code and ros program, and after running the program, use ``-h``for help regarding available options and usage.

## Directories

#### /crazyflie-documentation/*
To be included shortly when the M.Sc. thesis is published, and unofficial report can be provided at request.

#### /crazyflie-simulink/*
This directory contains enough code to explore the general quad-rotor dynamics in a Simulink/Matlab environment. A special case study of the Crazyflie is included, and the project can be navigated by running `init_project -h`.

All controllers and estimators outlined in the report can be simulated in this directory, and new methods of control can be explored in well tested dynamics. Discrete time implementations of the differential flatness equations are included, as well as RLS/ctRLS parameter estimators, KF/UKF/EKF/GPF/AHRS state-estimators, PID/MRAC/MOPC/LQR/SE(3) controllers. The details of all the mentioned algorithms with references can be found in the /crazy_documentation/*.

#### /crazyflie-ros/*
This code is deprecated and will be committed as soon as it is in a good state.

[1]: http://wiki.ros.org/openni_kinect
[2]: http://wiki.ros.org/ros_numpy
[3]: https://github.com/mgreiff/crazyflie_ros
[4]: https://github.com/mgreiff/crazyflie-firmware
