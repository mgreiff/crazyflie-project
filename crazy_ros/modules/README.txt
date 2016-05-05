This entire directory defines the crazylib module and sets up the necessary
c-wrappers for running a simple sum example (using the c-code in /modules/sum/sum.c
and implementing the cvxgen solver (located in /modules/cvxgen/*). Note that all the
custom scripts in '/cvxgen/* are prefixed with ROS, everything else is as it was
generated from the makefile of CVXGEN. The c-code is compiled as dynamical libraries
(.so) and included in the crazylib module using cython.

Currently, the module contains:
1)  An example wrapper for computing the sum of integers in a list, showing how the
    c-fucntion is accessed through the module.
2)  A function which calls a customized cvxgen solver using it's c-interface,
    wrapped by cython just like sum.c
3)  The regular kalman update
4)  The asynchronous kalman update
5)  The quadcopter dynamics defined as a class object

All the functions of the crazylib module can be tested by running the scripts in
/modules/examples/*, and the corresponding examples are:
1)  mpc_cython_sum.py
2)  mpc_cython_cvx.py
3)  kinect_KF_1D_triple_integrator.py
3)  kinect_KF_3D_double_integrator.py
4)  kinect_AKF_1D_triple_integrator.py
5)  quadcopter_dynamics.py

In order to run the examples and use the crazylib module, the LD_LIBRARY_PATH must
be extended to the directories in which the dynamical libraries are kept. Thi can
be done by sourcing the shell script init.sh in crazy_ros.
