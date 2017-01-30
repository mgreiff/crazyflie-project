#!/usr/bin/env python

import rospy
import sys, os, inspect
import numpy as np
from select import select
import termios
import contextlib
from std_msgs.msg import String, Float64
from crazy_ros.msg import NumpyArrayFloat64
from geometry_msgs.msg import Point
from time import sleep, time
from math import sin, pi, sqrt, cos, sin
from json import dumps, load
import signal
from geometry_msgs.msg import Twist
from ControllerSuper import Controller

class TvLQRcontroller(Controller):

    def __init__(self):
        """
        Launches the super class constructor and sets the necessary attributes
        for reading reference trajectories with one point on the prediction
        horizon.
        """
        key = 'TVLQR'
        super(self.__class__, self).__init__(key)
        
        # Load the gain matrix K, which in the TI-case is constant, computed
        # as the optimal control law for flying at a stable hovering point
        self.K = np.array(self.param['K'])

    def compute_control(self):

        # Start time
        start_time = time() 

        # Gets the reference trajectory
        t = time() - self.traj_start
        ref = self.compute_reference(t)

        # Compute linearization point and control signal based on the current state
        m = self.modParam['m']
        g = self.modParam['g']

        mode = 1
        if mode == 1:
            # TODO write code for interpolating TVLQR
            u = np.array([[m*g],[0.],[0.],[0.]])
        elif mode == 2:
            # TODO write code for iterative TVLQR
            u = np.array([[m*g],[0.],[0.],[0.]])
        
        self.twist.linear.z = u[0,0]              # Thrust [N]
        self.twist.angular.x = u[1,0]             # Pitch  [degrees].
        self.twist.angular.y = u[2,0]             # Roll   [degrees].
        self.twist.angular.z = u[3,0]             # Yaw    [degrees].

        # Publish control signal, reference and execution time
        self.control_pub.publish(self.twist)
        self.reference_pub.publish(ref)
        execution_time = time() - start_time
        self.execution_time_pub.publish(execution_time)
        return execution_time

def signal_handler(signal, frame):
    print 'Shutting down controller node nicely!'
    sys.exit(0)

@contextlib.contextmanager
def raw_mode(file):
    old_attrs = termios.tcgetattr(file.fileno())
    new_attrs = old_attrs[:]
    new_attrs[3] = new_attrs[3] & ~(termios.ECHO | termios.ICANON)
    try:
        termios.tcsetattr(file.fileno(), termios.TCSADRAIN, new_attrs)
        yield
    finally:
        termios.tcsetattr(file.fileno(), termios.TCSADRAIN, old_attrs)

def main():
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node('Outer Controller')
    TVLQR = TvLQRcontroller()
    TVLQR.run()
    
if __name__ == '__main__':
    main()
