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

class TiMPCcontroller(Controller):
    """
    This controller object represents the time invariant MPC, with the
    quadcopter dynamics linearized around a stable hovering point at all times.
    The controller inherits the Controller object, which contains all basic
    methods and callbacks.
    """
    def __init__(self):
        """
        Runs super class constructor with a unique key and defines the
        prediction horizon in the trajectory object.
        """
        key = 'TIMPC'
        super(self.__class__, self).__init__(key)
        self.trajectory.predN = self.param['predN']
        
    def compute_control(self):

        # Gets the reference trajectory
        t = time() - self.traj_start
        Rmat = self.compute_reference(t)

        # TODO Compute control signal
        start_time = time() # Start time
        self.twist.linear.z = 4000  # Thrust
        self.twist.angular.x = 0    # Pitch [degrees]. Positive moves "forward"
        self.twist.angular.y = 0    # Roll, [degrees]. Positive moves "left"
        self.twist.angular.z = 0    # Yaw,  [degrees].

        # Publish control signal, reference and execution time
        self.control_pub.publish(self.twist)
        self.reference_pub.publish(np.transpose(Rmat[:,0]))
        execution_time = time() - start_time
        self.execution_time_pub.publish(execution_time)
        return execution_time

def signal_handler(signal, frame):
    print 'Shutting down controller node nicely!'
    sys.exit(0)

def main():
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node('Outer Controller')
    TiMPC = TiMPCcontroller()
    while True:
        if TiMPC.status:
            execution_time = TiMPC.compute_control()
            h = TiMPC.timestep
            wait_time = h - execution_time
        else:
            wait_time = 0.05
            h = 0.05
        if wait_time >= 0:
            sleep(wait_time)
        else:
            print (("Warning, the execution time of %f exceeds the loop time %f"+
                    "in the outer controller") % (execution_time, h))
    
if __name__ == '__main__':
    main()
