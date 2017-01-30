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

class PIDcontroller(Controller):

    def __init__(self):
        """
        This is a 2-dof PID-type controller with setpoint weighing and 
        conditional anti windup, mapping control errors in x and y to 
        references in pitch and roll with a maximum derivative gain of N.
        """
        super(self.__class__, self).__init__('PID')
        self.reset()
        
    def reset(self):
        """
        Loads parameters from the self.param attribute and resets integer and
        derivative parts.
        Args:
        Returns:
        """
        self.Im1, self.Dm1 = np.array([0.0, 0.0]), np.array([0.0, 0.0])
        self.xym1, self.rm1 = np.array([0.0, 0.0]), np.array([0.0, 0.0])
        self.tm1 = None
        
        try:
            self.maxlim = np.array([self.param['x']['maxlim'], self.param['y']['maxlim']])
            self.minlim = np.array([self.param['x']['minlim'], self.param['y']['minlim']])
            self.Kp = np.array([self.param['x']['Kp'], self.param['y']['Kp']])
            self.Ti = np.array([self.param['x']['Ki'], self.param['y']['Ki']])
            self.Td = np.array([self.param['x']['Kd'], self.param['y']['Kd']])
            self.beta = np.array([self.param['x']['beta'], self.param['y']['beta']])
            self.beta = np.array([self.param['x']['gamma'], self.param['y']['gamma']])
        except:
            raise ValueError('Could not load parameters in PID controller, '+
                             'check that the config file exists and is '+
                             'corretly written.')
        
    def compute_control(self):
        
        # Gets the reference trajectory
        start_time = time()                  # Start time of loop
        trajT = start_time - self.traj_start # Offset from the start of the trajectory
        ref = self.compute_reference(trajT)  # Reference vector
        
        # Compute control signal
        xy_ref = np.array(ref[0,0], ref[1,0])
        xy_hat = np.array(ref[0,0], ref[1,0])

        # TODO add correct equations
        P = self.Kp * (self.beta * xy_ref - xy_hat)
        if self.tm1 is not None:
            h = start_time - self.tm1
            I = self.Im1 + h * (xy_ref - xy_hat) /self.Ti
            D = self.Dm1
        else:
            I = np.array([0.0, 0.0])
            D = np.array([0.0, 0.0])
        u = P + I + D
        
        # Saturates control signal
        u, stat = self.sat(u, self.maxlim, self.minlim)
        
        # Conditional anti windup
        sat_ind = np.array([not ii for ii in stat])
        self.Im1[sat_ind] = I[sat_ind]
        self.Dm1 = D
        self.tm1 = start_time

        # Sets control signal
        self.twist.linear.z = self.get_thrust_PD(self.xhat[3,0], self.xhat[6,0])
        self.twist.angular.x = u[0]    # Pitch [rad]. Positive moves "forward"
        self.twist.angular.y = u[1]    # Roll, [rad]. Positive moves "left"
        self.twist.angular.z = 0       # Yaw,  [rad].
        
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
    PID = PIDcontroller()
    PID.run()
    
if __name__ == '__main__':
    main()
