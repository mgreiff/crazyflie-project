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

class MANcontroller(Controller):

    def __init__(self):

        super(self.__class__, self).__init__('MAN')
        self.td = self.param['thrust_d']
        self.pd = self.param['pitch_d']
        self.rd = self.param['roll_d']
        self.yd = self.param['yaw_d']
    
    def handle_status(self, msg):
        if msg.data == self.key:
            print ('\nType -h for help or ^C to exit the entire program.\n')
            cont  = True
            while cont:
                command = raw_input('(@ %s) Command: ' % str(self))
                if command == '-h':
                    print ('USAGE: KEYWORD ARG\n    1) "q" - Immediately kills all'+
                           ' control, setting references to zero\n    2) "b" - '+
                           'Returns to the master\n    4) "s"  - Starts the'+
                           ' manual controller, using the key combinations\n'+
                           '        (r,f)-(a,d)-(w,s)-(z,c) for thrust, pitch'+
                           ' roll and and yaw, where\n        (increase, decrease).')
                elif command == 's':
                    cont = False
                    self.status = True
                elif command == 'b':
                    self.status = False
                    cont = False
                    self.master_status_pub.publish('True')
                elif len(command) > 0:
                    print '(@ %s) WARNING. Unsupported keyword %s, type -h for help.' % (str(self), command)
                else:
                    print '(@ %s) WARNING. No command received' % str(self)

    def compute_control(self):
        try:
            with raw_mode(sys.stdin):
                try:
                    rlist, _, _ = select([sys.stdin], [], [], 1/35)
                    if rlist:
                        ch = sys.stdin.read(1)
                        if ch == 'r':
                            print '\rup (thrust)'
                            self.twist.linear.z += self.td
                        elif ch == 'f':
                            print '\rdown (thrust)'
                            self.twist.linear.z -= self.td
                        elif ch == 'a':
                            self.twist.angular.x -= self.rd
                            print '\rleft (roll)'
                        elif ch == 'd':
                            self.twist.angular.x += self.rd
                            print '\rright (roll)'
                        elif ch == 'w':
                            print '\rforward (pitch)'
                            self.twist.angular.y += self.pd
                        elif ch == 's':
                            print '\rbackward (pitch)'
                            self.twist.angular.y -= self.pd
                        elif ch == 'z':
                            print '\rrotate left (yaw)'
                            self.twist.angular.z -= self.yd
                        elif ch == 'c':
                            print '\ryawrate right (yaw)'
                            self.twist.angular.z += self.yd
                        elif ch == 'q':
                            print '\r\nSetting references to zero and exiting back to MAN UI'
                            self.twist.linear.z = 0
                            self.twist.angular.x = 0
                            self.twist.angular.y = 0
                            self.twist.angular.z = 0
                            self.status = False                   # Pause controller
                            self.handle_status(String(self.key))  # Return to UI
                        else:
                            print '\rThe command "%s" is not supported' % ch
                except (KeyboardInterrupt, EOFError):
                    pass
        except:
            pass
        self.control_pub.publish(self.twist)
        execution_time = 0
        self.execution_time_pub.publish(execution_time)
        return execution_time
    
class LQRcontroller(Controller):

    def __init__(self):
        """
        Launches the super class constructor and sets the necessary attributes
        for reading reference trajectories with one point on the prediction
        horizon.
        """
        super(self.__class__, self).__init__('LQR')

    def compute_control(self):
        #print 'LQR control signal'
        start_time = time() # Start time
        

        # Gets the reference trajectory
        t = time() - self.traj_start
        ref = self.compute_reference(t)

        # Compute control signal
        self.twist.linear.z = 5000  # Thrust
        self.twist.angular.x = 0    # Pitch [degrees]. Positive moves "forward"
        self.twist.angular.y = 0    # Roll, [degrees]. Positive moves "left"
        self.twist.angular.z = 0    # Yaw,  [degrees].

        # Publish control signal, reference and execution time
        self.control_pub.publish(self.twist)
        self.reference_pub.publish(ref)
        execution_time = time() - start_time
        self.execution_time_pub.publish(execution_time)
        return execution_time

class MPCcontroller(Controller):

    def __init__(self):
        super(self.__class__, self).__init__('MPC')
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
    rospy.init_node('Outer Controller')
    
    signal.signal(signal.SIGINT, signal_handler)
    
    # The controllers map state vectors 
    MAN = MANcontroller()
    LQR = LQRcontroller()
    MPC = MPCcontroller()
    PID = PIDcontroller()

    while True:
        # Computes and publishes the control signal
        if MAN.status:
            execution_time = MAN.compute_control()
            wait_time = h - execution_time
            h = MAN.timestep
        elif LQR.status:
            execution_time = LQR.compute_control()
            h = LQR.timestep
            wait_time = h - execution_time
        elif MPC.status:
            execution_time = MPC.compute_control()
            h = MPC.timestep
            wait_time = h - execution_time
        elif PID.status:
            execution_time = PID.compute_control()
            h = PID.timestep
            wait_time = h - execution_time
        else:
            wait_time = 0.05
            h = 0.05
        if wait_time >= 0:
            sleep(wait_time)
        else:
            print "Warning, the execution time of %f exceeds the loop time %f in the outer controller" % (execution_time, h)

if __name__ == '__main__':
    main()
