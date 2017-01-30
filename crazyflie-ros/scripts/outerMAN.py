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
        print 'here'
        if msg.data == self.key:
            self.useManReference = True
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
    MAN = MANcontroller()
    MAN.run()
    
if __name__ == '__main__':
    main()
