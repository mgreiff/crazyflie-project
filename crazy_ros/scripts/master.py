#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import os
import sys
import signal
from json import dumps, load
import time

class Master(object):

    def __init__(self):
        # Sets up publishers and subscribers for the reference generator
        self.Kinect_status_pub = rospy.Publisher('/system/kinect', String, queue_size = 10)
        self.status_master = rospy.Subscriber('/system/status_master', String, self.handle_system_status)
        self.status_controller = rospy.Publisher('/system/status_controller', String, queue_size = 10)

        # Attributes
        self.status = False # Waits until the kinect node is done calibrating

        print ('\nThe master node is used to communicate with the system.\n'+
              'Type -h for help or ^C and ^D to exit once the calibration is complete.')

    def handle_system_status(self, msg):
        # Enables the system to continue operation
        if msg.data == 'True':
            self.status = True

    def help(self):
        print ('USAGE: NODE KEYWORD ARG\n'+
               '    NODE - Can be set to:\n\n'
               '        1) "q" - Immediately kills all control signals\n\n'+
               '        2) "Controller" - Defines how the control signal is computed, can be set\n'+
               '            with the following keywords:\n'+
               '            * "RefGen" - Starts the reference generator node and shuts pauses\n'+
               '                the MPC and PID nodes, generates an omega sequence similar to\n'+
               '                that in the work of Lukkonen.\n'+
               '            * "MPC" - Starts the MPC node and shuts pauses the reference\n'+
               '                generator and PID nodes.\n'+
               '            * "PID" - Starts the PID node and shuts pauses the reference\n'+
               '                generator and MPC nodes.\n\n'+
               '        3) "Kinect" - Communicate with the Kinect-link node, here the keyword\n'+
               '            can be set to "Calibrate" in order to recalibrate the camera\n')

def error(argument, value):
    print 'ERROR. Invalid use of argument %s, cannot be set to "%s".' % (argument, value)

def signal_handler(signal, frame):
    print 'Shutting down Master node nicely!'
    sys.exit(0)

def main():
    rospy.init_node('master')
    master = Master()
    signal.signal(signal.SIGINT, signal_handler)

    while True:
        if master.status:
            try:
                command = raw_input('(@ master) Command: ')
                data = command.split(' ')

                if data[0] == 'q':
                    # Pauses all controllers
                    master.status_controller.publish(data[0])

                elif data[0] == '-h':
                    # Displays help
                    master.help()

                elif data[0] == 'Kinect' and len(data) == 2:
                    # Sends command to the Kinect node and forces the master
                    # node to pause while calibrating (as
                    # the kinect node then requires the terminal prompt).
                    if data[1] == 'Calibrate':
                        master.Kinect_status_pub.publish(data[1])
                        master.status = False
                    else:
                        error('KEYWORD', data[1])

                elif data[0] == 'Controller' and len(data) == 2:
                    # Checks that the user has supplied valid inputs and sends
                    # commands to the /controller/* generator.
                    if data[1] in ['MPC','PID','RefGen']:
                        master.status_controller.publish(data[1])
                    else:
                        error('KEYWORD', data[1])
                else:
                    error('NODE', data[0])

            except EOFError:
                print 'EOFError ecoundered in master!'
                return

if __name__ == '__main__':
    main()
