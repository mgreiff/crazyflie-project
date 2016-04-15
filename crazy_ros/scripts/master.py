#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import os
import sys
from json import dumps, load
import time

class Master(object):

    def __init__(self):
        # Sets up publishers and subscribers for the reference generator
        self.RG_status_pub = rospy.Publisher('RG_status', String, queue_size = 10)
        self.RG_mode_pub = rospy.Publisher('RG_mode', String, queue_size = 10)

        # sets up publisher for kinect camera
        self.Kinect_status_pub = rospy.Publisher('/system/kinect', String, queue_size = 10)

        # System status subscriber
        self.status_sub = rospy.Subscriber('/system/status', String, self.handle_system_status)

        # Attributes
        self.status = False # Waits until the kinect node is done calibrating

        print ('\nThe master node is used to communicate with the system.\n'+
              'Type -h for help or ^C and ^D to exit once the calibration is complete.')

    def handle_system_status(self, msg):
        # Enables the system to continue operation
        if msg.data == 'True':
            self.status = True

    def help(self):
        print 'USAGE: NODE KEYWORD ARG\n'
        print '    NODE - Can be set to:'
        print ('        i) RG - Communicate with the reference generator, here the keyword can\n'+
               '            be set to "STATUS" (with arguments "IDLE" or "ACTIVE") or "MODE" (with\n'+
               '            an integer from 0 to 2).\n')
        print ('        i) Kinect - Communicate with the Kinect-link node, here the keyword can\n'+
               '            be set to "Calibrate" in order to recalibrate the camera\n')

def error(string):
    print 'ERROR. Invalid use of argument %s' % (string)

def main():
    rospy.init_node('master')
    master = Master()
    while True:
        if master.status:
            try:
                command = raw_input('(@ master) Command: ')
                data = command.split(' ')
                if data[0] == '-h':
                    # Displays help
                    master.help()

                elif data[0] == 'Kinect' and len(data) == 2:
                    # Sends command to the Kinect node and forces the master
                    # node to pause if the depth is to be calibrated (as
                    # the kinect node then requires the terminal prompt).
                    if data[1] == 'Calibrate':
                        master.Kinect_status_pub.publish(data[1])
                        master.status = False
                    else:
                        error('KEYWORD')

                elif data[0] == 'RG' and len(data) == 3:
                    # Checks that the user has supplied valid inputs and sends
                    # commands to the reference generator.
                    if data[1] == 'STATUS':
                        if data[2] == 'IDLE' or data[2] == 'ACTIVE':
                            master.RG_status_pub.publish(data[2])
                        else:
                            error('ARG')
                    elif data[1] == 'MODE':
                        try:
                            mode = int(data[2])
                            if mode >= 0 and mode <= 2:
                                master.RG_mode_pub.publish(data[2])
                            else:
                                error('ARG')
                        except:
                            error('ARG')
                    else:
                        error('KEYWORD')
                else:
                    error('NODE')

            except EOFError:
                print 'Shutting down Master...'
                return

if __name__ == '__main__':
    main()
