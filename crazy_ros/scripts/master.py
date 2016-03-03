#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import os
import sys
from json import dumps, load
import time

class Master(object):

    def __init__(self):
        # Sets up publishers and subscribers
        self.RG_status_pub = rospy.Publisher('RG_status', String, queue_size = 10)
        self.RG_mode_pub = rospy.Publisher('RG_mode', String, queue_size = 10)

        print 'This node can be used to communicate with the reference generator.\n'
        print 'USAGE: NODE KEYWORD ARG\n'
        print '    NODE - Can be set to:'
        print ('        i) RG - Communicate with the reference generator, here the keyword can'+
               '            be set to STATUS (with arguments IDLE or ACTIVE) or MODE (with'+
               '            an integer from 0 to 2).\n')

    def handle_status(self, msg):
        self.status = True

def error(string):
    print 'ERROR. Invalid use of argument %s' % (string)

def main():
    rospy.init_node('master')
    master = Master()
    while True:
        try:
            command = raw_input('Command: ')
            data = command.split(' ')
            if data[0] == 'RG' and len(data) == 3:
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
