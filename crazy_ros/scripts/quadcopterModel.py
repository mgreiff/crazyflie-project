#!/usr/bin/env python

from std_msgs.msg import String, Float64
import rospy
import os
import sys
from json import dumps, load
import time

class QuadcopterModel(object):

    def __init__(self):
        # Sets up subscribers
        self.reference_sub = rospy.Subscriber('reference_signal', Float64, self.handle_reference_data)

        # Sets up publishers
        self.measured_states_pub = rospy.Publisher('measured_states', Float64, queue_size = 10)

        params = None
        # Loads configuration parameters
        for filename in rospy.myargv(argv=sys.argv):
            if os.path.basename(filename) == 'configparam.cnf':
                with open(filename) as configfile:
                    params = load(configfile)
                configfile.close()
        if params != None:
            # Sets configuration parameters
            self.timestep = params['global']['timestep']
        else:
            print 'ERROR. Could not load configuration parameters in %s' % (str(self))

    def handle_reference_data(self, msg):
        # TODO include discretised quadcopter dynamics and publish measured states
        self.measured_states_pub.publish(msg.data)

    def __str__(self):
        return 'QuadcopterModel Node'

def main():
    rospy.init_node('QuadcopterModel')
    quad = QuadcopterModel()
    rospy.spin()

if __name__ == '__main__':
    main()
