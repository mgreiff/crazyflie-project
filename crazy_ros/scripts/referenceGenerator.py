#!/usr/bin/env python

from std_msgs.msg import String, Float64
import rospy
import os
import sys
from json import dumps, load
import time

class ReferenceGenerator(object):

    def __init__(self):
        # Sets up subscribers
        self.mode_sub = rospy.Subscriber('RG_mode', String, self.handle_mode_data)
        self.status_sub = rospy.Subscriber('RG_status', String, self.handle_status_data)

        # Sets up publishers
        self.reference_pub = rospy.Publisher('reference_signal', Float64, queue_size = 10)
 
        # Initial attributes
        self.status = False
        self.mode = 0

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

    def handle_mode_data(self, msg):
        self.mode = int(msg.data.split(' ')[0])
        
    def handle_status_data(self, msg):
        data = msg.data.split(' ')[0]
        if data == 'IDLE':
            self.status = False
        elif data == 'ACTIVE':
            self.status = True
            self.generate_reference()
        else:
            print 'ERROR. The cannot set the status in the reference generator to %s' % (data)

    def generate_reference(self):
        while self.status:
            if self.mode == 0:
                self.reference_pub.publish(self.timestep)
            elif self.mode == 1:
                self.reference_pub.publish(self.timestep)
            elif self.mode == 2:
                self.reference_pub.publish(self.timestep)
            time.sleep(0.1)

    def __str__(self):
        return 'reference generator node'

def main():
    rospy.init_node('ReferenceGenerator')
    rG = ReferenceGenerator()
    rospy.spin()

if __name__ == '__main__':
    main()
