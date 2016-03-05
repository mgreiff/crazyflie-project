#!/usr/bin/env python

from std_msgs.msg import String, Float64
from crazy_ros.msg import NumpyArrayFloat64 # Can handle messages of type np.array, list and tuple
import numpy as np
from math import sin, pi, sqrt
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
        self.reference_pub = rospy.Publisher('reference_signal', NumpyArrayFloat64, queue_size = 10)
        self.et_pub = rospy.Publisher('reference_executiontime', Float64, queue_size = 10)
 
        # Initial attributes
        self.status = False
        self.mode = 0
        self.totalTime = 0

        params = None
        # Loads configuration parameters
        for filename in rospy.myargv(argv=sys.argv):
            if os.path.basename(filename) == 'configparam.cnf':
                with open(filename) as configfile:
                    param = load(configfile)
                configfile.close()
        if param != None:
            # Sets configuration parameters
            self.timestep = param['global']['timestep']
            self.param = param
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
        runtime = time.time()
        while self.status:

            # Tries to compensate for execution time
            startTime = time.time()

            # Publish omega reference for testing the quadcompter dynamics 
            if self.mode == 0:
                self.totalTime += self.timestep
                t = self.totalTime
                g = self.param['quadcopter_model']['g']
                m = self.param['quadcopter_model']['m']
                k = self.param['quadcopter_model']['k']
                hover_omega = sqrt(g * m / (4 * k));
                omegaAmp = 25.
                omegaOscillation = np.ones([4,1])
                if t<=0.5:
                    omegaOscillation *= 2 * sin(t*4*pi)
                elif 0.5<t and t<=1:
                    omegaOscillation[0] = 0
                    omegaOscillation[1] *= -sin(t*4*pi)
                    omegaOscillation[2] = 0
                    omegaOscillation[3] *= sin(t*4*pi)
                elif 1<t and t<=1.5:
                    omegaOscillation[0] *= -sin(t*4*pi)
                    omegaOscillation[1] = 0
                    omegaOscillation[2] *= sin(t*4*pi)
                    omegaOscillation[3] = 0
                elif 1.5<t and t<=2.:
                    omegaOscillation[0] *= -sin(t*4*pi)
                    omegaOscillation[1] *= sin(t*4*pi)
                    omegaOscillation[2] *= -sin(t*4*pi)
                    omegaOscillation[3] *= sin(t*4*pi)
                else:
                    self.status = False
                    self.totalTime = 0
                ref = hover_omega*np.ones([4,1]) + omegaAmp*omegaOscillation

            # Dummy publisher
            elif self.mode == 1:
                ref = [1,1,1,1,1]

            # Dummy publisher
            elif self.mode == 2:
                ref = [2,2,2,2,2]

            executionTime = time.time() - startTime
            time.sleep(self.timestep - executionTime)

            self.et_pub.publish(time.time()-runtime)
            runtime = time.time()

            # Publish reference
            self.reference_pub.publish(ref)

    def __str__(self):
        return 'reference generator node'

def main():
    rospy.init_node('ReferenceGenerator')
    rG = ReferenceGenerator()
    rospy.spin()

if __name__ == '__main__':
    main()
