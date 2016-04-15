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
import signal

class ReferenceGenerator(object):

    def __init__(self):
        # Sets up subscribers
        self.status_sub = rospy.Subscriber('/system/status_controller', String, self.handle_status_data)
        self.control_pub = rospy.Publisher('/system/control_signal', NumpyArrayFloat64, queue_size = 10)
        self.et_pub = rospy.Publisher('reference_executiontime', Float64, queue_size = 10)
 
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

    def handle_status_data(self, msg):
        # Callback for commands from master which generates a trajectory
        # if RefGen is broadcasted, but turns of the controller if anything
        # else is broadcasted and sets all conttrol signals to 0 if "q"
        # is broadcasted
        if msg.data == 'RefGen':
            self.status = True
        else:
            self.status = False
            if msg.data == 'q':
                self.control_pub.publish(np.zeros((4,1)))

    def __str__(self):
        return 'reference generator node'

def signal_handler(signal, frame):
    print 'Shutting down PID node nicely!'
    sys.exit(0)

def main():
    rospy.init_node('ReferenceGenerator')
    refGen = ReferenceGenerator()
    signal.signal(signal.SIGINT, signal_handler)

    g = refGen.param['quadcopter_model']['g']
    m = refGen.param['quadcopter_model']['m']
    k = refGen.param['quadcopter_model']['k']

    runtime = time.time()
    while True:
        if not refGen.status:
            refGen.totalTime = 0
        else:
            # Tries to compensate for execution time
            startTime = time.time()

            # Publish omega reference for testing the quadcompter dynamics 
            refGen.totalTime += refGen.timestep
            t = refGen.totalTime
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
                refGen.status = False
                refGen.totalTime = 0
            ref = hover_omega*np.ones([4,1]) + omegaAmp*omegaOscillation

            executionTime = time.time() - startTime
            time.sleep(refGen.timestep - executionTime)

            refGen.et_pub.publish(time.time()-runtime)
            runtime = time.time()

            # Publish reference
            refGen.control_pub.publish(ref)

if __name__ == '__main__':
    main()
