#!/usr/bin/env python

import rospy
import ros_numpy
import math
import numpy as np
import signal
import sys
from time import sleep
from crazy_ros.msg import NumpyArrayFloat64

class DummyCamera(object):
    def __init__(self):
        # Publishes data to the /kinect/dummyimage topic at 20 Hz as a np.array
        self.pub = rospy.Publisher('/kinect/dummy', NumpyArrayFloat64, queue_size = 10)
def signal_handler(signal, frame):
    print 'Shutting down Master node nicely!'
    sys.exit(0)

def main():
    rospy.init_node('dummyCamera')
    dC = DummyCamera()
    signal.signal(signal.SIGINT, signal_handler)

    while True:
        sleep(0.05)
        data = np.random.rand(480*640) + 10
        dC.pub.publish(data)

if __name__ == '__main__':
    main()
