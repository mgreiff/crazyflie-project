#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import os, sys, inspect
import signal
from json import dumps, load
import time
from geometry_msgs.msg import Twist
from crazyflie_trajectory.srv import UpdateParams
from crazyflie_trajectory.msg import GenericLogData
from crazyflie_trajectory.msg import SynchronizationPacket
from crazyflie_trajectory.msg import TrajectoryPacket
from crazyflie_trajectory.msg import PointPacket
from std_srvs.srv import Empty
import roslaunch
import numpy as np
import matplotlib.pyplot as plt

class Converter(object):

    def __init__(self):
        self.meas_pose_sub = rospy.Subscriber('/crazyflie/log_pos', GenericLogData, self.handle_measured_pose)
        self.ref_pose_sub  = rospy.Subscriber('/crazyflie/log_ref', GenericLogData, self.handle_reference_pose)
        self.ref_pose_sub  = rospy.Subscriber('/save', String, self.handle_save)
        self.measurement = []
        self.reference = []
        
    def handle_measured_pose(self, data):
        line = list(data.values) + [data.header.seq] + [data.header.stamp.secs] + [data.header.stamp.nsecs]
        self.measurement += [line]

    def handle_reference_pose(self, data):
        line = list(data.values) + [data.header.seq] + [data.header.stamp.secs] + [data.header.stamp.nsecs]
        self.reference += [line]
    
    def handle_save(self, msg):
        directory = '/home/mgreiff/catkin_ws/src/crazyflie_trajectory/converted_data/'
        print 'Saving...'
        with open(directory + msg.data + '.reference.txt', 'w') as ref_file:
            ref_file.write(str(self.reference))
        with open(directory + msg.data + '.measurement.txt', 'w') as meas_file:
            meas_file.write(str(self.measurement))
        print 'done!'
        data = np.array(self.measurement)
        plt.plot(data[:,0])
        plt.plot(data[:,1])
        plt.plot(data[:,2])
        plt.show()
        
def error(argument, value):
    print 'ERROR. Invalid use of %s, cannot be set to "%s".' % (argument, value)

def signal_handler(signal, frame):
    print 'Shutting down master node nicely!'
    sys.exit(0)

def main():
    rospy.init_node('master')
    converter = Converter()
    signal.signal(signal.SIGINT, signal_handler)
    rospy.spin()

if __name__ == '__main__':
    main()
