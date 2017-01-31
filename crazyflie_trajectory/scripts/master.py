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
#from crazyflie_trajectory.msg import FullControl
from std_srvs.srv import Empty
import roslaunch

# Loads the trajectorylib library module from the /modules directory
curdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
dir_trajectories = os.path.join(os.path.dirname(curdir),'trajectories')
dir_modules = os.path.join(os.path.dirname(curdir),'modules')
sys.path.insert(0, dir_modules)
from trajectorylib import Trajectory

class Master(object):

    def __init__(self):
        # Publishers for communicating with the crazyflie
        self.controlPub = rospy.Publisher('/crazyflie/cmd_vel', Twist, queue_size = 1)
        self.traj_packet_pub = rospy.Publisher('/crazyflie/trajectory_packet', TrajectoryPacket, queue_size = 100)
        #self.full_control_packet_pub = rospy.Publisher('/crazyflie/full_control', FullControl, queue_size=1, latch=True)
        self.sync_packet_pub = rospy.Publisher('/crazyflie/synchronization_packet', SynchronizationPacket, queue_size = 100)

        # Services
        self.parameterserver = rospy.ServiceProxy('/crazyflie/update_params', UpdateParams)
        self.emergency = rospy.ServiceProxy('/crazyflie/emergency', Empty)

        # Trajectory object
        self.trajectory = None

        print ('\nThe master node is used to communicate with the system, type -h for help.')

    def writeParam(self, value, param):
        rosparam = 'crazyflie/'+param
        rospy.set_param(rosparam, value)
        self.parameterserver([param])

    def synchronize_trajectory(self, synchronize, time):

        if self.trajectory == None:
            self.issue_warning('A trajectory must be loaded before the trajectory may be visualized')
            return

        try:
            sp = SynchronizationPacket()
            sp.synchronize = synchronize
            sp.circular = self.trajectory.circular
            sp.number   = self.trajectory.number
            sp.time = [float(time),float(time),float(time),float(time)]
            self.sync_packet_pub.publish(sp)
        except:
            self.issue_warning('Could not form synchronization packet.')
            pass 
        
    def loadPoint(self):
        # Send a random packet to show that it is done very quickly and bypasses the evaluation section
        #fcPacket = FullControl()
        fcPacket.enable = False
        fcPacket.xmode = 0b111
        fcPacket.ymode = 0b111
        fcPacket.zmode = 0b111
        fcPacket.x = [1,0,0]
        fcPacket.y = [1,0,0]
        fcPacket.z = [1,0,0]
        fcPacket.yaw = [1,0]
        #self.self.full_control_packet_pub.publish(fcPacket)

    def load_trajectory(self, filename, fileformat):
        """
        Loads a trajectory file in the self.trajectory containing a trajectory object
        """
        directory = dir_trajectories
        self.trajectory = Trajectory(directory, filename, fileformat)

    def visualize_trajectory(self, dimension, projection):
        """
        Visualizes the trajectory
        """
        if self.trajectory == None:
            self.issue_warning('A trajectory must be loaded before the trajectory may be visualized')
            return
        
        self.trajectory.visualize_fixed(dimension, projection)

    def transfer_trajectory(self):
        if self.trajectory == None:
            self.issue_warning('A trajectory must be loaded before being transferred to the UAV')
            return

        # Open data and send a synchronization packet to clear old data on the crazyflie
        self.synchronize_trajectory(synchronize = 0, time = 0.0) # Clear old data on the crazyflie and load settings
        
        # Represent the trajectory as a set of packets
        data = self.trajectory.packet()
        
        try:
            for ii in range(20):
                for packet in data['packets']:
                    time.sleep(0.005)
                    tp = TrajectoryPacket()
                    tp.data = packet['data']
                    tp.time = packet['time']
                    tp.dimension = packet['dimension']
                    tp.index = packet['index']
                    tp.number = packet['number']
                    tp.type = packet['type'];
                    self.traj_packet_pub.publish(tp)
        except:
            print('Failed to create data packet, please chack that %s contains a "settings" field' % filename)
            return

    def transfer_setpoint(self, x, y, z, yaw):
        print 'Not yet written'

    def issue_warning(self, message):
        print('WARNING. %s' % message)

    def help(self):
        print ('USAGE: KEYWORD ARG1 [ARG...]. The KEYWORD Can be set to:\n\n'+
               '    * "q" - Calls emergency service.\n'+
               '    * "l" - Load a trajectory file from /trajectories.\n'+
               '    * "v" - Visualize a trajectory file.\n'+
               '    * "t" - Transfer a trajectory file to the UAV.\n'+
               '    * "s" - Synchronizes crazyflie(s), ARG0 specifies\n'+
               '            a time delay from synchronization to the start\n'+
               '        of the trajectory.\n'+
               '    * "c" - Continuous feed of setpoints, as done by Mike.\n'+
               '    * "p" - Sends a setpoint in (ARG1=x,ARG1=y,ARG1=z,ARG1=yaw).\n\n'+
               'EXAMPLE USAGE: Commands for loading, visualising and flying along a trajectory\n'
               '    l p_1\n'+
               '    v\n'+
               '    t\n'+
               '    s 0\n'+
               '    q\n\n')

def error(argument, value):
    print 'ERROR. Invalid use of %s, cannot be set to "%s".' % (argument, value)

def signal_handler(signal, frame):
    print 'Shutting down master node nicely!'
    sys.exit(0)

def main():
    rospy.init_node('master')
    master = Master()
    signal.signal(signal.SIGINT, signal_handler)
    
    combinations = {'q':1, 'l':2, 'v':1, 's':2, 't':1, 'c':3, 'p':5}
    
    while True:
        try:
            command = raw_input('(@ master) Command: ')
            data = command.split(' ')
            if data[0] == '-h' or data[0] == '--help':
                master.help()
            elif not data[0] in combinations.keys():
                master.issue_warning(('Unsupported KEYWORD %s, type -h for help'+
                                      'and options') % str(data[0]))
            elif not combinations[data[0]] == len(data):
                master.issue_warning(('Invalid number of arguments %s, KEYWORD'+
                                     '%s must have %i entries') % (len(data)-1,
                                     data[0],combinations[data[0]]-1))
            else:
                if data[0] == 'q':
                    # Calls emergency exit through the ros driver
                    master.emergency()
                elif data[0] == 'l':
                    # Load a trajectory file
                    master.load_trajectory(filename = data[1], fileformat = 'json')
                elif data[0] == 'v':
                    # Visualize the loaded trajectory
                    # TODO allow the user to specify dimension and projection
                    master.visualize_trajectory(dimension = ['x','y','z'], projection = ['1D', '2D', '3D'])
                elif data[0] == 's':
                    # TODO allow the user to clear the crazyflie data
                    # Synchronizes trajectory without erasing the data and writes a start time delay
                    master.synchronize_trajectory(synchronize=1, time=float(data[1]))
                elif data[0] == 't':
                    # Transfer trajectory file to the crazyflie
                    master.transfer_trajectory()
                elif data[0] == 'c':
                    # Runs a continuous feed of points, as done by Mike, here for debugging purposes
                    master.transfer_continuous(data[1], data[2])
                elif data[0] == 'p':
                    # Load a setpoint which is smoothed
                    master.transfer_setpoint(x=data[1],y=data[2],z=data[3],yaw=data[4])
        except EOFError:
            master.issue_warning('EOF error encountered in Master.')
            return

if __name__ == '__main__':
    main()
