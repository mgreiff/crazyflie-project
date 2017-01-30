#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import os, sys, inspect
import signal
from json import dumps, load
import time
from geometry_msgs.msg import Twist
from crazy_ros.srv import UpdateParams
from crazy_ros.msg import GenericLogData
from crazy_ros.msg import SynchronizationPacket
from crazy_ros.msg import TrajectoryPacket
from crazy_ros.msg import PointPacket
from std_srvs.srv import Empty
import roslaunch

# Loads the crazyflie library module from the /modules directory
curdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
trajdir = os.path.join(os.path.dirname(curdir),'trajectories')
libdir = os.path.join(os.path.dirname(curdir),'modules')
sys.path.insert(1,libdir)
from crazylib import Trajectory

class Master(object):

    def __init__(self):
        # Sets up publishers and subscribers for the reference generator
        self.kinect_status_pub = rospy.Publisher('/kinect/in', String, queue_size = 10)
        self.kinect_status_sub = rospy.Subscriber('/kinect/out', String, self.handle_system_status)
        self.status_master = rospy.Subscriber('/system/status_master', String, self.handle_system_status)
        self.status_controller = rospy.Publisher('/system/status_controller', String, queue_size = 10)
        
        # Publishers for communicating with the crazyflie
        self.controlPub = rospy.Publisher('/crazyflie/cmd_vel', Twist, queue_size = 1)
        self.traj_packet_pub = rospy.Publisher('/crazyflie/trajectory_packet', TrajectoryPacket, queue_size = 100)
        self.point_packet_pub = rospy.Publisher('/crazyflie/point_packet', PointPacket, queue_size = 100)
        self.sync_packet_pub = rospy.Publisher('/crazyflie/synchronization_packet', SynchronizationPacket, queue_size = 100)

        # Visualization publishing
        self.traj_file_pub = rospy.Publisher('/visualization/trajectory_file', String, queue_size = 1)
        self.pcl_save_pub = rospy.Publisher('/visualization/pointcloud_save', String, queue_size = 1)
        self.pcl_load_pub = rospy.Publisher('/visualization/pointcloud_load', String, queue_size = 1)
        self.pcl_active_pub = rospy.Publisher('/visualization/pointcloud_active', String, queue_size = 1)
        
        # Services
        self.parameterserver = rospy.ServiceProxy('/crazyflie/update_params', UpdateParams)
        self.emergency = rospy.ServiceProxy('/crazyflie/emergency', Empty)

        # Waits until the kinect node is done calibrating
        self.status = True 

        # Loads the trajectory object with an absolute path to the /trajectoies directory
        # and sets the internal trajectory attribute, loading Trajectory2.mat by default
        # here we will only use the trajectory object to handle files, so the prediction
        # horison predN and timestep can be set arbitrarily. The maximum number of
        # coeficients that can be sent for each polynomial coefficient vector due to
        # restrictions in the ros driver. TODO: Disable the mentioned restrictions in
        # Wolfgang's ROS driver.
        curdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
        trajdir = os.path.join(os.path.dirname(curdir),'trajectories')
        self.trajectory = Trajectory('Trajectory2.mat', trajdir, 1, 1)
        self.maxNumCoeff = 45 # Currently a hard coded upper bound in the firmware

        # Loads the input arguments and configures the system mode
        self.mode = {'useKinect':0,'useOptitrack':0,'useUWBLPS':0,'useCrazyflie':0,
                     'useSimulation':0,'useInternal':0,'useExternal':0}
        self.mappingActive, self.internalActive, self.externalActive = 0, 0, 0
        for arg in rospy.myargv(argv=sys.argv):
            data = arg.split(',')
            if len(arg.split(',')) > 1:
                if data[0] == 'useKinect':
                    self.mode['useKinect'] = int(data[1])
                elif data[0] == 'useOptitrack':
                    self.mode['useOptitrack'] = int(data[1])
                elif data[0] == 'useUWBLPS':
                    self.mode['useUWBLPS'] = int(data[1])
                elif data[0] == 'useCrazyflie':
                    self.mode['useCrazyflie'] = int(data[1])
                elif data[0] == 'useSimulation':
                    self.mode['useSimulation'] = int(data[1])
                elif data[0] == 'useInternal':
                    self.mode['useInternal'] = int(data[1])
                elif data[0] == 'useExternal':
                    self.mode['useExternal'] = int(data[1])

        print ('\nThe master node is used to communicate with the system.\n'+
              'Type -h for help or ^C and ^D to exit once the calibration is complete.')

    def handle_system_status(self, msg):
        # Enables the system to continue operation
        if msg.data == 'True':
            self.status = True

    def loadMode(self, setting):
        groups = rospy.get_param("/crazyflie")
        if "controller" in groups:
            # Set controller mode
            value = ["IDLE", "LOAD", "FLY", "LAND", "EMERGENCY"].index(setting)
            param = 'controller/mode'
            rosparam = 'crazyflie/'+param
            rospy.set_param(rosparam, value)
            self.parameterserver([param])
        else:
            error = ('Could not find controller in TOC. Please make sure that the\n'+
                     '"controller/" groups are defined in the crazyflie firmware and recompile')
            raise Exception(error)

    def loadSetPoint(self, x, y, z):
        groups = rospy.get_param("/crazyflie")
        if 'setpoint' in groups:
            rospy.set_param('crazyflie/setpoint/x', x)
            rospy.set_param('crazyflie/setpoint/y', y)
            rospy.set_param('crazyflie/setpoint/z', z)
            rospy.set_param("flightmode/posSet", 1)
            self.parameterserver(['setpoint/x', 'setpoint/y', 'setpoint/z', 'flightmode/posSet'])
        else:
            error = ('Could not find setpoint in TOC. Please make sure that the\n'+
                     '"setpoint" groups are defined in the crazyflie firmware and recompile')
            raise Exception(error)

    def synchronizeTrajectory(self, synchronize):
        # Sends a single synchronization package
        
        # Todo load true data from the trajectoryObject
        sp = SynchronizationPacket()
        sp.synchronize = 1
        if (int(synchronize) == 0):
            sp.synchronize = 0
        sp.circular = [0,0,0,0]
        sp.number   = [6,6,6,0]
        sp.time = [3.0,4.0,5.0,0.0]
        print sp
        self.sync_packet_pub.publish(sp)

    def loadPoint(self):
        pp = PointPacket()
        pp.enable = 0;
        pp.x   = [1.0,2.0,3.0,4.0]
        pp.y   = [1.1,2.1,3.1,4.1]
        pp.z   = [1.2,2.2,3.2,4.2]
        pp.yaw = [1.3,2.3]
        print pp
        self.point_packet_pub.publish(pp)

    def loadTrajectory(self, filename):

        try:
            with open(os.path.join(trajdir, filename)) as data:
                trajectory = load(data)
        except:
            print('Could not open file %s, no such file or directory' % filename)
            return

        # Send file to visualization node
        self.traj_file_pub.publish(filename)
        
        # Open data and send a synchronization packet to clear old data on the crazyflie
        # and set trajectory settings
        try:
            sp = SynchronizationPacket()
            sp.synchronize = 0
            sp.circular = trajectory['settings']['circular']
            sp.number   = trajectory['settings']['number']
            sp.time     = [0.0,0.0,0.0,0.0]   # Irrelevant, set when initializing later on
            self.sync_packet_pub.publish(sp)
        except:
            print('Failed to create and send settings packet, please chack that %s contains a "settings" field' % filename)
            return
        
        # Send packet data of entire trajectory (multiple times in order to get around problems with lost packets?)
        try:
            for ii in range(10):
                for packet in trajectory['packets']:
                    # TODO: Strange that we have to sleep in order for the CRTP to functon?
                    # TODO: Load data from trajectory objet
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

        try:
            sp = SynchronizationPacket()
            sp.synchronize = 1
            sp.circular = trajectory['settings']['circular']
            sp.number   = trajectory['settings']['number']
            sp.time     = [0.0,0.0,0.0,0.0] # can be zero separate
            self.sync_packet_pub.publish(sp)
        except:
            print('Failed to create and send initialization packet, please chack that %s contains a "settings" field' % filename)
            return

    def help(self):
        if self.mappingActive:
            # Mapping mode
            print ('USAGE: KEYWORD ARG\n'+
                   '    KEYWORD - Can be set to:\n\n'+
                   '        1) "q" - Exits mapping mode\n'+
                   '        1) "save" - Saves the current PCL in RVIZ, here ARG is the name of the\n'+
                   '            file to be written.\n'
                   '        2) "load" - Loads a PCL into RVIZ, here ARG is the name of a file in\n'+
                   '            /data directory.\n'
                   '        3) "auto" - Automatically enter the rotation and translation of the\n'+
                   '            camera from the LPS, in this case ARG is the fps in the automatic,\n'+
                   '            generation, set to >1.'
                   '        4) "man" -  Manually enter the rotation and translation of the camera\n'+
                   'EXAMPLE USAGE:\n\n'+
                   '    load SeminairRoom2211b.pcl\n\n')
        elif self.internalActive:
            # Internal mode
            print ('USAGE: KEYWORD ARG\n'+
                   '    KEYWORD - Can be set to:\n\n'+
                   '        1) "tf" - Transfer a trajectory file to the crazyflie from the\n'+
                   '           /trajectories directory where ARG specifies the desired file name.\n'+
                   '        2) "sy" - Synchronizes crazyflie and sets attributes of the trajectory.\n'+
                   '        3) "ctr" - Switches controller in the firmware, valid arguments ARG are:'+
                   '            * "IDLE" - Sets the quadrotor in an idle state.\n'+
                   '            * "HOVER" - Initialises the small angle hover controller controller\n'+
                   '            * "TRACK" - Initialises the trajectory following controller, using\n'+
                   '                this requires that trajectory to be defined and loaded.\n'+
                   '            * "LAND" - Lands the quadrotor safely from the current position.\n'+
                   '        4) "set" - Sends a setpoint in (x,y,z) when in HOVER or LAND.\n\n'+
                   '        5) "man" - Changes a setpoint when in HOVER or LAND using arrow keys.\n\n'+
                   'EXAMPLE USAGE:\n\n'+
                   '    setp 1 1 1\n    ctr HOVER\n    traj Trajectory1\n    ctr TRACK\n    ctr LAND\n\n'+
                   '    This sequence of commands makes the quadcopter hover in (1,1,1) until the\n'+
                   '    trajectory has been loaded. By starting at the fourth command, the\n'+
                   '    trajectory is followed and when using ctr LAND, exits and lands safely.\n')
        elif self.externalActive:
            # External mode
            print ('USAGE: NODE KEYWORD ARG\n'+
                   '    NODE - Can be set to:\n\n'
                   '        1) "q" - Immediately kills all control signals\n\n'+
                   '        2) "ctr" - Defines how the control signal is computed, can be set\n'+
                   '            with the following keywords:\n'+
                   '            * "MAN" - Manually define control signals using w-a-s-d-z-c-r-f keys\n'+
                   '            * "PID" - Starts the PID controller with zero yaw shuts down all \n'+
                   '                other controllers\n'+
                   '            * "TILQR" - Starts the time varying LQRi --||--\n'+
                   '            * "TVLQR" - Starts the time varying LQRi --||--\n'+
                   '            * "TVMPC" - Starts the time invariant MPC --||--\n'+
                   '            * "TVMPC" - Starts the time varying MPC --||--\n\n'+
                   '        3) "kin" - Communicate with the Kinect-link node if launched, the\n'+
                   '            keyword can be set to:\n'+
                   '            * "cal" - Recalibrates both background and the camera angle\n'+
                   '            * "bcg" - Displays a static image of the background sample\n'+
                   '            * "plt" - Real time plotting of the background and raw position\n')
        else:
            # No mode selected
            print ('USAGE: KEYWORD\n'+
                   '    KEYWORD - Can be set to:\n\n'
                   '        1) "q" - Shuts down the program safely.\n'+
                   '        2) "map" - Launches the kinect camera and maps the room as a PCL.\n'+
                   '        3) "int" - Enters the internal control mode, using which requires\n'+
                   '                other controllers\n'+
                   '        4) "ext" - Enters the external control mode, using exterior MOCCAP such\n'+
                   '                as the Kinect or Optitrack/Qualisys systems.\n\n')

    def sanity_check(self):
        # Sanity checks for the system configuration
        if self.mode['useCrazyflie'] + self.mode['useSimulation'] != 1:
            raise Exception('ERROR. The system must be configured to use either the simulation or the crazyflie.')
        if self.mode['useCrazyflie'] and self.mode['useKinect'] + self.mode['useOptitrack'] + self.mode['useUWBLPS'] != 1:
            raise Exception('ERROR. The system must be launched with one and only one of the moccap systems.')
        if self.mode['useSimulation'] and self.mode['useKinect'] + self.mode['useOptitrack'] + self.mode['useUWBLPS'] != 0:
            raise Exception('ERROR. Cannot launch in simulation mode with moccap systems active.')
        if self.mode['internalMode'] + self.mode['externalMode'] != 1:
            raise Exception('ERROR. Must launch in either internal or external mode.')

def error(argument, value):
    print 'ERROR. Invalid use of %s, cannot be set to "%s".' % (argument, value)

def signal_handler(signal, frame):
    print 'Shutting down master node nicely!'
    sys.exit(0)

def main():
    rospy.init_node('master')
    master = Master()
    signal.signal(signal.SIGINT, signal_handler)
    
    # Accepted mode KEYWORDs
    ModeOpt = ['map', 'int', 'ext']
    
    # Accepted Kinect KEYWORDs
    KinOpt = ['cal', 'bcg', 'plt']

    # Accepted Controller KEYWORDs
    CtrOpt = ['MAN','PID', 'TILQR', 'TVLQR', 'TIMPC', 'TVMPC', ]

    timeAuto = 0
    while True:
        if time.time() - timeAuto > 600:
            master.status = True
        if master.status:
            try:
                command = raw_input('(@ master) Command: ')
                data = command.split(' ')
                if data[0] == '-h' or data[0] == '-help':
                    master.help()
                elif data[0] == 'map':
                    master.mappingActive = 1
                elif data[0] == 'int':
                    master.internalActive = 1
                elif data[0] == 'ext':
                    master.esternalActive = 1
                elif master.mappingActive:
                    if data[0] == 'q':
                        master.mappingActive = 0;
                        master.pcl_active_pub.publish('False')
                    elif len(data) < 2:
                        print 'ERROR. No argument was provided'
                    elif data[0] == 'save':
                        try:
                            master.pcl_save_pub.publish(data[1])
                        except:
                            error('ARG', data[1])
                    elif data[0] == 'load':
                        try:
                            master.pcl_load_pub.publish(data[1])
                        except:
                            error('ARG', data[1])
                    elif data[0] == 'auto':
                        master.pcl_active_pub.publish('True')
                        timeAuto = time.time()
                        master.status = False
                    else:
                        error('KEYWORD', data[0])
                elif master.internalActive:
                    if data[0] == 'q':
                        # Calls emergency exit through the ros driver
                        master.emergency()
                        master.internalActive = 0
                    elif data[0] == 'l':
                        # Sends a trajectory file to the crazyflie
                        master.loadTrajectory(data[1])
                    elif data[0] == 's':
                        # Synchronizes trajectory based on the current trajectory object
                        master.synchronizeTrajectory(data[1])
                    elif data[0] == 'p':
                        # Sends a point in flat output space for debugging purposes
                        master.loadPoint()
                    elif data[0] == 'ctr':
                        # Initializes a controller
                        if data[1] in ["IDLE", "LOAD", "FLY", "LAND", "EMERGENCY"]:
                            master.loadMode(data[1])
                        else:
                            error('ARG', data[1])
                    elif data[0] == 'setp':
                        print data
                        a = float(data[1])
                        #try:
                        x, y, z = float(data[1]), float(data[2]), float(data[3])
                        master.loadSetPoint(x, y, z)
                        #except Exception:
                        #    error('ARG', data[1:])
                    else:
                        error('KEYWORD', data[0])
                elif master.externalActive:
                    if data[0] == 'q':
                        # Pauses all controllers
                        master.status_controller.publish('q')
                        master.internalActive = 0
                    elif data[0] == 'kin' and len(data) == 2:
                        # Sends command to the Kinect node and forces the master
                        # node to pause while calibrating (as
                        # the kinect node then requires the terminal prompt).
                        if data[1] in ['cal', 'bcg', 'plt']:
                            master.kinect_status_pub.publish('recalibrate')
                            master.status = False
                        else:
                            error('KEYWORD', data[1])
                    elif data[0] == 'ctr' and len(data) == 2:
                        # Checks that the user has supplied valid inputs and sends
                        # commands to the /controller/* generator.
                        if data[1] in CtrOpt:
                            master.status_controller.publish(data[1])
                            master.status = False
                        else:
                            error('KEYWORD', data[1])
                    else:
                        error('NODE', data[0])
                else:
                    error('KEYWORD', data[0])
            except EOFError:
                print 'EOFError encountered in master!'
                return

if __name__ == '__main__':
    main()
