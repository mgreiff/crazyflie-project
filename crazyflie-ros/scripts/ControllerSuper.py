#!/usr/bin/env python

import rospy
import sys, os, inspect
import numpy as np
from select import select
import termios
import contextlib
from std_msgs.msg import String, Float64, Header
from crazy_ros.msg import NumpyArrayFloat64
from geometry_msgs.msg import Point, PointStamped
from time import sleep, time
from math import sin, pi, sqrt, cos, sin
from json import dumps, load
import signal
from geometry_msgs.msg import Twist

# Loads the crazyflie library module from the /modules directory
curdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
libdir = os.path.join(os.path.dirname(curdir),'modules')
sys.path.insert(1,libdir)
from crazylib import Trajectory

@contextlib.contextmanager
def raw_mode(file):
    old_attrs = termios.tcgetattr(file.fileno())
    new_attrs = old_attrs[:]
    new_attrs[3] = new_attrs[3] & ~(termios.ECHO | termios.ICANON)
    try:
        termios.tcsetattr(file.fileno(), termios.TCSADRAIN, new_attrs)
        yield
    finally:
        termios.tcsetattr(file.fileno(), termios.TCSADRAIN, old_attrs)

class Controller(object):

    def __init__(self, key):
        self.twist = Twist()
        self.key = key
        self.status = False
        self.xhat = np.zeros((12,1))
        self.useKinect = 0
        self.useCrazyflie = 0
        self.useSimulation = 0
        self.useOptitrack = 0
        self.useUWBLPS = 0

        # Loads configuration parameters
        try:
            for arg in rospy.myargv(argv=sys.argv):
                if len(arg.split(',')) > 1:
                    if arg.split(',')[0] == 'useKinect':
                        self.useKinect = int(arg.split(',')[1])
                        if self.useKinect:
                            # Subscribes to the kinect data if applicable (Point data)
                            self.kinect_pos_sub = rospy.Subscriber('/kinect/pos', Point, self.handle_pos_point)
                            self.kinect_vel_sub = rospy.Subscriber('/kinect/vel', Point, self.handle_vel_point)

                    elif arg.split(',')[0] == 'useCrazyflie':
                        self.useCrazyflie = int(arg.split(',')[1])
                        # TODO include subscription of IMU data from the crazyflie
                        if self.useCrazyflie:
                            print '(@ %s) WARNING. Crazyflie IMU data subscription is not yet supported.' % str(self)
                        
                    elif arg.split(',')[0] == 'useSimulation':
                        self.useSimulation = int(arg.split(',')[1])
                        # Subscribes to the simulation states if applicable (numpy array floatd64)
                        if self.useSimulation:
                            self.measurement_sub = rospy.Subscriber('/model/states', NumpyArrayFloat64, self.handle_model_data)

                    elif arg.split(',')[0] == 'useOptitrack':
                        self.useOptitrack = int(arg.split(',')[1])
                        # TODO include optitrack subscription, connecting to the subscriber
                        if self.useOptitrack:
                            print '(@ %s) WARNING. Optitrack data subscription is not yet supported.' % str(self)
                        
                    elif arg.split(',')[0] == 'useUWBLPS':
                        self.useUWBLPS = int(arg.split(',')[1])
                        # TODO include UWBLPS subscription, connecting to the subscriber
                        if self.useUWBLPS:
                            print '(@ %s) WARNING. UWBLPS data subscription is not yet supported.' % str(self)

                if os.path.basename(arg) == 'configparam.cnf':
                    with open(arg) as configfile:
                        param = load(configfile)
                        self.timestep = param['global']['outer_loop_h']
                        self.modParam = param['quadcopter_model']
                        self.param = param[self.key]
                        self.param_PD = param['PD'] # Trust mapping parameters
                    configfile.close()
        except:
            raise ValueError('(@ %s) ERROR. Could not load configuration parameters.' % (str(self)))
            
        # Loads the trajectory object with an absolute path to the /trajectoies directory
        # and sets the internal trajectory attribute
        curdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
        trajdir = os.path.join(os.path.dirname(curdir),'trajectories')
        if self.key == 'MPC':
            predN = self.param['predN']
        else:
            predN = 1
        self.useManReference = False
        self.trajectory = Trajectory('Trajectory2.mat', trajdir, predN, self.timestep)
        self.manualReference = np.zeros((12, predN))

        # Checks that a controller target exists
        if (self.useSimulation + self.useCrazyflie) != 1:
            raise ValueError('ERROR. Either of the useSimulation and useCrazyflie must be 1 in %s' % (str(self)))

        # Controller status subscription
        self.status_sub = rospy.Subscriber('/system/status_controller', String, self.handle_status)

        # Control signal publisher, operates regardless of using simulation or process
        self.control_pub = rospy.Publisher('/crazyflie/cmd_vel', Twist, queue_size = 10)
        self.reference_pub = rospy.Publisher('/controller/reference', NumpyArrayFloat64, queue_size = 10)
        self.execution_time_pub = rospy.Publisher('/controller/execution_time', Float64, queue_size = 10)
        self.master_status_pub = rospy.Publisher('/system/status_master', String, queue_size = 10)
        
        # Visualisation topics
        self.viz_pos_pub = rospy.Publisher('/system/viz_ref_pos', PointStamped, queue_size = 10)

    def handle_status(self, msg):
        """
        Callback for the controller status, effectively the controller UI.
        If the message matches the controller key, the user is allowed to interact with the node
        Args:
            msg - Message containing a controller key (/std_messages/String) 
        Returns:
        """
        if msg.data == self.key:
            print ('\nType -h for help or ^C to exit the entire program.\n')
            cont  = True
            while cont:
                command = raw_input('(@ %s) Command: ' % str(self))
                if command == '-h':
                    print ('USAGE: KEYWORD ARG\n'+
                           '    1) "q" - Immediately kills all control, setting references to zero\n'+
                           '    2) "b" - Returns to the mastern\n'+
                           '    3) "l [FILE]" - Load the FILE trajectory file (suffix .json or .mat included)\n'+
                           '    4) "s"  - Starts the controller, following the FILE trajectory\n'+
                           '    5) "m"  - Starts the manual reference generator UI\n')
                elif command.split(' ')[0] == 'l' and len(command.split(' ')) > 1:
                    try:
                        self.trajectory.load(command.split(' ')[1])
                    except:
                        print '(@ %s) WARNING. Could not load the trajectory file %s.' % (str(self), command.split(' ')[1])
                elif command == 'b':
                    self.status = False
                    cont = False
                    self.master_status_pub.publish('True')
                elif command == 's':
                    self.useManReference = False
                    self.traj_start = time()
                    cont = False
                    self.status = True
                elif command == 'm':
                    self.useManReference = True
                    cont = False
                    self.traj_start = time()
                    self.status = True
                elif len(command) > 0:
                    print '(@ %s) WARNING. Unsupported keyword %s, type -h for help.' % (str(self), command)
                else:
                    print '(@ %s) WARNING. No command received' % str(self)
        else:
            self.status = False
            if msg.data == 'q':
                self.twist.linear.z = 0
                self.twist.angular.x = 0
                self.twist.angular.y = 0
                self.twist.angular.z = 0
                self.control_pub.publish(self.twist)
    
    def handle_pos_point(self, pos):
        """
        Updates the internal positional states in the controller as soon as new
        data is received. This is a callback for the kienct node from the
        kinect_vision project.
        Args:
            vel - Positional data from the kinect (geometry_msgs/Point)
        Returns:
        """
        self.xhat[0,0] = pos.x
        self.xhat[1,0] = pos.y
        self.xhat[2,0] = pos.z

    def handle_vel_point(self, vel):
        """
        Updates the internal velocital states in the controller as soon as new
        data is received. This is a callback for the kienct node from the
        kinect_vision project.
        Args:
            vel - Positional data from the kinect (geometry_msgs/Point)
        Returns:
        """
        self.xhat[3,0] = pos.x
        self.xhat[4,0] = pos.y
        self.xhat[5,0] = pos.z
    
    def handle_model_data(self, xhat):
        """
        Updates the internal velocital states in the controller as soon as new
        data is received. This is a callback for the kienct node from the
        kinect_vision project.
        Args:
            xhat - Positional data from the kinect (crazy_ros.msg/NumpyArrayFloat46)
        Returns:
        """
        self.xhat = xhat.data
    
    
    def compute_control(self):
        """
        Computes the control signal in the outer controller.
        Args:
        Returns:
        """
        raise Exception('compute_control() chould be over-written in the'+
                        'inheriting controller')

    def sat(self, arr, minlim, maxlim):
        """
        Checks if the specified value is within the bounds of saturation and
        returns the saturated value and a tag specifying if the value 
        Args:
            arr - A a floating point array (1xN numpy array).
            minlim - The lower bound of the saturation (1xN numpy array).
            maxlim - The upper bound of the saturation (1xN numpy array).
        Returns:
            arr - The saturated array (1xN numpy array).
            status - A boolean value set to 1 if the initial value was outside
                the bounds of saturation (1xN Boolean).
        """
        maxstat = arr >= maxlim
        minstat = arr <= minlim
        stat = maxstat + minstat
        if maxstat.any():
            arr[maxstat] = maxlim[maxstat]
        if minstat.any():
            arr[minstat] = minlim[minstat]
        return arr, stat

    def compute_reference(self, t):
        pos_d = 0.05    # [m]
        ang_d = 0.01    # [rad]
        # Key bindings
        combinations = {'a':{'d':[0,pos_d],'n':'x [m]'},'z':{'d':[0,-pos_d],'n':'x [m]'},
                        's':{'d':[1,pos_d],'n':'y [m]'},'x':{'d':[1,-pos_d],'n':'y [m]'},
                        'd':{'d':[2,pos_d],'n':'z [m]'},'c':{'d':[2,-pos_d],'n':'z [m]'},
                        'f':{'d':[3,ang_d],'n':'pitch [rad]'},'v':{'d':[3,-ang_d],'n':'pitch [rad]'},
                        'g':{'d':[4,ang_d],'n':'roll [rad]'},'b':{'d':[4,-ang_d],'n':'roll [rad]'},
                        'h':{'d':[5,ang_d],'n':'yaw [rad]'},'n':{'d':[5,-ang_d],'n':'yaw [rad]'}}
        if self.useManReference == True:
            try:
                with raw_mode(sys.stdin):
                    try:
                        rlist, _, _ = select([sys.stdin], [], [], 1/35)
                        if rlist:
                            ch = sys.stdin.read(1)
                            if ch == 'q':
                                print '\r\nSetting references to zero and exiting back to controller'
                                self.manualReference *= 0.0 # Set references to zero
                                self.twist.linear.z = 0.0
                                self.twist.angular.x = 0.0
                                self.twist.angular.y = 0.0
                                self.twist.angular.z = 0.0 
                                self.status = False                     # Pause controller
                                self.handle_status(String(self.key))    # Return to UI
                            elif self.useManReference and ch in combinations.keys():
                                # Changes the current setpoint incrementally if manual
                                # setpoint generation is used
                                index = combinations[ch]['d'][0]
                                value = combinations[ch]['d'][1]
                                self.manualReference[index,:] += value
                                print '\r%s = %f' % (combinations[ch]['n'], self.manualReference[index,0])
                            else:
                                print '\rThe command "%s" is not supported' % ch
                    except (KeyboardInterrupt, EOFError):
                        pass
            except:
                pass
        elif self.useManReference == False:
            # Evaluates the currently trajectory object for the currently
            # loaded file if using automatic reference generation
            result = self.trajectory(t)
            self.manualReference = result
        
        my_header = Header(stamp=rospy.Time.now(), frame_id="map")
        my_point = Point(self.manualReference[0,0], self.manualReference[1,0], self.manualReference[2,0])
        p = PointStamped(header=my_header, point=my_point)
        self.viz_pos_pub.publish(p)
        return self.manualReference
            
    def get_thrust_PD(self, z_ref, zd_ref):
        """
        Maps the control error in z and \dot{z} to a thrust setpoint using
        a PD controller. This will be done by the Crazyflie firmware in future
        implementations, but is currently not compatible compatible with the
        crazyflie_ros driver data protocols.
        Args:
            
        Returns:
            
        """
        Kp = self.param_PD['z']['Kp']
        Kd = self.param_PD['z']['Kd']
        N = self.param_PD['z']['Kd']
        maxlim = np.array([self.param_PD['z']['maxlim']])
        minlim = np.array([self.param_PD['z']['minlim']])
        den = cos(self.xhat[7,0]) * cos(self.xhat[8,0])
        
        # Returns maximum thrust if the quadcopter is close to \pm pi/2 in roll
        # or theta, saturating the control signal to keep it within resonable
        # bounds. Here we set the thrust reference to minimum if the quadcopter is
        # upside down, should be done smoother in future iterations
        if abs(den) < 1e-10:
            return maxlim[0]
        # TODO add m and g
        u = (Kp * (z_ref - self.xhat[3,0]) + Kd * (z_ref - self.xhat[6,0])) / den
        u_sat, _ = self.sat(np.array([u]), maxlim, minlim)
        return u_sat[0]

    def run(self):
        while True:
            if self.status:
                # Check if the emergency stop has been pressed since the previous loop.
                # This is done automatically if generating manual references, and is 
                # omitted due to conflicts in termios
                if not self.useManReference:
                    try:
                        with raw_mode(sys.stdin):
                            try:
                                rlist, _, _ = select([sys.stdin], [], [], 1/35)
                                if rlist:
                                    ch = sys.stdin.read(1)
                                    if ch == 'q':
                                        self.status = False
                                        self.master_status_pub.publish('True')
                            except (KeyboardInterrupt, EOFError):
                                pass
                    except:
                        pass

                # Computes control law
                execution_time = self.compute_control()
                h = self.timestep
                
                # Computes wait time
                wait_time = h - execution_time
            else:
                wait_time = 0.05
                h = 0.05
            if wait_time >= 0:
                sleep(wait_time)
            else:
                print (("Warning, the execution time of %f exceeds the loop time %f "+
                        "in the outer %s") % (execution_time, h, self.__str__()))

    def __str__(self):
        """
        String representation of the object.
        Args:
        Returns:
            str - A string representation of the object (String)
        """
        return '%s controller' % self.key

