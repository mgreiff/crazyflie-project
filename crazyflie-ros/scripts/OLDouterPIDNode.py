#!/usr/bin/env python

import rospy
import os
import sys
from select import select
import termios
import contextlib
from time import sleep
import numpy as np
from std_msgs.msg import String, Float64
from crazy_ros.msg import NumpyArrayFloat64 # Can handle messages of type np.array, list and tuple
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
import signal
from math import sin, pi, sqrt
from json import dumps, load

class PIDcontroller(object):

    def __init__(self):
        # Sets up publishers and subscribers
        self.status_sub = rospy.Subscriber('/system/status_controller', String, self.handle_status)
        self.kinect_sub = rospy.Subscriber('/kinect/position', Point, self.handle_kinect_position)
        self.control_pub = rospy.Publisher('/system/control_signal', NumpyArrayFloat64, queue_size = 10)
        self.error_pub = rospy.Publisher('/system/error', Point, queue_size = 10)
        self.cmd_vel_pub = rospy.Publisher('crazyflie/cmd_vel', Twist, queue_size = 10)
        self.status_master = rospy.Publisher('/system/status_master', String, queue_size = 10)
        
        self.twist = Twist()
        self.r = rospy.Rate(30)
 
        self.status = False
        self.kinect_position = None
        params = None
        
        self.K = 28000
        self.Ki = 6000
        self.KD = 4000
        self.I = 0
        self.e_old = 0
        self.thrust_offset = 40000
        self.takeoff_time = 20
        self.takeoff_thrust = 55000

        self.thrust_min = 35000
        self.thrust_max = 60000
        
        self.ref = [0, 0.0, 0]

        # Loads configuration parameters
        for filename in rospy.myargv(argv=sys.argv):
            if os.path.basename(filename) == 'configparam.cnf':
                with open(filename) as configfile:
                    param = load(configfile)
                configfile.close()
        try:
            if param != None:
                # Sets configuration parameters
                self.timestep = param['global']['outer_loop_h']
                self.param = param
        except:
            print 'ERROR. Could not load configuration parameters in %s' % (str(self))
    
    def pub_cmd_vel(self, thrust, pitch, roll, yaw_rate = 0):
        self.twist.linear.z = thrust
        self.twist.linear.y = roll # Roll, degrees. Positive moves "right"
        self.twist.linear.x = pitch # Pitch, degrees. Positive moves "forward"
        self.twist.angular.z = yaw_rate
        
        self.cmd_vel_pub.publish(self.twist)

    
    def get_thrust(self):
        if self.kinect_position is None:
            return self.thrust_min

        if self.setup:
            return 0

        if self.landing:
            return self.thrust_min

        yr = self.ref[1]
        y = self.kinect_position.y
        e = yr - y
        
        h = rospy.get_time() - self.time
        self.time = rospy.get_time()
        
        self.I += h * e * self.Ki
        
        thrust = self.K * e + self.I + self.KD * (e - self.e_old) / h
        
        self.e_old = e

        thrust += self.thrust_offset

        if thrust > self.thrust_max:
            self.I -= thrust - self.thrust_max
            thrust = self.thrust_max
        elif thrust < self.thrust_min:
            self.I += self.thrust_min - thrust
            thrust = self.thrust_min

        return thrust

    def handle_status(self, msg):
        # Callback for commands from master which generates a trajectory
        # if MPC is broadcasted, but turns of the controller if anything
        # else is broadcasted and sets all conttrol signals to 0 if "q"
        # is broadcasted
        if msg.data == 'PID':
            self.status = True
            self.setup = True
            self.landing = False
            self.time = rospy.get_time()
            self.takeoff_frames = self.takeoff_time
            print 'Now reading keyboard data for position control, steer with a-s-d-w'
        else:
            self.status = False
            if msg.data == 'q':
                self.control_pub.publish(np.zeros((4,1)))

    def handle_kinect_position(self, point):
        self.kinect_position = point

    def __str__(self):
        return 'PID controller node'

def signal_handler(signal, frame):
    print 'Shutting down PID node nicely!'
    sys.exit(0)

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

def main():
    rospy.init_node('PIDcontroller')
    PID = PIDcontroller()
    signal.signal(signal.SIGINT, signal_handler)

    # Have the while loop outside of the node in order to allow
    # interruptions from the handle_status callback
    thrust = 0
    pitch = 0
    roll = 0
    yaw_rate = 0
    
    thrust_d = 1000
    pitch_d = 1
    roll_d = 1
    yaw_rate_d = 5
    yr_d = 0.1
    
    while True:
        
        if PID.status:
            with raw_mode(sys.stdin):
                try:
                    rlist, _, _ = select([sys.stdin], [], [], 1/35)
                    if rlist:
                        ch = sys.stdin.read(1)
                        if ch == 'a':
                            roll -= roll_d
                            print 'left'
                        elif ch == 'd':
                            roll += roll_d
                            print 'right'
                        elif ch == 'w':
                            print 'forward'
                            pitch += pitch_d
                        elif ch == 's':
                            print 'backward'
                            pitch -= pitch_d
                        elif ch == 'r':
                            print 'up'
                            #thrust += thrust_d
                            PID.ref[1] += yr_d
                        elif ch == 'f':
                            print 'down'
                            #thrust -= thrust_d
                            PID.ref[1] -= yr_d
                        elif ch == 'z':
                            print 'yawrate left'
                            yaw_rate -= yaw_rate_d
                        elif ch == 'c':
                            print 'yawrate right'
                            yaw_rate += yaw_rate_d
                            
                        elif ch == 'u':
                            PID.K += 1000
                        elif ch == 'j':
                            PID.K -= 1000
                        elif ch == 'i':
                            PID.Ki += 1000
                        elif ch == 'k':
                            PID.Ki -= 1000
                        elif ch == 'o':
                            PID.KD += 1000
                        elif ch == 'l':
                            PID.KD -= 1000
                            
                        elif ch == 'q':
                            print 'Exiting PID node and returning to master.'
                            thrust = 0
                            roll = 0
                            pitch = 0
                            yaw_rate = 0
                            PID.status = False
                            PID.landing = False
                            PID.pub_cmd_vel(0, 0, 0, 0)
                            PID.status_master.publish('True')

                        elif ch == 'g' and not PID.landing:
                            print 'Landing'
                            PID.landing = True
                        elif ch == 'g' and PID.landing:
                            print 'Abort landing'
                            PID.landing = False

                        if not ch or ch == chr(4):
                            break
                    else:
                        pass
                        #print 'Nothing'
                        #thrust = 0
                except (KeyboardInterrupt, EOFError):
                    pass
            
            
            # TODO compute PID control signal
            PID.control_pub.publish(200*np.ones((4,1)))
            
            thrust = PID.get_thrust()
            
            if PID.takeoff_frames > 0:
                thrust = PID.takeoff_thrust
                PID.takeoff_frames -= 1
            if PID.takeoff_frames == 0:
                print 'Takeoff finished'
                PID.takeoff_frames = -1

            if pitch > 30:
                pitch = 30
            elif pitch < -30:
                pitch = -30
            if roll > 30:
                roll = 30
            elif roll < -30:
                roll = -30
            if yaw_rate > 200:
                yaw_rate = 200
            elif yaw_rate < -200:
                yaw_rate = -200
            
            print 'Thrust: %.1f, Pitch: %.1f, Roll: %.1f, Yawrate: %.1f, y_ref: %.1f' % (thrust, pitch, roll, yaw_rate, PID.ref[1])
            print 'K: %.1f, Ki: %.1f, Kd: %.1f, e: %.3f' % (PID.K, PID.Ki, PID.KD, PID.e_old)
            PID.error_pub.publish(Point(x=0, y=PID.e_old, z=0))
            PID.pub_cmd_vel(thrust, pitch, roll, yaw_rate)
        else:
            PID.pub_cmd_vel(0, 0, 0, 0)
            PID.I = 0
        PID.r.sleep()

if __name__ == '__main__':
    main()
