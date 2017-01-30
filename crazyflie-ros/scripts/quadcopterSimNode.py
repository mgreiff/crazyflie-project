#!/usr/bin/env python

from std_msgs.msg import String, Float64, Header
from crazy_ros.msg import NumpyArrayFloat64
from geometry_msgs.msg import Twist, PointStamped
import numpy as np
import scipy.linalg as spl
from scipy.signal import cont2discrete
from math import sin, cos, sqrt
import rospy
import os
import sys
from json import dumps, load
import time
import rospy

class QuadcopterModel(object):

    def __init__(self):

        self.reference_sub = rospy.Subscriber('/crazyflie/cmd_vel',
                                              Twist,
                                              self.handle_reference_data)
        self.measured_states_pub = rospy.Publisher('measured_states',
                                                   NumpyArrayFloat64,
                                                   queue_size = 10)
        # Visualisation topics
        self.viz_pos_pub = rospy.Publisher('/system/viz_sim_pos',
                                            PointStamped,
                                            queue_size = 10)
        
        params = None
        # Loads configuration parameters
        for filename in rospy.myargv(argv=sys.argv):
            if os.path.basename(filename) == 'configparam.cnf':
                with open(filename) as configfile:
                    param = load(configfile)
                configfile.close()
        if param != None:
            # Sets configuration parameters
            self.timestep = param['global']['inner_loop_h']
            self.x_init = np.array(param['quadcopter_model']['x0'])
            self.x = self.x_init

            self.g = param['quadcopter_model']['g']
            self.m = param['quadcopter_model']['m']
            self.k = param['quadcopter_model']['k']
            self.A = param['quadcopter_model']['A']
            self.I = param['quadcopter_model']['I']
            self.l = param['quadcopter_model']['l']
            self.b = param['quadcopter_model']['b']
        else:
            errmsg = ('ERROR. Could not find parameters in %s...\n, shutting down node'%(str(self)))
            raise Exception(errmsg)

    def handle_reference_data(self, msg):
        # TODO Include stabiising controller and/or PWM mappings
        T = msg.linear.z            # Thrust [N]
        tau_phi =  msg.angular.x    # Pitch  [degrees].
        tau_theta =  msg.angular.y  # Roll   [degrees].
        tau_psi =  msg.angular.z    # Yaw    [degrees].
        tau_b = np.array([[tau_phi],[tau_theta],[tau_psi]])
        
        x = self.x
        Ts = self.timestep
        g = self.g
        k = self.k
        A = self.A
        I = self.I
        l = self.l
        b = self.b
        m = self.m

        # Equations from Teppo Luukkonen, Modelling and control of quadcopter
        # Extracs the euler angle states and positional derivatives
        phi, theta, psi, phidot, thetadot, psidot = np.reshape(x[6:12,0:1],[1,6])[0]
        
        # eq. (7)
        T = k * sum(omega ** 2)

        # Define the C-matrix, see eq. (19)
        Ixx, Iyy, Izz = I
        
        Sphi = sin(phi)
        Stheta = sin(theta)
        Spsi = sin(psi);
        Cphi = cos(phi)
        Ctheta = cos(theta)
        Cpsi = cos(psi)
        
        C11 = 0.
        C12 = ((Iyy - Izz) * (thetadot * Cphi * Sphi +
               psidot * Sphi ** 2 *Ctheta) +
               (Izz - Iyy) * psidot * Cphi ** 2 *Ctheta -
               Ixx * psidot * Ctheta)
        C13 = (Izz - Iyy) * psidot * Cphi * Sphi * Ctheta ** 2
        C21 = ((Izz - Iyy) * (thetadot * Cphi * Sphi +
               psidot * Sphi * Ctheta) +
               (Iyy - Izz) * psidot * Cphi ** 2 * Ctheta -
               Ixx * psidot * Ctheta)
        C22 = (Izz - Iyy) * phidot * Cphi * Sphi
        C23 = (-Ixx * psidot * Stheta * Ctheta +
               Iyy * psidot * Sphi ** 2 * Stheta * Ctheta +
               Izz * psidot * Cphi ** 2 * Stheta * Ctheta)
        C31 = ((Iyy - Izz) * psidot * Ctheta ** 2 * Sphi * Cphi -
               Ixx * thetadot * Ctheta)
        C32 = ((Izz - Iyy) * (thetadot * Cphi * Sphi * Stheta +
               phidot * Sphi ** 2 * Ctheta) +
               (Iyy - Izz) * phidot * Cphi ** 2 * Ctheta +
               Ixx * psidot * Stheta * Ctheta -
               Iyy * psidot * Sphi ** 2 * Stheta * Ctheta -
               Izz * psi * Cphi ** 2 * Stheta * Ctheta)
        C33 = ((Iyy - Izz) * phidot * Cphi * Sphi * Ctheta ** 2 -
               Iyy * thetadot * Sphi ** 2 * Ctheta * Stheta -
               Izz * thetadot * Cphi ** 2 * Ctheta * Stheta +
               Ixx * thetadot * Ctheta * Stheta)
        
        C = np.array([[C11,C12,C13],
                      [C21,C22,C23],
                      [C31,C32,C33]]);   
        
        # eq. (16)
        J11 = Ixx
        J12 = 0.
        J13 = -Ixx*Stheta
        J21 = 0.
        J22 = Iyy * Cphi ** 2 + Izz * Sphi ** 2
        J23 = (Iyy - Izz) * Cphi * Sphi * Ctheta
        J31 = -Ixx * Stheta
        J32 = (Iyy - Izz) * Cphi * Sphi * Ctheta
        J33 = (Ixx * Stheta ** 2 + Iyy * Sphi ** 2 * Ctheta ** 2 +
               Izz * Cphi ** 2 * Ctheta ** 2)
        J = np.array([[J11,J12,J13],
                      [J21,J22,J23],
                      [J31,J32,J33]])
        
        invJ = spl.inv(J)

        # Sets up continuous system
        I3 = np.diag(np.ones([1,3])[0])
        
        Ac = np.zeros([12,12])
        Ac[0:3,3:6] = I3
        Ac[3:6,3:6] = -(1/m)*np.diag(A)
        Ac[6:9,9:12] = I3
        Ac[9:12,9:12] = -np.dot(invJ,C)
        
        Rz = (1/m) * np.array([[cos(psi) * sin(theta) * cos(phi) + sin(psi) * sin(phi)],
                               [sin(psi) * sin(theta) * cos(phi) - cos(psi) * sin(phi)],
                               [cos(theta) * cos(phi)]])
        
        Bc = np.zeros([12,4])
        Bc[3:6,0:1] = Rz
        Bc[9:12,1:4] = invJ
        
        Cc = np.zeros([6,12])
        Cc[0,0] = 1. # x
        Cc[1,1] = 1. # y
        Cc[2,2] = 1. # z
        Cc[3,6] = 1. # phi
        Cc[4,7] = 1. # theta
        Cc[5,8] = 1. # psi
        
        Dc = np.array([])
        
        discreteSystem = cont2discrete((Ac,Bc,Cc,Dc), Ts, method='zoh', alpha=None)
        
        Ad = discreteSystem[0]
        Bd = discreteSystem[1]
        Cd = discreteSystem[2]
        Dd = discreteSystem[3]
        G = np.zeros([12,1])
        G[5,0]=-g
        
        # Sets up control signal
        u = np.concatenate([[[T]],tau_b])
        
        xout = np.dot(Ad,x) + np.dot(Bd,u) + G*Ts
        yout = np.dot(Cd,x)
        
        self.x = xout
        self.measured_states_pub.publish(xout)

        my_header = Header(stamp=rospy.Time.now(), frame_id="map")
        my_point = Point(xout[0,0], xout[1,0], xout[2,0])
        p = PointStamped(header=my_header, point=my_point)
        print p
        print '----'
        self.viz_pos_pub.publish(p)

    def __str__(self):
        return 'quadcopter model node'

def main():
    rospy.init_node('QuadcopterModel')
    quad = QuadcopterModel()
    rospy.spin()

if __name__ == '__main__':
    main()
