#!/usr/bin/env python

from parameterLoader import ParameterLoader
from crazy_ros.msg import NumpyArrayFloat64
from math import sin, cos, sqrt
import rospy
import numpy as np


class InnerPDController(ParameterLoader):

    def __init__(self):
        # Sets up subscribers
        self.measured_states_pub = rospy.Subscriber('measured_states',
                                              NumpyArrayFloat64,
                                              self.handle_measured_data)

        # Sets up publishers
        self.omega_pub = rospy.Publisher('reference_signal',
                                                   NumpyArrayFloat64,
                                                   queue_size = 10)

        self.load_parameters()
        self.inner = InnerPD(self.param)
        self.ref = np.array([5.0, 0.0, 0.0, 0.0])

    def handle_measured_data(self, msg):
        x = np.array(msg.data)
        omega = self.inner(self.ref, np.array([0.0, 0.0, 0.0, 0.0]), x)
        #print omega
        self.omega_pub.publish(list(omega))


class InnerPD(object):

    def __init__(self, param):
        self._set_param(param)

    def _set_param(self, param):
        # Global parameters
        self.Ts = param['global']['timestep']
        self.g = param['quadcopter_model']['g']
        self.m = param['quadcopter_model']['m']
        self.k = param['quadcopter_model']['k']
        self.Ixx = param['quadcopter_model']['I'][0]
        self.Iyy = param['quadcopter_model']['I'][1]
        self.Izz = param['quadcopter_model']['I'][2]
        self.l = param['quadcopter_model']['l']
        self.b = param['quadcopter_model']['b']
        self.Kp = np.transpose(np.array(param['inner_PD']['Kp']))
        self.Kd = np.transpose(np.array(param['inner_PD']['Kd']))

    def __call__(self, ref, dref, x):
        pstates = x[[2,6,7,8]]    #z, phi, theta, psi
        dstates = x[[5,9,10,11]]
        perror = ref - pstates
        derror = dref - dstates

        Dpart = self.Kd * derror
        Ppart = self.Kp * perror
        Gacceleration = np.array([self.g,0,0,0])

        phi = x[6]
        theta = x[7]

        InertiaTransform = np.diag([self.m/(cos(phi)*cos(theta)),self.Ixx,self.Iyy,self.Izz])

        Tvec = np.dot(InertiaTransform, Gacceleration + Dpart[0] + Ppart[0])

        Tvec2omegaMap = np.array([[1./(4*self.k), 0., -1./(2*self.k*self.l), -1/(4.*self.b)],
                                  [1./(4*self.k), -1./(2*self.k*self.l), 0., 1/(4.*self.b)],
                                  [1./(4*self.k), 0.,  1./(2*self.k*self.l), -1/(4.*self.b)],
                                  [1./(4*self.k),  1./(2*self.k*self.l), 0., 1/(4.*self.b)]])
        omegasquared = np.dot(Tvec2omegaMap, Tvec)
        """print phi, theta
        print "osq:
        print omegasquared
        print Gacceleration, Dpart, Ppart"""
        omegasquared[omegasquared<0] = 0
        omega = np.sqrt(omegasquared)
        """print 's', perror, derror, omega
        print self.Kd
        print Tvec
        print omegasquared
        print omega"""
        return omega

def main():
    rospy.init_node('innerPDController')
    inner = InnerPDController()
    rospy.spin()

if __name__ == '__main__':
    main()
