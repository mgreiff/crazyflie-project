# -*- coding: utf-8 -*-
import numpy as np
import scipy as sp
from math import sin, cos
from scipy.signal import cont2discrete

class QuadcopterProcess(object):
    
    def __init__(self, param):
        self._set_param(param)
    
    def _set_param(self, param):
        # Global parameters
        self.Ts = param['global']['timestep']
        
        # Local parameters
        self.g = param['quadcopter_model']['g']
        self.m = param['quadcopter_model']['m']
        self.k = param['quadcopter_model']['k']
        self.A = param['quadcopter_model']['A']
        self.I = param['quadcopter_model']['I']
        self.l = param['quadcopter_model']['l']
        self.b = param['quadcopter_model']['b']
        
        # initial condition
        self.x = np.array(param['quadcopter_model']['x_init'])
        
    def __call__(self, omega):
        phi, theta, psi, phidot, thetadot, psidot = np.reshape(self.x[6:12,0:1],[1,6])[0]

        Ts = self.Ts
        
        # Local parameters
        g = self.g
        m = self.m
        k = self.k
        A = self.A
        I = self.I
        l = self.l
        b = self.b
        
        omega = omega[:,0]
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
        
        # eq. (8) Modified from the work of lukkonen, here rotor 2 and 4 spin in
        # the opposite directions
        tau_phi =  l * k * (-omega[1] ** 2 + omega[3] ** 2)
        tau_theta =  l * k * (-omega[0] ** 2 + omega[2] ** 2)
        tau_psi =  b * (omega[0] ** 2 - omega[1] ** 2 +
                        omega[2] ** 2 - omega[3] ** 2)
        
        tau_b = np.array([[tau_phi],[tau_theta],[tau_psi]])
        
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
        
        invJ = sp.linalg.inv(J)
    
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

        G = np.zeros([12,1])
        G[5,0]=-g
        
        # Sets up control signal
        u = np.concatenate([[[T]],tau_b])
        
        xout = np.dot(Ad,self.x) + np.dot(Bd,u) + G*Ts
        yout = np.dot(Cd,self.x)
        
        self.x = xout

        return self.x, yout
        
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
        self.Kp = np.array(param['inner_PD']['Kp'])
        self.Kd = np.array(param['inner_PD']['Kd'])
    
    def __call__(self, ref, dref, x):
        pstates = x[[2,6,7,8],:]    #z, phi, theta, psi 
        dstates = x[[5,9,10,11],:]
        perror = ref - pstates
        derror = dref - dstates
        
        Dpart = self.Kd * derror
        Ppart = self.Kp * perror
        Gacceleration = np.array([[self.g],[0],[0],[0]])

        phi = x[6,0]
        theta = x[7,0]
        
        InertiaTransform = np.diag([self.m/(cos(phi)*cos(theta)),self.Ixx,self.Iyy,self.Izz])
        
        Tvec = np.dot(InertiaTransform, Gacceleration + Dpart + Ppart)
        
        Tvec2omegaMap = np.array([[1./(4*self.k), 0., -1./(2*self.k*self.l), -1/(4.*self.b)],
                                  [1./(4*self.k), -1./(2*self.k*self.l), 0., 1/(4.*self.b)],
                                  [1./(4*self.k), 0.,  1./(2*self.k*self.l), -1/(4.*self.b)],
                                  [1./(4*self.k),  1./(2*self.k*self.l), 0., 1/(4.*self.b)]])
        omegasquared = np.dot(Tvec2omegaMap, Tvec)
        omega = np.sqrt(omegasquared)
        return omega

class DiscreteKalman(object):
    
    def __init__(self, param):
        self._set_param(param)

    def _set_param(self, param):
        # Global parameters
        self.Ts = param['global']['timestep']
    
    def __call__(self, x, u, z):
        # TODO write complete kalman filter
        xhat = phid
        return xhat
        