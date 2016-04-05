# -*- coding: utf-8 -*-
import numpy as np
import scipy as sp
from math import sin, cos
from scipy.signal import cont2discrete
from copy import copy

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
        xout,yout = self.evaluate_system(self.x, omega)
        self.x = xout
        return xout,yout
        
        
    def evaluate_system(self, x, u):
        phi, theta, psi, phidot, thetadot, psidot = np.reshape(x[6:12,0:1],[1,6])[0]

        Ts = self.Ts
        
        # Local parameters
        g = self.g
        m = self.m
        k = self.k
        A = self.A
        I = self.I
        l = self.l
        b = self.b
        
        u = u[:,0]
        T = k * sum(u ** 2)
    
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
        tau_phi =  l * k * (-u[1] ** 2 + u[3] ** 2)
        tau_theta =  l * k * (-u[0] ** 2 + u[2] ** 2)
        tau_psi =  b * (u[0] ** 2 - u[1] ** 2 +
                        u[2] ** 2 - u[3] ** 2)
        
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
        
        Cc = np.zeros([7,12])
        Cc[0:7,2:9] = np.diag(np.ones([1,7])[0]) # z,rdot,eta
        
        Dc = np.array([])
        
        discreteSystem = cont2discrete((Ac,Bc,Cc,Dc), Ts, method='zoh', alpha=None)
        
        Ad = discreteSystem[0]
        Bd = discreteSystem[1]
        Cd = discreteSystem[2]

        G = np.zeros([12,1])
        G[5,0]=-g
        
        # Sets up control signal
        u = np.concatenate([[[T]],tau_b])
        
        xout = np.dot(Ad,x) + np.dot(Bd,u) + G*Ts
        yout = np.dot(Cd,x)

        return xout, yout
        
class InnerPD(object):
    
    def __init__(self, param):
        self._set_param(param)
        
    def _set_param(self, param):
        # Global parameters
        self.Ts = param['global']['timestep']
        self.g = param['quadcopter_model']['g']
        self.m = param['quadcopter_model']['m']
        self.k = param['quadcopter_model']['k']
        self.A = param['quadcopter_model']['A']
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

        # Create continouous time linearized model
        Ac = np.zeros([12,12])
        Ac[0:3,3:6] = np.diag([1.,1.,1.])
        Ac[3:6,3:6] = -np.diag(self.A)/self.m
        Ac[6:9,9:12] = np.diag([1.,1.,1.])
        
        Bc = np.zeros([12,4])
        Bc[5,:] =self.k
        Bc[9:12,0:4] = np.array([[0., -self.k*self.l/self.Ixx, 0., self.k*self.l/self.Ixx],
                                 [-self.k*self.l/self.Iyy, 0., self.k*self.l/self.Ixx, 0.],
                                 [-self.b/self.Izz, self.b/self.Izz, -self.b/self.Izz, self.b/self.Izz]])
                                 
        Cc = np.zeros([7,12])
        Cc[0:7,2:9] = np.diag(np.ones([1,7])[0])
        
        Dc = np.zeros([7,4])
        
        # Discretize model
        discreteSystem = cont2discrete((Ac,Bc,Cc,Dc), self.Ts, method='zoh', alpha=None)

        # Set discrete matrices
        self.Ad = discreteSystem[0]
        self.Bd = discreteSystem[1]
        self.Cd = discreteSystem[2]
        
    def _set_param(self, param):
        # Global parameters
        self.Ts = param['global']['timestep']
        
        # Local parameters
        self.g = param['quadcopter_model']['g']
        self.m = param['quadcopter_model']['m']
        self.k = param['quadcopter_model']['k']
        self.A = param['quadcopter_model']['A']
        self.Ixx = param['quadcopter_model']['I'][0]
        self.Iyy = param['quadcopter_model']['I'][1]
        self.Izz = param['quadcopter_model']['I'][2]
        self.l = param['quadcopter_model']['l']
        self.b = param['quadcopter_model']['b']
        
        self.P = np.diag(np.ones([1,12])[0]) #param['discrete_kalman_filter']['P0']
        self.Q = np.diag(np.ones([1,12])[0]) #param['discrete_kalman_filter']['Q']
        self.R = np.diag(0.001*np.ones([1,7])[0]) #param['discrete_kalman_filter']['R']
    
    def __call__(self, x, u, z):
        # Predictor step
        xf = np.dot(self.Ad,x) + np.dot(self.Bd,u);
        Pf = np.dot(np.dot(self.Ad, self.P), np.transpose(self.Ad)) + self.Q;
        
        # Corrector step
        Knumerator =  np.dot(Pf, np.transpose(self.Cd))
        Kdenominator = np.dot(np.dot(self.Cd, Pf), np.transpose(self.Cd)) + self.R
        K = np.dot(Knumerator,sp.linalg.inv(Kdenominator))
        xhat = xf + np.dot(K, (z - np.dot(self.Cd, xf)))
        I = np.diag(np.ones([1,12])[0])
        self.P = np.dot((I - np.dot(K, self.Cd)), Pf)
        return xhat

class ExtendedDiscreteKalman(object):
    
    def __init__(self, param):
        self._set_param(param)

        #define quadcopter model
        self.quadCopterModel = QuadcopterProcess(param)
        
    def _set_param(self, param):
        # Global parameters
        self.Ts = param['global']['timestep']
        
        # Local parameters
        self.g = param['quadcopter_model']['g']
        self.m = param['quadcopter_model']['m']
        self.k = param['quadcopter_model']['k']
        self.A = param['quadcopter_model']['A']
        self.Ixx = param['quadcopter_model']['I'][0]
        self.Iyy = param['quadcopter_model']['I'][1]
        self.Izz = param['quadcopter_model']['I'][2]
        self.l = param['quadcopter_model']['l']
        self.b = param['quadcopter_model']['b']
        
        self.P = np.diag(np.ones([1,12])[0]) #param['discrete_kalman_filter']['P0']
        self.Q = np.diag(np.ones([1,12])[0]) #param['discrete_kalman_filter']['Q']
        self.R = np.diag(0.001*np.ones([1,7])[0]) #param['discrete_kalman_filter']['R']
    
        Cc = np.zeros([7,12])
        Cc[0:7,2:9] = np.diag(np.ones([1,7])[0])
        self.Cd = Cc

    def __call__(self, x, u, z):
        # Predictor step

        J = self.compute_jacobian(x, 0*u)
        xf, yf = self.quadCopterModel.evaluate_system(x, u)
        Pf = np.dot(np.dot(J, self.P), np.transpose(J)) + self.Q;
        
        # Corrector step
        Knumerator =  np.dot(Pf, np.transpose(self.Cd))
        Kdenominator = np.dot(np.dot(self.Cd, Pf), np.transpose(self.Cd)) + self.R
        K = np.dot(Knumerator,sp.linalg.inv(Kdenominator))
        xhat = xf + np.dot(K, (z - yf))
        I = np.diag(np.ones([1,12])[0])
        self.P = np.dot((I - np.dot(K, self.Cd)), Pf)
        return xhat
    
    def compute_jacobian(self, x, u):
        deltax = 0.0001
        J = np.zeros([len(x),len(x)])
        for ii in range(len(x)):
            fm, _ = self.quadCopterModel.evaluate_system(self.shift(x,ii,-deltax),u)
            f, _ = self.quadCopterModel.evaluate_system(x,u)
            fp, _ = self.quadCopterModel.evaluate_system(self.shift(x,ii,deltax),u)
            J[0:12,ii:ii+1] = (fm - 2*f + fp)/deltax
        return J
    
    def shift(self, vector, index, delta):
        output = copy(vector)
        output[index,0] += delta
        return output

class UnscentedDiscreteKalman(object):
    
    def __init__(self, param):
        self._set_param(param)
        
        #define quadcopter model
        self.quadCopterModel = QuadcopterProcess(param)
        
        # UKF variables
        self.L = 12        # Dimension of the ranodm variable x
        self.alpha = 1e-3  # Scaling parameter
        self.beta = 2      # Optimal for gaussian distibutions
        self.keta = 0      # Scaling parameter
        self.lam = self.alpha ** 2 * (self.L + self.keta) - self.L
        
        # UKF weights
        self.Wom = 1./(self.L + self.lam)
        self.Woc = 1./(self.L + self.lam) + (1 - self.alpha ** 2 + self.beta)
        self.Wim = 1./(2 * (self.L + self.lam))
        
        # Initial parameters
        self.xest = np.zeros([12,1])
        self.covariance = np.diag(np.ones(12))
        
    def _set_param(self, param):
        # Global parameters
        self.Ts = param['global']['timestep']
        
        # Local parameters
        self.g = param['quadcopter_model']['g']
        self.m = param['quadcopter_model']['m']
        self.k = param['quadcopter_model']['k']
        self.A = param['quadcopter_model']['A']
        self.Ixx = param['quadcopter_model']['I'][0]
        self.Iyy = param['quadcopter_model']['I'][1]
        self.Izz = param['quadcopter_model']['I'][2]
        self.l = param['quadcopter_model']['l']
        self.b = param['quadcopter_model']['b']
        
        self.P = np.diag(np.ones([1,12])[0]) #param['discrete_kalman_filter']['P0']
        self.Q = np.diag(np.ones([1,12])[0]) #param['discrete_kalman_filter']['Q']
        self.R = np.diag(0.001*np.ones([1,7])[0]) #param['discrete_kalman_filter']['R']
        
        Cc = np.zeros([7,12])
        Cc[0:7,2:9] = np.diag(np.ones([1,7])[0])
        self.Cd = Cc
    
    def __call__(self, x, u, z):
        # Calculate sigma points
        ssCov = sp.linalg.sqrtm((self.lam + self.L) * self.covariance)
        Xi = np.zeros([self.L,2 * self.L + 1])
        for ii in range(2 * self.L + 1):
            Xi[:,ii] = np.transpose(x)[0]
            if ii > 0 and ii <= self.L:
                Xi[:,ii] += ssCov[:, ii - 1]
            elif  ii > self.L:
                Xi[:,ii] += ssCov[:, ii - (1 + self. L)]
        
        # Time update
        
        # Measurement update
        return x
