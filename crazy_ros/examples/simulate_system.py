# -*- coding: utf-8 -*-
import numpy as np
import scipy as sp
from math import sqrt, sin, cos, pi
from scipy.signal import cont2discrete
import matplotlib.pylab as plt

param = {'global': {'timestep': 0.01},
 'quadcopter_model': {'A': [0.25, 0.25, 0.25],
  'I': [0.004856, 0.004856, 0.008801],
  'Im': 3.357e-05,
  'b': 1.14e-07,
  'g': 9.81,
  'k': 2.98e-06,
  'l': 0.225,
  'm': 0.468,
  'x_init': [[0.],[0.],[0.],[0.],[0.],[0.],[0.],[0.],[0.],[0.],[0.],[0.]]}}

# Local parameters
Ts = param['global']['timestep']

# Local parameters
g = param['quadcopter_model']['g']
m = param['quadcopter_model']['m']
k = param['quadcopter_model']['k']
A = param['quadcopter_model']['A']
I = param['quadcopter_model']['I']
l = param['quadcopter_model']['l']
b = param['quadcopter_model']['b']

simulationTime = 2. # s
numberOfTimesteps = int(np.ceil(2/Ts))

# Generate command
timevec = np.zeros([1,numberOfTimesteps])
omegavec = np.zeros([4,numberOfTimesteps])
measurements = np.zeros([6,numberOfTimesteps])
states = np.zeros([12,numberOfTimesteps])

# Computes the angular velocity required to hover
hover_omega = sqrt(g*m/(4*k));
omegaAmp = 25.
x = np.array(param['quadcopter_model']['x_init'])

for ii in range(numberOfTimesteps):
    t = ii*Ts
    timevec[0,ii] = t
    
    # Declares reference signal object for generating omega
    hover_omega = sqrt(g*m/(4*k));
    omegaAmp = 25.
    omegaOscillation = np.ones([4,1])
    if t<=0.5:
        omegaOscillation *= 2 * sin(t*4*pi)
    elif 0.5<t and t<=1:
        omegaOscillation[0] = 0
        omegaOscillation[1] *= -sin(t*4*pi)
        omegaOscillation[2] = 0
        omegaOscillation[3] *= sin(t*4*pi)
    elif 1<t and t<=1.5:
        omegaOscillation[0] *= -sin(t*4*pi)
        omegaOscillation[1] = 0
        omegaOscillation[2] *= sin(t*4*pi)
        omegaOscillation[3] = 0
    else:
        omegaOscillation[0] *= -sin(t*4*pi)
        omegaOscillation[1] *= sin(t*4*pi)
        omegaOscillation[2] *= -sin(t*4*pi)
        omegaOscillation[3] *= sin(t*4*pi)
    
    omega = hover_omega*np.ones([4,1]) + omegaAmp*omegaOscillation
    
    # Adds omega to the solution history
    omegavec[0:4,ii:ii+1] = omega
    omega = np.reshape(omega,[1,4])[0]
    
    
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
    Dd = discreteSystem[3]
    G = np.zeros([12,1])
    G[5,0]=-g
    
    # Sets up control signal
    u = np.concatenate([[[T]],tau_b])
    
    xout = np.dot(Ad,x) + np.dot(Bd,u) + G*Ts
    yout = np.dot(Cd,x)
    
    states[0:12,ii:ii+1] = xout
    measurements[0:6,ii:ii+1] = yout
    x = xout

# Visualize simulation
if 1: # Omega
    plt.figure(1)
    plt.step(np.transpose(timevec),np.transpose(omegavec))
    plt.legend(['$\omega_1$','$\omega_2$','$\omega_3$','$\omega_4$'])
    plt.xlabel('Time, [s]')
    plt.ylabel('Angular velocity, [rad/s]')

if 1: # States
    plt.figure(1)
    f, axarr = plt.subplots(2, 2)
    axarr[0,0].step(np.transpose(timevec),np.transpose(states[0:3,:]))
    axarr[0,0].set_xlabel('Time, [s]')
    axarr[0,0].set_ylabel('$\mathbf{r}(t)$')
    axarr[0,0].legend(['x','y','z'])

    axarr[0,1].step(np.transpose(timevec),np.transpose(states[3:6,:]))
    axarr[0,1].set_xlabel('Time, [s]')
    axarr[0,1].set_ylabel('$\dot{\mathbf{r}}(t)$')
    axarr[0,1].legend(['$\dot{x}$','$\dot{y}$','$\dot{z}$'])

    axarr[1,0].step(np.transpose(timevec),np.transpose(states[6:9,:]))
    axarr[1,0].set_xlabel('Time, [s]')
    axarr[1,0].set_ylabel('${\mathbf{\eta}}(t)$')
    axarr[1,0].legend(['$\phi$','$\Theta$','$\psi$'])

    axarr[1,1].step(np.transpose(timevec),np.transpose(states[9:12,:]))
    axarr[1,1].set_xlabel('Time, [s]')
    axarr[1,1].set_ylabel('$\dot{\mathbf{\eta}}(t)$')
    axarr[1,1].legend(['$\dot{\phi}$','$\dot{\Theta}}$','$\dot{\psi}$'])
    
plt.show()
            
            
            
