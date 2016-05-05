# -*- coding: utf-8 -*-
import numpy as np
from math import sqrt, sin, pi
import matplotlib.pylab as plt
import os,sys,inspect
from json import load

# Sets up paths and imports crazylib
curdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
modulesdir = os.path.dirname(curdir)
configdir = os.path.join(os.path.dirname(modulesdir),'config')
sys.path.insert(0,modulesdir)

from crazylib import quadcopter_dynamics, discrete_UKF_update

# Load parameters from configuration file in /config/*
with open(os.path.join(configdir,'configparam.cnf')) as configfile:
    param = load(configfile)
configfile.close()

# Simulation parameters
simulationTime = 2. # s
Ts = param['global']['inner_loop_h']
numberOfTimesteps = int(np.ceil(2/Ts))

# Sorage variables for plotting purposes
timevec = np.zeros([1,numberOfTimesteps])
omegavec = np.zeros([4,numberOfTimesteps])
measurements = np.zeros([7,numberOfTimesteps])
states = np.zeros([12,numberOfTimesteps])

# Desgn parameter for omega referene example
omegaAmp = 25.

for ii in range(numberOfTimesteps):
    t = ii*Ts
    timevec[0,ii] = t
    
    # Generates omega-trajectory
    g = param['quadcopter_model']['g']
    m = param['quadcopter_model']['m']
    k = param['quadcopter_model']['k']
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
    if ii == 0:
        x = np.array(param['quadcopter_model']['x0'])
    else:
        x = states[0:12,ii-1:ii]

    # Simulates the process in discrete time
    xout, yout = quadcopter_dynamics(x, omega, param)
    
    # Performs a UKF update
    P = np.eye(12)
    Q = np.eye(12)
    R = np.eye(12)
    z = np.random.rand(12,1)
    x = np.random.rand(12,1)
    
    # Example of how to implement the measurement function
    H = lambda x: x[[0,1,2,3,4,5,6,7,8,9,10,11]]
    x, P = discrete_UKF_update(x, omega, z, quadcopter_dynamics, H, param, P, Q, R)
    
    omegavec[0:4,ii:ii+1] = omega
    states[0:12,ii:ii+1] = xout
    measurements[0:7,ii:ii+1] = yout


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
    axarr[0,0].legend(['x','y','z'],loc=3,fontsize=6)
    
    axarr[0,1].step(np.transpose(timevec),np.transpose(states[3:6,:]))
    axarr[0,1].set_xlabel('Time, [s]')
    axarr[0,1].set_ylabel('$\dot{\mathbf{r}}(t)$')
    axarr[0,1].legend(['$\dot{x}$','$\dot{y}$','$\dot{z}$'],loc=3,fontsize=6)
    
    axarr[1,0].step(np.transpose(timevec),np.transpose(states[6:9,:]))
    axarr[1,0].set_xlabel('Time, [s]')
    axarr[1,0].set_ylabel('${\mathbf{\eta}}(t)$')
    axarr[1,0].legend(['$\phi$','$\Theta$','$\psi$'],loc=2,fontsize=6)
    
    axarr[1,1].step(np.transpose(timevec),np.transpose(states[9:12,:]))
    axarr[1,1].set_xlabel('Time, [s]')
    axarr[1,1].set_ylabel('$\dot{\mathbf{\eta}}(t)$')
    axarr[1,1].legend(['$\dot{\phi}$','$\dot{\theta}$','$\dot{\psi}$'],loc=2,fontsize=6)

plt.show()



