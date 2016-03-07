# -*- coding: utf-8 -*-
import numpy as np
from math import sqrt, sin, pi
import matplotlib.pylab as plt
from json import load
from quad_classes import QuadcopterProcess

# Load parameters from configuration file
with open('configparam.cnf') as configfile:
    param = load(configfile)
configfile.close()
                
# Simulation parameters
simulationTime = 2. # s
Ts = param['global']['timestep']
numberOfTimesteps = int(np.ceil(2/Ts))

# Sorage variables for plotting purposes
timevec = np.zeros([1,numberOfTimesteps])
omegavec = np.zeros([4,numberOfTimesteps])
measurements = np.zeros([6,numberOfTimesteps])
states = np.zeros([12,numberOfTimesteps])

# Desgn parameter for omega referene example
omegaAmp = 25.

# Declare a quadcopter process object
qProcess = QuadcopterProcess(param)

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
    
    # Adds omega to the solution history
    
    # Simulates the process
    xout, yout = qProcess(omega)

    omegavec[0:4,ii:ii+1] = omega
    states[0:12,ii:ii+1] = xout
    measurements[0:6,ii:ii+1] = yout


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
            
            
            
