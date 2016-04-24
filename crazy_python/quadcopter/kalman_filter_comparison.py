# -*- coding: utf-8 -*-
import numpy as np
from math import sqrt, sin, pi
import matplotlib.pylab as plt
from json import load
from quad_classes import QuadcopterProcess, DiscreteKalman, ExtendedDiscreteKalman

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
measurements = np.zeros([7,numberOfTimesteps])
states = np.zeros([12,numberOfTimesteps])
estimatesKalman = np.zeros([12,numberOfTimesteps])
estimatesExtended = np.zeros([12,numberOfTimesteps])

# Desgn parameter for omega referene example
omegaAmp = 25.

# Declare a quadcopter process object
qProcess = QuadcopterProcess(param)

# Declare kalman filter
x = np.array(param['quadcopter_model']['x_init']) # Initial state
kalmanFilter = DiscreteKalman(param)
extendedKalmanFilter = ExtendedDiscreteKalman(param)

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
    
    # Simulates the process
    xnew, y = qProcess(omega)
    
    # Adds measurement noise to the measured states
    variance = 0.001
    y += np.random.normal(0,sqrt(variance),[len(y),1])

    xhatKalman = kalmanFilter(x, omega, y)
    xhatExtended = extendedKalmanFilter(x, omega, y)
    
    x = xnew
    omegavec[0:4,ii:ii+1] = omega
    states[0:12,ii:ii+1] = x
    measurements[0:7,ii:ii+1] = y
    estimatesKalman[0:12,ii:ii+1] = xhatKalman
    estimatesExtended[0:12,ii:ii+1] = xhatExtended


# Visualize simulation

if 1: # States
    plt.figure(1)
    plt.plot(np.transpose(timevec),np.transpose(estimatesKalman[2,:]))
    plt.plot(np.transpose(timevec),np.transpose(estimatesExtended[2,:]))
    plt.step(np.transpose(timevec),np.transpose(states[2,:]))
    plt.xlabel('Time, [s]')
    plt.ylabel('$\mathbf{r}(t)$')
    plt.legend(['x','y','z'],loc=1,fontsize=6)

    plt.figure(2)
    plt.plot(np.transpose(timevec),np.transpose(estimatesKalman[6,:]))
    plt.plot(np.transpose(timevec),np.transpose(estimatesExtended[6,:]))
    plt.step(np.transpose(timevec),np.transpose(states[6,:]))
    plt.xlabel('Time, [s]')
    plt.ylabel('${\mathbf{\eta}}(t)$')
    plt.legend(['$\phi$','$\Theta$','$\psi$'],loc=1,fontsize=6)

plt.show()
            
            
            
