# -*- coding: utf-8 -*-
import numpy as np
from math import sqrt, sin, pi
import matplotlib.pylab as plt
from json import load
from quad_classes import QuadcopterProcess, InnerPD

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

# Declare a quadcopter process object
qProcess = QuadcopterProcess(param)
innerPD = InnerPD(param)

# Initial state
x = np.array(param['quadcopter_model']['x_init'])

################ Simulation ################

for ii in range(numberOfTimesteps):

    # Generates reference-trajectory
    zd = 1.
    phid = 0.05
    thetad = 0.04
    psid = 0.
    pref = np.array([[zd],[phid],[thetad],[psid]]) # positional reference
    dref = np.array([[0.],[0.],[0.],[0.]])         # velocity reference

    # Generates omega in inner PD controller
    omega = innerPD(pref, dref, x)

    # Gets system response from the non-linear model 
    x, y = qProcess(omega)

    timevec[0,ii] = ii*Ts
    states[0:12,ii:ii+1] = x
    measurements[0:6,ii:ii+1] = y

################ Visualize simulation ################
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
            
            
            
