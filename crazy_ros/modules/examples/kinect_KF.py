# -*- coding: utf-8 -*-

import numpy as np
from math import cos, pi
import matplotlib.pylab as plt
import os,sys,inspect

# Loads crazylib from the parent directory
curdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
pardir = os.path.dirname(curdir)
sys.path.insert(0,pardir) 
from crazylib import discrete_KF_update

####################################################################################
# This example shows how the KF is implemented and used in the kinect node by
# simulating a tripple integrator over 10 seconds, estimating the position, velocity 
# and acceleration of based on noisy positional measuements in [0,3] with a
# standard deviaiton of 0.1 and very messy accelerometer data in [0,1] with a 
# standard deviation of 0.5. The accelerometer data is not delayed here, as the
# regular kalman filter performs exceptionally bad for asynchronous data. This
# example merely shows how the kalman filter is used with the purpose of estimate  
# the positions and velocities and states as good as possible not knowing velocities
# (see plots).
####################################################################################

# Time constants
Ts = 0.01
Nt = 1000
tt = np.linspace(0,(Nt-1)*Ts,Nt)

# Dynamics
Nstates = 3
A = np.eye(3) + np.diag([Ts,Ts], k = 1) + np.diag([Ts**2/2.], k = 2)
B = []
C = np.array([[1.,0.,0.],[0.,0.,1.]])

# Filter parameters
P = np.eye(Nstates,Nstates)
Q = 0.01*np.eye(Nstates,Nstates)
std_R = np.array([[.1],[.5]])
R = np.diag(np.transpose(std_R)[0])

# Empty vectors for solution history
states = np.zeros((Nstates,Nt))
states[:,0] = [0.,0.,1.] # Initial conditions
estimates = np.zeros((Nstates,Nt))
estimates[:,0] = [0.,0.,1.] # Initial conditions
measurements = np.zeros((2,Nt))

for ii in range(1,len(tt)):
    
    # Replace with measurements
    states[2,ii] = cos(4*pi*tt[ii]/(Nt*Ts))
    states[1,ii] = states[1,ii-1] + (states[2,ii] + states[2,ii - 1])*Ts/2
    states[0,ii] = states[0,ii-1] + (states[1,ii] + states[1,ii - 1])*Ts/2
    
    # Find input variables
    vk = np.dot(np.random.randn(1,2),R)[0]
    zk = np.array([states[0,ii],states[2,ii]]) + vk   
    xm1 = estimates[:,ii-1]

    xhat, P = discrete_KF_update(xm1,[],zk,A,[],C,P,Q,R)

    estimates[:,ii] = xhat
    measurements[:,ii] = zk

# Plot results
plt.figure(1)
plt.subplot(211)
plt.title('The true and correpted measurements\nof states used in the estimation')
plt.plot(tt,measurements[0,:],'b')
plt.plot(tt,states[0,:],'r')
plt.legend(['True x','Measured x'])
plt.subplot(212)
plt.plot(tt,measurements[1,:],'b')
plt.plot(tt,states[2,:],'r')
plt.legend(['True ddx','Measured ddx'])

plt.figure(2)
plt.subplot(311)
plt.title('Estimated (xhat) and true states (E[x])')
plt.plot(tt,estimates[0,:],'b')
plt.plot(tt,states[0,:],'r')
plt.legend(['xhat','E[x]'])
plt.subplot(312)
plt.plot(tt,estimates[1,:],'b')
plt.legend(['dxhat'])
plt.subplot(313)
plt.plot(tt,estimates[2,:],'b')
plt.plot(tt,states[2,:],'r')
plt.legend(['ddxhat','E[ddx]'])
plt.show()
