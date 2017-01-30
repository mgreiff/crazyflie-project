# -*- coding: utf-8 -*-

import numpy as np
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
Ts = 0.03
Nt = 1000
tt = np.linspace(0,(Nt-1)*Ts,Nt)

# Dynamics
Nstates = 6
A = np.eye(6) + np.diag([Ts,Ts,Ts], k = 3)
B = []
C = np.zeros((3,6))
C[0,0] = 1.
C[1,1] = 1.   
C[2,2] = 1.

# Filter parameters
P = np.eye(Nstates,Nstates)
Q = 0.01*np.eye(Nstates,Nstates)
std_R = np.array([[.1],[.1],[.1]])
R = np.diag(np.transpose(std_R)[0])

# Empty vectors for solution history
states = np.zeros((Nstates,Nt))
states[:,0] = [0.,0.,0.,0.,0.,0.] # Initial conditions
estimates = np.zeros((Nstates,Nt))
estimates[:,0] = [0.,0.,0.,0.,0.,0.] # Initial conditions
measurements = np.zeros((3,Nt))

print A
print B
print C
print '--'*10
print Q
print R
print P
print '--'*10

for ii in range(1,len(tt)):

    
    states[0,ii-1] = tt[ii]
    states[1,ii-1] = tt[ii]
    states[2,ii-1] = tt[ii]

    # Find input variables
    vk = np.dot(np.random.randn(1,3),R)[0]
    zk = states[0:3,ii-1] + vk   
    xm1 = estimates[:,ii-1]

    xhat, P = discrete_KF_update(xm1,[],zk,A,[],C,P,Q,R)

    estimates[:,ii] = xhat
    measurements[:,ii] = zk

print P

# Plot results
plt.figure(1)
plt.title('The true and correpted measurements\nof states used in the estimation')
plt.plot(tt,measurements[0,:],'b')
plt.plot(tt,states[0,:],'r')
plt.legend(['True ddx','Measured ddx'])

plt.figure(2)
plt.subplot(311)
plt.title('Estimated (xhat) and true states (E[x])')
plt.plot(tt,estimates[0,:],'b')
plt.plot(tt,states[0,:],'r')
plt.legend(['dxhat','E[x]'])
plt.subplot(312)
plt.plot(tt,estimates[3,:],'b')
plt.plot(tt,states[3,:],'r')
plt.legend(['ddxhat','E[ddx]'])
plt.show()
