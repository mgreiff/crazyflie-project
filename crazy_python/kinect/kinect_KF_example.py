# -*- coding: utf-8 -*-
"""
Created on Thu Apr 21 23:26:54 2016

@author: mgreiff
"""

import numpy as np
from math import cos, pi
import matplotlib.pylab as plt
from kalman_filters import discrete_KF_update

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

# Plot results
plt.figure(1)
plt.subplot(211)
plt.plot(tt,measurements[0,:],'b')
plt.plot(tt,states[0,:],'r')
plt.subplot(212)
plt.plot(tt,measurements[1,:],'b')
plt.plot(tt,states[2,:],'r')

plt.figure(2)
plt.subplot(211)
plt.plot(tt,estimates[0,:],'b')
plt.plot(tt,states[0,:],'r')
plt.subplot(212)
plt.plot(tt,estimates[2,:],'b')
plt.plot(tt,states[2,:],'r')