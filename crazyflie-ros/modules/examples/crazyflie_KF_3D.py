"""
This example shows how the KF is implemented and used in the kinect node by
simulating a tripple integrator over 10 seconds, estimating the position, velocity 
and acceleration of based on noisy positional measuements in [0,3] with a
standard deviaiton of 0.1 and very messy accelerometer data in [0,1] with a 
standard deviation of 0.5. The accelerometer data is not delayed here, as the
regular kalman filter performs exceptionally bad for asynchronous data. This
example merely shows how the kalman filter is used with the purpose of estimate  
the positions and velocities and states as good as possible not knowing velocities
(see plots).
"""
import numpy as np
import matplotlib.pylab as plt
import os,sys,inspect
from math import sin, sqrt, cos

# Loads crazylib from the parent directory
curdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
pardir = os.path.dirname(curdir)
sys.path.insert(0,pardir) 
from crazylib import discrete_KF_update

if __name__ == "__main__":
    # Time constants
    Ts = 0.01
    Nt = 2000
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
    P = 100*np.eye(Nstates,Nstates)
    Q = 0.01*np.eye(Nstates,Nstates)
    std_R = 0.1*np.array([[1.0],[1.0],[1.0]])
    R = np.diag(np.transpose(std_R)[0])
    
    # Empty vectors for solution history
    states = np.zeros((Nstates,Nt))
    states[:,0] = [0.,0.,0.,0.,0.,0.] # Initial conditions
    estimates = np.zeros((Nstates,Nt))
    estimates[:,0] = [0.,0.,0.,0.,0.,0.] # Initial conditions
    measurements = np.zeros((3,Nt))
    
    C1 = [1.0,0.5,1.5]
    C2 = [0.5,0.012,0.3]
    C3 = [2.0,0.0003,-3.5]
    for ii in range(1,len(tt)):
    
        # linear state trajectory in position  
        states[0,ii-1] = C1[0]*sin(C2[0]*tt[ii]) + C3[0]*tt[ii]
        states[1,ii-1] = C1[1]*sqrt(C2[1]*(tt[ii]+1.0)) + C3[1]
        states[2,ii-1] = C1[2]*sin(C2[2]*tt[ii]) + C3[2]*tt[ii]
    
        # Analytical derivative of the trajectory
        states[3,ii-1] = C1[0]*C2[0]*cos(C2[0]*tt[ii]) + C3[0]
        states[4,ii-1] = C1[1]*C2[1]/sqrt(C2[1]*(tt[ii]+1.0))
        states[5,ii-1] = C1[2]*C2[2]*cos(C2[2]*tt[ii]) + C3[2]
    
        # Find input variables
        vk = np.dot(np.random.randn(1,3),R)[0]
        zk = states[0:3,ii-1] + vk   
        xm1 = estimates[:,ii-1]
    
        # Kalman update
        xhat, P = discrete_KF_update(xm1,[],zk,A,[],C,P,Q,R)
    
        # logging
        estimates[:,ii] = xhat
        measurements[:,ii] = zk
    
    # Plot results
    plt.figure(1)
    plt.title('The true and correpted measurements\nof states used in the estimation')
    plt.plot(tt,np.transpose(measurements[0:3,:]),'b')
    plt.plot(tt,np.transpose(states[0:3,:]),'r')
    plt.legend(['True x','Measured x'])
    
    plt.figure(2)
    plt.subplot(211)
    plt.title('Estimated (xhat) and true states (E[p])')
    plt.plot(tt,np.transpose(estimates[0:3,:]),'b')
    plt.plot(tt,np.transpose(states[0:3,:]),'r')
    plt.legend(['phat','E[p]'])
    plt.subplot(212)
    plt.title('Estimated (phat) and true states (E[dp])')
    plt.plot(tt,np.transpose(estimates[3:6,:]),'b')
    plt.plot(tt,np.transpose(states[3:6,:]),'r')
    plt.legend(['dphat','E[dp]'])
    plt.show()
