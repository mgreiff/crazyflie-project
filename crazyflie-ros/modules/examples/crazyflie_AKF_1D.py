"""
This example shows how the AKF is implemented and used in the kinect node by
simulating the system over 10 seconds, estimating the position, velocity and
acceleration of the quadcopter based on noisy positional measuements in [0,3] with
a standard deviaiton of 0.1 and very messy accelerometer data in [0,1] with a 
standard deviation of 0.5. The accelerometer data is time delayed by 0.5 s and the
purpose is to estimate the velocities and states as good as possible (see plots).
"""
import numpy as np
from math import cos, pi
import matplotlib.pylab as plt
import os,sys,inspect

# Loads crazylib from the parent directory
curdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
pardir = os.path.dirname(curdir)
sys.path.insert(0,pardir) 
from crazylib import discrete_AKF_update

if __name__ == "__main__":
    # Acceleration offset
    amp = 0.8 
    amp = 1.
    amp = 1.2
    
    # Time constants
    Ts = 0.01
    Nt = 1000
    tt = np.linspace(0,(Nt-1)*Ts,Nt)
    receiveDelay = .5
    sendDelay = 1.
    Nrec = int(round(receiveDelay/Ts))
    Nsend = int(round(sendDelay/Ts))
    
    # Dynamics
    Nstates = 3
    Nmeasurements = 2
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
    
    # AKF specific parameters
    row = [np.nan for ii in range(0,Nmeasurements)]
    zhist = np.array([row for ii in range(0,Nrec)])
    
    for ii in range(1,len(tt)):
        
        trueAcc = lambda t: cos(4*pi*t/(Nt*Ts))        # Used as ground truth
        
        # Offset between E[acc] and the reference trajectory used in the state
        # prediction
        trajectory = lambda t: amp*cos(4*pi*t/(Nt*Ts)) # Used in state prediciton
    
        # Replace with measurements
        states[2,ii] = trueAcc(tt[ii])
        states[1,ii] = states[1,ii-1] + (states[2,ii] + states[2,ii - 1])*Ts/2
        states[0,ii] = states[0,ii-1] + (states[1,ii] + states[1,ii - 1])*Ts/2
    
        # Find input variables (here the acceleration is deleayed by delayRec)
        vk = np.dot(np.random.randn(1,2),R)[0]
        if ii < Nrec:
            zk = np.array([states[0,ii],states[2,0]]) + vk   
        else:
            zk = np.array([states[0,ii],states[2,ii-Nrec]]) + vk  
        xm1 = estimates[:,ii-1]
    
        xhat, xpred, P, zhist = discrete_AKF_update(xm1, [], zk, zhist, A, [], C, P, Q, R, trajectory, tt[ii], Ts)
    
        estimates[:,ii] = xpred
        measurements[:,ii] = zk
    
    # Plot results
    plt.figure(1)
    plt.subplot(211)
    plt.title('The true and correpted/delayed measurements\nof states used in the estimation')
    plt.plot(tt,measurements[0,:],'b')
    plt.plot(tt,states[0,:],'r')
    plt.legend(['True x','Measured x'])
    plt.subplot(212)
    plt.plot(tt,measurements[1,:],'b')
    plt.plot(tt,states[2,:],'r')
    plt.legend(['True ddx','Measured ddx'])

    plt.figure(2)
    plt.subplot(311)
    plt.title('Estimated (xhat) and true states (E[x]) with\nan acceleration amplitude offset of %s' % (str(amp)))
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
