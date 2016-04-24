# -*- coding: utf-8 -*-
import numpy as np
from scipy.linalg import inv

def discrete_KF_update(x, u, z, A, B, C, P, Q, R):
    # Kalman prediction
    if not B:
        xf = np.transpose(np.dot(A,x))
    else:
        xf = np.transpose(np.dot(A,x)) + np.transpose(np.dot(B,u))
    Pf = np.dot(np.dot(A,P),np.transpose(A)) + Q
    
    # Kalman update
    Knum =  np.dot(Pf,np.transpose(C))
    Kden = np.dot(C,np.dot(Pf,np.transpose(C))) + R
    K = np.dot(Knum,inv(Kden))
    xhat = xf + np.dot(K, (z - np.dot(C,xf)))
    Pnew = np.dot((np.eye(Q.shape[0]) - np.dot(K,C)), Pf)

    return xhat, Pnew

def discrete_AKF_update(x, u, z, zhist, A, B, C, P, Q, R, trajectory, t, Ts):
    
    # Updates zhistory vector
    zhist[0:-1] = zhist[1:]
    zhist[-1] = z
    
    # Updates the covariance with the most recent synchronized measurement
    if np.isnan(zhist[0][0]):
        xhat = x
        xpred = x
    else:
        xhat, P = discrete_KF_update(x, u, zhist[-1], A, B, C, P, Q, R)
    
        # Predics what the future state will look like based on current data
        xpred = xhat
        Ppred = P
        for ii in range(len(zhist)-1):
            zpred = zhist[ii+1]
            zpred[1] = trajectory(t + (1 + ii - len(zhist))*Ts) + np.dot(np.random.randn(1),R[1,1])[0]
            xpred, Ppred = discrete_KF_update(xpred, u, zhist[ii+1], A, B, C, Ppred, Q, R)

    return xhat, xpred, P, zhist