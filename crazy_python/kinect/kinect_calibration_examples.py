# -*- coding: utf-8 -*-
from scipy.linalg import svd, norm, solve
import numpy as np
from math import acos, pi
import matplotlib.pylab as plt

def regression(xx,yy,option):
    # Performs a linear and exponential regression
    #
    # Linear model:      y = c_0 + c_1 * x
    # Exponential model: y = exp(c_0 + c_1 * x)
    #
    # ARGS:
    #    xx - Array of shape (N,)
    #    yy - Array of shape (N,)
    #    option - String, either 'exp' or 'lin'
    a = np.transpose(np.array([np.ones(N),tt]))
    if option == 'lin':
        b = np.transpose(np.array([yy]))
    elif option == 'exp':
        b = np.transpose(np.array([np.log(yy)]))
    return solve(np.dot(np.transpose(a),a),np.dot(np.transpose(a),b))
    
if 1:
    # Calibrates the angle by fitting a plane through the set of points and
    # and checking the angle if the normal projected into the xz-plane.
    points = np.random.rand(10,3)
    
    x = points[:,0]*0.5
    y = points[:,1]
    z = points[:,2]
    
    P = np.array([sum(x),sum(y),sum(z)])/len(x);
    [U,S,V] = svd(np.transpose(np.array([x-P[0],y-P[1],z-P[2]])));
    N = -1./V[2,2]*V[2,:]
    
    XZnormal = np.array([[N[0]],[N[2]]])
    angle = (N[0]/abs(N[0]))*acos(N[0]/norm(XZnormal))
    
    if abs(angle) > pi/2 and abs(angle) < 3*pi/2:
        angle=pi-angle
    
# -*- coding: utf-8 -*-
from scipy.linalg import svd, norm, solve
import numpy as np
from math import acos, pi
import matplotlib.pylab as plt

def regression(xx,yy,option):
    # Performs a linear and exponential regression
    #
    # Linear model:      y = c_0 + c_1 * x
    # Exponential model: y = exp(c_0 + c_1 * x)
    #
    # ARGS:
    #    xx - Array of shape (N,)
    #    yy - Array of shape (N,)
    #    option - String, either 'exp' or 'lin'
    a = np.transpose(np.array([np.ones(N),tt]))
    if option == 'lin':
        b = np.transpose(np.array([yy]))
    elif option == 'exp':
        b = np.transpose(np.array([np.log(yy)]))
    return solve(np.dot(np.transpose(a),a),np.dot(np.transpose(a),b))
    
if 1:
    # Calibrates the angle by fitting a plane through the set of points and
    # and checking the angle if the normal projected into the xz-plane.
    points = np.random.rand(10,3)
    
    x = points[:,0]*0.5
    y = points[:,1]
    z = points[:,2]
    
    P = np.array([sum(x),sum(y),sum(z)])/len(x);
    [U,S,V] = svd(np.transpose(np.array([x-P[0],y-P[1],z-P[2]])));
    N = -1./V[2,2]*V[2,:]
    
    XZnormal = np.array([[N[0]],[N[2]]])
    angle = (N[0]/abs(N[0]))*acos(N[0]/norm(XZnormal))
    
    if abs(angle) > pi/2 and abs(angle) < 3*pi/2:
        angle=pi-angle
    
    print('---\nCalibration angle: %s\n---'  % (angle*180./pi))

if 1:
    # Finds a linear or exponential regression to map the measured depth to
    # actual depth.
    option = 'exp'
    N = 20
    tt = np.arange(N)
    yy = 10*np.exp(-0.1*tt) + np.random.normal(0,0.1,(1,N))[0]
    plt.plot(tt,yy)
    
    coeff = regression(tt, yy, option)
    print coeff
    
    if option == 'lin':
        plt.plot(tt,coeff[0] + coeff[1]*tt)
    if option == 'exp':
        plt.plot(tt,np.exp(coeff[0])*np.exp(coeff[1]*tt))

if 1:
    # Finds a linear or exponential regression to map the measured depth to
    # actual depth.
    option = 'exp'
    N = 20
    tt = np.arange(N)
    yy = 10*np.exp(-0.1*tt) + np.random.normal(0,0.1,(1,N))[0]
    plt.plot(tt,yy)
    
    coeff = regression(tt, yy, option)
    print('---\nRegression coefficients:\n%s\n---'  % (str(coeff)))
    
    if option == 'lin':
        plt.plot(tt,coeff[0] + coeff[1]*tt)
    if option == 'exp':
        plt.plot(tt,np.exp(coeff[0])*np.exp(coeff[1]*tt))
    plt.show()
