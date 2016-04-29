#!/usr/bin/env python

import rospy
import ros_numpy
import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import svd, norm, solve, inv
from math import acos, pi
from sensor_msgs.msg import Image
from rospy.numpy_msg import numpy_msg
from geometry_msgs.msg import Point
from std_msgs.msg import String
from crazy_ros.msg import NumpyArrayFloat64
import os,sys,inspect

# Loads the crazyflie library module from the /modules directory
curdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
libdir = os.path.join(os.path.dirname(curdir),'modules')
sys.path.insert(1,libdir)
import crazylib as cl

class KinectNode(object):

    def __init__(self):
        self.background = None
        self.angle = None     # The calibration angle of 
        self.cal_frames = 0   # Counter used in the calibration of the background, which startes if < 30
        self.scatter = None   # Figure handle for scatter plot 
        self.ims = None       # Figure handle for plotting

        # Settings
        self.plot = False     # Plot the depth data in real time (may be slow)
        self.useDummy = True  # Run node reading data from the dummy
        self.useKalman = True # Compute kalman estimate and print covariance and estimate
        if self.plot:
            plt.ion()

        # Camera centers and focal lengths (see /camera/depth/camera_info)
        self.f_x = 570.34
        self.f_y = 570.34
        self.c_x = 314.5
        self.c_y = 235.5

        # Kalman filter parameters
        # TODO - Load from configuration file
        self.Q = np.diag(0.001*np.ones((1,6)))
        self.R = np.diag([0.1,0.1,0.1])
        self.P = np.eye(6)
        self.Ts = 1./30
        self.A = np.eye(6) + np.diag(self.Ts*np.ones((1,3))[0],3)

        self.C = np.zeros((3,6))
        self.C[0,0] = 1.
        self.C[1,1] = 1.   
        self.C[2,2] = 1.

        self.B = np.zeros((6,3))
        self.B[3,0] = 1.
        self.B[4,1] = 1.
        self.B[5,2] = 1.

        self.xhat = np.zeros((1,6))[0]

        # Sets up publishers and subscribers
        if self.useDummy:
            self.disparity_sub = rospy.Subscriber('/kinect/dummy', NumpyArrayFloat64, self.handle_disparity_image)
        else:
            self.disparity_sub = rospy.Subscriber('/camera/depth/image_rect', Image, self.handle_disparity_image)
        self.status_sub = rospy.Subscriber('/system/kinect', String, self.handle_status) # Commands from the master
        self.point_pub = rospy.Publisher('/kinect/position', Point, queue_size = 10) # Publishes position
        self.status_pub = rospy.Publisher('/system/status_master', String, queue_size = 10) # Publish to the master

    def handle_status(self, msg):
        if msg.data == 'Calibrate':
            self.background == None
            self.cal_frames = 0
            self.angle = None

    def handle_disparity_image(self, image):

        if self.useDummy:
            np_image = np.reshape(image.data, (480,640))
        else:
            np_image = ros_numpy.numpify(image)
        np_image_rel = np_image

        if self.cal_frames < 30:
            # Calibrates background
            if not self.cal_frames:
                print '\n(@ kinectNode) Calibrating background...'
            if self.background is None:
                self.background = np.zeros(np_image.shape)
            self.background += np_image / 30.0
            self.cal_frames += 1
            mask = np.isnan(self.background)
            self.background[mask] = np.interp(np.flatnonzero(mask), np.flatnonzero(~mask), self.background[~mask])
        else:
            # Calibrates angle
            np_image_rel = self.background - np_image
            if self.angle is None:
                print '(@ kinectNode) Calibrating angle...'

                self.calibrate_angle_SVD()
                print '(@ kinectNode) Angle using SVD: %f degrees' % (180/3.1415 * self.angle)

                self.calibrate_angle()
                print '(@ kinectNode) Angle using poyfit: %f degrees' % (180/3.1415 * self.angle)
                print '(@ kinectNode) Calibration complete!\n'
                self.status_pub.publish('True')

            # Runs in regular mode, publishing the position in the global coordinate system
            i, j = np.mean(np.where(np_image_rel>(np.nanmax(np_image_rel)-0.1)), axis=1)

            if self.plot:
                if self.ims is None:
                    self.ims = plt.imshow(np_image_rel, vmin = 0, vmax = 5)
                    plt.colorbar()
                else:
                    self.ims.set_data(np_image_rel)

                    if self.scatter is not None:
                        self.scatter.remove()
                    self.scatter = plt.scatter([j], [i], color='red')
                plt.draw()
                plt.pause(0.01)

            z, y, x = self.point_from_ij(i, j, np_image)

            # Kalman filter update - This currently just computes, updates and prints the position and covariance
            if self.useKalman:
                zk = np.array([x,y,z])             # Measure positions
                uk = 0.1 * np.random.randn(1,3)[0] # Add noise to acceleration states
                self.xhat, self.P = cl.discrete_KF_update(self.xhat, uk, zk, self.A, self.B, self.C, self.P, self.Q, self.R)
                print self.xhat
                print zk
                print uk
                print self.P

            p = Point(x=x, y=y, z=z)
            self.point_pub.publish(p)

    def point_from_ij(self, i, j, np_image, rotate = True):
        x_c = np.round(j)
        y_c = np.round(i)
        
        z = np_image[y_c, x_c]
        x = (x_c - self.c_x)*z/self.f_x
        y = -(y_c - self.c_y)*z/self.f_y

        if self.angle is not None and rotate:
            s = math.sin(self.angle)
            c = math.cos(self.angle)
            y, z = c*y + s*z, -s*y + c*z

        return x, y, z

    def calibrate_angle(self):
        self.angle = None
        for i in range(0, 400, 50):
            angle, r = self.calibrate_angle_o(i)
            if self.angle is None or r < best_r:
                best_r = r
                self.angle = angle
                best_i = i

                # Found roof
                if self.angle < 0:
                    #print 'roof'
                    self.angle += math.pi/2

    def calibrate_angle_o(self, o):
        j = 320
        n = 100
        i_l = range(o, o + n)
        y_l = np.zeros(n)
        z_l = np.zeros(n)

        for ind, i in enumerate(i_l):
            x, y, z = self.point_from_ij(i, j, self.background, rotate = False)
            y_l[ind] = y
            z_l[ind] = z
            
        p, r, _, _, _ = np.polyfit(y_l, z_l, 1, full = True)
        return np.arctan(p[0]), r

    def calibrate_angle_SVD(self):
        # Approximates a 3D-plane to the set of points and computes the angle.
        # See the /crazy_documentation for the mathematics involved.

        # Samples background matrix, finds S
        nZ = nY = 11
        indY = range(nY)
        indZ = range(nZ)
        x = np.zeros(nY*nZ)
        y = np.zeros(nY*nZ)
        z = np.zeros(nY*nZ)
        count = 0
        for ii in indY:
            for jj in indZ:
                x[count] = self.background[ii,jj]
                y[count] = ii
                z[count] = jj
                count += 1

        # Sets up P and computes 3D-plane normal
        P = np.array([sum(x),sum(y),sum(z)])/len(x);
        [U,S,V] = svd(np.transpose(np.array([x-P[0],y-P[1],z-P[2]])));
        N = -1./V[2,2]*V[2,:]
        XZnormal = np.array([[N[0]],[N[2]]])

        # Computes angle
        self.angle = (N[0]/abs(N[0]))*acos(N[0]/norm(XZnormal))
        if abs(self.angle) > pi/2 and abs(self.angle) < 3*pi/2:
            self.angle=pi-self.angle

if __name__ == '__main__':
    rospy.init_node('kinectNode')
    kin = KinectNode()
    rospy.spin()
