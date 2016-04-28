#!/usr/bin/env python

import rospy
import ros_numpy
import math
import numpy as np
import matplotlib.pyplot as plt
from stereo_msgs.msg import DisparityImage
from sensor_msgs.msg import Image
from rospy.numpy_msg import numpy_msg
from geometry_msgs.msg import Point
from scipy.signal import medfilt

class KinectReader(object):
    def __init__(self):
        plt.ion()
        self.ims = None
        self.background = None
        self.cal_frames = 0
        self.scatter = None
        self.plot = True

        # Camera centers and focal lengths, from /camera/depth/camera_info
        self.f_x = 570.34
        self.f_y = 570.34
        self.c_x = 314.5
        self.c_y = 235.5

        self.angle = None 

        self.disparity_sub = rospy.Subscriber('/camera/depth/image_rect', Image, self.handle_disparity_image)
        self.point_pub = rospy.Publisher('/kinect/position', Point, queue_size = 10)


    def handle_disparity_image(self, image):
        #print type(image)
        np_image = ros_numpy.numpify(image)
        np_image_rel = np_image
        #print type(np_image)
        #print np_image.shape
        #print np.nanmax(np_image)
        #print np.nanmin(np_image)
        #np.savetxt('/home/daniel/test.out', np_image)
        #print '-----'

        if self.cal_frames < 30:
            if self.background is None:
                self.background = np.zeros(np_image.shape)
            self.background += np_image / 30.0
            self.cal_frames += 1
            mask = np.isnan(self.background)
            self.background[mask] = np.interp(np.flatnonzero(mask), np.flatnonzero(~mask), self.background[~mask])
        else:
            np_image_rel = self.background - np_image
            
            if self.angle is None:
                print 'Calibration complete'
                
                self.calibrate_angle()
                print 'Angle: %f degrees' % (180/3.1415 * self.angle)
            
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

        x, y, z = self.point_from_ij(i, j, np_image)
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
        for i in range(0, 400, 50):
            angle, r = self.calibrate_angle_o(i)
            #print i, r
            if self.angle is None or r < best_r:
                best_r = r
                self.angle = angle
                best_i = i

                # Found roof
                if self.angle < 0:
                    #print 'roof'
                    self.angle += math.pi/2
        #print best_i


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


if __name__ == '__main__':
    rospy.init_node('KinectReader')
    kinect = KinectReader()
    rospy.spin()
