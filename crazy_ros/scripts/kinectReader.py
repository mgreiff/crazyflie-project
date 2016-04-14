#!/usr/bin/env python

import rospy
import ros_numpy
import numpy as np
import matplotlib.pyplot as plt
from stereo_msgs.msg import DisparityImage
from sensor_msgs.msg import Image
from rospy.numpy_msg import numpy_msg

class KinectReader(object):
    def __init__(self):
        self.disparity_sub = rospy.Subscriber('/camera/depth/image_rect', Image, self.handle_disparity_image)
        plt.ion()
        self.ims = None
        self.background = None
        self.cal_frames = 0
        self.scatter = None


    def handle_disparity_image(self, image):
        print 'Image recieved'
        #print type(image)
        #np_image = ros_numpy.numpify(image)
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
            np_image = self.background - np_image

        if self.ims is None:
            self.ims = plt.imshow(np_image, vmin = 0, vmax = 5)
            plt.colorbar()
        else:
            self.ims.set_data(np_image)
            i, j = np.mean(np.where(np_image>(np.nanmax(np_image)-0.1)), axis=1)
            print i, j

            if self.scatter is not None:
                self.scatter.remove()
            self.scatter = plt.scatter([j], [i], color='red')

            x_c = np.round(j)
            y_c = np.round(i)
            #self.ims.autoscale()
            plt.draw()



        plt.pause(0.01)
        print ''


if __name__ == '__main__':
    rospy.init_node('KinectReader')
    kinect = KinectReader()
    rospy.spin()
