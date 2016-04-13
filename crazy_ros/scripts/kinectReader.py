#!/usr/bin/env python

import rospy
import ros_numpy
import numpy as np
from stereo_msgs.msg import DisparityImage
from rospy.numpy_msg import numpy_msg

class KinectReader(object):
    def __init__(self):
        self.disparity_sub = rospy.Subscriber('/camera/depth/disparity', numpy_msg(DisparityImage), self.handle_disparity_image)


    def handle_disparity_image(self, d_image):
        print 'Image recieved'
        image = d_image.image
        np_image = ros_numpy.numpify(image)
        max_ind = np.argmax(np_image)
        i,j = np.unravel_index(np_image.argmax(), np_image.shape)

        print i, j, np.max(np_image)

        print ''


if __name__ == '__main__':
    rospy.init_node('KinectReader')
    kinect = KinectReader()
    rospy.spin()
