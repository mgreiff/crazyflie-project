#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

class CmdVelTestPublisher(object):
    def __init__(self):
        self.p = rospy.Publisher('crazyflie/cmd_vel', Twist, queue_size = 10)
        self.twist = Twist()
        self.r = rospy.Rate(50)
        
        self.start_pub()
    
    def start_pub(self):
        self.twist.linear.z = 40000
        self.twist.angular.y = 0 # Pitch, degrees. Positive is "right"
        self.twist.angular.x = 0 # Roll, degrees. Positive is "forward"
        
        d = -4000/50
        L = 60000
        while not rospy.is_shutdown():
            self.twist.linear.z += d
            print 'linear.z = %d' % self.twist.linear.z
            
            if self.twist.linear.z >= L:
                d = -d
            if self.twist.linear.z < 10000:
                self.stop()
                break
                    
            self.p.publish(self.twist)
            self.r.sleep()
           
    def stop(self):
        self.twist.linear.z = 0
        self.p.publish(self.twist)
        

if __name__ == '__main__':
    rospy.init_node('cmd_vel_test_publisher')
    cmdvel = CmdVelTestPublisher()
