#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

class CmdVelTestPublisher(object):
    def __init__(self):
        self.p = rospy.Publisher('crazyflie/cmd_vel', Twist, queue_size = 10)
        self.twist = Twist()
        self.twist_z = Twist()
        self.r = rospy.Rate(50)
        
        self.start_pub()
    
    def start_pub(self):
        self.twist.linear.z = 5000
        self.twist.angular.y = 0 # Roll, degrees. Positive moves "left"
        self.twist.angular.x = 0 # Pitch, degrees. Positive moves "forward"
        
        d = -0/50
        L = 1000000
        Lt = 50
        c = 0
        while not rospy.is_shutdown():
            #self.twist.linear.z += d
            c += 1
            print 'linear.z = %d' % self.twist.linear.z
            
            """if self.twist.linear.z >= L and d > 0:
                d = -d"""
            if c == Lt:
                self.twist.linear.z = 60000
            if c == Lt + 60:
                self.twist.linear.z = 40000
            if c == Lt + 85:
                self.twist.linear.z = 45000
            if c == Lt + 110:
                self.twist.linear.z = 0
            
            if self.twist.linear.z <= 0:
                self.stop()
                break
            
            #self.p.publish(self.twist_z)        
            self.p.publish(self.twist)
            self.r.sleep()
           
    def stop(self):
        self.twist.linear.z = 0
        self.p.publish(self.twist)
        

if __name__ == '__main__':
    rospy.init_node('cmd_vel_test_publisher')
    cmdvel = CmdVelTestPublisher()
