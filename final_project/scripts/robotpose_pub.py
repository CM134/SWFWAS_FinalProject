#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry

def callback(msg):
    print "----"
    print msg
    print "===position---x"
    print msg.pose.pose.position.x

rospy.init_node('check_odometry')
odom_sub = rospy.Subscriber('/odom', Odometry, callback)
rospy.spin()