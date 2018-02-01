#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist

pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
rospy.init_node('naboris_motor_command_test')
r = rospy.Rate(4)  # hz


twist_msg = Twist()

def publish_twist(mm_per_s):
    rospy.loginfo("Commanding %smm/s" % mm_per_s)
    twist_msg.linear.x = mm_per_s
    pub.publish(twist_msg)
    r.sleep()
    return rospy.is_shutdown()


while not rospy.is_shutdown():
    for i in xrange(0, 90):
        if publish_twist(i):
            break
    for i in xrange(90, -90, -1):
        if publish_twist(i):
            break
    for i in xrange(-90, 0):
        if publish_twist(i):
            break
