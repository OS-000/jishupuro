#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64

def callback(msg):
    rospy.loginfo("I heard {}".format(msg.data))

def listener():
    rospy.init_node('listener')
    rospy.Subscriber('arduino/distance', Float64, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException: pass 
