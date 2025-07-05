#! /usr/bin/env python

import rospy
from ch1.msg import person  

if __name__ == "__main__":
    rospy.init_node("test_msg")

    pub = rospy.Publisher("test_msg_pub",person,queue_size=10)

    p = person()
    p.name = "jp"
    p.age = 18
    p.height = 1.85

    rate = rospy.Rate(1)
    
    while not rospy.is_shutdown():
        pub.publish(p)
        rospy.loginfo("massage:%s,%d,%.2f",p.name,p.age,p.height)
        rate.sleep()