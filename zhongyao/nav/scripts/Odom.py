"""
import rospy
from nav_msgs.msg import Odometry

def doOdom(pubOdomAftMapped):
    om = Odometry()
    om.header.frame_id = "odom"
    om.header.stamp = rospy.Time.now()
    om.child_frame_id = "base_footprint"
    om.pose.pose.position.x = pubOdomAftMapped.pose.pose.position.x
    om.pose.pose.position.y = pubOdomAftMapped.pose.pose.position.y    
    om.pose.pose.position.z = pubOdomAftMapped.pose.pose.position.z

    om.pose.pose.orientation.x = pubOdomAftMapped.pose.pose.orientation.x
    om.pose.pose.orientation.y = pubOdomAftMapped.pose.pose.orientation.y
    om.pose.pose.orientation.z = pubOdomAftMapped.pose.pose.orientation.z
    om.pose.pose.orientation.w = pubOdomAftMapped.pose.pose.orientation.w

    pub = rospy.Publisher("/Odom",Odometry,queue_size=10)
    
    pub.publish(om)

if __name__ == "__main__":

    rospy.init_node("odometry")

    sub = rospy.Subscriber("/aft_mapped_to_init",Odometry,doOdom,queue_size=1)

    rospy.spin()
"""

import rospy
from nav_msgs.msg import Odometry
import tf

def doOdom(pubOdomAftMapped):
    om = Odometry()
    om.header.frame_id = "odometry"
    om.header.stamp = rospy.Time.now()  # 使用当前时间戳
    om.child_frame_id = "base_footprint"
    om.pose.pose.position.x = pubOdomAftMapped.pose.pose.position.x
    om.pose.pose.position.y = pubOdomAftMapped.pose.pose.position.y    
    om.pose.pose.position.z = pubOdomAftMapped.pose.pose.position.z

    om.pose.pose.orientation.x = pubOdomAftMapped.pose.pose.orientation.x
    om.pose.pose.orientation.y = pubOdomAftMapped.pose.pose.orientation.y
    om.pose.pose.orientation.z = pubOdomAftMapped.pose.pose.orientation.z
    om.pose.pose.orientation.w = pubOdomAftMapped.pose.pose.orientation.w

    pub.publish(om)


    br = tf.TransformBroadcaster()
    current_time = rospy.Time.now()
    br.sendTransform((om.pose.pose.position.x, om.pose.pose.position.y, om.pose.pose.position.z),
                     (om.pose.pose.orientation.x, om.pose.pose.orientation.y, om.pose.pose.orientation.z, om.pose.pose.orientation.w),
                     current_time,
                     "base_footprint",
                     "odometry")

if __name__ == "__main__":
    rospy.init_node("odometry")
    pub = rospy.Publisher("/odometry", Odometry, queue_size=10)
    sub = rospy.Subscriber("/aft_mapped_to_init", Odometry, doOdom, queue_size=1)
    rospy.spin()