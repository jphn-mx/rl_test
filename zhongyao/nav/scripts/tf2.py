import rospy
import tf.transformations
import tf2_ros
import tf
from geometry_msgs.msg import TransformStamped

if __name__ == "__main__":
    rospy.init_node("map2odom")

    broadcaster = tf2_ros.StaticTransformBroadcaster()

    tfs = TransformStamped()

    tfs.header.frame_id = "map"
    tfs.header.stamp = rospy.Time.now()
    tfs.header.seq = 101

    tfs.child_frame_id="odometry"

    tfs.transform.translation.x = 0.
    tfs.transform.translation.y = 0.
    tfs.transform.translation.z = 0.

    qtn = tf.transformations.quaternion_from_euler(0,0,0)
    tfs.transform.rotation.x = qtn[0]
    tfs.transform.rotation.y = qtn[1]
    tfs.transform.rotation.z = qtn[2]
    tfs.transform.rotation.w = qtn[3]

    broadcaster.sendTransform(tfs)

    rospy.spin()




