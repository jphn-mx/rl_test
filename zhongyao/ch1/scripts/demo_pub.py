import rospy
from std_msgs.msg import String

if __name__ == "__main__":
    rospy.init_node("pub_demo")
    
    name_data = "dyf,2022114627"

    pub = rospy.Publisher("name_num",String,queue_size=10)

    pub.publish(name_data)

    while not rospy.is_shutdown:
        rospy.loginfo("已经发布名字和学号：%s",name_data)
        rospy.spin()