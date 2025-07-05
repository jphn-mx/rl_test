import rospy
from std_msgs.msg import String

if __name__ == "__main__":
    rospy.init_node("sub_demo")

    sub = rospy.Subscriber("sub_demo",String,"name_num",queue_size=1)

    rospy.loginfo("收到名字和学号：%s",sub)