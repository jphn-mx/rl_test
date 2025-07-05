import rospy
from sim.msg import Student

def domsg(data):
    rospy.loginfo("收到名字：%s,学号:%d",data.name,data.num)

if __name__ == "__main__":
    rospy.init_node("sub_demo")

    sub = rospy.Subscriber("name_num",Student,domsg,queue_size=1)
    
    rospy.spin()