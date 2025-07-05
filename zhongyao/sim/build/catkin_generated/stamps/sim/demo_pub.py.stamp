import rospy
from sim.msg import Student

if __name__ == "__main__":
    rospy.init_node("pub_demo")
    
    name_data = Student()

    name_data.name = "dyf"

    name_data.num = 2022114627

    pub = rospy.Publisher("name_num",Student,queue_size=10)

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():

        pub.publish(name_data)

        rospy.loginfo("已经发布名字：%s学号:%d",name_data.name,name_data.num)

        rate.sleep()