import rospy
from turtlesim.srv import Spawn, SpawnRequest, SpawnResponse

if __name__ == "__main__":
    rospy.init_node("follow_client")

    client = rospy.ServiceProxy("/spawn",Spawn)

    client.wait_for_service()

    req = SpawnRequest()

    req.x =1.0
    req.y =1.0
    req.theta =3.14
    req.name = "turtle2"

    try:
        response = client.call(req)
        rospy.loginfo("创建成功：name:%s",response.name)
    except Exception as e:
        rospy.loginfo("服务器调用失败...")