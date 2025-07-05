import rospy
from std_msgs.msg import Float64
import sys, select, termios, tty
move_bindings = ['w','s','a','d','q','e','i']

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin],[],[],0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def control(key):
    # forward
    if key == 'w':
        FL_calf.publish(-1.0)
    
    # backward
    if key == 's':
        pass

    # left
    if key == 'a':
        pass

    # right
    if key == 'd':
        pass

    # turn left
    if key == 'q':
        pass

    # turn right
    if key == 'e':
        pass

    if key == 'i':
        FL_hip.publish(0.0)
        rospy.sleep(2)
        FL_thigh.publish(0.0)
        FL_calf.publish(-0.0)
        RL_hip.publish(0.0)
        RL_thigh.publish(0.0)
        RL_calf.publish(-0.0)
        FR_hip.publish(0.0)
        FR_thigh.publish(-0.0)
        FR_calf.publish(0.0)
        RR_hip.publish(0.0)
        RR_thigh.publish(-0.0)
        RR_calf.publish(0.0)

        print("position init")

        rospy.sleep(2)


if __name__ == "__main__":
    rospy.init_node("control")

    FL_hip = rospy.Publisher("/horse/FL_hip_joint/command",Float64,queue_size=10)
    FL_thigh = rospy.Publisher("/horse/FL_thigh_joint/command",Float64,queue_size=10)
    FL_calf = rospy.Publisher("/horse/FL_calf_joint/command",Float64,queue_size=10)
    RL_hip = rospy.Publisher("/horse/RL_hip_joint/command",Float64,queue_size=10)
    RL_thigh = rospy.Publisher("/horse/RL_thigh_joint/command",Float64,queue_size=10)
    RL_calf = rospy.Publisher("/horse/RL_calf_joint/command",Float64,queue_size=10)
    FR_hip = rospy.Publisher("/horse/FR_hip_joint/command",Float64,queue_size=10)
    FR_thigh = rospy.Publisher("/horse/FR_thigh_joint/command",Float64,queue_size=10)
    FR_calf = rospy.Publisher("/horse/FR_calf_joint/command",Float64,queue_size=10)
    RR_hip = rospy.Publisher("/horse/RR_hip_joint/command",Float64,queue_size=10)
    RR_thigh = rospy.Publisher("/horse/RR_thigh_joint/command",Float64,queue_size=10)
    RR_calf = rospy.Publisher("/horse/RR_calf_joint/command",Float64,queue_size=10)

    rospy.sleep(2)  

    print("控制提示：")
    print("  w: 前进")
    print("  s: 后退")
    print("  a: 左移")
    print("  d: 右移")
    print("  q: 左旋转")
    print("  e: 右旋转")
    print("  i: 初始化")
    print("按Ctrl+C退出")

    settings = termios.tcgetattr(sys.stdin)

    try:
        while not rospy.is_shutdown():
            key = get_key(settings)
            if key in move_bindings:
                control(key)
            
            if key == '\x03':
                break
    except rospy.ROSInterruptException:
        pass

    finally:
        # 恢复终端设置
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)



