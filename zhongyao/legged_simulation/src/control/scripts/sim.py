import rospy
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float64,Float64MultiArray
from pynput import keyboard
import threading
from gazebo_msgs.srv import SetModelState,SetModelStateRequest
"""
    i 0--11
    name: 
  - FL_calf_joint
  - FL_hip_joint
  - FL_thigh_joint
  - FR_calf_joint
  - FR_hip_joint
  - FR_thigh_joint
  - RL_calf_joint
  - RL_hip_joint
  - RL_thigh_joint
  - RR_calf_joint
  - RR_hip_joint
  - RR_thigh_joint
"""

def KeyboardInterface(key):
    try:
        if hasattr(key, 'char'):
            if key.char == 'w':
                state.control.x += 0.1
            elif key.char == 's':
                state.control.x -= 0.1
            elif key.char == 'd':
                state.control.y +=0.1
            elif key.char == 'a':
                state.control.y -= 0.1
            elif key.char == 'e':
                state.control.yaw += 0.1
            elif key.char == 'q':
                state.control.yaw -= 0.1
            # elif key.char == 'r':
            #     ResetModel()
        else:
            if key == keyboard.Key.space:
                state.control.x = 0.
                state.control.y = 0.
                state.control.yaw = 0.

        state.control.command = [state.control.x,state.control.y,state.control.yaw]
    except AttributeError:
        pass

def JointStateCallback(msg):
    for i in range(12):
        state.motor_state.p[i] = msg.position[i]
        state.motor_state.v[i] = msg.velocity[i]
        state.motor_state.tor[i] = msg.effort[i]
    """
    i 0--11
    name: 
  - FL_calf_joint
  - FL_hip_joint
  - FL_thigh_joint
  - FR_calf_joint
  - FR_hip_joint
  - FR_thigh_joint
  - RL_calf_joint
  - RL_hip_joint
  - RL_thigh_joint
  - RR_calf_joint
  - RR_hip_joint
  - RR_thigh_joint
    """

def MappedDataCallback(msg):
    # for i in range(12):
    #     state.mapped_data.run_data[i] = msg[i]
    state.mapped_data.run_data = msg.data

def ModelStateCallback(msg):
    pos = msg.pose[2]
    vel = msg.twist[2]
    state.imu.quaternion[0] = pos.orientation.x
    state.imu.quaternion[1] = pos.orientation.y
    state.imu.quaternion[2] = pos.orientation.z
    state.imu.quaternion[3] = pos.orientation.w

    state.imu.gyroscope[0] = vel.angular.x
    state.imu.gyroscope[1] = vel.angular.y
    state.imu.gyroscope[2] = vel.angular.z

def RunModel():
    for i in range(12):
        publisher_command[joint_name[i]]=state.mapped_data.run_data[i]
        joint_publishers[joint_name[i]].publish(publisher_command[joint_name[i]])

def DataPub():
    data = [] # joint p v e, weizi, jiaosudu, command 36+4+3+3
    data = data + state.motor_state.p \
                + state.motor_state.v \
                + state.motor_state.tor \
                + state.imu.quaternion \
                + state.imu.gyroscope \
                + state.control.command
    data = [float(i) for i in data]
    data = Float64MultiArray(data=data)
    # for i in range(len(data)):
    #     data_publishers.publish(data[i])
    data_publishers.publish(data)

def ResetModel():
    set_model_state = SetModelStateRequest().model_state
    gazebo_model_name = 'myrobot'
    set_model_state.model_name = gazebo_model_name
    set_model_state.pose.position.z = 5.
    set_model_state.pose.orientation.x = 0.
    set_model_state.pose.orientation.y= 0.
    set_model_state.pose.orientation.z= 0.
    set_model_state.pose.orientation.w= 0.
    set_model_state.reference_frame = 'ground_plane'
    gazebo_set_model_state_client(set_model_state)
    for i in range(12):
        publisher_command[joint_name[i]]=-10
        joint_publishers[joint_name[i]].publish(publisher_command[joint_name[i]])

class STATE():
    def __init__(self):
        self.imu = self.IMU()
        self.motor_state = self.MotorState()
        self.control = self.Control()
        self.mapped_data = self.MappedData()

    class IMU:
        def __init__(self):
            self.quaternion = [0.,0.,0.,1.] # x,y,z,w weizi siyuanshu
            self.gyroscope = [0.,0.,0.] # jiaosudu

    class MotorState:
        def __init__(self):
            self.p = [0.]*12 # position
            self.v = [0.]*12 # velocity
            self.tor = [0.]*12 # torque
    
    class Control:
        def __init__(self):
            self.x = 0.
            self.y = 0.
            self.yaw = 0.
            self.command = [0.,0.,0.]

    class MappedData:
        def __init__(self):
            self.run_data = [0.]*12

if __name__ == "__main__":
    rospy.init_node("test")
    state = STATE()
    
    joint_name = ['FL_calf_joint','FL_hip_joint','FL_thigh_joint',
                  'FR_calf_joint','FR_hip_joint','FR_thigh_joint',
                  'RL_calf_joint','RL_hip_joint','RL_thigh_joint',
                  'RR_calf_joint','RR_hip_joint','RR_thigh_joint']

    # TODO Reset Model Service 
    gazebo_set_model_state_client = rospy.ServiceProxy("/gazebo/set_model_state",SetModelState)

    # Sub JointStates
    JointStateSubscriber = rospy.Subscriber("/horse/joint_states",JointState,JointStateCallback,queue_size=10)
    # Sub ModelStates
    ModelStateSubscriber = rospy.Subscriber("/gazebo/model_states",ModelStates,ModelStateCallback,queue_size=10)

    # Pub joint
    joint_publishers = {}
    publisher_command = {}
    for i in range(12):
        publisher_command[joint_name[i]] = 0.0
        joint_topic = "/horse/"+joint_name[i]+"/command"
        joint_publishers[joint_name[i]] = rospy.Publisher(joint_topic,Float64,queue_size=10)
    
    # Pub sorted data
    data_topic = '/simulation_data'
    data_publishers = rospy.Publisher(data_topic,Float64MultiArray,queue_size=10) # Float64
    DataPub()

    # Sub mapped data
    RunDataSubscriber = rospy.Subscriber("/mpc_test",Float64MultiArray,MappedDataCallback,queue_size=10)

    listener_keyboard = keyboard.Listener(on_press=KeyboardInterface)
    listener_keyboard.daemon = True
    listener_keyboard.start()

    while not rospy.is_shutdown():
        print('\r'+f"Controller x: {state.control.x:.1f} y: {state.control.y:.1f} yaw: {state.control.yaw:.1f}   ", end='', flush=True)
        DataPub()
        RunModel()
        
    rospy.spin()
