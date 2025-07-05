import rospy
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float64,Float64MultiArray
from geometry_msgs.msg import WrenchStamped
from pynput import keyboard
import threading
from gazebo_msgs.srv import SetModelState,SetModelStateRequest
from unitree_legged_msgs.msg import MotorCmd
from sensor_msgs.msg import Imu
from std_srvs.srv import Empty
import time
import numpy as np

def KeyboardInterface(key):
    try:
        if hasattr(key, 'char'):
            if key.char == 'r':
                ResetModel()
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

def ModelStateCallback(msg):
    pos = msg.pose[-1]
    vel = msg.twist[-1]

    state.imu.quaternion[0] = pos.orientation.x
    state.imu.quaternion[1] = pos.orientation.y
    state.imu.quaternion[2] = pos.orientation.z
    state.imu.quaternion[3] = pos.orientation.w

    state.imu.pose[0] = pos.position.x
    state.imu.pose[1] = pos.position.y
    state.imu.pose[2] = pos.position.z

    state.imu.velocity[0] = vel.linear.x
    state.imu.velocity[1] = vel.linear.y
    state.imu.velocity[2] = vel.linear.z

    state.imu.gyroscope[0] = vel.angular.x
    state.imu.gyroscope[1] = vel.angular.y
    state.imu.gyroscope[2] = vel.angular.z

def ImuCallback(msg):
    state.imu.IMU[0] = msg.angular_velocity.x # jiaojiasudu
    state.imu.IMU[1] = msg.angular_velocity.y
    state.imu.IMU[2] = msg.angular_velocity.z

    state.imu.IMU[3] = msg.linear_acceleration.x # xianjiasudu
    state.imu.IMU[4] = msg.linear_acceleration.y
    state.imu.IMU[5] = msg.linear_acceleration.z

def FLContactCallback(msg):
    state.contact_force.forces[0] = msg.wrench.force.z

def FRContactCallback(msg):
    state.contact_force.forces[1] = msg.wrench.force.z

def RLContactCallback(msg):
    state.contact_force.forces[2] = msg.wrench.force.z

def RRContactCallback(msg):
    state.contact_force.forces[3] = msg.wrench.force.z

def DataPub():
    data = []       # joint p v e(36), 
                    # force (fl,fr,rl,rr 4)
                    # matoufangxiang(4), 
                    # weizhi(3),
                    # jiaosudu(3), 
                    # xiansudu(3),
                    # imu(jiaojiasudu 3 + xianjiasudu 3),
                    # commands(3) shanchu
    data = data + state.motor_state.p \
                + state.motor_state.v \
                + state.motor_state.tor \
                + state.contact_force.forces \
                + state.imu.quaternion \
                + state.imu.pose \
                + state.imu.gyroscope \
                + state.imu.velocity \
                + state.imu.IMU 
                # + state.control.command
    data = [float(i) for i in data]
    data = Float64MultiArray(data=data)
    # for i in range(len(data)):
    #     data_publishers.publish(data[i])
    data_publishers.publish(data)

def ResetModel():
    set_model_state = SetModelStateRequest().model_state
    gazebo_model_name = 'myrobot'
    set_model_state.model_name = gazebo_model_name
    set_model_state.pose.position.z = 0.5
    set_model_state.pose.orientation.x = 0.
    set_model_state.pose.orientation.y= 0.
    set_model_state.pose.orientation.z= 0.
    set_model_state.pose.orientation.w= 0.
    set_model_state.reference_frame = 'ground_plane'
    gazebo_set_model_state_client(set_model_state)
    for i in range(12):
        publisher_command_commands[i].tau = 0.0
        publisher_command_commands[i].q = -0.0
        publisher_command_commands[i].Kp = 100.
        publisher_command_commands[i].Kd = 1.
        if i in [0,3,6,9]:
            publisher_command_commands[i].q = -1.2
        if i in [2,5,8,11]:
            publisher_command_commands[i].q = 0.9
        joint_publishers[joint_name[i]].publish(publisher_command_commands[i])
    time.sleep(0.2)
    for i in range(12):
        # publisher_command_commands[i].q = -0.0
        publisher_command_commands[i].Kp = 0.
        publisher_command_commands[i].Kd = 0.
        publisher_command_commands[i].tau = 0.0
        joint_publishers[joint_name[i]].publish(publisher_command_commands[i])
    
class STATE():
    def __init__(self):
        self.imu = self.IMU()
        self.motor_state = self.MotorState()
        self.control = self.Control()
        self.mapped_data = self.MappedData()
        self.contact_force = self.ContactForce()
        self.model_state = self.ModelState()

    class IMU:
        def __init__(self):
            self.quaternion = [0.,0.,0.,1.] # x,y,z,w weizi siyuanshu
            self.pose = [0.,0.,0.]
            self.velocity = [0.,0.,0.] # xiansudu
            self.gyroscope = [0.,0.,0.] # jiaosudu
            self.IMU = [0.,0.,0.,0.,0.,0.]

    class MotorState:
        def __init__(self):
            self.p = [0.]*12 # position
            self.v = [0.]*12 # velocity
            self.tor = [0.]*12 # torque

    class ContactForce:
        def __init__(self):
            # self.fl_force = 0.
            # self.fr_force = 0.
            # self.rl_force = 0.
            # self.rr_force = 0.
            # self.forces = [self.fl_force,self.fr_force,self.rl_force,self.rr_force]
            self.forces = [0.,0.,0.,0.]

    class Control:
        def __init__(self):
            self.x = 0.
            self.y = 0.
            self.yaw = 0.
            self.command = [0.,0.,0.]

    class MappedData:
        def __init__(self):
            self.run_data = [MotorCmd() for _ in range(12)]

    class ModelState:
        def __init__(self):
            self.runing = True

if __name__ == "__main__":
    rospy.init_node("test")
    state = STATE()
    
    joint_name = ['FL_calf_joint','FL_hip_joint','FL_thigh_joint',
                  'FR_calf_joint','FR_hip_joint','FR_thigh_joint',
                  'RL_calf_joint','RL_hip_joint','RL_thigh_joint',
                  'RR_calf_joint','RR_hip_joint','RR_thigh_joint']
    
    foot_name = ['FL_foot','FR_foot','RL_foot','RR_foot']

    # Reset Model Service 
    gazebo_set_model_state_client = rospy.ServiceProxy("/gazebo/set_model_state",SetModelState)
    # Pause Model Service
    gazebo_pause_client = rospy.ServiceProxy("/gazebo/pause_physics",Empty)
    gazebo_unpause_client = rospy.ServiceProxy("/gazebo/unpause_physics",Empty)

    # Sub JointStates
    JointStateSubscriber = rospy.Subscriber("/horse/joint_states",JointState,JointStateCallback,queue_size=10)
    # Sub ModelStates
    ModelStateSubscriber = rospy.Subscriber("/gazebo/model_states",ModelStates,ModelStateCallback,queue_size=10)
    # Sub ContactForce
    FLContactSubscriber = rospy.Subscriber("/visual/FL_foot/the_force",WrenchStamped,FLContactCallback,queue_size=10)
    FRContactSubscriber = rospy.Subscriber("/visual/FR_foot/the_force",WrenchStamped,FRContactCallback,queue_size=10)
    RLContactSubscriber = rospy.Subscriber("/visual/RL_foot/the_force",WrenchStamped,RLContactCallback,queue_size=10)
    RRContactSubscriber = rospy.Subscriber("/visual/RR_foot/the_force",WrenchStamped,RRContactCallback,queue_size=10)
    # Sub IMU
    IMUSubscriber = rospy.Subscriber("/base_imu",Imu,ImuCallback,queue_size=10)

    # Pub joint
    joint_publishers = {}
    publisher_command_commands = [MotorCmd() for _ in range(12)]
    for i in range(12):
        joint_topic = "/horse/"+joint_name[i]+"/command"
        joint_publishers[joint_name[i]] = rospy.Publisher(joint_topic,MotorCmd,queue_size=10)
        publisher_command_commands[i].mode = 0x0A
    # Pub sorted data
    data_topic = '/simulation_data'
    data_publishers = rospy.Publisher(data_topic,Float64MultiArray,queue_size=10) # Float64
    DataPub()

    listener_keyboard = keyboard.Listener(on_press=KeyboardInterface)
    listener_keyboard.daemon = True
    listener_keyboard.start()

    while not rospy.is_shutdown():
        print('\r'+f"simulation start ...  press r to reset model ", end='', flush=True)
        DataPub()

    rospy.spin()
