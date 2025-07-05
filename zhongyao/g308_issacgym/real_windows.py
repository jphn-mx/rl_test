#!/home/g308/anaconda3/envs/rl/bin/python
import sys
import os
import torch
import threading
import time
import rospy
import time
import logging
import numpy as np
from std_srvs.srv import Empty
# import serial
import struct
import pygame
path = os.path.abspath(".")
sys.path.insert(0, path + "/src/rl_sar/scripts")
from rl_sdk import *
from observation_buffer import *
from obs_buffer import *
from Port import *
class RobotCommand:
    def __init__(self):
        self.commands = [0.0]*12
class Control:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

class Observations:
    def __init__(self):
        self.lin_vel = None
        self.ang_vel = None
        self.gravity_vec = None
        self.commands = None
        self.base_quat = None
        self.dof_pos = None
        self.dof_vel = None
        self.actions = None

class ModelParams:
    def __init__(self):
        self.model_name = None
        self.crawl_name = None
        self.jump_name = None
        self.framework = None
        self.dt = None
        self.decimation = None
        self.num_observations = None
        self.observations = None
        self.observations_history = None
        self.damping = None
        self.stiffness = None
        self.action_scale = None
        self.hip_scale_reduction = None
        self.hip_scale_reduction_indices = None
        self.clip_actions_upper = None
        self.clip_actions_lower = None
        self.num_of_dofs = None
        self.lin_vel_scale = None
        self.ang_vel_scale = None
        self.dof_pos_scale = None
        self.dof_vel_scale = None
        self.clip_obs = None
        self.torque_limits = None
        self.rl_kd = None
        self.rl_kp = None
        self.fixed_kp = None
        self.fixed_kd = None
        self.commands_scale = None
        self.default_dof_pos = None
        self.joint_controller_names = None
class RobotState:
    def __init__(self):
        self.imu = self.IMU()
        self.motor_state = self.MotorState()

    class IMU:
        def __init__(self):
            self.quaternion = [1.0, 0.0, 0.0, 0.0]  # w, x, y, z
            self.gyroscope = [0.0, 0.0, 0.0]
            self.accelerometer = [0.0, 0.0, 0.0]

    class MotorState:
        def __init__(self):
            self.q = [0.0] * 32
            self.dq = [0.0] * 32
            self.ddq = [0.0] * 32
            self.tauEst = [0.0] * 32
            self.cur = [0.0] * 32

class RL_Real:
    def __init__(self):
        rospy.init_node("rl_real")
        
        ## log ##
        logging.basicConfig(
            filename='dynamic_lists.log',
            level=logging.INFO,
            format='%(asctime)s - %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )

        self.params = ModelParams()
        self.obs = Observations()
        self.control = Control()
        self.robot_state = RobotState()
        self.robot_command = RobotCommand()
        self.robot_name = "g308_isaacgym"
        self.ReadYaml(self.robot_name)
        if len(self.params.observations_history) != 0:
            self.obs_buff = obs_buffer(1,self.params.num_observations,len(self.params.observations_history))
        
        model_path = os.path.join(os.path.dirname(__file__), f"../models/{self.robot_name}/{self.params.model_name}")
        self.model = torch.jit.load(model_path)
        self.model = self.model.to('cpu')
        
        self.recived_data = [0.]*30 # jiashe FL FR RL RR dof_pos12 dof_vel24 gravity27 ang_vel30
        self.state = 'running'
        self.obs.actions = torch.tensor([[0.]*12])
        self.control.x = 0.
        self.control.y = 0.
        self.control.yaw = 0.

        self.InitObservations()
        self.InitOutputs()
        self.InitControl()

        # self.port = Port()
        self.lock = threading.Lock
        self.thread_control = threading.Thread(target=self.ThreadControl)
        self.thread_rl = threading.Thread(target=self.ThreadRL)
        self.thread_control.start()
        self.thread_rl.start()
    # def Joy(self):
    #     for event in pygame.event.get():
    #         if event.type == pygame.JOYAXISMOTION:
    #             if event.axis == 1:
    #                 self.x = event.value
    #             elif event.axis == 0:
    #                 self.y = event.value
    #             elif event.axis == 2:
    #                 self.yaw = event.value
    #         elif event.type == pygame.JOYBUTTONDOWN:
    #             if event.button == 0:
    #                 self.state = 'running'
    #             elif event.button == 1:
    #                 self.state = 'crawl'
    #             elif event.button == 3:
    #                 self.state = 'jump'

    #     print(f'x:{self.x:.2f}  ,y:{self.y:.2f}  ,yaw:{self.yaw:.2f}  ,state:{self.state}  ')
    def JoyCallback(self,msg):
        self.control.y = round(msg.axes[0],2)
        self.control.x = round(msg.axes[1],2)
        self.control.yaw = round(msg.axes[2],2)
        if msg.buttons[0] == 1:
            self.state = 'running'
        elif msg.buttons[1] == 1:
            self.state = 'crawl'
        elif msg.buttons[3] == 1:
            self.state = 'jump'
        if self.state == 'jump':
            self.control.x = 2.
    
    def RunModel(self):
        # self.obs.dof_pos = self.recived_data[0:12]
        # self.obs.dof_vel = self.recived_data[12:24]
        # self.obs.gravity_vec = self.recived_data[24:27]
        # self.obs.ang_vel = self.recived_data[27:30]
    

        logging.info(f"sub: {self.recived_data}")

        clamped_actions = self.Forward()
        # clamped_actions 0:FL_hip 1:FL_thigh 3:FL_calf FR RL RR 

        for i in self.params.hip_scale_reduction_indices:
            clamped_actions[0][i] *= self.params.hip_scale_reduction

        self.obs.actions = clamped_actions

        origin_output_torques = self.ComputeTorques(self.obs.actions)

        # self.TorqueProtect(origin_output_torques)

        self.output_torques = torch.clamp(origin_output_torques, -(self.params.torque_limits), self.params.torque_limits)
        ########### pub data ############
        self.output_dof_pos = self.ComputePosition(self.obs.actions)
        self.robot_command.commands = self.output_dof_pos.squeeze(0).numpy().tolist()

    def RobotControl(self):
        # self.port.send_data(self.output_dof_pos.squeeze(0).numpy().tolist())
        # logging.info(f"pub: {self.output_dof_pos.squeeze(0).numpy().tolist()}")
        self.GetState(self.robot_state)
        self.SetCommand(self.robot_command)

    def GetState(self,state):
        self.recived_data = self.port.receive_data()
        if self.params.framework == "isaacgym":
            state.imu.quaternion[3] = self.recived_data[27]
            state.imu.quaternion[0] = self.recived_data[24]
            state.imu.quaternion[1] = self.recived_data[25]
            state.imu.quaternion[2] = self.recived_data[26]
        
        state.imu.gyroscope[0] = self.recived_data[28]
        state.imu.gyroscope[1] = self.recived_data[29]
        state.imu.gyroscope[2] = self.recived_data[30]

        for i in range(self.params.num_of_dofs):
            state.motor_state.q[i] = self.recived_data[i]
            state.motor_state.dq[i] = self.recived_data[i+12]

    def SetCommand(self,command):
        # self.port.send_data(self.output_dof_pos.squeeze(0).numpy().tolist())
        self.port.send_data(command.commands)
    
    def Forward(self):
        # torch.set_grad_enabled(False)
        # clamped_obs = self.ComputeObservation()

        # # HIM
        # if len(self.params.observations_history) != 0:
        #     obs = self.obs_buff.insert(clamped_obs).to('cpu')
        #     actions = self.model(obs).detach()
        # else:
        #     actions = self.model.forward(clamped_obs)
        # if self.params.clip_actions_lower is not None and self.params.clip_actions_upper is not None:
        #     return torch.clamp(actions, self.params.clip_actions_lower, self.params.clip_actions_upper)
        # else:
        #     return actions
        torch.set_grad_enabled(False)
        clamped_obs = self.ComputeObservation()
        if self.state == 'running':
            if len(self.params.observations_history) != 0:
                obs = self.obs_buff.insert(clamped_obs).to('cpu')
                actions = self.model(obs).detach()
            else:
                actions = self.model.forward(clamped_obs)
            if self.params.clip_actions_lower is not None and self.params.clip_actions_upper is not None:
                return torch.clamp(actions, self.params.clip_actions_lower, self.params.clip_actions_upper)
            else:
                return actions
        elif self.state == 'crawl':
            if len(self.params.observations_history) != 0:
                obs = self.obs_buff.insert(clamped_obs).to('cpu')
                actions = self.crawl_model(obs).detach()
            else:
                actions = self.crawl_model.forward(clamped_obs)
            if self.params.clip_actions_lower is not None and self.params.clip_actions_upper is not None:
                return torch.clamp(actions, self.params.clip_actions_lower, self.params.clip_actions_upper)
            else:
                return actions
        elif self.state == 'jump':
            if len(self.params.observations_history) != 0:
                obs = self.obs_buff.insert(clamped_obs).to('cpu')
                actions = self.jump_model(obs).detach()
            else:
                actions = self.jump_model.forward(clamped_obs)
            if self.params.clip_actions_lower is not None and self.params.clip_actions_upper is not None:
                return torch.clamp(actions, self.params.clip_actions_lower, self.params.clip_actions_upper)
            else:
                return actions

    def ComputePosition(self, actions):
        actions_scaled = actions * self.params.action_scale
        return actions_scaled + self.params.default_dof_pos
    
    def ComputeTorques(self, actions):
        actions_scaled = actions * self.params.action_scale
        output_torques = self.params.rl_kp * (actions_scaled + self.params.default_dof_pos - self.obs.dof_pos) - self.params.rl_kd * self.obs.dof_vel
        return output_torques

    def InitObservations(self):
        self.obs.lin_vel = torch.zeros(1, 3, dtype=torch.float)
        self.obs.ang_vel = torch.zeros(1, 3, dtype=torch.float)
        self.obs.gravity_vec = torch.tensor([[0.0, 0.0, -1.0]])
        self.obs.commands = torch.zeros(1, 3, dtype=torch.float)
        self.obs.base_quat = torch.zeros(1, 4, dtype=torch.float)
        self.obs.dof_pos = self.params.default_dof_pos
        self.obs.dof_vel = torch.zeros(1, self.params.num_of_dofs, dtype=torch.float)
        self.obs.actions = torch.zeros(1, self.params.num_of_dofs, dtype=torch.float)

    def InitOutputs(self):
        self.output_torques = torch.zeros(1, self.params.num_of_dofs, dtype=torch.float)
        self.output_dof_pos = self.params.default_dof_pos

    def InitControl(self):
        self.control.x = 0.0
        self.control.y = 0.0
        self.control.yaw = 0.0

    def ComputeObservation(self):
        obs_list = []
        for observation in self.params.observations:
            """
                The first argument of the QuatRotateInverse function is the quaternion representing the robot's orientation, and the second argument is in the world coordinate system. The function outputs the value of the second argument in the body coordinate system.
                In IsaacGym, the coordinate system for angular velocity is in the world coordinate system. During training, the angular velocity in the observation uses QuatRotateInverse to transform the coordinate system to the body coordinate system.
                In Gazebo, the coordinate system for angular velocity is also in the world coordinate system, so QuatRotateInverse is needed to transform the coordinate system to the body coordinate system.
                In some real robots like Unitree, if the coordinate system for the angular velocity is already in the body coordinate system, no transformation is necessary.
                Forgetting to perform the transformation or performing it multiple times may cause controller crashes when the rotation reaches 180 degrees.
            """
            if observation == "lin_vel":
                obs_list.append(self.obs.lin_vel * self.params.lin_vel_scale)
            elif observation == "ang_vel":
                obs_list.append(self.obs.ang_vel * self.params.ang_vel_scale)
            # elif observation == "ang_vel_world":
            #     obs_list.append(self.QuatRotateInverse(self.obs.base_quat, self.obs.ang_vel, self.params.framework) * self.params.ang_vel_scale)
            elif observation == "gravity_vec": ### x,y,z 
                obs_list.append(self.obs.gravity_vec)
            elif observation == "commands":
                obs_list.append(self.obs.commands * self.params.commands_scale)
            elif observation == "dof_pos":
                obs_list.append((self.obs.dof_pos - self.params.default_dof_pos) * self.params.dof_pos_scale)
            elif observation == "dof_vel":
                obs_list.append(self.obs.dof_vel * self.params.dof_vel_scale)
            elif observation == "actions":
                obs_list.append(self.obs.actions)
        obs = torch.cat(obs_list, dim=-1)
        clamped_obs = torch.clamp(obs, -self.params.clip_obs, self.params.clip_obs)
        return clamped_obs
    
    def ThreadRL(self):
        thread_period = self.params.dt * self.params.decimation
        thread_name = "thread_rl"
        print(f"[Thread Start] named: {thread_name}, period: {thread_period * 1000:.0f}(ms), cpu unspecified")
        while not rospy.is_shutdown():
            self.RunModel()
            time.sleep(thread_period)
        print("[Thread End] named: " + thread_name)

    def ThreadControl(self):
        thread_period = self.params.dt
        thread_name = "thread_control"
        print(f"[Thread Start] named: {thread_name}, period: {thread_period * 1000:.0f}(ms), cpu unspecified")
        while not rospy.is_shutdown():
            self.RobotControl()
            time.sleep(thread_period)
        print("[Thread End] named: " + thread_name)

    def InitObservations(self):
        self.obs.lin_vel = torch.zeros(1, 3, dtype=torch.float)
        self.obs.ang_vel = torch.zeros(1, 3, dtype=torch.float)
        self.obs.gravity_vec = torch.tensor([[0.0, 0.0, -1.0]])
        self.obs.commands = torch.zeros(1, 3, dtype=torch.float)
        self.obs.base_quat = torch.zeros(1, 4, dtype=torch.float)
        self.obs.dof_pos = self.params.default_dof_pos
        self.obs.dof_vel = torch.zeros(1, self.params.num_of_dofs, dtype=torch.float)
        self.obs.actions = torch.zeros(1, self.params.num_of_dofs, dtype=torch.float)
    
    def ReadVectorFromYaml(self, values, framework, rows, cols):
        if framework == "isaacsim":
            transposed_values = [0] * cols * rows
            for r in range(rows):
                for c in range(cols):
                    transposed_values[c * rows + r] = values[r * cols + c]
            return transposed_values
        elif framework == "isaacgym":
            return values
        else:
            raise ValueError(f"Unsupported framework: {framework}")

    def ReadYaml(self, robot_name):
        # The config file is located at "rl_sar/src/rl_sar/models/<robot_name>/config.yaml"
        config_path = os.path.join(BASE_PATH, "models", robot_name, "config.yaml")
        try:
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)[robot_name]
        except FileNotFoundError as e:
            print(LOGGER.ERROR + f"The file '{config_path}' does not exist")
            return

        self.params.model_name = config["model_name"]
        self.params.crawl_name = config["crawl_name"]
        self.params.jump_name = config["jump_name"]
        self.params.framework = config["framework"]
        rows = config["rows"]
        cols = config["cols"]
        self.params.dt = config["dt"]
        self.params.decimation = config["decimation"]
        self.params.num_observations = config["num_observations"]
        self.params.observations = config["observations"]
        self.params.observations_history = config["observations_history"]
        self.params.clip_obs = config["clip_obs"]
        self.params.action_scale = config["action_scale"]
        self.params.hip_scale_reduction = config["hip_scale_reduction"]
        self.params.hip_scale_reduction_indices = config["hip_scale_reduction_indices"]
        if config["clip_actions_lower"] is None and config["clip_actions_upper"] is None:
            self.params.clip_actions_upper = None
            self.params.clip_actions_lower = None
        else:
            self.params.clip_actions_upper = torch.tensor(self.ReadVectorFromYaml(config["clip_actions_upper"], self.params.framework, rows, cols)).view(1, -1)
            self.params.clip_actions_lower = torch.tensor(self.ReadVectorFromYaml(config["clip_actions_lower"], self.params.framework, rows, cols)).view(1, -1)
        self.params.num_of_dofs = config["num_of_dofs"]
        self.params.lin_vel_scale = config["lin_vel_scale"]
        self.params.ang_vel_scale = config["ang_vel_scale"]
        self.params.dof_pos_scale = config["dof_pos_scale"]
        self.params.dof_vel_scale = config["dof_vel_scale"]
        self.params.commands_scale = torch.tensor([self.params.lin_vel_scale, self.params.lin_vel_scale, self.params.ang_vel_scale])
        self.params.rl_kp = torch.tensor(self.ReadVectorFromYaml(config["rl_kp"], self.params.framework, rows, cols)).view(1, -1)
        self.params.rl_kd = torch.tensor(self.ReadVectorFromYaml(config["rl_kd"], self.params.framework, rows, cols)).view(1, -1)
        self.params.fixed_kp = torch.tensor(self.ReadVectorFromYaml(config["fixed_kp"], self.params.framework, rows, cols)).view(1, -1)
        self.params.fixed_kd = torch.tensor(self.ReadVectorFromYaml(config["fixed_kd"], self.params.framework, rows, cols)).view(1, -1)
        self.params.torque_limits = torch.tensor(self.ReadVectorFromYaml(config["torque_limits"], self.params.framework, rows, cols)).view(1, -1)
        self.params.default_dof_pos = torch.tensor(self.ReadVectorFromYaml(config["default_dof_pos"], self.params.framework, rows, cols)).view(1, -1)
        self.params.joint_controller_names = self.ReadVectorFromYaml(config["joint_controller_names"], self.params.framework, rows, cols)

if __name__ == "__main__":
    rl_sim = RL_Real()
    rospy.spin()