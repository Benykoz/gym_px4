#!/usr/bin/env python3

import math
import numpy as np
import rospy
import time
import subprocess
import sys
import asyncio
from threading import Thread

import pygazebo
from pygazebo import Manager
from pygazebo.msg import world_control_pb2
from pygazebo.msg.world_control_pb2 import WorldControl

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Vector3
from mavros_msgs.msg import ActuatorControl
from mavros_msgs.msg import AttitudeTarget
from mavros_msgs.msg import Thrust
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import SetModeRequest
from mavros_msgs.srv import SetModeResponse
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import CommandBoolRequest
from mavros_msgs.srv import CommandBoolResponse
from mavros_msgs.srv import StreamRate, StreamRateRequest
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from std_srvs.srv import Empty, EmptyRequest
import mavros.setpoint

import gym
from gym import spaces


def quaternion_to_euler(x, y, z, w):

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        X = math.degrees(math.atan2(t0, t1))

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = math.degrees(math.asin(t2))

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        Z = math.degrees(math.atan2(t3, t4))

        return X, Y, Z

class gymPX4(gym.Env):

    def __init__(self):

        ### define gym spaces ###
        self.min_action = 0.0
        self.max_action = 1.0

        self.min_position = 0.1
        self.max_position = 25
        self.max_speed = 2

        # self.low_state = np.array([self.min_position, -self.max_speed])
        # self.high_state = np.array([self.max_position, self.max_speed])
        self.low_state = np.array([self.min_position])
        self.high_state = np.array([self.max_position])

        self.action_space = spaces.Box(low = self.min_action, high=self.max_action, shape=(1,), dtype=np.float32)

        self.observation_space = spaces.Box(low = self.low_state, high = self.high_state, dtype = np.float32)

        self.current_state = State()
        self.imu_data = Imu()
        self.act_controls = ActuatorControl()
        self.pose = PoseStamped()
        self.mocap_pose = PoseStamped()
        self.thrust_ctrl = Thrust()
        self.attitude_target = AttitudeTarget()
        self.local_velocity = TwistStamped()

        ### define ROS messages ###

        self.offb_set_mode = SetModeRequest()
        self.offb_set_mode.custom_mode = "OFFBOARD"

        self.arm_cmd = CommandBoolRequest()
        self.arm_cmd.value = True

        self.disarm_cmd = CommandBoolRequest()
        self.disarm_cmd.value = False

        ### define pygazebo messages ###

        self.pause_msg = WorldControl()
        self.pause_msg.pause = True

        self.unpause_msg = WorldControl()
        self.unpause_msg.pause = False

        self.step_msg = WorldControl()
        self.step_msg.step = True

        ## ROS Subscribers
        self.state_sub = rospy.Subscriber("/mavros/state",State, self.state_cb,queue_size=10)
        self.imu_sub = rospy.Subscriber("/mavros/imu/data",Imu, self.imu_cb, queue_size=10)
        self.local_pos_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.lp_cb, queue_size=10)
        self.local_vel_sub = rospy.Subscriber("/mavros/local_position/velocity", TwistStamped, self.lv_cb, queue_size=10)
        self.act_control_sub = rospy.Subscriber("/mavros/act_control/act_control_pub", ActuatorControl, self.act_cb,queue_size=10)

        ## ROS Publishers
        self.local_pos_pub = rospy.Publisher("/mavros/setpoint_position/local",PoseStamped,queue_size=10)
        self.mocap_pos_pub = rospy.Publisher("/mavros/mocap/pose",PoseStamped,queue_size=10)
        self.acutator_control_pub = rospy.Publisher("/mavros/actuator_control",ActuatorControl,queue_size=10)
        self.setpoint_raw_pub = rospy.Publisher("/mavros/setpoint_raw/attitude",AttitudeTarget,queue_size=10)
        self.thrust_pub = rospy.Publisher("/mavros/setpoint_attitude/thrust",Thrust,queue_size=10)

        ## initiate gym enviornment and connect to pygazebo

        self.init_env()
        
        self.loop = asyncio.get_event_loop()
        self.loop.run_until_complete(self.reset_pygzb())
    
        ## ROS Services

        rospy.wait_for_service('mavros/cmd/arming')
        self.arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)

        rospy.wait_for_service('mavros/set_mode')
        self.set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)

        rospy.wait_for_service('mavros/set_stream_rate')
        set_stream_rate=rospy.ServiceProxy("mavros/set_stream_rate",StreamRate)

        set_stream_rate(stream_id=StreamRateRequest.STREAM_ALL, message_rate=20, on_off=True)

        self.setpoint_msg = mavros.setpoint.PoseStamped(header=mavros.setpoint.Header(frame_id="att_pose",stamp=rospy.Time.now()),)

        self.offb_arm()

    def init_env(self):

        ### Initiate ROS node
        rospy.init_node('gym_px4_mavros',anonymous=True)
        rate =  rospy.Rate(1000.0)

        print('-- Connecting to mavros')
        while not rospy.is_shutdown() and not self.current_state.connected:
            rate.sleep()
            rospy.loginfo(self.current_state.connected)
        # print("#"*80)
        print ('-- connected')

    async def reset_pygzb(self):
        print('-- Connecting to pygazbo')
        self.manager = await pygazebo.connect(('localhost', 11345))   ## connect to pygazebo
        print ('-- connected')
        self.pub_world_control = await self.manager.advertise('/gazebo/default/world_control','gazebo.msgs.WorldControl')
        await asyncio.sleep(0.5)

    def reset(self):

        # self.loop.run_until_complete(self.unpause_asyn())  ## unpause sim for reset position

        self.success_steps = 0
        self.steps=0

        self.desired = 4
        self.initial = 2

        print('Initial: ', self.initial, 'Desired: ', self.desired)

        reset_pos=[0,0,self.initial,0]

        print('-- Resetting position')     
		
        while True:   ### wait for quad to arrive to desired position

            if self.current_state.armed == False:    ## if not armed (e.g. crash landing disarm), re-arm
                self.offb_arm()
            
            self.setpoint_msg.pose.position.x = reset_pos[0]
            self.setpoint_msg.pose.position.y = reset_pos[1]
            self.setpoint_msg.pose.position.z = reset_pos[2]

            self.local_pos_pub.publish(self.setpoint_msg)  ### send desired set_point for reset

            x=self.local_position.pose.position.x
            y=self.local_position.pose.position.y
            z=self.local_position.pose.position.z
            lin_pos = [x,y,z]

            vx=self.local_velocity.twist.linear.x
            vy=self.local_velocity.twist.linear.y
            vz=self.local_velocity.twist.linear.z
            lin_vel = [vx,vy,vz]
            
            if np.abs(np.linalg.norm(lin_pos[2] - reset_pos[2])) < 0.2 and np.abs(np.linalg.norm(lin_vel)) < 0.2 :   ### wait for drone to reach desired position
                time.sleep(0.2)
                break

            time.sleep(0.5)

        print('-------- Position reset --------') 

        x=self.local_position.pose.position.x
        y=self.local_position.pose.position.y
        z=self.local_position.pose.position.z
        lin_pos = [x,y,z]
        ob = np.array([ self.desired - lin_pos[2] ])

        self.last_pos = lin_pos

        # self.loop.run_until_complete(self.pause_asyn())   ## pause sim for new episode

        return ob  # reward, done, info can't be included

    def step(self, action):

        start_time=time.time()

        # self.loop.run_until_complete(self.step_asyn())    ## perform one sim time step

		### recieve updated position
        qx=self.local_position.pose.orientation.x
        qy=self.local_position.pose.orientation.y
        qz=self.local_position.pose.orientation.z
        qw=self.local_position.pose.orientation.w

        roll, pitch, yaw = quaternion_to_euler(qx,qy,qz,qw)
        ang_pos = [roll, pitch, yaw]

        while True:
            x=self.local_position.pose.position.x
            y=self.local_position.pose.position.y
            z=self.local_position.pose.position.z
            lin_pos = [x,y,z]
            if lin_pos[2] != self.last_pos[2]:
                break


        ### send set_point attitude rate commands
        self.attitude_target.type_mask = AttitudeTarget.IGNORE_ROLL_RATE | AttitudeTarget.IGNORE_PITCH_RATE | AttitudeTarget.IGNORE_YAW_RATE | AttitudeTarget.IGNORE_ATTITUDE
        # attitude_target.type_mask = AttitudeTarget.IGNORE_ATTITUDE
        self.attitude_target.thrust = action
        # quad_rate = Vector3()
        # quad_rate.x = 0
        # quad_rate.y = 0
        # quad_rate.z = 0
        # attitude_target.body_rate = quad_rate

        self.setpoint_raw_pub.publish(self.attitude_target)

        reward = -np.power( self.desired - lin_pos[2], 2)

        ob = np.array([ self.desired - lin_pos[2] ])
        
        done = False
        reset = 'No'

        if  np.abs(lin_pos[0]) > 5 or np.abs(lin_pos[1]) > 5 or lin_pos[2] > 2.5 or lin_pos[2] < 1.5 :
            done = True
            reset = 'out of region'
            reward = -1000
            print('----------------', reset, '----------------')

        if self.steps > 5000 :
            done = True
            reset = 'limit time steps'
            print('----------------', reset ,'----------------')

        if  np.abs(ob[0]) < 0.1 :
            self.success_steps+=1
            if self.success_steps > 150:
                done = True
                reset = 'sim success'
                print('----------------', reset, '----------------')
                reward += 10000
        
        self.steps=self.steps+1

        self.last_pos = lin_pos

        step_len=time.time()-start_time

        info = {"state" : ob, "action": action, "reward": reward, "step": self.steps, "step length": step_len, "reset reason": reset}
        return ob, reward, done, info

    def offb_arm(self):

        if self.current_state.mode == "OFFBOARD" and self.current_state.armed == True:
            return
        print ('-- Enabling offboard mode and arming')
        self.set_mode_client(0,'OFFBOARD')
        self.arming_client(True)
        rospy.loginfo('-- Ready to fly')

    def render(self):
        pass

    def close (self):
        pass

    def lp_cb(self,data):
        self.local_position = data

    def state_cb(self,data):
        self.current_state = data

    def imu_cb(self,data):
        self.imu_data = data

    def act_cb(self,data):
        self.act_controls = data
    
    def pause(self):
        self.loop.run_until_complete(self.pause_asyn())

    def unpause(self):
        self.loop.run_until_complete(self.unpause_asyn())

    async def pause_asyn(self):
        await self.pub_world_control.publish(self.pause_msg)

    async def unpause_asyn(self):
        await self.pub_world_control.publish(self.unpause_msg)

    async def step_asyn(self):
        await self.pub_world_control.publish(self.step_msg)
