#!/usr/bin/env python3

import math
import numpy as np
import rospy
import time
import subprocess
import sys
import asyncio
# from threading import Thread

# import pygazebo
# from pygazebo import Manager
# from pygazebo.msg import world_control_pb2
# from pygazebo.msg.world_control_pb2 import WorldControl
# from pygazebo.msg.world_stats_pb2 import WorldStatistics

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Vector3
from mavros_msgs.msg import ActuatorControl
from mavros_msgs.msg import AttitudeTarget
from mavros_msgs.msg import Thrust
from mavros_msgs.msg import State
from sensor_msgs.msg import Imu
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import SetModeRequest
from mavros_msgs.srv import SetModeResponse
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import CommandBoolRequest
from mavros_msgs.srv import CommandBoolResponse
from mavros_msgs.srv import StreamRate, StreamRateRequest
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from std_msgs.msg import Float64
from std_srvs.srv import Empty, EmptyRequest
import mavros.setpoint
from gazebo_msgs.msg import ModelStates

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

        self.wins = 0
        self.bounds_counter = 15
        self.bounds = np.exp(-self.bounds_counter*0.1)

        ### define gym spaces ###
        self.min_action = 0.0
        self.max_action = 1.0

        self.min_position = 0.1
        self.max_position = 25
        self.max_speed = 3
        self.max_accel = 20

        self.low_state = np.array([self.min_position, -self.max_speed, -self.max_accel])
        self.high_state = np.array([self.max_position, self.max_speed, self.max_accel])
        # self.low_state = np.array([self.min_position])
        # self.high_state = np.array([self.max_position])

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
        self.global_velocity = TwistStamped()
        self.ground_truth = ModelStates()
        self.local_accel = Imu()

        # rospy.signal_shutdown('Resrating rospy')
        # time.sleep(2)
        # subprocess.Popen(args='make px4_sitl gazebo', cwd='../Firmware', shell=True, start_new_session=True)
        # time.sleep(10)
        # subprocess.Popen(args='roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"', cwd='../Firmware', shell=True, start_new_session=True)
        # time.sleep(10)

        ### define ROS messages ###

        self.offb_set_mode = SetModeRequest()
        self.offb_set_mode.custom_mode = "OFFBOARD"

        self.arm_cmd = CommandBoolRequest()
        self.arm_cmd.value = True

        self.disarm_cmd = CommandBoolRequest()
        self.disarm_cmd.value = False

        ## ROS Subscribers
        self.state_sub = rospy.Subscriber("/mavros/state",State, self.state_cb,queue_size=10)
        self.imu_sub = rospy.Subscriber("/mavros/imu/data",Imu, self.imu_cb, queue_size=10)
        self.local_pos_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.lp_cb, queue_size=10)
        self.local_vel_sub = rospy.Subscriber("/mavros/local_position/velocity_local", TwistStamped, self.lv_cb, queue_size=10)
        self.local_acc_sub = rospy.Subscriber("/mavros/imu/data", Imu, self.lacc_cb, queue_size=10)
        self.act_control_sub = rospy.Subscriber("/mavros/act_control/act_control_pub", ActuatorControl, self.act_cb,queue_size=10)

        self.global_alt_sub = rospy.Subscriber("/mavros/global_position/rel_alt", Float64, self.ra_cb, queue_size=10)
        self.global_pos_sub = rospy.Subscriber("/mavros/global_position/gp_vel", TwistStamped, self.gv_cb, queue_size=10)

        self.ground_truth_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.gt_cb, queue_size=10)


        ## ROS Publishers
        self.local_pos_pub = rospy.Publisher("/mavros/setpoint_position/local",PoseStamped,queue_size=10)
        self.mocap_pos_pub = rospy.Publisher("/mavros/mocap/pose",PoseStamped,queue_size=10)
        self.acutator_control_pub = rospy.Publisher("/mavros/actuator_control",ActuatorControl,queue_size=10)
        self.setpoint_raw_pub = rospy.Publisher("/mavros/setpoint_raw/attitude",AttitudeTarget,queue_size=10)
        self.thrust_pub = rospy.Publisher("/mavros/setpoint_attitude/thrust",Thrust,queue_size=10)
        

        ## initiate gym enviornment 

        self.init_env()

    
        ## ROS Services

        rospy.wait_for_service('mavros/cmd/arming')
        self.arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)

        rospy.wait_for_service('mavros/set_mode')
        self.set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)

        rospy.wait_for_service('mavros/set_stream_rate')
        set_stream_rate=rospy.ServiceProxy("mavros/set_stream_rate",StreamRate)

        set_stream_rate(StreamRateRequest.STREAM_POSITION, 50, 1)
        set_stream_rate(StreamRateRequest.STREAM_ALL, 50, 1)

        self.setpoint_msg = mavros.setpoint.PoseStamped(header=mavros.setpoint.Header(frame_id="att_pose",stamp=rospy.Time.now()),)

        self.offb_arm()

  
	
    def init_env(self):

        ### Initiate ROS node
        print('-- Connecting to mavros')
        rospy.init_node('gym_px4_mavros',anonymous=True)
        print ('connected')

    def reset(self):

        # unpause()

        self.success_steps = 0
        self.steps=0
        reset_steps = 0

        # self.desired = np.random.randint(2,10)
        # self.initial = np.random.randint(2,10)

        ### sim limitations and objectives:
        self.desired = 2
        self.initial = 2

        print('current bound: ',np.exp(-self.bounds_counter*0.1) , 'bound wins: ' , self.wins)

        if self.wins > 10:
            self.bounds_counter += 1
            self.bounds = np.exp(-self.bounds_counter*0.1)
            self.wins = 0

        print('Initial: ', self.initial, 'Desired: ', self.desired)

        reset_pos=[0,0,self.initial,0]

        print('-- Resetting position')
        
		### wait for quad to arrive to desired position
        while True:

            if self.current_state.armed == False:    ## if not armed (e.g. crash landing disarm), re-arm
                self.offb_arm()
            
            self.setpoint_msg.pose.position.x = reset_pos[0]
            self.setpoint_msg.pose.position.y = reset_pos[1]
            self.setpoint_msg.pose.position.z = reset_pos[2]

            self.local_pos_pub.publish(self.setpoint_msg)

            x=self.local_position.pose.position.x
            y=self.local_position.pose.position.y
            z=self.local_position.pose.position.z
            lin_pos = [x,y,z]

            vx=self.local_velocity.twist.linear.x
            vy=self.local_velocity.twist.linear.y
            vz=self.local_velocity.twist.linear.z
            lin_vel = [vx,vy,vz]

            acx=self.local_accel.linear_acceleration.x
            acy=self.local_accel.linear_acceleration.y
            acz=self.local_accel.linear_acceleration.z
            lin_accel = [acx,acy,acz]
            
            if np.abs(np.linalg.norm(lin_pos[2] - reset_pos[2])) < 0.2 and np.abs(np.linalg.norm(lin_vel)) < 0.2 :   ### wait for drone to reach desired position
                time.sleep(0.2)
                break
            
            print('Resetting position: ', reset_steps, '/100')
            sys.stdout.write("\033[F")
            reset_steps+=1
            time.sleep(0.5)

        print('-------- Position reset --------') 

        ob = np.array([ self.desired - lin_pos[2] , lin_vel[2] , lin_accel[2] ]) 

        self.last_pos = lin_pos

        return ob  # reward, done, info can't be included

    def step(self, action):

        start_time=time.time()
        rate = rospy.Rate(20)


		### recieve updated position and velocity
        qx=self.local_position.pose.orientation.x
        qy=self.local_position.pose.orientation.y
        qz=self.local_position.pose.orientation.z
        qw=self.local_position.pose.orientation.w

        roll, pitch, yaw = quaternion_to_euler(qx,qy,qz,qw)
        ang_pos = [roll, pitch, yaw]

        while True:
            # st = time.time()
            x=self.local_position.pose.position.x
            y=self.local_position.pose.position.y
            z=self.local_position.pose.position.z
            lin_pos = [x,y,z]
            if lin_pos[2] != self.last_pos[2]:
                self.last_pos = lin_pos
                break
    
        vx=self.local_velocity.twist.linear.x
        vy=self.local_velocity.twist.linear.y
        vz=self.local_velocity.twist.linear.z
        lin_vel = [vx,vy,vz]

        acx=self.local_accel.linear_acceleration.x
        acy=self.local_accel.linear_acceleration.y
        acz=self.local_accel.linear_acceleration.z
        lin_accel = [acx,acy,acz]

        real_lin_pos = [self.ground_truth.pose[2].position.x , self.ground_truth.pose[2].position.y , self.ground_truth.pose[2].position.z]
        real_ang_pos = [self.ground_truth.pose[2].orientation.x , self.ground_truth.pose[2].orientation.y , self.ground_truth.pose[2].orientation.z]
        real_lin_vel = [self.ground_truth.twist[2].linear.x , self.ground_truth.twist[2].linear.y , self.ground_truth.twist[2].linear.z]
        real_ang_vel = [self.ground_truth.twist[2].angular.x , self.ground_truth.twist[2].angular.y , self.ground_truth.twist[2].angular.z]

        # gvx=self.global_velocity.twist.linear.x
        # gvy=self.global_velocity.twist.linear.y
        # gvz=self.global_velocity.twist.linear.z
        # glob_lin_vel = [gvx,gvy,gvz]

        # rel_alt = self.relative_altitude

		# ### send actuator control commands
        # self.act_controls.group_mix=0
        # self.act_controls.controls[0]=0
        # self.act_controls.controls[1]=0
        # self.act_controls.controls[2]=0
        # self.act_controls.controls[3]=action
        # self.acutator_control_pub.publish(self.act_controls)

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

        reward = -100*np.power( self.desired - real_lin_pos[2], 2) -60*np.power( real_lin_vel[2], 2)

        ob = np.array([ self.desired - lin_pos[2] , lin_vel[2], lin_accel[2] ])
        
        done = False
        reset = 'No'

        if  np.abs(real_lin_pos[0]) > 1.5 or np.abs(real_lin_pos[1]) > 1.5 or lin_pos[2] > 2+1 or lin_pos[2] < 2-1 :
            done = True
            reset = 'out of region'
            reward = -10000
            # self.wins-=1
            print('----------------', reset, '----------------')

        if self.steps > 5000 :
            done = True
            reset = 'limit time steps'
            # self.wins-=1
            print('----------------', reset ,'----------------')

        # if  np.abs( self.desired - real_lin_pos[2] ) < self.bounds*0.5: # and np.abs(real_lin_vel[2]) < 0.2:
        #     self.success_steps+=1
        #     if self.success_steps > 50:
        #         done = True
        #         reset = 'sim success'
        #         print('----------------', reset, '----------------')
        #         # reward += 10000
        #         self.wins+=1
                
        
        self.steps=self.steps+1

        self.last_pos = lin_pos

        step_prelen=time.time()-start_time

        if step_prelen < 0.03:
            time.sleep(0.03-step_prelen)

        step_len=time.time()-start_time

        rate.sleep()

        print('state: ', ob , 'action: ', action , 'reward: ', reward, 'time: ', step_len)
        # print(2-self.bounds, '/' , lin_pos[2], '/' , 2+self.bounds)

        info = {"state" : ob, "action": action, "reward": reward, "step": self.steps, "step length": step_len, "reset reason": reset}
        # print('action: ', action, 'state: ', [real_lin_pos[2]-self.desired ,real_lin_vel[2] ] , 'reward: ', reward)  
        return ob, reward, done, info

    def offb_arm(self):

        last_request = rospy.Time.now()

        flag1 = False
        flag2 = False

        prev_imu_data = Imu()
        prev_time = rospy.Time.now()
        count = 0

        # if self.current_state.mode == "OFFBOARD" and self.current_state.armed == True:
        #     return

        print ('-- Enabling offboard mode and arming')

        # while not rospy.is_shutdown():   ## quad initiation - set mode and arm
            
        #     if self.current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_request > rospy.Duration(5.0)):
        #         offb_set_mode_response = self.set_mode_client(self.offb_set_mode)
        #         if offb_set_mode_response.mode_sent:
        #             flag1 = True
        #         last_request = rospy.Time.now()
        #     else:
        #         if self.current_state.armed == False:
        #             arm_cmd_response = self.arming_client(self.arm_cmd)
        #             if arm_cmd_response.success :
        #                 flag2 = True
        #             last_request = rospy.Time.now()

        #     if flag1 and flag2:
        #         print("-- Offboard enabled")
        #         print("-- Vehicle armed")
        #         break 

        rospy.wait_for_service('mavros/set_mode')
        self.set_mode_client(0,'OFFBOARD')
        self.arming_client(True)

        # if self.current_state.mode != "OFFBOARD" or self.current_state.armed == False:
        #     while True:   ### change to offboard mode
        #         if self.current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_request > rospy.Duration(5.0)):
        #             offb_set_mode_response = self.set_mode_client(self.offb_set_mode)
        #             if offb_set_mode_response.mode_sent:
        #                 flag1 = True
        #         if self.current_state.armed == False:
        #             arm_cmd_response = self.arming_client(self.arm_cmd)
        #             if arm_cmd_response.success :
        #                 flag2 = True
        #         if flag1 and flag2:
        #             print("-- Offboard enabled")
        #             print("-- Vehicle armed")
        #             break

        rospy.loginfo('-- Ready to fly')

    def render(self):
        pass

    def close (self):
        pass

    def lacc_cb(self,data):
        self.local_accel = data

    def lv_cb(self,data):
        self.local_velocity = data

    def lp_cb(self,data):
        self.local_position = data

    def state_cb(self,data):
        self.current_state = data

    def imu_cb(self,data):
        self.imu_data = data

    def act_cb(self,data):
        self.act_controls = data

    def gv_cb(self,data):
        self.global_velocity = data

    def ra_cb(self,data):
        self.relative_altitude = data

    def gt_cb(self,data):
        self.ground_truth = data
    
    # def pause(self):
    #     self.loop.run_until_complete(self.pause_asyn())

    # def unpause(self):
    #     self.loop.run_until_complete(self.unpause_asyn())
        
    # async def pause_asyn(self):
    #     await self.pub_world_control.publish(self.pause_msg)

    # async def unpause_asyn(self):
    #     await self.pub_world_control.publish(self.unpause_msg)

    # async def step_asyn(self):
    #     await self.pub_world_control.publish(self.step_msg)

        

