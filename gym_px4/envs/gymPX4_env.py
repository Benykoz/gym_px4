#!/usr/bin/env python3
import gym
from gym import spaces

import numpy as np
import asyncio
import math

from mavsdk import System
from mavsdk import (OffboardError, PositionNedYaw, ActuatorControl, AttitudeRate)

import pygazebo
from pygazebo import Manager
from pygazebo.msg import world_control_pb2
from pygazebo.msg.world_control_pb2 import WorldControl

def lat_lon_to_coords(cords):
    radius_of_earth = 6371000
    rad_lat = math.radians(cords[0])
    rad_lon = math.radians(cords[1])
    x = radius_of_earth * math.cos(rad_lat) * math.cos(rad_lon)
    y = radius_of_earth * math.cos(rad_lat) * math.sin(rad_lon)
    z = cords[2]
    return x, y, z

async def is_armed():
        async for is_armed in drone.telemetry.armed():  ## check arm status
            return is_armed

async def init_env():

    global drone, manager, pub_world_control, home_pos, desired

    drone = System()
    await drone.connect(system_address="udp://:14550")   ## connect to mavsdk
    async for state in drone.core.connection_state():
        if state.is_connected:
            break
    await asyncio.sleep(1)
    
    print('-- Connecting to Gazebo')
    manager = await pygazebo.connect(('localhost', 11345))   ## connect to pygazebo
    await asyncio.sleep(1)
    print('-- Connected')
    pub_world_control = await manager.advertise('/gazebo/default/world_control','gazebo.msgs.WorldControl')
    await asyncio.sleep(1)

    async for home_pos in drone.telemetry.home():  ## get absolute home position
        home_pos = np.array([home_pos.latitude_deg, home_pos.longitude_deg, home_pos.absolute_altitude_m])
        home_pos=np.array(lat_lon_to_coords(home_pos))
        break

    asyncio.ensure_future(get_lin_pos())  ## initiate linear position stream
    asyncio.ensure_future(get_lin_vel())  ## initiate linear velocity stream

    armed = await is_armed()

    if not armed:  ## if not armed, arm and change to OFFBOARD mode
        print("-- Arming")
        await drone.action.arm()
        await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))
        print("-- Starting offboard")
        try:
            await drone.offboard.start()
        except OffboardError as error:
            print(f"Starting offboard mode failed with error code: {error._result.result}")
            print("-- Disarming")
            await drone.action.disarm()

async def reset_async(reset_pos):

    # unpause_msg = WorldControl()
    # unpause_msg.pause = False
    # await asyncio.sleep(0.001)
    # await pub_world_control.publish(unpause_msg)

    print('-- Resetting position')

    await drone.offboard.set_position_ned(PositionNedYaw(reset_pos[0],reset_pos[1],-reset_pos[2],reset_pos[3]))
    while True:
        # lin_pos = await get_lin_pos()
        # lin_vel = await get_lin_vel()
        armed = await is_armed()

        await asyncio.sleep(0.1)
        if not armed:  ## if not armed, arm and change to OFFBOARD mode
            print("-- Arming")
            await drone.action.arm()
            await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))
            await asyncio.sleep(0.1)
            print("-- Starting offboard")
            try:
                await drone.offboard.start()
            except OffboardError as error:
                print(f"Starting offboard mode failed with error code: {error._result.result}")
                print("-- Disarming")
                await drone.action.disarm()
            await asyncio.sleep(0.1)
            await drone.offboard.set_position_ned(PositionNedYaw(reset_pos[0],reset_pos[1],-reset_pos[2],reset_pos[3]))


        if np.abs(np.linalg.norm(lin_pos - reset_pos[0:3])) < 0.7 and np.abs(np.linalg.norm(lin_vel)) < 0.7 :
            await asyncio.sleep(1)
            break
    print('-- Position reset')

    # pause_msg = WorldControl()
    # pause_msg.pause = True
    # await asyncio.sleep(0.001)
    # await pub_world_control.publish(pause_msg)

async def step_async(action):

    # step_msg = WorldControl()
    # step_msg.step = True
    # await asyncio.sleep(0.001)
    # await pub_world_control.publish(step_msg)  ## perform pone step in gazebo


    ### for attitude contorl
    action = [0,0,0,action]
    ### for attitude contorl

    await drone.offboard.set_attitude_rate(AttitudeRate(action[0],action[1],action[2],action[3]))   ## publish action in deg/s, thrust [0:1]
    # lin_pos = await get_lin_pos()
    # lin_vel = await get_lin_vel()
    # return lin_pos, lin_vel

async def get_lin_pos():  ## in m
    async for position in drone.telemetry.position():
        global lin_pos
        glob_pos = np.array([position.latitude_deg, position.longitude_deg, position.absolute_altitude_m])
        lin_pos = np.array(lat_lon_to_coords(glob_pos)) - home_pos
        # return lin_pos

async def get_lin_vel():  ## in m/s
    async for vel in drone.telemetry.ground_speed_ned():
        global lin_vel
        lin_vel = np.array([vel.velocity_north_m_s, vel.velocity_east_m_s, vel.velocity_down_m_s])
        # return lin_vel

async def get_ang_pos(): ## in rad
    async for ang_pos in drone.telemetry.attitude_euler():
        return [ang_pos.roll_deg, ang_pos.pitch_deg, ang_pos.yaw_deg]

async def get_ang_vel():  ## in rad/s
    async for ang_vel in drone.telemetry.attitude_angular_velocity_body():
        return np.array([ang_vel.roll_rad_s, ang_vel.pitch_rad_s, ang_vel.yaw_rad_s])


async def asyland():
    print('-- Landing')
    # unpause_msg = WorldControl()
    # unpause_msg.pause = False
    # await asyncio.sleep(0.5)
    # await pub_world_control.publish(unpause_msg)
    await asyncio.sleep(0.5)
    await drone.action.land()

class gymPX4(gym.Env):
    steps = 0
    def __init__(self):
        self._start_sim()


        # self.min_action = -1.0
        # self.max_action = 1.0
        # self.min_position = -1.2
        # self.max_position = 0.6
        # self.max_speed = 0.07
        # self.goal_position = 0.45 # was 0.5 in gym, 0.45 in Arnaud de Broissia's version
        # self.goal_velocity = goal_velocity
        # self.power = 0.0015

        # self.low_state = np.array([self.min_position, -self.max_speed])
        # self.high_state = np.array([self.max_position, self.max_speed])

        # self.action_space = spaces.Box(low=self.min_action, high=self.max_action,
        #                                shape=(1,), dtype=np.float32)
        # self.observation_space = spaces.Box(low=self.low_state, high=self.high_state,
        #                                     dtype=np.float32)
        self.min_action = 0
        self.max_action = 1.0

        self.min_position = 0.1
        self.max_position = 25
        self.max_speed = 2

        self.low_state = np.array([self.min_position, -self.max_speed])
        self.high_state = np.array([self.max_position, self.max_speed])

        self.action_space = spaces.Box(low = self.min_action, high=self.max_action, shape=(1,), dtype=np.float32)

        self.observation_space = spaces.Box(low = self.low_state, high = self.high_state, dtype = np.float32)

        # self.observation_space = spaces.Box( np.array([1.0,-2.0]), np.array([20.0,2.0]), dtype=np.float32)
        # self.action_space = spaces.Box(0, 1, shape=(1,), dtype=np.float32)

    def _start_sim(self):
        global loop
        loop = asyncio.get_event_loop()
        loop.run_until_complete(init_env())
        # asyncio.run(init_env())

    def reset(self):
        global steps, desired
        self.steps=0

        # desired = np.random.randint(4,10)
        desired = np.round(np.random.rand()*10,2)+2

        # initial = np.random.randint(2,10)
        initial = np.round(np.random.rand()*10,2)+2

        reset_pos=[0,0,initial,0]
        print('Initial: ', initial, 'Desired: ', desired)

        # lin_pos, lin_vel = loop.run_until_complete(reset_async(reset_pos))
        loop.run_until_complete(reset_async(reset_pos))

        observation = [desired - lin_pos[2], lin_vel[2]]

        return observation  # reward, done, info can't be included

    def step(self, action):
        global  desired
        # lin_pos, lin_vel = loop.run_until_complete(step_async(action))
        loop.run_until_complete(step_async(action))
        
        reward = -np.abs( desired - lin_pos[2] ) - 2*np.abs( lin_vel[2] )
        ob = [desired - lin_pos[2], lin_vel[2]]

        done = False

        if  np.abs(lin_pos[0]) > 5 or np.abs(lin_pos[1]) > 5 or np.abs(lin_pos[2]) > 15 or np.abs(lin_pos[2]) < 0.5 :
            done = True
            info = '--------- out of bounds ----------'
            print(info)
        if self.steps > 25000 :
            done = True
            info = '--------- limit time steps ----------'
            print(info)
        if  np.abs(ob[0]) < 0.2 and np.abs(ob[1]) < 0.1 :
            done = True
            info = '---------- sim success ----------'
            print(info)
        info = {"state" : ob, "action": action, "reward": reward, "steps": self.steps}
        self.steps=self.steps+1
        return ob, reward, done, info

    def render(self):
        pass

    def close (self):
        pass

    def land(self):
        loop.run_until_complete(asyland())