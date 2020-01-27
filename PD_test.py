import asyncio
import sys
import time
import gym
import pygazebo
import os
import subprocess

def main():
	# os.system("cd .. Firmware")
	# os.system("make px4_sitl gazebo")
	# subprocess.call("cd ~/Firmware", shell=True)
	# subprocess.call("make /Firmware/px4_sitl gazebo", shell=True)
	
	env = gym.make('gym_px4:px4-v0')
	time.sleep(1)
	ob = env.reset()

	try:
		num_tries = 1
		Kp = 0.02
		Kd = 0.05
		rewerds=0
		while True:

			action = 0.565 + Kp*ob[0] + Kd*ob[1]

			# ob, reward, done, info = env.step(action)
			ob, reward, done, info = env.step(action)
			rewerds=rewerds+reward
			sys.stdout.write("\033[F")
			print('z_error: ', ob,'reward: ', reward, 'done: ', done)
			

			if done:
				print('try number: ', num_tries, 'reset reason: ', info)
				ob = env.reset()
				rewerds=0
				num_tries+=1
	except:
		env.land()

	######
	# reset_msg = WorldControl()
    # reset_msg.reset.model_only = True
    # await asyncio.sleep(0.001)
    # await pub_world_control.publish(reset_msg)

if __name__ == "__main__":
	main()
