import asyncio
import sys
import time
import gym
import pygazebo
import os
import subprocess

def main():

	env = gym.make('gym_px4:px4-v0')
	time.sleep(1)
	ob = env.reset()

	num_tries = 1
	Kp = 0.05
	Kd = 0.05
	rewards=0
	while True:
		
		action = 0.564 + Kp*ob[0] - Kd*ob[1]

		# ob, reward, done, info = env.step(action)
		ob, reward, done, info = env.step(action)
		rewards=rewards+reward
		
		print('z_error: ', ob)
		sys.stdout.write("\033[F")

		if done:
			print('try number: ', num_tries, 'total reward: ', rewards, 'reset reason: ', info['reset reason'])
			print('nest run')
			ob = env.reset()
			rewards=0
			num_tries+=1

if __name__ == "__main__":
	main()