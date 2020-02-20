#!/usr/bin/env python3

import os
from stable_baselines.sac.policies import MlpPolicy as sac_MlpPolicy
from stable_baselines.ddpg.policies import MlpPolicy as ddpg_MlpPolicy
from stable_baselines.common.policies import MlpPolicy as Common_MlpPolicy

from stable_baselines.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise, AdaptiveParamNoiseSpec
from stable_baselines import TRPO
from stable_baselines import DDPG
from stable_baselines import PPO1
from stable_baselines import SAC
from stable_baselines import logger
from os import system
import gym
import time
import numpy as np

n_steps=0
save_interval = 500
best_mean_reward = -np.inf

def save_fn(_locals, _globals):
    global model, n_steps, best_mean_reward, best_model_path, last_model_path
    if (n_steps + 1) % save_interval == 0:
        
        # Evaluate policy training performance
        reward_buffer=np.array(_locals['episode_rewards'])
        mean_reward = np.mean(reward_buffer)
        print(n_steps + 1, 'timesteps')
        print("Best mean reward: {:.2f} - Last mean reward: {:.2f}".format(best_mean_reward, mean_reward))

        # New best model, save the agent
        if mean_reward > best_mean_reward:
            best_mean_reward = mean_reward
            print("Saving new best model")
            model.save(best_model_path+'_rew_'+str(np.round(best_mean_reward,2)))
        else:
            model.save(last_model_path)
    n_steps+=1
    pass

def main():
    global model, best_model_path, last_model_path
    env = gym.make('gym_px4:px4-v0')
    train_model = True

    if train_model:

        for k in range(133,138):

            best_model_path = '/home/graphics/gym_px4_mavros_stable_bl/model_dir/sac_simple_{}'.format(k)
            last_model_path = '/home/graphics/gym_px4_mavros_stable_bl/model_dir/sac_simple_{}'.format(k)   
            
            num_timesteps = int(1e6)

            log_dir = '/home/graphics/gym_px4_mavros_stable_bl/log_dir/sac_simple_{}'.format(k)

            # policy_kwargs = dict(layers = [4,4])

            # model = SAC(sac_MlpPolicy, env, gamma=0.99, learning_rate=3e-4, buffer_size=500000,
            #      learning_starts=1000, train_freq=8, batch_size=64,
            #      tau=0.005, ent_coef='auto', target_update_interval=4,
            #      gradient_steps=1, target_entropy='auto', action_noise=None,
            #      random_exploration=0.0, verbose=2, tensorboard_log=log_dir,
            #      _init_setup_model=True, policy_kwargs=None, full_tensorboard_log=True,
            #      seed=None, n_cpu_tf_sess=None)
            
            new_objects=dict(learning_starts = 0)
            model = SAC.load('/home/graphics/gym_px4_mavros_stable_bl/model_dir/sac_simple_133', env=env, custom_objects=new_objects)
            
            model.learn(total_timesteps=num_timesteps, callback=save_fn)

            # model = PPO1(Common_MlpPolicy, env, gamma=0.99, timesteps_per_actorbatch=256, clip_param=0.2, entcoeff=0.01,
            #      optim_epochs=4, optim_stepsize=1e-3, optim_batchsize=64, lam=0.95, adam_epsilon=1e-5,
            #      schedule='linear', verbose=0, tensorboard_log=None, _init_setup_model=True,
            #      policy_kwargs=None, full_tensorboard_log=False, seed=None, n_cpu_tf_sess=1)


            # log_dir = '/home/graphics/gym_px4_installed_sb/log_dir/ddpg_p_{}'.format(str(k))
            # model = TRPO(MlpPolicy, env, timesteps_per_batch=4096, tensorboard_log=log_dir, verbose=1)
            # model.learn(total_timesteps=500000)
            # model.save("/home/graphics/gym_px4_installed_sb/model_dir/trpo_gym_px4_p_{}".format(str(k)))

            # action_noise = AdaptiveParamNoiseSpec()

            # model = PPO1(policy=MlpPolicy, env=env, gamma=0.99, timesteps_per_actorbatch=512, clip_param=0.2, entcoeff=0.01,  
            #      optim_epochs=32, optim_stepsize=1e-3, optim_batchsize=64, lam=0.95, adam_epsilon=1e-5,  
            #      schedule='linear', verbose=0, tensorboard_log=log_dir, _init_setup_model=True,  
            #      policy_kwargs=policy_kwargs, full_tensorboard_log=True, seed=k, n_cpu_tf_sess=None)


    else:
            
        # model = SAC.load('/home/graphics/gym_px4_mavros_stable_bl/model_dir/sac_simple_133_rew_5351.05', env=env)

        model = SAC.load('/home/graphics/gym_px4_mavros_stable_bl/model_dir/sac_simple_135', env=env)
            
        for _ in range(20):

            obs = env.reset()
            done = False
            while not done:
                action, _states =  model.predict(obs) #model.predict(np.array([0,0.0895]))#
                obs, reward, done, info = env.step(action)
                print('state: ', obs, 'action: ', action)


if __name__ == '__main__':
    main()
