#!/usr/bin/env python3

import os
from baselines.common import tf_util as U
from baselines.common.tf_util import load_variables
import numpy as np
from plot_step_response import plot_step_response
from train_agent import train
from os import system
import gym
import time


def main():
    num_timesteps = 1e7
    train_model = True

    if train_model:
        print('-------------Initiate Learning----------------')
        conf_dir='/home/graphics/gym_px4_installed/log_dir/log_'+str('2')
        from baselines import logger
        logger.configure(dir=conf_dir, format_strs=['stdout', 'log', 'csv', 'tensorboard'])

        model_path = '/home/graphics/gym_px4_installed/model_dir/model_2'
        # train the model
        train(num_timesteps=num_timesteps, model_path=model_path, train_model=train_model)
    else:
        model_path = '/home/graphics/gym_px4_installed/model_dir/model_test_last'

        # construct the model object, load pre-trained model and render

        pi = train(num_timesteps=1, train_model=train_model)
        U.load_variables(model_path)

        env = gym.make('gym_px4:px4-v0')

        # best_err = np.inf
        # worst_err = 0

        tot_sim_num = 5

        for k in range(1,tot_sim_num+1):  # test for 10 runs, plot best run

            ob = env.reset()
            actuals = []
            desireds = []
            print('Begin Sim num: {}'.format(k))
            time.sleep(4)

            while True:
                # desired = env.omega_target
                # actual = env.omega_actual
                # actuals.append(actual)
                # desireds.append(desired)
                # print ("Desired rate=", desired, " Actual rate=", actual)
                action = pi.act(stochastic=False, ob=ob)[0]
                # if disturb:
                #     action=wind(action, info[0])
                ob, _, done, info =  env.step(action)
                print(info)
                if done:
                    break

        #     if env_id == 'AttFC_GyroErr-MotorVel_M4_Ep-v0':    # test episodal enviorment
        #         err_num = sum(np.abs(sum(np.array(desireds)-np.array(actuals))))
        #         err_den = sum(np.abs(np.array(desireds[0])-np.array(actuals[0])))*len(desireds)
        #         err = err_num/err_den
        #         print ('Norm error =', err)
        #         if err < best_err:
        #             print ('New best run')
        #             best_err = err
        #             best_des = desireds
        #             best_act = actuals
        #         if err > worst_err:
        #             print ('New worst run')
        #             worst_err = err
        #             worst_des = desireds
        #             worst_act = actuals

        #     if env_id == 'AttFC_GyroErr-MotorVel_M4_Con-v0':    # test continous enviorment
        #         np.savetxt('des_seed_{}_run{}.csv'.format(seed,_), np.array(best_des), delimiter=',')
        #         np.savetxt('act_seed_{}_run{}.csv'.format(seed,_), np.array(best_act), delimiter=',')

        # if env_id == 'AttFC_GyroErr-MotorVel_M4_Ep-v0':
        #     plot_step_response(desired=np.array(best_des), actual=np.array(best_act), title='best', seed=seed)
        #     plot_step_response(desired=np.array(worst_des), actual=np.array(worst_act), title='worst', seed=seed)


if __name__ == '__main__':
    main()
