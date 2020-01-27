import os
from baselines.common import tf_util as U
from baselines import logger
import numpy as np
from mpi4py import MPI
from baselines.ppo1 import mlp_policy, pposgd_simple
from baselines.ppo1.pposgd_simple import traj_segment_generator
from ppo_util import build_policy_training_vars, build_counters, adjust_policy_learning_rate, update_policy, log_iter_info, calc_end_training
from policy_setup import policy_init
import gym

best_mean_reward = -np.inf
n_steps = 0
best_model_path = '_'
last_model_path = '_'
save_interval = 50

def save_fn(_locals, _globals):

    global n_steps, best_mean_reward, best_model_path, last_model_path
    # save model every save_interval itereations
    if (n_steps + 1) % save_interval == 0:

        # Evaluate policy training performance
        reward_buffer=np.array(_locals['rewbuffer'])
        mean_reward = np.mean(reward_buffer)
        print(n_steps + 1, 'timesteps')
        print("Best mean reward: {:.2f} - Last mean reward: {:.2f}".format(best_mean_reward, mean_reward))

        # New best model, save the agent
        if mean_reward > best_mean_reward:
            best_mean_reward = mean_reward
            # Example for saving best model
            print("Saving new best model")
            U.save_variables(best_model_path+'rew_'+str(np.round(best_mean_reward,2)))
        # else:
    U.save_variables(last_model_path)
    n_steps += 1
    return True

def train(num_timesteps, model_path, train_model):

    global best_model_path, last_model_path

    best_model_path = model_path+'_best'
    last_model_path = model_path+'_last'

    U.make_session(num_cpu=8).__enter__()

    env = gym.make('gym_px4:px4-v0')


    def policy_fn(name, ob_space, ac_space):
        return mlp_policy.MlpPolicy(name=name, ob_space=ob_space, ac_space=ac_space,
            hid_size=32, num_hid_layers=2)

    # Hyperperameters
    clip_param=0.2
    entcoeff=0.0
    clip_param=0.2
    entcoeff=0.0
    adam_epsilon=1e-5

    # initiate the policies and loss functions
    pi, oldpi, loss_names, var_list, lossandgrad, adam, assign_old_eq_new, compute_losses, mpi_moments_fn, allgather_fn = policy_init(env = env, policy_fn = policy_fn, clip_param = clip_param, entcoeff = entcoeff, adam_epsilon = adam_epsilon)

    if not train_model:
        return pi

    # start learning
    pi = learn(pi=pi, oldpi=oldpi, env=env, max_timesteps=num_timesteps,
            loss_names=loss_names, var_list=var_list,
            lossandgrad=lossandgrad, adam=adam,
            assign_old_eq_new=assign_old_eq_new, compute_losses=compute_losses,
            mpi_moments_fn=mpi_moments_fn, allgather_fn=allgather_fn,
            callback=save_fn
        )
    return pi

def learn(pi, oldpi, env,
        max_timesteps,
        loss_names, var_list,
        lossandgrad, adam,
        assign_old_eq_new, compute_losses,
        mpi_moments_fn, allgather_fn, callback,
        max_episodes=0,
        max_iters=0,
        max_seconds=0,  # time constraint
        timesteps_per_actorbatch=4096,
        optim_epochs=10,
        optim_stepsize=3e-4,
        optim_batchsize=64,
        gamma=0.99,
        lam=0.95,
        schedule='linear'
        ):

    # Prepare for rollouts
    # ----------------------------------------
    seg_gen = traj_segment_generator(pi, env, timesteps_per_actorbatch, stochastic=True)

    iters_so_far, episodes_so_far, timesteps_so_far, tstart, lenbuffer, rewbuffer = build_counters()

    assert sum([max_iters>0, max_timesteps>0, max_episodes>0, max_seconds>0])==1, "Only one time constraint permitted"

    while True:
        if callback: callback(locals(), globals())

        if calc_end_training(max_timesteps, timesteps_so_far,
                             max_episodes, episodes_so_far,
                             max_iters, iters_so_far,
                             max_seconds, tstart):
            break

        logger.log("********** Iteration %i ************"%iters_so_far)

        seg = seg_gen.__next__()

        cur_lrmult = adjust_policy_learning_rate(schedule, max_timesteps, timesteps_so_far, max_episodes, episodes_so_far, max_iters, iters_so_far)
        vpredbefore, tdlamret, optim_batchsize = update_policy(pi, seg, gamma, lam,
                                                     logger, optim_epochs, optim_batchsize, optim_stepsize, cur_lrmult,
                                                     loss_names, lossandgrad, adam, assign_old_eq_new, compute_losses,
                                                     mpi_moments_fn)

        episodes_so_far, timesteps_so_far = log_iter_info(lenbuffer, rewbuffer, tstart,
                                                          vpredbefore, tdlamret, seg,
                                                          episodes_so_far, timesteps_so_far,
                                                          MPI.COMM_WORLD.Get_rank()==0, allgather_fn)
        iters_so_far += 1

    return pi
