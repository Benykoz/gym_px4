from baselines.common import tf_util as U
from ppo_util import build_policy_training_vars
from mpi4py import MPI
from baselines.common.mpi_moments import mpi_moments

def policy_init(env, policy_fn, clip_param, entcoeff, adam_epsilon):
    ob_space = env.observation_space
    ac_space = env.action_space
    pi = policy_fn("pi", ob_space, ac_space) # Construct network for new policy
    oldpi = policy_fn("oldpi", ob_space, ac_space) # Network for old policy

    loss_names, var_list, lossandgrad, adam, assign_old_eq_new, compute_losses = build_policy_training_vars(pi, oldpi, clip_param, entcoeff, adam_epsilon)
    mpi_moments_fn = lambda losses: mpi_moments(losses, axis=0)
    allgather_fn = MPI.COMM_WORLD.allgather

    U.initialize()
    adam.sync()

    return pi, oldpi, loss_names, var_list, lossandgrad, adam, assign_old_eq_new, compute_losses, mpi_moments_fn, allgather_fn
