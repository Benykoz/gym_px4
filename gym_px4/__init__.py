from gym.envs.registration import register

register(
    id='px4-v0',
    entry_point='gym_px4.envs:gymPX4',
)