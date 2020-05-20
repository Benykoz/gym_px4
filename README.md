# gym_px4
`gym-px4` is an openai/gym environment for PX4 Gazebo SITL using MAVROS. This environment enables the use of any gym RL library (e.g. baselines, stable-baselines, spinning-up, keras-rl etc) to train low-level quadcopter controllers.


# Installation
## Requirements

- python3.6 (or 3.7) environment.
- [ROS melodic]
      *to troubleshoot python3 - ROS issues, I recommend https://medium.com/@beta_b0t/how-to-setup-ros-with-python-3-44a69ca36674
- [gazebo9]
- [gym](https://github.com/openai/gym.git) 
mpi
opencv-python
joblib

Note1. the code was tested on Ubuntu 18.04.

## Install Dependencies:
follow the instructions to install gazebo with px4 sitl: https://dev.px4.io/v1.9.0/en/simulation/gazebo.html or by simply running this script: https://raw.githubusercontent.com/PX4/Devguide/v1.9.0/build_scripts/ubuntu_sim_ros_melodic.sh
make sure no errors are given and all dependencies are met.
run 'sudo pip3 install rospkg catkin_pkg'
run 'sudo pip3 install gym'


## Install gym_px4:
clone/download the repo and install the package using pip install -e gym_px4

# Using the Environment
Navigate to the px4 Firmware diractory and run gazebo with ros wrappers (instructions here: https://dev.px4.io/v1.9.0/en/simulation/ros_interface.html).
In a new terminal run `roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"`

Then the environment can be used as anyother gym environments. This can be done by doing the following in your script
```
import gym
...

env = gym.make('gym_px4:px4-v0')
...
env.reset()
...
env.step(action)
```

`gym_px4:px4-v0-v0` is the environment ID that is registered in the gym environment.
The environment can be also be tested using the openai baselines package, for example as the following.
```
python -m gym_reinmav.run --alg=ppo2 --env=gym_px4:px4-v0 --network=mlp --num_timesteps=2e7
```

## Additional notes

The environment gets information from different mavros topics and uses the /mavros/setpoint_raw/ publisher to control the drone. 

*** sometimes mavros has trouble switching to offboard mode, not sure what's the source of this bug but you can manually switch to offboard (e.g. using consule commander (run 'commander offboard' in px4 consule or simply using QGroundControl) ***

Action space = thrust[0..1], roll[-1..1], pitch[-1..1], yaw[-1..1]

Observation space = [ linear position[x,y,z], linear velocity[x,y,z], linear acceleration[x,y,z], angular_position[roll,pitch,yaw], angular_velocity[roll,pitch,yaw] ]

