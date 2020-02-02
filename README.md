# gym_px4
openai/gym wrapper for PX4 Gazebo SITL using MAVSDK

###work in progress### 
The env.step fucntion only sends mavlink commands to the drone while the simulation runs freely. using Pygazebo we can pause the simulation and advance it step by step as a real gym interface should be, however using WorldControl().step from pygazebo seems to crash gazebo oftenly. any help would be appriciated.


## Installation:
follow the instructions to install gazebo with px4 sitl: https://dev.px4.io/v1.9.0/en/simulation/gazebo.html or by simply running this script: https://raw.githubusercontent.com/PX4/Devguide/v1.9.0/build_scripts/ubuntu_sim_ros_melodic.sh

clone/download the repo and install the package using pip install -e gym-px4

-- Not neccessery currently: modify the max_step_size, real_time_factor and real_time_update_rate in /Firmware/Tools/sitl_gazebo/worlds/iris.world :

      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>

## Additional notes
For the moment the environment is modified for control on the z axis only.

you can easily modify for control around all axis by modifying gymPX4_env.py

Each time the simulation is reset a new initial and desired height is generated.

Action space = [pitch rate[-1..1], roll rate[-1..1], yaw rate[-1..1], thrust[0..1]] 

Observation space = [linear position[x,y,z], linear velocity[x,y,z], angular position[x,y,z], angular velocity[x,y,z]]

can be used to test position controllers along with gymfc (https://github.com/wil3/gymfc)

there are 3 functions in the env that you cam use other then the default gym function:
env.land : lands the iris (assuming px4 is not exacuting any other command)
env.pause() : pauses gazebo
env.unpause() : unpauses gazebo    -  these can be used to pause simulation during offpolicy optimizationions

Off-policy optimization algorithms that take a few seconds to update the new policy each epoch result with no new signals sent to the gazebo sim. hence we use env.pause and env.unpause to stop the sim during policy updates.

## Lunching the example:
You can run 'make px4_sitl gazebo' from Firmware folder to see the px4 updates in a different consule.
If gazebo is not open the px4_gym initiation will open the default iris model using 'make px4_sitl gazebo'
run PD_test.py
