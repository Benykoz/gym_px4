# gym_px4
openai/gym wrapper for PX4 Gazebo SITL using MAVSDK 

This is a work in progress with two current branches - master branch with which the env.step fucntion only sends commands to the uav and the sim_step branch where the simulation is actualy discrete and env.step advances the sim one time step using pygazebo. However sometimes pygazebo causes gazebo to freeze and crash after several simulation. trying to find the problem and any help would be appriciated.


## Installation:
follow the instructions to install gazebo with px4 sitl: https://dev.px4.io/v1.9.0/en/simulation/gazebo.html or by simply running this script: https://raw.githubusercontent.com/PX4/Devguide/v1.9.0/build_scripts/ubuntu_sim_ros_melodic.sh

clone/download the repo and install the package using pip install -e gym-px4

-- Not neccessery currently: modify the max_step_size, real_time_factor and real_time_update_rate in /Firmware/Tools/sitl_gazebo/worlds/iris.world :

      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>

## Additional info
For the moment the environment is modified for contorl on the z axis only.

you can easily modify for control around all axis by modifying gymPX4_env.py


Each time the simulation is reset a new initial and desired heigh is generated.

Action space = [pitch rate[-1..1], roll rate[-1..1], yaw rate[-1..1], thrust[0..1]] 

Observation space = [linear position[x,y,z], linear velocity[x,y,z], angular position[x,y,z], angular velocity[x,y,z]]


can be used to test position controllers along with gymfc (https://github.com/wil3/gymfc)

## Lunching the example:
run the simulation using 'make px4_sitl gazebo' (also works in headless mode)
run PD_test.py
