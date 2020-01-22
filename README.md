## gym_px4
openai/gym wrapper for PX4 Gazebo SITL using MAVSDK and pygazebo

work in progress - at the moment gazebo freezes after a while, maybe due to asyncio overloading, working on a fix.

Installation:
follow the instructions to install gazebo with px4 sitl: https://dev.px4.io/v1.9.0/en/simulation/gazebo.html 
clone/download the repo and install the package using pip install -e gym-px4


Example PD height controller available.

For the moment the environment is modified for contorl on the z axis only.
you can easily modify for control around all axis by modifying gymPX4_env.py

Each time the simulation is reset a new initial and desired heigh is generated.


Action space = [pitch rate[-1..1], roll rate[-1..1], yaw rate[-1..1], thrust[0..1]] 
Observation space = [linear position[x,y,z], linear velocity[x,y,z], angular position[x,y,z], angular velocity[x,y,z]]


can be used to test position controllers along with gymfc (https://github.com/wil3/gymfc)


lunching the demo:
run the simulation using 'make px4_sitl gazebo' (also works in headless mode)
run PD_test.py
