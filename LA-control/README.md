# Quad-rotor-Project
RBE501 Robot Dynamics Project
## How to run
Using  roslaunch quadrotor_project.launch world_file_name:='world_file_name' to launch the gazebo simulation, for example:

roslaunch quadrotor_project.launch world_file_name:='world_demo_xy.world'

In another terminal, enable the motor service: 

rosservice call /enable_motors "enable: true"

then run: roslaunch 

ref_publisher reference.launch world_file_name:='world_file_name'

for example: 

roslaunch ref_publisher reference.launch world_file_name:='world_demo_xy.world'

