catkin_make

source devel/setup.bash

roslaunch quadog_gazebo quadog_world_moveit.launch

rosrun quadog_control_cpp main
