This is an RTT component that connects to Gazebo.
It allows writing torque commands and reading joint states.

:warning: GAZEBO_MODEL_PATH must contain the path to your model, or Gazebo will approximate your geometry as a pile of cubes.

Some of the Gazebo API functions used are specific to Gazebo8.

ROS Kinetic can be upgraded to Gazebo8 with: 

curl -ssL http://get.gazebosim.org | sh
sudo apt install ros-kinetic-gazebo8-*









