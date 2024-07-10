This repository realizes an ROS interface for mujoco simulation. It makes mujoco simulation as a ROS node and communicates with other ROS node via ROS topics.


### How to compile

mkdir build
cd build && cmake .. && cmake --build .

cd _dep/ambot_msgs-build && make

cd ../../

cmake --build .

