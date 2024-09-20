This repository realizes an ROS interface for mujoco simulation. It makes mujoco simulation as a ROS node and communicates with other ROS node via ROS topics.

The controller callback function of mujoco was realized in rosinterface/rosinterface.cc

The rostopic use a message named [ambot_msgs](https://github.com/sunzhon/ambot_msgs.git), which have RobotState and RobotAction for defining the robot states and commands.

This repository integrated this rosinterface into mujoco when compile it. 



### How to compile

mkdir build
cd build && cmake .. && cmake --build .

cd _dep/ambot_msgs-build && make

cd ../../

cmake --build .

### How to run 

1. open a terminal and run ``` roscore ```
2. open a terminal and run ``` cd ./build/bin/ &&./ros_simulate [your_robot_model.xml] [your_command_topic_name] ```

### Trouble shooting
- compile error:
mujoco_ros/rosinterface/rosinterface.h:40:10: fatal error: ambot_msgs/RobotState.h: No such file or directory
   40 | #include <ambot_msgs/RobotState.h>
      |          ^~~~~~~~~~~~~~~~~~~~~~~~~
compilation terminated.
- solution:
  go to build/_deps/ambot_msgs-build/ and run make, and then cd ./../../ to run cmake --build .
  ```
  cd _deps/ambot_msgs-build/ && make && cd ./../../ && cmake --build .
  
  ```
