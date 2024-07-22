This repository realizes an ROS interface for mujoco simulation. It makes mujoco simulation as a ROS node and communicates with other ROS node via ROS topics.


### How to compile

mkdir build
cd build && cmake .. && cmake --build .

cd _dep/ambot_msgs-build && make

cd ../../

cmake --build .



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
  
  `
