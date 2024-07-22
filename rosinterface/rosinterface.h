#ifndef ROS_INTERFACE_H_
#define ROS_INTERFACE_H_

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <new>
#include <string>
#include <thread>

#include <mujoco/mujoco.h>
#include "simulate.h"

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>

// for ros
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <ros/node_handle.h>
#include <std_msgs/Float64.h>
#include "std_msgs/Bool.h"
#include <std_msgs/Float32MultiArray.h>
#include <ros/spinner.h>
#include "ros/ros.h"
#include <sys/time.h>
#include <ambot_msgs/RobotState.h>
#include <ambot_msgs/RobotAction.h>

namespace mj=::mujoco;


extern "C" {
    void controller(const mjModel* m, mjData* d);
}
std::unique_ptr<ros::NodeHandle> RosInterface(int argc, char** argv);

void motorValueCallback(const ambot_msgs::RobotAction msg);

void pubSensorThread(ros::NodeHandle* node, mj::Simulate* sim);
void subMotorThread(ros::NodeHandle* node, mj::Simulate* sim);



#endif

