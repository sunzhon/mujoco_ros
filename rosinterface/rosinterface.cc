#include "rosinterface.h"


// variables init


int motor_num=12;
int sensor_num=48;

ros::Subscriber motorValueSub;
ros::Publisher sensorValuePub;
std::recursive_mutex motorvalue_mutex;
bool recive_action_flag = false;


std::vector<float> motorValue;
std::vector<float> kp;
std::vector<float> kd;
std::vector<float> sensorValue;




std::unique_ptr<ros::NodeHandle> RosInterface(int argc, char** argv){
    //1) ROS interface
    ros::init(argc, argv, "mujoco_sim");

    //2) .create multi receive thread for topic,avoid receive data block
    //3) .check if ros master node is ok
    if(!ros::master::check()){
        ROS_ERROR("ros::master::check() did not pass!");
        ros::shutdown();
        std::exit(1);
    }

    //4) instance rosNodeHandle
    std::unique_ptr<ros::NodeHandle> node = std::make_unique<ros::NodeHandle>();

    //5) define variables
    motorValue.resize(motor_num);
    kp.resize(motor_num);
    kd.resize(motor_num);
    sensorValue.resize(sensor_num);


    return node;
}


void controller(const mjModel* m, mjData* d)
{

    if( m->nu==motor_num){
        const std::unique_lock<std::recursive_mutex> lock(motorvalue_mutex);
        if(recive_action_flag){
            for(int idx=0;idx<motor_num;idx++){
                //1) control method 1: set desired dof pos
                //d->ctrl[idx] = motorValue[map_joints[idx]];
                //2) control method 2: set desired dof torque calculated via PD control
                d->ctrl[idx] = kp[idx]*(motorValue[idx] - d->qpos[idx+7]) - kd[idx] * d->qvel[idx+6];
                //printf("idx:%i, desired value: %f, error: %f, force: %f\n", idx, motorValue[map_joints[idx]], (motorValue[map_joints[idx]] - d->qpos[idx+7]), d->ctrl[idx]);
            }
        }
    }
}

void pubSensorThread(ros::NodeHandle* node, mj::Simulate* sim){
    auto rate = std::make_unique<ros::Rate>(200);
    // pub states topics
    {
        const std::unique_lock<std::recursive_mutex> lock(sim->mtx);
        sensorValuePub =node->advertise<ambot_msgs::RobotState>("/ambot_v1/states_mujoco", 1);
    }
    // waiting for model loaded
    while(!sim->m_) 
    {
        sleep(1);
    }
    ambot_msgs::RobotState msg;
    msg.motorState.resize(motor_num);
    printf("nq:%d\n",sim->m_->nq);
    while(1){
        if(sim->exitrequest.load()) {
            break;
        }
        if(sim->m_->nq>0){
            {    
                const std::unique_lock<std::recursive_mutex> lock(sim->mtx);
                // updata msg
                msg.motor_num = motor_num;

                // base linear velocity
                /*
                   msg.imu.linvel.x = sim->d_->qvel[0];
                   msg.imu.linvel.y = sim->d_->qvel[1];
                   msg.imu.linvel.z = sim->d_->qvel[2];

                // angular velocity/ gyroscope
                msg.imu.gyroscope.x = sim->d_->qvel[3];
                msg.imu.gyroscope.y = sim->d_->qvel[4];
                msg.imu.gyroscope.z = sim->d_->qvel[5];
                printf("sim gyro:%f, %f, %f\n", sim->d_->qvel[3], sim->d_->qvel[4],  sim->d_->qvel[5]);
                printf("imu gyro:%f, %f, %f\n", sim->d_->sensordata[0], sim->d_->sensordata[1], sim->d_->sensordata[2]);
                printf("up imu gyro:%f, %f, %f\n", sim->d_->sensordata[3], sim->d_->sensordata[4], sim->d_->sensordata[5]);
                printf("imu acce:%f, %f, %f\n", sim->d_->sensordata[6], sim->d_->sensordata[7], sim->d_->sensordata[8]);
                printf("sim ori:%f, %f, %f %f\n", sim->d_->qpos[3], sim->d_->qpos[4], sim->d_->qpos[5], sim->d_->qpos[6]);
                printf("imu ori:%f, %f, %f, %f\n", sim->d_->sensordata[6], sim->d_->sensordata[7], sim->d_->sensordata[8], sim->d_->sensordata[9]);

                // orientation
                msg.imu.quaternion.w = sim->d_->qpos[3];
                msg.imu.quaternion.x = sim->d_->qpos[4];
                msg.imu.quaternion.y = sim->d_->qpos[5];
                msg.imu.quaternion.z = sim->d_->qpos[6];
                */



                msg.imu.linvel.x = sim->d_->qvel[0];
                msg.imu.linvel.y = sim->d_->qvel[1];
                msg.imu.linvel.z = sim->d_->qvel[2];
                // angular velocity/ gyroscope
                msg.imu.gyroscope.x = sim->d_->sensordata[0];
                msg.imu.gyroscope.y = sim->d_->sensordata[1];
                msg.imu.gyroscope.z = sim->d_->sensordata[2];
                // acceleration
                msg.imu.acceleration.x = sim->d_->sensordata[3];
                msg.imu.acceleration.y = sim->d_->sensordata[4];
                msg.imu.acceleration.z = sim->d_->sensordata[5];
                // orientation
                msg.imu.quaternion.w = sim->d_->sensordata[6];
                msg.imu.quaternion.x = sim->d_->sensordata[7];
                msg.imu.quaternion.y = sim->d_->sensordata[8];
                msg.imu.quaternion.z = sim->d_->sensordata[9];


                // dof pos
                for(uint8_t idx=0;idx<motor_num;idx++){
                    msg.motorState[idx].pos =sim->d_->qpos[idx+7];
                }
                // dof vel
                for(uint8_t idx=0;idx<motor_num;idx++){
                    msg.motorState[idx].vel =sim->d_->qvel[idx+6];
                }
            }

            // pub sensor values
            sensorValuePub.publish(msg);
            if(!ros::ok()){
                ROS_ERROR("mujoco did not connect to ROS!");
                ros::shutdown();
                std::exit(1);
            }
        }
        rate->sleep();

    }
}


void motorValueCallback(const ambot_msgs::RobotAction msg){
    assert(msg.motorAction.size()==motor_num);
    const std::unique_lock<std::recursive_mutex> lock(motorvalue_mutex);
    for(uint8_t idx=0;idx<motor_num;idx++){
        motorValue[idx] = msg.motorAction[idx].pos;
        kp[idx] = msg.motorAction[idx].kp;
        kd[idx] = msg.motorAction[idx].kd;
    }
    recive_action_flag = true;
}


void subMotorThread(ros::NodeHandle *node, mj::Simulate* sim){
    auto  spinner = std::make_unique<ros::AsyncSpinner>(0);
    spinner->start();
    //subsciber
    {
        const std::unique_lock<std::recursive_mutex> lock(sim->mtx);
        motorValueSub = node->subscribe<ambot_msgs::RobotAction>("/ambot_v1/actions", 1, motorValueCallback);
    }
    while(!sim->exitrequest.load()) {
        sleep(1);
        if(!ros::ok()){
            ROS_ERROR("mujoco did not connect to ROS!");
            ros::shutdown();
        }
    }
}
    
