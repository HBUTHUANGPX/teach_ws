#include "ros/ros.h"
#include "test_1/self_msg_1.h"
#include "test_1/self_msg_2.h"
#include <unistd.h> // 包含usleep函数的头文件
#include <std_msgs/Bool.h>
void doMsg(const test_1::self_msg_1::ConstPtr& msg)
{
    ROS_INFO("message receive  ,%f",msg->height);
    usleep(990);
}

void doMsg2(const test_1::self_msg_2::ConstPtr& msg)
{
    ROS_INFO("message receive 2,%f",msg->wsnd.height);
    usleep(990);
}
int main(int argc, char **argv)
{
    ros::init(argc,argv,"sub_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<test_1::self_msg_1>("haha_chat",1,doMsg);
    ros::Subscriber sub2 = nh.subscribe<test_1::self_msg_2>("haha_chat2",1,doMsg2);
    ros::AsyncSpinner _subSpinner(2); 
    _subSpinner.start();
    ros::waitForShutdown();
    // ros::spin();
    return 0;
}
// Lidar 100Hz 9.9ms
// imu 200Hz 0.1ms