#include "ros/ros.h"
#include "controller/interface.h"
#include <cstdlib>
#include "std_msgs/String.h"
#include "controller/pose.h"
#include "controller/setpos.h"
// #include "dynamixel_sdk/dynamixel_sdk.h"
// #include "stdio.h"
// #include "unistd.h"
// Baud rate 57142

void call_service2(ros::ServiceClient & client2,uint16_t cmd_in)
{
  controller::setpos srv;
  srv.request.cmd = cmd_in;
  client2.call(srv);
  return;
}


void chatterCallback(const controller::pose::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->pose.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fake_client");

  ros::NodeHandle n;
  //ros::ServiceClient client = n.serviceClient<controller::interface>("getcmd");
  ros::ServiceClient client2 = n.serviceClient<controller::setpos>("gettargetpos");
  ros::Rate loop_rate(1);
  ros::Subscriber sub = n.subscribe("kannh_topic", 1000, chatterCallback);
  //while (ros::ok())
  //{
    //std::string cmd="sh";
    //call_service(client,cmd);

    call_service2(client2,350);
    //ros::spinOnce();
    //loop_rate.sleep();

  //}
  return 0;
}