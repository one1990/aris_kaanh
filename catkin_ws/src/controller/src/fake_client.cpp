#include "ros/ros.h"
#include "controller/interface.h"
#include <cstdlib>
#include "std_msgs/String.h"
#include "controller/pose.h"

void call_service(ros::ServiceClient & client,std::string cmd_in)
{
  controller::interface srv;
  srv.request.cmd = cmd_in;
  if (client.call(srv))
  {
    ROS_INFO("Sum: %ld", (long int)srv.response.return_code);
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return;
  }
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
  ros::ServiceClient client = n.serviceClient<controller::interface>("getcmd");

  ros::Rate loop_rate(1);
  ros::Subscriber sub = n.subscribe("kannh_topic", 1000, chatterCallback);
  while (ros::ok())
  {
    std::string cmd="sh";
    call_service(client,cmd);

    ros::spinOnce();
    loop_rate.sleep();

  }
  return 0;
}