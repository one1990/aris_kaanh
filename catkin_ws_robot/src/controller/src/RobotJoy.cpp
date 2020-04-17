#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <sstream>
#include "controller/rob_param.h"

// create the RobotJoy class and define the joyCallback function that will take a joy msg
class RobotJoy
{
	public:
		RobotJoy();

		void joyrobotCallback(const sensor_msgs::Joy::ConstPtr& joy);
		ros::NodeHandle nh_;
		ros::Publisher robot_pub_;
		ros::Subscriber joy_robot_sub_;
};

RobotJoy::RobotJoy()
{
	// create a publisher that will publish the cmd from xbox //
	robot_pub_ = nh_.advertise<controller::rob_param>("robot_joy_topic", 1);

	// subscribe to the joystick topic //
	joy_robot_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 1, &RobotJoy::joyrobotCallback, this);
}

int x_j1=2, y_j2 = 3, z_j3 =5 ,rx_j4 = 0,ry_j5 = 1,rz_j6 = 5,j7 = 4, mve=2, forward_back=1,start=7,gear=7; 
int select_mode=6;
int rs_button = 4, md_ds_button = 6;

void RobotJoy::joyrobotCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	controller::rob_param msg;

    msg.x = joy->buttons[x_j1];
    msg.y = joy->buttons[y_j2];
    msg.z = joy->buttons[z_j3];
    msg.rx = joy->buttons[rx_j4];
    msg.ry = joy->buttons[ry_j5];
    msg.rz = joy->axes[rz_j6];
    msg.j1 = joy->buttons[x_j1];
    msg.j2 = joy->buttons[y_j2];
    msg.j3 = joy->buttons[z_j3];
    msg.j4 = joy->buttons[rx_j4];
    msg.j5 = joy->buttons[ry_j5];
    msg.j6 = joy->axes[rz_j6];//按下的数值为-1
	msg.j7 = joy->axes[j7];//向上按下，数值为1
    msg.select_mode = joy->buttons[select_mode];
    msg.start = joy->buttons[start];
    msg.forward_back = joy->axes[forward_back];//1前进；-1后退
    msg.gear = joy->axes[gear];
    msg.rs_button = joy->buttons[rs_button];
    msg.md_ds_button = joy->axes[md_ds_button];
	msg.movee = joy->axes[mve];
    robot_pub_.publish(msg);
}

int main(int argc, char** argv)
{
	sensor_msgs::Joy::ConstPtr joy;

	// initialize our ROS node, create a robot_joy, and spin our node until Ctrl-C is pressed
	ros::init(argc, argv, "RobotJoy");
	RobotJoy robot_joy;
	ros::Rate loop_rate(20);

	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
    //ros::spin();
}


