#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "controller/rob_param.h"
#include "controller/pose.h"
#include "controller/interface.h"
#include <cmath>
#include <atomic>
//void chatterCallBack(const robot_joy::pose::ConstPtr& msg)
//the gear initialed;

//dynamixel_gear:舵机调速器，robot_gear:机器人调速器//
int dynamixel_gear = 0;
int robot_gear = 0;

//power_on=false means joystick is defaultly disabled, mode =false means robot is defaultly on joint space
bool power_on=false;
bool mode = false;
double threshold_down = 0.5;
double threshold_up = 0.9;
//coordinate: value1 means Local coordinate system,value0 means world coordinate system
int coordinate = 0;

//attention,vmax need to be verified
double va_percent=0;

//the directiong to move the robot
int d=1; 
std::vector<std::string> cmd_vec;

//std::string cmd;
std::atomic_bool en_cmd=false;

class listener
{
public:
	listener();

	void robotCallBack(const controller::rob_param::ConstPtr& msg);
	ros::NodeHandle nlistener_;
	ros::Subscriber robot_sub;
};
listener::listener()
{
	robot_sub = nlistener_.subscribe<controller::rob_param>("robot_joy_topic", 1, &listener::robotCallBack, this);
}
void listener::robotCallBack(const controller::rob_param::ConstPtr& msg)
{
	ROS_INFO("getting robot xbox command");
	cmd_vec.clear();

	//按钮误操作检查//
	{
		int num_b = 0;
		if (msg->x == 1) num_b += 1;
		if (msg->y == 1) num_b += 1;
		if (msg->z == 1) num_b += 1;
		if (msg->rx == 1) num_b += 1;
		if (msg->ry == 1) num_b += 1;
		if (msg->rz <= -0.9) num_b += 1;
		if (abs(msg->j7) > threshold_up) num_b += 1;
		if (msg->select_mode == 1) num_b += 1;
		if (msg->start == 1) num_b += 1;
		if (abs(msg->forward_back) > threshold_up) num_b += 1;
		if (msg->gear == 1) num_b += 1;
		if (msg->rs_button == 1) num_b += 1;
		if (msg->md_ds_button == 1) num_b += 1;
		if (msg->movee <= -0.9) num_b += 1;
		if (num_b > 2)
		{
			std::cout << "listener: Please don't hold down more than two buttons at the same time." << std::endl;
			std::cout << "The robot is in safety mode.The gear is zeroed.Please increase gear." << std::endl;
			//dynamixel_gear = 0;
			//robot_gear = 0;
			return;
		}
	}

	// 开启、关闭手柄 //
	if (msg->start == 1)
	{
		power_on = !power_on;
	}

	// 手柄开启 //
	if (power_on)
	{
		//选择控制目标，mode=0：机械臂关节+外部轴；mode=1：机械臂tcp+外部轴//
		if (msg->select_mode == 1)
		{
			mode = !mode;
			if(mode == false)
			{
				//机械臂joint//
				std::cout<<"robot joint control"<<std::endl;
			}
			else
			{
				//机械臂tcp//
				std::cout<<"robot terminal control"<<std::endl;
			}
			//每次切换模式，速度档位清零//
			dynamixel_gear = 0;
			robot_gear = 0;
		}

		// 机械臂关节控制 //
		if (mode == false)
		{
			std::cout << "joint control" << std::endl;
			// 手柄加减档 //
			if (msg->gear == 1)
			{
				robot_gear += 1;
			}
			else if (msg->gear == -1)
			{
				robot_gear -= 1;
			}
			robot_gear = std::max(0, robot_gear);
			robot_gear = std::min(5, robot_gear);
			//使能//
			if (msg->md_ds_button == 1)
			{
				cmd_vec.push_back("md");
				cmd_vec.push_back("en");
				cmd_vec.push_back("rc");
				std::cout<<"enable"<<std::endl;
			}
			//去使能//
			if (msg->md_ds_button == -1)
			{
				cmd_vec.push_back("ds");
				std::cout<<"disable"<<std::endl;
			}
			//复位//
			if (msg->rs_button == 1 &&
				msg->md_ds_button == 0 &&
				abs(msg->j7) <= threshold_down &&
				msg->x == 0 &&
				msg->y == 0 &&
				msg->z == 0 &&
				msg->rx == 0 &&
				msg->ry == 0 &&
				msg->rz >= -0.9 &&
				msg->movee >= -0.9 &&
				msg->select_mode == 0 &&
				msg->start == 0 &&
				msg->gear == 0 &&
				abs(msg->forward_back) < threshold_down)
			{
				cmd_vec.push_back("rs");
			}

			va_percent = abs(msg->forward_back) * robot_gear *0.2 * 100;
			if (msg->forward_back >= threshold_up) d = 1;
			else if (msg->forward_back < -threshold_up) d = -1;

			if (msg->j1 == 1 && 
				abs(msg->forward_back) > threshold_up &&
				abs(msg->j7) <= threshold_down &&
				msg->rs_button == 0 &&
				msg->md_ds_button == 0 &&
				msg->j2 == 0 &&
				msg->j3 == 0 &&
				msg->j4 == 0 &&
				msg->j5 == 0 &&
				msg->j6 >= -0.9 &&
				msg->movee >= -0.9 &&
				msg->select_mode == 0 &&
				msg->start == 0 &&
				msg->gear == 0
				)
			{
				cmd_vec.push_back("j1 --direction=" + std::to_string(d) + " --vel_percent=" + std::to_string(va_percent));
			}
			else if (msg->j2 == 1 && 
				abs(msg->forward_back) > threshold_up &&
				abs(msg->j7) <= threshold_down &&
				msg->rs_button == 0 &&
				msg->md_ds_button == 0 &&
				msg->j1 == 0 &&
				msg->j3 == 0 &&
				msg->j4 == 0 &&
				msg->j5 == 0 &&
				msg->j6 >= -0.9 &&
				msg->movee >= -0.9 &&
				msg->select_mode == 0 &&
				msg->start == 0 &&
				msg->gear == 0
				)
			{
				cmd_vec.push_back("j2 --direction=" + std::to_string(d) + " --vel_percent=" + std::to_string(va_percent));
			}
			else if (msg->j3 == 1 && 
				abs(msg->forward_back) > threshold_up &&
				abs(msg->j7) <= threshold_down &&
				msg->rs_button == 0 &&
				msg->md_ds_button == 0 &&
				msg->j1 == 0 &&
				msg->j2 == 0 &&
				msg->j4 == 0 &&
				msg->j5 == 0 &&
				msg->j6 >= -0.9 &&
				msg->movee >= -0.9 &&
				msg->select_mode == 0 &&
				msg->start == 0 &&
				msg->gear == 0
				)
			{
				cmd_vec.push_back("j3 --direction=" + std::to_string(d) + " --vel_percent=" + std::to_string(va_percent));
			}
			else if (msg->j4 == 1 && 
				abs(msg->forward_back) > threshold_up &&
				abs(msg->j7) <= threshold_down &&
				msg->rs_button == 0 &&
				msg->md_ds_button == 0 &&
				msg->j1 == 0 &&
				msg->j2 == 0 &&
				msg->j3 == 0 &&
				msg->j5 == 0 &&
				msg->j6 >= -0.9 &&
				msg->movee >= -0.9 &&
				msg->select_mode == 0 &&
				msg->start == 0 &&
				msg->gear == 0
				)
			{
				cmd_vec.push_back("j4 --direction=" + std::to_string(d) + " --vel_percent=" + std::to_string(va_percent));
			}
			else if (msg->j5 == 1 && 
				abs(msg->forward_back) > threshold_up &&
				abs(msg->j7) <= threshold_down &&
				msg->rs_button == 0 &&
				msg->md_ds_button == 0 &&
				msg->j1 == 0 &&
				msg->j2 == 0 &&
				msg->j3 == 0 &&
				msg->j4 == 0 &&
				msg->j6 >= -0.9 &&
				msg->movee >= -0.9 &&
				msg->select_mode == 0 &&
				msg->start == 0 &&
				msg->gear == 0
				)
			{
				cmd_vec.push_back("j5 --direction=" + std::to_string(d) + " --vel_percent=" + std::to_string(va_percent));
			}
			else if (msg->j6 <= -0.9 && 
				abs(msg->forward_back) > threshold_up &&
				abs(msg->j7) <= threshold_down &&
				msg->rs_button == 0 &&
				msg->md_ds_button == 0 &&
				msg->j1 == 0 &&
				msg->j2 == 0 &&
				msg->j3 == 0 &&
				msg->j4 == 0 &&
				msg->j5 >= -0.9 &&
				msg->movee >= -0.9 &&
				msg->select_mode == 0 &&
				msg->start == 0 &&
				msg->gear == 0
				)
			{
				cmd_vec.push_back("j6 --direction=" + std::to_string(d) + " --vel_percent=" + std::to_string(va_percent));
			}
			else
			{
			std::cout << "no cmd in joint control mode" << std::endl;
			}
		}

		// 机械臂tcp控制 //
		if (mode == true)
		{
			std::cout << "tcp control" << std::endl;
			// 手柄加减档 //
			if (msg->gear == 1)
			{
				robot_gear += 1;
			}
			else if (msg->gear == -1)
			{
				robot_gear -= 1;
			}
			robot_gear = std::max(0, robot_gear);
			robot_gear = std::min(5, robot_gear);
			//使能//
			if (msg->md_ds_button == 1 &&
				msg->rs_button == 0 &&
				abs(msg->j7) <= threshold_down &&
				msg->x == 0 &&
				msg->y == 0 &&
				msg->z == 0 &&
				msg->rx == 0 &&
				msg->ry == 0 &&
				msg->rz >= -0.9 &&
				msg->movee >= -0.9 &&
				msg->select_mode == 0 &&
				msg->start == 0 &&
				msg->gear == 0 &&
				abs(msg->forward_back) < threshold_down)
			{
				cmd_vec.push_back("md");
				cmd_vec.push_back("en");
				cmd_vec.push_back("rc");
			}
			//去使能//
			if (msg->md_ds_button == -1)
			{
				cmd_vec.push_back("ds");
			}
			//复位//
			if (msg->rs_button == 1 &&
				msg->md_ds_button == 0 &&
				abs(msg->j7) <= threshold_down &&
				msg->x == 0 &&
				msg->y == 0 &&
				msg->z == 0 &&
				msg->rx == 0 &&
				msg->ry == 0 &&
				msg->rz >= -0.9 &&
				msg->movee >= -0.9 &&
				msg->select_mode == 0 &&
				msg->start == 0 &&
				msg->gear == 0 &&
				abs(msg->forward_back) < threshold_down)
			{
				cmd_vec.push_back("mve0");
			}

			va_percent = abs(msg->forward_back) * robot_gear *0.2 * 100;

			// 0:世界坐标系 1:工具坐标系 //
			coordinate = 0;

			//the value 1 means the robot is on the positive direction
			if (msg->forward_back > 0.00) d = 1;
			else d = -1;
			if (msg->x == 1 && 
				abs(msg->forward_back) > threshold_up &&
				abs(msg->j7) <= threshold_down &&
				msg->rs_button == 0 &&
				msg->md_ds_button == 0 &&
				msg->y == 0 &&
				msg->z == 0 &&
				msg->rx == 0 &&
				msg->ry == 0 &&
				msg->rz >= -0.9 &&
				msg->movee >= -0.9 &&
				msg->select_mode == 0 &&
				msg->start == 0 &&
				msg->gear == 0
				)
			{
				cmd_vec.push_back("jx --direction=" + std::to_string(d) + " --cor=" + std::to_string(coordinate) + " --vel_percent=" + std::to_string(va_percent));
			}
			else if (msg->y == 1 && 
				abs(msg->forward_back) > threshold_up &&
				abs(msg->j7) <= threshold_down &&
				msg->rs_button == 0 &&
				msg->md_ds_button == 0 &&
				msg->x == 0 &&
				msg->z == 0 &&
				msg->rx == 0 &&
				msg->ry == 0 &&
				msg->rz >= -0.9 &&
				msg->movee >= -0.9 &&
				msg->select_mode == 0 &&
				msg->start == 0 &&
				msg->gear == 0
				)
			{
				cmd_vec.push_back("jy --direction=" + std::to_string(d) + " --cor=" + std::to_string(coordinate) + " --vel_percent=" + std::to_string(va_percent));
			}
			else if (msg->z == 1 && 
				abs(msg->forward_back) > threshold_up &&
				abs(msg->j7) <= threshold_down &&
				msg->rs_button == 0 &&
				msg->md_ds_button == 0 &&
				msg->x == 0 &&
				msg->y == 0 &&
				msg->rx == 0 &&
				msg->ry == 0 &&
				msg->rz >= -0.9 &&
				msg->movee >= -0.9 &&
				msg->select_mode == 0 &&
				msg->start == 0 &&
				msg->gear == 0
				)
			{
				cmd_vec.push_back("jz --direction=" + std::to_string(d) + " --cor=" + std::to_string(coordinate) + " --vel_percent=" + std::to_string(va_percent));
			}
			else if (msg->rx == 1 && 
				abs(msg->forward_back) > threshold_up &&
				abs(msg->j7) <= threshold_down &&
				msg->rs_button == 0 &&
				msg->md_ds_button == 0 &&
				msg->x == 0 &&
				msg->y == 0 &&
				msg->z == 0 &&
				msg->ry == 0 &&
				msg->rz >= -0.9 &&
				msg->movee >= -0.9 &&
				msg->select_mode == 0 &&
				msg->start == 0 &&
				msg->gear == 0
				)
			{
				cmd_vec.push_back("jrx --direction=" + std::to_string(d) + " --cor=" + std::to_string(coordinate) + " --vel_percent=" + std::to_string(va_percent));
			}
			else if (msg->ry == 1 && 
				abs(msg->forward_back) > threshold_up &&
				abs(msg->j7) <= threshold_down &&
				msg->rs_button == 0 &&
				msg->md_ds_button == 0 &&
				msg->x == 0 &&
				msg->y == 0 &&
				msg->z == 0 &&
				msg->rx == 0 &&
				msg->rz >= -0.9 &&
				msg->movee >= -0.9 &&
				msg->select_mode == 0 &&
				msg->start == 0 &&
				msg->gear == 0)
			{
				cmd_vec.push_back("jry --direction=" + std::to_string(d) + " --cor=" + std::to_string(coordinate) + " --vel_percent=" + std::to_string(va_percent));
			}
			else if (msg->rz < -0.9 && 
				abs(msg->forward_back) > threshold_up &&
				abs(msg->j7) <= threshold_down &&
				msg->rs_button == 0 &&
				msg->md_ds_button == 0 &&
				msg->x == 0 &&
				msg->y == 0 &&
				msg->z == 0 &&
				msg->rx == 0 &&
				msg->ry == 0 &&
				msg->movee >= -0.9 &&
				msg->select_mode == 0 &&
				msg->start == 0 &&
				msg->gear == 0)//the default value of msg.rz & msg.j6 is 1
			{
				cmd_vec.push_back("jrz --direction=" + std::to_string(d) + " --cor=" + std::to_string(coordinate) + " --vel_percent=" + std::to_string(va_percent));
			}
			else
			{
				std::cout << "no cmd in tcp control mode" <<std::endl;
			}
		}
		en_cmd.store(true);
	}
	else
	{
		std::cout << "xbox is disabled, please press start button to enable" <<std::endl;
		en_cmd.store(false);
	}
}
void call_service(ros::ServiceClient & cmd_client,std::string cmd_in)
{
	controller::interface srv;
	srv.request.cmd = cmd_in;
	if (cmd_client.call(srv))
	{
		ROS_INFO("Sum: %ld", (long int)srv.response.return_code);
	}
	else
	{
		return;
	}
	return;
}

int main(int argc,char ** argv)
{
	ros::init(argc, argv, "listener");
	//subscriber//
	listener listener_node;
	//ros::Rate loop_rate(20);
	ros::Rate loop_rate(40);

	//service client//
	ros::ServiceClient cmd_client = listener_node.nlistener_.serviceClient<controller::interface>("getcmd");

	while (ros::ok())
  	{
        if(en_cmd.load())
        {
            for(int i=0; i<cmd_vec.size(); i++)
            {
				call_service(cmd_client,cmd_vec[i]);
            }
        }
		ros::spinOnce();
		loop_rate.sleep();
  	}
	return 0;
}
