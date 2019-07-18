#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "controller/rob_param.h"
#include "controller/pose.h"
#include "controller/interface.h"
#include <cmath>

    

//void chatterCallBack(const robot_joy::pose::ConstPtr& msg)
int vel_gear =0;//the gear initialed;
bool power_on=false, mode = false;
double threshold = 0.01;
//coordinate: value1 means Local coordinate system,value0 means world coordinate system
int coordinate = 1;
//attention,vmax need to be verified????????
double va_percent=0;
int d=1;//the directiong to move the robot 
std::string cmd;
int init=0, init1 =1;
double init_d=0.00,init_d1=1.00;

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
    ROS_ERROR("Failed to call service controller");
    return;
  }
  return;
}

void chatterCallBack(const controller::rob_param::ConstPtr& msg)//?????
{	
	ROS_INFO("listener recieve from joy:[%d]", msg->x);
	//.c_str()
	if(msg->gear ==1)
	{
		vel_gear += 1;

	}
	else if(msg->gear == -1)
	{
		vel_gear -= 1;
	}
	vel_gear = std::max(0,vel_gear);
	vel_gear = std::min(5,vel_gear);
	
	
	//ROS_INFO("rec:[%s]", msg->data.c_str());//???int64tostring?
	int num_b =0;
	if(msg->x ==1) num_b +=1;
	if(msg->y ==1) num_b +=1;
	if(msg->z ==1) num_b +=1;
	if(msg->rx ==1) num_b +=1;
	if(msg->ry ==1) num_b +=1;
	if(msg->rz <=-0.9) num_b +=1;
	if(num_b >2)
	{
		std::cout<<"listener: Please don't hold down more than two buttons at the same time.";
	}
    if (msg->start == 1)
    {
    	power_on = !power_on;
    }
    if (power_on)
    {
    	 	
    	if(msg->select_mode == 1)
    	{
    		mode = !mode;
    	}
    	std::cout<<"listener: mode:"<<mode<<std::endl;


		if(msg->md_ds_button == 1 && 
			abs(msg->start_stop)<threshold && 
			abs(msg->rc_en_button)<threshold && 
			msg->rs_button ==0 && msg->x ==0 && msg->y ==0 && msg->z ==0 && msg->rx ==0 && msg->ry ==0 && msg->rz >=-0.9 && msg->select_mode == 0 && msg->start == 0 && msg->gear == 0 && abs(msg->forward_back)<threshold)
		{
		cmd="md";

		}
		if(msg->md_ds_button == -1)
		{
		cmd ="ds";
		}
		if(msg->rc_en_button >=0.9 && abs(msg->start_stop)<threshold && msg->md_ds_button ==0 && msg->rs_button ==0 && msg->x ==0 && msg->y ==0 && msg->z ==0 && msg->rx ==0 && msg->ry ==0 && msg->rz >=-0.9 && msg->select_mode == 0 && msg->start == 0 && msg->gear == 0 && abs(msg->forward_back)<threshold)
		{
		cmd ="en";
		}
		if(msg->rc_en_button <=-0.9 && abs(msg->start_stop)<threshold  && msg->md_ds_button ==0 && msg->rs_button ==0 && msg->x ==0 && msg->y ==0 && msg->z ==0 && msg->rx ==0 && msg->ry ==0 && msg->rz >=-0.9 && msg->select_mode == 0 && msg->start == 0 && msg->gear == 0 && abs(msg->forward_back)<threshold)
		{
		cmd ="rc";
		}
		if(msg->rs_button ==1 && abs(msg->start_stop)<threshold && msg->md_ds_button == 0 && msg->x ==0 && msg->y ==0 && msg->z ==0 && msg->rx ==0 && msg->ry ==0 && msg->rz >=-0.9 && msg->select_mode == 0 && msg->start == 0 && msg->gear == 0 && abs(msg->forward_back)<threshold)
		{
		cmd ="rs --vel=0.02";
		} 

    		//enable joystick
    	if(mode)//on end space
    	{
    		
			std::cout<<"listener: now is on end space";   		
    		va_percent= abs(msg->forward_back) * vel_gear *0.2*100;
    		coordinate = 1;
    		if(msg->forward_back>0) d=1;
    		else d=-1;

    		if(msg->start_stop >= 0.9)
    		{
    			cmd ="movePoint --start";
    		}
    		else if(msg->start_stop <= -0.9)
    		{
    			cmd ="movePoint --stop";
    		}

    		else if(msg->x ==1 && abs(msg->forward_back) > threshold)
    		{
    			cmd ="movePoint --x="+std::to_string(d) +" --cor=" + std::to_string(coordinate) +" --vel_percent=" + std::to_string(va_percent);
    			//call_service(cmd_client,cmd);
    			//ros::spinOnce();
    		}
    		else if(msg->y ==1 && abs(msg->forward_back) > threshold)
    		{	
    			cmd ="movePoint --y="+std::to_string(d) +" --cor=" + std::to_string(coordinate) +" --vel_percent=" + std::to_string(va_percent);
    			//call_service(cmd_client,cmd);
    			//double v = msg.forward_back/32767.0000*V0;
    		}
    		else if(msg->z ==1 && abs(msg->forward_back) > threshold)
    		{
    			cmd ="movePoint --z="+std::to_string(d) +" --cor=" + std::to_string(coordinate) +" --vel_percent=" + std::to_string(va_percent);
    			//call_service(cmd_client,cmd);
    			//double v = msg.forward_back/32767.0000*V0;
    		}
    		else if(msg->rx ==1 && abs(msg->forward_back) > threshold)
    		{
    			
    			cmd ="movePoint --a="+std::to_string(d) +" --cor=" + std::to_string(coordinate) +" --vel_percent=" + std::to_string(va_percent);
    			//call_service(cmd_client,cmd);
    			//double v = msg.forward_back/32767.0000*V0;
    		}
    		else if(msg->ry ==1 && abs(msg->forward_back) > threshold)
    		{
    			
    			cmd ="movePoint --b="+std::to_string(d) +" --cor=" + std::to_string(coordinate) +" --vel_percent=" + std::to_string(va_percent);
    			//call_service(cmd_client,cmd);
    			//double v = msg.forward_back/32767.0000*V0;
    		}
    		else if(msg->rz <= -0.9 && abs(msg->forward_back) > threshold)//the default value of msg.rz & msg.j6 is 1
    		{
    			
    			cmd ="movePoint --c="+std::to_string(d) +" --cor=" + std::to_string(coordinate) +" --vel_percent=" + std::to_string(va_percent);
    			//call_service(cmd_client,cmd);
    			//double v = msg.forward_back/32767.0000*V0;
    		}
    		else
    		{
    			std::cout<<"listener: You can manipulate the direction key to control the robot.";
    		}
    	}
    	if(!mode)//on joint space
    	{
   			std::cout<<"listener: now is on joint space";
    		
    		va_percent= abs(msg->forward_back) * vel_gear *0.2*100;

    		if(msg->forward_back>0) d=1;
    		else d=-1;

    		if(msg->start_stop >= 0.9)
    		{
    			cmd ="moveJP --start";
    		}
    		else if(msg->start_stop <= -0.9)
    		{
    			cmd ="moveJP --stop";
    		}
    		else if(msg->j1==1 && abs(msg->forward_back) > threshold)
    		{
    			
    			cmd ="moveJP -m=0 --direction=" + std::to_string(d)+" --vel_percent=" + std::to_string(va_percent);
    			//call_service(cmd_client,cmd);
    			//double v = msg.forward_back/32767.0000*V0;
    		}
    		else if(msg->j2==1 && abs(msg->forward_back) > threshold)
    		{
    			
    			cmd ="moveJP -m=1 --direction=" + std::to_string(d)+" --vel_percent=" + std::to_string(va_percent);
    			//call_service(cmd_client,cmd);
    			//double v = msg.forward_back/32767.0000*V0;
    		}
    		else if(msg->j3 ==1 && abs(msg->forward_back) > threshold)
    		{
    			
    			cmd ="moveJP -m=2 --direction=" + std::to_string(d)+" --vel_percent=" + std::to_string(va_percent);
    			//call_service(cmd_client,cmd);
    			//double v = msg.forward_back/32767.0000*V0;
    		}
    		else if(msg->j4 ==1 && abs(msg->forward_back) > threshold)
    		{
    			
    			cmd ="moveJP -m=3 --direction=" + std::to_string(d)+" --vel_percent=" + std::to_string(va_percent);
    			//call_service(cmd_client,cmd);
    			//double v = msg.forward_back/32767.0000*V0;
    		}
    		else if(msg->j5 ==1 && abs(msg->forward_back) > threshold)
    		{
    			std::cout<<"j5:  "<<msg->j5<<std::endl;
    			cmd ="moveJP -m=4 --direction=" + std::to_string(d)+" --vel_percent=" + std::to_string(va_percent);
    			//call_service(cmd_client,cmd);
    			//double v = msg.forward_back/32767.0000*V0;
    		}
    		else if(msg->j6 <=-0.9 && abs(msg->forward_back) > threshold)
    		{
    			
    			cmd ="moveJP -m=5 --direction=" + std::to_string(d)+" --vel_percent=" + std::to_string(va_percent);
    			//call_service(cmd_client,cmd);
    			//double v = msg.forward_back/32767.0000*V0;
    		}
    		else
    		{
    			std::cout<<"listener: You can manipulate the direction key to control the robot.";
    		}
    	}

    	if( abs(msg->start_stop)<threshold &&
    		abs(msg->rc_en_button)<threshold && 
    		msg->md_ds_button == 0 && 
    		msg->rs_button ==0 && 
    		msg->x ==0 && 
    		msg->y ==0 && 
    		msg->z ==0 && 
    		msg->rx ==0 && 
    		msg->ry ==0 && 
    		msg->rz >=-0.9 && 
    		msg->select_mode == 0 && 
    		msg->start == 0 && 
    		msg->gear == 0 && 
    		abs(msg->forward_back)<threshold)
    	{
    		cmd = "";
    	}

    }
    else
    {
    	std::cout<<"listener: joystick is disabled, press start button to enable";
    }
}



int main(int argc,char ** argv)
{
	
	ros::init(argc, argv, "listener");
	ros::NodeHandle n;
	//power_on=-1 means joystick is defaultly disabled, mode = -1 means robot is defaultly on joint space
	//static int power_on=-1, mode = -1;
	ros::ServiceClient cmd_client = n.serviceClient<controller::interface>("getcmd");
	//ros::Rate loop_rate(1);
	ros::Rate loop_rate(1);

	ros::Subscriber sub = n.subscribe("robot_joy_topic",1,chatterCallBack);
	//ros::Subscriber sub = n.subscribe("robot_joy_topi`c",1000,chatterCallBack);

	while (ros::ok())
  	{
    	call_service(cmd_client,cmd);
    	ros::spinOnce();
    	//loop_rate.sleep();
  	}

	return 0;
}