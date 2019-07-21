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

  //void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  ros::NodeHandle nh_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;

  double l_scale_, a_scale_;

};

//:参数的初始化列表
//RobotJoy::RobotJoy(): linear_(1), angular_(2)
RobotJoy::RobotJoy()
{
  //  initialize some parameters

  //nh_.param("axis_linear", linear_, linear_);  
  //nh_.param("axis_angular", angular_, angular_);
  //nh_.param("scale_angular", a_scale_, a_scale_);
  //nh_.param("scale_linear", l_scale_, l_scale_);

 // create a publisher that will advertise on the command_velocity topic of the turtle
  vel_pub_ = nh_.advertise<controller::rob_param>("robot_joy_topic", 1);

  // subscribe to the joystick topic for the input to drive the turtle
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 1, &RobotJoy::joyCallback, this);
}


int x_j1=2, y_j2 = 3, z_j3 =5 ,rx_j4 = 0,ry_j5 = 1,rz_j6 = 5,forward_back=1,start=7,gear=7; 
int select_mode=6;
int rs_button = 4, md_ds_button = 6, rc_en_button =4, start_stop =3;

//void RobotJoy::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
void RobotJoy::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{

  controller::rob_param msg;
    //geometry_msgs::Twist twist;
    //geometry_msgs::Twist twist2;

  // take the data from the joystick and manipulate it by scaling it and using independent axes to control the linear and angular velocities of the turtle
  //twist.angular.z = a_scale_*joy->axes[angular_];
  //twist.linear.x = l_scale_*joy->axes[linear_];
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
    msg.j6 = joy->axes[rz_j6];
    msg.select_mode = joy->buttons[select_mode];
    msg.start = joy->buttons[start];
    msg.forward_back = joy->axes[forward_back];
    msg.gear = joy->axes[gear];
    msg.rs_button = joy->buttons[rs_button];
    msg.md_ds_button = joy->axes[md_ds_button];
    msg.rc_en_button = joy->axes[rc_en_button];
    msg.start_stop = joy->axes[start_stop];
    std::cout<<"RobotJoy: joy->buttons[x_j1] "<<msg.x<<std::endl;

    vel_pub_.publish(msg);
    

}


int main(int argc, char** argv)
{
  //sensor_msgs::Joy::ConstPtr& joy;//???????
  sensor_msgs::Joy::ConstPtr joy;
  //int linear_, angular_;   // used to define which axes of the joystick will control our turtle
  
  // initialize our ROS node, create a robot_joy, and spin our node until Ctrl-C is pressed
  ros::init(argc, argv, "RobotJoy");
  
  RobotJoy robot_joy;
  
 
  //joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, RobotJoy::joyCallback);

    ros::Rate loop_rate(20);

    
//twist2.linear.x = a_scale_*joy->axes[forward_back];




    ros::spin();//attention!!!!ros::spin();
    loop_rate.sleep();
}

