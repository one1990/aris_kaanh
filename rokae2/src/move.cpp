#include<iostream>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <robot_state_publisher/robot_state_publisher.h>
#include <tf/transform_broadcaster.h>
using namespace std;

int main(int argc, char** argv)
 {
    ros::init(argc, argv, "move_Node"); //节点的名称
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1); 
    tf::TransformBroadcaster broadcaster;   
    //设置一个发布者，将消息topic(joint_states)发布出去,发布到相应的节点中去
    ros::Rate loop_rate(10);    //这个设置的太大，效果很不好，目前觉得为10最好了
    const double degree = M_PI/180;
    const double radius = 2;
    int i=-69;
    // robot state
    double angle= 0;
    // message declarations
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.frame_id = "odom";  
    odom_trans.child_frame_id = "base_link";

    sensor_msgs::JointState joint_state;
    //  for(int j=0;j<6;++j)
    // {
        //joint_state.velocity.push_back(1);
        //joint_state.effort.push_back(200);
    //}

   

    while (ros::ok()) {
        //update joint_state
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(1);
        joint_state.position.resize(1);
	joint_state.name[0]="Joint_1";
        joint_state.position[0] = i*degree;
	
	

        // update transform  
        // (moving in a circle with radius)  
        odom_trans.header.stamp = ros::Time::now();  
        odom_trans.transform.translation.x = radius * cos(angle);  
        odom_trans.transform.translation.y = radius * sin(angle);  
        odom_trans.transform.translation.z = 0.0;  
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(angle);  

	



        if(i<=70)
            i++;
        else
            i=-69; 

        //send the joint state and transform
        joint_pub.publish(joint_state);

	broadcaster.sendTransform(odom_trans);

	// Create new car angle
        angle += degree/4;

        // This will adjust as needed per iteration
        loop_rate.sleep();
    }

    return 0;
}

