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
    geometry_msgs::TransformStamped odom_trans2;
    odom_trans2.header.frame_id = "odom";  
    odom_trans2.child_frame_id = "Link_1";
    geometry_msgs::TransformStamped odom_trans3;
    odom_trans3.header.frame_id = "odom";  
    odom_trans3.child_frame_id = "Link_2";
    geometry_msgs::TransformStamped odom_trans4;
    odom_trans4.header.frame_id = "odom";  
    odom_trans4.child_frame_id = "Link_3";
    geometry_msgs::TransformStamped odom_trans5;
    odom_trans5.header.frame_id = "odom";  
    odom_trans5.child_frame_id = "Link_4";
    geometry_msgs::TransformStamped odom_trans6;
    odom_trans6.header.frame_id = "odom";  
    odom_trans6.child_frame_id = "Link_5";
    geometry_msgs::TransformStamped odom_trans7;
    odom_trans7.header.frame_id = "odom";  
    odom_trans7.child_frame_id = "Link_6";



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
        odom_trans.transform.translation.x = angle;  
        odom_trans.transform.translation.y = 0;  
        odom_trans.transform.translation.z = 0  ;  
        //odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(angle);  
	odom_trans.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0.0, i*degree, 0.0);

        odom_trans2.header.stamp = ros::Time::now();  
        odom_trans2.transform.translation.x = 0;  
        odom_trans2.transform.translation.y = angle;  
        odom_trans2.transform.translation.z = 0  ;  
        //odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(angle);  
	odom_trans2.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0, 0.0);

        odom_trans3.header.stamp = ros::Time::now();  
        odom_trans3.transform.translation.x = 0;  
        odom_trans3.transform.translation.y = 0;  
        odom_trans3.transform.translation.z = angle  ;  
        //odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(angle);  
	odom_trans3.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0, 0.0);

        odom_trans4.header.stamp = ros::Time::now();  
        odom_trans4.transform.translation.x = 0;  
        odom_trans4.transform.translation.y = 0;  
        odom_trans4.transform.translation.z = 0  ;  
        //odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(angle);  
	odom_trans4.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(angle, 0, 0.0);

        odom_trans5.header.stamp = ros::Time::now();  
        odom_trans5.transform.translation.x = 0;  
        odom_trans5.transform.translation.y = 0;  
        odom_trans5.transform.translation.z = 0  ;  
        //odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(angle);  
	odom_trans5.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0, angle);

        odom_trans6.header.stamp = ros::Time::now();  
        odom_trans6.transform.translation.x = 0;  
        odom_trans6.transform.translation.y = 0;  
        odom_trans6.transform.translation.z = 0  ;  
        //odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(angle);  
	odom_trans6.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0.0, angle, 0.0);

        odom_trans7.header.stamp = ros::Time::now();  
        odom_trans7.transform.translation.x = 0;  
        odom_trans7.transform.translation.y = 0;  
        odom_trans7.transform.translation.z = angle  ;  
        //odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(angle);  
	odom_trans7.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0, 0.0);	



        if(i<=70)
            i++;
        else
            i=-69; 

        //send the joint state and transform
        joint_pub.publish(joint_state);

	broadcaster.sendTransform(odom_trans);
	broadcaster.sendTransform(odom_trans2);
	broadcaster.sendTransform(odom_trans3);
	broadcaster.sendTransform(odom_trans4);
	broadcaster.sendTransform(odom_trans5);
	broadcaster.sendTransform(odom_trans6);
	broadcaster.sendTransform(odom_trans7);

	// Create new car angle
        angle += degree/4;

        // This will adjust as needed per iteration
        loop_rate.sleep();
    }

    return 0;
}

