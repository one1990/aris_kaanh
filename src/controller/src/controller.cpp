#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <aris.hpp>
#include "kaanh.h"
#include <atomic>
#include <string>
#include <filesystem>
#include "controller/interface.h"
#include "controller/pose.h"

using namespace aris::dynamic;

auto&cs = aris::server::ControlServer::instance();

bool do_cmd(std::string command_in)
{
	try
	{
		auto target = cs.executeCmd(aris::core::Msg(command_in));
	}
	catch (std::exception &e)
	{
		std::cout << e.what() << std::endl;
	}
	return true;
}

bool exector(controller::interface::Request  &req,
	controller::interface::Response &res)
{
	ROS_INFO("controller: receive request from client: cmd=%s", req.cmd.c_str());
	do_cmd(req.cmd);
	res.return_code=1;
	ROS_INFO("controller: sending back response to client: [%ld]", (long int)res.return_code);
	return true;
}

int main(int argc,char ** argv)
{
	ros::init(argc, argv, "controller");
	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("kannh_topic", 1000);
	ros::Rate loop_rate(10);

	ros::ServiceServer service = n.advertiseService("getcmd", exector);

	//auto xmlpath = std::filesystem::absolute(".");
	//const std::string xmlfile = "kaanh.xml";
	auto xmlpath = std::filesystem::absolute(".");//获取当前工程所在的路径
	auto uixmlpath = std::filesystem::absolute(".");
	const std::string xmlfile = "kaanh.xml";
	const std::string uixmlfile = "interface_kaanh.xml";
	xmlpath = xmlpath / xmlfile;
	uixmlpath = uixmlpath / uixmlfile;

	//cs.resetController(kaanh::createControllerRokaeXB4().release());
	//cs.resetModel(kaanh::createModelRokae().release());
	//cs.resetPlanRoot(kaanh::createPlanRootRokaeXB4().release());
	//cs.resetSensorRoot(new aris::sensor::SensorRoot);
    //cs.saveXmlFile(xmlpath.string().c_str());


    cs.loadXmlFile(xmlpath.string().c_str());
    //cs.interfaceRoot().loadXmlFile(uixmlpath.string().c_str());
    //cs.saveXmlFile(xmlpath.string().c_str());
	cs.start();

	while (ros::ok())
	{
		//controller::pose msg;
		//std_msgs::String msg;
		//std::stringstream ss;
		//ss << "moveJR -m=0 --pos=0.1";
		//ss << "[1,2,3,0.1,0.2,0.3]";
		//msg.data = ss.str();
		//chatter_pub.publish(msg);

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
