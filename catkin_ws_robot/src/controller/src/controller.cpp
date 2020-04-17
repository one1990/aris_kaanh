#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <aris.hpp>
#include <atomic>
#include <string>
#include <filesystem>
#include "controller/interface.h"
#include "controller/rob_param.h"
#include "controller/pose.h"
#include "json.hpp"


using namespace aris::dynamic;

auto&cs = aris::server::ControlServer::instance();
// á??ó2￠·￠?ímsg //
aris::core::Socket client("client");

bool exector(controller::interface::Request  &req, controller::interface::Response &res)
{
	ROS_INFO("controller: receive request from client: cmd=%s", req.cmd.c_str());

	try
	{
		client.sendMsg(aris::core::Msg(req.cmd.c_str()));
	}
	catch (std::exception &e)
	{
		std::cout << e.what() << std::endl;
	}
	res.return_code=1;
	ROS_INFO("controller: sending back response to client: [%ld]", (long int)res.return_code);
	return true;
}

int main(int argc,char ** argv)
{
	client.setConnectType(aris::core::Socket::TCP);
    client.setRemoteIP("127.0.0.1");
    client.setPort("5867");	
	auto connectSocket = [&]()->void
	{
		for (;;)
		{
			try
			{
				client.connect();
				break;
			}
			catch (std::exception &e)
			{
				std::cout << "failed to connect server, will retry in 1 second" << std::endl;
				std::this_thread::sleep_for(std::chrono::seconds(1));
			}
		}
	};
	connectSocket();
	client.setOnLoseConnection([&](aris::core::Socket *sock)->int {	connectSocket(); return 0; });
	client.setOnReceivedMsg([&](aris::core::Socket *sock, aris::core::Msg &msg) ->int
	{
		std::cout << msg.toString() << std::endl;// 接收到的返回值	
		return 0;
	});

	ros::init(argc, argv, "controller");
	ros::NodeHandle n;
	ros::ServiceServer service = n.advertiseService("getcmd", exector);
	ros::spin();

	return 0;
}

