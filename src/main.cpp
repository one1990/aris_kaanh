#include <iostream>
#include <aris.h>
#include"rokae.h"

using namespace aris::dynamic;

int main(int argc, char *argv[])
{
	auto&cs = aris::server::ControlServer::instance();

	cs.resetController(rokae::createControllerRokaeXB4().release());
	cs.resetModel(rokae::createModelRokaeXB4().release());
	cs.resetPlanRoot(rokae::createPlanRootRokaeXB4().release());
	cs.resetSensorRoot(new aris::sensor::SensorRoot);

	cs.start();

	//try 
	//{
	//	cs.executeCmd(aris::core::Msg("moveJS"));
	//}
	//catch (std::exception &e)
	//{
	//	std::cout << e.what() << std::endl;
	//	//1LOG_ERROR << e.what() << std::endl;
	//}
	
	// ½ÓÊÕÃüÁî //
	for (std::string command_in; std::getline(std::cin, command_in);)
	{
		try
		{
			if (command_in == "start")
			{
				cs.start();
			}
			else if (command_in == "stop")
			{
				cs.stop();
			}
			else
			{
				auto id = cs.executeCmd(aris::core::Msg(command_in));
				std::cout << "command id:" << id << std::endl;
			}
		}
		catch (std::exception &e)
		{
			std::cout << e.what() << std::endl;
			LOG_ERROR << e.what() << std::endl;
		}
	}

	return 0;
}