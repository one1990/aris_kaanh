#include<aris.h>
#include "rokae.h"

int main(int argc, char *argv[])
{
	auto&cs = aris::server::ControlServer::instance();

	cs.resetController(rokae::createControllerRokaeXB4().release());
    cs.resetModel(aris::dynamic::createModelRokaeXB4().release());
	cs.resetPlanRoot(rokae::createPlanRootRokaeXB4().release());

	std::cout << "start" << std::endl;

	cs.start();
	//µÈ´ý²éÑ¯
	for (std::string command_in; std::getline(std::cin, command_in);)
	{
		try
		{
			auto id = cs.executeCmd(aris::core::Msg(command_in));
			std::cout << "command id:" << id << std::endl;
		}
		catch (std::exception &e)
		{
			std::cout << e.what() << std::endl;
		}
	}

}


