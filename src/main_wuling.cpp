#include <iostream>
#include <aris.hpp>
#include "wulingconfig.h"
#include "wuling.h"
#include<atomic>
#include<string>
#include<filesystem>


using namespace aris::dynamic;

//global vel//
kaanh::Speed g_vel = { 0.1, 0.1, 3.4, 0.0, 0.0 };
std::atomic_int g_vel_percent = 0;
//global vel//

//state machine flag//
std::atomic_bool g_is_enabled = false;
std::atomic_bool g_is_error = false;
std::atomic_bool g_is_manual = false;
std::atomic_bool g_is_auto = false;
std::atomic_bool g_is_running = false;
//state machine flag//

auto xmlpath = std::filesystem::absolute(".");//获取当前工程所在的路径
auto uixmlpath = std::filesystem::absolute(".");
auto modelxmlpath = std::filesystem::absolute(".");
const std::string xmlfile = "kaanh.xml";
const std::string uixmlfile = "interface_kaanh.xml";
const std::string modelxmlfile = "model_rokae.xml";


//添加var类型
aris::core::Calculator g_cal;
aris::dynamic::Model g_model;

int main(int argc, char *argv[])
{
    std::cout <<"new"<<std::endl;
    xmlpath = xmlpath / xmlfile;
	uixmlpath = uixmlpath / uixmlfile;
	modelxmlpath = modelxmlpath / modelxmlfile;
    std::cout<< xmlpath <<std::endl;
	auto&cs = aris::server::ControlServer::instance();
	auto port = argc < 2 ? 5866 : std::stoi(argv[1]);

    /*
    //-------for wuling robot begin//
    cs.resetController(wulingconfig::createControllerWuling().release());
    cs.resetModel(wulingconfig::createModelWuling().release());
    cs.resetPlanRoot(wulingconfig::createPlanRoot().release());
	cs.interfacePool().add<aris::server::WebInterface>("", "5866", aris::core::Socket::WEB);
	cs.interfacePool().add<aris::server::WebInterface>("", "5867", aris::core::Socket::TCP);
	cs.resetSensorRoot(new aris::sensor::SensorRoot);
	cs.interfaceRoot().loadXmlFile(uixmlpath.string().c_str());
	//cs.model().saveXmlFile(modelxmlpath.string().c_str());	//for new model
	cs.model().loadXmlFile(modelxmlpath.string().c_str());
	cs.saveXmlFile(xmlpath.string().c_str());
    //-------for wuling robot end//
    */

    cs.loadXmlFile(xmlpath.string().c_str());
	cs.init();

    auto &cal = cs.model().calculator();
    wulingconfig::createUserDataType(cal);

    //cs.start();

	//实时回调函数，每个实时周期调用一次//
	cs.setRtPlanPostCallback(kaanh::update_state);
	g_model = cs.model();


#ifdef WIN32
	for (auto &m : cs.controller().slavePool())
	{
		dynamic_cast<aris::control::EthercatMotor&>(m).setVirtual(true);
	}
#endif // WIN32

	//Start Web Socket//
    cs.open();

	//Receive Command//
	cs.runCmdLine();

	return 0;
}
