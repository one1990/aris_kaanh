#include <iostream>
#include <aris.hpp>
#include "config.h"
#include "kaanh.h"
#include <atomic>
#include <string>
#include <filesystem>


using namespace aris::dynamic;

auto xmlpath = std::filesystem::absolute(".");//获取当前工程所在的路径
auto uixmlpath = std::filesystem::absolute(".");
auto modelxmlpath = std::filesystem::absolute(".");
auto logpath = std::filesystem::absolute(".");
const std::string xmlfile = "kaanh.xml";
const std::string uixmlfile = "interface_kaanh.xml";
const std::string modelxmlfile = "model_rokae.xml";
const std::string logfolder = "log";


int main(int argc, char *argv[])
{
    std::cout <<"new"<<std::endl;
    xmlpath = xmlpath / xmlfile;
	uixmlpath = uixmlpath / uixmlfile;
	modelxmlpath = modelxmlpath / modelxmlfile;
	logpath = logpath / logfolder;
    
	auto&cs = aris::server::ControlServer::instance();
	auto port = argc < 2 ? 5866 : std::stoi(argv[1]);
	auto path = argc < 2 ? xmlpath : argv[2];
	auto logp = argc < 2 ? logpath : argv[3];

	std::cout << "port:" << port << std::endl;
    std::cout << "xmlpath:" << xmlpath << std::endl;
    std::cout << "path:" << path << std::endl;
	std::cout << "logfolder:" << logp << std::endl;

	/*
	//生成kaanh.xml文档
	cs.resetController(config::createController().release());	//根据createController()返回值创建controller
	cs.resetModel(config::createModel().release());				//根据createModel()返回值创建controller
	cs.resetPlanRoot(config::createPlanRoot().release());
	cs.interfacePool().add<aris::server::ProgramWebInterface>("", "5866", aris::core::Socket::WEB);
	cs.interfacePool().add<aris::server::WebInterface>("", "5867", aris::core::Socket::TCP);
	cs.resetSensorRoot(new aris::sensor::SensorRoot);
	cs.model().saveXmlFile(modelxmlpath.string().c_str());	//when creat new model
	cs.saveXmlFile(xmlpath.string().c_str());
	*/

    cs.loadXmlFile(path.string().c_str());
	cs.init();

	//修改log路径
	aris::core::logDirectory(logp);

	//kaanh namespace变量初始化函数
    auto &cal = cs.model().calculator();
    kaanhconfig::createUserDataType(cal);
	kaanhconfig::createPauseTimeSpeed();
	g_model = cs.model();

	//开启controller server
	//cs.start();

	//实时回调函数，每个实时周期执行后调用一次， kaanh::update_state函数可以替换成用户的函数
	cs.setRtPlanPostCallback(kaanh::update_state);

	//设置虚拟轴，保证window不做ethercat连接检查
#ifdef WIN32
	for (auto &m : cs.controller().slavePool())
	{
		dynamic_cast<aris::control::EthercatMotor&>(m).setVirtual(true);
	}
#endif

	//start Web Socket
    cs.open();

	//Receive Command from terminal
	cs.runCmdLine();

	return 0;
}
