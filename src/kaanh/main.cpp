#include <iostream>
#include <aris.hpp>
#include "kaanh.h"
#include<atomic>
#include<string>
#include<filesystem>


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
    //-------for rokae robot begin
    cs.resetController(kaanhconfig::createControllerRokaeXB4().release());
    cs.resetModel(kaanhconfig::createModelRokae().release());
    cs.resetPlanRoot(kaanhconfig::createPlanRoot().release());
    cs.interfacePool().add<aris::server::ProgramWebInterface>("", "5866", aris::core::Socket::WEB);
	//cs.interfacePool().add<kaanh::ProInterface>("", "5866", aris::core::Socket::WEB);
	cs.interfacePool().add<aris::server::WebInterface>("", "5867", aris::core::Socket::TCP);
    cs.resetSensorRoot(new aris::sensor::SensorRoot);
	//cs.interfaceRoot().loadXmlFile(uixmlpath.string().c_str());
	//cs.model().saveXmlFile(modelxmlpath.string().c_str());	//when creat new model
    cs.model().loadXmlFile(modelxmlpath.string().c_str());
    cs.saveXmlFile(xmlpath.string().c_str());
    //-------for rokae robot end// 
    */
    
    /*
	auto ret_load = cal.calculateExpression("pose({1,2,3,4,5,6,7})");
	std::cout << ret_load.first << std::endl;
	auto mat = std::any_cast<aris::core::Matrix>(ret_load.second);
	std::cout << mat.data()[0] << std::endl;

	cal.addVariable("p10", "pose", aris::core::Matrix({ 1,2,3,4,5,6,7 }));
	auto ret_ff = cal.calculateExpression("pose(p10)");
	std::cout << ret_ff.first << std::endl;
	auto ret_ff2 = cal.calculateExpression("pose({6,5,4,3,2,1,0})");
	std::cout << ret_ff2.first << std::endl;
    */

    /*
	//构造一个类型来接收UI变量//
	auto &cal = cs.model().calculator();
	cal.addTypename("load");
	cal.addFunction("load", std::vector<std::string>{"Matrix"}, "load", [](std::vector<std::any>&params)->std::any 
	{
		if (std::any_cast<aris::core::Matrix>(params[0]).size() != 11)
		{
			THROW_FILE_LINE("input data error");
		}
		Load a;
		a.mass = std::any_cast<aris::core::Matrix>(params[0]).data()[0];
		auto temp = std::any_cast<aris::core::Matrix&>(params[0]).data();
		std::copy(temp + 1, temp + 11, a.pq);

		return a;
	});
	auto ret_load = cal.calculateExpression("load({1,2,3,4,5,6,7,8,9,0,1})");
	std::cout << ret_load.first << std::endl;
	auto mat = std::any_cast<Load>(ret_load.second);
	cal.calculateExpression("load({2,3,4,5,6,7,8,9,0,1})");
	*/

	/*
	g_cal.addVariable("tool_pq", "Matrix", aris::core::Matrix(1.0));
	g_cal.addVariable("test", "String", std::string("1121"));
	//({ tool_pq,0.3 }*0.5 + 0.1) + 0.1;
	auto ret_mat = std::any_cast<aris::core::Matrix>(g_cal.calculateExpression("tool_pq").second);
	auto is_true = std::any_cast<std::string>(g_cal.calculateExpression("test").second);
	std::cout << ret_mat.toString() << std::endl;
	std::cout << is_true << std::endl;
	*/

    /*
    //-------for sanxiang robot begin//
    cs.resetController(kaanhconfig::createControllerSanXiang().release());
    cs.resetModel(kaanhconfig::createModelSanXiang().release());
    cs.resetPlanRoot(kaanhconfig::createPlanRoot().release());
	cs.interfacePool().add<aris::server::WebInterface>("", "5866", aris::core::Socket::WEB);
	cs.interfacePool().add<aris::server::WebInterface>("", "5867", aris::core::Socket::TCP);
	cs.resetSensorRoot(new aris::sensor::SensorRoot);
	cs.interfaceRoot().loadXmlFile(uixmlpath.string().c_str());
	//cs.model().saveXmlFile(modelxmlpath.string().c_str());	//for new model
	cs.model().loadXmlFile(modelxmlpath.string().c_str());
	cs.saveXmlFile(xmlpath.string().c_str());
    //-------for sanxiang robot end//
    */

    /*
    //-------for daye robot begin//
    cs.resetController(kaanhconfig::createControllerDaye().release());
    cs.resetModel(kaanhconfig::createModelDaye().release());
    cs.resetPlanRoot(kaanhconfig::createPlanRoot().release());
	cs.interfacePool().add<aris::server::WebInterface>("", "5866", aris::core::Socket::WEB);
	cs.interfacePool().add<aris::server::WebInterface>("", "5867", aris::core::Socket::TCP);
	cs.resetSensorRoot(new aris::sensor::SensorRoot);
	cs.interfaceRoot().loadXmlFile(uixmlpath.string().c_str());
	//cs.model().saveXmlFile(modelxmlpath.string().c_str());	//for new model
	cs.model().loadXmlFile(modelxmlpath.string().c_str());
	cs.saveXmlFile(xmlpath.string().c_str());
    //-------for daye robot end//
    */

    /*
	//-------for qifan robot begin//
	cs.resetController(kaanhconfig::createControllerQifan().release());
	cs.resetModel(kaanhconfig::createModelQifan().release());
	cs.resetPlanRoot(kaanhconfig::createPlanRoot().release());
	cs.interfacePool().add<aris::server::WebInterface>("", "5866", aris::core::Socket::WEB);
	cs.interfacePool().add<aris::server::WebInterface>("", "5867", aris::core::Socket::TCP);
	cs.resetSensorRoot(new aris::sensor::SensorRoot);
	cs.interfaceRoot().loadXmlFile(uixmlpath.string().c_str());
	//cs.model().saveXmlFile(modelxmlpath.string().c_str());	//for new model
	cs.model().loadXmlFile(modelxmlpath.string().c_str());
	cs.saveXmlFile(xmlpath.string().c_str());
	//-------for qifan robot end// 
    */

    cs.loadXmlFile(path.string().c_str());
	cs.init();

	aris::core::logDirectory(logp);

    auto &cal = cs.model().calculator();
    kaanhconfig::createUserDataType(cal);
	kaanhconfig::createPauseTimeSpeed();
    //cs.start();

	//实时回调函数，每个实时周期调用一次//
	cs.setRtPlanPostCallback(kaanh::update_state);
	g_model = cs.model();

#ifdef WIN32
	for (auto &m : cs.controller().motionPool())
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
