#include <iostream>
#include <aris.hpp>
#include "config.h"
#include "kaanh.h"
#include <atomic>
#include <string>
#include <filesystem>


auto xmlpath = std::filesystem::absolute(".");//获取当前工程所在的路径
auto logpath = std::filesystem::absolute(".");//获取当前工程所在的路径
const std::string xmlfile = "kaanh.xml";//指定配置文件名
const std::string logfolder = "log";//指定log文件夹名


int main(int argc, char *argv[])
{
    xmlpath = xmlpath / xmlfile;	//配置文件kaanh.xml的保存路径
	logpath = logpath / logfolder;	//log文件保存路径
    

	auto&cs = aris::server::ControlServer::instance();//创建控制器服务实例
	
	//生成控制器配置文件kaanh.xml，里面包括从站配置信息、模型、websocket\socket端口等信息
	//生成一次kaanh.xml配置后，用户可以取消如下注释，后面可以自动加载kaanh.xml配置

	cs.resetController(config::createController().release());	//根据createController()返回值创建controller对象池
    cs.resetModel(config::createModel().release());				//根据createModel()返回值创建model对象池

    cs.resetPlanRoot(config::createPlanRoot().release());		//根据createPlanRoot()函数的返回值创建plan对象池
    cs.interfacePool().add<aris::server::ProgramWebInterface>("", "5866", aris::core::Socket::WEB);	//创建websocket服务，端口号5866
	cs.interfacePool().add<aris::server::WebInterface>("", "5867", aris::core::Socket::TCP);		//创建socket服务，端口号5867
	cs.resetSensorRoot(new aris::sensor::SensorRoot);			//aris自身配置，原封不动添加即可
	cs.saveXmlFile(xmlpath.string().c_str());					//保存上述配置信息到xmlpath指定的路径
    //cs.loadXmlFile(xmlpath.string().c_str());//加载已有的kaanh.xml配置
    cs.init();//注册controller、model、plan对象池

    cs.controller().setSamplePeriodNs(2000000);
	//修改log路径
	aris::core::logDirectory(logpath);

	//kaanh namespace变量初始化函数
    auto &cal = cs.model().calculator();
    kaanhconfig::createUserDataType(cal);
	kaanhconfig::createPauseTimeSpeed();
	g_model = cs.model();

	//开启控制器服务，cs.start()也可以注释，通过指令"cs_start"来开启
	cs.start();

	//实时回调函数，每个实时周期执行后调用一次， kaanh::update_state函数可以替换成用户的函数
    cs.setRtPlanPostCallback(kaanh::update_state);



	//设置虚拟轴，保证window不做ethercat连接检查
#ifdef WIN32
	for (auto &m : cs.controller().slavePool())
	{
		dynamic_cast<aris::control::EthercatMotor&>(m).setVirtual(true);
	}
#endif

	//开启websocket，soccket服务
    cs.open();

	//本条指令会进入一个死循环，等待指令输入，所以，用户添加的程序必须放在本条指令前面
	cs.runCmdLine();

	return 0;
}
