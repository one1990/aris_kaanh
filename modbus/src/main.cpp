#include <iostream>
#include <aris.hpp>
#include "kaanh.h"
#include "kaanh/kaanhconfig.h"
#include<atomic>
#include<string>
#include<filesystem>
#include "controlboard.h"


using namespace aris::dynamic;
auto xmlpath = std::filesystem::absolute(".");	//获取当前工程所在的路径
auto logpath = std::filesystem::absolute(".");	//获取当前工程所在的路径
const std::string xmlfile = "kaanh.xml";		//控制配置文件名称
const std::string logfolder = "log";			//log文件夹名称
std::thread t_modbus;


int main(int argc, char *argv[])
{
	
    xmlpath = xmlpath / xmlfile;				//拼接控制器配置文件路径
	logpath = logpath / logfolder;				//拼接log文件夹路径

	auto&cs = aris::server::ControlServer::instance();
	
    cs.loadXmlFile(xmlpath.string().c_str());	//加载kaanh.xml配置
	cs.init();									//初始化
	aris::core::logDirectory(logpath);			//设置log路径

    auto &cal = cs.model().calculator();		//UI变量求解器
    kaanhconfig::createUserDataType(cal);		//预定义UI界面变量集
	kaanhconfig::createPauseTimeSpeed();		//初始化UI停止暂停功能参数
    cs.start();

	//实时回调函数，每个实时周期调用一次//
	cs.setRtPlanPostCallback(kaanh::update_state);
	g_model = cs.model();

	//开启windows下虚拟轴功能
#ifdef WIN32
	for (auto &m : cs.controller().slavePool())
	{
		dynamic_cast<aris::control::EthercatMotor&>(m).setVirtual(true);
	}
#endif 

	//开启WebSocket/socket服务器//
    cs.open();

    //示教器线程;
    t_modbus = std::thread([&]()->bool
    {
       // 创建modbus master
       modbus mb = modbus("192.168.0.21", 502);	//从站的ip地址和端口号
       mb.modbus_set_slave_id(1);					// set slave id
       controlboard cbd;

       // 连接modbus slave，并且响应modbus slave的信息
   start:
       for (;;)
       {
           try
           {
               mb.modbus_connect();				// connect with the server
               break;
           }
           catch (std::exception &e)
           {
               std::cout << "failed to connect server, will retry in 1 second" << std::endl;
               std::this_thread::sleep_for(std::chrono::seconds(1));
           }
       }

       // 主要功能逻辑区
       try
       {
           cs.executeCmd("setvel --vel_percent=1;");

   #if print_time
           struct timeb t1;
           double time_new, time_last, time_max = 0;
           ftime(&t1);
           time_new = time_last = t1.time + 0.001* t1.millitm;
   #endif

           uint16_t read_input_regs[2];
           while (1)
           {
   #if print_time
               ftime(&t1);
               time_last = time_new;
               time_new = t1.time + 0.001* t1.millitm;
               if (time_new - time_last > time_max)
                   time_max = time_new - time_last;
               printf("[%.3f %.3f %.3f]\n", time_new, time_new - time_last, time_max);
   #endif

               /*************以上打印时间相关**************************/
               //read_input_regs[0]存地址为30001的寄存器,read_input_regs[1]存地址为30002的寄存器，读输入寄存器 功能码04
               mb.modbus_read_input_registers(0, 2, read_input_regs);
               if (read_input_regs[0] != 0 || read_input_regs[1] != 0)			//有按键按下
                   cbd.send_command(read_input_regs, cs);					//发送指令
               std::this_thread::sleep_for(std::chrono::milliseconds(100));	//100ms
           }

       }
       catch (std::exception &e)
       {
           std::cout << e.what() << std::endl;
       }

       mb.modbus_close();	// close connection

       goto start;	// 断连接后，尝试重新连接

       delete(&mb); // 释放空间

       return 0 ;
    });

	//分离示教器线程
    t_modbus.detach();

	//等待终端输入函数，本函数不能去掉，否则实时线程和主线程都会结束//
	cs.runCmdLine();

	return 0;
}
