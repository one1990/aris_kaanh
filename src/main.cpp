#include <iostream>
#include <aris.h>
#include"rokae.h"
#include<atomic>
#include<string>
#include<direct.h>


std::atomic_bool is_automatic = false;
using namespace aris::dynamic;

double fce_data[4000], fce_send[4000];
int data_num = 0, data_num_send = 0;
int select_id = 0;
std::vector<std::vector<std::string>> plantrack(6, std::vector<std::string>());
std::atomic_int which_di = 0;
char *buffer;
std::string xmlpath = getcwd(buffer, 255);
const std::string xmlfile = "\\plan.xml";


int main(int argc, char *argv[])
{
	xmlpath = xmlpath + xmlfile;
	auto&cs = aris::server::ControlServer::instance();
	auto port = argc < 2 ? 5866 : std::stoi(argv[1]);

	cs.resetController(rokae::createControllerRokaeXB4().release());
	cs.resetModel(rokae::createModelRokaeXB4().release());
	cs.resetPlanRoot(rokae::createPlanRootRokaeXB4().release());
	cs.resetSensorRoot(new aris::sensor::SensorRoot);

	aris::core::WebSocket socket;
	
	//创建一个di信号监控线程//
	std::thread watch_di_thread;

	socket.setOnReceivedMsg([&](aris::core::WebSocket *socket, aris::core::Msg &msg)->int
	{
		std::string msg_data = msg.toString();

		//std::cout << "recv:" << msg_data << std::endl;

		if (msg.header().msg_id_ == M_RUN)
		{
			LOG_INFO << "the request is cmd:"
				<< msg.header().msg_size_ << "&"
				<< msg.header().msg_id_ << "&"
				<< msg.header().msg_type_ << "&"
				<< msg.header().reserved1_ << "&"
				<< msg.header().reserved2_ << "&"
				<< msg.header().reserved3_ << ":"
				<< msg_data << std::endl;

			try
			{
				auto id = cs.executeCmd(aris::core::Msg(msg_data));
				std::cout << "command id:" << id << std::endl;
				socket->sendMsg(aris::core::Msg());
			}
			catch (std::exception &e)
			{
				std::cout << e.what() << std::endl;
				LOG_ERROR << e.what() << std::endl;
				socket->sendMsg(aris::core::Msg(e.what()));
			}
		}
		else if (msg.header().msg_id_ == READ_RT_DATA)
		{
			LOG_INFO_EVERY_N(10) << "socket receive request msg:"
				<< msg.header().msg_size_ << "&"
				<< msg.header().msg_id_ << "&"
				<< msg.header().msg_type_ << "&"
				<< msg.header().reserved1_ << "&"
				<< msg.header().reserved2_ << "&"
				<< msg.header().reserved3_ << ":"
				<< msg_data << std::endl;

			auto part_pm_vec = std::make_any<std::vector<double> >(cs.model().partPool().size() * 16);
			cs.getRtData([](aris::server::ControlServer& cs, std::any& data)
			{
				for (aris::Size i(-1); ++i < cs.model().partPool().size();)
					cs.model().partPool().at(i).getPm(std::any_cast<std::vector<double>&>(data).data() + i * 16);
				// rt copy fce data //
				std::copy_n(fce_data, data_num_send, fce_send);
				// clear data num //
				data_num = 0;
			}, part_pm_vec);

			std::vector<double> part_pq(cs.model().partPool().size() * 7);
			for (aris::Size i(-1); ++i < cs.model().partPool().size();)
			{
				aris::dynamic::s_pm2pq(std::any_cast<std::vector<double>&>(part_pm_vec).data() + i * 16, part_pq.data() + i * 7);
			}
			//// return binary ////
			aris::core::Msg msg;
			msg.copy(part_pq.data(), part_pq.size() * 8);
			msg.copyMore(fce_send, data_num_send * 8);
			socket->sendMsg(aris::core::Msg("read rt data"));
			socket->sendMsg(msg);
		}
		else if (msg.header().msg_id_ == READ_XML)
		{
			LOG_INFO << "the request is cmd:"
				<< msg.header().msg_size_ << "&"
				<< msg.header().msg_id_ << "&"
				<< msg.header().msg_type_ << "&"
				<< msg.header().reserved1_ << "&"
				<< msg.header().reserved2_ << "&"
				<< msg.header().reserved3_ << ":"
				<< msg_data << std::endl;

			tinyxml2::XMLDocument xml_data;
			tinyxml2::XMLPrinter printer;
			try
			{
				xml_data.LoadFile(xmlpath.c_str());
				xml_data.Print(&printer);
				std::cout << "read xml msg id:" << msg.header().msg_id_ << std::endl;
				socket->sendMsg(aris::core::Msg("read xml"));
				socket->sendMsg(aris::core::Msg(printer.CStr()));
			}
			catch (std::exception &e)
			{
				std::cout << e.what() << std::endl;
				LOG_ERROR << e.what() << std::endl;
				socket->sendMsg(aris::core::Msg(e.what()));
			}
		}
		else if (msg.header().msg_id_ == A_RUN)
		{
			std::cout << "switch to automatic mode:" << msg.header().msg_id_ << std::endl;
			LOG_INFO << "switch to automatic mode:"
				<< msg.header().msg_size_ << "&"
				<< msg.header().msg_id_ << "&"
				<< msg.header().msg_type_ << "&"
				<< msg.header().reserved1_ << "&"
				<< msg.header().reserved2_ << "&"
				<< msg.header().reserved3_ << ":"
				<< msg_data << std::endl;

			//读取字符串并将其存储在xmlpath指定的路径下//	
			try
			{
				tinyxml2::XMLDocument xml_data;
				xml_data.Parse(msg_data.c_str());
				xml_data.SaveFile(xmlpath.c_str());
			}  
			catch (std::exception &e)
			{
				std::cout << e.what() << std::endl;
				LOG_ERROR << e.what() << std::endl;
				socket->sendMsg(aris::core::Msg(e.what()));
			}

			//加载指定路径下的xml文件//
			tinyxml2::XMLDocument doc;
			tinyxml2::XMLError errXml = doc.LoadFile(xmlpath.c_str());
			if (errXml != tinyxml2::XML_SUCCESS)
			{
				std::cout << errXml << std::endl;
				LOG_ERROR << errXml << std::endl;
				socket->sendMsg(aris::core::Msg(errXml));
				return false;
			}
			tinyxml2::XMLElement* root = doc.RootElement();
			tinyxml2::XMLElement* CmdListNode = root->FirstChildElement("CmdList");
			while (CmdListNode != NULL)
			{
				if (0 == strcmp("1", (CmdListNode->Attribute("CH"))))
				{
					tinyxml2::XMLElement* cmdNode = CmdListNode->FirstChildElement("cmd");
					for (std::int16_t i = 0; i < atoi(CmdListNode->Attribute("Count")); i++)
					{
						plantrack[0].push_back(cmdNode->GetText());
						cmdNode = cmdNode->NextSiblingElement();
					}
				}
				else if (0 == strcmp("2", (CmdListNode->Attribute("CH"))))
				{
					tinyxml2::XMLElement* cmdNode = CmdListNode->FirstChildElement("cmd");
					for (std::int16_t i = 0; i < atoi(CmdListNode->Attribute("Count")); i++)
					{
						plantrack[1].push_back(cmdNode->GetText());
						cmdNode = cmdNode->NextSiblingElement();
					}
				}
				else if (0 == strcmp("3", (CmdListNode->Attribute("CH"))))
				{
					tinyxml2::XMLElement* cmdNode = CmdListNode->FirstChildElement("cmd");
					for (std::int16_t i = 0; i < atoi(CmdListNode->Attribute("Count")); i++)
					{
						plantrack[2].push_back(cmdNode->GetText());
						cmdNode = cmdNode->NextSiblingElement();
					}
				}
				else if (0 == strcmp("4", (CmdListNode->Attribute("CH"))))
				{
					tinyxml2::XMLElement* cmdNode = CmdListNode->FirstChildElement("cmd");
					for (std::int16_t i = 0; i < atoi(CmdListNode->Attribute("Count")); i++)
					{
						plantrack[3].push_back(cmdNode->GetText());
						cmdNode = cmdNode->NextSiblingElement();
					}
				}
				else if (0 == strcmp("5", (CmdListNode->Attribute("CH"))))
				{
					tinyxml2::XMLElement* cmdNode = CmdListNode->FirstChildElement("cmd");
					for (std::int16_t i = 0; i < atoi(CmdListNode->Attribute("Count")); i++)
					{
						plantrack[4].push_back(cmdNode->GetText());
						cmdNode = cmdNode->NextSiblingElement();
					}
				}
				else if (0 == strcmp("6", (CmdListNode->Attribute("CH"))))
				{
					tinyxml2::XMLElement* cmdNode = CmdListNode->FirstChildElement("cmd");
					for (std::int16_t i = 0; i < atoi(CmdListNode->Attribute("Count")); i++)
					{
						plantrack[5].push_back(cmdNode->GetText());
						cmdNode = cmdNode->NextSiblingElement();
					}
				}
				else
				{
					CmdListNode = CmdListNode->NextSiblingElement();
				}
			}

			//自动模式判断标志位，true为自动，false切出自动//
			is_automatic = true;

			//di信号监控线程实现//
			watch_di_thread = std::thread([&]()->void {
				
				while (is_automatic)
				{
					int which_di_local = which_di.load();
					
					switch (which_di_local)
					{
					case 0:
						break;
					case 1:
					{
						for (std::uint16_t i = 0; i < plantrack[which_di_local].size(); i++)
						{
							auto id = cs.executeCmd(aris::core::Msg(plantrack[which_di_local][i]));
						}		
						which_di.store(0);
						break;
					}
					case 2:
					{
						for (std::uint16_t i = 0; i < plantrack[which_di_local].size(); i++)
						{
							auto id = cs.executeCmd(aris::core::Msg(plantrack[which_di_local][i]));
						}
						which_di.store(0);
						break;
					}
					case 3:
					{
						for (std::uint16_t i = 0; i < plantrack[which_di_local].size(); i++)
						{
							auto id = cs.executeCmd(aris::core::Msg(plantrack[which_di_local][i]));
						}
						which_di.store(0);
						break;
					}
					case 4:
					{
						for (std::uint16_t i = 0; i < plantrack[which_di_local].size(); i++)
						{
							auto id = cs.executeCmd(aris::core::Msg(plantrack[which_di_local][i]));
						}
						which_di.store(0);
						break;
					}
					case 5:
					{
						for (std::uint16_t i = 0; i < plantrack[which_di_local].size(); i++)
						{
							auto id = cs.executeCmd(aris::core::Msg(plantrack[which_di_local][i]));
						}
						which_di.store(0);
						break;
					}
					case 6:
					{
						for (std::uint16_t i = 0; i < plantrack[which_di_local].size(); i++)
						{
							auto id = cs.executeCmd(aris::core::Msg(plantrack[which_di_local][i]));
						}
						which_di.store(0);
						break;
					}
					default:
						;
					}
					
					//实时线程休息1ms//
					std::this_thread::sleep_for(std::chrono::milliseconds(1));
				}
			});
			
			//开启di信号实时监控//
			try
			{
				auto id = cs.executeCmd(aris::core::Msg("listenDI"));
				std::cout << "command id:" << id << std::endl;
			}
			catch (std::exception &e)
			{
				std::cout << e.what() << std::endl;
				LOG_ERROR << e.what() << std::endl;
			}
		}
		else if (msg.header().msg_id_ == A_QUIT)
		{
			std::cout << "quit automatic mode:" << msg.header().msg_id_ << std::endl;
			LOG_INFO << "quit automatic mode:" 
				<< msg.header().msg_id_ << "&"
				<< msg_data << std::endl;
			is_automatic = false;
			
			//回收栈资源//
			watch_di_thread.join();
		}
		else
		{
			std::cout << "undefined msg_id:" << msg.header().msg_id_ << std::endl;
			LOG_INFO << "undefined msg_id:" 
				<< msg.header().msg_size_ << "&"
				<< msg.header().msg_id_ << "&"
				<< msg.header().msg_type_ << "&"
				<< msg.header().reserved1_ << "&"
				<< msg.header().reserved2_ << "&"
				<< msg.header().reserved3_ << ":"
				<< msg_data << std::endl;
			socket->sendMsg(aris::core::Msg());
		}
		return 0;
	});
	socket.setOnReceivedConnection([](aris::core::WebSocket *sock, const char *ip, int port)->int
	{
		std::cout << "socket receive connection" << std::endl;
		LOG_INFO << "socket receive connection:\n"
			<< std::setw(aris::core::LOG_SPACE_WIDTH) << "|" << "  ip:" << ip << "\n"
			<< std::setw(aris::core::LOG_SPACE_WIDTH) << "|" << "port:" << port << std::endl;
		return 0;
	});
	socket.setOnLoseConnection([](aris::core::WebSocket *socket)
	{
		std::cout << "socket lose connection" << std::endl;
		LOG_INFO << "socket lose connection" << std::endl;
		for (;;)
		{
			try
			{
				socket->startServer("5866");
				break;
			}
			catch (aris::core::Socket::StartServerError &e)
			{
				std::cout << e.what() << std::endl << "will try to restart server socket in 1s" << std::endl;
				LOG_ERROR << e.what() << std::endl << "will try to restart server socket in 1s" << std::endl;
				std::this_thread::sleep_for(std::chrono::seconds(1));
			}
		}
		std::cout << "socket restart successful" << std::endl;
		LOG_INFO << "socket restart successful" << std::endl;

		return 0;
	});
	
	cs.start();
	socket.startServer(std::to_string(port));
	//try 
	//{
	//	cs.executeCmd(aris::core::Msg("moveJS"));
	//}
	//catch (std::exception &e)
	//{
	//	std::cout << e.what() << std::endl;
	//	//1LOG_ERROR << e.what() << std::endl;
	//}
	

	// 接收命令 //
	for (std::string command_in; std::getline(std::cin, command_in);)
	{
		try
		{
			if (command_in == "start")
			{
				cs.start();
				socket.startServer(std::to_string(port));
			}
			else if (command_in == "stop")
			{
				cs.stop();
				socket.stop();
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