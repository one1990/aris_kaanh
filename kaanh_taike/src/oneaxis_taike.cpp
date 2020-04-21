#include<iostream>
#include<aris.hpp>
#include"oneaxis_taike.h"

using namespace std;
//调用aris库中的plan模块
using namespace aris::plan;


namespace taike
{
	//创建ethercat主站控制器controller，并根据xml文件添加从站信息
	auto createController()->std::unique_ptr<aris::control::Controller>
	{
		//pos_factor=\"3358585\";
		std::unique_ptr<aris::control::Controller> controller(new aris::control::EthercatController);
		std::string xml_str =
			"<EthercatMotor phy_id=\"0\" product_code=\"0x1030\""
			" vendor_id=\"0xAB\" revision_num=\"0x10003\" dc_assign_activate=\"0x0330\""
			" min_pos=\"-536870\" max_pos=\"536870\" max_vel=\"6.28\" min_vel=\"-6.28\""
			" max_acc=\"31.4\" min_acc=\"-31.4\" max_pos_following_error=\"0.5\" max_vel_following_error=\"0.1\""
			" home_pos=\"0\" pos_factor=\"20860.76\" pos_offset=\"0.0\">"
			"	<SyncManagerPoolObject>"
			"		<SyncManager is_tx=\"false\"/>"
			"		<SyncManager is_tx=\"true\"/>"
			"		<SyncManager is_tx=\"false\">"
			"			<Pdo index=\"0x1601\" is_tx=\"false\">"
			"				<PdoEntry name=\"target_vel\" index=\"0x60ff\" subindex=\"0x00\" size=\"32\"/>"
			"			</Pdo>"
			"			<Pdo index=\"0x1700\" is_tx=\"false\">"
			"				<PdoEntry name=\"control_word\" index=\"0x6040\" subindex=\"0x00\" size=\"16\"/>"
			"				<PdoEntry name=\"target_pos\" index=\"0x607A\" subindex=\"0x00\" size=\"32\"/>"
			"				<PdoEntry name=\"offset_vel\" index=\"0x60B1\" subindex=\"0x00\" size=\"32\"/>"
			"				<PdoEntry name=\"tor_offset\" index=\"0x60B2\" subindex=\"0x00\" size=\"16\"/>"
			"			</Pdo>"
			"		</SyncManager>"
			"		<SyncManager is_tx=\"true\">"
			"			<Pdo index=\"0x1b00\" is_tx=\"true\">"
			"				<PdoEntry name=\"status_word\" index=\"0x6041\" subindex=\"0x00\" size=\"16\"/>"
			"				<PdoEntry name=\"pos_actual_value\" index=\"0x6064\" subindex=\"0x00\" size=\"32\"/>"
			"				<PdoEntry name=\"vel_actual_value\" index=\"0x606c\" subindex=\"0x00\" size=\"32\"/>"
			"				<PdoEntry name=\"cur_actual_value\" index=\"0x6077\" subindex=\"0x00\" size=\"16\"/>"
			"				<PdoEntry name=\"following_error\" index=\"0x60F4\" subindex=\"0x00\" size=\"32\"/>"
			"			</Pdo>"
			"		</SyncManager>"
			"	</SyncManagerPoolObject>"
			"</EthercatMotor>";
		controller->slavePool().add<aris::control::EthercatMotor>().loadXmlStr(xml_str);
		return controller;
	};


	// 单关节正弦往复轨迹 //
	struct MoveJSParam
	{
		double j1;
		double time;
		uint32_t timenum;
	};
	auto MoveJS::prepareNrt()->void
	{
		MoveJSParam param;

		param.j1 = 0.0;
		param.time = 0.0;
		param.timenum = 0;

		for (auto &p : cmdParams())
		{
			if (p.first == "j1")
			{
				if (p.second == "current_pos")
				{
					param.j1 = controller()->motionPool()[0].actualPos();
				}
				else
				{
					param.j1 = doubleParam(p.first);
				}

			}
			else if (p.first == "time")
			{
				param.time = doubleParam(p.first);
			}
			else if (p.first == "timenum")
			{
				param.timenum = int32Param(p.first);
			}
		}
		this->param() = param;
		std::vector<std::pair<std::string, std::any>> ret_value;
		ret() = ret_value;
	}
	auto MoveJS::executeRT()->int
	{
		auto &param = std::any_cast<MoveJSParam&>(this->param());
		auto time = static_cast<int32_t>(param.time * 1000);
		auto totaltime = static_cast<int32_t>(param.timenum * time);
		static double begin_pjs;
		static double step_pjs;
		// 访问主站 //
		auto &cout = controller()->mout();

		if ((1 <= count()) && (count() <= time / 2))
		{
			// 获取当前起始点位置 //
			if (count() == 1)
			{
				begin_pjs = controller()->motionPool()[0].actualPos();
				step_pjs = controller()->motionPool()[0].actualPos();
			}
			step_pjs = begin_pjs + param.j1 * (1 - std::cos(2 * PI*count() / time)) / 2;
			controller()->motionPool().at(0).setTargetPos(step_pjs);
		}
		else if ((time / 2 < count()) && (count() <= totaltime - time / 2))
		{
			// 获取当前起始点位置 //
			if (count() == time / 2 + 1)
			{
				begin_pjs = controller()->motionPool()[0].actualPos();
				step_pjs = controller()->motionPool()[0].actualPos();
			}

			step_pjs = begin_pjs - 2 * param.j1 * (1 - std::cos(2 * PI*(count() - time / 2) / time)) / 2;
			controller()->motionPool().at(0).setTargetPos(step_pjs);
		}
		else if ((totaltime - time / 2 < count()) && (count() <= totaltime))
		{
			// 获取当前起始点位置 //
			if (count() == totaltime - time / 2 + 1)
			{
				begin_pjs = controller()->motionPool()[0].actualPos();
				step_pjs = controller()->motionPool()[0].actualPos();
			}
			step_pjs = begin_pjs - param.j1 * (1 - std::cos(2 * PI*(count() - totaltime + time / 2) / time)) / 2;
			controller()->motionPool().at(0).setTargetPos(step_pjs);
		}

		// 打印 //
		if (count() % 100 == 0)
		{
			cout << "pos" << ":" << controller()->motionAtAbs(0).actualPos() << "  ";
			cout << std::endl;
		}

		// log //
		auto &lout = controller()->lout();
		lout << controller()->motionAtAbs(0).targetPos() << ",";
		lout << std::endl;

		return totaltime - count();
	}
	auto MoveJS::collectNrt()->void {}
	MoveJS::MoveJS(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"moveJS\">"
			"	<GroupParam>"
			"		<Param name=\"j1\" default=\"current_pos\"/>"
			"		<Param name=\"time\" default=\"1.0\" abbreviation=\"t\"/>"
			"		<Param name=\"timenum\" default=\"2\" abbreviation=\"n\"/>"
			"	</GroupParam>"
			"</Command>");
	}


	// moveEAP //
	struct MoveEAPParam
	{
		double axis_begin_pos;
		double axis_pos;
		double axis_vel;
		double axis_acc;
		double axis_dec;
		bool abs;
	};
	auto MoveEAP::prepareNrt()->void
	{
		auto c = controller();
		MoveEAPParam param;
		param.axis_begin_pos = 0.0;

		for (auto &p : cmdParams())
		{
			if (p.first == "pos")
			{
				param.axis_pos = doubleParam(p.first);
			}
			else if (p.first == "vel")
			{
				param.axis_vel = doubleParam(p.first);
			}
			else if (p.first == "acc")
			{
				param.axis_acc = doubleParam(p.first);
			}
			else if (p.first == "dec")
			{
				param.axis_dec = doubleParam(p.first);
			}
			else if (p.first == "ab")
			{
				param.abs = int32Param(p.first);
			}
		}

		this->param() = param;
		for (auto &option : motorOptions())	option |= CHECK_NONE;
		std::vector<std::pair<std::string, std::any>> ret_value;
		ret() = ret_value;
	}
	auto MoveEAP::executeRT()->int
	{
		auto &param = std::any_cast<MoveEAPParam&>(this->param());

		// 第一个count，取各个电机的当前位置
		if (count() == 1)
		{
			param.axis_begin_pos = controller()->motionAtAbs(0).actualPos();
		}
		aris::Size total_count{ 1 };
		double p, v, a;
		aris::Size t_count;
		//绝对轨迹//
		if (param.abs)
		{
			//梯形轨迹规划函数moveAbsolute对输入的量(count(), param.axis_begin_pos_vec[i], param.axis_pos_vec[i], param.axis_vel_vec[i] / 1000, param.axis_acc_vec[i] / 1000 / 1000, param.axis_dec_vec[i] / 1000 / 1000)都会取绝对值
			//然后规划出当前count的指令位置p，指令速度v，指令加速度a，以及本梯形轨迹的总count数t_count 
			aris::plan::moveAbsolute(count(), param.axis_begin_pos, param.axis_pos, param.axis_vel / 1000, param.axis_acc / 1000 / 1000, param.axis_dec / 1000 / 1000, p, v, a, t_count);
			controller()->motionAtAbs(0).setTargetPos(p);
			total_count = std::max(total_count, t_count);
		}
		//相对轨迹//
		else
		{
			aris::plan::moveAbsolute(count(), param.axis_begin_pos, param.axis_begin_pos + param.axis_pos, param.axis_vel / 1000, param.axis_acc / 1000 / 1000, param.axis_dec / 1000 / 1000, p, v, a, t_count);
			controller()->motionAtAbs(0).setTargetPos(p);
			total_count = std::max(total_count, t_count);
		}

		// 每1000ms打印 目标位置、实际位置、实际速度、实际电流 //
		auto &cout = controller()->mout();
		if (count() % 1000 == 0)
		{
			cout << controller()->motionAtAbs(0).targetPos() << std::endl;
		}
		// log 目标位置、实际位置、实际速度、实际电流 //

		auto &lout = controller()->lout();
		lout << controller()->motionAtAbs(0).targetPos();
		lout << std::endl;
		// 返回total_count - count()给aris实时核，值为-1，报错；值为0，结束；值大于0，继续执行下一个count
		return total_count - count();
	}
	auto MoveEAP::collectNrt()->void {}
	MoveEAP::MoveEAP(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"moveEAP\">"
			"	<GroupParam>"
			"		<Param name=\"begin_pos\" default=\"0.1\" abbreviation=\"b\"/>"
			"		<Param name=\"pos\" default=\"0.1\"/>"
			"		<Param name=\"vel\" default=\"1\"/>"
			"		<Param name=\"acc\" default=\"5\"/>"
			"		<Param name=\"dec\" default=\"-5\"/>"
			"		<Param name=\"ab\" default=\"0\"/>"
			"	</GroupParam>"
			"</Command>");
	}


	struct SetBrakeParam
	{
		int is_on;
	};
	auto SetBrake::prepareNrt()->void
	{
		SetBrakeParam param;

		for (auto &p : cmdParams())
		{
			if (p.first == "ct")
			{
				param.is_on = int32Param(p.first);
			}
		}

		if (param.is_on)
		{
			for (aris::Size i = 0; i < controller()->slavePool().size(); i++)
			{
				uint16_t input = 0x02;
				uint16_t brake = 0x0000;
				uint16_t control_word1 = 0x0006;
				if (i == 0)
				{
					std::cout << "1" << std::endl;
					ecController()->motionPool().at(i).writeSdo(0x2193, 0x02, &input, 2);
					std::cout << "2" << std::endl;
					ecController()->motionPool().at(i).writePdo(0x6040, 0x00, &control_word1, 16);
					std::cout << "3" << std::endl;
					ecController()->motionPool().at(i).writeSdo(0x2194, 0x00, &brake, 2);
				}
			}
		}
		else
		{
			for (aris::Size i = 0; i < controller()->slavePool().size(); i++)
			{
				uint32_t input = 0x40000100;
				uint16_t brake = 0x0002;
				if (i == 0)
				{
					std::cout << "1" << std::endl;
					ecController()->motionPool().at(i).writeSdo(0x2194, 0x00, &brake, 2);
					std::cout << "2" << std::endl;
					ecController()->motionPool().at(i).writeSdo(0x2193, 0x02, &input, 4);
					std::cout << "3" << std::endl;
				}
			}
		}

		for (auto &option : motorOptions())	option |= CHECK_NONE;
		std::vector<std::pair<std::string, std::any>> ret_value;
		ret() = ret_value;

	}
	auto SetBrake::collectNrt()->void {}
	SetBrake::SetBrake(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"setbrake\">"
			"	<GroupParam>"
			"		<Param name=\"on\" default=\"1\"/>"
			"	</GroupParam>"
			"</Command>");
	}


	struct ClearParam
	{
		uint16_t control_word;
	};
	auto Clear::prepareNrt()->void
	{
		ClearParam param;
		for (auto &p : cmdParams())
		{
			if (p.first == "value")
			{
				param.control_word = int32Param(p.first);
			}
		}

		ecController()->motionPool().at(0).writePdo(0x6040, 0x00, &param.control_word, 16);
		std::cout << "success" << std::endl;
		for (auto &option : motorOptions())	option |= CHECK_NONE;
		std::vector<std::pair<std::string, std::any>> ret_value;
		ret() = ret_value;
	}
	auto Clear::collectNrt()->void {}
	Clear::Clear(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"clear\">"
			"	<GroupParam>"
			"		<Param name=\"value\" default=\"8\"/>"
			"	</GroupParam>"
			"</Command>");
	}


	auto Status::prepareNrt()->void
	{
		uint8_t modeofoperation = 0;
		ecController()->motionPool().at(0).readSdo(0x6061, 0x00, &modeofoperation, 1);
		std::cout << "modeofoperation:" << uint16_t(modeofoperation) << std::endl;

		int16_t input;
		ecController()->motionPool().at(0).readSdo(0x2200, 0x00, &input, 2);
		controller()->mout() << "force sensor:" << "\t" << input << std::endl;

		for (auto &option : motorOptions())	option |= CHECK_NONE;
		std::vector<std::pair<std::string, std::any>> ret_value;
		ret() = ret_value;
	}
	auto Status::executeRT()->int
	{
		uint16_t status_word = 0;
		ecController()->motionPool().at(0).readPdo(0x6041, 0x00, &status_word, 16);
		controller()->mout() << "status_word:" << status_word << std::endl;

		return 1 - count();
	}
	auto Status::collectNrt()->void {}
	Status::Status(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"status\">"
			"</Command>");
	}


	struct MotorModeParam
	{
		uint8_t mode;
	};
	auto MotorMode::prepareNrt()->void
	{
		MotorModeParam param;
		param.mode = 1;

		for (auto &p : cmdParams())
		{
			if (p.first == "mode")
			{
				param.mode = int32Param(p.first);
			}
		}

		uint16_t status_word = 0;
		for (aris::Size i = 0; i < controller()->slavePool().size(); i++)
		{
			uint8_t mode = 1;
			if (i == 0)
			{
				std::cout << "1" << std::endl;
				ecController()->motionPool().at(i).writeSdo(0x6060, 0x00, &param.mode, 1);
				std::cout << "2" << std::endl;
				ecController()->motionPool().at(i).readPdo(0x6041, 0x00, &status_word, 16);
				std::cout << status_word << std::endl;
			}
		}

		for (auto &option : motorOptions())	option |= CHECK_NONE;
		std::vector<std::pair<std::string, std::any>> ret_value;
		ret() = ret_value;
	}
	auto MotorMode::collectNrt()->void {}
	MotorMode::MotorMode(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"motormode\">"
			"	<GroupParam>"
			"		<Param name=\"mode\" default=\"8\"/>"
			"	</GroupParam>"
			"</Command>");
	}


	auto EnableMotor::prepareNrt()->void
	{
		for (auto &option : motorOptions())	option |= CHECK_NONE;
		std::vector<std::pair<std::string, std::any>> ret_value;
		ret() = ret_value;
	}
	auto EnableMotor::executeRT()->int
	{
		uint16_t step2 = 0x0007;
		uint16_t step3 = 0x000f;
		uint16_t step4 = 0x007f;
		uint16_t status_word;
		ecController()->motionPool().at(0).readPdo(0x6041, 0x00, &status_word, 16);
		static bool ret = true;
		static std::int64_t ref_count = 1;
		static int16_t temp = 1;

		if ((status_word & 0x02) == 0x00)
		{
			auto cur_pos = ecController()->motionPool().at(0).actualPos();
			ecController()->motionPool().at(0).setTargetPos(cur_pos);
			//ecController()->motionPool().at(0).setModeOfOperation(8);
			//ecController()->motionPool().at(0).writePdo(0x607a, 0x00, &cur_pos, 32);
			ecController()->motionPool().at(0).writePdo(0x6040, 0x00, &step2, 16);
			controller()->mout() << "1" << std::endl;
		}
		else if ((status_word & 0x02) == 0x02)
		{
			ecController()->motionPool().at(0).writePdo(0x6040, 0x00, &step3, 16);
			controller()->mout() << "2" << std::endl;
			if (temp == 1)
			{
				ref_count = count() + 1000;
				temp = 0;
			}
			ret = false;
		}
		else if ((status_word & 0x04) == 0x04)
		{
			if (temp == 1)
			{
				ref_count = count() + 1000;
				temp = 0;
			}
			ret = false;
		}

		return ret ? 1 : (ref_count - count());

	}
	auto EnableMotor::collectNrt()->void {}
	EnableMotor::EnableMotor(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"motorenable\">"
			"</Command>");
	}


	// 将创建的轨迹添加到轨迹规划池planPool中 //
	auto createPlanRoot()->std::unique_ptr<aris::plan::PlanRoot>
	{
		std::unique_ptr<aris::plan::PlanRoot> plan_root(new aris::plan::PlanRoot);

		plan_root->planPool().add<aris::plan::Enable>();
		plan_root->planPool().add<aris::plan::Disable>();
		plan_root->planPool().add<aris::plan::Mode>();
		plan_root->planPool().add<aris::plan::Show>();
		plan_root->planPool().add<aris::plan::Recover>();
		auto &rs = plan_root->planPool().add<aris::plan::Reset>();
		rs.command().findParam("pos")->setDefaultValue("{0.01}");
		plan_root->planPool().add<MoveJS>();
		plan_root->planPool().add<MoveEAP>();
		plan_root->planPool().add<SetBrake>();
		plan_root->planPool().add<MotorMode>();
		plan_root->planPool().add<EnableMotor>();
		plan_root->planPool().add<Status>();
		plan_root->planPool().add<Clear>();
		return plan_root;
	}

}

// 主函数
int main(int argc, char *argv[])
{
	//创建Ethercat主站对象
    //aris::control::EthercatMaster mst;
	//自动扫描，连接从站
    //mst.scan();
    //std::cout<<mst.xmlString()<<std::endl;


	//cs代表成员函数的引用，aris是头文件，server是命名空间，ControlServer是结构体
    auto&cs = aris::server::ControlServer::instance();
    cs.resetController(taike::createController().release());
    cs.resetPlanRoot(taike::createPlanRoot().release());
    cs.interfacePool().add<aris::server::ProgramWebInterface>("", "5866", aris::core::Socket::WEB);//创建一个websocket服务，端口5866
    cs.interfacePool().add<aris::server::WebInterface>("", "5867", aris::core::Socket::TCP); //创建一个socket服务，端口5867
    std::cout<<"start controller server"<<std::endl;
	//启动线程

    cs.init();
	cs.start();

    auto &mot = dynamic_cast<aris::control::EthercatMotor &>(cs.controller().motionAtAbs(0));

	//Start Web Socket//
	cs.open();

	//Receive Command//
	cs.runCmdLine();
}
