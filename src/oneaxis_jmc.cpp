#include<iostream>
#include<aris.hpp>
#include"oneaxis_jmc.h"

using namespace std;
//调用aris库中的plan模块
using namespace aris::plan;

//创建ethercat主站控制器controller，并根据xml文件添加从站信息
auto createController()->std::unique_ptr<aris::control::Controller>	
{
	std::unique_ptr<aris::control::Controller> controller(new aris::control::EthercatController);
	std::string xml_str =
        "<EthercatMotor phy_id=\"0\" product_code=\"0x20181302\""
        " vendor_id=\"0x66668888\" revision_num=\"0x20181011\" dc_assign_activate=\"0x0300\""
        " min_pos=\"-536870\" max_pos=\"536870\" max_vel=\"6.28\" min_vel=\"-6.28\""
        " max_acc=\"31.4\" min_acc=\"-31.4\" max_pos_following_error=\"0.1\" max_vel_following_error=\"0.01\""
        " home_pos=\"0\" pos_factor=\"4000\" pos_offset=\"0.0\">"
		"	<SyncManagerPoolObject>"
        "       <SyncManager is_tx=\"false\"/>"
        "       <SyncManager is_tx=\"true\"/>"
        "       <SyncManager is_tx=\"false\">"
        "           <Pdo index=\"0x1600\" is_tx=\"false\">"
        "               <PdoEntry name=\"entry\" index=\"0x6040\" subindex=\"0x00\" size=\"16\"/>"
        "               <PdoEntry name=\"entry\" index=\"0x6060\" subindex=\"0x00\" size=\"8\"/>"
        "               <PdoEntry name=\"entry\" index=\"0x607a\" subindex=\"0x00\" size=\"32\"/>"
        "               <PdoEntry name=\"entry\" index=\"0x60b8\" subindex=\"0x00\" size=\"16\"/>"
        "               <PdoEntry name=\"entry\" index=\"0x60fe\" subindex=\"0x01\" size=\"32\"/>"
        "               <PdoEntry name=\"entry\" index=\"0x60fe\" subindex=\"0x02\" size=\"32\"/>"
        "           </Pdo>"
        "       </SyncManager>"
        "       <SyncManager is_tx=\"true\">"
        "           <Pdo index=\"0x1a00\" is_tx=\"true\">"
        "               <PdoEntry name=\"entry\" index=\"0x603f\" subindex=\"0x00\" size=\"16\"/>"
        "               <PdoEntry name=\"entry\" index=\"0x6041\" subindex=\"0x00\" size=\"16\"/>"
        "               <PdoEntry name=\"entry\" index=\"0x6061\" subindex=\"0x00\" size=\"8\"/>"
        "               <PdoEntry name=\"entry\" index=\"0x6064\" subindex=\"0x00\" size=\"32\"/>"
        "               <PdoEntry name=\"entry\" index=\"0x60b9\" subindex=\"0x00\" size=\"16\"/>"
        "               <PdoEntry name=\"entry\" index=\"0x60ba\" subindex=\"0x00\" size=\"32\"/>"
        "               <PdoEntry name=\"entry\" index=\"0x60bb\" subindex=\"0x00\" size=\"32\"/>"
        "               <PdoEntry name=\"entry\" index=\"0x60bc\" subindex=\"0x00\" size=\"32\"/>"
        "               <PdoEntry name=\"entry\" index=\"0x60bd\" subindex=\"0x00\" size=\"32\"/>"
        "               <PdoEntry name=\"entry\" index=\"0x60fd\" subindex=\"0x00\" size=\"32\"/>"
        "           </Pdo>"
        "       </SyncManager>"
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
auto MoveJS::prepairNrt()->void
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
				param.j1 = std::stod(p.second);
			}

		}
		else if (p.first == "time")
		{
			param.time = std::stod(p.second);
		}
		else if (p.first == "timenum")
		{
			param.timenum = std::stoi(p.second);
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
		cout << "pos"  << ":" << controller()->motionAtAbs(0).actualPos() << "  ";
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
auto MoveEAP::prepairNrt()->void
{
    auto c = controller();
	MoveEAPParam param;
    param.axis_begin_pos = 0.0;

	for (auto &p : cmdParams())
	{
		if (p.first == "pos")
		{
			param.axis_pos = std::stod(p.second);
		}
		else if (p.first == "vel")
		{	
			param.axis_vel = std::stod(p.second);
		}
		else if (p.first == "acc")
		{
			param.axis_acc = std::stod(p.second);
		}
		else if (p.first == "dec")
		{
			param.axis_dec = std::stod(p.second);
		}
		else if (p.first == "ab")
		{
			param.abs = std::stoi(p.second);
		}
	}

	this->param() = param;
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
		aris::plan::moveAbsolute(count(), param.axis_begin_pos, param.axis_begin_pos + param.axis_pos, param.axis_vel / 1000, param.axis_acc / 1000 / 1000, param.axis_dec /1000 / 1000, p, v, a, t_count);
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
		"		<Param name=\"vel\" default=\"0.02\"/>"
		"		<Param name=\"acc\" default=\"0.3\"/>"
		"		<Param name=\"dec\" default=\"-0.3\"/>"
		"		<Param name=\"ab\" default=\"0\"/>"
		"	</GroupParam>"
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
	return plan_root;
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
    cs.resetController(createController().release());
    cs.resetPlanRoot(createPlanRoot().release());
    std::cout<<"start controller server"<<std::endl;
	//启动线程
	cs.start();

	//Start Web Socket//
	cs.open();

	//Receive Command//
	cs.runCmdLine();
}
