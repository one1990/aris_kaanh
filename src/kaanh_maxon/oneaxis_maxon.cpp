#include<iostream>
#include<aris.hpp>
#include"oneaxis_maxon.h"

using namespace std;

//调用aris库中的plan模块
using namespace aris::plan;

//创建ethercat主站控制器controller，并根据电机驱动xml文件添加从站信息，具体可以参考佳安控制器使用手册-UI界面版
auto createController()->std::unique_ptr<aris::control::Controller>	
{
	std::unique_ptr<aris::control::Controller> controller(new aris::control::EthercatController);
	std::string xml_str =
        "<EthercatMotor phy_id=\"0\" product_code=\"0x65510000\""
        " vendor_id=\"0xFB\" revision_num=\"0x01400000\" dc_assign_activate=\"0x300\""
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
        "               <PdoEntry name=\"entry\" index=\"0x60ff\" subindex=\"0x00\" size=\"32\"/>"
        "           </Pdo>"
        "       </SyncManager>"
        "       <SyncManager is_tx=\"true\">"
        "           <Pdo index=\"0x1a00\" is_tx=\"true\">"
        "               <PdoEntry name=\"entry\" index=\"0x6041\" subindex=\"0x00\" size=\"16\"/>"
        "               <PdoEntry name=\"entry\" index=\"0x6061\" subindex=\"0x00\" size=\"8\"/>"
        "               <PdoEntry name=\"entry\" index=\"0x6064\" subindex=\"0x00\" size=\"32\"/>"
        "               <PdoEntry name=\"entry\" index=\"0x606C\" subindex=\"0x00\" size=\"16\"/>"
        "           </Pdo>"
        "       </SyncManager>"
		"	</SyncManagerPoolObject>"
        "</EthercatMotor>";

	//在controller对象的从站池中添加一个电机从站//
	controller->slavePool().add<aris::control::EthercatMotor>().loadXmlStr(xml_str);
	return controller;
};


// 单关节正弦往复运动指令。根据构造函数可知，本条指令的接口为:moveJS --j1=0.1 --time=2 --timenum=2
// 其中j1为目标角度，默认为current_pos，单位为rad；time为正弦运动周期，默认为1，单位为s；timenum为运动周期数，默认为2//
// 例如，用户可以在terminal终端输入字符串:moveJS --j1=0.1，那么目标电机将以幅值0.05、默认周期1s、默认运行周期数2运动
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

	//控制器提供两条基本的函数writePdo()和readPdo()，与驱动器的交互都可以根据这两条基本函数完成
	if ((1 <= count()) && (count() <= time / 2))
	{
		// 获取当前起始点位置 //
		if (count() == 1)
		{
			//本条函数controller()->motionPool()[0].actualPos()是aris封装好的读取电机当前实际位置的函数，后面类似
			//可以用如下两条注释的代码替代，但是pos的值为电机编码器返回的脉冲数
			//int32_t pos;
			//dynamic_cast<aris::control::EthercatMotor &>(controller()->motionAtAbs(0)).readPdo(0x6064, 0x00, &pos, 32);
			begin_pjs = controller()->motionPool()[0].actualPos();
			step_pjs = controller()->motionPool()[0].actualPos();
		}
		step_pjs = begin_pjs + param.j1 * (1 - std::cos(2 * PI*count() / time)) / 2;
		
		//本条函数controller()->motionPool().at(0).setTargetPos(step_pjs)是aris封装好的写电机目标位置的函数，后面类似
		//可以用如下注释的代码替代,但是需要将step_pjs转换成电机编码器的脉冲数
		//dynamic_cast<aris::control::EthercatMotor &>(controller()->motionAtAbs(0)).writePdo(0x607a, 0x00, &step_pjs, 32);
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

	// 实时打印函数 //
	auto &cout = controller()->mout();
	// 打印 //
	if (count() % 100 == 0)
	{
		cout << "pos"  << ":" << controller()->motionAtAbs(0).actualPos() << "  ";
		cout << std::endl;
	}

	// 实时记录log函数 //
	auto &lout = controller()->lout();
	// logging //
	lout << controller()->motionAtAbs(0).actualPos() << ",";
	lout << std::endl;

	//1、本条指令的executeRT()函数每1ms被实时线程调用一次，并且实时线程有且只有1个；
	//2、本条指令的executeRT()独占实时线程，直到运行结束，如有其他指令，按照FIFO原则依次执行；
	//3、return值：大于0，继续执行；等于0，正常结束，并释放实时线程；小于0，报错退出，并释放实时线程
	//4、count()返回实时线程调用本函数的次数
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


// 将创建的运动指令(本例为MoveJS)添加到轨迹规划池planPool中，以指令的类名添加即可；添加后，本程序才接受以字符串形式发送的调用命令 //
auto createPlanRoot()->std::unique_ptr<aris::plan::PlanRoot>
{
	std::unique_ptr<aris::plan::PlanRoot> plan_root(new aris::plan::PlanRoot);

	plan_root->planPool().add<aris::plan::Enable>();//aris库提供的使能指令
	plan_root->planPool().add<aris::plan::Disable>();//aris库提供的去使能指令
	plan_root->planPool().add<aris::plan::Mode>();//aris库提供的切换电机控制模式的指令
	auto &rs = plan_root->planPool().add<aris::plan::Reset>();//aris库提供的复位指令
	rs.command().findParam("pos")->setDefaultValue("{0.01}");//设置复位指令的目标位置为0.01rad位置
	plan_root->planPool().add<MoveJS>();//本例提供的用户开发的指令
	return plan_root;
}


// 主函数
int main(int argc, char *argv[])
{
	//cs代表成员函数的引用，aris是头文件，server是命名空间，ControlServer是结构体
    auto&cs = aris::server::ControlServer::instance();//创建一个控制器服务对象
    cs.resetController(createController().release());//创建一个ethercat主站对象
    cs.resetPlanRoot(createPlanRoot().release());//创建一个plan对象
	cs.interfacePool().add<aris::server::ProgramWebInterface>("", "5866", aris::core::Socket::WEB);//创建一个websocket服务，端口5866
	cs.interfacePool().add<aris::server::WebInterface>("", "5867", aris::core::Socket::TCP); //创建一个socket服务，端口5867
    std::cout<<"start controller server"<<std::endl;
	
	//启动控制器服务
	cs.start();

	//aris定义的函数接口，用于设置ethercat从站，本例为maxon驱动器的时间周期
    auto &mot = dynamic_cast<aris::control::EthercatMotor &>(cs.controller().motionAtAbs(0));
    std::uint8_t inter_time = 1;
    mot.writeSdo(0x60C2, 0x01, &inter_time);

	//Start Web Socket//
	cs.open();

	//Receive Command from terminal//
	cs.runCmdLine();
}
