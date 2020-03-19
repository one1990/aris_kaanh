#include<iostream>
#include<aris.hpp>
#include"singleMotor.h"

using namespace std;

//调用aris库中的plan模块
using namespace aris::plan;

//创建ethercat主站控制器controller，并根据电机驱动xml文件添加从站信息，具体可以参考佳安控制器使用手册-UI界面版

/**********************************执行器参数*****************************************/
//执行器配置：半闭环/全闭环
//电机型号：MADLN15BE
//电机速度：3000 rpm
//电机最大转速： xxxx rpm
//电机侧编码器位数：23, 单圈2^23脉冲
//输出侧编码器位数：
//减速比
//...

/**********************************从站参数配置*****************************************/
//min_pos = -100000*2*Pi = -628318.52				/*最小位置，100000圈*/
//max_pos = 100000*2*Pi = 628318.52				/*最大位置，100000圈*/
//min_vel = -3000/60*2*Pi = -314.16 rad/s			/*最小速度*/
//max_vel = 3000/60*2*Pi = 314.16 rad/s			/*最大速度*/
//min_acc= -628.32 rad/s^2							/*最小加速度*/
//max_acc= 628.32  rad/s^2							/*最大加速度*/
//pos_factor = 2^23/2/Pi = 1335088.45 pulses/rad，   /*单位行程脉冲数，实际要根据传动机构参数确定，此处为单圈脉冲数*/
//pos_offset = 0								/*断电重启后电机初始位置相对零点的偏移*/
//home_pos = 0 °								/*初始位置*/
//max_pos_following_error = 10°=5/360*2*Pi = 0.175 rad					/*最大位置跟随误差*/
//max_vel_following_error = 30°/s = 5rpm =0.5236 rad/s		/*最大速度跟随误差*/
//dc_assign_activate = 0x0300					/*同步模式，0x0000：非同步模式；0x0300：同步模式*/

auto createController()->std::unique_ptr<aris::control::Controller>	
{
	std::unique_ptr<aris::control::Controller> controller(new aris::control::EthercatController);
	std::string xml_str =
		/*参数配置位置、速度、加速度除以2*Pi转换单位为rad，注意转换*/
        "<EthercatMotor phy_id=\"0\" product_code=\"0x60380005\""
        " vendor_id=\"0x0000066f\" revision_num=\"0x00010000\" dc_assign_activate=\"0x300\""
        " min_pos=\"-628318.52\" max_pos=\"628318.52\" max_vel=\"314.16\" min_vel=\"-314.16\""
        " max_acc=\"628.32\" min_acc=\"-628.32\" max_pos_following_error=\"0.5\" max_vel_following_error=\"1\""
        " home_pos=\"0\" pos_factor=\"1335088.45\" pos_offset=\"0.0\">"
        "	<SyncManagerPoolObject>"
        "       <SyncManager is_tx=\"false\"/>"
        "       <SyncManager is_tx=\"true\"/>"
        "       <SyncManager is_tx=\"false\">"
        "           <Pdo index=\"0x1600\" is_tx=\"false\">"
        "               <PdoEntry name=\"entry\" index=\"0x6040\" subindex=\"0x00\" size=\"16\"/>"
        "               <PdoEntry name=\"entry\" index=\"0x6060\" subindex=\"0x00\" size=\"8\"/>"
        "               <PdoEntry name=\"entry\" index=\"0x607a\" subindex=\"0x00\" size=\"32\"/>"
        "               <PdoEntry name=\"entry\" index=\"0x60b8\" subindex=\"0x00\" size=\"16\"/>"
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
		"               <PdoEntry name=\"entry\" index=\"0x60f4\" subindex=\"0x00\" size=\"32\"/>"
		"               <PdoEntry name=\"entry\" index=\"0x60fd\" subindex=\"0x00\" size=\"32\"/>"
        "           </Pdo>"
        "       </SyncManager>"
		"	</SyncManagerPoolObject>"
        "</EthercatMotor>";
	//在controller对象的从站池中添加一个电机从站//
	controller->slavePool().add<aris::control::EthercatMotor>().loadXmlStr(xml_str);
	return controller;
};


// 单关节正弦往复运动指令。根据构造函数可知，本条指令的接口为:moveJS --j1=0.1 --time=1 --timenum=2
// 例如，用户可以在terminal终端输入字符串:moveJS --j1=0.1，那么目标电机将以幅值0.1、默认周期1s、默认运行周期数2运动
struct MoveJSParam
{
	double j1;							//正弦运动幅值，默认当前位置current_pos，单位rad
	double time;						//正弦运动周期，默认1，单位s
	uint32_t timenum;					//运动周期数，默认为2
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

    this->motorOptions()[0] |= aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;

    //std::cout <<"option:"<< this->option() || aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER << std::endl;

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
		step_pjs = begin_pjs + param.j1 * (1 - std::cos(2 * PI*count() / time));
		
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

		step_pjs = begin_pjs - 2 * param.j1 * (1 - std::cos(2 * PI*(count() - time / 2) / time));
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
		step_pjs = begin_pjs - param.j1 * (1 - std::cos(2 * PI*(count() - totaltime + time / 2) / time));
		controller()->motionPool().at(0).setTargetPos(step_pjs);
	}

	// 实时打印函数 //
	auto &cout = controller()->mout();
	// 打印 //
	if (count() % 500 == 0)  //500ms打印一次
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
    plan_root->planPool().add<aris::plan::Clear>();
    plan_root->planPool().add<aris::plan::Recover>();
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
//	cs.interfacePool().add<aris::server::ProgramWebInterface>("", "5866", aris::core::Socket::WEB);//创建一个websocket服务，端口5866
//	cs.interfacePool().add<aris::server::WebInterface>("", "5867", aris::core::Socket::TCP); //创建一个socket服务，端口5867
    std::cout<<"start controller server"<<std::endl;
	
    std::cout<<cs.xmlString()<<std::endl;


	//启动控制器服务
    cs.init();
	cs.start();

	////aris定义的函数接口，用于设置ethercat从站，本例为maxon驱动器的时间周期
 //   auto &mot = dynamic_cast<aris::control::EthercatMotor &>(cs.controller().motionAtAbs(0));
 //   std::uint8_t inter_time = 1;
 //   mot.writeSdo(0x60C2, 0x01, &inter_time);

	//Start Web Socket//
//	cs.open();

	//Receive Command from terminal//
	cs.runCmdLine();
}
