#include <iostream>
#include <aris.hpp> //程序引用aris库的头文件
#include <string>
#include <pthread.h>
#include <time.h>
#include "test.h"
#include <unistd.h>


//创建ethercat主站控制器controller，并根据电机驱动xml文件添加从站信息，具体可以参考佳安控制器使用手册-UI界面版
auto createController()->std::unique_ptr<aris::control::Controller>
{
	std::unique_ptr<aris::control::Controller> controller(new aris::control::EthercatController);
	std::string xml_str =
		"<EthercatMotor phy_id=\"0\" product_code=\"0x00030924\" "
		" vendor_id=\"154\" revision_num=\"0x00010420\" dc_assign_activate=\"0x300\" sync0_shift_ns=\"1000000\""
		" min_pos=\"-100000\" max_pos=\"100000\" max_vel=\"26.18\" min_vel=\"-26.18\""
		" max_acc=\"26.18\" min_acc=\"-26.18\" max_pos_following_error=\"0.5\" max_vel_following_error=\"0.2\""
		" home_pos=\"0\" pos_factor=\"79205\" pos_offset=\"0.0\">"
		"	<SyncManagerPoolObject>"
		"       <SyncManager is_tx=\"false\"/>"
		"       <SyncManager is_tx=\"true\"/>"
		"       <SyncManager is_tx=\"false\">"
		"           <Pdo index=\"0x1605\">"
		"               <PdoEntry name=\"entry\" index=\"0x607a\" subindex=\"0x00\" size=\"32\"/>"
		"               <PdoEntry name=\"entry\" index=\"0x60ff\" subindex=\"0x00\" size=\"32\"/>"
		"               <PdoEntry name=\"entry\" index=\"0x6071\" subindex=\"0x00\" size=\"16\"/>"
		"               <PdoEntry name=\"entry\" index=\"0x6072\" subindex=\"0x00\" size=\"16\"/>"
		"               <PdoEntry name=\"entry\" index=\"0x6040\" subindex=\"0x00\" size=\"16\"/>"
		"               <PdoEntry name=\"entry\" index=\"0x6060\" subindex=\"0x00\" size=\"8\"/>"
		"           </Pdo>"
		"       </SyncManager>"
		"       <SyncManager is_tx=\"true\">"
		"           <Pdo index=\"0x1a04\" is_tx=\"true\">"
		"               <PdoEntry name=\"entry\" index=\"0x6064\" subindex=\"0x00\" size=\"32\"/>"
		"               <PdoEntry name=\"entry\" index=\"0x60f4\" subindex=\"0x00\" size=\"32\"/>"
		"               <PdoEntry name=\"entry\" index=\"0x6077\" subindex=\"0x00\" size=\"16\"/>"
		"               <PdoEntry name=\"entry\" index=\"0x6041\" subindex=\"0x00\" size=\"16\"/>"
		"               <PdoEntry name=\"entry\" index=\"0x6061\" subindex=\"0x00\" size=\"8\"/>"
		"           </Pdo>"
		"       </SyncManager>"
		"	</SyncManagerPoolObject>"
		"</EthercatMotor>";
	//在controller对象的从站池中添加第一个电机从站，phy_id=0//
	controller->slavePool().add<aris::control::EthercatMotor>().loadXmlStr(xml_str);
	
	xml_str =
		"<EthercatMotor phy_id=\"1\" product_code=\"0x00030924\" "
		" vendor_id=\"154\" revision_num=\"0x00010420\" dc_assign_activate=\"0x300\" sync0_shift_ns=\"1000000\""
		" min_pos=\"-100000\" max_pos=\"100000\" max_vel=\"26.18\" min_vel=\"-26.18\""
		" max_acc=\"26.18\" min_acc=\"-26.18\" max_pos_following_error=\"0.5\" max_vel_following_error=\"0.2\""
		" home_pos=\"0\" pos_factor=\"79205\" pos_offset=\"0.0\">"
		"	<SyncManagerPoolObject>"
		"       <SyncManager is_tx=\"false\"/>"
		"       <SyncManager is_tx=\"true\"/>"
		"       <SyncManager is_tx=\"false\">"
		"           <Pdo index=\"0x1605\">"
		"               <PdoEntry name=\"entry\" index=\"0x607a\" subindex=\"0x00\" size=\"32\"/>"
		"               <PdoEntry name=\"entry\" index=\"0x60ff\" subindex=\"0x00\" size=\"32\"/>"
		"               <PdoEntry name=\"entry\" index=\"0x6071\" subindex=\"0x00\" size=\"16\"/>"
		"               <PdoEntry name=\"entry\" index=\"0x6072\" subindex=\"0x00\" size=\"16\"/>"
		"               <PdoEntry name=\"entry\" index=\"0x6040\" subindex=\"0x00\" size=\"16\"/>"
		"               <PdoEntry name=\"entry\" index=\"0x6060\" subindex=\"0x00\" size=\"8\"/>"
		"           </Pdo>"
		"       </SyncManager>"
		"       <SyncManager is_tx=\"true\">"
		"           <Pdo index=\"0x1a04\" is_tx=\"true\">"
		"               <PdoEntry name=\"entry\" index=\"0x6064\" subindex=\"0x00\" size=\"32\"/>"
		"               <PdoEntry name=\"entry\" index=\"0x60f4\" subindex=\"0x00\" size=\"32\"/>"
		"               <PdoEntry name=\"entry\" index=\"0x6077\" subindex=\"0x00\" size=\"16\"/>"
		"               <PdoEntry name=\"entry\" index=\"0x6041\" subindex=\"0x00\" size=\"16\"/>"
		"               <PdoEntry name=\"entry\" index=\"0x6061\" subindex=\"0x00\" size=\"8\"/>"
		"           </Pdo>"
		"       </SyncManager>"
		"	</SyncManagerPoolObject>"
		"</EthercatMotor>";
	//在controller对象的从站池中添加第二个电机从站，phy_id=1//
	controller->slavePool().add<aris::control::EthercatMotor>().loadXmlStr(xml_str);

	xml_str =
		"<EthercatSlave phy_id=\"2\" product_code=\"0x00201\""
		" vendor_id=\"0x00000A09\" revision_num=\"0x64\" dc_assign_activate=\"0x00\">"
		"	<SyncManagerPoolObject>"
		"		<SyncManager is_tx=\"false\">"
		"			<Pdo index=\"0x1600\" is_tx=\"false\">"
		"				<PdoEntry name=\"Dout_0_7\" index=\"0x7001\" subindex=\"0x01\" size=\"8\"/>"
		"			</Pdo>"
		"		</SyncManager>"
		"		<SyncManager is_tx=\"false\">"
		"			<Pdo index=\"0x1601\" is_tx=\"false\">"
		"				<PdoEntry name=\"Dout_8_15\" index=\"0x7001\" subindex=\"0x02\" size=\"8\"/>"
		"			</index_1601>"
		"		</SyncManager>"
		"		<SyncManager is_tx=\"true\">"
		"			<Pdo index=\"0x1a00\" is_tx=\"true\">"
		"				<PdoEntry name=\"Din_0_7\" index=\"0x6001\" subindex=\"0x01\" size=\"8\"/>"
		"			</Pdo>"
		"			<Pdo index=\"0x1a01\" is_tx=\"true\">"
		"				<PdoEntry name=\"Din_8_15\" index=\"0x6001\" subindex=\"0x02\" size=\"8\"/>"
		"			</Pdo>"
		"		</SyncManager>"
		"	</SyncManagerPoolObject>"
		"</EthercatSlave>";
	//在controller对象的从站池中添加第1个io从站，phy_id=2//
	controller->slavePool().add<aris::control::EthercatSlave>().loadXmlStr(xml_str);

	return controller;
};

// 关节正弦往复运动指令。根据构造函数可知，本条指令的接口为:moveJS --j1=0.1 --j2=0.1 --time=1 --timenum=2
// 其中j1为1轴目标角度，j2为2轴目标角度,默认为current_pos，单位为rad；time为正弦运动周期，默认为1，单位为s；timenum为运动周期数，默认为2//
// 例如，用户可以在terminal终端输入字符串:moveJS --j1=0.1，那么目标电机将以幅值0.05、默认周期1s、默认运行周期数2运动
struct MoveJSParam
{
	double j1;
	double time;
	uint32_t timenum;
	std::vector<int> active_motor;			//目标电机
	std::vector<double> begin_pjs;			//起始位置
	std::vector<double> step_pjs;			//目标位置
};
//解析指令
auto MoveJS::prepareNrt()->void
{
	MoveJSParam param;

	param.active_motor.clear();
	param.active_motor.resize(controller()->motionPool().size(), 0);
	param.begin_pjs.resize(controller()->motionPool().size(), 0.0);
	param.step_pjs.resize(controller()->motionPool().size(), 0.0);
	param.j1 = 0.0;
	param.time = 0.0;
	param.timenum = 0;

	for (auto &p : cmdParams())
	{
		if (p.first == "all")
		{
			std::fill(param.active_motor.begin(), param.active_motor.end(), 1);
		}
		else if (p.first == "motion_id")
		{
			param.active_motor.at(int32Param(p.first)) = 1;
		}
		else if (p.first == "j1")
		{
			if (p.second == "current_pos")
			{
				param.j1 = 0;
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
//计算每一毫秒的目标位置
auto MoveJS::executeRT()->int
{
	auto &param = std::any_cast<MoveJSParam&>(this->param());
	auto time = static_cast<int32_t>(param.time * 1000);
	auto totaltime = static_cast<int32_t>(param.timenum * time);

	for (int i = 0; i < param.active_motor.size(); i++)
	{
		//控制器提供两条基本的函数writePdo()和readPdo()，与驱动器的交互都可以根据这两条基本函数完成
		if ((1 <= count()) && (count() <= time / 2))
		{
			// 获取当前起始点位置 //
			if (count() == 1)
			{
				//本条函数controller()->motionPool()[i].actualPos()是aris封装好的读取电机当前实际位置的函数，后面类似
				//可以用如下两条注释的代码替代，但是pos的值为电机编码器返回的脉冲数
				//int32_t pos;
				//dynamic_cast<aris::control::EthercatMotor &>(controller()->motionAtAbs(0)).readPdo(0x6064, 0x00, &pos, 32);
				param.begin_pjs[i] = controller()->motionPool()[i].targetPos();
				param.step_pjs[i] = controller()->motionPool()[i].targetPos();
			}
			param.step_pjs[i] = param.begin_pjs[i] + param.j1 * (1 - std::cos(2 * PI*count() / time)) / 2;

			//本条函数controller()->motionPool().at(i).setTargetPos(param.step_pjs[i])是aris封装好的写电机目标位置的函数，后面类似
			//可以用如下注释的代码替代,但是需要将param.step_pjs[i]转换成电机编码器的脉冲数
			//dynamic_cast<aris::control::EthercatMotor &>(controller()->motionAtAbs(0)).writePdo(0x607a, 0x00, &param.step_pjs[i], 32);
			controller()->motionPool().at(i).setTargetPos(param.step_pjs[i]);
		}
		else if ((time / 2 < count()) && (count() <= totaltime - time / 2))
		{
			// 获取当前起始点位置 //
			if (count() == time / 2 + 1)
			{
				param.begin_pjs[i] = controller()->motionPool()[i].targetPos();
				param.step_pjs[i] = controller()->motionPool()[i].targetPos();
			}

			param.step_pjs[i] = param.begin_pjs[i] - 2 * param.j1 * (1 - std::cos(2 * PI*(count() - time / 2) / time)) / 2;
			controller()->motionPool().at(i).setTargetPos(param.step_pjs[i]);

		}
		else if ((totaltime - time / 2 < count()) && (count() <= totaltime))
		{
			// 获取当前起始点位置 //
			if (count() == totaltime - time / 2 + 1)
			{
				param.begin_pjs[i] = controller()->motionPool()[i].targetPos();
				param.step_pjs[i] = controller()->motionPool()[i].targetPos();
			}
			param.step_pjs[i] = param.begin_pjs[i] - param.j1 * (1 - std::cos(2 * PI*(count() - totaltime + time / 2) / time)) / 2;
			controller()->motionPool().at(i).setTargetPos(param.step_pjs[i]);
		}

	}

	// 实时打印函数 //
	auto &cout = controller()->mout();
	auto &lout = controller()->lout();

	// 打印 //
	if (count() % 100 == 0)
	{
		cout << "pos" << ":" << controller()->motionAtAbs(0).targetPos() << "  ";
		cout << std::endl;
	}

	//读写io//
	{
		static std::uint8_t value_dq{ 0x01 };
		static std::uint8_t value_di{ 0x00 };
		if (count() % 1000 == 0)
		{
			//EtherCAT IO板卡输出口实现“走马灯”
			value_dq = value_dq << 1;
			if (value_dq == 0) value_dq = 0x01;

			//成员函数controller()->mout()是实时核打印函数接口，成员函数controller()->lout()是实时核log函数接口
			cout << "count:" << std::dec << count() << std::endl;

			//成员函数ecSlavePool()创建从站vector, at(2)表示第一个从站，即EtherCAT IO板卡
			ecController()->slavePool().at(2).writePdo(0x7001, 0x02, &value_dq, 8);	//writePdo是写函数，第一个形参是index，第二个形参是subindex，第三个形参写DO的数值，第四个形参表示写操作的bit数
			ecController()->slavePool().at(2).readPdo(0x6001, 0x01, &value_di, 8);	//readPdo是读函数，第一个形参是index，第二个形参是subindex，第三个形参读DI的数值，第四个形参表示读操作的bit数
		}
	}

	// logging //
	lout << "Pos:" << controller()->motionAtAbs(0).actualPos() << ",";
	lout << "Vel:" << controller()->motionAtAbs(0).actualVel() << ",";
	lout << std::endl;

	//1、本条指令的executeRT()函数每1ms被实时线程调用一次，并且实时线程有且只有1个；
	//2、本条指令的executeRT()独占实时线程，直到运行结束，如有其他指令，按照FIFO原则依次执行；
	//3、return值：大于0，继续执行；等于0，正常结束，并释放实时线程；小于0，报错退出，并释放实时线程
	//4、count()返回实时线程调用本函数的次数
	return totaltime - count();
}
auto MoveJS::collectNrt()->void {}
//定义指令形式，默认值
MoveJS::MoveJS(const std::string &name) :Plan(name)
{
	command().loadXmlStr(
		"<Command name=\"moveJS\">"
		"	<GroupParam>"
		"		<Param name=\"j1\" default=\"current_pos\"/>"
		"		<Param name=\"time\" default=\"1.0\" abbreviation=\"t\"/>"
		"		<Param name=\"timenum\" default=\"2\" abbreviation=\"n\"/>"
		"		<UniqueParam default=\"all\">"\
		"			<Param name=\"all\" abbreviation=\"a\"/>"\
		"			<Param name=\"motion_id\" abbreviation=\"m\" default=\"0\"/>"
		"		</UniqueParam>"
		"	</GroupParam>"
		"</Command>");
}
//moveJS --j1=current_pos --time=1.0 --timenum=2//向终端发送命令，其中time表示运行周期时间，timenum为运行周期数
//moveJS --j1=current_pos -t=1.0 -n=2//可根据说明进行简化输入,-t即代表--time,-n代表--timenum


// 将创建的运动指令(本例为MoveJS,Move)添加到轨迹规划池planPool中，以指令的类名添加即可；添加后，本程序才接受以字符串形式发送的调用命令 //
auto createPlanRoot()->std::unique_ptr<aris::plan::PlanRoot>
{
	std::unique_ptr<aris::plan::PlanRoot> plan_root(new aris::plan::PlanRoot);
	plan_root->planPool().add<aris::plan::Enable>();//aris库提供的使能指令     en
	plan_root->planPool().add<aris::plan::Disable>();//aris库提供的去使能指令  ds
	plan_root->planPool().add<aris::plan::Clear>();//aris库提供的cl指令       cl
	plan_root->planPool().add<aris::plan::Mode>();//aris库提供的切换电机控制模式的指令  md
	plan_root->planPool().add<aris::plan::Show>();//aris库提供的显示当前位置的指令  sh
	auto &rs = plan_root->planPool().add<aris::plan::Reset>();//aris库提供的复位指令   rs
	rs.command().findParam("pos")->setDefaultValue("{0.01}");//设置复位指令的目标位置为0.01rad位置
	plan_root->planPool().add<MoveJS>();//本例提供的用户开发的指令
	return plan_root;
}

int NUM=0;
void  *func1(void *arg)
{
    //int a=0;
    while(1)
   {

        std::cout << "func1 ="<<++NUM<<std::endl;
        //std::this_thread::sleep_for(std::chrono::seconds(2));
        sleep(1);
    }
}

void  *func2(void *arg)
{
   // int a=0;
    while(1)
   {

        std::cout << "func2="<<--NUM<<std::endl;
        //std::this_thread::sleep_for(std::chrono::seconds(10));
        sleep(10);
    }
}

int main()
{
    std::cout << "start testing IO board" << std::endl;
	
	//cs代表成员函数的引用，aris是头文件ds，server是命名空间，ControlServer是结构体
	auto&cs = aris::server::ControlServer::instance();//创建一个控制器服务对象	
	cs.resetController(createController().release());//创建一个ethercat主站对象
	cs.resetPlanRoot(createPlanRoot().release());//创建一个plan对象
	std::cout << "start controller server" << std::endl;


    //启动控制器服务
	cs.init();
	cs.start();

	//设置elmo驱动器最大力矩//
	auto &c = dynamic_cast<aris::control::EthercatMotor &>(cs.controller().motionAtAbs(0));
	uint16_t value = 1000;
	c.writePdo(0x6072, 0x00, &value);

    //使能接口函数
	cs.executeCmd("md --mode=8");//切换电机到csp模式
	cs.executeCmd("en");//使能电机，切电机模式和本条指令要一起使用
	cs.executeCmd("moveJS --j1=0.5 --time=4 --timenum=10");//控制电机正弦运动10个周期，峰峰值0.5rad

    int b=add(1,2);
    std::cout<<"addfun"<<b<<std::endl;

   /* std::thread ttt([]()->void
    {

        while(true)
        {
            int a=0;
            std::cout << "aaaaa  "<<++a<<std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }


    });*/


    pthread_t tid1;
    pthread_attr_t pthread_pro1;
    pthread_attr_init(& pthread_pro1);
    pthread_attr_setschedpolicy(& pthread_pro1,SCHED_FIFO);//
    struct sched_param s_parm1;
    s_parm1.sched_priority=10;//sched_get_priority_max(SCHED_FIFO);
    pthread_attr_setschedparam(& pthread_pro1,&s_parm1);

    if(pthread_create(&tid1,&pthread_pro1,func1,NULL))
    {
        std::cout << "create thread error  "<<std::endl;
        return -1;
    }

    pthread_t tid2;
    pthread_attr_t pthread_pro2;
    pthread_attr_init(& pthread_pro2);
    pthread_attr_setschedpolicy(& pthread_pro2,SCHED_FIFO);//
    struct sched_param s_parm2;
    s_parm2.sched_priority=sched_get_priority_max(SCHED_FIFO);
    pthread_attr_setschedparam(& pthread_pro2,&s_parm2);

    if(pthread_create(&tid2,&pthread_pro2,func2,NULL))
    {
        std::cout << "create thread error  "<<std::endl;
        return -1;
    }

	
	//非实时线程睡眠100秒钟
     //std::this_thread::sleep_for(std::chrono::seconds(100));
     while(1)
     {
       std::this_thread::sleep_for(std::chrono::seconds(1));
     }
	
     //Receive Command from terminal//
     cs.runCmdLine();

	//关闭实时线程
    cs.stop();

	return 0;
}
