#include<iostream>
#include<aris.hpp>
#include"kaanh.h"
#include"oneaxis_maxon.h"
#include <QDateTime>
#include <QFile>

using namespace std;

//调用aris库中的plan模块
using namespace aris::plan;
using namespace aris::dynamic;
//创建ethercat主站控制器controller，并根据电机驱动xml文件添加从站信息，具体可以参考佳安控制器使用手册-UI界面版
auto createController()->std::unique_ptr<aris::control::Controller>	
{
	std::unique_ptr<aris::control::Controller> controller(new aris::control::EthercatController);
    controller->slavePool().clear();
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

	//在controller对象的从站池中添加一个电机从站//
    controller->slavePool().add<aris::control::EthercatMotor>().loadXmlStr(xml_str);
    return controller;
};

// 多关节正弦往复运动指令。根据构造函数可知，本条指令的接口为:moveJS --j1=0.1 --j2=0.1 --time=1 --timenum=2
// 其中j1为1轴目标角度，j2为2轴目标角度,默认为current_pos，单位为rad；time为正弦运动周期，默认为1，单位为s；timenum为运动周期数，默认为2//
// 例如，用户可以在terminal终端输入字符串:moveJS --j1=0.1，那么目标电机将以幅值0.05、默认周期1s、默认运行周期数2运动
struct MoveJSParam
{
	double j1;
    //double j2;
	double time;
	uint32_t timenum;
};

//解析指令
auto MoveJS::prepareNrt()->void
{
	MoveJSParam param;

	param.j1 = 0.0;
    //param.j2 = 0.0;
	param.time = 0.0;
	param.timenum = 0;

	for (auto &p : cmdParams())
	{
		if (p.first == "j1")
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
        /*else if (p.first == "j2")
        {
            if (p.second == "current_pos")
            {
                param.j2 = 0;
            }
            else
            {
                param.j2 = doubleParam(p.first);
            }

        }*/
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
    static double begin_pjs;//,begin_pjs_j2;
    static double step_pjs;//,step_pjs_j2;

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
            begin_pjs = controller()->motionPool()[0].targetPos();
            step_pjs = controller()->motionPool()[0].targetPos();
            //begin_pjs_j2 = controller()->motionPool()[1].targetPos();
            //step_pjs_j2 = controller()->motionPool()[1].targetPos();
		}
		step_pjs = begin_pjs + param.j1 * (1 - std::cos(2 * PI*count() / time)) / 2;
        //step_pjs_j2 = begin_pjs_j2 + param.j2 * (1 - std::cos(2 * PI*count() / time)) / 2;
		//本条函数controller()->motionPool().at(0).setTargetPos(step_pjs)是aris封装好的写电机目标位置的函数，后面类似
		//可以用如下注释的代码替代,但是需要将step_pjs转换成电机编码器的脉冲数
		//dynamic_cast<aris::control::EthercatMotor &>(controller()->motionAtAbs(0)).writePdo(0x607a, 0x00, &step_pjs, 32);
		controller()->motionPool().at(0).setTargetPos(step_pjs);
        //controller()->motionPool().at(1).setTargetPos(step_pjs_j2);
	}
	else if ((time / 2 < count()) && (count() <= totaltime - time / 2))
	{
		// 获取当前起始点位置 //
		if (count() == time / 2 + 1)
		{
            begin_pjs = controller()->motionPool()[0].targetPos();
            step_pjs = controller()->motionPool()[0].targetPos();

            //begin_pjs_j2 = controller()->motionPool()[1].targetPos();
           // step_pjs_j2 = controller()->motionPool()[1].targetPos();
		}

		step_pjs = begin_pjs - 2 * param.j1 * (1 - std::cos(2 * PI*(count() - time / 2) / time)) / 2;
		controller()->motionPool().at(0).setTargetPos(step_pjs);

        //step_pjs_j2 = begin_pjs_j2 - 2 * param.j2 * (1 - std::cos(2 * PI*(count() - time / 2) / time)) / 2;
        //controller()->motionPool().at(1).setTargetPos(step_pjs_j2);
	}
	else if ((totaltime - time / 2 < count()) && (count() <= totaltime))
	{
		// 获取当前起始点位置 //
		if (count() == totaltime - time / 2 + 1)
		{
            begin_pjs = controller()->motionPool()[0].targetPos();
            step_pjs = controller()->motionPool()[0].targetPos();

            //begin_pjs_j2 = controller()->motionPool()[1].targetPos();
            //step_pjs_j2 = controller()->motionPool()[1].targetPos();
		}
		step_pjs = begin_pjs - param.j1 * (1 - std::cos(2 * PI*(count() - totaltime + time / 2) / time)) / 2;
		controller()->motionPool().at(0).setTargetPos(step_pjs);

        //step_pjs_j2 = begin_pjs_j2 - param.j2 * (1 - std::cos(2 * PI*(count() - totaltime + time / 2) / time)) / 2;
        //controller()->motionPool().at(1).setTargetPos(step_pjs_j2);
	}

	// 实时打印函数 //
	auto &cout = controller()->mout();
	// 打印 //
	if (count() % 100 == 0)
	{
        cout << "pos"  << ":" << controller()->motionAtAbs(0).targetPos() << "  ";
        //cout << "pos2"  << ":" << controller()->motionAtAbs(1).targetPos() << "  ";

		cout << std::endl;
	}

	// 实时记录log函数 //
	auto &lout = controller()->lout();
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
		"	</GroupParam>"
		"</Command>");
}
//moveJS --j1=current_pos --time=1.0 --timenum=2//向终端发送命令，其中time表示运行周期时间，timenum为运行周期数
//moveJS --j1=current_pos -t=1.0 -n=2//可根据说明进行简化输入,-t即代表--time,-n代表--timenum

//多关节单向运动指令。根据构造函数可知，本条指令的接口为:move --j1=0.1 --j2=0.1 --time=1 --timenum=2
struct MoveParam
{
    double j1;
    double time;
    uint32_t timenum;
};

//解析指令
auto Move::prepareNrt()->void
{
    MoveParam param;

    param.j1 = 0.0;
    param.time = 0.0;
    param.timenum = 0;

    for (auto &p : cmdParams())
    {
        if (p.first == "j1")
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
auto Move::executeRT()->int
{
    auto &param = std::any_cast<MoveParam&>(this->param());
    auto time = static_cast<int32_t>(param.time * 1000);
    auto totaltime = static_cast<int32_t>(param.timenum * time);
    static double begin_pjs;
    static double step_pjs;

    //控制器提供两条基本的函数writePdo()和readPdo()，与驱动器的交互都可以根据这两条基本函数完成
    //if ((1 <= count()) && (count() <= time / 2))
    {
        // 获取当前起始点位置 //
        if (count() == 1)
        {
            //本条函数controller()->motionPool()[0].actualPos()是aris封装好的读取电机当前实际位置的函数，后面类似
            //可以用如下两条注释的代码替代，但是pos的值为电机编码器返回的脉冲数
            //int32_t pos;
            //dynamic_cast<aris::control::EthercatMotor &>(controller()->motionAtAbs(0)).readPdo(0x6064, 0x00, &pos, 32);
            begin_pjs = controller()->motionPool()[0].actualPos();
            //step_pjs = controller()->motionPool()[0].actualPos();
            cout << "beginpos"  << ":" << begin_pjs << "  ";
            cout << std::endl;
        }
        //本条函数controller()->motionPool().at(0).setTargetPos(step_pjs)是aris封装好的写电机目标位置的函数，后面类似
        //可以用如下注释的代码替代,但是需要将step_pjs转换成电机编码器的脉冲数
        //dynamic_cast<aris::control::EthercatMotor &>(controller()->motionAtAbs(0)).writePdo(0x607a, 0x00, &step_pjs, 32);
        step_pjs = begin_pjs + param.j1 * count() / time;
        cout << "steppos"  << ":" << step_pjs << "  ";
        cout << std::endl;
        controller()->motionPool().at(0).setTargetPos(step_pjs);

    }
    // 实时打印函数 //
    auto &cout = controller()->mout();
    // 打印 //
    if (count() % 100 == 0)
    {
        cout << "pos"  << ":" << controller()->motionAtAbs(0).actualPos() << "  ";
        cout << std::endl;
        cout << "vel"  << ":" << controller()->motionAtAbs(0).actualVel() << "  ";
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
auto Move::collectNrt()->void {}
//定义指令形式，默认值
Move::Move(const std::string &name) :Plan(name)
{
    command().loadXmlStr(
        "<Command name=\"move\">"
        "	<GroupParam>"
        "		<Param name=\"j1\" default=\"current_pos\"/>"
        "		<Param name=\"time\" default=\"1.0\" abbreviation=\"t\"/>"
        "		<Param name=\"timenum\" default=\"2\" abbreviation=\"n\"/>"
        "	</GroupParam>"
        "</Command>");
}


struct MoveAbsParam
{
    std::vector<double> axis_begin_pos_vec;
    std::vector<double> axis_pos_vec;
    std::vector<double> axis_vel_vec;
    std::vector<double> axis_acc_vec;
    std::vector<double> axis_dec_vec;
    std::vector<double> axis_jerk_vec;
    std::vector<int> active_motor;
};
auto MoveAbs::prepareNrt()->void
{
    std::cout << "mvaj:" << std::endl;
    MoveAbsParam param;

    param.active_motor.clear();
    param.active_motor.resize(controller()->motionPool().size(), 0);
    param.axis_begin_pos_vec.resize(controller()->motionPool().size(), 0.0);
    for (auto cmd_param : cmdParams())
    {
        if (cmd_param.first == "all")
        {
            std::fill(param.active_motor.begin(), param.active_motor.end(), 1);
        }
        else if (cmd_param.first == "motion_id")
        {
            param.active_motor.at(int32Param(cmd_param.first)) = 1;
        }
        else if (cmd_param.first == "pos")
        {
            auto p = matrixParam(cmd_param.first);
            if (p.size() == 1)
            {
                param.axis_pos_vec.resize(controller()->motionPool().size(), p.toDouble());//将axis_pos_vec动态分配为电机数量大小的数组，全部填充p
            }
            else if (p.size() == controller()->motionPool().size())
            {
                param.axis_pos_vec.assign(p.begin(), p.end());//将p的值依次赋值给axis_pos_vec
            }
            else
            {
                THROW_FILE_LINE("");
            }
            for (Size i = 0; i < controller()->motionPool().size(); ++i)
            {
                if (param.axis_pos_vec[i] > controller()->motionPool()[i].maxPos() || param.axis_pos_vec[i] < controller()->motionPool()[i].minPos())
                    THROW_FILE_LINE("input pos beyond range");
            }
        }
        else if (cmd_param.first == "acc")
        {
            auto a = matrixParam(cmd_param.first);

            if (a.size() == 1)
            {
                param.axis_acc_vec.resize(controller()->motionPool().size(), a.toDouble());
            }
            else if (a.size() == controller()->motionPool().size())
            {
                param.axis_acc_vec.assign(a.begin(), a.end());
            }
            else
            {
                THROW_FILE_LINE("");
            }
            for (Size i = 0; i < controller()->motionPool().size(); ++i)
            {
                if (param.axis_acc_vec[i] > controller()->motionPool()[i].maxAcc() || param.axis_acc_vec[i] < controller()->motionPool()[i].minAcc())
                    THROW_FILE_LINE("input acc beyond range");
            }

        }
        else if (cmd_param.first == "vel")
        {
            auto v = matrixParam(cmd_param.first);

            if (v.size() == 1)
            {
                param.axis_vel_vec.resize(controller()->motionPool().size(), v.toDouble());
            }
            else if (v.size() == controller()->motionPool().size())
            {
                param.axis_vel_vec.assign(v.begin(), v.end());
            }
            else
            {
                THROW_FILE_LINE("");
            }
            for (Size i = 0; i < controller()->motionPool().size(); ++i)
            {
                if (param.axis_vel_vec[i] > controller()->motionPool()[i].maxVel() || param.axis_vel_vec[i] < controller()->motionPool()[i].minVel())
                    THROW_FILE_LINE("input vel beyond range");
                std::cout <<"vel:" << controller()->motionPool()[i].minVel()<<std::endl;
            }
        }
        else if (cmd_param.first == "dec")
        {
            auto d = matrixParam(cmd_param.first);

            if (d.size() == 1)
            {
                param.axis_dec_vec.resize(controller()->motionPool().size(), d.toDouble());
            }
            else if (d.size() == controller()->motionPool().size())
            {
                param.axis_dec_vec.assign(d.begin(), d.end());
            }
            else
            {
                THROW_FILE_LINE("");
            }
            for (Size i = 0; i < controller()->motionPool().size(); ++i)
            {
                if (param.axis_dec_vec[i] > controller()->motionPool()[i].maxAcc() || param.axis_dec_vec[i] < controller()->motionPool()[i].minAcc())
                    THROW_FILE_LINE("input dec beyond range");

            }
        }
        else if (cmd_param.first == "jerk")//加加速度
        {
            auto d = matrixParam(cmd_param.first);

            if (d.size() == 1)
            {
                param.axis_jerk_vec.resize(controller()->motionPool().size(), d.toDouble());
            }
            else if (d.size() == controller()->motionPool().size())
            {
                param.axis_jerk_vec.assign(d.begin(), d.end());
            }
            else
            {
                THROW_FILE_LINE("");
            }
            for (Size i = 0; i < controller()->motionPool().size(); ++i)
            {
                if (param.axis_jerk_vec[i] > controller()->motionPool()[i].maxAcc()*100.0 || param.axis_jerk_vec[i] < controller()->motionPool()[i].minAcc()*100.0)
                    THROW_FILE_LINE("input jerk beyond range");
            }
        }
    }

    std::cout<<"param finished"<<std::endl;
    this->param() = param;
    for (auto &option : motorOptions()) option |= aris::plan::Plan::CHECK_NONE;
    std::vector<std::pair<std::string, std::any>> ret_value;
    ret() = ret_value;
}
auto MoveAbs::executeRT()->int
{
    long long currentTime = QDateTime::currentDateTime().toMSecsSinceEpoch();

    auto param = std::any_cast<MoveAbsParam>(&this->param());

    if (count() == 1)
    {
        for (Size i = 0; i < param->active_motor.size(); ++i)
        {
            if (param->active_motor[i])
            {
                param->axis_begin_pos_vec[i] = controller()->motionPool()[i].actualPos();
            }
        }
    }

    double p, v, a, j;
    aris::Size total_count{ 1 };
    for (Size i = 0; i < param->active_motor.size(); ++i)
    {
        if (param->active_motor[i])
        {

            aris::Size t_count;
            //梯形轨迹规划
            aris::plan::moveAbsolute(count(),
                param->axis_begin_pos_vec[i], param->axis_pos_vec[i],
                param->axis_vel_vec[i] / 1000, param->axis_acc_vec[i] / 1000 / 1000, param->axis_dec_vec[i] / 1000 / 1000,
                p, v, a, t_count);

            //s形规划//
            //traplan::sCurve(count(), param->axis_begin_pos_vec[i], param->axis_pos_vec[i],
            //    param->axis_vel_vec[i] / 1000, param->axis_acc_vec[i] / 1000 / 1000, param->axis_jerk_vec[i] / 1000 / 1000 / 1000,
            //    p, v, a, j, t_count);
            controller()->motionPool()[i].setTargetPos(p);
//            if (i < model()->motionPool().size())
//            {
//                model()->motionPool()[i].setMp(p);
//            }
            total_count = std::max(total_count, t_count);

        }
    }

    if (count() % 100 == 0)
    {
        cout << "pos"  << ":" << controller()->motionAtAbs(0).actualPos() << "  ";
        cout << std::endl;
        //cout << "endpos"  << ":" << p << "  ";
        //cout << "beginpos2"  << ":" << param->axis_begin_pos_vec[1] << "  ";
        //cout << "endpos2"  << ":" << param->axis_pos_vec[1] << "  ";
        cout << "vel"  << ":" << controller()->motionAtAbs(0).actualVel() << "  ";
        cout << std::endl;
    }


    // 实时记录log函数 //
    auto &lout = controller()->lout();
    // logging //
    lout << "actPos:" << controller()->motionAtAbs(0).actualPos() << ",";
    lout << "actVel:" << controller()->motionAtAbs(0).actualVel() << ",";
    //lout << std::endl;
    lout << "Pos:" << p << ",";
    lout << "Vel:" << v << ",";
    lout << std::endl;
//    // 获得求解器 //
    //auto &solver = dynamic_cast<aris::dynamic::Serial3InverseKinematicSolver&>(model()->solverPool()[1]);

//    // 设置末端位置 //
   // double ee[3]{ 1.68070023071933, 0.35446729674924, -0.22165182186613 };
    //solver.setPosEE(ee);

//    // 设置解，一共4个，设为4时会选最优解 //
    //solver.setWhichRoot(4);

//    // 求解 //
    //if (solver.kinPos())return -1;

    if(timetmp != 0)
    {
        QFile file("myfile.txt");
        if(!file.open(QIODevice::ReadWrite))
        {
            //qDebug()<<file.errorString();
        }
        qint64 pos;
        pos = file.size();
        //重新定位文件输入位置，这里是定位到文件尾端
        file.seek(pos);
        QString gap = QString::number(currentTime-timetmp)+"\n";
        file.write(gap.toLatin1().data());
        file.close();
    }
    timetmp = currentTime;

    return total_count - count();
}
MoveAbs::~MoveAbs() = default;
MoveAbs::MoveAbs(const std::string &name) : Plan(name)
{
    command().loadXmlStr(
        "<Command name=\"mvaj\">"
        "	<GroupParam>"
        "		<Param name=\"pos\" default=\"0.0\"/>"
        "		<Param name=\"vel\" default=\"1.0\"/>"
        "		<Param name=\"acc\" default=\"1.0\"/>"
        "		<Param name=\"dec\" default=\"1.0\"/>"
        "		<Param name=\"jerk\" default=\"10.0\"/>"
        "		<UniqueParam default=\"all\">"\
        "			<Param name=\"all\" abbreviation=\"a\"/>"\
        "			<Param name=\"motion_id\" abbreviation=\"m\" default=\"0\"/>"
        "		</UniqueParam>"
        "	</GroupParam>"
        "</Command>");
}

// 将创建的运动指令(本例为MoveJS,Move)添加到轨迹规划池planPool中，以指令的类名添加即可；添加后，本程序才接受以字符串形式发送的调用命令 //
auto createPlanRoot()->std::unique_ptr<aris::plan::PlanRoot>
{
    std::unique_ptr<aris::plan::PlanRoot> plan_root(new aris::plan::PlanRoot);
    plan_root->planPool().add<aris::plan::Enable>();//aris库提供的使能指令     en
    plan_root->planPool().add<aris::plan::Disable>();//aris库提供的去使能指令  ds
    plan_root->planPool().add<aris::plan::Clear>();//aris库提供的cl指令       cl
    plan_root->planPool().add<aris::plan::Mode>();//aris库提供的切换电机控制模式的指令  md
    plan_root->planPool().add<aris::plan::Show>();//aris库提供的显示当前位置的指令  sh
    plan_root->planPool().add<kaanh::Get>();//aris库提供的切换电机控制模式的指令  md
    auto &rs = plan_root->planPool().add<aris::plan::Reset>();//aris库提供的复位指令   rs
	rs.command().findParam("pos")->setDefaultValue("{0.01}");//设置复位指令的目标位置为0.01rad位置
    plan_root->planPool().add<MoveJS>();//本例提供的用户开发的指令
    plan_root->planPool().add<Move>();//本例提供的用户开发的指令
    plan_root->planPool().add<MoveAbs>();//本例提供的用户开发的指令
    return plan_root;
}

// 主函数
int main(int argc, char *argv[])
{
	//cs代表成员函数的引用，aris是头文件ds，server是命名空间，ControlServer是结构体
    auto&cs = aris::server::ControlServer::instance();//创建一个控制器服务对象

    cs.resetController(createController().release());//创建一个ethercat主站对象
    cs.resetPlanRoot(createPlanRoot().release());//创建一个plan对象
    cs.interfacePool().add<aris::server::ProgramWebInterface>("", "5866", aris::core::Socket::WEB);//创建一个websocket服务，端口5866
    cs.interfacePool().add<aris::server::WebInterface>("", "5867", aris::core::Socket::TCP); //创建一个socket服务，端口5867
    std::cout<<"start controller server"<<std::endl;
	
    //cs.controller().setSamplePeriodNs(2000000);//设置通讯周期为2ms

    //std::cout << cs.xmlString() <<std::endl;

    //启动控制器服务
    cs.init();
    cs.start();

    auto &c = dynamic_cast<aris::control::EthercatMotor &>(cs.controller().motionAtAbs(0));
    uint16_t value = 1000;
    c.writePdo(0x6072, 0x00, &value);
	//Start Web Socket//
    cs.open();

	//Receive Command from terminal//
    cs.runCmdLine();
}
