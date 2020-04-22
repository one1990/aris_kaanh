#include <algorithm>
#include <array>
#include <stdlib.h>
#include <string>
#include <bitset>
#include "kaanh.h"
#include "robot.h"

using namespace aris::dynamic;
using namespace aris::plan;

namespace robot
{
//MoveJS的指令参数结构体，长度单位是m，角度单位是rad
//指令的执行顺序
//1、先执行prepareNrt，每条指令只执行一次
//2、然后执行executeRT,executeRT每一个循环周期(默认1ms)会被实时核调用一次，执行的总时间由用户给定
//3、执行结束后，本指令会被析构
//指令功能：某一电机或者所有电机1-cos(theta)轨迹运行，幅值为pos，周期为time，周期数为timenum
	struct MoveSParam
	{
		double pos;
		double time;
		uint32_t timenum;
        std::vector<bool> active_motor;			//目标电机
		std::vector<double> begin_pjs;			//起始位置
		std::vector<double> step_pjs;			//目标位置
	};
	auto MoveS::prepareNrt()->void
	{
		MoveSParam param;
		param.active_motor.clear();
        param.active_motor.resize(controller()->motionPool().size(), false);
		param.begin_pjs.resize(controller()->motionPool().size(), 0.0);
		param.step_pjs.resize(controller()->motionPool().size(), 0.0);
		param.pos = 0.0;
		param.time = 0.0;
		param.timenum = 0;

		//解析指令参数
		for (auto &p : cmdParams())
		{
			if (p.first == "all")
			{
                std::fill(param.active_motor.begin(), param.active_motor.end(), true);
			}
			else if (p.first == "motion_id")
			{
                param.active_motor.at(int32Param(p.first)) = true;
			}
            else if (p.first == "pos")
			{
				if (p.second == "current_pos")
				{
					param.pos = 0;
				}
				else
				{
					param.pos = doubleParam(p.first);
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
	auto MoveS::executeRT()->int
	{
        auto &param = std::any_cast<MoveSParam&>(this->param());
		auto time = static_cast<int32_t>(param.time * 1000);		//运行周期
		auto totaltime = static_cast<int32_t>(param.timenum * time);//运行总时间

		//第一个周期设置log文件名称，获取当前电机所在位置
		if (count() == 1)
		{
			controller()->logFileRawName("motion_replay");//设置log文件名称

			for (Size i = 0; i < param.active_motor.size(); ++i)
			{
				if (param.active_motor[i])
				{
					param.begin_pjs[i] = controller()->motionPool()[i].targetPos();
				}
			}

		}
		
		//1-cos(theta)轨迹运行
		for (Size i = 0; i < param.active_motor.size(); ++i)
		{
			if (param.active_motor[i])
			{
                param.step_pjs[i] = param.begin_pjs[i] + param.pos*(1.0 - std::cos(2.0 * PI*count() / time));
				controller()->motionPool().at(i).setTargetPos(param.step_pjs[i]);
			}
		}

		//打印
        auto &cout = controller()->mout();
        if (count() % 100 == 0)
		{
			for (int i = 0; i < param.active_motor.size(); i++)
			{
                cout << std::setprecision(10) << "targetpos:" << param.step_pjs[i] << "  ";
				cout << std::setprecision(10) << "actualpos:" << controller()->motionPool()[i].actualPos() << "  ";
			}
			cout << std::endl;

		}
		
        //记录
        auto &lout = controller()->lout();
		for(int i=0; i<param.active_motor.size();i++)
		{
			lout << controller()->motionPool()[i].targetPos()<<"  ";
			lout << controller()->motionPool()[i].actualPos()<<"  ";
		}
		lout << std::endl;

		//返回0表示正常结束，返回负数表示报错，返回正数表示正在执行
		return totaltime - count();
	}
	auto MoveS::collectNrt()->void {}
	MoveS::~MoveS() = default;
	MoveS::MoveS(const std::string &name) : Plan(name)
	{
		//构造函数参数说明，构造函数通过xml的格式定义本条指令的接口，name表示参数名，default表示输入参数，abbreviation表示参数名的缩写(缩写只能单个字符)
		//1 GroupParam下面的各个节点都是输入参数，如果没有给定会使用默认值
		//2 UniqueParam下面的各个节点互斥，有且只能使用其中的一个
		//3 例如，通过terminal或者socket发送“mvs --pos=0.1”，控制器实际会按照mvs --pos=0.1rad --time=1s --timenum=2 --all执行
		command().loadXmlStr(
			"<Command name=\"mvs\">"
			"	<GroupParam>"
			"		<Param name=\"pos\" default=\"current_pos\"/>"
			"		<Param name=\"time\" default=\"1.0\" abbreviation=\"t\"/>"
			"		<Param name=\"timenum\" default=\"2\" abbreviation=\"n\"/>"
			"		<UniqueParam default=\"all\">"\
			"			<Param name=\"all\" abbreviation=\"a\"/>"\
			"			<Param name=\"motion_id\" abbreviation=\"m\" default=\"0\"/>"
			"		</UniqueParam>"
			"	</GroupParam>"
			"</Command>");
	}

	auto createPlanRoot()->std::unique_ptr<aris::plan::PlanRoot>
	{
		std::unique_ptr<aris::plan::PlanRoot> plan_root(new aris::plan::PlanRoot);
		//用户自己开发指令集
		plan_root->planPool().add<robot::MoveS>();

		//aris库提供指令集
		plan_root->planPool().add<aris::plan::Enable>();
		plan_root->planPool().add<aris::plan::Disable>();
		plan_root->planPool().add<aris::plan::Start>();
		plan_root->planPool().add<aris::plan::Stop>();
		plan_root->planPool().add<aris::plan::Mode>();
		plan_root->planPool().add<aris::plan::Clear>();
		plan_root->planPool().add<aris::server::GetInfo>();
		//kaanh库提供指令集
		plan_root->planPool().add<kaanh::Home>();
		plan_root->planPool().add<kaanh::Sleep>();
		plan_root->planPool().add<kaanh::Recover>();
		plan_root->planPool().add<kaanh::Reset>();
		plan_root->planPool().add<kaanh::MoveAbsJ>();
		plan_root->planPool().add<kaanh::MoveL>();
		plan_root->planPool().add<kaanh::MoveJ>();
		plan_root->planPool().add<kaanh::MoveC>();
		plan_root->planPool().add<kaanh::Get>();
		plan_root->planPool().add<kaanh::Var>();
		plan_root->planPool().add<kaanh::Evaluate>();
		plan_root->planPool().add<kaanh::JogJ1>();
		plan_root->planPool().add<kaanh::JogJ2>();
		plan_root->planPool().add<kaanh::JogJ3>();
		plan_root->planPool().add<kaanh::JogJ4>();
		plan_root->planPool().add<kaanh::JogJ5>();
		plan_root->planPool().add<kaanh::JogJ6>();
		plan_root->planPool().add<kaanh::JogJ7>();
		plan_root->planPool().add<kaanh::JX>();
		plan_root->planPool().add<kaanh::JY>();
		plan_root->planPool().add<kaanh::JZ>();
		plan_root->planPool().add<kaanh::JRX>();
		plan_root->planPool().add<kaanh::JRY>();
		plan_root->planPool().add<kaanh::JRZ>();
		plan_root->planPool().add<kaanh::SetDH>();
		plan_root->planPool().add<kaanh::SetPG>();
		plan_root->planPool().add<kaanh::SetPPath>();
		plan_root->planPool().add<kaanh::SetUI>();
		plan_root->planPool().add<kaanh::SetDriver>();
		plan_root->planPool().add<kaanh::SaveXml>();
		plan_root->planPool().add<kaanh::ScanSlave>();
		plan_root->planPool().add<kaanh::GetEsiPdoList>();
		plan_root->planPool().add<kaanh::SetEsiPath>();
		plan_root->planPool().add<kaanh::GetXml>();
		plan_root->planPool().add<kaanh::SetXml>();
		plan_root->planPool().add<kaanh::SetCT>();
		plan_root->planPool().add<kaanh::SetVel>();
		plan_root->planPool().add<kaanh::Run>();
		plan_root->planPool().add<kaanh::MoveF>();
		plan_root->planPool().add<kaanh::Switch>();
		return plan_root;
	}
}
