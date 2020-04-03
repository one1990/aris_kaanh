#include <algorithm>
#include <array>
#include <stdlib.h>
#include <string>
#include <bitset>
#include "kaanh.h"
#include "kaanh/kaanhconfig.h"
#include "config.h"

using namespace aris::dynamic;
using namespace aris::plan;

namespace config
{
	//创建controller，具体参考手册
    auto createController()->std::unique_ptr<aris::control::Controller>	/*函数返回的是一个类指针，指针指向Controller,controller的类型是智能指针std::unique_ptr*/
    {
        std::unique_ptr<aris::control::Controller> controller(new aris::control::EthercatController);/*创建std::unique_ptr实例*/
 
        for (aris::Size i = 0; i < 4; ++i)
        {
#ifdef WIN32
			//配置零位偏置
            double pos_offset[4]
            {
                0,0,0,0
            };
#endif
#ifdef UNIX
			//配置零位偏置
            double pos_offset[4]
            {
                0.0,   0.0,   0.0,   0.0
            };
#endif
			//配置规划值与电机count数的比例系数=2的N次方*减速比/2Π，其中N为编码器位数
            double pos_factor[4]
            {
                131072.0 * 129.6 / 2 / PI, -131072.0 * 100 / 2 / PI, 131072.0 * 101 / 2 / PI, 131072.0 * 101 / 2 / PI
            };
			//关节最大位置，角度单位转换成弧度单位
            double max_pos[4]
            {
                170.0 / 360 * 2 * PI, 40.0 / 360 * 2 * PI,	150.0 / 360 * 2 * PI,  150.0 / 360 * 2 * PI
            };
			//关节最小位置，角度单位转换成弧度单位
            double min_pos[4]
            {
                -170.0 / 360 * 2 * PI, -165.0 / 360 * 2 * PI, -125.0 / 360 * 2 * PI, -125.0 / 360 * 2 * PI
            };
			//关节最大速度，角度单位转换成弧度单位
            double max_vel[4]
            {
                230.0 / 360 * 2 * PI, 300.0 / 360 * 2 * PI, 300.0 / 360 * 2 * PI, 300.0 / 360 * 2 * PI
            };
			//关节最大加速度，角度单位转换成弧度单位
            double max_acc[4]
            {
                1150.0 / 360 * 2 * PI, 1500.0 / 360 * 2 * PI, 1500.0 / 360 * 2 * PI, 1500.0 / 360 * 2 * PI
            };
			//根据从站的ESI文件，以及上述参数配置从站信息，格式是xml格式
            std::string xml_str =
                "<EthercatMotor phy_id=\"" + std::to_string(i) + "\" product_code=\"0x01\""
                " vendor_id=\"0x00000748\" revision_num=\"0x0002\" dc_assign_activate=\"0x0300\""
                " min_pos=\"" + std::to_string(min_pos[i]) + "\" max_pos=\"" + std::to_string(max_pos[i]) + "\" max_vel=\"" + std::to_string(max_vel[i]) + "\" min_vel=\"" + std::to_string(-max_vel[i]) + "\""
                " max_acc=\"" + std::to_string(max_acc[i]) + "\" min_acc=\"" + std::to_string(-max_acc[i]) + "\" max_pos_following_error=\"0.1\" max_vel_following_error=\"0.5\""
                " home_pos=\"0\" pos_factor=\"" + std::to_string(pos_factor[i]) + "\" pos_offset=\"" + std::to_string(pos_offset[i]) + "\">"
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
				"               <PdoEntry name=\"entry\" index=\"0x606C\" subindex=\"0x00\" size=\"32\"/>"
				"           </Pdo>"
				"       </SyncManager>"
				"	</SyncManagerPoolObject>"
                "</EthercatMotor>";

            controller->slavePool().add<aris::control::EthercatMotor>().loadXmlStr(xml_str);//添加从站
        }
        return controller;
    }
    //创建model，具体参考手册
	auto createModel(const double *robot_pm)->std::unique_ptr<aris::dynamic::Model>
    {
		aris::dynamic::Serial3Param param;
		param.a1 = 0.7;
		param.a2 = 0.5;
		param.a3 = 0.6;

		auto model = aris::dynamic::createModelSerial3Axis(param);
		return std::move(model);
    }
	
	//MoveAbs的指令参数结构体，单位为m或rad制
	//每条指令先执行prepareNrt;然后执行executeRT,执行的时间由用户指定或者规划出;执行结束后，本指令会被析构
	struct MoveAbsParam
	{
		std::vector<double> axis_begin_pos_vec;	//起始位置
		std::vector<double> axis_pos_vec;		//目标位置
		std::vector<double> axis_vel_vec;		//目标速度
		std::vector<double> axis_acc_vec;		//目标加速度
		std::vector<double> axis_dec_vec;		//目标加速度
		std::vector<double> axis_jerk_vec;		//目标跃度
		std::vector<int> active_motor;			//目标电机
	};
	auto MoveAbs::prepareNrt()->void
	{
		std::cout << "mvaj:" << std::endl;
		MoveAbsParam param;

		param.active_motor.clear();
		param.active_motor.resize(controller()->motionPool().size(), 0);
		param.axis_begin_pos_vec.resize(controller()->motionPool().size(), 0.0);
		
		//解析指令参数
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
					param.axis_pos_vec.resize(controller()->motionPool().size(), p.toDouble());
				}
				else if (p.size() == controller()->motionPool().size())
				{
					param.axis_pos_vec.assign(p.begin(), p.end());
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
			else if (cmd_param.first == "jerk")
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

		this->param() = param;
		std::vector<std::pair<std::string, std::any>> ret_value;
		ret() = ret_value;
	}
	auto MoveAbs::executeRT()->int
	{
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

		aris::Size total_count{ 1 };
		for (Size i = 0; i < param->active_motor.size(); ++i)
		{
			if (param->active_motor[i])
			{
				double p, v, a, j;
				aris::Size t_count;
				//梯形轨迹规划
				//aris::plan::moveAbsolute(count(),
				//	param->axis_begin_pos_vec[i], param->axis_pos_vec[i],
				//	param->axis_vel_vec[i] / 1000, param->axis_acc_vec[i] / 1000 / 1000, param->axis_dec_vec[i] / 1000 / 1000,
				//	p, v, a, t_count);

				//s形规划//
				traplan::sCurve(count(), param->axis_begin_pos_vec[i], param->axis_pos_vec[i],
					param->axis_vel_vec[i] / 1000, param->axis_acc_vec[i] / 1000 / 1000, param->axis_jerk_vec[i] / 1000 / 1000 / 1000,
					p, v, a, j, t_count);
				controller()->motionPool()[i].setTargetPos(p);
				if (i < model()->motionPool().size())
				{
					model()->motionPool()[i].setMp(p);
				}
				total_count = std::max(total_count, t_count);

			}
		}
		
		// 设置末端位置 //
		//double ee[3]{ 1.68070023071933, 0.35446729674924, -0.22165182186613 };
		//solver.setPosEE(ee);

		// 获得求解器，求反解 //
		auto &solver = dynamic_cast<aris::dynamic::Serial3InverseKinematicSolver&>(model()->solverPool()[0]);
		// 设置解，一共4个，设为4时会选最优解 //
		solver.setWhichRoot(4);
		if (solver.kinPos())return -1;

		// 获取正解求解器，求正解//
		auto &forward = dynamic_cast<aris::dynamic::ForwardKinematicSolver&>(model()->solverPool()[1]);
		forward.kinPos();

		//返回0表示正常结束，返回负数表示报错，返回正数表示正在执行
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
	
	//MoveLine的指令参数结构体，单位为m或rad
	//每条指令先执行prepareNrt;然后执行executeRT,执行的时间由用户指定或者规划出;执行结束后，本指令会被析构
	struct MoveLineParam
	{
		std::vector<double> axis_begin_pos_vec;	//起始位置
		std::vector<double> axis_pos_vec;		//目标位置
		double axis_vel_vec;		//目标速度
		double axis_acc_vec;		//目标加速度
		double axis_dec_vec;		//目标加速度
		double axis_jerk_vec;		//目标跃度
	};
	auto MoveLine::prepareNrt()->void
	{
		std::cout << "movel:" << std::endl;
		MoveLineParam param;
		param.axis_begin_pos_vec.resize(model()->motionPool().size(), 0.0);

		//解析指令参数，单位为m或rad
		for (auto cmd_param : cmdParams())
		{
			if (cmd_param.first == "pos")
			{
				auto p = matrixParam(cmd_param.first);
				if (p.size() == model()->motionPool().size())
				{
					param.axis_pos_vec.assign(p.begin(), p.end());
				}
				else
				{
					THROW_FILE_LINE("");
				}
				//tcp位置的最大最小值需要用户自己设定，这里默认设置为0.5
				for (Size i = 0; i < model()->motionPool().size(); ++i)
				{
					if (param.axis_pos_vec[i] > 0.5 || param.axis_pos_vec[i] < -0.5)
						THROW_FILE_LINE("input pos beyond range");
				}
			}
			else if (cmd_param.first == "acc")
			{
				param.axis_acc_vec = doubleParam(cmd_param.first);

				//线加速度的最大最小值需要用户自己设定，这里默认设置为50
				if (param.axis_acc_vec > 50 || param.axis_acc_vec < -50)
					THROW_FILE_LINE("input acc beyond range");


			}
			else if (cmd_param.first == "vel")
			{
				param.axis_vel_vec = doubleParam(cmd_param.first);

				//线加速度的最大最小值需要用户自己设定，这里默认设置为10
				if (param.axis_vel_vec > 10 || param.axis_vel_vec < -10)
					THROW_FILE_LINE("input vel beyond range");
			}
			else if (cmd_param.first == "dec")
			{
				param.axis_dec_vec = doubleParam(cmd_param.first);

				//线加速度的最大最小值需要用户自己设定
				if (param.axis_dec_vec > 50 || param.axis_dec_vec < -50)
					THROW_FILE_LINE("input dec beyond range");
			}
			else if (cmd_param.first == "jerk")
			{
				param.axis_jerk_vec = doubleParam(cmd_param.first);

				//线加速度的最大最小值需要用户自己设定
				if (param.axis_jerk_vec > 100 || param.axis_jerk_vec < -100)
					THROW_FILE_LINE("input jerk beyond range");
			}
		}

		this->param() = param;
		
		for (auto &option : motorOptions())	option |= USE_TARGET_POS;//使用模型
		
		//返回值格式
		std::vector<std::pair<std::string, std::any>> ret_value;
		ret() = ret_value;
	}
	auto MoveLine::executeRT()->int
	{
		auto param = std::any_cast<MoveLineParam>(&this->param());
		static double xyz_norm;
		if (count() == 1)
		{
			model()->generalMotionPool().at(0).getMpe(param->axis_begin_pos_vec.data());
			xyz_norm = std::sqrt(std::pow(param->axis_pos_vec[0]-param->axis_begin_pos_vec[0], 2) + std::pow(param->axis_pos_vec[1]-param->axis_begin_pos_vec[1], 2) + std::pow(param->axis_pos_vec[2]-param->axis_begin_pos_vec[2], 2));
		}

		aris::Size total_count{ 1 };

		double p, v, a, j;
		aris::Size t_count;
		//梯形轨迹规划
		//aris::plan::moveAbsolute(count(),
		//	0.0, xyz_norm,
		//	param->axis_vel_vec / 1000, param->axis_acc_vec / 1000 / 1000, param->axis_dec_vec / 1000 / 1000,
		//	p, v, a, t_count);

		//s形规划//
		traplan::sCurve(count(), 0.0, xyz_norm,
			param->axis_vel_vec / 1000, param->axis_acc_vec / 1000 / 1000, param->axis_jerk_vec / 1000 / 1000 / 1000,
			p, v, a, j, t_count);
		total_count = std::max(total_count, t_count);

		// 获得求解器，求反解 //
		auto &solver = dynamic_cast<aris::dynamic::Serial3InverseKinematicSolver&>(model()->solverPool()[0]);
		
		// 设置解，一共4个，设为4时会选最优解 //
		solver.setWhichRoot(4);
		
		// 设置末端位置 //
		double ee[3]{ param->axis_begin_pos_vec[0] + p*(param->axis_pos_vec[0] - param->axis_begin_pos_vec[0])/xyz_norm, 
			param->axis_begin_pos_vec[1] + p * (param->axis_pos_vec[1] - param->axis_begin_pos_vec[1]) / xyz_norm, 
		param->axis_begin_pos_vec[2] + p * (param->axis_pos_vec[2] - param->axis_begin_pos_vec[2]) / xyz_norm};
		solver.setPosEE(ee);

		//求反解，模型会相应的给电机发指令
		if (solver.kinPos())return -1;//prepareNrt中添加了USE_TARGET_POS,控制才会生效

		//返回0表示正常结束，返回负数表示报错，返回正数表示正在执行
		return total_count - count();
	}
	MoveLine::~MoveLine() = default;
	MoveLine::MoveLine(const std::string &name) : Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"movel\">"
			"	<GroupParam>"
			"		<Param name=\"pos\" default=\"{0.0, 0.0, 0.0}\"/>"
			"		<Param name=\"vel\" default=\"1.0\"/>"
			"		<Param name=\"acc\" default=\"1.0\"/>"
			"		<Param name=\"dec\" default=\"1.0\"/>"
			"		<Param name=\"jerk\" default=\"10.0\"/>"
			"	</GroupParam>"
			"</Command>");
	}

	//每一条用户开发的指令都要添加到如下planPool()中，格式参考其他指令
	auto createPlanRoot()->std::unique_ptr<aris::plan::PlanRoot>
	{
		std::unique_ptr<aris::plan::PlanRoot> plan_root(new aris::plan::PlanRoot);

		plan_root->planPool().add<aris::plan::Enable>();
		plan_root->planPool().add<aris::plan::Disable>();
		plan_root->planPool().add<aris::plan::Start>();
		plan_root->planPool().add<aris::plan::Stop>();
		plan_root->planPool().add<aris::plan::Mode>();
		plan_root->planPool().add<aris::plan::Clear>();

		plan_root->planPool().add<kaanh::Recover>();
		plan_root->planPool().add<kaanh::Reset>();
		plan_root->planPool().add<config::MoveAbs>();
		plan_root->planPool().add<kaanh::MoveL>();
		plan_root->planPool().add<kaanh::MoveJ>();
		plan_root->planPool().add<kaanh::MoveC>();

		plan_root->planPool().add<aris::server::GetInfo>();
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
		plan_root->planPool().add<kaanh::ClearCon>();
		plan_root->planPool().add<kaanh::SetCon>();
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
