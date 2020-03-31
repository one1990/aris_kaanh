#include <algorithm>
#include "wulingconfig.h"
#include "wuling.h"
#include <array>
#include <stdlib.h>
#include <string>
#include "planfuns.h"
#include <bitset>


using namespace aris::dynamic;
using namespace aris::plan;

namespace wulingconfig
{
	auto createControllerWuling()->std::unique_ptr<aris::control::Controller>	/*函数返回的是一个类指针，指针指向Controller,controller的类型是智能指针std::unique_ptr*/
	{
		std::unique_ptr<aris::control::Controller> controller(new aris::control::EthercatController);/*创建std::unique_ptr实例*/

		for (aris::Size i = 0; i < 6; ++i)
		{
#ifdef WIN32
			double pos_offset[6]
			{
				0,0,0,0,0,0
			};
#endif
#ifdef UNIX
			double pos_offset[6]
			{
				0.0,   0.0,   0.0,   0.0,   0.0,   0.0
			};
#endif
			double pos_factor[6]
			{
				131072.0 * 161 / 2 / PI, 131072.0 * 161 / 2 / PI, 131072.0 * 101 / 2 / PI, 131072.0 * 101 / 2 / PI, 131072.0 * 101 / 2 / PI, 131072.0 * 101 / 2 / PI
			};
			double max_pos[6]
			{
				100.0 / 360 * 2 * PI, 120.0 / 360 * 2 * PI,	120.0 / 360 * 2 * PI, 120.0 / 360 * 2 * PI, 100.0 / 360 * 2 * PI, 360.0 / 360 * 2 * PI
			};
			double min_pos[6]
			{
				-100.0 / 360 * 2 * PI, -120.0 / 360 * 2 * PI, -120.0 / 360 * 2 * PI, -120.0 / 360 * 2 * PI, -100.0 / 360 * 2 * PI, -360.0 / 360 * 2 * PI
			};
			double max_vel[6]
			{
				120.0 / 360 * 2 * PI, 120.0 / 360 * 2 * PI, 180.0 / 360 * 2 * PI, 180.0 / 360 * 2 * PI, 180.0 / 360 * 2 * PI, 180.0 / 360 * 2 * PI
			};
			double max_acc[6]
			{
				600.0 / 360 * 2 * PI, 600.0 / 360 * 2 * PI, 900.0 / 360 * 2 * PI, 900.0 / 360 * 2 * PI, 900.0 / 360 * 2 * PI, 900.0 / 360 * 2 * PI
			};

			std::string xml_str =
				"<EthercatMotor phy_id=\"" + std::to_string(i) + "\" product_code=\"0x1030\""
				" vendor_id=\"0xAB\" revision_num=\"0x10003\" dc_assign_activate=\"0x0330\""
				" min_pos=\"" + std::to_string(min_pos[i]) + "\" max_pos=\"" + std::to_string(max_pos[i]) + "\" max_vel=\"" + std::to_string(max_vel[i]) + "\" min_vel=\"" + std::to_string(-max_vel[i]) + "\""
				" max_acc=\"" + std::to_string(max_acc[i]) + "\" min_acc=\"" + std::to_string(-max_acc[i]) + "\" max_pos_following_error=\"0.1\" max_vel_following_error=\"0.5\""
				" home_pos=\"0\" pos_factor=\"" + std::to_string(pos_factor[i]) + "\" pos_offset=\"" + std::to_string(pos_offset[i]) + "\">"
				"	<SyncManagerPoolObject>"
				"		<SyncManager is_tx=\"false\"/>"
				"		<SyncManager is_tx=\"true\"/>"
				"		<SyncManager is_tx=\"false\">"
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
		}

		return controller;
	}
    auto createModelWuling(const double *robot_pm)->std::unique_ptr<aris::dynamic::Model>
    {
		aris::dynamic::PumaParam param;
		param.d1 = 0.130;
		param.a1 = 0.0;
		param.a2 = 0.613;
		param.d3 = 0.04;
		param.a3 = 0.0;
		param.d4 = 0.6005;

        param.tool0_pe[2] = 0.2525;

		auto model = aris::dynamic::createModelPuma(param);
		/*
		//根据tool0，添加一个tool1，tool1相对于tool0在x方向加上0.1m//
		auto &tool0 = model->partPool().back().markerPool().findByName("general_motion_0_i");//获取tool0

		double pq_ee_i[7];
		s_pm2pq(*tool0->prtPm(), pq_ee_i);
		pq_ee_i[0] += 0.1;//在tool0的x方向加上0.1m

		double pm_ee_i[16];
		s_pq2pm(pq_ee_i, pm_ee_i);

		auto &tool1 = model->partPool().back().markerPool().add<Marker>("tool1", pm_ee_i);//添加tool1

		//在根据tool1位姿反解到每一个关节时，需要调用下面两行代码来实现
		//tool1.setPm(pm_ee_i);
		//model->generalMotionPool()[0].updMpm();
		*/
		return std::move(model);
    }
	
    auto createUserDataType(aris::core::Calculator &cal)->void
	{
        std::cout << "create user data!" <<std::endl;
		cal.addTypename("array");
		cal.addFunction("array", std::vector<std::string>{"Matrix"}, "array", [](std::vector<std::any>&params)->std::any
		{
			return params[0];
		});
		cal.addTypename("load");
		cal.addFunction("load", std::vector<std::string>{"Matrix"}, "load", [](std::vector<std::any>&params)->std::any
		{
			if (std::any_cast<aris::core::Matrix>(params[0]).size() != 11)
			{
				THROW_FILE_LINE("input data error");
			}
			kaanh::Load a;
			auto temp = std::any_cast<aris::core::Matrix&>(params[0]).data();
			std::copy(temp, temp + 1, &a.mass);
			std::copy(temp + 1, temp + 4, &a.cog[0]);
			std::copy(temp + 4, temp + 8, &a.pq[0]);
			std::copy(temp + 8, temp + 11, &a.iner[0]);

			return a;
		});
		cal.addTypename("pose");
		cal.addFunction("pose", std::vector<std::string>{"Matrix"}, "pose", [](std::vector<std::any>&params)->std::any
		{
			if (std::any_cast<aris::core::Matrix>(params[0]).size() != 7)
			{
				THROW_FILE_LINE("input data error");
			}
			return params[0];
		});
		cal.addTypename("jointtarget");
		cal.addFunction("jointtarget", std::vector<std::string>{"Matrix"}, "jointtarget", [](std::vector<std::any>&params)->std::any
		{
			if (std::any_cast<aris::core::Matrix>(params[0]).size() != 6)
			{
				THROW_FILE_LINE("input data error");
			}
			return params[0];
		});
		cal.addTypename("robtarget");
		cal.addFunction("robtarget", std::vector<std::string>{"Matrix"}, "robtarget", [](std::vector<std::any>&params)->std::any
		{
			if (std::any_cast<aris::core::Matrix>(params[0]).size() != 7)
			{
				THROW_FILE_LINE("input data error");
			}
			return params[0];
		});


		cal.addTypename("zone");
		cal.addFunction("zone", std::vector<std::string>{"Matrix"}, "zone", [](std::vector<std::any>&params)->std::any
		{
			if (std::any_cast<aris::core::Matrix>(params[0]).size() != 2)
			{
				THROW_FILE_LINE("input data error");
			}
			kaanh::Zone z;
			auto temp = std::any_cast<aris::core::Matrix&>(params[0]).data();
			std::copy(temp, temp + 1, &z.dis);
			std::copy(temp + 1, temp + 2, &z.per);

			return z;
		});
        cal.addBinaryOperatorFunction("=", "zone", "Matrix", "zone", [](std::any &left, std::any &right)->std::any
        {
            if (std::any_cast<aris::core::Matrix>(right).size() != 2)
            {
                THROW_FILE_LINE("input data error");
            }
            kaanh::Zone z;
            auto temp = std::any_cast<aris::core::Matrix&>(right).data();
            std::copy(temp, temp + 1, &z.dis);
            std::copy(temp + 1, temp + 2, &z.per);

            left = z;
            return left;
        });
		// add zone variables
		/*
		cal.addVariable("fine", "zone", kaanh::Zone({ 0.0, 0.0 }));
		cal.addVariable("z1", "zone", kaanh::Zone({ 0.001, 0.01 }));
		cal.addVariable("z5", "zone", kaanh::Zone({ 0.005, 0.03 }));
		cal.addVariable("z10", "zone", kaanh::Zone({ 0.01, 0.05 }));
		cal.addVariable("z15", "zone", kaanh::Zone({ 0.015, 0.08 }));
		cal.addVariable("z20", "zone", kaanh::Zone({ 0.02, 0.1 }));
		cal.addVariable("z30", "zone", kaanh::Zone({ 0.03, 0.15 }));
		cal.addVariable("z40", "zone", kaanh::Zone({ 0.04, 0.2 }));
		cal.addVariable("z50", "zone", kaanh::Zone({ 0.05, 0.25 }));
		cal.addVariable("z60", "zone", kaanh::Zone({ 0.06, 0.3 }));
		cal.addVariable("z80", "zone", kaanh::Zone({ 0.08, 0.4 }));
		cal.addVariable("z100", "zone", kaanh::Zone({ 0.1, 0.5 }));
		cal.addVariable("z150", "zone", kaanh::Zone({ 0.15, 0.75 }));
		cal.addVariable("z200", "zone", kaanh::Zone({ 0.2, 1.0 }));
		*/

		cal.addTypename("speed");
		cal.addFunction("speed", std::vector<std::string>{"Matrix"}, "speed", [](std::vector<std::any>&params)->std::any
		{
			if (std::any_cast<aris::core::Matrix>(params[0]).size() != 5)
			{
				THROW_FILE_LINE("input data error");
			}
			kaanh::Speed z;
			auto temp = std::any_cast<aris::core::Matrix&>(params[0]).data();
			std::copy(temp, temp + 1, &z.w_per);
			std::copy(temp + 1, temp + 2, &z.v_tcp);
			std::copy(temp + 2, temp + 3, &z.w_tcp);
			std::copy(temp + 3, temp + 4, &z.w_ext);
			std::copy(temp + 4, temp + 5, &z.v_ext);
			
			return z;

		});
        cal.addBinaryOperatorFunction("=", "speed", "Matrix", "speed", [](std::any &left, std::any &right)->std::any
        {
            if (std::any_cast<aris::core::Matrix>(right).size() != 5)
            {
                THROW_FILE_LINE("input data error");
            }
            kaanh::Speed z;
            auto temp = std::any_cast<aris::core::Matrix&>(right).data();
            std::copy(temp, temp + 1, &z.w_per);
            std::copy(temp + 1, temp + 2, &z.v_tcp);
            std::copy(temp + 2, temp + 3, &z.w_tcp);
            std::copy(temp + 3, temp + 4, &z.w_ext);
            std::copy(temp + 4, temp + 5, &z.v_ext);

            left = z;
            return left;
        });
		// add velocity variables
		/*
        cal.addVariable("v5", "speed", kaanh::Speed({ 0.005, 0.005, 200 * aris::PI / 180, 0.0, 0.0 }));
        cal.addVariable("v10", "speed", kaanh::Speed({ 0.01, 0.01, 200 * aris::PI / 180, 0.0, 0.0 }));
        cal.addVariable("v25", "speed", kaanh::Speed({ 0.025, 0.025, 200 * aris::PI / 180, 0.0, 0.0 }));
        cal.addVariable("v30", "speed", kaanh::Speed({ 0.03, 0.03, 200 * aris::PI / 180, 0.0, 0.0 }));
        cal.addVariable("v40", "speed", kaanh::Speed({ 0.04, 0.04, 200 * aris::PI / 180, 0.0, 0.0 }));
        cal.addVariable("v50", "speed", kaanh::Speed({ 0.05, 0.05, 200 * aris::PI / 180, 0.0, 0.0 }));
        cal.addVariable("v60", "speed", kaanh::Speed({ 0.06, 0.06, 200 * aris::PI / 180, 0.0, 0.0 }));
		cal.addVariable("v80", "speed", kaanh::Speed({ 0.08, 0.08, 200 * aris::PI / 180, 0.0, 0.0 }));
		cal.addVariable("v100", "speed", kaanh::Speed({ 0.1, 0.1, 200 * aris::PI / 180, 0.0, 0.0 }));
		cal.addVariable("v150", "speed", kaanh::Speed({ 0.15, 0.15, 200 * aris::PI / 180, 0.0, 0.0 }));
		cal.addVariable("v200", "speed", kaanh::Speed({ 0.2, 0.2, 200 * aris::PI / 180, 0.0, 0.0 }));
		cal.addVariable("v300", "speed", kaanh::Speed({ 0.3, 0.3, 200 * aris::PI / 180, 0.0, 0.0 }));
		cal.addVariable("v400", "speed", kaanh::Speed({ 0.4, 0.4, 200 * aris::PI / 180, 0.0, 0.0 }));
		cal.addVariable("v500", "speed", kaanh::Speed({ 0.5, 0.5, 200 * aris::PI / 180, 0.0, 0.0 }));
		cal.addVariable("v600", "speed", kaanh::Speed({ 0.6, 0.6, 200 * aris::PI / 180, 0.0, 0.0 }));
		cal.addVariable("v800", "speed", kaanh::Speed({ 0.8, 0.8, 200 * aris::PI / 180, 0.0, 0.0 }));
		cal.addVariable("v1000", "speed", kaanh::Speed({ 1.0, 1.0, 200 * aris::PI / 180, 0.0, 0.0 }));
		cal.addVariable("v1500", "speed", kaanh::Speed({ 1.0, 1.5, 200 * aris::PI / 180, 0.0, 0.0 }));
		cal.addVariable("v2000", "speed", kaanh::Speed({ 1.0, 2.0, 200 * aris::PI / 180, 0.0, 0.0 }));
		cal.addVariable("v3000", "speed", kaanh::Speed({ 1.0, 3.0, 200 * aris::PI / 180, 0.0, 0.0 }));
		cal.addVariable("v4000", "speed", kaanh::Speed({ 1.0, 4.0, 200 * aris::PI / 180, 0.0, 0.0 }));
		cal.addVariable("v5000", "speed", kaanh::Speed({ 1.0, 5.0, 200 * aris::PI / 180, 0.0, 0.0 }));
		cal.addVariable("v6000", "speed", kaanh::Speed({ 1.0, 6.0, 200 * aris::PI / 180, 0.0, 0.0 }));
		*/

		cal.addTypename("tool");
		cal.addFunction("tool", std::vector<std::string>{"Matrix"}, "tool", [](std::vector<std::any>&params)->std::any
		{
			if (std::any_cast<aris::core::Matrix>(params[0]).size() != 6)
			{
				THROW_FILE_LINE("input data error");
			}
			return params[0];
		});
        cal.addBinaryOperatorFunction("=", "tool", "Matrix", "tool", [](std::any &left, std::any &right)->std::any
        {
            if (std::any_cast<aris::core::Matrix>(right).size() != 6)
            {
                THROW_FILE_LINE("input data error");
            }
            left = right;
            return left;
        });
		// add tool offset, inertia variables
		cal.addVariable("tool0_axis_home", "tool", aris::core::Matrix(1, 6, 0.0));
		for (int i = 1; i < 17; ++i)
		{
			cal.addVariable("tool" + std::to_string(i) + "_axis_offset", "tool", aris::core::Matrix(1, 6, 0.0));
			cal.addVariable("tool" + std::to_string(i) + "_inertia", "tool", aris::core::Matrix(1, 10, 0.0));
		}
		

		cal.addTypename("wobj");
		cal.addFunction("wobj", std::vector<std::string>{"Matrix"}, "wobj", [](std::vector<std::any>&params)->std::any
		{
			if (std::any_cast<aris::core::Matrix>(params[0]).size() != 6)
			{
				THROW_FILE_LINE("input data error");
			}
			return params[0];
		});
        cal.addBinaryOperatorFunction("=", "wobj", "Matrix", "wobj", [](std::any &left, std::any &right)->std::any
        {
            if (std::any_cast<aris::core::Matrix>(right).size() != 6)
            {
                THROW_FILE_LINE("input data error");
            }
            left = right;
            return left;
        });


	}

    auto createPauseTimeSpeed()->void
    {
        for (int i = 0; i < 101; i++)
        {
            timespeed[i] = 1.0*i / 100;
        }
    }

	auto createPlanRoot()->std::unique_ptr<aris::plan::PlanRoot>
	{
		std::unique_ptr<aris::plan::PlanRoot> plan_root(new aris::plan::PlanRoot);

		//plan_root->planPool().add<aris::plan::Enable>();
		plan_root->planPool().add<kaanh::EnableMotor>();
		plan_root->planPool().add<aris::plan::Disable>();
		plan_root->planPool().add<aris::plan::Start>();
		plan_root->planPool().add<aris::plan::Stop>();
		//plan_root->planPool().add<aris::plan::Mode>();
		plan_root->planPool().add<kaanh::MotorMode>();
		plan_root->planPool().add<aris::plan::Clear>();
		plan_root->planPool().add<kaanh::Home>();
		plan_root->planPool().add<kaanh::Sleep>();
		plan_root->planPool().add<kaanh::Recover>();
		plan_root->planPool().add<kaanh::Reset>();
		plan_root->planPool().add<aris::server::GetInfo>();

		plan_root->planPool().add<kaanh::MoveAbsJ>();
		plan_root->planPool().add<kaanh::MoveL>();
		plan_root->planPool().add<kaanh::MoveJ>();

		plan_root->planPool().add<kaanh::Get>();
		plan_root->planPool().add<kaanh::Var>();
		plan_root->planPool().add<kaanh::Evaluate>();
		
		plan_root->planPool().add<kaanh::MoveC>();
		plan_root->planPool().add<kaanh::JogC>();
		plan_root->planPool().add<kaanh::JogJ>();
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
