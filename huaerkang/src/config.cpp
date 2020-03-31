#include <algorithm>
#include <array>
#include <stdlib.h>
#include <string>
#include <bitset>
#include "kaanh.h"
#include "config.h"

using namespace aris::dynamic;
using namespace aris::plan;

namespace config
{
    auto createController()->std::unique_ptr<aris::control::Controller>	/*函数返回的是一个类指针，指针指向Controller,controller的类型是智能指针std::unique_ptr*/
    {
        std::unique_ptr<aris::control::Controller> controller(aris::robot::createControllerRokaeXB4());/*创建std::unique_ptr实例*/
        controller->slavePool().clear();	//清除slavePool中的元素，后面重新添加
        for (aris::Size i = 0; i < 4; ++i)
        {
#ifdef WIN32
            double pos_offset[4]
            {
                0,0,0,0
            };
#endif
#ifdef UNIX
            double pos_offset[4]
            {
                0.0,   0.0,   0.0,   0.0
            };
#endif
            double pos_factor[4]
            {
                131072.0 * 129.6 / 2 / PI, -131072.0 * 100 / 2 / PI, 131072.0 * 101 / 2 / PI, 131072.0 * 101 / 2 / PI
            };
            double max_pos[4]
            {
                170.0 / 360 * 2 * PI, 40.0 / 360 * 2 * PI,	150.0 / 360 * 2 * PI,  150.0 / 360 * 2 * PI
            };
            double min_pos[4]
            {
                -170.0 / 360 * 2 * PI, -165.0 / 360 * 2 * PI, -125.0 / 360 * 2 * PI, -125.0 / 360 * 2 * PI
            };
            double max_vel[4]
            {
                230.0 / 360 * 2 * PI, 300.0 / 360 * 2 * PI, 300.0 / 360 * 2 * PI, 300.0 / 360 * 2 * PI
            };
            double max_acc[4]
            {
                1150.0 / 360 * 2 * PI, 1500.0 / 360 * 2 * PI, 1500.0 / 360 * 2 * PI, 1500.0 / 360 * 2 * PI
            };

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

            controller->slavePool().add<aris::control::EthercatMotor>().loadXmlStr(xml_str);
        }
        return controller;
    }
    auto createModel(const double *robot_pm)->std::unique_ptr<aris::dynamic::Model>
    {
		aris::dynamic::Serial3Param param;
		param.a1 = 0.7;
		param.a2 = 0.5;
		param.a3 = 0.6;

		auto model = aris::dynamic::createModelSerial3Axis(param);
		return std::move(model);
    }
	
	auto createPlanRoot()->std::unique_ptr<aris::plan::PlanRoot>
	{
		std::unique_ptr<aris::plan::PlanRoot> plan_root(new aris::plan::PlanRoot);

		plan_root->planPool().add<aris::plan::Enable>();
		plan_root->planPool().add<aris::plan::Disable>();
		plan_root->planPool().add<aris::plan::Start>();
		plan_root->planPool().add<aris::plan::Stop>();
		plan_root->planPool().add<aris::plan::Mode>();
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
