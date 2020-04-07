#include <algorithm>
#include "kaanh/kaanhconfig.h"
#include "kaanh/kinematic.h"
#include "kaanh.h"
#include <array>
#include <stdlib.h>
#include <string>
#include "kaanh/planfuns.h"
#include <bitset>


using namespace aris::dynamic;
using namespace aris::plan;

namespace kaanhconfig
{
	auto createControllerEA()->std::unique_ptr<aris::control::Controller>
	{
		std::unique_ptr<aris::control::Controller> controller(new aris::control::EthercatController);
		std::string xml_str =
			"<EthercatMotion phy_id=\"0\" product_code=\"0x60380007\""
			" vendor_id=\"0x0000066F\" revision_num=\"0x00010000\" dc_assign_activate=\"0x0300\""
			" min_pos=\"0.01\" max_pos=\"0.23\" max_vel=\"0.125\" min_vel=\"-0.125\""
			" max_acc=\"2.0\" min_acc=\"-2.0\" max_pos_following_error=\"0.005\" max_vel_following_error=\"0.005\""
			" home_pos=\"0\" pos_factor=\"-3355443200\" pos_offset=\"0.0\">"
			"	<SyncManagerPoolObject>"
			"		<SyncManager is_tx=\"false\"/>"
			"		<SyncManager is_tx=\"true\"/>"
			"		<SyncManager is_tx=\"false\">"
			"			<Pdo index=\"0x1600\" is_tx=\"false\">"
			"				<PdoEntry name=\"control_word\" index=\"0x6040\" subindex=\"0x00\" size=\"16\"/>"
			"				<PdoEntry name=\"mode_of_operation\" index=\"0x6060\" subindex=\"0x00\" size=\"8\"/>"
			"				<PdoEntry name=\"target_pos\" index=\"0x607A\" subindex=\"0x00\" size=\"32\"/>"
			"				<PdoEntry name=\"target_vel\" index=\"0x60FF\" subindex=\"0x00\" size=\"32\"/>"
			"				<PdoEntry name=\"offset_vel\" index=\"0x60B1\" subindex=\"0x00\" size=\"32\"/>"
			"				<PdoEntry name=\"targer_tor\" index=\"0x6071\" subindex=\"0x00\" size=\"16\"/>"
			"				<PdoEntry name=\"offset_tor\" index=\"0x60B2\" subindex=\"0x00\" size=\"16\"/>"
			"			</Pdo>"
			"		</SyncManager>"
			"		<SyncManager is_tx=\"true\">"
			"			<Pdo index=\"0x1A00\" is_tx=\"true\">"
			"				<PdoEntry name=\"status_word\" index=\"0x6041\" subindex=\"0x00\" size=\"16\"/>"
			"				<PdoEntry name=\"mode_of_display\" index=\"0x6061\" subindex=\"0x00\" size=\"8\"/>"
			"				<PdoEntry name=\"pos_actual_value\" index=\"0x6064\" subindex=\"0x00\" size=\"32\"/>"
			"				<PdoEntry name=\"vel_actual_value\" index=\"0x606c\" subindex=\"0x00\" size=\"32\"/>"
            "				<PdoEntry name=\"tor_actual_value\" index=\"0x6077\" subindex=\"0x00\" size=\"16\"/>"
			"			</Pdo>"
			"		</SyncManager>"
			"	</SyncManagerPoolObject>"
			"</EthercatMotion>";
		controller->slavePool().add<aris::control::EthercatMotor>().loadXmlStr(xml_str);
		return controller;
	};

	auto createControllerRokaeXB4()->std::unique_ptr<aris::control::Controller>	/*函数返回的是一个类指针，指针指向Controller,controller的类型是智能指针std::unique_ptr*/
	{
		std::unique_ptr<aris::control::Controller> controller(aris::robot::createControllerRokaeXB4());/*创建std::unique_ptr实例*/

#ifdef UNIX

        dynamic_cast<aris::control::Motor&>(controller->slavePool()[0]).setPosOffset(0.083326167813560906);
        dynamic_cast<aris::control::Motor&>(controller->slavePool()[1]).setPosOffset(0.40688722035956698);
        dynamic_cast<aris::control::Motor&>(controller->slavePool()[2]).setPosOffset(-0.063596878644675794);
        dynamic_cast<aris::control::Motor&>(controller->slavePool()[3]).setPosOffset(0.65575523199999997);
        dynamic_cast<aris::control::Motor&>(controller->slavePool()[4]).setPosOffset(-1.49538803280913);
        dynamic_cast<aris::control::Motor&>(controller->slavePool()[5]).setPosOffset(-1.76544794265974);
		
        for(int i=0; i<6; i++)
        {
             dynamic_cast<aris::control::EthercatMotor&>(controller->slavePool()[i]).setDcAssignActivate(0x300);
        }
       // controller->slavePool().add<aris::control::EthercatSlave>();
       // controller->slavePool().back().setPhyId(6);
       // dynamic_cast<aris::control::EthercatSlave&>(controller->slavePool().back()).scanInfoForCurrentSlave();
       // dynamic_cast<aris::control::EthercatSlave&>(controller->slavePool().back()).scanPdoForCurrentSlave();
       // dynamic_cast<aris::control::EthercatSlave&>(controller->slavePool().back()).setDcAssignActivate(0x300);

       // controller->slavePool().add<aris::control::EthercatSlave>();
       // controller->slavePool().back().setPhyId(7);
        //dynamic_cast<aris::control::EthercatSlave&>(controller->slavePool().back()).scanInfoForCurrentSlave();
        //dynamic_cast<aris::control::EthercatSlave&>(controller->slavePool().back()).scanPdoForCurrentSlave();


        controller->slavePool().add<aris::control::EthercatSlave>();
        controller->slavePool().back().setPhyId(6);
        dynamic_cast<aris::control::EthercatSlave&>(controller->slavePool().back()).scanInfoForCurrentSlave();
        dynamic_cast<aris::control::EthercatSlave&>(controller->slavePool().back()).scanPdoForCurrentSlave();
        dynamic_cast<aris::control::EthercatSlave&>(controller->slavePool().back()).setDcAssignActivate(0x00);
#endif

		return controller;
	};
	auto createModelRokaeXB4(const double *robot_pm)->std::unique_ptr<aris::dynamic::Model>
	{
		std::unique_ptr<aris::dynamic::Model> model = std::make_unique<aris::dynamic::Model>("model");

		// 设置重力 //
		const double gravity[6]{ 0.0,0.0,-9.8,0.0,0.0,0.0 };
		model->environment().setGravity(gravity);

		// 添加变量 //
		model->calculator().addVariable("PI", "string" , aris::core::Matrix(PI));

		// add part //
		auto &p1 = model->partPool().add<Part>("L1");
		auto &p2 = model->partPool().add<Part>("L2");
		auto &p3 = model->partPool().add<Part>("L3");
		auto &p4 = model->partPool().add<Part>("L4");
		auto &p5 = model->partPool().add<Part>("L5");
		auto &p6 = model->partPool().add<Part>("L6");

		// add joint //
		const double j1_pos[3]{ 0.0, 0.0, 0.176 };
		const double j2_pos[3]{ 0.04, -0.0465, 0.3295, };
		const double j3_pos[3]{ 0.04, 0.0508, 0.6045 };
		const double j4_pos[3]{ -0.1233, 0.0, 0.6295, };
		const double j5_pos[3]{ 0.32, -0.03235, 0.6295, };
		const double j6_pos[3]{ 0.383, 0.0, 0.6295, };

		const double j1_axis[6]{ 0.0, 0.0, 1.0 };
		const double j2_axis[6]{ 0.0, 1.0, 0.0 };
		const double j3_axis[6]{ 0.0, 1.0, 0.0 };
		const double j4_axis[6]{ 1.0, 0.0, 0.0 };
		const double j5_axis[6]{ 0.0, 1.0, 0.0 };
		const double j6_axis[6]{ 1.0, 0.0, 0.0 };

		auto &j1 = model->addRevoluteJoint(p1, model->ground(), j1_pos, j1_axis);
		auto &j2 = model->addRevoluteJoint(p2, p1, j2_pos, j2_axis);
		auto &j3 = model->addRevoluteJoint(p3, p2, j3_pos, j3_axis);
		auto &j4 = model->addRevoluteJoint(p4, p3, j4_pos, j4_axis);
		auto &j5 = model->addRevoluteJoint(p5, p4, j5_pos, j5_axis);
		auto &j6 = model->addRevoluteJoint(p6, p5, j6_pos, j6_axis);

		// add actuation //
		auto &m1 = model->addMotion(j1);
		auto &m2 = model->addMotion(j2);
		auto &m3 = model->addMotion(j3);
		auto &m4 = model->addMotion(j4);
		auto &m5 = model->addMotion(j5);
		auto &m6 = model->addMotion(j6);

		// add ee general motion //
		double pq_ee_i[]{ 0.398, 0.0, 0.6295, 0.0, 0.0, 0.0, 1.0 };		//x方向加上0.1
		double pm_ee_i[16];
		double pm_ee_j[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };

		s_pq2pm(pq_ee_i, pm_ee_i);

		auto &makI = p6.markerPool().add<Marker>("ee_makI", pm_ee_i);
		auto &makJ = model->ground().markerPool().add<Marker>("ee_makJ", pm_ee_j);
		auto &ee = model->generalMotionPool().add<aris::dynamic::GeneralMotion>("ee", &makI, &makJ, false);

		/*
		double pq_ee_i[]{ 0.5, 0.0, 0.6295, 0.0, 0.0, 0.0, 1.0 };
		auto &tool1 = p6.markerPool().add<Marker>("tool1", pm_ee_i);
		tool1.prtPm();
		*/

		// change robot pose //
		if (robot_pm)
		{
			p1.setPm(s_pm_dot_pm(robot_pm, *p1.pm()));
			p2.setPm(s_pm_dot_pm(robot_pm, *p2.pm()));
			p3.setPm(s_pm_dot_pm(robot_pm, *p3.pm()));
			p4.setPm(s_pm_dot_pm(robot_pm, *p4.pm()));
			p5.setPm(s_pm_dot_pm(robot_pm, *p5.pm()));
			p6.setPm(s_pm_dot_pm(robot_pm, *p6.pm()));
			j1.makJ().setPrtPm(s_pm_dot_pm(robot_pm, *j1.makJ().prtPm()));
		}

		// add solver
		auto &inverse_kinematic = model->solverPool().add<aris::dynamic::PumaInverseKinematicSolver>();
		auto &forward_kinematic = model->solverPool().add<ForwardKinematicSolver>();

		inverse_kinematic.allocateMemory();
		forward_kinematic.allocateMemory();

		inverse_kinematic.setWhichRoot(8);

		return model;
	}	
	auto createModelRokae()->std::unique_ptr<aris::dynamic::Model>
	{
		aris::dynamic::PumaParam param;
		param.d1 = 0.3295;
		param.a1 = 0.04;
		param.a2 = 0.275;
		param.d3 = 0.0;
		param.a3 = 0.025;
		param.d4 = 0.28;

		param.tool0_pe[2] = 0.078;

		param.iv_vec =
		{
			{ 0.00000000000000,   0.00000000000000,   0.00000000000000,   0.00000000000000,   0.00000000000000,   0.00000000000000,   0.59026333537827,   0.00000000000000,   0.00000000000000,   0.00000000000000 },
			{ 0.00000000000000, -0.02551872200978,   0.00000000000000,   3.05660683326413,   2.85905166943306,   0.00000000000000,   0.00000000000000, -0.00855352993039, -0.09946674483372, -0.00712210734359 },
			{ 0.00000000000000,   0.00000000000000,   0.00000000000000,   0.02733022277747,   0.00000000000000,   0.37382629693302,   0.00000000000000,   0.00312006493276, -0.00578410451516,   0.00570606128540 },
			{ 0.00000000000000,   1.06223330086669,   0.00000000000000,   0.00311748242960,   0.00000000000000,   0.24420385558544,   0.24970286555981,   0.00305759215246, -0.66644096559686,   0.00228253380852 },
			{ 0.00000000000000,   0.05362286897910,   0.00528925153464, -0.00842588023014,   0.00128498153337, -0.00389810210572,   0.00000000000000, -0.00223677867576, -0.03365036368035, -0.00415647085627 },
			{ 0.00000000000000,   0.00000000000000,   0.00066049870832,   0.00012563800445, -0.00085124094833,   0.04209529937135,   0.04102481443654, -0.00067596644891,   0.00017482449876, -0.00041025776053 },
		};

		param.mot_frc_vec =
		{
			{ 9.34994758321915, 7.80825641041495, 0.00000000000000 },
			{ 11.64080253106441, 13.26518528472506, 3.55567932576820 },
			{ 4.77014054273075, 7.85644357492508, 0.34445460269183 },
			{ 3.63141668516122, 3.35461524886318, 0.14824771620542 },
			{ 2.58310846982020, 1.41963212641879, 0.04855267273770 },
			{ 1.78373986219597, 0.31920640440152, 0.03381545544099 },
		};

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

    auto createControllerDaye()->std::unique_ptr<aris::control::Controller>	/*函数返回的是一个类指针，指针指向Controller,controller的类型是智能指针std::unique_ptr*/
    {
        std::unique_ptr<aris::control::Controller> controller(aris::robot::createControllerRokaeXB4());/*创建std::unique_ptr实例*/
        controller->slavePool().clear();	//清除slavePool中的元素，后面重新添加
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
                0.0345045068966465,   0.151295566371175,   -0.181133422007823,   0.00569660673541914,   0.0119907348546894,   0.0908806917782888
            };
#endif
            double pos_factor[6]
            {
                131072.0 * 129.6 / 2 / PI, -131072.0 * 100 / 2 / PI, 131072.0 * 101 / 2 / PI, 131072.0 * 81.6 / 2 / PI, 131072.0 * 81 / 2 / PI, 131072.0 * 51 / 2 / PI
            };
            double max_pos[6]
            {
                170.0 / 360 * 2 * PI, 40.0 / 360 * 2 * PI,	150.0 / 360 * 2 * PI, 180.0 / 360 * 2 * PI, 125.0 / 360 * 2 * PI, 360.0 / 360 * 2 * PI
            };
            double min_pos[6]
            {
                -170.0 / 360 * 2 * PI, -165.0 / 360 * 2 * PI, -125.0 / 360 * 2 * PI, -180.0 / 360 * 2 * PI, -125.0 / 360 * 2 * PI, -360.0 / 360 * 2 * PI
            };
            double max_vel[6]
            {
                230.0 / 360 * 2 * PI, 300.0 / 360 * 2 * PI, 300.0 / 360 * 2 * PI, 300.0 / 360 * 2 * PI, 375.0 / 360 * 2 * PI, 600.0 / 360 * 2 * PI
            };
            double max_acc[6]
            {
                1150.0 / 360 * 2 * PI, 1500.0 / 360 * 2 * PI, 1500.0 / 360 * 2 * PI, 1500.0 / 360 * 2 * PI, 1875.0 / 360 * 2 * PI, 3000.0 / 360 * 2 * PI
            };

            std::string xml_str =
                "<EthercatMotor phy_id=\"" + std::to_string(i) + "\" product_code=\"0x01\""
                " vendor_id=\"0x00000748\" revision_num=\"0x0002\" dc_assign_activate=\"0x0300\""
                " min_pos=\"" + std::to_string(min_pos[i]) + "\" max_pos=\"" + std::to_string(max_pos[i]) + "\" max_vel=\"" + std::to_string(max_vel[i]) + "\" min_vel=\"" + std::to_string(-max_vel[i]) + "\""
                " max_acc=\"" + std::to_string(max_acc[i]) + "\" min_acc=\"" + std::to_string(-max_acc[i]) + "\" max_pos_following_error=\"0.1\" max_vel_following_error=\"0.5\""
                " home_pos=\"0\" pos_factor=\"" + std::to_string(pos_factor[i]) + "\" pos_offset=\"" + std::to_string(pos_offset[i]) + "\">"
                "	<SyncManagerPoolObject>"
                "		<SyncManager is_tx=\"false\"/>"
                "		<SyncManager is_tx=\"true\"/>"
                "		<SyncManager is_tx=\"false\">"
                "			<Pdo index=\"0x1600\" is_tx=\"false\">"
                "				<PdoEntry name=\"control_word\" index=\"0x6040\" subindex=\"0x00\" size=\"16\"/>"
                "				<PdoEntry name=\"target_pos\" index=\"0x607A\" subindex=\"0x00\" size=\"32\"/>"
                "				<PdoEntry name=\"target_vel\" index=\"0x60FF\" subindex=\"0x00\" size=\"32\"/>"
                "				<PdoEntry name=\"target_tor\" index=\"0x6071\" subindex=\"0x00\" size=\"16\"/>"
                "				<PdoEntry name=\"mode_of_operation\" index=\"0x6060\" subindex=\"0x00\" size=\"8\"/>"
                "				<PdoEntry name=\"dummy_byte\" index=\"0x5FFE\" subindex=\"0x00\" size=\"8\"/>"
                "				<PdoEntry name=\"touch_probe_function\" index=\"0x60B8\" subindex=\"0x00\" size=\"16\"/>"
                "				<PdoEntry name=\"pos_offset\" index=\"0x60B0\" subindex=\"0x00\" size=\"32\"/>"
                "				<PdoEntry name=\"offset_vel\" index=\"0x60B1\" subindex=\"0x00\" size=\"32\"/>"
                "				<PdoEntry name=\"tor_offset\" index=\"0x60B2\" subindex=\"0x00\" size=\"16\"/>"
                "			</Pdo>"
                "		</SyncManager>"
                "		<SyncManager is_tx=\"true\">"
                "			<Pdo index=\"0x1A00\" is_tx=\"true\">"
                "				<PdoEntry name=\"status_word\" index=\"0x6041\" subindex=\"0x00\" size=\"16\"/>"
                "				<PdoEntry name=\"pos_actual_value\" index=\"0x6064\" subindex=\"0x00\" size=\"32\"/>"
                "				<PdoEntry name=\"vel_actual_value\" index=\"0x606c\" subindex=\"0x00\" size=\"32\"/>"
                "				<PdoEntry name=\"cur_actual_value\" index=\"0x6077\" subindex=\"0x00\" size=\"16\"/>"
                "				<PdoEntry name=\"mode_of_display\" index=\"0x6061\" subindex=\"0x00\" size=\"8\"/>"
                "				<PdoEntry name=\"dummy_byte\" index=\"0x5FFF\" subindex=\"0x00\" size=\"8\"/>"
                "				<PdoEntry name=\"following_error\" index=\"0x60F4\" subindex=\"0x00\" size=\"32\"/>"
                "				<PdoEntry name=\"digital_inputs\" index=\"0x60FD\" subindex=\"0x00\" size=\"32\"/>"
                "				<PdoEntry name=\"digital_inputs\" index=\"0x60B9\" subindex=\"0x00\" size=\"16\"/>"
                "				<PdoEntry name=\"digital_inputs\" index=\"0x60BA\" subindex=\"0x00\" size=\"32\"/>"
                "			</Pdo>"
                "		</SyncManager>"
                "	</SyncManagerPoolObject>"
                "</EthercatMotor>";

            controller->slavePool().add<aris::control::EthercatMotor>().loadXmlStr(xml_str);
        }


        //ATI force sensor//
        std::string xml_str =
            "<EthercatSlave phy_id=\"6\" product_code=\"0x26483053\""
            " vendor_id=\"0x00000732\" revision_num=\"0x00000001\" dc_assign_activate=\"0x00\">"
            "	<SyncManagerPoolObject>"
            "		<SyncManager is_tx=\"false\"/>"
            "		<SyncManager is_tx=\"true\"/>"
            "		<SyncManager is_tx=\"false\">"
            "			<Pdo index=\"0x1601\" is_tx=\"false\">"
            "				<PdoEntry name=\"Control_1\" index=\"0x7010\" subindex=\"0x01\" size=\"32\"/>"
            "				<PdoEntry name=\"Control_2\" index=\"0x7010\" subindex=\"0x02\" size=\"32\"/>"
            "			</Pdo>"
            "		</SyncManager>"
            "		<SyncManager is_tx=\"true\">"
            "			<Pdo index=\"0x1A00\" is_tx=\"true\">"
            "				<PdoEntry name=\"Int_Input_Fx\" index=\"0x6000\" subindex=\"0x01\" size=\"32\"/>"
            "				<PdoEntry name=\"Int_Input_Fy\" index=\"0x6000\" subindex=\"0x02\" size=\"32\"/>"
            "				<PdoEntry name=\"Int_Input_Fz\" index=\"0x6000\" subindex=\"0x03\" size=\"32\"/>"
            "				<PdoEntry name=\"Int_Input_Mx\" index=\"0x6000\" subindex=\"0x04\" size=\"32\"/>"
            "				<PdoEntry name=\"Int_Input_My\" index=\"0x6000\" subindex=\"0x05\" size=\"32\"/>"
            "				<PdoEntry name=\"Int_Input_Mz\" index=\"0x6000\" subindex=\"0x06\" size=\"32\"/>"
            "				<PdoEntry name=\"Status_Code\" index=\"0x6010\" subindex=\"0x00\" size=\"32\"/>"
            "				<PdoEntry name=\"Sample_Counter\" index=\"0x6020\" subindex=\"0x00\" size=\"32\"/>"
            "			</Pdo>"
            "		</SyncManager>"
            "	</SyncManagerPoolObject>"
            "</EthercatSlave>";

        controller->slavePool().add<aris::control::EthercatSlave>().loadXmlStr(xml_str);

        return controller;
    }
    auto createModelDaye(const double *robot_pm)->std::unique_ptr<aris::dynamic::Model>
    {
		aris::dynamic::PumaParam param;
		param.d1 = 0.332;
		param.a1 = 0.088;
		param.a2 = 0.46;
		param.d3 = 0.0;
		param.a3 = 0.04;
		param.d4 = 0.43;

        param.tool0_pe[2] = 0.106+0.07;

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
	
	auto createControllerSanXiang()->std::unique_ptr<aris::control::Controller>	/*函数返回的是一个类指针，指针指向Controller,controller的类型是智能指针std::unique_ptr*/
	{
		std::unique_ptr<aris::control::Controller> controller(new aris::control::EthercatController);/*创建std::unique_ptr实例*/

        for (aris::Size i = 0; i < 7; ++i)
		{

#ifdef WIN32
            double pos_offset[7]
			{
                0,   0,	  0,   0,   0,   0,   0
			};
#endif
#ifdef UNIX
            double pos_offset[7]
			{
                0.0,0.0,0.0,0.0,0.0,0.0,0.0
			};
#endif
            double pos_factor[7]
			{
                -262144.0 * 120 / 2 / PI, -262144.0 * 120 / 2 / PI, -262144.0 * 120 / 2 / PI, -262144.0 * 100 / 2 / PI, -262144.0 * 100 / 2 / PI, -262144.0 * 100 / 2 / PI, -262144.0 * 100 / 2 / PI
			};
            double max_pos[7]
			{
                360.0 / 360 * 2 * PI, 100.0 / 360 * 2 * PI,	360.0 / 360 * 2 * PI, 100.0 / 360 * 2 * PI, 360.0 / 360 * 2 * PI, 100.0 / 360 * 2 * PI, 360.0 / 360 * 2 * PI
			};
            double min_pos[7]
			{
                -360.0 / 360 * 2 * PI, -100.0 / 360 * 2 * PI, -360.0 / 360 * 2 * PI, -100.0 / 360 * 2 * PI, -360.0 / 360 * 2 * PI, -100.0 / 360 * 2 * PI, -360.0 / 360 * 2 * PI
			};
            double max_vel[7]
			{
                150.0 / 360 * 2 * PI, 150.0 / 360 * 2 * PI, 150.0 / 360 * 2 * PI, 180.0 / 360 * 2 * PI, 180.0 / 360 * 2 * PI, 180.0 / 360 * 2 * PI, 180.0 / 360 * 2 * PI
			};
            double max_acc[7]
			{
                450.0 / 360 * 2 * PI, 450.0 / 360 * 2 * PI, 450.0 / 360 * 2 * PI, 540.0 / 360 * 2 * PI, 540.0 / 360 * 2 * PI, 540.0 / 360 * 2 * PI, 540.0 / 360 * 2 * PI
			};

			std::string xml_str =
				"<EthercatMotor phy_id=\"" + std::to_string(i) + "\" product_code=\"0x201\""
				" vendor_id=\"0x000022D2\" revision_num=\"0x0a000002\" dc_assign_activate=\"0x0300\""
				" min_pos=\"" + std::to_string(min_pos[i]) + "\" max_pos=\"" + std::to_string(max_pos[i]) + "\" max_vel=\"" + std::to_string(max_vel[i]) + "\" min_vel=\"" + std::to_string(-max_vel[i]) + "\""
				" max_acc=\"" + std::to_string(max_acc[i]) + "\" min_acc=\"" + std::to_string(-max_acc[i]) + "\" max_pos_following_error=\"0.1\" max_vel_following_error=\"0.5\""
				" home_pos=\"0\" pos_factor=\"" + std::to_string(pos_factor[i]) + "\" pos_offset=\"" + std::to_string(pos_offset[i]) + "\">"
				"	<SyncManagerPoolObject>"
				"		<SyncManager is_tx=\"false\"/>"
				"		<SyncManager is_tx=\"true\"/>"
				"		<SyncManager is_tx=\"false\">"
				"			<Pdo index=\"0x1600\" is_tx=\"false\">"
				"				<PdoEntry name=\"control_word\" index=\"0x6040\" subindex=\"0x00\" size=\"16\"/>"
				"				<PdoEntry name=\"mode_of_operation\" index=\"0x6060\" subindex=\"0x00\" size=\"8\"/>"
				"				<PdoEntry name=\"target_pos\" index=\"0x607A\" subindex=\"0x00\" size=\"32\"/>"
				"				<PdoEntry name=\"target_vel\" index=\"0x60FF\" subindex=\"0x00\" size=\"32\"/>"
				"				<PdoEntry name=\"targer_tor\" index=\"0x6071\" subindex=\"0x00\" size=\"16\"/>"
				"				<PdoEntry name=\"offset_vel\" index=\"0x60B1\" subindex=\"0x00\" size=\"32\"/>"
				"			</Pdo>"
				"		</SyncManager>"
				"		<SyncManager is_tx=\"true\">"
				"			<Pdo index=\"0x1A00\" is_tx=\"true\">"
				"				<PdoEntry name=\"status_word\" index=\"0x6041\" subindex=\"0x00\" size=\"16\"/>"
				"				<PdoEntry name=\"mode_of_display\" index=\"0x6061\" subindex=\"0x00\" size=\"8\"/>"
				"				<PdoEntry name=\"pos_actual_value\" index=\"0x6064\" subindex=\"0x00\" size=\"32\"/>"
				"				<PdoEntry name=\"vel_actual_value\" index=\"0x606c\" subindex=\"0x00\" size=\"32\"/>"
				"				<PdoEntry name=\"cur_actual_value\" index=\"0x6077\" subindex=\"0x00\" size=\"16\"/>"
				"			</Pdo>"
				"		</SyncManager>"
				"	</SyncManagerPoolObject>"
				"</EthercatMotor>";
            controller->slavePool().add<aris::control::EthercatMotor>().loadXmlStr(xml_str);
		}
        dynamic_cast<aris::control::EthercatController*>(controller.get())->scanInfoForCurrentSlaves();
        dynamic_cast<aris::control::EthercatController*>(controller.get())->scanPdoForCurrentSlaves();

		return controller;
	};
	auto createModelSanXiang()->std::unique_ptr<aris::dynamic::Model>
	{
		aris::dynamic::SevenAxisParam param;

		param.d1 = 0.3705;
		param.d3 = 0.330;
		param.d5 = 0.320;
		param.tool0_pe[2] = 0.2205;

		auto m = aris::dynamic::createModelSevenAxis(param);

        //double pe[6]{ 0.45 , 0.12 , 0.32 , 0 , 0 , 0 };
        //m->generalMotionPool()[0].setMpe(pe, "321");

		return std::move(m);
	}

	auto createControllerQifan()->std::unique_ptr<aris::control::Controller>
	{
		std::unique_ptr<aris::control::Controller> controller(new aris::control::EthercatController);

		for (aris::Size i = 0; i < 6; ++i)
		{
#ifdef WIN32
			double pos_offset[6]
			{
				0.0,   0.0,   0.0,   0.0,   0.0,   0.0
			};
#endif // WIN32
#ifdef UNIX
			double pos_offset[6]
			{
                -0.438460099997905,   1.01949192131931,   -1.00835441988747,   0.0315385382644258,   0.0943992339950059,   3.4310965015334
			};
#endif
			double pos_factor[6]
			{
                8388608.0 * 166 / 2 / PI, -8388608.0 * 166 / 2 / PI, 8388608.0 * 88 / 2 / PI, 8388608.0 * 80 / 2 / PI, 8388608.0 * 80 / 2 / PI, -8388608.0 * 80 / 2 / PI
			};
			double max_pos[6]
			{
				145.0 / 360 * 2 * PI, 110.0 / 360 * 2 * PI,	55.0 / 360 * 2 * PI, 122.0 / 360 * 2 * PI, 135.0 / 360 * 2 * PI, 360.0 / 360 * 2 * PI,
			};
			double min_pos[6]
			{
				-145.0 / 360 * 2 * PI, -60.0 / 360 * 2 * PI, -55.0 / 360 * 2 * PI, -122.0 / 360 * 2 * PI, -135.0 / 360 * 2 * PI, -360.0 / 360 * 2 * PI
			};
			double max_vel[6]
			{
				73.0 / 360 * 2 * PI, 73.0 / 360 * 2 * PI, 136.0 / 360 * 2 * PI, 225.0 / 360 * 2 * PI, 225.0 / 360 * 2 * PI, 225.0 / 360 * 2 * PI,
			};
			double max_acc[6]
			{
				350.0 / 360 * 2 * PI, 350.0 / 360 * 2 * PI, 700.0 / 360 * 2 * PI, 1100.0 / 360 * 2 * PI, 1100.0 / 360 * 2 * PI, 1100.0 / 360 * 2 * PI,
			};

			if (i == 0)
			{
				std::string xml_str =
					"<EthercatMotor phy_id=\"" + std::to_string(i) + "\" product_code=\"0x60380008\""
					" vendor_id=\"0x0000066F\" revision_num=\"0x00010000\" dc_assign_activate=\"0x0300\""
					" min_pos=\"" + std::to_string(min_pos[i]) + "\" max_pos=\"" + std::to_string(max_pos[i]) + "\" max_vel=\"" + std::to_string(max_vel[i]) + "\" min_vel=\"" + std::to_string(-max_vel[i]) + "\""
					" max_acc=\"" + std::to_string(max_acc[i]) + "\" min_acc=\"" + std::to_string(-max_acc[i]) + "\" max_pos_following_error=\"0.1\" max_vel_following_error=\"0.5\""
					" home_pos=\"0\" pos_factor=\"" + std::to_string(pos_factor[i]) + "\" pos_offset=\"" + std::to_string(pos_offset[i]) + "\">"
					"	<SyncManagerPoolObject>"
					"		<SyncManager is_tx=\"false\"/>"
					"		<SyncManager is_tx=\"true\"/>"
					"		<SyncManager is_tx=\"false\">"
					"			<Pdo index=\"0x1600\" is_tx=\"false\">"
					"				<PdoEntry name=\"control_word\" index=\"0x6040\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"mode_of_operation\" index=\"0x6060\" subindex=\"0x00\" size=\"8\"/>"
					"				<PdoEntry name=\"target_pos\" index=\"0x607A\" subindex=\"0x00\" size=\"32\"/>"
					"				<PdoEntry name=\"touch_probe\" index=\"0x60B8\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"targer_tor\" index=\"0x6071\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"offset_tor\" index=\"0x60B2\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"offset_vel\" index=\"0x60B1\" subindex=\"0x00\" size=\"32\"/>"
					"			</Pdo>"
					"		</SyncManager>"
					"		<SyncManager is_tx=\"true\">"
					"			<Pdo index=\"0x1a01\" is_tx=\"true\">"
					"				<PdoEntry name=\"error_code\" index=\"0x603F\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"status_word\" index=\"0x6041\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"mode_of_display\" index=\"0x6061\" subindex=\"0x00\" size=\"8\"/>"
					"				<PdoEntry name=\"pos_actual_value\" index=\"0x6064\" subindex=\"0x00\" size=\"32\"/>"
					"				<PdoEntry name=\"vel_actual_value\" index=\"0x606C\" subindex=\"0x00\" size=\"32\"/>"
					"				<PdoEntry name=\"cur_actual_value\" index=\"0x6077\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"touch_probe_status\" index=\"0x60b9\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"touch_probe_pos1\" index=\"0x60ba\" subindex=\"0x00\" size=\"32\"/>"
					"				<PdoEntry name=\"digital_input\" index=\"0x60fd\" subindex=\"0x00\" size=\"32\"/>"
					"			</Pdo>"
					"		</SyncManager>"
					"	</SyncManagerPoolObject>"
					"</EthercatMotor>";

				controller->slavePool().add<aris::control::EthercatMotor>().loadXmlStr(xml_str);
			}
			else if (i == 1)
			{
				std::string xml_str =
					"<EthercatMotor phy_id=\"" + std::to_string(i) + "\" product_code=\"0x60380009\""
					" vendor_id=\"0x0000066F\" revision_num=\"0x00010000\" dc_assign_activate=\"0x0300\""
					" min_pos=\"" + std::to_string(min_pos[i]) + "\" max_pos=\"" + std::to_string(max_pos[i]) + "\" max_vel=\"" + std::to_string(max_vel[i]) + "\" min_vel=\"" + std::to_string(-max_vel[i]) + "\""
					" max_acc=\"" + std::to_string(max_acc[i]) + "\" min_acc=\"" + std::to_string(-max_acc[i]) + "\" max_pos_following_error=\"0.1\" max_vel_following_error=\"0.5\""
					" home_pos=\"0\" pos_factor=\"" + std::to_string(pos_factor[i]) + "\" pos_offset=\"" + std::to_string(pos_offset[i]) + "\">"
					"	<SyncManagerPoolObject>"
					"		<SyncManager is_tx=\"false\"/>"
					"		<SyncManager is_tx=\"true\"/>"
					"		<SyncManager is_tx=\"false\">"
					"			<Pdo index=\"0x1600\" is_tx=\"false\">"
					"				<PdoEntry name=\"control_word\" index=\"0x6040\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"mode_of_operation\" index=\"0x6060\" subindex=\"0x00\" size=\"8\"/>"
					"				<PdoEntry name=\"target_pos\" index=\"0x607A\" subindex=\"0x00\" size=\"32\"/>"
					"				<PdoEntry name=\"touch_probe\" index=\"0x60B8\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"targer_tor\" index=\"0x6071\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"offset_tor\" index=\"0x60B2\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"offset_vel\" index=\"0x60B1\" subindex=\"0x00\" size=\"32\"/>"
					"			</Pdo>"
					"		</SyncManager>"
					"		<SyncManager is_tx=\"true\">"
					"			<Pdo index=\"0x1a01\" is_tx=\"true\">"
					"				<PdoEntry name=\"error_code\" index=\"0x603F\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"status_word\" index=\"0x6041\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"mode_of_display\" index=\"0x6061\" subindex=\"0x00\" size=\"8\"/>"
					"				<PdoEntry name=\"pos_actual_value\" index=\"0x6064\" subindex=\"0x00\" size=\"32\"/>"
					"				<PdoEntry name=\"vel_actual_value\" index=\"0x606C\" subindex=\"0x00\" size=\"32\"/>"
					"				<PdoEntry name=\"cur_actual_value\" index=\"0x6077\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"touch_probe_status\" index=\"0x60b9\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"touch_probe_pos1\" index=\"0x60ba\" subindex=\"0x00\" size=\"32\"/>"
					"				<PdoEntry name=\"digital_input\" index=\"0x60fd\" subindex=\"0x00\" size=\"32\"/>"
					"			</Pdo>"
					"		</SyncManager>"
					"	</SyncManagerPoolObject>"
					"</EthercatMotor>";

				controller->slavePool().add<aris::control::EthercatMotor>().loadXmlStr(xml_str);
			}
			else if (i == 2)
			{
				std::string xml_str =
					"<EthercatMotor phy_id=\"" + std::to_string(i) + "\" product_code=\"0x60380008\""
					" vendor_id=\"0x0000066F\" revision_num=\"0x00010000\" dc_assign_activate=\"0x0300\""
					" min_pos=\"" + std::to_string(min_pos[i]) + "\" max_pos=\"" + std::to_string(max_pos[i]) + "\" max_vel=\"" + std::to_string(max_vel[i]) + "\" min_vel=\"" + std::to_string(-max_vel[i]) + "\""
					" max_acc=\"" + std::to_string(max_acc[i]) + "\" min_acc=\"" + std::to_string(-max_acc[i]) + "\" max_pos_following_error=\"0.1\" max_vel_following_error=\"0.5\""
					" home_pos=\"0\" pos_factor=\"" + std::to_string(pos_factor[i]) + "\" pos_offset=\"" + std::to_string(pos_offset[i]) + "\">"
					"	<SyncManagerPoolObject>"
					"		<SyncManager is_tx=\"false\"/>"
					"		<SyncManager is_tx=\"true\"/>"
					"		<SyncManager is_tx=\"false\">"
					"			<Pdo index=\"0x1600\" is_tx=\"false\">"
					"				<PdoEntry name=\"control_word\" index=\"0x6040\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"mode_of_operation\" index=\"0x6060\" subindex=\"0x00\" size=\"8\"/>"
					"				<PdoEntry name=\"target_pos\" index=\"0x607A\" subindex=\"0x00\" size=\"32\"/>"
					"				<PdoEntry name=\"touch_probe\" index=\"0x60B8\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"targer_tor\" index=\"0x6071\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"offset_tor\" index=\"0x60B2\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"offset_vel\" index=\"0x60B1\" subindex=\"0x00\" size=\"32\"/>"
					"			</Pdo>"
					"		</SyncManager>"
					"		<SyncManager is_tx=\"true\">"
					"			<Pdo index=\"0x1a01\" is_tx=\"true\">"
					"				<PdoEntry name=\"error_code\" index=\"0x603F\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"status_word\" index=\"0x6041\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"mode_of_display\" index=\"0x6061\" subindex=\"0x00\" size=\"8\"/>"
					"				<PdoEntry name=\"pos_actual_value\" index=\"0x6064\" subindex=\"0x00\" size=\"32\"/>"
					"				<PdoEntry name=\"vel_actual_value\" index=\"0x606C\" subindex=\"0x00\" size=\"32\"/>"
					"				<PdoEntry name=\"cur_actual_value\" index=\"0x6077\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"touch_probe_status\" index=\"0x60b9\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"touch_probe_pos1\" index=\"0x60ba\" subindex=\"0x00\" size=\"32\"/>"
					"				<PdoEntry name=\"digital_input\" index=\"0x60fd\" subindex=\"0x00\" size=\"32\"/>"
					"			</Pdo>"
					"		</SyncManager>"
					"	</SyncManagerPoolObject>"
					"</EthercatMotor>";

				controller->slavePool().add<aris::control::EthercatMotor>().loadXmlStr(xml_str);
			}
			else
			{
				std::string xml_str =
					"<EthercatMotor phy_id=\"" + std::to_string(i) + "\" product_code=\"0x60380006\""
					" vendor_id=\"0x0000066F\" revision_num=\"0x00010000\" dc_assign_activate=\"0x0300\""
					" min_pos=\"" + std::to_string(min_pos[i]) + "\" max_pos=\"" + std::to_string(max_pos[i]) + "\" max_vel=\"" + std::to_string(max_vel[i]) + "\" min_vel=\"" + std::to_string(-max_vel[i]) + "\""
					" max_acc=\"" + std::to_string(max_acc[i]) + "\" min_acc=\"" + std::to_string(-max_acc[i]) + "\" max_pos_following_error=\"0.1\" max_vel_following_error=\"0.5\""
					" home_pos=\"0\" pos_factor=\"" + std::to_string(pos_factor[i]) + "\" pos_offset=\"" + std::to_string(pos_offset[i]) + "\">"
					"	<SyncManagerPoolObject>"
					"		<SyncManager is_tx=\"false\"/>"
					"		<SyncManager is_tx=\"true\"/>"
					"		<SyncManager is_tx=\"false\">"
					"			<Pdo index=\"0x1600\" is_tx=\"false\">"
					"				<PdoEntry name=\"control_word\" index=\"0x6040\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"mode_of_operation\" index=\"0x6060\" subindex=\"0x00\" size=\"8\"/>"
					"				<PdoEntry name=\"target_pos\" index=\"0x607A\" subindex=\"0x00\" size=\"32\"/>"
					"				<PdoEntry name=\"touch_probe\" index=\"0x60B8\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"targer_tor\" index=\"0x6071\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"offset_tor\" index=\"0x60B2\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"offset_vel\" index=\"0x60B1\" subindex=\"0x00\" size=\"32\"/>"
					"			</Pdo>"
					"		</SyncManager>"
					"		<SyncManager is_tx=\"true\">"
					"			<Pdo index=\"0x1a01\" is_tx=\"true\">"
					"				<PdoEntry name=\"error_code\" index=\"0x603F\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"status_word\" index=\"0x6041\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"mode_of_display\" index=\"0x6061\" subindex=\"0x00\" size=\"8\"/>"
					"				<PdoEntry name=\"pos_actual_value\" index=\"0x6064\" subindex=\"0x00\" size=\"32\"/>"
					"				<PdoEntry name=\"vel_actual_value\" index=\"0x606C\" subindex=\"0x00\" size=\"32\"/>"
					"				<PdoEntry name=\"cur_actual_value\" index=\"0x6077\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"touch_probe_status\" index=\"0x60b9\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"touch_probe_pos1\" index=\"0x60ba\" subindex=\"0x00\" size=\"32\"/>"
					"				<PdoEntry name=\"digital_input\" index=\"0x60fd\" subindex=\"0x00\" size=\"32\"/>"
					"			</Pdo>"
					"		</SyncManager>"
					"	</SyncManagerPoolObject>"
					"</EthercatMotor>";

				controller->slavePool().add<aris::control::EthercatMotor>().loadXmlStr(xml_str);
			}
		}
		return controller;
	};
	auto createModelQifan()->std::unique_ptr<aris::dynamic::Model>
	{
		aris::dynamic::PumaParam param;
		
		
		param.d1 = 0.596;
		param.a1 = 0.220;
		param.a2 = 1.020;
		param.d3 = 0.0;
		param.a3 = 0.0;
		param.d4 = 0.86;
		//param.d4 = 1.010;

		param.tool0_pe[2] = 0.153;

		auto model = aris::dynamic::createModelPuma(param);

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
		//auto &rs = plan_root->planPool().add<kaanh::Reset>();
		//rs.command().findParam("pos")->setDefaultValue("{0.5,0.3925,0.7899,0.5,0.5,0.5}");

		plan_root->planPool().add<aris::server::GetInfo>();
		//qifan//
		//rs.command().findParam("pos")->setDefaultValue("{0.5,0.353,0.5,0.5,0.5,0.5}");

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
        plan_root->planPool().add<kaanh::Yuli>();
        plan_root->planPool().add<kaanh::MoveJoint>();
        plan_root->planPool().add<kaanh::FCStop>();
		plan_root->planPool().add<kaanh::SetFS>();
	
		plan_root->planPool().add<CalibT4P>();
		plan_root->planPool().add<CalibT5P>();
		plan_root->planPool().add<CalibT6P>();
		plan_root->planPool().add<SetTF>();
		plan_root->planPool().add<CalibZF>();
		plan_root->planPool().add<CalibZO>();
		plan_root->planPool().add<CalibZL>();
		plan_root->planPool().add<SwitchTool>();
		
		return plan_root;
	}
}
