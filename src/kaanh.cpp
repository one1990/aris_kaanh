#include <algorithm>
#include "kaanh.h"
#include "sixdistalfc.h"
#include "jointfc.h"
#include "sevenjointfc.h"
#include <array>
#include <stdlib.h>
#include"move_series.h"


using namespace aris::dynamic;
using namespace aris::plan;

extern double fce_data[buffer_length];
extern int data_num, data_num_send;
extern std::atomic_int which_di;
extern std::atomic_bool is_automatic;

namespace kaanh
{
	aris::dynamic::Marker tool1;
	auto createInterface()->std::unique_ptr<aris::server::InterfaceRoot>
	{
		std::unique_ptr<aris::server::InterfaceRoot> interfaceroot;/*创建std::unique_ptr实例*/
	
		auto uixmlpath = std::filesystem::absolute(".");//获取当前工程所在的路径
		const std::string uixmlfile = "interface_kaanh.xml";
		uixmlpath = uixmlpath / uixmlfile;
		aris::core::XmlDocument ui_doc;
		ui_doc.LoadFile(uixmlpath.string().c_str());

		interfaceroot->loadXmlDoc(ui_doc);

		return interfaceroot;
	};

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
			"				<PdoEntry name=\"cur_actual_value\" index=\"0x6078\" subindex=\"0x00\" size=\"16\"/>"
			"			</Pdo>"
			"		</SyncManager>"
			"	</SyncManagerPoolObject>"
			"</EthercatMotion>";
		controller->slavePool().add<aris::control::EthercatMotion>().loadXmlStr(xml_str);
		return controller;
	};

	auto createControllerRokaeXB4()->std::unique_ptr<aris::control::Controller>	/*函数返回的是一个类指针，指针指向Controller,controller的类型是智能指针std::unique_ptr*/
	{
		std::unique_ptr<aris::control::Controller> controller(aris::robot::createControllerRokaeXB4());/*创建std::unique_ptr实例*/

#ifdef UNIX

        dynamic_cast<aris::control::Motion&>(controller->slavePool()[0]).setPosOffset(0.00293480352126769);
        dynamic_cast<aris::control::Motion&>(controller->slavePool()[1]).setPosOffset(-2.50023777179214);
        dynamic_cast<aris::control::Motion&>(controller->slavePool()[2]).setPosOffset(-0.292382537944081);
        dynamic_cast<aris::control::Motion&>(controller->slavePool()[3]).setPosOffset(0.0582675097338009);
        dynamic_cast<aris::control::Motion&>(controller->slavePool()[4]).setPosOffset(1.53363576057128);
        dynamic_cast<aris::control::Motion&>(controller->slavePool()[5]).setPosOffset(25.924544999999998);
		
        for(int i=0; i<6; i++)
        {
             dynamic_cast<aris::control::EthercatMotion&>(controller->slavePool()[i]).setDcAssignActivate(0x300);
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
		model->calculator().addVariable("PI", aris::core::Matrix(PI));

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
                "<EthercatMotion phy_id=\"" + std::to_string(i) + "\" product_code=\"0x01\""
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
                "</EthercatMotion>";

            controller->slavePool().add<aris::control::EthercatMotion>().loadXmlStr(xml_str);
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
				"<EthercatMotion phy_id=\"" + std::to_string(i) + "\" product_code=\"0x201\""
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
				"</EthercatMotion>";
            controller->slavePool().add<aris::control::EthercatMotion>().loadXmlStr(xml_str);
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


	// 获取末端位姿pq,pe,关节角度j //
	auto Get::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		std::any pq_pe_j_vec;
		for (auto cmd_param : params)
		{
			if (cmd_param.first == "all")
			{
				pq_pe_j_vec = std::make_any<std::vector<double> >(13 + 3 * target.model->motionPool().size());

				target.server->getRtData([](aris::server::ControlServer& cs, std::any& data)
				{
					cs.model().generalMotionPool().at(0).getMpq(std::any_cast<std::vector<double>&>(data).data());
					cs.model().generalMotionPool().at(0).getMpe(std::any_cast<std::vector<double>&>(data).data() + 7);
					for (aris::Size i = 13; i < 13 + cs.model().motionPool().size(); i++)
					{
						std::any_cast<std::vector<double>&>(data)[i] = cs.model().motionPool()[i - 13].mp();
					}
					for (aris::Size i = 13 + cs.model().motionPool().size(); i < 13 + 2 * cs.model().motionPool().size(); i++)
					{
						std::any_cast<std::vector<double>&>(data)[i] = cs.model().motionPool()[i - 13 - cs.model().motionPool().size()].mv();
					}
					for (aris::Size i = 13 + 2 * cs.model().motionPool().size(); i < 13 + 3 * cs.model().motionPool().size(); i++)
					{
						std::any_cast<std::vector<double>&>(data)[i] = cs.model().motionPool()[i - 13 - 2 * cs.model().motionPool().size()].ma();
					}

				}, pq_pe_j_vec);
				std::cout << "pq:" << " ";
				for (aris::Size i = 0; i < 7; i++)
				{
					std::cout << std::any_cast<std::vector<double>&>(pq_pe_j_vec)[i] << " ";
				}
				std::cout << std::endl;
				std::cout << "pe:" << " ";
				for (aris::Size i = 7; i < 13; i++)
				{
					std::cout << std::any_cast<std::vector<double>&>(pq_pe_j_vec)[i] << " ";
				}
				std::cout << std::endl;
				std::cout << "j:" << " ";
				for (aris::Size i = 13; i < 13 + target.model->motionPool().size(); i++)
				{
					std::cout << std::any_cast<std::vector<double>&>(pq_pe_j_vec)[i] << " ";
				}
				std::cout << std::endl;
			}
			else if (cmd_param.first == "pq")
			{
				pq_pe_j_vec = std::make_any<std::vector<double> >(7);

				target.server->getRtData([](aris::server::ControlServer& cs, std::any& data)
				{
					cs.model().generalMotionPool().at(0).getMpq(std::any_cast<std::vector<double>&>(data).data());
				}, pq_pe_j_vec);
				for (aris::Size i = 0; i < 7; i++)
				{
					std::cout << std::any_cast<std::vector<double>&>(pq_pe_j_vec)[i] << " ";
				}
				std::cout << std::endl;
			}
			else if (cmd_param.first == "pe")
			{
				pq_pe_j_vec = std::make_any<std::vector<double> >(6);

				target.server->getRtData([](aris::server::ControlServer& cs, std::any& data)
				{
					cs.model().generalMotionPool().at(0).getMpe(std::any_cast<std::vector<double>&>(data).data());
				}, pq_pe_j_vec);
				for (aris::Size i = 0; i < 6; i++)
				{
					std::cout << std::any_cast<std::vector<double>&>(pq_pe_j_vec)[i] << " ";
				}
				std::cout << std::endl;
			}
			else if (cmd_param.first == "j")
			{
				pq_pe_j_vec = std::make_any<std::vector<double> >(3*target.model->motionPool().size());
				target.server->getRtData([](aris::server::ControlServer& cs, std::any& data)
				{
					for (aris::Size i = 0; i < cs.model().motionPool().size(); i++)
					{
						std::any_cast<std::vector<double>&>(data)[i] = cs.model().motionPool()[i].mp();
					}
					for (aris::Size i = cs.model().motionPool().size(); i < 2*cs.model().motionPool().size(); i++)
					{
						std::any_cast<std::vector<double>&>(data)[i] = cs.model().motionPool()[i- cs.model().motionPool().size()].mv();
					}		
					for (aris::Size i = 2*cs.model().motionPool().size(); i < 3*cs.model().motionPool().size(); i++)
					{
						std::any_cast<std::vector<double>&>(data)[i] = cs.model().motionPool()[i- 2*cs.model().motionPool().size()].ma();
					}
					
				}, pq_pe_j_vec);

				for (aris::Size i = 0; i < target.model->motionPool().size(); i++)
				{
					std::cout << std::any_cast<std::vector<double>&>(pq_pe_j_vec)[i] << " ";
				}
				std::cout << std::endl;
			}
		}
		auto pq_pe_j = std::any_cast<std::vector<double>&>(pq_pe_j_vec);
		//取小数点后三位//
		for (aris::Size i = 0; i < pq_pe_j.size(); i++)
		{
			pq_pe_j[i] = std::floor(pq_pe_j[i]*1000.0f + 0.5)/1000.0f;
		}
		
		std::string ret(reinterpret_cast<char*>(pq_pe_j.data()), pq_pe_j.size() * sizeof(double));
		target.ret = ret;
		target.option |= NOT_RUN_EXECUTE_FUNCTION | NOT_PRINT_CMD_INFO | NOT_PRINT_CMD_INFO;
	}
	auto Get::collectNrt(PlanTarget &target)->void {}
	Get::Get(const std::string &name) : Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"get\">"
			"	<GroupParam>"
			"		<UniqueParam default=\"all\">"
			"			<Param name=\"all\" abbreviation=\"a\"/>"
			"			<Param name=\"pq\"/>"
			"			<Param name=\"pe\"/>"
			"			<Param name=\"j\"/>"
			"		</UniqueParam>"
			"	</GroupParam>"
			"</Command>");
	}


	// 末端四元数xyz方向余弦轨迹；速度前馈//
	struct MoveXParam
	{
		double x, y, z;
		double time;
	};
	auto MoveX::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
		{
			MoveXParam param ={0.0,0.0,0.0,0.0};
			for (auto &p : params)
			{
				if (p.first == "x")
				{
					param.x = std::stod(p.second);
				}
				else if (p.first == "y")
				{
					param.y = std::stod(p.second);
				}
				else if (p.first == "z")
				{
					param.z = std::stod(p.second);
				}
				else if (p.first == "time")
				{
					param.time = std::stod(p.second);
				}
			}
			target.param = param;

			std::fill(target.mot_options.begin(), target.mot_options.end(),
				//用于使用模型轨迹驱动电机//
				Plan::USE_TARGET_POS);

		}
	auto MoveX::executeRT(PlanTarget &target)->int
		{ 
			auto &ee = target.model->generalMotionPool().at(0);
			auto &param = std::any_cast<MoveXParam&>(target.param);

			auto time = static_cast<int>(param.time*1000);
			static double begin_pq[7];
			if (target.count == 1)
			{
				ee.getMpq(begin_pq);
			}
			double pq2[7];
			double pqv[7] = {0.0, 0.0 ,0.0, 0.0, 0.0 ,0.0, 0.0};
			ee.getMpq(pq2);
			pq2[0] = begin_pq[0] + param.x*(1 - std::cos(2 * PI*target.count / time)) / 2;
			pq2[1] = begin_pq[1] + param.y*(1 - std::cos(2 * PI*target.count / time)) / 2;
			pq2[2] = begin_pq[2] + param.z*(1 - std::cos(2 * PI*target.count / time)) / 2;
			ee.setMpq(pq2);
			//速度前馈//
            pqv[0] = 1000 * param.x*(PI / time)*std::sin(2 * PI*target.count / time);
            pqv[1] = 1000 * param.y*(PI / time)*std::sin(2 * PI*target.count / time);
            pqv[2] = 1000 * param.z*(PI / time)*std::sin(2 * PI*target.count / time);
            ee.setMvq(pqv);

			if (target.model->solverPool().at(0).kinPos())return -1;
            target.model->solverPool().at(0).kinVel();

			// 访问主站 //
			auto controller = target.controller;

			// 打印电流 //
			auto &cout = controller->mout();
			if (target.count % 100 == 0)
			{
				 cout <<"cur:"<< controller->motionAtAbs(0).actualCur() <<"  "<< controller->motionAtAbs(1).actualCur() << std::endl;
			}
			
			// log 电流 //
			auto &lout = controller->lout();
			lout << controller->motionAtAbs(0).actualCur() << "  " << controller->motionAtAbs(1).actualCur() << std::endl;

			return time-target.count;
		}
	auto MoveX::collectNrt(PlanTarget &target)->void {}
	MoveX::MoveX(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"moveX\">"
			"	<GroupParam>"
			"		<UniqueParam default=\"x\">"
			"			<Param name=\"x\" default=\"0.1\"/>"
			"			<Param name=\"y\" default=\"0.1\"/>"
			"			<Param name=\"z\" default=\"0.1\"/>"
			"		</UniqueParam>"
			"		<Param name=\"time\" default=\"1.0\"/>"
			"	</GroupParam>"
			"</Command>");
	}


	// 单关节正弦往复轨迹 //
	struct MoveJSParam
	{
		double j[6];
		double time;
		uint32_t timenum;
		std::vector<bool> joint_active_vec;
	};
	auto MoveJS::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
		{
			MoveJSParam param;
			for (Size i = 0; i < 6; i++)
			{
				param.j[i] = 0.0;
			}
			param.time = 0.0;
			param.timenum = 0;	

			param.joint_active_vec.clear();
			param.joint_active_vec.resize(target.model->motionPool().size(), true);

			for (auto &p : params)
			{
				if (p.first == "j1")
				{
					if (p.second == "current_pos")
					{
						param.joint_active_vec[0] = false;
						param.j[0] = target.model->motionPool()[0].mp();
					}
					else
					{
						param.joint_active_vec[0] = true;
						param.j[0] = std::stod(p.second);
					}
							
				}
				else if (p.first == "j2")
				{
					if (p.second == "current_pos")
					{
						param.joint_active_vec[1] = false;
						param.j[1] = target.model->motionPool()[1].mp();
					}
					else
					{
						param.joint_active_vec[1] = true;
						param.j[1] = std::stod(p.second);
					}
				}
				else if (p.first == "j3")
				{
					if (p.second == "current_pos")
					{
						param.joint_active_vec[2] = false;
						param.j[2] = target.model->motionPool()[2].mp();
					}
					else
					{
						param.joint_active_vec[2] = true;
						param.j[2] = std::stod(p.second);
					}
				}
				else if (p.first == "j4")
				{
					if (p.second == "current_pos")
					{
						param.joint_active_vec[3] = false;
						param.j[3] = target.model->motionPool()[3].mp();
					}
					else
					{
						param.joint_active_vec[3] = true;
						param.j[3] = std::stod(p.second);
					}
				}
				else if (p.first == "j5")
				{
					if (p.second == "current_pos")
					{
						param.joint_active_vec[4] = false;
						param.j[4] = target.model->motionPool()[4].mp();
					}
					else
					{
						param.joint_active_vec[4] = true;
						param.j[4] = std::stod(p.second);
					}
				}
				else if (p.first == "j6")
				{
					if (p.second == "current_pos")
					{
						param.joint_active_vec[5] = false;
						param.j[5] = target.model->motionPool()[5].mp();
					}
					else
					{
						param.joint_active_vec[5] = true;
						param.j[5] = std::stod(p.second);
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
			target.param = param;

			std::fill(target.mot_options.begin(), target.mot_options.end(),
				Plan::USE_TARGET_POS);

		}
	auto MoveJS::executeRT(PlanTarget &target)->int
		{
			auto &param = std::any_cast<MoveJSParam&>(target.param);
			auto time = static_cast<int32_t>(param.time * 1000);
			auto totaltime = static_cast<int32_t>(param.timenum * time);
			static double begin_pjs[6];
			static double step_pjs[6];
			
			if ((1 <= target.count) && (target.count <= time / 2))
			{
				// 获取当前起始点位置 //
				if (target.count == 1)
				{
					for (Size i = 0; i < param.joint_active_vec.size(); ++i)
					{
						begin_pjs[i] = target.model->motionPool()[i].mp();
						step_pjs[i] = target.model->motionPool()[i].mp();
					}
				}
				for (Size i = 0; i < param.joint_active_vec.size(); ++i)
				{
					step_pjs[i] = begin_pjs[i] + param.j[i] * (1 - std::cos(2 * PI*target.count / time)) / 2;
					target.model->motionPool().at(i).setMp(step_pjs[i]);
				}
			}
			else if ((time / 2 < target.count) && (target.count <= totaltime - time/2))
			{
				// 获取当前起始点位置 //
				if (target.count == time / 2+1)
				{
					for (Size i = 0; i < param.joint_active_vec.size(); ++i)
					{
						begin_pjs[i] = target.model->motionPool()[i].mp();
						step_pjs[i] = target.model->motionPool()[i].mp();
					}
				}
				for (Size i = 0; i < param.joint_active_vec.size(); ++i)
				{
					step_pjs[i] = begin_pjs[i] - 2*param.j[i] * (1 - std::cos(2 * PI*(target.count-time/2) / time)) / 2;
					target.model->motionPool().at(i).setMp(step_pjs[i]);
				}

			}
			else if ((totaltime - time / 2 < target.count) && (target.count <= totaltime))
			{
				// 获取当前起始点位置 //
				if (target.count == totaltime - time / 2 + 1)
				{
					for (Size i = 0; i < param.joint_active_vec.size(); ++i)
					{
						begin_pjs[i] = target.model->motionPool()[i].mp();
						step_pjs[i] = target.model->motionPool()[i].mp();
					}
				}
				for (Size i = 0; i < param.joint_active_vec.size(); ++i)
				{
					step_pjs[i] = begin_pjs[i] - param.j[i] * (1 - std::cos(2 * PI*(target.count - totaltime + time / 2) / time)) / 2;
					target.model->motionPool().at(i).setMp(step_pjs[i]);
				}
			}

			if (target.model->solverPool().at(1).kinPos())return -1;

			// 访问主站 //
			auto controller = target.controller;

			// 打印电流 //
			auto &cout = controller->mout();
			if (target.count % 100 == 0)
			{
				for (Size i = 0; i < 6; i++)
				{
					cout << "pos" << i+1 << ":" << controller->motionAtAbs(i).actualPos() << "  " ;
					cout << "vel" << i+1 << ":" << controller->motionAtAbs(i).actualVel() << "  ";
					cout << "cur" << i+1 << ":" << controller->motionAtAbs(i).actualCur() << "  ";
				}		
				cout << std::endl;
			}

			// log 电流 //
			auto &lout = controller->lout();
			for (Size i = 0; i < 6; i++)
			{
				lout << controller->motionAtAbs(i).targetPos() << ",";
				lout << controller->motionAtAbs(i).actualPos() << ",";
				lout << controller->motionAtAbs(i).actualVel() << ",";
				lout << controller->motionAtAbs(i).actualCur() << ",";
			}
			lout << std::endl;
			
			return totaltime - target.count;
		}
	auto MoveJS::collectNrt(PlanTarget &target)->void {}
	MoveJS::MoveJS(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"moveJS\">"
			"	<GroupParam>"
			"		<Param name=\"j1\" default=\"current_pos\"/>"
			"		<Param name=\"j2\" default=\"current_pos\"/>"
			"		<Param name=\"j3\" default=\"current_pos\"/>"
			"		<Param name=\"j4\" default=\"current_pos\"/>"
			"		<Param name=\"j5\" default=\"current_pos\"/>"
			"		<Param name=\"j6\" default=\"current_pos\"/>"
			"		<Param name=\"time\" default=\"1.0\" abbreviation=\"t\"/>"
			"		<Param name=\"timenum\" default=\"2\" abbreviation=\"n\"/>"
			"	</GroupParam>"
			"</Command>");
	}


	// 任意关节正弦往复轨迹 //
	struct MoveJSNParam
	{
		std::vector<double> axis_pos_vec;
		std::vector<double> axis_time_vec;
		std::vector<bool> joint_active_vec;
		uint32_t timenum;
	};
	auto MoveJSN::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
		{
			MoveJSNParam param;
			auto c = target.controller;
			param.axis_pos_vec.clear();
			param.axis_pos_vec.resize(target.model->motionPool().size(), 0.0);

			param.axis_time_vec.clear();
			param.axis_time_vec.resize(target.model->motionPool().size(), 1.0);

			param.joint_active_vec.clear();
			param.joint_active_vec.resize(target.model->motionPool().size(), true);

			param.timenum = 0;
			for (auto &p : params)
			{
				if (p.first == "pos")
				{
					auto pos = target.model->calculator().calculateExpression(p.second);
					if (pos.size() == 1)
					{
						param.axis_pos_vec.resize(param.axis_pos_vec.size(), pos.toDouble());
					}
					else if (pos.size() == param.axis_pos_vec.size())
					{
						param.axis_pos_vec.assign(pos.begin(), pos.end());
					}
					else
					{
						throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
					}

					for (Size i = 0; i < param.axis_pos_vec.size(); ++i)
					{
						//超阈值保护//
						if (param.axis_pos_vec[i] > 1.0)
						{
							param.axis_pos_vec[i] = 1.0;
						}
						if (param.axis_pos_vec[i] < -1.0)
						{
							param.axis_pos_vec[i] = -1.0;
						}
						if (param.axis_pos_vec[i] >= 0)
						{
							param.axis_pos_vec[i] = param.axis_pos_vec[i] * c->motionPool()[i].maxPos();
						}
						else
						{
							param.axis_pos_vec[i] = param.axis_pos_vec[i] * c->motionPool()[i].minPos();
						}
					}

				}
				else if (p.first == "time")
				{
					auto t = target.model->calculator().calculateExpression(p.second);
					if (t.size() == 1)
					{
						param.axis_time_vec.resize(param.axis_time_vec.size(), t.toDouble());
					}
					else if (t.size() == param.axis_time_vec.size())
					{
						param.axis_time_vec.assign(t.begin(), t.end());
					}
					else
					{
						throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
					}

					for (Size i = 0; i < param.axis_time_vec.size(); ++i)
					{
						//超阈值保护，机器人单关节运动频率不超过5Hz//
						if (param.axis_time_vec[i] < 0.2)
						{
							param.axis_time_vec[i] = 0.2;
						}
					}
				}
				else if (p.first == "timenum")
				{
					param.timenum = std::stoi(p.second);
				}
			}
			target.param = param;

			std::fill(target.mot_options.begin(), target.mot_options.end(),
				Plan::USE_TARGET_POS);
/*
#ifdef WIN32
				Plan::NOT_CHECK_POS_MIN |
				Plan::NOT_CHECK_POS_MAX |
				Plan::NOT_CHECK_POS_CONTINUOUS |
				Plan::NOT_CHECK_POS_CONTINUOUS_AT_START |
				Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
				Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START |
				Plan::NOT_CHECK_POS_FOLLOWING_ERROR |
#endif
				Plan::NOT_CHECK_VEL_MIN |
				Plan::NOT_CHECK_VEL_MAX |
				Plan::NOT_CHECK_VEL_CONTINUOUS |
				Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START |
				Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;
*/
		}
	auto MoveJSN::executeRT(PlanTarget &target)->int
		{
			auto &param = std::any_cast<MoveJSNParam&>(target.param);
			static int32_t time[6];
			static int32_t totaltime[6];
			static int32_t totaltime_max = 0;
			for (Size i = 0; i < 6; i++)
			{
				time[i] = static_cast<uint32_t>(param.axis_time_vec[i] * 1000);
				totaltime[i] = static_cast<uint32_t>(param.timenum * time[i]);
				if (totaltime[i] > totaltime_max)
				{
					totaltime_max = totaltime[i];
				}
			}

			static double begin_pjs[6];
			static double step_pjs[6];

			for (Size i = 0; i < param.axis_time_vec.size(); i++)
			{
				if ((1 <= target.count) && (target.count <= time[i] / 2))
				{
					// 获取当前起始点位置 //
					if (target.count == 1)
					{
						begin_pjs[i] = target.model->motionPool()[i].mp();
						step_pjs[i] = target.model->motionPool()[i].mp();
					}
					step_pjs[i] = begin_pjs[i] + param.axis_pos_vec[i] * (1 - std::cos(2 * PI*target.count / time[i])) / 2;
					target.model->motionPool().at(i).setMp(step_pjs[i]);
				}
				else if ((time[i] / 2 < target.count) && (target.count <= totaltime[i] - time[i] / 2))
				{
					// 获取当前起始点位置 //
					if (target.count == time[i] / 2 + 1)
					{
						begin_pjs[i] = target.model->motionPool()[i].mp();
						step_pjs[i] = target.model->motionPool()[i].mp();
					}
					step_pjs[i] = begin_pjs[i] - 2 * param.axis_pos_vec[i] * (1 - std::cos(2 * PI*(target.count - time[i] / 2) / time[i])) / 2;
					target.model->motionPool().at(i).setMp(step_pjs[i]);

				}
				else if ((totaltime[i] - time[i] / 2 < target.count) && (target.count <= totaltime[i]))
				{
					// 获取当前起始点位置 //
					if (target.count == totaltime[i] - time[i] / 2 + 1)
					{
						begin_pjs[i] = target.model->motionPool()[i].mp();
						step_pjs[i] = target.model->motionPool()[i].mp();
					}
					step_pjs[i] = begin_pjs[i] - param.axis_pos_vec[i] * (1 - std::cos(2 * PI*(target.count - totaltime[i] + time[i] / 2) / time[i])) / 2;
					target.model->motionPool().at(i).setMp(step_pjs[i]);
				}
			}

			if (target.model->solverPool().at(1).kinPos())return -1;

			// 访问主站 //
			auto controller = target.controller;

			// 打印电流 //
			auto &cout = controller->mout();
			if (target.count % 100 == 0)
			{
				for (Size i = 0; i < 6; i++)
				{
					cout << "pos" << i + 1 << ":" << controller->motionAtAbs(i).actualPos() << "  ";
					cout << "vel" << i + 1 << ":" << controller->motionAtAbs(i).actualVel() << "  ";
					cout << "cur" << i + 1 << ":" << controller->motionAtAbs(i).actualCur() << "  ";
				}
				cout << std::endl;
			}

			// log 电流 //
			auto &lout = controller->lout();
			for (Size i = 0; i < 6; i++)
			{
				lout << controller->motionAtAbs(i).targetPos() << ",";
				lout << controller->motionAtAbs(i).actualPos() << ",";
				lout << controller->motionAtAbs(i).actualVel() << ",";
				lout << controller->motionAtAbs(i).actualCur() << ",";
			}
			lout << std::endl;

			return totaltime_max - target.count;
		}
	auto MoveJSN::collectNrt(PlanTarget &target)->void {}
	MoveJSN::MoveJSN(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"moveJSN\">"
			"	<GroupParam>"
			"		<Param name=\"pos\" default=\"{0.1,0.2,0.2,0.2,0.2,0.2}\" abbreviation=\"p\"/>"
			"		<Param name=\"time\" default=\"{1.0,1.0,1.0,1.0,1.0,1.0}\" abbreviation=\"t\"/>"
			"		<Param name=\"timenum\" default=\"2\" abbreviation=\"n\"/>"
			"	</GroupParam>"
			"</Command>");
	}
		

	// 单关节相对运动轨迹--输入单个关节，角度位置；关节按照梯形速度轨迹执行；速度前馈//
	struct MoveJRParam
	{
		double vel, acc, dec;
		std::vector<double> joint_pos_vec, begin_joint_pos_vec;
		std::vector<bool> joint_active_vec;
	};
	auto MoveJR::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		auto c = target.controller;
		MoveJRParam param;

		for (auto cmd_param : params)
		{
			if (cmd_param.first == "all")
			{
				param.joint_active_vec.resize(target.model->motionPool().size(), true);
			}
			else if (cmd_param.first == "none")
			{
				param.joint_active_vec.resize(target.model->motionPool().size(), false);
			}
			else if (cmd_param.first == "motion_id")
			{
				param.joint_active_vec.resize(target.model->motionPool().size(), false);
				param.joint_active_vec.at(std::stoi(cmd_param.second)) = true;
			}
			else if (cmd_param.first == "physical_id")
			{
				param.joint_active_vec.resize(c->motionPool().size(), false);
				param.joint_active_vec.at(c->motionAtPhy(std::stoi(cmd_param.second)).phyId()) = true;
			}
			else if (cmd_param.first == "slave_id")
			{
				param.joint_active_vec.resize(c->motionPool().size(), false);
				param.joint_active_vec.at(c->motionAtPhy(std::stoi(cmd_param.second)).slaId()) = true;
			}
			else if (cmd_param.first == "pos")
			{
				aris::core::Matrix mat = target.model->calculator().calculateExpression(cmd_param.second);
				if (mat.size() == 1)param.joint_pos_vec.resize(c->motionPool().size(), mat.toDouble());
				else
				{
					param.joint_pos_vec.resize(mat.size());
					std::copy(mat.begin(), mat.end(), param.joint_pos_vec.begin());
				}
			}
			else if (cmd_param.first == "vel")
			{
				param.vel = std::stod(cmd_param.second);
			}
			else if (cmd_param.first == "acc")
			{
				param.acc = std::stod(cmd_param.second);
			}
			else if (cmd_param.first == "dec")
			{
				param.dec = std::stod(cmd_param.second);
			}
		}

		param.begin_joint_pos_vec.resize(target.model->motionPool().size());

		target.param = param;

		//std::fill(target.mot_options.begin(), target.mot_options.end(), Plan::USE_TARGET_POS);

	}
	auto MoveJR::executeRT(PlanTarget &target)->int
	{
		auto &param = std::any_cast<MoveJRParam&>(target.param);
		auto controller = target.controller;

		if (target.count == 1)
		{
			for (Size i = 0; i < param.joint_active_vec.size(); ++i)
			{
				if (param.joint_active_vec[i])
				{
					param.begin_joint_pos_vec[i] = controller->motionAtAbs(i).actualPos();
				}
			}
		}

		aris::Size total_count{ 1 };
		for (Size i = 0; i < param.joint_active_vec.size(); ++i)
		{
			if (param.joint_active_vec[i])
			{
				double p, v, a;
				aris::Size t_count;
				aris::plan::moveAbsolute(target.count, param.begin_joint_pos_vec[i], param.begin_joint_pos_vec[i]+param.joint_pos_vec[i], param.vel / 1000, param.acc / 1000 / 1000, param.dec / 1000 / 1000, p, v, a, t_count);
				controller->motionAtAbs(i).setTargetPos(p);
				controller->motionAtAbs(i).setTargetVel(v);
				target.model->motionPool().at(i).setMp(p);
				total_count = std::max(total_count, t_count);
			}
		}

		//controller与模型同步，保证3D仿真模型同步显示
		if (target.model->solverPool().at(1).kinPos())return -1;

		// 打印电流 //
		auto &cout = controller->mout();
		if (target.count % 100 == 0)
		{
            for (Size i = 0; i < param.joint_active_vec.size(); i++)
			{
                cout << "pos" << i + 1 << ":" << controller->motionAtAbs(i).targetPos() << "  ";
				cout << "vel" << i + 1 << ":" << controller->motionAtAbs(i).actualVel() << "  ";
			}
			cout << std::endl;
		}

		// log 电流 //
		auto &lout = controller->lout();
		for (Size i = 0; i < param.joint_active_vec.size(); i++)
		{
			lout << controller->motionAtAbs(i).targetPos() << ",";
			lout << controller->motionAtAbs(i).actualPos() << ",";
		}
		lout << std::endl;

		return total_count - target.count;
	}
	auto MoveJR::collectNrt(PlanTarget &target)->void {}
	MoveJR::MoveJR(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"moveJR\">"
			"	<GroupParam>"
			"		<UniqueParam default=\"all\">"
			"			<Param name=\"all\" abbreviation=\"a\"/>"
			"			<Param name=\"motion_id\" abbreviation=\"m\" default=\"0\"/>"
			"			<Param name=\"physical_id\" abbreviation=\"p\" default=\"0\"/>"
			"			<Param name=\"slave_id\" abbreviation=\"s\" default=\"0\"/>"
			"		</UniqueParam>"
			"		<Param name=\"pos\" default=\"0\"/>"
            "		<Param name=\"vel\" default=\"0.3\"/>"
            "		<Param name=\"acc\" default=\"0.6\"/>"
            "		<Param name=\"dec\" default=\"0.6\"/>"
			"	</GroupParam>"
			"</Command>");
	}
	

	// 梯形轨迹2测试--输入单个关节，角度位置；关节按照梯形速度轨迹执行；速度前馈//
	struct MoveTTTParam
	{
		std::vector<double> axis_vel_vec;
		std::vector<double> axis_acc_vec;
		std::vector<double> axis_dec_vec;
		std::vector<double> begin_axis_vel_vec;
		std::vector<double> begin_axis_acc_vec;
		std::vector<double> begin_axis_dec_vec;
		std::vector<double> joint_pos_vec, begin_joint_pos_vec;
		std::vector<bool> joint_active_vec;
	};
	auto MoveTTT::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
		{
			auto c = target.controller;
			MoveTTTParam param;

			for (auto cmd_param : params)
			{
				if (cmd_param.first == "all")
				{
					param.joint_active_vec.resize(c->motionPool().size(), true);
				}
				else if (cmd_param.first == "none")
				{
					param.joint_active_vec.resize(c->motionPool().size(), false);
				}
				else if (cmd_param.first == "motion_id")
				{
					param.joint_active_vec.resize(c->motionPool().size(), false);
					param.joint_active_vec.at(std::stoi(cmd_param.second)) = true;
				}
				else if (cmd_param.first == "physical_id")
				{
					param.joint_active_vec.resize(c->motionPool().size(), false);
					param.joint_active_vec.at(c->motionAtPhy(std::stoi(cmd_param.second)).phyId()) = true;
				}
				else if (cmd_param.first == "slave_id")
				{
					param.joint_active_vec.resize(c->motionPool().size(), false);
					param.joint_active_vec.at(c->motionAtPhy(std::stoi(cmd_param.second)).slaId()) = true;
				}
				else if (cmd_param.first == "pos")
				{
					aris::core::Matrix mat = target.model->calculator().calculateExpression(cmd_param.second);
					if (mat.size() == 1)param.joint_pos_vec.resize(c->motionPool().size(), mat.toDouble());
					else
					{
						param.joint_pos_vec.resize(mat.size());
						std::copy(mat.begin(), mat.end(), param.joint_pos_vec.begin());
					}
				}
				else if (cmd_param.first == "vel")
				{
					auto v = target.model->calculator().calculateExpression(cmd_param.second);
					if (v.size() == 1)
					{
						param.axis_vel_vec.resize(c->motionPool().size(), v.toDouble());
					}
					else if (v.size() == c->motionPool().size())
					{
						param.axis_vel_vec.assign(v.begin(), v.end());
					}
					else
					{
						throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
					}

					for (Size i = 0; i < c->motionPool().size(); ++i)
					{
						if (param.axis_vel_vec[i] > 1.0)
						{
							param.axis_vel_vec[i] = 1.0;
						}
						if (param.axis_vel_vec[i] < 0.0)
						{
							param.axis_vel_vec[i] = 0.0;
						}
						param.axis_vel_vec[i] = param.axis_vel_vec[i] * c->motionPool()[i].maxVel();
					}
				}
				else if (cmd_param.first == "acc")
				{
					auto a = target.model->calculator().calculateExpression(cmd_param.second);
					if (a.size() == 1)
					{
						param.axis_acc_vec.resize(c->motionPool().size(), a.toDouble());
					}
					else if (a.size() == c->motionPool().size())
					{
						param.axis_acc_vec.assign(a.begin(), a.end());
					}
					else
					{
						throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
					}

					for (Size i = 0; i < c->motionPool().size(); ++i)
					{
						if (param.axis_acc_vec[i] > 1.0)
						{
							param.axis_acc_vec[i] = 1.0;
						}
						if (param.axis_acc_vec[i] < 0.0)
						{
							param.axis_acc_vec[i] = 0.0;
						}
						param.axis_acc_vec[i] = param.axis_acc_vec[i] * c->motionPool()[i].maxAcc();
					}
				}
				else if (cmd_param.first == "dec")
				{
					auto d = target.model->calculator().calculateExpression(cmd_param.second);
					if (d.size() == 1)
					{
						param.axis_dec_vec.resize(c->motionPool().size(), d.toDouble());
					}
					else if (d.size() == c->motionPool().size())
					{
						param.axis_dec_vec.assign(d.begin(), d.end());
					}
					else
					{
						throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
					}

					for (Size i = 0; i < c->motionPool().size(); ++i)
					{
						if (param.axis_dec_vec[i] > 1.0)
						{
							param.axis_dec_vec[i] = 1.0;
						}
						if (param.axis_dec_vec[i] < 0.0)
						{
							param.axis_dec_vec[i] = 0.0;
						}
						param.axis_dec_vec[i] = param.axis_dec_vec[i] * c->motionPool()[i].minAcc();
					}
				}
			}

			param.begin_joint_pos_vec.resize(c->motionPool().size(), 0.0);
			param.begin_axis_vel_vec.resize(c->motionPool().size(), 0.0);
			param.begin_axis_acc_vec.resize(c->motionPool().size(), 0.0);
			param.begin_axis_dec_vec.resize(c->motionPool().size(), 0.0);


			target.param = param;

			std::fill(target.mot_options.begin(), target.mot_options.end(),
				Plan::USE_TARGET_POS);

		}
	auto MoveTTT::executeRT(PlanTarget &target)->int
		{
			auto &param = std::any_cast<MoveTTTParam&>(target.param);
			auto controller = target.controller;

			if (target.count == 1)
			{
				for (Size i = 0; i < param.joint_active_vec.size(); ++i)
				{
					if (param.joint_active_vec[i])
					{
						//param.begin_joint_pos_vec[i] = controller->motionPool()[i].actualPos();
						param.begin_joint_pos_vec[i] = target.model->motionPool().at(i).mp();
					}
				}
			}

			aris::Size total_count{ 1 };
			for (Size i = 0; i < param.joint_active_vec.size(); ++i)
			{
				if (param.joint_active_vec[i])
				{
					double p, v, a;
					aris::Size t_count;
					auto result = aris::plan::moveAbsolute2(param.begin_joint_pos_vec[i], param.begin_axis_vel_vec[i], param.begin_axis_acc_vec[i], param.joint_pos_vec[i], 0.0, 0.0, param.axis_vel_vec[i], param.axis_acc_vec[i], param.axis_acc_vec[i], 1e-3, 1e-10, p, v, a, t_count);
					//controller->motionAtAbs(i).setTargetPos(p);
					target.model->motionPool().at(i).setMp(p);
                    total_count = result;
                    //total_count = std::max(total_count, t_count);

					param.begin_joint_pos_vec[i] = p;
					param.begin_axis_vel_vec[i] = v;
					param.begin_axis_acc_vec[i] = a;
				}
			}

            if (target.model->solverPool().at(1).kinPos())return -1;
			   
			// 打印电流 //
			auto &cout = controller->mout();
            if (target.count % 1000 == 0)
			{
				for (Size i = 0; i < 6; i++)
				{
                    if (param.joint_active_vec[i])
                    {
                        cout << "pos" << i + 1 << ":" << controller->motionAtAbs(i).actualPos() << "  ";
                        cout << "vel" << i + 1 << ":" << controller->motionAtAbs(i).actualVel() << "  ";
                        cout << "cur" << i + 1 << ":" << controller->motionAtAbs(i).actualCur() << "  ";
                    }
				}
				cout << std::endl;
			}

			auto &fwd = dynamic_cast<aris::dynamic::ForwardKinematicSolver&>(target.model->solverPool()[1]);
			fwd.cptJacobi();

			auto &lout = controller->lout();
			for (Size i = 0; i < 36; ++i)
			{
				lout <<fwd.Jf()[i] << ",";
			}
			for (Size i = 0; i < param.joint_active_vec.size(); ++i)
			{
				if (param.joint_active_vec[i])
				{
					lout << param.begin_joint_pos_vec[i] << ",";
					lout << param.begin_axis_vel_vec[i] << ",";
					lout << param.begin_axis_acc_vec[i] << ",";
					lout << param.joint_pos_vec[i] << ",";
					lout << param.axis_vel_vec[i] << ",";
					lout << param.axis_acc_vec[i] << ",";
					lout << std::endl;
				}
			}

			// log 电流 //
            //auto &lout = controller->lout();
            //for (Size i = 0; i < 6; i++)
            //{
                //lout << controller->motionAtAbs(i).targetPos() << ",";
                //lout << controller->motionAtAbs(i).actualPos() << ",";
                //lout << controller->motionAtAbs(i).actualVel() << ",";
                //lout << controller->motionAtAbs(i).actualCur() << ",";
            //}
            //lout << std::endl;

            //return total_count - target.count;
            return total_count;
		}
	auto MoveTTT::collectNrt(PlanTarget &target)->void {}
	MoveTTT::MoveTTT(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
            "<Command name=\"moveTTT\">"
			"	<GroupParam>"
			"		<UniqueParam default=\"all\">"
			"			<Param name=\"all\" abbreviation=\"a\"/>"
			"			<Param name=\"motion_id\" abbreviation=\"m\" default=\"0\"/>"
			"			<Param name=\"physical_id\" abbreviation=\"p\" default=\"0\"/>"
			"			<Param name=\"slave_id\" abbreviation=\"s\" default=\"0\"/>"
			"		</UniqueParam>"
			"		<Param name=\"pos\" default=\"0\"/>"
            "		<Param name=\"vel\" default=\"0.04\"/>"
            "		<Param name=\"acc\" default=\"0.1\"/>"
            "		<Param name=\"dec\" default=\"0.1\"/>"
			"	</GroupParam>"
			"</Command>");
	}


	// 多关节混合插值梯形轨迹；速度前馈 //
	struct MoveJMParam
	{
		std::vector<Size> total_count_vec;
		std::vector<double> axis_begin_pos_vec;
		std::vector<double> axis_pos_vec;
		std::vector<double> axis_vel_vec;
		std::vector<double> axis_acc_vec;
		std::vector<double> axis_dec_vec;
		bool ab;
	};
	auto MoveJM::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
		{
			auto c = target.controller;
			MoveJMParam param;
			param.total_count_vec.resize(6, 1);
			param.axis_begin_pos_vec.resize(6, 0.0);

			//params.at("pos")
			for (auto &p : params)
			{
				if (p.first == "pos")
				{
					if (p.second == "current_pos")
					{
						target.option |= aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;
					}
					else
					{
						auto pos = target.model->calculator().calculateExpression(p.second);
						if (pos.size() == 1)
						{
							param.axis_pos_vec.resize(param.axis_begin_pos_vec.size(), pos.toDouble());
						}
						else if (pos.size() == param.axis_begin_pos_vec.size())
						{
							param.axis_pos_vec.assign(pos.begin(), pos.end());
						}
						else
						{
							throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
						}

						for (Size i = 0; i < param.axis_begin_pos_vec.size(); ++i)
						{
							//超阈值保护//
							if (param.axis_pos_vec[i] > 1.0)
							{
								param.axis_pos_vec[i] = 1.0;
							}
							if (param.axis_pos_vec[i] < -1.0)
							{
								param.axis_pos_vec[i] = -1.0;
							}
							if (param.axis_pos_vec[i] >= 0)
							{
								param.axis_pos_vec[i] = param.axis_pos_vec[i] * c->motionPool()[i].maxPos();
							}
							else
							{
								param.axis_pos_vec[i] = param.axis_pos_vec[i] * c->motionPool()[i].minPos();
							}					
						}
					}
				}
				else if (p.first == "vel")
				{
					auto v = target.model->calculator().calculateExpression(p.second);
					if (v.size() == 1)
					{
						param.axis_vel_vec.resize(param.axis_begin_pos_vec.size(), v.toDouble());
					}
					else if (v.size() == param.axis_begin_pos_vec.size())
					{
						param.axis_vel_vec.assign(v.begin(), v.end());
					}
					else
					{
						throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
					}

					for (Size i = 0; i < param.axis_begin_pos_vec.size(); ++i)
					{
						//if (param.axis_vel_vec[i] > 1.0 || param.axis_vel_vec[i] < 0.01)
						//	throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
						if (param.axis_vel_vec[i] > 1.0)
						{
							param.axis_vel_vec[i] = 1.0;
						}
						if (param.axis_vel_vec[i] < 0.0)
						{
							param.axis_vel_vec[i] = 0.0;
						}
						param.axis_vel_vec[i] = param.axis_vel_vec[i] * c->motionPool()[i].maxVel();
					}
				}
				else if (p.first == "acc")
				{
					auto a = target.model->calculator().calculateExpression(p.second);
					if (a.size() == 1)
					{
						param.axis_acc_vec.resize(param.axis_begin_pos_vec.size(), a.toDouble());
					}
					else if (a.size() == param.axis_begin_pos_vec.size())
					{
						param.axis_acc_vec.assign(a.begin(), a.end());
					}
					else
					{
						throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
					}

					for (Size i = 0; i < param.axis_begin_pos_vec.size(); ++i)
					{
						if (param.axis_acc_vec[i] > 1.0)
						{
							param.axis_acc_vec[i] = 1.0;
						}
						if (param.axis_acc_vec[i] < 0.0)
						{
							param.axis_acc_vec[i] = 0.0;
						}
						param.axis_acc_vec[i] = param.axis_acc_vec[i] * c->motionPool()[i].maxAcc();
					}
				}
				else if (p.first == "dec")
				{
					auto d = target.model->calculator().calculateExpression(p.second);
					if (d.size() == 1)
					{
						param.axis_dec_vec.resize(param.axis_begin_pos_vec.size(), d.toDouble());
					}
					else if (d.size() == param.axis_begin_pos_vec.size())
					{
						param.axis_dec_vec.assign(d.begin(), d.end());
					}
					else
					{
						throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
					}

					for (Size i = 0; i < param.axis_begin_pos_vec.size(); ++i)
					{
						if (param.axis_dec_vec[i] > 1.0)
						{
							param.axis_dec_vec[i] = 1.0;
						}
						if (param.axis_dec_vec[i] < 0.0)
						{
							param.axis_dec_vec[i] = 0.0;
						}
						param.axis_dec_vec[i] = param.axis_dec_vec[i] * c->motionPool()[i].minAcc();
					}
				}
				else if (p.first == "ab")
				{
					param.ab = std::stod(p.second);
				}
			}
			target.param = param;

			//std::fill(target.mot_options.begin(), target.mot_options.end(), Plan::USE_VEL_OFFSET);

		}
	auto MoveJM::executeRT(PlanTarget &target)->int
		{
			//获取驱动//
			auto controller = target.controller;
			auto &param = std::any_cast<MoveJMParam&>(target.param);
			static double begin_pos[6];
			static double pos[6];
			// 取得起始位置 //
			if (target.count == 1)
			{
				for (Size i = 0; i < param.axis_begin_pos_vec.size(); ++i)
				{
					param.axis_begin_pos_vec[i] = controller->motionPool().at(i).targetPos();
				}
			}
			// 设置驱动器的位置 //
			if (param.ab)
			{
				for (Size i = 0; i < param.axis_begin_pos_vec.size(); ++i)
				{
					double p, v, a;
					aris::plan::moveAbsolute(target.count, param.axis_begin_pos_vec[i], param.axis_pos_vec[i], param.axis_vel_vec[i] / 1000
						, param.axis_acc_vec[i] / 1000 / 1000, param.axis_dec_vec[i] / 1000 / 1000, p, v, a, param.total_count_vec[i]);
					controller->motionAtAbs(i).setTargetPos(p);
					//速度前馈//
					controller->motionAtAbs(i).setOffsetVel(v * 1000);
					target.model->motionPool().at(i).setMp(p);
				}
			}
			else
			{
				for (Size i = 0; i < param.axis_begin_pos_vec.size(); ++i)
				{
					double p, v, a;
					aris::plan::moveAbsolute(target.count, param.axis_begin_pos_vec[i], param.axis_begin_pos_vec[i] + param.axis_pos_vec[i], param.axis_vel_vec[i] / 1000
						, param.axis_acc_vec[i] / 1000 / 1000, param.axis_dec_vec[i] / 1000 / 1000, p, v, a, param.total_count_vec[i]);
					controller->motionAtAbs(i).setTargetPos(p);
					//速度前馈//
					controller->motionAtAbs(i).setOffsetVel(v * 1000);
					target.model->motionPool().at(i).setMp(p);
				}
			}
				
			if (target.model->solverPool().at(1).kinPos())return -1;

			// 打印电流 //
			auto &cout = controller->mout();
			if (target.count % 100 == 0)
			{
				for (Size i = 0; i < 6; i++)
				{
					cout << "pos" << i + 1 << ":" << controller->motionAtAbs(i).actualPos() << "  ";
					cout << "vel" << i + 1 << ":" << controller->motionAtAbs(i).actualVel() << "  ";
					cout << "cur" << i + 1 << ":" << controller->motionAtAbs(i).actualCur() << "  ";
				}
				cout << std::endl;
			}

			// log 电流 //
			auto &lout = controller->lout();
			for (Size i = 0; i < 6; i++)
			{
				lout << controller->motionAtAbs(i).targetPos() << ",";
				lout << controller->motionAtAbs(i).actualPos() << ",";
				lout << controller->motionAtAbs(i).actualVel() << ",";
				lout << controller->motionAtAbs(i).actualCur() << ",";
			}
			lout << std::endl;

			return (static_cast<int>(*std::max_element(param.total_count_vec.begin(), param.total_count_vec.end())) > target.count) ? 1 : 0;
		}
	auto MoveJM::collectNrt(PlanTarget &target)->void {}
	MoveJM::MoveJM(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"moveJM\">"
			"	<GroupParam>"
			"		<Param name=\"pos\" default=\"current_pos\"/>"
			"		<Param name=\"vel\" default=\"{0.2,0.2,0.2,0.2,0.2,0.2}\" abbreviation=\"v\"/>"
			"		<Param name=\"acc\" default=\"{0.1,0.1,0.1,0.1,0.1,0.1}\" abbreviation=\"a\"/>"
			"		<Param name=\"dec\" default=\"{0.1,0.1,0.1,0.1,0.1,0.1}\" abbreviation=\"d\"/>"
			"		<Param name=\"ab\" default=\"1\"/>"
			"	</GroupParam>"
			"</Command>");
	}


	// 关节插值运动轨迹--输入末端pq姿态，各个关节的速度、加速度；各关节按照梯形速度轨迹执行；速度前馈 //
	struct MoveJIParam
	{
		std::vector<double> pq;
		std::vector<Size> total_count_vec;
		std::vector<double> axis_begin_pos_vec;
		std::vector<double> axis_pos_vec;
		std::vector<double> axis_vel_vec;
		std::vector<double> axis_acc_vec;
		std::vector<double> axis_dec_vec;
	};
	auto MoveJI::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
		{
			auto c = target.controller;
			MoveJIParam param;
			param.pq.resize(7, 0.0);
			param.total_count_vec.resize(6, 1);
			param.axis_begin_pos_vec.resize(6, 0.0);
			param.axis_pos_vec.resize(6, 0.0);

			//params.at("pq")
			for (auto &p : params)
			{
				if (p.first == "pq")
				{
					if (p.second == "current_pos")
					{
						target.option |= aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;
					}
					else
					{
						auto pqarray = target.model->calculator().calculateExpression(p.second);
						param.pq.assign(pqarray.begin(), pqarray.end());
					}
				}
				else if (p.first == "vel")
				{
					auto v = target.model->calculator().calculateExpression(p.second);
					if (v.size() == 1)
					{
						param.axis_vel_vec.resize(param.axis_pos_vec.size(), v.toDouble());
					}
					else if (v.size() == param.axis_pos_vec.size())
					{
						param.axis_vel_vec.assign(v.begin(), v.end());
					}
					else
					{
						throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
					}

					for (Size i = 0; i < param.axis_pos_vec.size(); ++i)
					{
						//if (param.axis_vel_vec[i] > 1.0 || param.axis_vel_vec[i] < 0.01)
						//	throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
						if (param.axis_vel_vec[i] > 1.0)
						{
							param.axis_vel_vec[i] = 1.0;
						}
						if (param.axis_vel_vec[i] < 0.0)
						{
							param.axis_vel_vec[i] = 0.0;
						}
						param.axis_vel_vec[i] = param.axis_vel_vec[i] * c->motionPool()[i].maxVel();
					}
				}
				else if (p.first == "acc")
				{
					auto a = target.model->calculator().calculateExpression(p.second);
					if (a.size() == 1)
					{
						param.axis_acc_vec.resize(param.axis_pos_vec.size(), a.toDouble());
					}
					else if (a.size() == param.axis_pos_vec.size())
					{
						param.axis_acc_vec.assign(a.begin(), a.end());
					}
					else
					{
						throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
					}

					for (Size i = 0; i < param.axis_pos_vec.size(); ++i)
					{
						if (param.axis_acc_vec[i] > 1.0)
						{
							param.axis_acc_vec[i] = 1.0;
						}
						if (param.axis_acc_vec[i] < 0.0)
						{
							param.axis_acc_vec[i] = 0.0;
						}
						param.axis_acc_vec[i] = param.axis_acc_vec[i] * c->motionPool()[i].maxAcc();
					}
				}
				else if (p.first == "dec")
				{
					auto d = target.model->calculator().calculateExpression(p.second);
					if (d.size() == 1)
					{
						param.axis_dec_vec.resize(param.axis_pos_vec.size(), d.toDouble());
					}
					else if (d.size() == param.axis_pos_vec.size())
					{
						param.axis_dec_vec.assign(d.begin(), d.end());
					}
					else
					{
						throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
					}

					for (Size i = 0; i < param.axis_pos_vec.size(); ++i)
					{
						if (param.axis_dec_vec[i] > 1.0)
						{
							param.axis_dec_vec[i] = 1.0;
						}	
						if (param.axis_dec_vec[i] < 0.0)
						{
							param.axis_dec_vec[i] = 0.0;
						}
						param.axis_dec_vec[i] = param.axis_dec_vec[i] * c->motionPool()[i].minAcc();
					}
				}
			}
			target.param = param;

			std::fill(target.mot_options.begin(), target.mot_options.end(),
				Plan::USE_VEL_OFFSET);

		}
	auto MoveJI::executeRT(PlanTarget &target)->int
		{
            //获取驱动//
			auto controller = target.controller;
			auto &param = std::any_cast<MoveJIParam&>(target.param);
			static double begin_pos[6];
			static double pos[6];
			// 取得起始位置 //
            if (target.count == 1)
			{
				target.model->generalMotionPool().at(0).setMpq(param.pq.data());	//generalMotionPool()指模型末端，at(0)表示第1个末端，对于6足就有6个末端，对于机器人只有1个末端
				if (target.model->solverPool().at(0).kinPos())return -1;
				for (Size i = 0; i < param.axis_pos_vec.size(); ++i)
				{
					param.axis_begin_pos_vec[i] = controller->motionPool().at(i).targetPos();
					param.axis_pos_vec[i] = target.model->motionPool().at(i).mp();		//motionPool()指模型驱动器，at(0)表示第1个驱动器
				}
			}
            // 设置驱动器的位置 //
			for (Size i = 0; i < param.axis_pos_vec.size(); ++i)
			{
				double p, v, a;
				aris::plan::moveAbsolute(target.count, param.axis_begin_pos_vec[i], param.axis_pos_vec[i], param.axis_vel_vec[i] / 1000
					, param.axis_acc_vec[i] / 1000 / 1000, param.axis_dec_vec[i] / 1000 / 1000, p, v, a, param.total_count_vec[i]);
				controller->motionAtAbs(i).setTargetPos(p);
				//速度前馈//
				controller->motionAtAbs(i).setOffsetVel(v*1000);
				target.model->motionPool().at(i).setMp(p);
			}		
			if (target.model->solverPool().at(1).kinPos())return -1;

			// 打印电流 //
			auto &cout = controller->mout();
			if (target.count % 100 == 0)
			{
				for (Size i = 0; i < 6; i++)
				{
					cout << "pos" << i + 1 << ":" << controller->motionAtAbs(i).actualPos() << "  ";
					cout << "vel" << i + 1 << ":" << controller->motionAtAbs(i).actualVel() << "  ";
					cout << "cur" << i + 1 << ":" << controller->motionAtAbs(i).actualCur() << "  ";
				}
                //cout <<endl;
			}


			// log 电流 //
			auto &lout = controller->lout();
			for (Size i = 0; i < 6; i++)
			{
				lout << controller->motionAtAbs(i).targetPos() << ",";
				lout << controller->motionAtAbs(i).actualPos() << ",";
				lout << controller->motionAtAbs(i).actualVel() << ",";
				lout << controller->motionAtAbs(i).actualCur() << ",";
			}
			lout << std::endl;

			return (static_cast<int>(*std::max_element(param.total_count_vec.begin(), param.total_count_vec.end())) > target.count) ? 1 : 0;
		}
	auto MoveJI::collectNrt(PlanTarget &target)->void {}
	MoveJI::MoveJI(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"moveJI\">"
			"	<GroupParam>"
			"		<Param name=\"pq\" default=\"current_pos\"/>"
            "		<Param name=\"vel\" default=\"{0.05,0.05,0.05,0.05,0.05,0.05}\" abbreviation=\"v\"/>"
			"		<Param name=\"acc\" default=\"{0.1,0.1,0.1,0.1,0.1,0.1}\" abbreviation=\"a\"/>"
			"		<Param name=\"dec\" default=\"{0.1,0.1,0.1,0.1,0.1,0.1}\" abbreviation=\"d\"/>"
			"	</GroupParam>"
			"</Command>");
	}

	
	// 示教运动--输入末端大地坐标系的位姿pe，控制动作 //
	struct JogCParam {};
	struct JogCStruct
	{
		bool jogc_is_running = false;
		int cor_system;
		int vel_percent;
		std::array<int, 6> is_increase;
	};
	struct JogC::Imp
	{
		JogCStruct s1_rt, s2_nrt;
		std::vector<double> pm_target;
		double vel[6], acc[6], dec[6];
		int increase_count;
	};
	std::atomic_bool jogc_is_changing = false;
	auto JogC::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		auto c = target.controller;
		JogCParam param;
		imp_->pm_target.resize(16, 0.0);
		
		std::string ret = "ok";
		target.ret = ret;
		
		for (auto &p : params)
		{
			if (p.first == "start")
			{
				if (imp_->s1_rt.jogc_is_running)throw std::runtime_error("auto mode already started");

				imp_->s2_nrt.jogc_is_running = true;
				std::fill_n(imp_->s2_nrt.is_increase.data(), 6, 0);
				imp_->s2_nrt.cor_system= 0;
				imp_->s2_nrt.vel_percent = 10;

				imp_->s1_rt.jogc_is_running = true;
				std::fill_n(imp_->s1_rt.is_increase.data(), 6, 0);
				imp_->s1_rt.cor_system = 0;
				imp_->s1_rt.vel_percent = 10;

				imp_->increase_count = std::stoi(params.at("increase_count"));
				if (imp_->increase_count < 0 || imp_->increase_count>1e5)THROW_FILE_AND_LINE("");

				auto mat = target.model->calculator().calculateExpression(params.at("vel"));
				if (mat.size() != 6)THROW_FILE_AND_LINE("");
				std::copy(mat.begin(), mat.end(), imp_->vel);

				mat = target.model->calculator().calculateExpression(params.at("acc"));
				if (mat.size() != 6)THROW_FILE_AND_LINE("");
				std::copy(mat.begin(), mat.end(), imp_->acc);

				mat = target.model->calculator().calculateExpression(params.at("dec"));
				if (mat.size() != 6)THROW_FILE_AND_LINE("");
				std::copy(mat.begin(), mat.end(), imp_->dec);

				std::fill(target.mot_options.begin(), target.mot_options.end(), USE_TARGET_POS);
                //target.option |= EXECUTE_WHEN_ALL_PLAN_COLLECTED | NOT_PRINT_EXECUTE_COUNT;
			}
			else if (p.first == "stop")
			{
				if (!imp_->s1_rt.jogc_is_running)throw std::runtime_error("manual mode not started, when stop");

				imp_->s2_nrt.jogc_is_running = false;
                std::fill_n(imp_->s2_nrt.is_increase.data(), 6, 0);

                target.option |= NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION;
				jogc_is_changing = true;
				while (jogc_is_changing.load())std::this_thread::sleep_for(std::chrono::milliseconds(1));
			}
			else if (p.first == "cor")
			{
				if (!imp_->s1_rt.jogc_is_running)throw std::runtime_error("manual mode not started, when pe");

				imp_->s2_nrt.cor_system = std::stoi(params.at("cor"));
				auto velocity = std::stoi(params.at("vel_percent"));
				velocity = std::max(std::min(100, velocity), -100);
				imp_->s2_nrt.vel_percent = velocity;
				imp_->s2_nrt.is_increase[0] = std::max(std::min(1, std::stoi(params.at("x"))), -1) * imp_->increase_count;
				imp_->s2_nrt.is_increase[1] = std::max(std::min(1, std::stoi(params.at("y"))), -1) * imp_->increase_count;
				imp_->s2_nrt.is_increase[2] = std::max(std::min(1, std::stoi(params.at("z"))), -1) * imp_->increase_count;
				imp_->s2_nrt.is_increase[3] = std::max(std::min(1, std::stoi(params.at("a"))), -1) * imp_->increase_count;
				imp_->s2_nrt.is_increase[4] = std::max(std::min(1, std::stoi(params.at("b"))), -1) * imp_->increase_count;
				imp_->s2_nrt.is_increase[5] = std::max(std::min(1, std::stoi(params.at("c"))), -1) * imp_->increase_count;

				imp_->s2_nrt.jogc_is_running = true;

				target.option |= NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION | NOT_PRINT_CMD_INFO | NOT_LOG_CMD_INFO;
				jogc_is_changing = true;
				while (jogc_is_changing.load())std::this_thread::sleep_for(std::chrono::milliseconds(1));
			}
		}
		
		target.param = param;
	}
	auto JogC::executeRT(PlanTarget &target)->int
	{	
		//获取驱动//
		auto controller = target.controller;
		auto &param = std::any_cast<JogCParam&>(target.param);
		char eu_type[4]{ '1', '2', '3', '\0' };
		
		// 前三维为xyz，后三维是w的积分，注意没有物理含义
		static double target_p[6];

		// get current pe //
		static double p_now[6], v_now[6], a_now[6];
		if (target.count == 1)
		{
			target.model->generalMotionPool()[0].getMpe(target_p);
			std::fill_n(target_p + 3, 3, 0.0);

			target.model->generalMotionPool()[0].getMpe(p_now, eu_type);
			std::fill_n(p_now + 3, 3, 0.0);

			target.model->generalMotionPool()[0].getMve(v_now, eu_type);
			target.model->generalMotionPool()[0].getMae(a_now, eu_type);
		}
		
		// init status //
		static int increase_status[6]{ 0,0,0,0,0,0 };
		double max_vel[6];
		if (jogc_is_changing)
		{
			imp_->s1_rt = imp_->s2_nrt;
			jogc_is_changing.store(false);
			for (int i = 0; i < 6; i++)
			{
				increase_status[i] = imp_->s1_rt.is_increase[i];
			}
			//target.model->generalMotionPool()[0].getMpe(imp_->pe_start, eu_type);
		}

		// calculate target pos and max vel //
		for (int i = 0; i < 6; i++)
		{
			max_vel[i] = imp_->vel[i]*1.0*imp_->s1_rt.vel_percent / 100.0;
			target_p[i] += aris::dynamic::s_sgn(increase_status[i])*max_vel[i] * 1e-3;
			increase_status[i] -= aris::dynamic::s_sgn(increase_status[i]);
		}
		//std::copy_n(target_pos, 6, imp_->pe_start);
		
		// 梯形轨迹规划 calculate real value //
		double p_next[6]{ 0,0,0,0,0,0 }, v_next[6]{ 0,0,0,0,0,0 }, a_next[6]{ 0,0,0,0,0,0 };
		for(int i=0; i<6; i++)
		{
			aris::Size t;
			aris::plan::moveAbsolute2(p_now[i], v_now[i], a_now[i]
				, target_p[i], 0.0, 0.0
				, max_vel[i], imp_->acc[i], imp_->dec[i]
				, 1e-3, 1e-10, p_next[i], v_next[i], a_next[i], t);
		}

		//将欧拉角转换成四元数，求绕任意旋转轴转动的旋转矩阵//
		double w[3], pm[16];
		aris::dynamic::s_vc(3, v_next + 3, w);
		auto normv = aris::dynamic::s_norm(3, w);
		if (std::abs(normv) > 1e-10)aris::dynamic::s_nv(3, 1 / normv, w); //数乘
		auto theta = normv * 1e-3;
		double pq[7]{ p_next[0] - p_now[0], p_next[1] - p_now[1], p_next[2] - p_now[2], w[0]*sin(theta / 2.0), w[1] * sin(theta / 2.0), w[2] * sin(theta / 2.0), cos(theta / 2.0) };
		s_pq2pm(pq, pm);

		// 获取当前位姿矩阵 //
		double pm_now[16];
		target.model->generalMotionPool()[0].getMpm(pm_now);

		// 保存下个周期的copy //
		s_vc(6, p_next, p_now);
		s_vc(6, v_next, v_now);
		s_vc(6, a_next, a_now);
		/*
		s_pe2pm(p_now, imp_->pm_now.data(), eu_type);
		for (int i = 0; i < 6; i++)
		{
			p_now[i] = p_next[i];
			v_now[i] = v_next[i];
			a_now[i] = a_next[i];
		}
		*/
		
		//绝对坐标系
		if (imp_->s1_rt.cor_system == 0)
		{
			s_pm_dot_pm(pm, pm_now, imp_->pm_target.data());
		}
		//工具坐标系
		else if (imp_->s1_rt.cor_system == 1)
		{
			s_pm_dot_pm(pm_now, pm, imp_->pm_target.data());
		}
		
		target.model->generalMotionPool().at(0).setMpm(imp_->pm_target.data());
		
		// 运动学反解 //
		if (target.model->solverPool().at(0).kinPos())return -1;

		// 打印 //
		auto &cout = controller->mout();
		if (target.count % 200 == 0)
		{
			cout << "pm_target:" << std::endl;
			for (Size i = 0; i < 16; i++)
			{
				cout << imp_->pm_target[i] << "  ";
			}
			cout << std::endl;
			cout << "increase_status:" << std::endl;
			for (Size i = 0; i < 6; i++)
			{
				cout << increase_status[i] << "  ";
			}
			cout << std::endl;
			cout << "p_next:" << std::endl;
			for (Size i = 0; i < 6; i++)
			{
				cout << p_next[i] << "  ";
			}
			cout << std::endl;
			cout << "v_next:" << std::endl;
			for (Size i = 0; i < 6; i++)
			{
				cout << v_next[i] << "  ";
			}
			cout << std::endl;
			cout << "p_now:" << std::endl;
			for (Size i = 0; i < 6; i++)
			{
				cout << p_now[i] << "  ";
			}
			cout << std::endl;
			cout << "v_now:" << std::endl;
			for (Size i = 0; i < 6; i++)
			{
				cout << v_now[i] << "  ";
			}
			cout << std::endl;
		}

		// log //
		auto &lout = controller->lout();
		for (int i = 0; i < 6; i++)
		{
			lout << target_p[i] << " ";
			lout << p_now[i] << " ";
			lout << v_now[i] << " ";
			lout << a_now[i] << " ";
			lout << controller->motionAtAbs(i).actualPos() << " ";
			lout << controller->motionAtAbs(i).actualVel() << " ";
		}
		lout << std::endl;

		return imp_->s1_rt.jogc_is_running ? 1 : 0;
	}
	auto JogC::collectNrt(PlanTarget &target)->void {}
	JogC::~JogC() = default;
	JogC::JogC(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"jogC\">"
			"	<GroupParam>"
			"		<UniqueParam>"
			"			<GroupParam name=\"start_group\">"
			"				<Param name=\"start\"/>"
			"				<Param name=\"increase_count\" default=\"100\"/>"
            "				<Param name=\"vel\" default=\"{0.05,0.05,0.05,0.25,0.25,0.25}\"/>"
            "				<Param name=\"acc\" default=\"{0.2,0.2,0.2,1,1,1}\"/>"
            "				<Param name=\"dec\" default=\"{0.2,0.2,0.2,1,1,1}\"/>"
			"			</GroupParam>"
			"			<Param name=\"stop\"/>"
			"			<GroupParam>"
			"				<Param name=\"cor\" default=\"0\"/>"
			"				<Param name=\"vel_percent\" default=\"10\"/>"
			"				<Param name=\"x\" default=\"0\"/>"
			"				<Param name=\"y\" default=\"0\"/>"
			"				<Param name=\"z\" default=\"0\"/>"
			"				<Param name=\"a\" default=\"0\"/>"
			"				<Param name=\"b\" default=\"0\"/>"
			"				<Param name=\"c\" default=\"0\"/>"
			"			</GroupParam>"
			"		</UniqueParam>"
			"	</GroupParam>"
			"</Command>");
	}
	JogC::JogC(const JogC &other) = default;
	JogC::JogC(JogC &other) = default;
	JogC& JogC::operator=(const JogC &other) = default;
	JogC& JogC::operator=(JogC &&other) = default;


	// 示教运动--关节空间点动 //
	struct JogJParam {};
	struct JogJStruct
	{
		bool jogj_is_running = false;
		int vel_percent;
		std::vector<int> is_increase;
	};
	struct JogJ::Imp
	{
		JogJStruct s1_rt, s2_nrt;
		double vel, acc, dec;
		std::vector<double> p_now, v_now, a_now, target_pos, max_vel;
		std::vector<int> increase_status;
		int increase_count;
	};
	std::atomic_bool jogj_is_changing = false;
	auto JogJ::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		auto c = target.controller;
		JogJParam param;

		std::string ret = "ok";
		target.ret = ret;

		imp_->p_now.resize(c->motionPool().size(), 0.0);
		imp_->v_now.resize(c->motionPool().size(), 0.0);
		imp_->a_now.resize(c->motionPool().size(), 0.0);
		imp_->target_pos.resize(c->motionPool().size(), 0.0);
		imp_->max_vel.resize(c->motionPool().size(), 0.0);
		imp_->increase_status.resize(c->motionPool().size(), 0);

		for (auto &p : params)
		{
			if (p.first == "start")
			{
				if (imp_->s1_rt.jogj_is_running)throw std::runtime_error("auto mode already started");

				imp_->s2_nrt.jogj_is_running = true;
				imp_->s2_nrt.is_increase.clear();
				imp_->s2_nrt.is_increase.resize(c->motionPool().size(), 0);
				imp_->s2_nrt.vel_percent = 10;

				imp_->s1_rt.jogj_is_running = true;
				imp_->s1_rt.is_increase.clear();
				imp_->s1_rt.is_increase.resize(c->motionPool().size(), 0);
				imp_->s1_rt.vel_percent = 10;

				imp_->increase_count = std::stoi(params.at("increase_count"));
				if (imp_->increase_count < 0 || imp_->increase_count>1e5)THROW_FILE_AND_LINE("");
				imp_->vel = std::stod(params.at("vel"));
				imp_->acc = std::stod(params.at("acc"));
				imp_->dec = std::stod(params.at("dec"));

                std::fill(target.mot_options.begin(), target.mot_options.end(), NOT_CHECK_POS_FOLLOWING_ERROR | USE_TARGET_POS);
                //target.option |= EXECUTE_WHEN_ALL_PLAN_COLLECTED | NOT_PRINT_EXECUTE_COUNT;
			}
			else if (p.first == "stop")
			{
				if (!imp_->s1_rt.jogj_is_running)throw std::runtime_error("manual mode not started, when stop");

				imp_->s2_nrt.jogj_is_running = false;
                imp_->s2_nrt.is_increase.assign(imp_->s2_nrt.is_increase.size(), 0);
				jogj_is_changing = true;
				while (jogj_is_changing.load())std::this_thread::sleep_for(std::chrono::milliseconds(1));
				target.option |= NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION;
			}
			else if (p.first == "vel_percent")
			{
				if (!imp_->s1_rt.jogj_is_running)throw std::runtime_error("manual mode not started, when pe");

				auto velocity = std::stoi(params.at("vel_percent"));
				velocity = std::max(std::min(100, velocity), -100);
				imp_->s2_nrt.vel_percent = velocity;

				imp_->s2_nrt.is_increase.assign(imp_->s2_nrt.is_increase.size(), 0);
				imp_->s2_nrt.is_increase[std::stoi(params.at("motion_id"))] = std::max(std::min(1, std::stoi(params.at("direction"))), -1) * imp_->increase_count;
				
				imp_->s2_nrt.jogj_is_running = true;	

				target.option |= NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION | NOT_PRINT_CMD_INFO | NOT_LOG_CMD_INFO;
				jogj_is_changing = true;
				while (jogj_is_changing.load())std::this_thread::sleep_for(std::chrono::milliseconds(1));
			}
		}

		target.param = param;
	}
	auto JogJ::executeRT(PlanTarget &target)->int
	{

		//获取驱动//
		auto controller = target.controller;
		auto &param = std::any_cast<JogJParam&>(target.param);

		// get current pe //
		if (target.count == 1)
		{
			for (Size i = 0; i < imp_->p_now.size(); ++i)
			{
                /*
				imp_->p_start[i] = target.model->motionPool().at(i).mp();
				imp_->p_now[i] = target.model->motionPool().at(i).mp();
				imp_->v_now[i] = target.model->motionPool().at(i).mv();
				imp_->a_now[i] = target.model->motionPool().at(i).ma();
                */
                imp_->target_pos[i] = controller->motionAtAbs(i).actualPos();
                imp_->p_now[i] = controller->motionAtAbs(i).actualPos();
                imp_->v_now[i] = controller->motionAtAbs(i).actualVel();
                imp_->a_now[i] = 0.0;
			}
		}
		// init status and calculate target pos and max vel //
		
		if (jogj_is_changing)
		{
			jogj_is_changing.store(false);
			imp_->s1_rt = imp_->s2_nrt;
			for (int i = 0; i < imp_->s1_rt.is_increase.size(); i++)
			{
				imp_->increase_status[i] = imp_->s1_rt.is_increase[i];
			}	
		}
		for (int i = 0; i < imp_->s1_rt.is_increase.size(); i++)
		{
			imp_->max_vel[i] = imp_->vel*1.0*imp_->s1_rt.vel_percent / 100.0;
			imp_->target_pos[i] += aris::dynamic::s_sgn(imp_->increase_status[i])*imp_->max_vel[i] * 1e-3;
			imp_->increase_status[i] -= aris::dynamic::s_sgn(imp_->increase_status[i]);
		}
		// 梯形轨迹规划 //
		static double p_next, v_next, a_next;
		for(int i=0; i< imp_->p_now.size(); i++)
		{
			aris::Size t;
			aris::plan::moveAbsolute2(imp_->p_now[i], imp_->v_now[i], imp_->a_now[i]
                , imp_->target_pos[i], 0.0, 0.0
				, imp_->max_vel[i], imp_->acc, imp_->dec
				, 1e-3, 1e-10, p_next, v_next, a_next, t);

            target.model->motionPool().at(i).setMp(p_next);
            controller->motionAtAbs(i).setTargetPos(p_next);
			imp_->p_now[i] = p_next;
			imp_->v_now[i] = v_next;
			imp_->a_now[i] = a_next;
		}

        // 运动学反解//
		if (target.model->solverPool().at(1).kinPos())return -1;

		// 打印 //
		auto &cout = controller->mout();
        if (target.count % 200 == 0)
		{
			for (int i = 0; i < imp_->p_now.size(); i++)
			{
				cout << imp_->increase_status[i] << "  ";
			}
			cout << std::endl;
			for (int i = 0; i < imp_->p_now.size(); i++)
			{
				cout << imp_->target_pos[i] << "  ";
			}
			cout << std::endl;
			for (int i = 0; i < imp_->p_now.size(); i++)
			{
				cout << imp_->p_now[i] << "  ";
			}
			cout << std::endl;
			for (int i = 0; i < imp_->p_now.size(); i++)
			{
				cout << imp_->v_now[i] << "  ";
			}
			cout << std::endl;
			cout << "------------------------------------------" <<std::endl;
		}

		// log //
		auto &lout = controller->lout();
		for (int i = 0; i < imp_->p_now.size(); i++)
		{
            lout << imp_->target_pos[i] << " ";
            lout << imp_->v_now[i] << " ";
            lout << imp_->a_now[i] << " ";
            lout << controller->motionAtAbs(i).actualPos() << " ";
            lout << controller->motionAtAbs(i).actualVel() << " ";
            //lout << controller->motionAtAbs(i).actualCur() << " ";
		}
		lout << std::endl;

		return imp_->s1_rt.jogj_is_running ? 1 : 0;
	}
	auto JogJ::collectNrt(PlanTarget &target)->void {}
	JogJ::~JogJ() = default;
	JogJ::JogJ(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"jogJ\">"
			"	<GroupParam>"
			"		<UniqueParam>"
			"			<GroupParam name=\"start_group\">"
			"				<Param name=\"start\"/>"
			"				<Param name=\"increase_count\" default=\"100\"/>"
			"				<Param name=\"vel\" default=\"1\" abbreviation=\"v\"/>"
			"				<Param name=\"acc\" default=\"5\" abbreviation=\"a\"/>"
			"				<Param name=\"dec\" default=\"5\" abbreviation=\"d\"/>"
			"			</GroupParam>"
			"			<Param name=\"stop\"/>"
			"			<GroupParam>"
			"				<Param name=\"vel_percent\" default=\"10\"/>"
			"				<Param name=\"motion_id\" default=\"0\" abbreviation=\"m\"/>"
			"				<Param name=\"direction\" default=\"1\"/>"
			"			</GroupParam>"
			"		</UniqueParam>"
			"	</GroupParam>"
			"</Command>");
	}
	JogJ::JogJ(const JogJ &other) = default;
	JogJ::JogJ(JogJ &other) = default;
	JogJ& JogJ::operator=(const JogJ &other) = default;
	JogJ& JogJ::operator=(JogJ &&other) = default;


	// 夹爪控制 //
	struct GraspParam
	{
		bool status;
	};
	auto Grasp::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
		{
			GraspParam param = { 0 };
			for (auto &p : params)
			{
				if (p.first == "status")
				{
					param.status = std::stod(p.second);
				}
			}
			target.param = param;

		}
	auto Grasp::executeRT(PlanTarget &target)->int
		{
			auto &param = std::any_cast<GraspParam&>(target.param);
			// 访问主站 //
			auto controller = dynamic_cast<aris::control::EthercatController*>(target.controller);
			static std::uint8_t dq = 0x01;
			if (param.status)
			{
				dq = 0x01;
				controller->slavePool().at(7).writePdo(0x7001, 0x01, dq);
			}
			else
			{
				dq = 0x00;
				controller->slavePool().at(7).writePdo(0x7001, 0x01, dq);
			}	
			//std::cout << int(a) << std::endl;
			return 0;
		}
	auto Grasp::collectNrt(PlanTarget &target)->void {}
	Grasp::Grasp(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"grasp\">"
			"	<GroupParam>"
			"		<Param name=\"status\" default=\"1\"/>"
			"	</GroupParam>"
			"</Command>");
	}


	// 监听DI信号 //
	auto ListenDI::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void{}
	auto ListenDI::executeRT(PlanTarget &target)->int
		{
			if (is_automatic)
			{
				// 访问主站 //
				auto controller = dynamic_cast<aris::control::EthercatController*>(target.controller);
				static std::uint8_t di = 0x00;
				static std::int16_t di_delay[6] = { 0,0,0,0,0,0 };
				controller->slavePool().at(7).readPdo(0x6001, 0x01, di);
				//模拟DI信号为1//
				//di = 0x01;

				auto &lout = controller->lout();
				auto &cout = controller->mout();
				//di信号持续100ms才会有效，其他情况会将di信号全部置为0//
				switch (di)
				{
				case 0x00:
				{
					which_di = 0;
					for (Size i = 0; i < 6; i++)
					{
						di_delay[i] = 0;
					}
					break;
				}
				case 0x01:
				{
					for (Size i = 0; i < 6; i++)
					{
						if (i == 0)
						{
							di_delay[i] = di_delay[i] + 1;
						}
						else
						{
							di_delay[i] = 0;
						}
					}
					if (di_delay[0] >= 100)
					{
						which_di = 1;
						di_delay[0] = 0;
					}
					break;
				}
				case 0x02:
				{
					for (Size i = 0; i < 6; i++)
					{
						if (i == 1)
						{
							di_delay[i] = di_delay[i] + 1;
						}
						else
						{
							di_delay[i] = 0;
						}
					}
					if (di_delay[1] >= 100)
					{
						which_di = 2;
						di_delay[1] = 0;
					}
					break;
				}
				case 0x04:
				{
					for (Size i = 0; i < 6; i++)
					{
						if (i == 2)
						{
							di_delay[i] = di_delay[i] + 1;
						}
						else
						{
							di_delay[i] = 0;
						}
					}
					if (di_delay[2] >= 100)
					{
						which_di = 3;
						di_delay[2] = 0;
					}
					break;
				}
				case 0x08:
				{
					for (Size i = 0; i < 6; i++)
					{
						if (i == 3)
						{
							di_delay[i] = di_delay[i] + 1;
						}
						else
						{
							di_delay[i] = 0;
						}
					}
					if (di_delay[3] >= 100)
					{
						which_di = 4;
						di_delay[3] = 0;
					}
					break;
				}
				case 0x10:
				{
					for (Size i = 0; i < 6; i++)
					{
						if (i == 4)
						{
							di_delay[i] = di_delay[i] + 1;
						}
						else
						{
							di_delay[i] = 0;
						}
					}
					if (di_delay[4] >= 100)
					{
						which_di = 5;
						di_delay[4] = 0;
					}
					break;
				}
				case 0x20:
				{
					for (Size i = 0; i < 6; i++)
					{
						if (i == 5)
						{
							di_delay[i] = di_delay[i] + 1;
						}
						else
						{
							di_delay[i] = 0;
						}
					}
					if (di_delay[5] >= 100)
					{
						which_di = 6;
						di_delay[5] = 0;
					}
					break;
				}
				default:
				{
					which_di = 0;
					for (Size i = 0; i < 6; i++)
					{
						di_delay[i] = 0;
					}
					lout << "unexpected di:" << di << std::endl;
				}
				}
				if (which_di == 0)
				{
					return 1;
				}
				else
				{
					return 0;
				}
				
			}
			else
			{
				return 0;
			}
			
		}
	ListenDI::ListenDI(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"listenDI\">"
			"</Command>");
	}
	

	// 电缸余弦往复轨迹 //
	struct MoveEAParam
	{
		double s;
		double time;
	};
	auto MoveEA::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
		{
			MoveEAParam param = { 0.0,0.0 };
			for (auto &p : params)
			{
				if (p.first == "s")
				{
					param.s = std::stod(p.second);
				}
				else if (p.first == "time")
				{
					param.time = std::stod(p.second);
				}
			}
			target.param = param;

			std::fill(target.mot_options.begin(), target.mot_options.end(),
				Plan::USE_TARGET_POS);

		}
	auto MoveEA::executeRT(PlanTarget &target)->int
		{
			auto &param = std::any_cast<MoveEAParam&>(target.param);

			auto time = static_cast<int>(param.time * 1000);
			static double begin_p;

			// 访问主站 //
			auto controller = target.controller;

			static double median_filter[MEDIAN_LENGTH] = { 0.0 };

			if (target.count == 1)
			{
				begin_p = controller->motionAtAbs(6).targetPos();
				fore_vel.assign(FORE_VEL_LENGTH + 1, controller->motionAtAbs(6).actualVel());
			}
			double end_p;
			end_p = begin_p + param.s*(1 - std::cos(2 * PI*target.count / time)) / 2;
			controller->motionAtAbs(6).setTargetPos(end_p);
			
			//根据电流值换算压力值//
			int phase;	//标记采用那一段公式计算压力值//
			double force = 0, ff = 0, fc = controller->motionAtAbs(6).actualCur() * ea_index, fg = ea_gra, fs = std::abs(ea_c * ea_index);
			if (std::abs(controller->motionAtAbs(6).actualVel()) > 0.001)
			{
				if (controller->motionAtAbs(6).actualVel() > 0)
				{
					ff = (-ea_a * controller->motionAtAbs(6).actualVel()*controller->motionAtAbs(6).actualVel() + ea_b * controller->motionAtAbs(6).actualVel() + ea_c) * ea_index;
					force = ff + fg + fc;
					phase = 1;
				}
				else
				{
					ff = (ea_a * controller->motionAtAbs(6).actualVel()*controller->motionAtAbs(6).actualVel() + ea_b * controller->motionAtAbs(6).actualVel() - ea_c) * ea_index;
					force = ff + fg + fc;
					phase = 2;
				}
			}
			else
			{
				if (std::abs(fc + fg) <= fs)
				{
					force = 0;
					phase = 3;
				}
				else
				{
					if (fc + fg < -fs)
					{
						force = fc + fg + fs;
						phase = 4;
					}
					else
					{
						force = fc + fg - fs;;
						phase = 5;
					}
				}
			}

			//对速度进行均值滤波, 对摩擦力进行滤波//
			double mean_vel, fe, filteredforce;

			for (Size i = 0; i < FORE_VEL_LENGTH; i++)
			{
				fore_vel[i] = fore_vel[i + 1];
			}
			fore_vel[FORE_VEL_LENGTH] = controller->motionAtAbs(6).actualVel();
			if (target.count < 21)
			{
				mean_vel = (fore_vel.back() - fore_vel.front()) * 1000 / target.count;
				filteredforce = iir.filter(force);
				tempforce = tempforce + force;
				fe = -tempforce / target.count + 1810 * mean_vel;
			}
			else
			{
				mean_vel = (fore_vel.back() - fore_vel.front()) * 1000 / FORE_VEL_LENGTH;
				filteredforce = iir.filter(force);
				fe = -filteredforce + 1810 * mean_vel;
			}

			//中值滤波//
			for (Size i = 0; i < MEDIAN_LENGTH - 1; i++)
			{
				median_filter[i] = median_filter[i + 1];
			}

			median_filter[MEDIAN_LENGTH - 1] = fe;

			double tem[MEDIAN_LENGTH];
			std::copy_n(median_filter, MEDIAN_LENGTH, tem);

			std::sort(tem, tem + MEDIAN_LENGTH);
			fe = tem[(MEDIAN_LENGTH - 1) / 2];

			//发送数据buffer//
			if (data_num >= buffer_length)
			{
				std::copy_n(&fce_data[4], buffer_length - 4, fce_data);
				fce_data[buffer_length - 4] = controller->motionAtAbs(6).actualPos();
				fce_data[buffer_length - 3] = controller->motionAtAbs(6).actualVel();
				fce_data[buffer_length - 2] = controller->motionAtAbs(6).actualCur();
				fce_data[buffer_length - 1] = fe;
				data_num = buffer_length;
			}
			else
			{
				fce_data[data_num++] = controller->motionAtAbs(6).actualPos();
				fce_data[data_num++] = controller->motionAtAbs(6).actualVel();
				fce_data[data_num++] = controller->motionAtAbs(6).actualCur();
				fce_data[data_num++] = fe;
			}

			// 打印 目标位置、实际位置、实际速度、实际电流、压力 //
			auto &cout = controller->mout();
			if (target.count % 100 == 0)
			{
				cout << controller->motionAtAbs(6).targetPos() << "  " << controller->motionAtAbs(6).actualPos() << "  " << controller->motionAtAbs(6).actualVel() << "  " << controller->motionAtAbs(6).actualCur() << "  " << force << "  " << phase << "  " << fe << std::endl;
			}
			// log 目标位置、实际位置、实际速度、实际电流、压力 //
			auto &lout = controller->lout();
			lout << controller->motionAtAbs(6).targetPos() << "  " << controller->motionAtAbs(6).actualPos() << "  " << controller->motionAtAbs(6).actualVel() << "  " << controller->motionAtAbs(6).actualCur() << "  " << force << "  " << filteredforce << "  " << phase << "  " << fe << std::endl;

			return time - target.count;
		}
	auto MoveEA::collectNrt(PlanTarget &target)->void {}
	MoveEA::MoveEA(const std::string &name):Plan(name), fore_vel(FORE_VEL_LENGTH + 1), tempforce(0)
	{
		command().loadXmlStr(
			"<Command name=\"moveEA\">"
			"	<GroupParam>"
			"		<Param name=\"s\" default=\"0.1\"/>"
			"		<Param name=\"time\" default=\"1.0\" abbreviation=\"t\"/>"
			"	</GroupParam>"
			"</Command>");

		std::vector<double> num_data(IIR_FILTER::num, IIR_FILTER::num + 20);
		std::vector<double> den_data(IIR_FILTER::den, IIR_FILTER::den + 20);
		iir.setPara(num_data, den_data);

	}


	// 电缸运动轨迹；速度前馈 //
	struct MoveEAPParam
	{
		double begin_pos, pos, vel, acc, dec;
		bool ab;
	};
	auto MoveEAP::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
		{
			MoveEAPParam param = {0.0, 0.0, 0.0, 0.0, 0.0, 0};
			for (auto &p : params)
			{
				if (p.first == "begin_pos")
				{
					param.begin_pos = std::stod(p.second);
				}
				else if (p.first == "pos")
				{
					param.pos = std::stod(p.second);
				}
				else if (p.first == "vel")
				{
					param.vel = std::stod(p.second);
				}
				else if (p.first == "acc")
				{
					param.acc = std::stod(p.second);
				}
				else if (p.first == "dec")
				{
					param.dec = std::stod(p.second);
				}
				else if (p.first == "ab")
				{
					param.ab = std::stod(p.second);
				}
			}
			target.param = param;

			std::fill(target.mot_options.begin(), target.mot_options.end(),
				Plan::USE_TARGET_POS |
				Plan::USE_VEL_OFFSET);

		}
	auto MoveEAP::executeRT(PlanTarget &target)->int
		{
			auto &param = std::any_cast<MoveEAPParam&>(target.param);
			// 访问主站 //
			auto controller = target.controller;

			static double median_filter[MEDIAN_LENGTH] = {0.0};
			if (target.count == 1)
			{
				param.begin_pos = controller->motionAtAbs(6).targetPos();
				fore_vel.assign(FORE_VEL_LENGTH + 1, controller->motionAtAbs(6).actualVel());
				
				/*iir.m_px.assign(iir.m_num_order, 0.0);
				iir.m_py.assign(iir.m_den_order, 0.0);*/
				//摩擦力滤波器初始化//		
			}
			aris::Size total_count{ 1 };
			double p, v, a;
			aris::Size t_count;
			//绝对轨迹//
			if(param.ab)
			{ 
				aris::plan::moveAbsolute(target.count, param.begin_pos, param.pos, param.vel / 1000, param.acc / 1000 / 1000, param.dec / 1000 / 1000, p, v, a, t_count);
			}
			//相对轨迹//
			else
			{
				aris::plan::moveAbsolute(target.count, param.begin_pos, param.begin_pos + param.pos, param.vel / 1000, param.acc / 1000 / 1000, param.dec / 1000 / 1000, p, v, a, t_count);
			}
			controller->motionAtAbs(6).setTargetPos(p);
			//速度前馈//
			controller->motionAtAbs(6).setOffsetVel(v*1000);

			total_count = std::max(total_count, t_count);

			//根据电流值换算压力值//
			int phase;	//标记采用那一段公式计算压力值//
			double fore_cur = 0, force = 0, ff = 0, fc, fg, fs;
			fc = controller->motionAtAbs(6).actualCur() * ea_index;
			fg = ea_gra;
			fs = std::abs(ea_c * ea_index);
			if(std::abs(controller->motionAtAbs(6).actualVel())>0.001)
			{
				if (controller->motionAtAbs(6).actualVel() > 0)
				{
					ff = (-ea_a * controller->motionAtAbs(6).actualVel()*controller->motionAtAbs(6).actualVel() + ea_b * controller->motionAtAbs(6).actualVel() + ea_c) * ea_index;
					force = ff + fg + fc;
					phase = 1;
					fore_cur = (1810 * a * 1000 * 1000 - ff - fg)/ ea_index;
				}
				else
				{
					ff = (ea_a * controller->motionAtAbs(6).actualVel()*controller->motionAtAbs(6).actualVel() + ea_b * controller->motionAtAbs(6).actualVel() - ea_c) * ea_index;
					force = ff + fg + fc;
					phase = 2;
					fore_cur = (1810 * a * 1000 * 1000 - ff - fg)/ ea_index;
				}

			}
			else
			{
				if (std::abs(fc + fg) <= fs)
				{
					force = 0;
					phase = 3;
					fore_cur = 0.0;
				}
				else
				{
					if (fc + fg < -fs)
					{
						force = fc + fg + fs;
						phase = 4;
						fore_cur = (1810 * a * 1000 * 1000 - fg - fs)/ ea_index;
					}
					else
					{
						force = fc + fg - fs;;
						phase = 5;
						fore_cur = (1810 * a * 1000 * 1000 - fg + fs)/ ea_index;;
					}
				}
			}

			//电流前馈//
			controller->motionAtAbs(6).setOffsetCur(fore_cur);

			//对速度进行均值滤波, 对摩擦力进行滤波//
			double mean_vel, fe, filteredforce;
			
			for(Size i=0;i< FORE_VEL_LENGTH;i++)
			{
				fore_vel[i] = fore_vel[i+1];
			}
			fore_vel[FORE_VEL_LENGTH] = controller->motionAtAbs(6).actualVel();
			if (target.count < 21)
			{
				mean_vel = (fore_vel.back() - fore_vel.front()) * 1000 / target.count;
				filteredforce = iir.filter(force);
				tempforce = tempforce + force;
				fe = - tempforce/target.count + 1810 * mean_vel;
			}
			else
			{
				mean_vel = (fore_vel.back() - fore_vel.front()) * 1000 / FORE_VEL_LENGTH;
				filteredforce = iir.filter(force);
				fe = -filteredforce + 1810 * mean_vel;
			}

			//中值滤波//
			for (Size i = 0; i < MEDIAN_LENGTH-1; i++)
			{
				median_filter[i] = median_filter[i + 1];
			}

			median_filter[MEDIAN_LENGTH - 1] = fe;

			double tem[MEDIAN_LENGTH];
			std::copy_n(median_filter, MEDIAN_LENGTH, tem);

			std::sort(tem, tem + MEDIAN_LENGTH);
			fe = tem[(MEDIAN_LENGTH-1)/2];
			
			//发送数据buffer//
			if (data_num >= buffer_length)
			{
				std::copy_n(&fce_data[4], buffer_length-4, fce_data);
				fce_data[buffer_length-4] = controller->motionAtAbs(6).actualPos();
				fce_data[buffer_length-3] = controller->motionAtAbs(6).actualVel();
				fce_data[buffer_length-2] = controller->motionAtAbs(6).actualCur();
				fce_data[buffer_length-1] = fe;
				data_num = buffer_length;
			}
			else
			{
				fce_data[data_num++] = controller->motionAtAbs(6).actualPos();
				fce_data[data_num++] = controller->motionAtAbs(6).actualVel();
				fce_data[data_num++] = controller->motionAtAbs(6).actualCur();
				fce_data[data_num++] = fe;
			}

			// 打印 目标位置、实际位置、实际速度、实际电流、压力 //
			auto &cout = controller->mout();
			if (target.count % 100 == 0)
			{
				cout << controller->motionAtAbs(6).targetPos() << "  " << controller->motionAtAbs(6).actualPos() << "  " << v << "  " << controller->motionAtAbs(6).actualVel() << "  "
					<< a << "  " << fore_cur << "  " << controller->motionAtAbs(6).actualCur() << "  " << ff << "  " << fg << "  " << fc << "  " << force << "  " << filteredforce << "  " << phase << "  " << fe << std::endl;
			}
			// log 目标位置、实际位置、实际速度、实际电流、压力 //
			auto &lout = controller->lout();
			lout << controller->motionAtAbs(6).targetPos() << "  " << controller->motionAtAbs(6).actualPos() << "  " << v << "  " << controller->motionAtAbs(6).actualVel() << "  " 
				<< a << "  " << fore_cur << "  " << controller->motionAtAbs(6).actualCur() << "  " << ff << "  " << fg << "  " << fc << "  " << force << "  " << filteredforce << "  " << phase << "  " << fe << std::endl;

			return total_count - target.count;
		}
	auto MoveEAP::collectNrt(PlanTarget &target)->void {}
	MoveEAP::MoveEAP(const std::string &name) :Plan(name), fore_vel(FORE_VEL_LENGTH + 1), tempforce(0)
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

		std::vector<double> num_data(IIR_FILTER::num, IIR_FILTER::num + 20);
		std::vector<double> den_data(IIR_FILTER::den, IIR_FILTER::den + 20);
		iir.setPara(num_data, den_data);
			
	}


    // 力传感器信号测试 //
    struct FSParam
    {
        bool real_data;
        int time;
        uint16_t datanum;
        float Fx,Fy,Fz,Mx,My,Mz;
    };
    auto FSSignal::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
        {
            FSParam param;
            for (auto &p : params)
            {
                if (p.first == "real_data")
                {
                    param.real_data = std::stod(p.second);
                }
                else if (p.first == "time")
                {
                    param.time = std::stoi(p.second);
                }
            }

            param.Fx = 0.0;
            param.Fy = 0.0;
            param.Fz = 0.0;
            param.Mx = 0.0;
            param.My = 0.0;
            param.Mz = 0.0;
            target.param = param;

#ifdef WIN32
            target.option |=

                Plan::NOT_CHECK_POS_MIN |
                Plan::NOT_CHECK_POS_MAX |
                Plan::NOT_CHECK_POS_CONTINUOUS |
                Plan::NOT_CHECK_POS_CONTINUOUS_AT_START |
                Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
                Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START |
                Plan::NOT_CHECK_POS_FOLLOWING_ERROR |
                Plan::NOT_CHECK_VEL_MIN |
                Plan::NOT_CHECK_VEL_MAX |
                Plan::NOT_CHECK_VEL_CONTINUOUS |
                Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START |
                Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;
#endif
        }
    auto FSSignal::executeRT(PlanTarget &target)->int
        {
            auto &param = std::any_cast<FSParam&>(target.param);
            // 访问主站 //
            auto controller = dynamic_cast<aris::control::EthercatController*>(target.controller);
            if (param.real_data)
            {
                controller->slavePool().at(6).readPdo(0x6030, 0x00, &param.datanum ,16);
                controller->slavePool().at(6).readPdo(0x6030, 0x01, &param.Fx ,32);
                controller->slavePool().at(6).readPdo(0x6030, 0x02, &param.Fy, 32);
                controller->slavePool().at(6).readPdo(0x6030, 0x03, &param.Fz, 32);
                controller->slavePool().at(6).readPdo(0x6030, 0x04, &param.Mx, 32);
                controller->slavePool().at(6).readPdo(0x6030, 0x05, &param.My, 32);
                controller->slavePool().at(6).readPdo(0x6030, 0x06, &param.Mz, 32);
            }
            else
            {
                controller->slavePool().at(6).readPdo(0x6030, 0x00, &param.datanum ,16);
                controller->slavePool().at(6).readPdo(0x6020, 0x01, &param.Fx, 32);
                controller->slavePool().at(6).readPdo(0x6020, 0x02, &param.Fy, 32);
                controller->slavePool().at(6).readPdo(0x6020, 0x03, &param.Fz, 32);
                controller->slavePool().at(6).readPdo(0x6020, 0x04, &param.Mx, 32);
                controller->slavePool().at(6).readPdo(0x6020, 0x05, &param.My, 32);
                controller->slavePool().at(6).readPdo(0x6020, 0x06, &param.Mz, 32);
            }

            //print//
            auto &cout = controller->mout();
            if (target.count % 100 == 0)
            {
                cout << std::setw(6) << param.datanum << "  ";
                cout << std::setw(6) << param.Fx << "  ";
                cout << std::setw(6) << param.Fy << "  ";
                cout << std::setw(6) << param.Fz << "  ";
                cout << std::setw(6) << param.Mx << "  ";
                cout << std::setw(6) << param.My << "  ";
                cout << std::setw(6) << param.Mz << "  ";
                cout << std::endl;
                cout << "----------------------------------------------------" << std::endl;
            }

            //log//
            auto &lout = controller->lout();
            {
                lout << param.Fx << " ";
                lout << param.Fy << " ";
                lout << param.Fz << " ";
                lout << param.Mx << " ";
                lout << param.My << " ";
                lout << param.Mz << " ";
                lout << std::endl;
            }
            param.time--;
            return param.time;
        }
    auto FSSignal::collectNrt(PlanTarget &target)->void {}
    FSSignal::FSSignal(const std::string &name) :Plan(name)
    {
        command().loadXmlStr(
            "<Command name=\"fssignal\">"
            "	<GroupParam>"
            "		<Param name=\"real_data\" default=\"1\"/>"
            "		<Param name=\"time\" default=\"100000\"/>"
            "	</GroupParam>"
            "</Command>");
    }


    // ATI Force Sensor //
    struct ATIFSParam
    {
        int time;
        std::int32_t Fx,Fy,Fz,Mx,My,Mz,status_code,sample_counter,controlcodes;
        std::int32_t forceindex, torqueindex;
        std::uint8_t forceunit, torqueunit;
    };
    auto ATIFS::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
        {
            auto controller = dynamic_cast<aris::control::EthercatController*>(target.controller);
            ATIFSParam param;
            param.Fx = 0;
            param.Fy = 0;
            param.Fz = 0;
            param.Mx = 0;
            param.My = 0;
            param.Mz = 0;
            param.status_code = 0;
            param.sample_counter = 0;
            param.controlcodes = 4096;
            //param.controlcodes = 4352;//little range

            for (auto &p : params)
            {
                if (p.first == "time")
                {
                    param.time = std::stoi(p.second);
                }
                else if (p.first == "controlcodes")
                {
                    param.controlcodes = std::stoi(p.second);
                }
            }

            /*
            controller->slavePool().at(6).readSdo(0x2021, 0x2f, &param.forceunit, 8);
            controller->slavePool().at(6).readSdo(0x2021, 0x30, &param.torqueunit, 8);
            controller->slavePool().at(6).readSdo(0x2021, 0x37, &param.forceindex, 32);
            controller->slavePool().at(6).readSdo(0x2021, 0x38, &param.torqueindex, 32);
            */

            target.param = param;
            std::fill(target.mot_options.begin(), target.mot_options.end(),
                Plan::NOT_CHECK_ENABLE);

        }
    auto ATIFS::executeRT(PlanTarget &target)->int
    {
        auto &param = std::any_cast<ATIFSParam&>(target.param);
        // 访问主站 //

        auto controller = dynamic_cast<aris::control::EthercatController*>(target.controller);
        if (target.count == 1)
        {
            controller->slavePool().at(6).writePdo(0x7010, 0x01, &param.controlcodes, 32);
        }
        controller->slavePool().at(6).readPdo(0x6000, 0x01, &param.Fx ,32);
        controller->slavePool().at(6).readPdo(0x6000, 0x02, &param.Fy, 32);
        controller->slavePool().at(6).readPdo(0x6000, 0x03, &param.Fz, 32);
        controller->slavePool().at(6).readPdo(0x6000, 0x04, &param.Mx, 32);
        controller->slavePool().at(6).readPdo(0x6000, 0x05, &param.My, 32);
        controller->slavePool().at(6).readPdo(0x6000, 0x06, &param.Mz, 32);
        controller->slavePool().at(6).readPdo(0x6010, 0x00, &param.status_code, 32);
        controller->slavePool().at(6).readPdo(0x6020, 0x00, &param.sample_counter, 32);

        /*
        //print//
        auto &cout = controller->mout();
        if (target.count % 100 == 0)
        {
            cout << std::setw(6) << param.Fx << "  ";
            cout << std::setw(6) << param.Fy << "  ";
            cout << std::setw(6) << param.Fz << "  ";
            cout << std::setw(6) << param.Mx << "  ";
            cout << std::setw(6) << param.My << "  ";
            cout << std::setw(6) << param.Mz << "  ";
            cout << std::setw(6) << param.status_code << "  ";
            cout << std::setw(6) << param.sample_counter << "  ";
            cout << std::setw(6) << param.forceunit << "  ";
            cout << std::setw(6) << param.torqueunit<< "  ";
            cout << std::setw(6) << param.forceindex << "  ";
            cout << std::setw(6) << param.forceindex << "  ";
            cout << std::endl;
            cout << "----------------------------------------------------" << std::endl;
        }
        */


        double Fx = param.Fx/1000000.0;
        double Fy = param.Fy/1000000.0;
        double Fz = param.Fz/1000000.0;
        double Mx = param.Mx/1000000.0;
        double My = param.My/1000000.0;
        double Mz = param.Mz/1000000.0;


        //print//
        auto &cout = controller->mout();
        if (target.count % 100 == 0)
        {
            cout << std::setw(6) << Fx << "  ";
            cout << std::setw(6) << Fy << "  ";
            cout << std::setw(6) << Fz << "  ";
            cout << std::setw(6) << Mx << "  ";
            cout << std::setw(6) << My << "  ";
            cout << std::setw(6) << Mz << "  ";
            cout << std::setw(6) << param.status_code << "  ";
            cout << std::setw(6) << param.sample_counter << "  ";
            cout << std::endl;
            cout << "----------------------------------------------------" << std::endl;
        }

        //log//
        auto &lout = controller->lout();
        {
            lout << Fx << " ";
            lout << Fy << " ";
            lout << Fz << " ";
            lout << Mx << " ";
            lout << My << " ";
            lout << Mz << " ";
            lout << param.status_code << " ";
            lout << param.sample_counter << " ";
            lout << std::endl;
        }

        param.time--;
        return param.time;
    }
    auto ATIFS::collectNrt(PlanTarget &target)->void {}
    ATIFS::ATIFS(const std::string &name) :Plan(name)
    {
        command().loadXmlStr(
            "<Command name=\"atifs\">"
            "	<GroupParam>"
            "		<Param name=\"time\" default=\"100000\"/>"
            "		<Param name=\"controlcodes\" default=\"4096\"/>"
            "	</GroupParam>"
            "</Command>");
    }


	// 清理slavepool //
	auto ClearCon::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		auto&cs = aris::server::ControlServer::instance();
		if (cs.running())throw std::runtime_error("cs is running, can not set position offset,please stop the cs!");

		auto &controller = target.controller;
		controller->slavePool().clear();

		std::string ret = "ok";
		target.ret = ret;
		target.option = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;
	}
	ClearCon::ClearCon(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"clearCon\">"
			"</Command>");
	}


	// 配置控制器参数 //
	struct SetConParam
	{
		uint16_t motion_phyid;
		uint16_t ecslave_phyid;
		uint16_t ecslave_dc;
		bool is_motion;
	};
	auto SetCon::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		auto&cs = aris::server::ControlServer::instance();
		if (cs.running())throw std::runtime_error("cs is running, can not set controller,please stop the cs!");

		SetConParam param;
		param.motion_phyid = 0;
		param.ecslave_phyid = 0;
		param.is_motion = 1;
		for (auto &p : params)
		{
			//motion physical id number//
			if (p.first == "motion_phyid")
			{
				param.motion_phyid = std::stoi(p.second);
				param.is_motion = 1;
			}
			//ethercat slave physical id number//
			else if (p.first == "ecslave_phyid")
			{
				param.ecslave_phyid = std::stoi(p.second);
				param.is_motion = 0;
				param.ecslave_dc = std::stoi(params.at("ecslave_dc"));
			}
		}

		//std::unique_ptr<aris::control::Controller> controller(aris::robot::createControllerRokaeXB4());
		//controller->slavePool().clear();	//清除slavePool中的元素，后面重新添加
		auto &controller = target.controller;

		//configure motion//
		if(param.is_motion)
		{
			controller->slavePool().add<aris::control::EthercatMotion>();
			controller->slavePool().back().setPhyId(param.motion_phyid);
			dynamic_cast<aris::control::EthercatMotion&>(controller->slavePool().back()).scanInfoForCurrentSlave();
			dynamic_cast<aris::control::EthercatMotion&>(controller->slavePool().back()).scanPdoForCurrentSlave();
			dynamic_cast<aris::control::EthercatMotion&>(controller->slavePool().back()).setDcAssignActivate(0x300);
		}	
		//configure ect slave//
		else if(!param.is_motion)
		{
			controller->slavePool().add<aris::control::EthercatSlave>();
			controller->slavePool().back().setPhyId(param.ecslave_phyid);
			dynamic_cast<aris::control::EthercatSlave&>(controller->slavePool().back()).scanInfoForCurrentSlave();
			dynamic_cast<aris::control::EthercatSlave&>(controller->slavePool().back()).scanPdoForCurrentSlave();
			//倍福ECT Slave 耦合器,ecslave_dc=0x300；耦合器后面的模块，,ecslave_dc=0x00//
			dynamic_cast<aris::control::EthercatSlave&>(controller->slavePool().back()).setDcAssignActivate(param.ecslave_dc);
		}
		//cs.resetController(controller);

		//std::cout << controller->xmlString() << std::endl;
		/*
		auto xmlpath = std::filesystem::absolute(".");
		const std::string xmlfile = "kaanh.xml";
		xmlpath = xmlpath / xmlfile;
		cs.saveXmlFile(xmlpath.string().c_str());
		*/

		std::string ret = "ok";
		target.ret = ret;
		target.option = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;
	}
	SetCon::SetCon(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"setCon\">"
			"	<GroupParam>"
			"		<UniqueParam>"
			"			<Param name=\"motion_phyid\" default=\"0\"/>"
			"			<GroupParam>"
			"				<Param name=\"ecslave_phyid\" default=\"6\"/>"
			"				<Param name=\"ecslave_dc\" default=\"768\"/>"
			"			</GroupParam>"
			"		</UniqueParam>"
			"	</GroupParam>"
			"</Command>");
	}


	// 配置DH参数 //
	struct SetDHParam
	{
		std::vector<double>dh;
		double tool_offset;
		int axis_num;
	};
	auto SetDH::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		auto&cs = aris::server::ControlServer::instance();
		if (cs.running())throw std::runtime_error("cs is running, can not set DH parameters,please stop the cs!");

		SetDHParam dhparam;
		dhparam.dh.clear();

		for (auto &p : params)
		{
			//6轴DH参数//
			if (p.first == "six_axes")
			{
				dhparam.dh.resize(6, 0.0);
				dhparam.dh[0] = std::stod(params.at("d1_six_axes"));
				dhparam.dh[1] = std::stod(params.at("a1_six_axes"));
				dhparam.dh[2] = std::stod(params.at("a2_six_axes"));
				dhparam.dh[3] = std::stod(params.at("d3_six_axes"));
				dhparam.dh[4] = std::stod(params.at("a3_six_axes"));
				dhparam.dh[5] = std::stod(params.at("d4_six_axes"));
				dhparam.tool_offset = std::stod(params.at("tool0_six_axes"));
				dhparam.axis_num = 6;
			}
			//7轴DH参数//
			else if (p.first == "seven_axes")
			{
				dhparam.dh.resize(3, 0.0);
				dhparam.dh[0] = std::stod(params.at("d1_seven_axes"));
				dhparam.dh[1] = std::stod(params.at("d3_seven_axes"));
				dhparam.dh[2] = std::stod(params.at("d5_seven_axes"));
				dhparam.tool_offset = std::stod(params.at("tool0_seven_axes"));
				dhparam.axis_num = 7;
			}
		}

		aris::dynamic::PumaParam param_puma;
		aris::dynamic::SevenAxisParam param_7axes;
		if (dhparam.axis_num == 6)
		{
			param_puma.d1 = dhparam.dh[0];
			param_puma.a1 = dhparam.dh[1];
			param_puma.a2 = dhparam.dh[2];
			param_puma.d3 = dhparam.dh[3];
			param_puma.a3 = dhparam.dh[4];
			param_puma.d4 = dhparam.dh[5];
			param_puma.tool0_pe[2] = dhparam.tool_offset;

			auto model = aris::dynamic::createModelPuma(param_puma);
			cs.resetModel(model.release());
		}
		else if (dhparam.axis_num == 7)
		{
			param_7axes.d1 = dhparam.dh[0];
			param_7axes.d3 = dhparam.dh[1];
			param_7axes.d5 = dhparam.dh[2];
			param_7axes.tool0_pe[2] = dhparam.tool_offset;

			auto m = aris::dynamic::createModelSevenAxis(param_7axes);
			cs.resetModel(m.release());
		}
		else{ }

		/*
		//动力学标定参数//
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
		*/
		
		/*
		auto xmlpath = std::filesystem::absolute(".");
		const std::string xmlfile = "kaanh.xml";
		xmlpath = xmlpath / xmlfile;
		cs.saveXmlFile(xmlpath.string().c_str());
		*/

		std::string ret = "ok";
		target.ret = ret;
		target.option = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;

	}
	SetDH::SetDH(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"setDH\">"
			"	<GroupParam>"
			"		<UniqueParam>"
			"			<GroupParam name=\"start_group\">"
			"				<Param name=\"six_axes\"/>"
			"				<Param name=\"d1_six_axes\" default=\"0.3295\"/>"
			"				<Param name=\"a1_six_axes\" default=\"0.04\"/>"
			"				<Param name=\"a2_six_axes\" default=\"0.275\"/>"
			"				<Param name=\"d3_six_axes\" default=\"0.0\"/>"
			"				<Param name=\"a3_six_axes\" default=\"0.025\"/>"
			"				<Param name=\"d4_six_axes\" default=\"0.28\"/>"
			"				<Param name=\"tool0_six_axes\" default=\"0.078\"/>"
			"			</GroupParam>"
			"			<GroupParam>"
			"				<Param name=\"seven_axes\"/>"
			"				<Param name=\"d1_seven_axes\" default=\"0.3705\"/>"
			"				<Param name=\"d3_seven_axes\" default=\"0.330\"/>"
			"				<Param name=\"d5_seven_axes\" default=\"0.320\"/>"
			"				<Param name=\"tool0_seven_axes\" default=\"0.2205\"/>"
			"			</GroupParam>"
			"		</UniqueParam>"
			"	</GroupParam>"
			"</Command>");
	}

	
	// 配置PG参数 //
	struct SetPGParam
	{
		std::string name;
		std::vector<double> pe;
		uint16_t part_id;
		std::string file_path;	
	};
	auto SetPG::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		auto&cs = aris::server::ControlServer::instance();
		if (cs.running())throw std::runtime_error("cs is running, can not set DH parameters,please stop the cs!");

		SetPGParam param;
		param.pe.clear();
		param.pe.resize(6, 0.0);
		for (auto &p : params)
		{
			if (p.first == "name")
			{
				param.name = p.second;
			}
			else if (p.first == "pe")
			{
				auto mat = target.model->calculator().calculateExpression(params.at("pe"));
				if (mat.size() == param.pe.size())
				{
					param.pe.assign(mat.begin(), mat.end());
				}
				else
				{
					throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
				}
			}
			else if (p.first == "part_id")
			{
				param.part_id = std::stoi(p.second);
			}
			else if (p.first == "file_path")
			{
				param.file_path = p.second;
			}
		}

		target.model->partPool().at(param.part_id).geometryPool().clear();
		target.model->partPool().at(param.part_id).geometryPool().add<FileGeometry>(param.name, param.file_path, param.pe.data());

		std::string ret = "ok";
		target.ret = ret;
		target.option = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;

	}
	SetPG::SetPG(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"setPG\">"
			"	<GroupParam>"
			"		<Param name=\"name\" default=\"test\"/>"
			"		<Param name=\"pe\" default=\"{0, 0, 0, -0, 0, -0}\"/>"
			"		<Param name=\"part_id\" default=\"6\"/>"
			"		<Param name=\"file_path\" default=\"/RobotGallery/Rokae/XB4/l0.data\"/>"
			"	</GroupParam>"
			"</Command>");
	}
	

	// 配置UI //
	struct SetUIParam
	{
		std::string ui_path;
	};
	auto SetUI::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		auto&cs = aris::server::ControlServer::instance();
		if (cs.running())throw std::runtime_error("cs is running, can not set UI, please stop the cs!");

		SetUIParam param;

		for (auto &p : params)
		{
			if (p.first == "ui_path")
			{
				param.ui_path = p.second;
			}
		}
		
		auto xmlpath = std::filesystem::absolute(".");
		const std::string xmlfile = "kaanh.xml";
		auto xmlpath_ui = xmlpath / param.ui_path;	
		xmlpath = xmlpath / xmlfile;

		cs.interfaceRoot().loadXmlFile(xmlpath_ui.string().c_str());
		//cs.saveXmlFile(xmlpath.string().c_str());

		std::string ret = "ok";
		target.ret = ret;
		target.option = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;

	}
	SetUI::SetUI(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"setUI\">"
			"	<GroupParam>"
			"		<Param name=\"ui_path\" default=\"interface_kaanh.xml\"/>"
			"	</GroupParam>"
			"</Command>");
	}


	// 配置关节——home偏置、角度-编码系数、角度、角速度、角加速度上下限 //
	struct SetDriverParam
	{
		std::vector<bool> joint_active_vec;
		double pos_factor;
		double pos_max;
		double pos_min;
		double vel_max;
		double vel_min;
		double acc_max;
		double acc_min;
		double pos_offset;
	};
	auto SetDriver::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		auto&cs = aris::server::ControlServer::instance();
		if (cs.running())throw std::runtime_error("cs is running, can not set pos_factor,pos,vel and acc,please stop the cs!");
		//std::unique_ptr<aris::control::Controller> controller(kaanh::createControllerRokaeXB4());
		auto &controller = target.controller;
		SetDriverParam param;

		//initial//
		{
			param.joint_active_vec.clear();
			param.pos_factor = 0.0;
			param.pos_max = 0.0;
			param.pos_min = 0.0;
			param.vel_max = 0.0;
			param.vel_min = 0.0;
			param.acc_max = 0.0;
			param.acc_min = 0.0;
			param.pos_offset = 0.0;
		}

		for (auto &p : params)
		{
			if (p.first == "motion_id")
			{
				param.joint_active_vec.resize(50, false);
				param.joint_active_vec.at(std::stoi(p.second)) = true;
			}
			else if(p.first == "pos_factor")
			{
                auto m = target.model->calculator().calculateExpression(p.second);
                param.pos_factor = m.toDouble() / (2.0 * PI);
			}
			else if (p.first == "pos_max")
			{
				auto m = target.model->calculator().calculateExpression(p.second);
				param.pos_max = m.toDouble() * 2 * PI / 360;
			}
			else if (p.first == "pos_min")
			{
				auto m = target.model->calculator().calculateExpression(p.second);
				param.pos_min = m.toDouble() * 2 * PI / 360;
			}
			else if (p.first == "vel_max")
			{
				auto m = target.model->calculator().calculateExpression(p.second);
				param.vel_max = m.toDouble() * 2 * PI / 360;
			}
			else if (p.first == "vel_min")
			{
				auto m = target.model->calculator().calculateExpression(p.second);
				param.vel_min = m.toDouble() * 2 * PI / 360;
			}
			else if (p.first == "acc_max")
			{
				auto m = target.model->calculator().calculateExpression(p.second);
				param.acc_max = m.toDouble() * 2 * PI / 360;
			}
			else if (p.first == "acc_min")
			{
				auto m = target.model->calculator().calculateExpression(p.second);
				param.acc_min = m.toDouble() * 2 * PI / 360;
			}
			else if (p.first == "pos_offset")
			{
				auto m = target.model->calculator().calculateExpression(p.second);
				param.pos_offset = m.toDouble();
			}
		}
		
		// 设置驱动pos_factor,pos,vel,acc //
		for (int i = 0; i < param.joint_active_vec.size(); i++)
		{
			if (param.joint_active_vec[i])
			{
				dynamic_cast<aris::control::Motion&>(controller->slavePool()[i]).setPosFactor(param.pos_factor);
				dynamic_cast<aris::control::Motion&>(controller->slavePool()[i]).setMaxPos(param.pos_max);
				dynamic_cast<aris::control::Motion&>(controller->slavePool()[i]).setMinPos(param.pos_min);
				dynamic_cast<aris::control::Motion&>(controller->slavePool()[i]).setMaxVel(param.vel_max);
				dynamic_cast<aris::control::Motion&>(controller->slavePool()[i]).setMinVel(param.vel_min);
				dynamic_cast<aris::control::Motion&>(controller->slavePool()[i]).setMaxAcc(param.acc_max);
				dynamic_cast<aris::control::Motion&>(controller->slavePool()[i]).setMinAcc(param.acc_min);
				dynamic_cast<aris::control::Motion&>(controller->slavePool()[i]).setPosOffset(param.pos_offset);
			}
		}

        std::cout << param.pos_factor << std::endl;
		//cs.resetController(controller);
		/*
		auto xmlpath = std::filesystem::absolute(".");
		const std::string xmlfile = "kaanh.xml";
		xmlpath = xmlpath / xmlfile;
		cs.saveXmlFile(xmlpath.string().c_str());
		*/
		std::string ret = "ok";
		target.ret = ret;
		target.option = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;
	}
	SetDriver::SetDriver(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"setDriver\">"
			"	<GroupParam>"
			"		<Param name=\"motion_id\" abbreviation=\"m\" default=\"0\"/>"
			"		<Param name=\"pos_factor\" default=\"0.0\"/>"
			"		<Param name=\"pos_max\" default=\"0.0\"/>"
			"		<Param name=\"pos_min\" default=\"0.0\"/>"
			"		<Param name=\"vel_max\" default=\"0.0\"/>"
			"		<Param name=\"vel_min\" default=\"0.0\"/>"
			"		<Param name=\"acc_max\" default=\"0.0\"/>"
			"		<Param name=\"acc_min\" default=\"0.0\"/>"
			"		<Param name=\"pos_offset\" default=\"0.0\"/>"
			"	</GroupParam>"
			"</Command>");
	}


	// 保存配置 //
	auto SaveConfig::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		auto&cs = aris::server::ControlServer::instance();
		if (cs.running())throw std::runtime_error("cs is running, can not set position offset,please stop the cs!");

        //cs.resetSensorRoot(new aris::sensor::SensorRoot);

		auto xmlpath = std::filesystem::absolute(".");
		const std::string xmlfile = "kaanh.xml";
		xmlpath = xmlpath / xmlfile;
		cs.saveXmlFile(xmlpath.string().c_str());

		std::string ret = "ok";
		target.ret = ret;
		target.option = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;
	}
	SaveConfig::SaveConfig(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"saveConfig\">"
			"</Command>");
	}


	// 开启controller server //
	auto StartCS::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
        std::cout<<0<<std::endl;
		auto&cs = aris::server::ControlServer::instance();
		if (cs.running())throw std::runtime_error("cs is already running!");
        std::cout<<1<<std::endl;
        cs.start();
		
        std::cout<<2<<std::endl;
		std::string ret = "ok";
		target.ret = ret;
		target.option = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;
	}
	StartCS::StartCS(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"startCS\">"
			"</Command>");
	}


	// update controller //
	auto Update::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		system("bash /home/kaanh/Document/kaanh/update_arislib.sh");
		std::string ret = "ok";
		target.ret = ret;
		target.option = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;
	}
	Update::Update(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"update_controller\">"
			"</Command>");
	}


	// 关闭controller server //
	auto StopCS::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		auto&cs = aris::server::ControlServer::instance();
		if (!cs.running())throw std::runtime_error("cs is already stopping!");
		cs.stop();
		std::string ret = "ok";
		target.ret = ret;
		target.option = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;
	}
	StopCS::StopCS(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"stopCS\">"
			"</Command>");
	}

	
	// set cycle_time for driver with SDO //
	struct SetCTParam
	{
		int16_t cycle_time;
	};
	auto SetCT::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		SetCTParam param;
		param.cycle_time = 1;
		auto controller = dynamic_cast<aris::control::EthercatController*>(target.controller);

		for (auto &p : params)
		{
			if (p.first == "ct")
			{
				param.cycle_time = std::stoi(p.second);
			}
		}

		for (aris::Size i = 0; i < controller->slavePool().size(); i++)
		{
			controller->slavePool().at(i).writeSdo(0x60c2, 0x01, &param.cycle_time);
		}
		
		std::string ret = "ok";
		target.ret = ret;
		target.option = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;
	}
	SetCT::SetCT(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"setCT\">"
			"	<GroupParam>"
			"		<Param name=\"ct\" abbreviation=\"t\" default=\"1\"/>"
			"	</GroupParam>"
			"</Command>");
	}
	

    auto createPlanRootRokaeXB4()->std::unique_ptr<aris::plan::PlanRoot>
	{
        std::unique_ptr<aris::plan::PlanRoot> plan_root(new aris::plan::PlanRoot);

        plan_root->planPool().add<aris::plan::Enable>();
        plan_root->planPool().add<aris::plan::Disable>();
        plan_root->planPool().add<aris::plan::Home>();
        plan_root->planPool().add<aris::plan::Mode>();
        plan_root->planPool().add<aris::plan::Show>();
        plan_root->planPool().add<aris::plan::Sleep>();
        plan_root->planPool().add<aris::plan::Recover>();
        auto &rs = plan_root->planPool().add<aris::plan::Reset>();
        rs.command().findParam("pos")->setDefaultValue("{0.5,0.5,0.5,0.5,0.5,0.5}");

        plan_root->planPool().add<aris::plan::MoveAbsJ>();

        plan_root->planPool().add<aris::plan::MoveL>();
        plan_root->planPool().add<aris::plan::MoveJ>();

        plan_root->planPool().add<aris::plan::GetPartPq>();
        plan_root->planPool().add<aris::plan::GetXml>();
        plan_root->planPool().add<aris::plan::SetXml>();
        plan_root->planPool().add<aris::plan::Start>();
        plan_root->planPool().add<aris::plan::Stop>();

		plan_root->planPool().add<kaanh::Get>();
		plan_root->planPool().add<kaanh::MoveX>();
		plan_root->planPool().add<kaanh::MoveJS>();
		plan_root->planPool().add<kaanh::MoveJSN>();
		plan_root->planPool().add<kaanh::MoveJR>();
		plan_root->planPool().add<kaanh::MoveTTT>();
		plan_root->planPool().add<forcecontrol::MoveJRC>();
		plan_root->planPool().add<forcecontrol::MovePQCrash>();
		plan_root->planPool().add<forcecontrol::MovePQB>();
		plan_root->planPool().add<forcecontrol::MoveJCrash>();
		plan_root->planPool().add<forcecontrol::MoveJF>();
		plan_root->planPool().add<forcecontrol::MoveJFB>();
		plan_root->planPool().add<forcecontrol::MoveJPID>();
		plan_root->planPool().add<forcecontrol::MoveEAC>();
		plan_root->planPool().add<forcecontrol::MoveStop>();
		plan_root->planPool().add<forcecontrol::MoveSPQ>();
		plan_root->planPool().add<kaanh::MoveJM>();
		plan_root->planPool().add<kaanh::MoveJI>();
		plan_root->planPool().add<kaanh::JogC>();
		plan_root->planPool().add<kaanh::JogJ>();
		plan_root->planPool().add<kaanh::Grasp>();
		plan_root->planPool().add<kaanh::ListenDI>();
		plan_root->planPool().add<kaanh::MoveEA>();
		plan_root->planPool().add<kaanh::MoveEAP>();
		plan_root->planPool().add<kaanh::FSSignal>();
        plan_root->planPool().add<kaanh::ATIFS>();
		plan_root->planPool().add<kaanh::ClearCon>();
		plan_root->planPool().add<kaanh::SetCon>();
		plan_root->planPool().add<kaanh::SetDH>();
		plan_root->planPool().add<kaanh::SetPG>();
		plan_root->planPool().add<kaanh::SetUI>();
		plan_root->planPool().add<kaanh::SetDriver>();
		plan_root->planPool().add<kaanh::SaveConfig>();
		plan_root->planPool().add<kaanh::StartCS>();
		plan_root->planPool().add<kaanh::StopCS>();
		plan_root->planPool().add<kaanh::SetCT>();

		plan_root->planPool().add<MoveXYZ>();
		plan_root->planPool().add<MoveJoint>();
		plan_root->planPool().add<MoveDistal>();
        plan_root->planPool().add<DistalTest>();
		plan_root->planPool().add<SetTool>();
		plan_root->planPool().add<MovePressure>();
		plan_root->planPool().add<MoveFeed>();
		plan_root->planPool().add<MovePressureToolYZ>();
		plan_root->planPool().add<MovePressureToolXY>();
		plan_root->planPool().add<GetForce>();
        plan_root->planPool().add<MoveSeriesGK>();

		//plan_root->planPool().add<GetError>();
		plan_root->planPool().add<JointDyna>();
		plan_root->planPool().add<JointTest>();
		plan_root->planPool().add<DragTeach>();
		plan_root->planPool().add<LoadDyna>();
		plan_root->planPool().add<SaveYYbase>();
		plan_root->planPool().add<SaveFile>();

		plan_root->planPool().add<SevenJointDyna>();
		plan_root->planPool().add<SevenJointTest>();
        plan_root->planPool().add<SevenDragTeach>();
		plan_root->planPool().add<SevenLoadDyna>();

		plan_root->planPool().add<cplan::MoveCircle>();
		plan_root->planPool().add<cplan::MoveTroute>();
		plan_root->planPool().add<cplan::MoveFile>();
		plan_root->planPool().add<cplan::RemoveFile>();
		plan_root->planPool().add<cplan::MoveinModel>();
		plan_root->planPool().add<cplan::FMovePath>();
		plan_root->planPool().add<cplan::OpenFile>();

		return plan_root;
	}

}
