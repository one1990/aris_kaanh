#include <algorithm>
#include "kaanh.h"
#include "sixdistalfc.h"
#include "jointfc.h"
#include "sevenjointfc.h"
#include <array>
#include <stdlib.h>
#include"move_series.h"
#include <string>
#include"robotplan.h"
#include"planfuns.h"
#include <bitset>


using namespace aris::dynamic;
using namespace aris::plan;

//global vel//
extern kaanh::Speed g_vel;
extern std::atomic_int g_vel_percent;
//global vel//

//state machine flag//
extern std::atomic_bool g_is_enabled;
extern std::atomic_bool g_is_error;
extern std::atomic_bool g_is_manual;
extern std::atomic_bool g_is_auto;
//state machine flag//

kaanh::CmdListParam cmdparam;


namespace kaanh
{
	std::atomic_bool g_move_stop = false;
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
            "				<PdoEntry name=\"tor_actual_value\" index=\"0x6077\" subindex=\"0x00\" size=\"16\"/>"
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

        dynamic_cast<aris::control::Motion&>(controller->slavePool()[0]).setPosOffset(0.083326167813560906);
        dynamic_cast<aris::control::Motion&>(controller->slavePool()[1]).setPosOffset(0.40688722035956698);
        dynamic_cast<aris::control::Motion&>(controller->slavePool()[2]).setPosOffset(-0.063596878644675794);
        dynamic_cast<aris::control::Motion&>(controller->slavePool()[3]).setPosOffset(0.65575523199999997);
        dynamic_cast<aris::control::Motion&>(controller->slavePool()[4]).setPosOffset(-1.49538803280913);
        dynamic_cast<aris::control::Motion&>(controller->slavePool()[5]).setPosOffset(-1.76544794265974);
		
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

#ifdef WIN32
		double pos_offset = 0.0 / 8388608.0 * 250;	//在零位时，为500count
		double pos_factor = 8388608.0 * 250;	//运行1m需要转250转
		double max_pos = 10.0;
		double min_pos = 0.0;
		double max_vel = 1.0;	//1m/s
		double max_acc = 5.0;	//5m/s2

		std::string xml_str =
			"<EthercatMotion phy_id=\"" + std::to_string(6) + "\" product_code=\"0x6038000D\""
			" vendor_id=\"0x0000066F\" revision_num=\"0x00010000\" dc_assign_activate=\"0x0300\""
			" min_pos=\"" + std::to_string(min_pos) + "\" max_pos=\"" + std::to_string(max_pos) + "\" max_vel=\"" + std::to_string(max_vel) + "\" min_vel=\"" + std::to_string(-max_vel) + "\""
			" max_acc=\"" + std::to_string(max_acc) + "\" min_acc=\"" + std::to_string(-max_acc) + "\" max_pos_following_error=\"0.1\" max_vel_following_error=\"0.5\""
			" home_pos=\"0\" pos_factor=\"" + std::to_string(pos_factor) + "\" pos_offset=\"" + std::to_string(pos_offset) + "\">"
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
			"</EthercatMotion>";

		controller->slavePool().add<aris::control::EthercatMotion>().loadXmlStr(xml_str);
#endif // WIN32

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
					"<EthercatMotion phy_id=\"" + std::to_string(i) + "\" product_code=\"0x60380008\""
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
					"</EthercatMotion>";

				controller->slavePool().add<aris::control::EthercatMotion>().loadXmlStr(xml_str);
			}
			else if (i == 1)
			{
				std::string xml_str =
					"<EthercatMotion phy_id=\"" + std::to_string(i) + "\" product_code=\"0x60380009\""
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
					"</EthercatMotion>";

				controller->slavePool().add<aris::control::EthercatMotion>().loadXmlStr(xml_str);
			}
			else if (i == 2)
			{
				std::string xml_str =
					"<EthercatMotion phy_id=\"" + std::to_string(i) + "\" product_code=\"0x60380008\""
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
					"</EthercatMotion>";

				controller->slavePool().add<aris::control::EthercatMotion>().loadXmlStr(xml_str);
			}
			else
			{
				std::string xml_str =
					"<EthercatMotion phy_id=\"" + std::to_string(i) + "\" product_code=\"0x60380006\""
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
					"</EthercatMotion>";

				controller->slavePool().add<aris::control::EthercatMotion>().loadXmlStr(xml_str);
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

	//更新实时状态：使能、报错//
	auto update_state(aris::server::ControlServer &cs)->void
	{
		static bool motion_state[6] = { false, false, false, false, false, false };
		
		//获取motion的使能状态，0表示去使能状态，1表示使能状态//
		for (aris::Size i = 0; i < 6; i++)
		{
			auto cm = dynamic_cast<aris::control::EthercatMotion*>(&cs.controller().motionPool()[i]);
			if ((cm->statusWord() & 0x6f) != 0x27)
			{
				motion_state[i] = 0;
			}
			else
			{
				motion_state[i] = 1;
			}
			//motion_state[i] = 1;
		}
		
		//获取ret_code的值，判断是否报错，if条件可以初始化变量，并且取变量进行条件判断//
		if (auto target = cs.currentExecuteTargetRt(); target)
		{
			g_is_error.store(target->ret_code);
		}
		
		if (motion_state[0] && motion_state[1] && motion_state[2] && motion_state[3] && motion_state[4] && motion_state[5])
		{
			g_is_enabled.store(true);
		}
		else
		{
			g_is_enabled.store(false);
		}
		
	}

	//获取状态字——100:去使能,200:手动,300:准自动,400:自动,500:错误//
	auto get_state_code()->std::int32_t
	{
		if (g_is_enabled.load())
		{
			if (g_is_error.load())
			{
				return 500;
			}
			else
			{
				if (g_is_manual.load())
				{
					return 200;
				}
				else
				{
					if (!g_is_auto.load())
					{
						return 300;
					}
					else
					{
						return 400;
					}
				}
			}
		}
		else
		{
			return 100;
		}
	}


#define CHECK_PARAM_STRING \
		"		<UniqueParam default=\"check_all\">" \
		"			<Param name=\"check_all\"/>" \
		"			<Param name=\"check_none\"/>" \
		"			<GroupParam>"\
		"				<UniqueParam default=\"check_enable\">"\
		"					<Param name=\"check_enable\"/>"\
		"					<Param name=\"not_check_enable\"/>"\
		"				</UniqueParam>"\
		"				<UniqueParam default=\"check_pos\">"\
		"					<Param name=\"check_pos\"/>"\
		"					<Param name=\"not_check_pos\"/>"\
		"					<GroupParam>"\
		"						<UniqueParam default=\"check_pos_max\">"\
		"							<Param name=\"check_pos_max\"/>"\
		"							<Param name=\"not_check_pos_max\"/>"\
		"						</UniqueParam>"\
		"						<UniqueParam default=\"check_pos_min\">"\
		"							<Param name=\"check_pos_min\"/>"\
		"							<Param name=\"not_check_pos_min\"/>"\
		"						</UniqueParam>"\
		"						<UniqueParam default=\"check_pos_continuous\">"\
		"							<Param name=\"check_pos_continuous\"/>"\
		"							<Param name=\"not_check_pos_continuous\"/>"\
		"						</UniqueParam>"\
		"						<UniqueParam default=\"check_pos_continuous_second_order\">"\
		"							<Param name=\"check_pos_continuous_second_order\"/>"\
		"							<Param name=\"not_check_pos_continuous_second_order\"/>"\
		"						</UniqueParam>"\
		"						<UniqueParam default=\"check_pos_following_error\">"\
		"							<Param name=\"check_pos_following_error\"/>"\
		"							<Param name=\"not_check_pos_following_error\"/>"\
		"						</UniqueParam>"\
		"					</GroupParam>"\
		"				</UniqueParam>"\
		"				<UniqueParam default=\"check_vel\">"\
		"					<Param name=\"check_vel\"/>"\
		"					<Param name=\"not_check_vel\"/>"\
		"					<GroupParam>"\
		"						<UniqueParam default=\"check_vel_max\">"\
		"							<Param name=\"check_vel_max\"/>"\
		"							<Param name=\"not_check_vel_max\"/>"\
		"						</UniqueParam>"\
		"						<UniqueParam default=\"check_vel_min\">"\
		"							<Param name=\"check_vel_min\"/>"\
		"							<Param name=\"not_check_vel_min\"/>"\
		"						</UniqueParam>"\
		"						<UniqueParam default=\"check_vel_continuous\">"\
		"							<Param name=\"check_vel_continuous\"/>"\
		"							<Param name=\"not_check_vel_continuous\"/>"\
		"						</UniqueParam>"\
		"						<UniqueParam default=\"check_vel_following_error\">"\
		"							<Param name=\"check_vel_following_error\"/>"\
		"							<Param name=\"not_check_vel_following_error\"/>"\
		"						</UniqueParam>"\
		"					</GroupParam>"\
		"				</UniqueParam>"\
		"			</GroupParam>"\
		"		</UniqueParam>"
	auto set_check_option(const std::map<std::string, std::string> &cmd_params, PlanTarget &target)->void
	{
		for (auto cmd_param : cmd_params)
		{
			if (cmd_param.first == "check_all")
			{
				for (auto &option : target.mot_options)	option &= ~(
					Plan::NOT_CHECK_POS_MIN |
					Plan::NOT_CHECK_POS_MAX |
					Plan::NOT_CHECK_POS_CONTINUOUS |
					Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
					Plan::NOT_CHECK_POS_FOLLOWING_ERROR |
					Plan::NOT_CHECK_VEL_MIN |
					Plan::NOT_CHECK_VEL_MAX |
					Plan::NOT_CHECK_VEL_CONTINUOUS |
					Plan::NOT_CHECK_VEL_FOLLOWING_ERROR);
			}
			else if (cmd_param.first == "check_none")
			{
				for (auto &option : target.mot_options)	option |=
					Plan::NOT_CHECK_POS_MIN |
					Plan::NOT_CHECK_POS_MAX |
					Plan::NOT_CHECK_POS_CONTINUOUS |
					Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
					Plan::NOT_CHECK_POS_FOLLOWING_ERROR |
					Plan::NOT_CHECK_VEL_MIN |
					Plan::NOT_CHECK_VEL_MAX |
					Plan::NOT_CHECK_VEL_CONTINUOUS |
					Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;
			}
			else if (cmd_param.first == "check_enable")
			{
				for (auto &option : target.mot_options) option &= ~(
					Plan::NOT_CHECK_ENABLE);
			}
			else if (cmd_param.first == "not_check_enable")
			{
				for (auto &option : target.mot_options) option |=
					Plan::NOT_CHECK_ENABLE;
			}
			else if (cmd_param.first == "check_pos")
			{
				for (auto &option : target.mot_options) option &= ~(
					Plan::NOT_CHECK_POS_MIN |
					Plan::NOT_CHECK_POS_MAX |
					Plan::NOT_CHECK_POS_CONTINUOUS |
					Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
					Plan::NOT_CHECK_POS_FOLLOWING_ERROR);
			}
			else if (cmd_param.first == "not_check_pos")
			{
				for (auto &option : target.mot_options) option |=
					Plan::NOT_CHECK_POS_MIN |
					Plan::NOT_CHECK_POS_MAX |
					Plan::NOT_CHECK_POS_CONTINUOUS |
					Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
					Plan::NOT_CHECK_POS_FOLLOWING_ERROR;
			}
			else if (cmd_param.first == "check_vel")
			{
				for (auto &option : target.mot_options) option &= ~(
					Plan::NOT_CHECK_VEL_MIN |
					Plan::NOT_CHECK_VEL_MAX |
					Plan::NOT_CHECK_VEL_CONTINUOUS |
					Plan::NOT_CHECK_VEL_FOLLOWING_ERROR);
			}
			else if (cmd_param.first == "not_check_vel")
			{
				for (auto &option : target.mot_options) option |=
					Plan::NOT_CHECK_VEL_MIN |
					Plan::NOT_CHECK_VEL_MAX |
					Plan::NOT_CHECK_VEL_CONTINUOUS |
					Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;
			}
			else if (cmd_param.first == "check_pos_min")
			{
				for (auto &option : target.mot_options) option &= ~Plan::NOT_CHECK_POS_MIN;
			}
			else if (cmd_param.first == "not_check_pos_min")
			{
				for (auto &option : target.mot_options) option |= Plan::NOT_CHECK_POS_MIN;
			}
			else if (cmd_param.first == "check_pos_max")
			{
				for (auto &option : target.mot_options) option &= ~Plan::NOT_CHECK_POS_MAX;
			}
			else if (cmd_param.first == "not_check_pos_max")
			{
				for (auto &option : target.mot_options) option |= Plan::NOT_CHECK_POS_MAX;
			}
			else if (cmd_param.first == "check_pos_continuous")
			{
				for (auto &option : target.mot_options) option &= ~Plan::NOT_CHECK_POS_CONTINUOUS;
			}
			else if (cmd_param.first == "not_check_pos_continuous")
			{
				for (auto &option : target.mot_options) option |= Plan::NOT_CHECK_POS_CONTINUOUS;
			}
			else if (cmd_param.first == "check_pos_continuous_second_order")
			{
				for (auto &option : target.mot_options) option &= ~Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
			}
			else if (cmd_param.first == "not_check_pos_continuous_second_order")
			{
				for (auto &option : target.mot_options) option |= Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
			}
			else if (cmd_param.first == "check_pos_following_error")
			{
				for (auto &option : target.mot_options) option &= ~Plan::NOT_CHECK_POS_FOLLOWING_ERROR;
			}
			else if (cmd_param.first == "not_check_pos_following_error")
			{
				for (auto &option : target.mot_options) option |= Plan::NOT_CHECK_POS_FOLLOWING_ERROR;
			}
			else if (cmd_param.first == "check_vel_min")
			{
				for (auto &option : target.mot_options) option &= ~Plan::NOT_CHECK_VEL_MIN;
			}
			else if (cmd_param.first == "not_check_vel_min")
			{
				for (auto &option : target.mot_options) option |= Plan::NOT_CHECK_VEL_MIN;
			}
			else if (cmd_param.first == "check_vel_max")
			{
				for (auto &option : target.mot_options) option &= ~Plan::NOT_CHECK_VEL_MAX;
			}
			else if (cmd_param.first == "not_check_vel_max")
			{
				for (auto &option : target.mot_options) option |= Plan::NOT_CHECK_VEL_MAX;
			}
			else if (cmd_param.first == "check_vel_continuous")
			{
				for (auto &option : target.mot_options) option &= ~Plan::NOT_CHECK_VEL_CONTINUOUS;
			}
			else if (cmd_param.first == "not_check_vel_continuous")
			{
				for (auto &option : target.mot_options) option |= Plan::NOT_CHECK_VEL_CONTINUOUS;
			}
			else if (cmd_param.first == "check_vel_following_error")
			{
				for (auto &option : target.mot_options) option &= ~Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;
			}
			else if (cmd_param.first == "not_check_vel_following_error")
			{
				for (auto &option : target.mot_options) option |= Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;
			}
		}
	}

	struct SetActiveMotor { std::vector<int> active_motor; };
#define SELECT_MOTOR_STRING \
		"		<UniqueParam default=\"all\">"\
		"			<Param name=\"all\" abbreviation=\"a\"/>"\
		"			<Param name=\"motion_id\" abbreviation=\"m\" default=\"0\"/>"\
		"			<Param name=\"physical_id\" abbreviation=\"p\" default=\"0\"/>"\
		"			<Param name=\"slave_id\" abbreviation=\"s\" default=\"0\"/>"\
		"		</UniqueParam>"
	auto set_active_motor(const std::map<std::string, std::string> &cmd_params, PlanTarget &target, SetActiveMotor &param)->void
	{
		param.active_motor.clear();
		param.active_motor.resize(target.controller->motionPool().size(), 0);

		for (auto cmd_param : cmd_params)
		{
			if (cmd_param.first == "all")
			{
				std::fill(param.active_motor.begin(), param.active_motor.end(), 1);
			}
			else if (cmd_param.first == "motion_id")
			{
				param.active_motor.at(std::stoi(cmd_param.second)) = 1;
			}
			else if (cmd_param.first == "physical_id")
			{
				param.active_motor.at(target.controller->motionAtPhy(std::stoi(cmd_param.second)).motId()) = 1;
			}
			else if (cmd_param.first == "slave_id")
			{
				param.active_motor.at(target.controller->motionAtSla(std::stoi(cmd_param.second)).motId()) = 1;
			}
		}
	}

	struct SetInputMovement
	{
		std::vector<double> axis_begin_pos_vec;
		std::vector<double> axis_pos_vec;
		std::vector<double> axis_vel_vec;
		std::vector<double> axis_acc_vec;
		std::vector<double> axis_dec_vec;
		std::vector<double> axis_jerk_vec;
	};
#define SET_INPUT_MOVEMENT_STRING \
		"		<Param name=\"pos\" default=\"0.5\"/>"\
		"		<Param name=\"acc\" default=\"0.1\"/>"\
		"		<Param name=\"vel\" default=\"0.1\"/>"\
		"		<Param name=\"dec\" default=\"0.1\"/>"
	auto set_input_movement(const std::map<std::string, std::string> &cmd_params, PlanTarget &target, SetInputMovement &param)->void
	{
		param.axis_begin_pos_vec.resize(target.controller->motionPool().size(), 0.0);
		for (auto cmd_param : cmd_params)
		{
			if (cmd_param.first == "pos")
			{
				auto p = target.model->calculator().calculateExpression(cmd_param.second);
				if (p.size() == 1)
				{
					param.axis_pos_vec.resize(target.controller->motionPool().size(), p.toDouble());
				}
				else if (p.size() == target.controller->motionPool().size())
				{
					param.axis_pos_vec.assign(p.begin(), p.end());
				}
				else
				{
					THROW_FILE_LINE("");
				}
			}
			else if (cmd_param.first == "acc")
			{
				auto a = target.model->calculator().calculateExpression(cmd_param.second);

				if (a.size() == 1)
				{
					param.axis_acc_vec.resize(target.controller->motionPool().size(), a.toDouble());
				}
				else if (a.size() == target.controller->motionPool().size())
				{
					param.axis_acc_vec.assign(a.begin(), a.end());
				}
				else
				{
					THROW_FILE_LINE("");
				}

			}
			else if (cmd_param.first == "vel")
			{
				auto v = target.model->calculator().calculateExpression(cmd_param.second);

				if (v.size() == 1)
				{
					param.axis_vel_vec.resize(target.controller->motionPool().size(), v.toDouble());
				}
				else if (v.size() == target.controller->motionPool().size())
				{
					param.axis_vel_vec.assign(v.begin(), v.end());
				}
				else
				{
					THROW_FILE_LINE("");
				}
			}
			else if (cmd_param.first == "dec")
			{
				auto d = target.model->calculator().calculateExpression(cmd_param.second);

				if (d.size() == 1)
				{
					param.axis_dec_vec.resize(target.controller->motionPool().size(), d.toDouble());
				}
				else if (d.size() == target.controller->motionPool().size())
				{
					param.axis_dec_vec.assign(d.begin(), d.end());
				}
				else
				{
					THROW_FILE_LINE("");
				}
			}
			else if (cmd_param.first == "jerk")
			{
				auto d = target.model->calculator().calculateExpression(cmd_param.second);

				if (d.size() == 1)
				{
					param.axis_jerk_vec.resize(target.controller->motionPool().size(), d.toDouble());
				}
				else if (d.size() == target.controller->motionPool().size())
				{
					param.axis_jerk_vec.assign(d.begin(), d.end());
				}
				else
				{
					THROW_FILE_LINE("");
				}
			}
		}
	}
	auto check_input_movement(const std::map<std::string, std::string> &cmd_params, PlanTarget &target, SetInputMovement &param, SetActiveMotor &active)->void
	{
		auto c = target.controller;
		for (Size i = 0; i < c->motionPool().size(); ++i)
		{
			if (active.active_motor[i])
			{
				if (param.axis_pos_vec[i] > c->motionPool()[i].maxPos() || param.axis_pos_vec[i] < c->motionPool()[i].minPos())
					THROW_FILE_LINE("input pos beyond range");

				if (param.axis_vel_vec[i] > c->motionPool()[i].maxVel() || param.axis_vel_vec[i] <= 0.0)
					THROW_FILE_LINE("input vel beyond range");

				if (param.axis_acc_vec[i] > c->motionPool()[i].maxAcc() || param.axis_acc_vec[i] <= 0.0)
					THROW_FILE_LINE("input acc beyond range");

				if (param.axis_dec_vec[i] > c->motionPool()[i].maxAcc() || param.axis_dec_vec[i] <= 0.0)
					THROW_FILE_LINE("input dec beyond range");

				if (param.axis_jerk_vec[i] > 1000*c->motionPool()[i].maxAcc() || param.axis_jerk_vec[i] <= 0.0)
					THROW_FILE_LINE("input dec beyond range");
			}
		}
	}


	// 获取part_pq，end_pq，end_pe等 //
	struct GetParam
	{
		std::vector<double> part_pq, end_pq, end_pe, motion_pos, motion_vel, motion_acc, motion_toq, ai;
		std::vector<bool> di;
		std::int32_t state_code;
		aris::control::EthercatController::SlaveLinkState sls[6];
		aris::control::EthercatController::MasterLinkState mls{};
		std::vector<int> motion_state;
		std::string currentplan;
		int vel_percent;
	};
	auto Get::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{		
		GetParam par;
		par.part_pq.resize(target.model->partPool().size() * 7, 0.0);
		par.end_pq.resize(7, 0.0);
		par.end_pe.resize(6, 0.0);
		par.motion_pos.resize(7, 0.0);
		par.motion_vel.resize(7, 0.0);
		par.motion_acc.resize(7, 0.0);
		par.motion_toq.resize(7, 0.0);
		par.ai.resize(100, 1.0);
		par.di.resize(100, false);
		par.motion_state.resize(6, 0);
		std::any param = par;
		//std::any param = std::make_any<GetParam>();

		target.server->getRtData([&](aris::server::ControlServer& cs, const aris::plan::PlanTarget *target, std::any& data)->void
		{
			for (aris::Size i(-1); ++i < cs.model().partPool().size();)
			{
				cs.model().partPool().at(i).getPq(std::any_cast<GetParam &>(data).part_pq.data() + i * 7);
			}

			cs.model().generalMotionPool().at(0).getMpq(std::any_cast<GetParam &>(data).end_pq.data());
			cs.model().generalMotionPool().at(0).getMpe(std::any_cast<GetParam &>(data).end_pe.data(), "321");

			for (aris::Size i = 0; i < 6; i++)
			{
#ifdef WIN32
				std::any_cast<GetParam &>(data).motion_pos[i] = cs.model().motionPool()[i].mp();
				std::any_cast<GetParam &>(data).motion_vel[i] = cs.model().motionPool()[i].mv();
				std::any_cast<GetParam &>(data).motion_acc[i] = cs.model().motionPool()[i].ma();
				std::any_cast<GetParam &>(data).motion_toq[i] = cs.model().motionPool()[i].ma();
#endif // WIN32

#ifdef UNIX
				std::any_cast<GetParam &>(data).motion_pos[i] = cs.controller().motionPool()[i].actualPos();
				std::any_cast<GetParam &>(data).motion_vel[i] = cs.controller().motionPool()[i].actualVel();
				std::any_cast<GetParam &>(data).motion_acc[i] = cs.model().motionPool()[i].ma();
				std::any_cast<GetParam &>(data).motion_toq[i] = cs.controller().motionPool()[i].actualToq();
#endif // UNIX
			}
			std::any_cast<GetParam &>(data).motion_pos[6] = cs.controller().motionPool()[6].actualPos();
			std::any_cast<GetParam &>(data).motion_vel[6] = cs.controller().motionPool()[6].actualVel();
			std::any_cast<GetParam &>(data).motion_acc[6] = 0;
			std::any_cast<GetParam &>(data).motion_toq[6] = cs.controller().motionPool()[6].actualToq();

			for (aris::Size i = 0; i < 100; i++)
			{
				std::any_cast<GetParam &>(data).ai[i] = 1.0;
				std::any_cast<GetParam &>(data).di[i] = false;
			}
			
			auto ec = dynamic_cast<aris::control::EthercatController*>(&cs.controller());
			ec->getLinkState(&std::any_cast<GetParam &>(data).mls, std::any_cast<GetParam &>(data).sls);

			//获取motion的使能状态，0表示去使能状态，1表示使能状态//
			for (aris::Size i = 0; i < 6; i++)
			{
				auto cm = dynamic_cast<aris::control::EthercatMotion*>(&cs.controller().motionPool()[i]);
				if ((cm->statusWord() & 0x6f) != 0x27)
				{
					std::any_cast<GetParam &>(data).motion_state[i] = 0;
				}
				else
				{
					std::any_cast<GetParam &>(data).motion_state[i] = 1;
				}
			}
			
			if (target == nullptr)
			{
				std::any_cast<GetParam &>(data).currentplan = "none";
			}
			else
			{
				std::any_cast<GetParam &>(data).currentplan = target->plan->command().name();
			}

		}, param);

		auto out_data = std::any_cast<GetParam &>(param);
		std::vector<int> slave_online(6, 0), slave_al_state(6, 0);
		for (aris::Size i = 0; i < 6; i++)
		{
			slave_online[i] = int(out_data.sls[i].online);
			slave_al_state[i] = int(out_data.sls[i].al_state);
		}
		out_data.state_code = get_state_code();

		std::vector<std::pair<std::string, std::any>> out_param;
		out_param.push_back(std::make_pair<std::string, std::any>("part_pq", out_data.part_pq));
		out_param.push_back(std::make_pair<std::string, std::any>("end_pq", out_data.end_pq));
		out_param.push_back(std::make_pair<std::string, std::any>("end_pe", out_data.end_pe));
		out_param.push_back(std::make_pair<std::string, std::any>("motion_pos", out_data.motion_pos));
		out_param.push_back(std::make_pair<std::string, std::any>("motion_vel", out_data.motion_vel));
		out_param.push_back(std::make_pair<std::string, std::any>("motion_acc", out_data.motion_acc));
		out_param.push_back(std::make_pair<std::string, std::any>("motion_toq", out_data.motion_toq));
		out_param.push_back(std::make_pair<std::string, std::any>("ai", out_data.ai));
		out_param.push_back(std::make_pair<std::string, std::any>("di", out_data.di));
		out_param.push_back(std::make_pair<std::string, std::any>("state_code", out_data.state_code));
		out_param.push_back(std::make_pair<std::string, std::any>("slave_link_num", std::int32_t(out_data.mls.slaves_responding)));
		out_param.push_back(std::make_pair<std::string, std::any>("slave_online_state", slave_online));
		out_param.push_back(std::make_pair<std::string, std::any>("slave_al_state", slave_al_state));
		out_param.push_back(std::make_pair<std::string, std::any>("motion_state", out_data.motion_state));
		out_param.push_back(std::make_pair<std::string, std::any>("current_plan", out_data.currentplan));
		out_param.push_back(std::make_pair<std::string, std::any>("current_plan_id", cmdparam.current_plan_id));

		target.ret = out_param;
		target.option |= NOT_RUN_EXECUTE_FUNCTION | NOT_PRINT_CMD_INFO | NOT_PRINT_CMD_INFO;
	}
	auto Get::collectNrt(PlanTarget &target)->void {}
	Get::Get(const std::string &name) : Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"get\">"
			"</Command>");
	}

	struct EnableParam :public SetActiveMotor { std::int32_t limit_time; };
	struct Enable::Imp {};
	auto Enable::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		EnableParam param;

		set_check_option(params, target);
		set_active_motor(params, target, param);
		param.limit_time = std::stoi(params.at("limit_time"));

		target.param = param;

		for (auto &option : target.mot_options) option |= aris::plan::Plan::NOT_CHECK_ENABLE | aris::plan::Plan::NOT_CHECK_POS_MAX | aris::plan::Plan::NOT_CHECK_POS_MIN;

		std::vector<std::pair<std::string, std::any>> ret_value;
		target.ret = ret_value;
	}
	auto Enable::executeRT(PlanTarget &target)->int
	{
		auto &param = std::any_cast<EnableParam &>(target.param);

		bool is_all_finished = true;
		for (std::size_t i = 0; i < target.controller->motionPool().size(); ++i)
		{
			if (param.active_motor[i])
			{
				auto &cm = target.controller->motionPool().at(i);
				auto ret = cm.enable();
				if (ret)
				{
					is_all_finished = false;

					if (target.count % 1000 == 0)
					{
						target.controller->mout() << "Unenabled motor, slave id: " << cm.id()
							<< ", absolute id: " << i << ", ret: " << ret << std::endl;
					}
				}
			}
		}

		return is_all_finished ? 0 : (target.count < param.limit_time ? 1 : aris::plan::PlanTarget::PLAN_OVER_TIME);
	}
	Enable::~Enable() = default;
	Enable::Enable(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"en\">"
			"	<GroupParam>"
			"		<Param name=\"limit_time\" default=\"5000\"/>"
			SELECT_MOTOR_STRING
			CHECK_PARAM_STRING
			"	</GroupParam>"
			"</Command>");
	}
	ARIS_DEFINE_BIG_FOUR_CPP(Enable);

	struct DisableParam :public SetActiveMotor { std::int32_t limit_time; };
	struct Disable::Imp {};
	auto Disable::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		DisableParam param;

		set_check_option(params, target);
		set_active_motor(params, target, param);
		param.limit_time = std::stoi(params.at("limit_time"));

		target.param = param;
		for (auto &option : target.mot_options) option |= aris::plan::Plan::NOT_CHECK_ENABLE | aris::plan::Plan::NOT_CHECK_POS_MAX | aris::plan::Plan::NOT_CHECK_POS_MIN;

		std::vector<std::pair<std::string, std::any>> ret_value;
		target.ret = ret_value;
	}
	auto Disable::executeRT(PlanTarget &target)->int
	{
		auto &param = std::any_cast<DisableParam &>(target.param);

		bool is_all_finished = true;
		for (std::size_t i = 0; i < target.controller->motionPool().size(); ++i)
		{
			if (param.active_motor[i])
			{
				auto &cm = target.controller->motionPool().at(i);
				auto ret = cm.disable();
				if (ret)
				{
					is_all_finished = false;

					if (target.count % 1000 == 0)
					{
						target.controller->mout() << "Undisabled motor, slave id: " << cm.id()
							<< ", absolute id: " << i << ", ret: " << ret << std::endl;
					}
				}
			}
		}

		return is_all_finished ? 0 : (target.count < param.limit_time ? 1 : aris::plan::PlanTarget::PLAN_OVER_TIME);
	}
	Disable::~Disable() = default;
	Disable::Disable(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"ds\">"
			"	<GroupParam>"
			"		<Param name=\"limit_time\" default=\"5000\"/>"
			SELECT_MOTOR_STRING
			CHECK_PARAM_STRING
			"	</GroupParam>"
			"</Command>");
	}
	ARIS_DEFINE_BIG_FOUR_CPP(Disable);

	struct HomeParam :public SetActiveMotor
	{
		std::int32_t limit_time;
		double offset;
	};
	struct Home::Imp {};
	auto Home::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		HomeParam param;

		set_active_motor(params, target, param);
		param.limit_time = std::stoi(params.at("limit_time"));

		for (aris::Size i = 0; i < param.active_motor.size(); ++i)
		{
			if (param.active_motor[i])
			{
				std::int8_t method = std::stoi(params.at(std::string("method")));
				if (method < 1 || method > 35) THROW_FILE_LINE("invalid home method");

				param.offset = std::stod(params.at(std::string("offset")));
				std::int32_t offset = std::stoi(params.at(std::string("offset")));
				std::uint32_t high_speed = std::stoi(params.at(std::string("high_speed")));
				std::uint32_t low_speed = std::stoi(params.at(std::string("low_speed")));
				std::uint32_t acc = std::stoi(params.at(std::string("acceleration")));

				auto &cm = dynamic_cast<aris::control::EthercatMotion &>(target.controller->motionPool()[i]);

				cm.writeSdo(0x6098, 0x00, method);
				std::int8_t method_read;
				cm.readSdo(0x6098, 0x00, method_read);
				if (method_read != method)THROW_FILE_LINE("home sdo write failed method");
				cm.writeSdo(0x607C, 0x00, offset);
				std::int32_t offset_read;
				cm.readSdo(0x607C, 0x00, offset_read);
				if (offset_read != offset)THROW_FILE_LINE("home sdo write failed offset");
				cm.writeSdo(0x6099, 0x01, high_speed);
				std::int32_t high_speed_read;
				cm.readSdo(0x6099, 0x01, high_speed_read);
				if (high_speed_read != high_speed)THROW_FILE_LINE("home sdo write failed high_speed");
				cm.writeSdo(0x6099, 0x02, low_speed);
				std::int32_t low_speed_read;
				cm.readSdo(0x6099, 0x02, low_speed_read);
				if (low_speed_read != low_speed)THROW_FILE_LINE("home sdo write failed low_speed");
				cm.writeSdo(0x609A, 0x00, acc);
				std::int32_t acc_read;
				cm.readSdo(0x609A, 0x00, acc_read);
				if (acc_read != acc)THROW_FILE_LINE("home sdo write failed acc");

			}
		}

		target.param = param;

		std::vector<std::pair<std::string, std::any>> ret_value;
		target.ret = ret_value;
	}
	auto Home::executeRT(PlanTarget &target)->int
	{
		auto &param = std::any_cast<HomeParam &>(target.param);

		bool is_all_finished = true;
		for (std::size_t i = 0; i < target.controller->motionPool().size(); ++i)
		{
			if (param.active_motor[i])
			{
				auto &cm = target.controller->motionPool().at(i);
				auto ret = cm.home();
				if (ret)
				{
					is_all_finished = false;

					if (target.count % 1000 == 0)
					{
						target.controller->mout() << "Unhomed motor, slave id: " << cm.id()
							<< ", absolute id: " << i << ", ret: " << ret << std::endl;
					}
				}
				else
				{
					param.active_motor[i] = false;
				}
			}
		}

		return is_all_finished ? 0 : 1;
	}
	Home::~Home() = default;
	Home::Home(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"hm\">"
			"	<GroupParam>"
			"		<Param name=\"method\" default=\"35\"/>"
			"		<Param name=\"offset\" default=\"0\"/>"
			"		<Param name=\"high_speed\" default=\"20000\"/>"
			"		<Param name=\"low_speed\" default=\"20\"/>"
			"		<Param name=\"acceleration\" default=\"100000\"/>"
			"		<Param name=\"limit_time\" default=\"5000\"/>"
			SELECT_MOTOR_STRING
			CHECK_PARAM_STRING
			"	</GroupParam>"
			"</Command>");
	}
	ARIS_DEFINE_BIG_FOUR_CPP(Home);

	struct ModeParam :public SetActiveMotor { std::int32_t limit_time, mode; };
	struct Mode::Imp {};
	auto Mode::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		ModeParam param;

		set_check_option(params, target);
		set_active_motor(params, target, param);
		param.limit_time = std::stoi(params.at("limit_time"));
		param.mode = std::stoi(params.at("mode"));

		if (param.mode > 10 && param.mode < 8)THROW_FILE_LINE("invalid mode, aris now only support mode 8,9,10");

		target.param = param;
		for (auto &option : target.mot_options) option |= aris::plan::Plan::NOT_CHECK_ENABLE | aris::plan::Plan::NOT_CHECK_POS_MAX | aris::plan::Plan::NOT_CHECK_POS_MIN;

		std::vector<std::pair<std::string, std::any>> ret_value;
		target.ret = ret_value;
	}
	auto Mode::executeRT(PlanTarget &target)->int
	{
		auto &param = std::any_cast<ModeParam &>(target.param);

		bool is_all_finished = true;
		for (std::size_t i = 0; i < target.controller->motionPool().size(); ++i)
		{
			if (param.active_motor[i])
			{
				auto &cm = target.controller->motionPool().at(i);
				auto ret = cm.mode(param.mode);
				if (target.count == 1)
				{
					switch (param.mode)
					{
					case 8:
						cm.setTargetPos(cm.actualPos());
						break;
					case 9:
						cm.setTargetVel(0.0);
						break;
					case 10:
						cm.setTargetToq(0.0);
						break;
					default:
						break;
					}
				}

				if (ret)
				{
					is_all_finished = false;

					if (target.count % 1000 == 0)
					{
						target.controller->mout() << "Unmoded motor, slave id: " << cm.id()
							<< ", absolute id: " << i << ", ret: " << ret << std::endl;
					}
				}
			}
		}

		return is_all_finished ? 0 : (target.count < param.limit_time ? 1 : aris::plan::PlanTarget::PLAN_OVER_TIME);
	}
	Mode::~Mode() = default;
	Mode::Mode(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"md\">"
			"	<GroupParam>"
			"		<Param name=\"limit_time\" default=\"5000\"/>"
			"       <Param name=\"mode\" abbreviation=\"d\" default=\"8\"/>"
			SELECT_MOTOR_STRING
			CHECK_PARAM_STRING
			"	</GroupParam>"
			"</Command>");
	}
	ARIS_DEFINE_BIG_FOUR_CPP(Mode);

	struct ResetParam :public SetActiveMotor, SetInputMovement { std::vector<Size> total_count_vec; };
	struct Reset::Imp {};
	auto Reset::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		ResetParam param;
		set_check_option(params, target);
		set_active_motor(params, target, param);
		set_input_movement(params, target, param);

		for (Size i = 0; i < target.controller->motionPool().size(); ++i)
		{
			auto &cm = target.controller->motionPool()[i];
			param.axis_pos_vec[i] = param.axis_pos_vec[i] * (cm.maxPos() - cm.minPos()) + cm.minPos();
			param.axis_acc_vec[i] = param.axis_acc_vec[i] * cm.maxAcc();
			param.axis_vel_vec[i] = param.axis_vel_vec[i] * cm.maxVel();
			param.axis_dec_vec[i] = param.axis_dec_vec[i] * cm.maxAcc();
		}
		check_input_movement(params, target, param, param);

		param.total_count_vec.resize(target.controller->motionPool().size(), 1);

		target.param = param;

		std::vector<std::pair<std::string, std::any>> ret_value;
		target.ret = ret_value;
	}
	auto Reset::executeRT(PlanTarget &target)->int
	{
		auto &param = std::any_cast<ResetParam&>(target.param);

		// 取得起始位置 //
		if (target.count == 1)
		{
			for (Size i = 0; i < target.controller->motionPool().size(); ++i)
			{
				if (param.active_motor[i])
				{
					param.axis_begin_pos_vec[i] = target.controller->motionPool().at(i).actualPos();
				}
			}
		}

		// 设置驱动器的位置 //
		for (Size i = 0; i < target.controller->motionPool().size(); ++i)
		{
			if (param.active_motor[i])
			{
				double p, v, a;
				aris::plan::moveAbsolute(static_cast<double>(target.count), param.axis_begin_pos_vec[i], param.axis_pos_vec[i], param.axis_vel_vec[i] / 1000
					, param.axis_acc_vec[i] / 1000 / 1000, param.axis_dec_vec[i] / 1000 / 1000, p, v, a, param.total_count_vec[i]);
				target.controller->motionAtAbs(i).setTargetPos(p);
			}
		}

		return (static_cast<int>(*std::max_element(param.total_count_vec.begin(), param.total_count_vec.end())) > target.count) ? 1 : 0;
	}
	Reset::~Reset() = default;
	Reset::Reset(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"rs\">"
			"	<GroupParam>"
			SET_INPUT_MOVEMENT_STRING
			SELECT_MOTOR_STRING
			CHECK_PARAM_STRING
			"	</GroupParam>"
			"</Command>");
	}
	ARIS_DEFINE_BIG_FOUR_CPP(Reset);

	struct RecoverParam
	{
		std::atomic_bool is_kinematic_ready_;
		std::atomic_bool is_rt_waiting_ready_;
		std::future<void> fut;
		int kin_ret;
	};
	auto Recover::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		auto p = std::make_shared<RecoverParam>();

		p->is_kinematic_ready_ = false;
		p->is_rt_waiting_ready_ = false;
		p->fut = std::async(std::launch::async, [](std::shared_ptr<RecoverParam> p, PlanTarget &target)
		{
			// 等待正解求解的需求 //
			while (!p->is_rt_waiting_ready_.load())std::this_thread::sleep_for(std::chrono::milliseconds(1));

			// 求正解 //
			p->kin_ret = target.model->solverPool()[1].kinPos();

			// 通知实时线程 //
			p->is_kinematic_ready_.store(true);
		}, p, std::ref(target));

		target.param = p;
		for (auto &option : target.mot_options) option |= NOT_CHECK_ENABLE | NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;

		std::vector<std::pair<std::string, std::any>> ret_value;
		target.ret = ret_value;
	}
	auto Recover::executeRT(PlanTarget &target)->int
	{
		auto param = std::any_cast<std::shared_ptr<RecoverParam> &>(target.param);

		if (target.count < 3)
		{
			for (Size i = 0; i < std::min(target.controller->motionPool().size(), target.model->motionPool().size()); ++i)
			{
				target.controller->motionPool()[i].setTargetPos(target.controller->motionPool().at(i).actualPos());
				target.model->motionPool()[i].setMp(target.controller->motionPool().at(i).actualPos());
			}

			param->is_rt_waiting_ready_.store(true);

			return 1;
		}

		return param->is_kinematic_ready_.load() ? param->kin_ret : 1;
	}
	auto Recover::collectNrt(PlanTarget &target)->void
	{
		if (target.count)
		{
			std::any_cast<std::shared_ptr<RecoverParam>&>(target.param)->fut.get();
		}
		else
		{
			// 此时前面指令出错，系统清理了该命令，这时设置一下 //
			std::any_cast<std::shared_ptr<RecoverParam>&>(target.param)->is_rt_waiting_ready_.store(true);
			std::any_cast<std::shared_ptr<RecoverParam>&>(target.param)->fut.get();
		}
	}
	Recover::~Recover() = default;
	Recover::Recover(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"rc\">"
			"</Command>");
	}
	ARIS_DEFINE_BIG_FOUR_CPP(Recover);

	struct SleepParam { int count; };
	struct Sleep::Imp {};
	auto Sleep::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		target.param = SleepParam{ std::stoi(params.at("count")) };
		for (auto &option : target.mot_options) option |= NOT_CHECK_ENABLE;

		std::vector<std::pair<std::string, std::any>> ret_value;
		target.ret = ret_value;
	}
	auto Sleep::executeRT(PlanTarget &target)->int { return std::any_cast<SleepParam&>(target.param).count - target.count; }
	Sleep::~Sleep() = default;
	Sleep::Sleep(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"sl\">"
			"	<Param name=\"count\" default=\"1000\" abbreviation=\"c\"/>"
			"</Command>");
	}
	ARIS_DEFINE_BIG_FOUR_CPP(Sleep);


	struct MoveAbsJParam :public SetActiveMotor, SetInputMovement {};
	auto MoveAbsJ::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		std::cout << "mvaj:" << std::endl;
		MoveAbsJParam param;

		set_check_option(params, target);
		set_active_motor(params, target, param);
		set_input_movement(params, target, param);
		check_input_movement(params, target, param, param);

		param.axis_begin_pos_vec.resize(target.controller->motionPool().size());
		target.param = param;
		//std::fill(target.mot_options.begin(), target.mot_options.end(), Plan::USE_TARGET_POS);
		for (auto &option : target.mot_options) option |= aris::plan::Plan::NOT_CHECK_ENABLE;

		std::vector<std::pair<std::string, std::any>> ret_value;
		target.ret = ret_value;
	}
	auto MoveAbsJ::executeRT(PlanTarget &target)->int
	{
		auto controller = target.controller;
		auto param = std::any_cast<MoveAbsJParam>(&target.param);

		if (target.count == 1)
		{
			for (Size i = 0; i < param->active_motor.size(); ++i)
			{
				if (param->active_motor[i])
				{
					param->axis_begin_pos_vec[i] = target.controller->motionPool()[i].targetPos();
					//param->axis_begin_pos_vec[i] = target.model->motionPool()[i].mp();
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
				//aris::plan::moveAbsolute(target.count,
				//	param->axis_begin_pos_vec[i], param->axis_pos_vec[i],
				//	param->axis_vel_vec[i] / 1000, param->axis_acc_vec[i] / 1000 / 1000, param->axis_dec_vec[i] / 1000 / 1000,
				//	p, v, a, t_count);
				
				//s形规划//
				traplan::sCurve(target.count, param->axis_begin_pos_vec[i], param->axis_pos_vec[i],
					param->axis_vel_vec[i] / 1000, param->axis_acc_vec[i] / 1000 / 1000, param->axis_jerk_vec[i] / 1000 / 1000 / 1000,
					p, v, a, j, t_count);
				target.controller->motionPool()[i].setTargetPos(p);
				target.model->motionPool()[i].setMp(p);
				total_count = std::max(total_count, t_count);

				// log //
				auto &lout = controller->lout();
				lout << p << " ";
				lout << v << " ";
				lout << a << " ";
				lout << j << " ";	
				lout << std::endl;
			}
		}
		if (target.model->solverPool().at(1).kinPos())return -1;

		if (g_move_stop)
		{
			return -4;
		}

		return total_count - target.count;
	}
	MoveAbsJ::~MoveAbsJ() = default;
	MoveAbsJ::MoveAbsJ(const std::string &name) : Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"mvaj\">"
			"	<GroupParam>"
			"		<Param name=\"pos\" default=\"0.0\"/>"
			"		<Param name=\"vel\" default=\"1.0\"/>"
			"		<Param name=\"acc\" default=\"1.0\"/>"
			"		<Param name=\"dec\" default=\"1.0\"/>"
			"		<Param name=\"jerk\" default=\"10.0\"/>"
			SELECT_MOTOR_STRING
			CHECK_PARAM_STRING
			"	</GroupParam>"
			"</Command>");
	}


	auto check_eul_validity(const std::string &eul_type)->bool
	{
		if (eul_type.size() < 3)return false;

		for (int i = 0; i < 3; ++i)if (eul_type[i] > '3' || eul_type[i] < '1')return false;

		if (eul_type[0] == eul_type[1] || eul_type[1] == eul_type[2]) return false;

		return true;
	}
	auto find_pq(const std::map<std::string, std::string> &params, PlanTarget &target, double *pq_out)->bool
	{
		double pos_unit;
		auto pos_unit_found = params.find("pos_unit");
		if (pos_unit_found == params.end()) pos_unit = 1.0;
		else if (pos_unit_found->second == "m")pos_unit = 1.0;
		else if (pos_unit_found->second == "mm")pos_unit = 0.001;
		else if (pos_unit_found->second == "cm")pos_unit = 0.01;
		else THROW_FILE_LINE("");

		for (auto cmd_param : params)
		{
			if (cmd_param.first == "pq")
			{
				auto pq_mat = target.model->calculator().calculateExpression(cmd_param.second);
				if (pq_mat.size() != 7)THROW_FILE_LINE("");
				aris::dynamic::s_vc(7, pq_mat.data(), pq_out);
				aris::dynamic::s_nv(3, pos_unit, pq_out);
				return true;
			}
			else if (cmd_param.first == "pm")
			{
				auto pm_mat = target.model->calculator().calculateExpression(cmd_param.second);
				if (pm_mat.size() != 16)THROW_FILE_LINE("");
				aris::dynamic::s_pm2pq(pm_mat.data(), pq_out);
				aris::dynamic::s_nv(3, pos_unit, pq_out);
				return true;
			}
			else if (cmd_param.first == "pe")
			{
				double ori_unit;
				auto ori_unit_found = params.find("ori_unit");
				if (ori_unit_found == params.end()) ori_unit = 1.0;
				else if (ori_unit_found->second == "rad")ori_unit = 1.0;
				else if (ori_unit_found->second == "degree")ori_unit = PI / 180.0;
				else THROW_FILE_LINE("");

				std::string eul_type;
				auto eul_type_found = params.find("eul_type");
				if (eul_type_found == params.end()) eul_type = "321";
				else if (check_eul_validity(eul_type_found->second.data()))	eul_type = eul_type_found->second;
				else THROW_FILE_LINE("");

				auto pe_mat = target.model->calculator().calculateExpression(cmd_param.second);
				if (pe_mat.size() != 6)THROW_FILE_LINE("");
				aris::dynamic::s_nv(3, ori_unit, pe_mat.data() + 3);
				aris::dynamic::s_pe2pq(pe_mat.data(), pq_out, eul_type.data());
				aris::dynamic::s_nv(3, pos_unit, pq_out);
				return true;
			}
		}

		THROW_FILE_LINE("No pose input");
	}
	struct MoveJParam
	{
		std::vector<double> joint_vel, joint_acc, joint_dec, joint_jerk, ee_pq, joint_pos_begin, joint_pos_end;
		std::vector<Size> total_count;
		aris::dynamic::Marker *tool, *wobj;
	};
	struct MoveJ::Imp {};
	auto MoveJ::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		set_check_option(params, target);

		MoveJParam mvj_param;

		// find ee pq //
		mvj_param.ee_pq.resize(7);
		find_pq(params, target, mvj_param.ee_pq.data());

		mvj_param.joint_pos_begin.resize(target.model->motionPool().size(), 0.0);
		mvj_param.joint_pos_end.resize(target.model->motionPool().size(), 0.0);
		mvj_param.total_count.resize(target.model->motionPool().size(), 0);

		auto tool_is_string = params.at("tool");
		auto wobj_is_string = params.at("wobj");

		mvj_param.tool = &*target.model->generalMotionPool()[0].makI().fatherPart().markerPool().findByName(params.at("tool"));
		mvj_param.wobj = &*target.model->generalMotionPool()[0].makJ().fatherPart().markerPool().findByName(params.at("wobj"));
		
		// find joint acc/vel/dec/
		for (auto cmd_param : params)
		{
			auto c = target.controller;
			if (cmd_param.first == "joint_acc")
			{
				mvj_param.joint_acc.clear();
				mvj_param.joint_acc.resize(target.model->motionPool().size(), 0.0);

				auto acc_mat = target.model->calculator().calculateExpression(cmd_param.second);
				if (acc_mat.size() == 1)std::fill(mvj_param.joint_acc.begin(), mvj_param.joint_acc.end(), acc_mat.toDouble());
				else if (acc_mat.size() == target.model->motionPool().size()) std::copy(acc_mat.begin(), acc_mat.end(), mvj_param.joint_acc.begin());
				else THROW_FILE_LINE("");

				for (int i = 0; i < 6; ++i)mvj_param.joint_acc[i] *= target.controller->motionPool()[i].maxAcc();

				// check value validity //
				for (Size i = 0; i < std::min(target.model->motionPool().size(), c->motionPool().size()); ++i)
					if (mvj_param.joint_acc[i] <= 0 || mvj_param.joint_acc[i] > c->motionPool()[i].maxAcc())
						THROW_FILE_LINE("");
			}
			else if (cmd_param.first == "joint_vel")
			{
				mvj_param.joint_vel.clear();
				mvj_param.joint_vel.resize(target.model->motionPool().size(), 0.0);

				auto vel_mat = target.model->calculator().calculateExpression(cmd_param.second);
				if (vel_mat.size() == 1)std::fill(mvj_param.joint_vel.begin(), mvj_param.joint_vel.end(), vel_mat.toDouble());
				else if (vel_mat.size() == target.model->motionPool().size()) std::copy(vel_mat.begin(), vel_mat.end(), mvj_param.joint_vel.begin());
				else THROW_FILE_LINE("");

				for (int i = 0; i < 6; ++i)mvj_param.joint_vel[i] *= target.controller->motionPool()[i].maxVel();

				// check value validity //
				for (Size i = 0; i < std::min(target.model->motionPool().size(), c->motionPool().size()); ++i)
					if (mvj_param.joint_vel[i] <= 0 || mvj_param.joint_vel[i] > c->motionPool()[i].maxVel())
						THROW_FILE_LINE("");
			}
			else if (cmd_param.first == "joint_dec")
			{
				mvj_param.joint_dec.clear();
				mvj_param.joint_dec.resize(target.model->motionPool().size(), 0.0);

				auto dec_mat = target.model->calculator().calculateExpression(cmd_param.second);
				if (dec_mat.size() == 1)std::fill(mvj_param.joint_dec.begin(), mvj_param.joint_dec.end(), dec_mat.toDouble());
				else if (dec_mat.size() == target.model->motionPool().size()) std::copy(dec_mat.begin(), dec_mat.end(), mvj_param.joint_dec.begin());
				else THROW_FILE_LINE("");

				for (int i = 0; i < 6; ++i) mvj_param.joint_dec[i] *= target.controller->motionPool()[i].maxAcc();

				// check value validity //
				for (Size i = 0; i < std::min(target.model->motionPool().size(), c->motionPool().size()); ++i)
					if (mvj_param.joint_dec[i] <= 0 || mvj_param.joint_dec[i] > c->motionPool()[i].maxAcc())
						THROW_FILE_LINE("");
			}
			else if (cmd_param.first == "joint_jerk")
			{
				mvj_param.joint_jerk.clear();
				mvj_param.joint_jerk.resize(target.model->motionPool().size(), 0.0);

				auto dec_mat = target.model->calculator().calculateExpression(cmd_param.second);
				if (dec_mat.size() == 1)std::fill(mvj_param.joint_jerk.begin(), mvj_param.joint_jerk.end(), dec_mat.toDouble());
				else if (dec_mat.size() == target.model->motionPool().size()) std::copy(dec_mat.begin(), dec_mat.end(), mvj_param.joint_jerk.begin());
				else THROW_FILE_LINE("");

				for (int i = 0; i < 6; ++i) mvj_param.joint_jerk[i] *= mvj_param.joint_acc[i];

				// check value validity //
				for (Size i = 0; i < std::min(target.model->motionPool().size(), c->motionPool().size()); ++i)
					if (mvj_param.joint_jerk[i] <= 0 || mvj_param.joint_jerk[i] > 1000*c->motionPool()[i].maxAcc())
						THROW_FILE_LINE("");
			}
		}

		target.param = mvj_param;

		std::vector<std::pair<std::string, std::any>> ret_value;
		target.ret = ret_value;
	}
	auto MoveJ::executeRT(PlanTarget &target)->int
	{
		auto mvj_param = std::any_cast<MoveJParam>(&target.param);
		auto controller = target.controller;

		// 取得起始位置 //
		double p, v, a, j;
		static Size max_total_count;
		if (target.count == 1)
		{
			// inverse kinematic //
			double end_pm[16];
			aris::dynamic::s_pq2pm(mvj_param->ee_pq.data(), end_pm);
			//target.model->generalMotionPool().at(0).setMpm(end_pm);
			mvj_param->tool->setPm(*mvj_param->wobj, end_pm);
			target.model->generalMotionPool().at(0).updMpm();

			if (target.model->solverPool().at(0).kinPos())return -1;

			// init joint_pos //
			for (Size i = 0; i < std::min(controller->motionPool().size(), target.model->motionPool().size()); ++i)
			{
				mvj_param->joint_pos_begin[i] = controller->motionPool()[i].targetPos();
				mvj_param->joint_pos_end[i] = target.model->motionPool()[i].mp();
				//梯形轨迹规划//
				//aris::plan::moveAbsolute(target.count, mvj_param->joint_pos_begin[i], mvj_param->joint_pos_end[i]
				//	, mvj_param->joint_vel[i] / 1000, mvj_param->joint_acc[i] / 1000 / 1000, mvj_param->joint_dec[i] / 1000 / 1000
				//	, p, v, a, mvj_param->total_count[i]);

				//S形轨迹规划//
				traplan::sCurve(target.count, mvj_param->joint_pos_begin[i], mvj_param->joint_pos_end[i],
					mvj_param->joint_vel[i] / 1000, mvj_param->joint_acc[i] / 1000 / 1000, mvj_param->joint_jerk[i] / 1000 / 1000 / 1000,
					p, v, a, j, mvj_param->total_count[i]);
			}

			max_total_count = *std::max_element(mvj_param->total_count.begin(), mvj_param->total_count.end());
		}

		for (Size i = 0; i < std::min(controller->motionPool().size(), target.model->motionPool().size()); ++i)
		{
			//梯形轨迹规划//
			//aris::plan::moveAbsolute(static_cast<double>(target.count) * mvj_param->total_count[i] / max_total_count,
			//	mvj_param->joint_pos_begin[i], mvj_param->joint_pos_end[i],
			//	mvj_param->joint_vel[i] / 1000, mvj_param->joint_acc[i] / 1000 / 1000, mvj_param->joint_dec[i] / 1000 / 1000,
			//	p, v, a, mvj_param->total_count[i]);

			//S形轨迹规划//
			traplan::sCurve(static_cast<double>(target.count) * mvj_param->total_count[i] / max_total_count,
				mvj_param->joint_pos_begin[i], mvj_param->joint_pos_end[i],
				mvj_param->joint_vel[i] / 1000, mvj_param->joint_acc[i] / 1000 / 1000, mvj_param->joint_jerk[i] / 1000 / 1000 / 1000,
				p, v, a, j, mvj_param->total_count[i]);

			controller->motionPool()[i].setTargetPos(p);
			target.model->motionPool()[i].setMp(p);
		}
		if (target.model->solverPool().at(1).kinPos())return -1;

		if (g_move_stop)
		{
			return -4;
		}
		return max_total_count == 0 ? 0 : max_total_count - target.count;
	}
	MoveJ::~MoveJ() = default;
	MoveJ::MoveJ(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"mvj\">"
			"	<GroupParam>"
			"		<Param name=\"pos_unit\" default=\"m\"/>"
			"		<UniqueParam default=\"pq\">"
			"			<Param name=\"pq\" default=\"{0,0,0,0,0,0,1}\"/>"
			"			<Param name=\"pm\" default=\"{1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1}\"/>"
			"			<GroupParam>"
			"				<Param name=\"pe\" default=\"{0,0,0,0,0,0}\"/>"
			"				<Param name=\"ori_unit\" default=\"rad\"/>"
			"				<Param name=\"eul_type\" default=\"321\"/>"
			"			</GroupParam>"
			"		</UniqueParam>"
			"		<Param name=\"joint_acc\" default=\"0.1\"/>"
			"		<Param name=\"joint_vel\" default=\"0.1\"/>"
			"		<Param name=\"joint_dec\" default=\"0.1\"/>"
			"		<Param name=\"joint_jerk\" default=\"10.0\"/>"
			"		<Param name=\"tool\" default=\"tool0\"/>"
			"		<Param name=\"wobj\" default=\"wobj0\"/>"
			CHECK_PARAM_STRING
			"	</GroupParam>"
			"</Command>");
	}
	ARIS_DEFINE_BIG_FOUR_CPP(MoveJ);

	struct MoveLParam
	{
		std::vector<double> joint_vel, joint_acc, joint_dec, joint_jerk, ee_pq, joint_pos_begin, joint_pos_end;
		Size total_count[6];

		double acc, vel, dec, jerk;
		double angular_acc, angular_vel, angular_dec, angular_jerk;
		aris::dynamic::Marker *tool, *wobj;
	};
	struct MoveL::Imp {};
	auto MoveL::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		set_check_option(params, target);

		MoveLParam mvl_param;
		mvl_param.ee_pq.resize(7);
		if (!find_pq(params, target, mvl_param.ee_pq.data()))THROW_FILE_LINE("");

		auto tool_is_string = params.at("tool");
		auto wobj_is_string = params.at("wobj");

		mvl_param.tool = &*target.model->generalMotionPool()[0].makI().fatherPart().markerPool().findByName(params.at("tool"));
		mvl_param.wobj = &*target.model->generalMotionPool()[0].makJ().fatherPart().markerPool().findByName(params.at("wobj"));

		for (auto cmd_param : params)
		{
			if (cmd_param.first == "acc")
			{
				mvl_param.acc = std::stod(cmd_param.second);
			}
			else if (cmd_param.first == "vel")
			{
				mvl_param.vel = std::stod(cmd_param.second);
			}
			else if (cmd_param.first == "dec")
			{
				mvl_param.dec = std::stod(cmd_param.second);
			}
			else if (cmd_param.first == "jerk")
			{
				mvl_param.jerk = std::stod(cmd_param.second)*mvl_param.acc;
			}
			else if (cmd_param.first == "angular_acc")
			{
				mvl_param.angular_acc = std::stod(cmd_param.second);
			}
			else if (cmd_param.first == "angular_vel")
			{
				mvl_param.angular_vel = std::stod(cmd_param.second);
			}
			else if (cmd_param.first == "angular_dec")
			{
				mvl_param.angular_dec = std::stod(cmd_param.second);
			}
			else if (cmd_param.first == "angular_jerk")
			{
				mvl_param.angular_jerk = std::stod(cmd_param.second)*mvl_param.angular_acc;
			}
		}

		for (auto &option : target.mot_options)	option |= USE_TARGET_POS;
		target.param = mvl_param;

		std::vector<std::pair<std::string, std::any>> ret_value;
		target.ret = ret_value;
	}
	auto MoveL::executeRT(PlanTarget &target)->int
	{
		auto mvl_param = std::any_cast<MoveLParam>(&target.param);
		auto controller = target.controller;

		// 取得起始位置 //
		static double begin_pm[16], relative_pm[16], relative_pa[6], pos_ratio, ori_ratio, norm_pos, norm_ori;
		double p, v, a, j;
		aris::Size pos_total_count, ori_total_count;
		if (target.count == 1)
		{
			double end_pm[16];
			aris::dynamic::s_pq2pm(mvl_param->ee_pq.data(), end_pm);
			target.model->generalMotionPool().at(0).updMpm();
			//target.model->generalMotionPool().at(0).getMpm(begin_pm);
			mvl_param->tool->getPm(*mvl_param->wobj, begin_pm);
			aris::dynamic::s_inv_pm_dot_pm(begin_pm, end_pm, relative_pm);


			// relative_pa //
			aris::dynamic::s_pm2pa(relative_pm, relative_pa);

			norm_pos = aris::dynamic::s_norm(3, relative_pa);
			norm_ori = aris::dynamic::s_norm(3, relative_pa + 3);

			//aris::plan::moveAbsolute(target.count, 0.0, norm_pos, mvl_param->vel / 1000, mvl_param->acc / 1000 / 1000, mvl_param->dec / 1000 / 1000, p, v, a, pos_total_count);
			//aris::plan::moveAbsolute(target.count, 0.0, norm_ori, mvl_param->angular_vel / 1000, mvl_param->angular_acc / 1000 / 1000, mvl_param->angular_dec / 1000 / 1000, p, v, a, ori_total_count);

			traplan::sCurve(target.count, 0.0, norm_pos, mvl_param->vel / 1000, mvl_param->acc / 1000 / 1000, mvl_param->jerk / 1000 / 1000 / 1000, p, v, a, j, pos_total_count);
			traplan::sCurve(target.count, 0.0, norm_ori, mvl_param->angular_vel / 1000, mvl_param->angular_acc / 1000 / 1000, mvl_param->angular_jerk / 1000 / 1000 / 1000, p, v, a, j, ori_total_count);

			pos_ratio = pos_total_count < ori_total_count ? double(pos_total_count) / ori_total_count : 1.0;
			ori_ratio = ori_total_count < pos_total_count ? double(ori_total_count) / pos_total_count : 1.0;

			//aris::plan::moveAbsolute(target.count, 0.0, norm_pos, mvl_param->vel / 1000 * pos_ratio, mvl_param->acc / 1000 / 1000 * pos_ratio* pos_ratio, mvl_param->dec / 1000 / 1000 * pos_ratio* pos_ratio, p, v, a, pos_total_count);
			//aris::plan::moveAbsolute(target.count, 0.0, norm_ori, mvl_param->angular_vel / 1000 * ori_ratio, mvl_param->angular_acc / 1000 / 1000 * ori_ratio * ori_ratio, mvl_param->angular_dec / 1000 / 1000 * ori_ratio * ori_ratio, p, v, a, ori_total_count);
		
			traplan::sCurve(target.count, 0.0, norm_pos, mvl_param->vel / 1000 * pos_ratio, mvl_param->acc / 1000 / 1000 * pos_ratio * pos_ratio, mvl_param->jerk / 1000 / 1000 / 1000 * pos_ratio* pos_ratio* pos_ratio, p, v, a, j, pos_total_count);
			traplan::sCurve(target.count, 0.0, norm_ori, mvl_param->angular_vel / 1000 * ori_ratio, mvl_param->angular_acc / 1000 / 1000 * ori_ratio * ori_ratio, mvl_param->angular_jerk / 1000 / 1000 / 1000 * ori_ratio * ori_ratio* ori_ratio, p, v, a, j, ori_total_count);
		
		}

		double pa[6]{ 0,0,0,0,0,0 }, pm[16], pm2[16];

		//aris::plan::moveAbsolute(target.count, 0.0, norm_pos, mvl_param->vel / 1000 * pos_ratio, mvl_param->acc / 1000 / 1000 * pos_ratio* pos_ratio, mvl_param->dec / 1000 / 1000 * pos_ratio* pos_ratio, p, v, a, pos_total_count);
		traplan::sCurve(target.count, 0.0, norm_pos, mvl_param->vel / 1000 * pos_ratio, mvl_param->acc / 1000 / 1000 * pos_ratio* pos_ratio, mvl_param->jerk / 1000 / 1000 /1000 * pos_ratio* pos_ratio* pos_ratio, p, v, a, j, pos_total_count);
		if (norm_pos > 1e-10)aris::dynamic::s_vc(3, p / norm_pos, relative_pa, pa);

		//aris::plan::moveAbsolute(target.count, 0.0, norm_ori, mvl_param->angular_vel / 1000 * ori_ratio, mvl_param->angular_acc / 1000 / 1000 * ori_ratio * ori_ratio, mvl_param->angular_dec / 1000 / 1000 * ori_ratio * ori_ratio, p, v, a, ori_total_count);
		traplan::sCurve(target.count, 0.0, norm_ori, mvl_param->angular_vel / 1000 * ori_ratio, mvl_param->angular_acc / 1000 / 1000 * ori_ratio * ori_ratio, mvl_param->angular_jerk / 1000 / 1000 /1000 * ori_ratio * ori_ratio * ori_ratio, p, v, a, j, ori_total_count);
		if (norm_ori > 1e-10)aris::dynamic::s_vc(3, p / norm_ori, relative_pa + 3, pa + 3);

		aris::dynamic::s_pa2pm(pa, pm);
		aris::dynamic::s_pm_dot_pm(begin_pm, pm, pm2);

		// 反解计算电机位置 //
		//target.model->generalMotionPool().at(0).setMpm(pm2);
		mvl_param->tool->setPm(*mvl_param->wobj, pm2);
		target.model->generalMotionPool().at(0).updMpm();
		if (target.model->solverPool().at(0).kinPos())return -1;

		////////////////////////////////////// log ///////////////////////////////////////
		//double pq[7];
		//aris::dynamic::s_pm2pq(*target.model->generalMotionPool().at(0).mpm(), pq);
		//target.controller->lout() << target.count << " " << pq[0] << " " << pq[1] << " " << pq[2] << " " << pq[3] << " " << pq[4] << " " << pq[5] << " " << pq[6] << "  ";

		//for (auto &cm : controller->motionPool())
		//{
		//	target.controller->lout() << "  " << cm.targetPos() << "  " << cm.actualPos() << "  " << cm.actualVel() << "  " << cm.actualCur() << "  ";
		//}
		//target.controller->lout() << "\n";
		//////////////////////////////////////////////////////////////////////////////////

		if (g_move_stop)
		{
			return -4;
		}
		return std::max(pos_total_count, ori_total_count) > target.count ? 1 : 0;
	}
	MoveL::~MoveL() = default;
	MoveL::MoveL(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"mvl\">"
			"	<GroupParam>"
			"		<Param name=\"pos_unit\" default=\"m\"/>"
			"		<UniqueParam default=\"pq\">"
			"			<Param name=\"pq\" default=\"{0,0,0,0,0,0,1}\"/>"
			"			<Param name=\"pm\" default=\"{1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1}\"/>"
			"			<GroupParam>"
			"				<Param name=\"pe\" default=\"{0,0,0,0,0,0}\"/>"
			"				<Param name=\"ori_unit\" default=\"rad\"/>"
			"				<Param name=\"eul_type\" default=\"321\"/>"
			"			</GroupParam>"
			"		</UniqueParam>"
			"		<Param name=\"acc\" default=\"0.1\"/>"
			"		<Param name=\"vel\" default=\"0.1\"/>"
			"		<Param name=\"dec\" default=\"0.1\"/>"
			"		<Param name=\"jerk\" default=\"10.0\"/>"
			"		<Param name=\"angular_acc\" default=\"0.1\"/>"
			"		<Param name=\"angular_vel\" default=\"0.1\"/>"
			"		<Param name=\"angular_dec\" default=\"0.1\"/>"
			"		<Param name=\"angular_jerk\" default=\"10.0\"/>"
			"		<Param name=\"tool\" default=\"tool0\"/>"
			"		<Param name=\"wobj\" default=\"wobj0\"/>"
			CHECK_PARAM_STRING
			"	</GroupParam>"
			"</Command>");
	}
	ARIS_DEFINE_BIG_FOUR_CPP(MoveL);


	//走圆弧//
	auto find_mid_pq(const std::map<std::string, std::string> &params, PlanTarget &target, double *mid_pq_out)->bool
	{
		double pos_unit;
		auto pos_unit_found = params.find("pos_unit");
		if (pos_unit_found == params.end()) pos_unit = 1.0;
		else if (pos_unit_found->second == "m")pos_unit = 1.0;
		else if (pos_unit_found->second == "mm")pos_unit = 0.001;
		else if (pos_unit_found->second == "cm")pos_unit = 0.01;
    else THROW_FILE_LINE("");


		for (auto cmd_param : params)
		{
			if (cmd_param.first == "mid_pq")
			{
				auto pq_mat = target.model->calculator().calculateExpression(cmd_param.second);
        if (pq_mat.size() != 7)THROW_FILE_LINE("");

				aris::dynamic::s_vc(7, pq_mat.data(), mid_pq_out);
				aris::dynamic::s_nv(3, pos_unit, mid_pq_out);
				return true;
			}
			else if (cmd_param.first == "mid_pm")
			{
				auto pm_mat = target.model->calculator().calculateExpression(cmd_param.second);

				if (pm_mat.size() != 16)THROW_FILE_LINE("");
				aris::dynamic::s_pm2pq(pm_mat.data(), mid_pq_out);
				aris::dynamic::s_nv(3, pos_unit, mid_pq_out);
				return true;
			}
			else if (cmd_param.first == "mid_pe")
			{
				double ori_unit;
				auto ori_unit_found = params.find("mid_ori_unit");
				if (ori_unit_found == params.end()) ori_unit = 1.0;
				else if (ori_unit_found->second == "rad")ori_unit = 1.0;
				else if (ori_unit_found->second == "degree")ori_unit = PI / 180.0;
				else THROW_FILE_LINE("");

				std::string eul_type;
				auto eul_type_found = params.find("mid_eul_type");
				if (eul_type_found == params.end()) eul_type = "321";
				else if (check_eul_validity(eul_type_found->second.data()))	eul_type = eul_type_found->second;
				else THROW_FILE_LINE("");

				auto pe_mat = target.model->calculator().calculateExpression(cmd_param.second);
				if (pe_mat.size() != 6)THROW_FILE_LINE("");
				aris::dynamic::s_nv(3, ori_unit, pe_mat.data() + 3);
				aris::dynamic::s_pe2pq(pe_mat.data(), mid_pq_out, eul_type.data());
				aris::dynamic::s_nv(3, pos_unit, mid_pq_out);
				return true;
			}
		}

		THROW_FILE_LINE("No mid pose input");
	}
	auto find_end_pq(const std::map<std::string, std::string> &params, PlanTarget &target, double *end_pq_out)->bool
	{
		double pos_unit;
		auto pos_unit_found = params.find("pos_unit");
		if (pos_unit_found == params.end()) pos_unit = 1.0;
		else if (pos_unit_found->second == "m")pos_unit = 1.0;
		else if (pos_unit_found->second == "mm")pos_unit = 0.001;
		else if (pos_unit_found->second == "cm")pos_unit = 0.01;
		else THROW_FILE_LINE("");

		for (auto cmd_param : params)
		{
			if (cmd_param.first == "end_pq")
			{
				auto pq_mat = target.model->calculator().calculateExpression(cmd_param.second);
				if (pq_mat.size() != 7)THROW_FILE_LINE("");
				aris::dynamic::s_vc(7, pq_mat.data(), end_pq_out);
				aris::dynamic::s_nv(3, pos_unit, end_pq_out);
				return true;
			}
			else if (cmd_param.first == "end_pm")
			{
				auto pm_mat = target.model->calculator().calculateExpression(cmd_param.second);
				if (pm_mat.size() != 16)THROW_FILE_LINE("");
				aris::dynamic::s_pm2pq(pm_mat.data(), end_pq_out);
				aris::dynamic::s_nv(3, pos_unit, end_pq_out);
				return true;
			}
			else if (cmd_param.first == "end_pe")
			{
				double ori_unit;
				auto ori_unit_found = params.find("mid_ori_unit");
				if (ori_unit_found == params.end()) ori_unit = 1.0;
				else if (ori_unit_found->second == "rad")ori_unit = 1.0;
				else if (ori_unit_found->second == "degree")ori_unit = PI / 180.0;
				else THROW_FILE_LINE("");

				std::string eul_type;
				auto eul_type_found = params.find("mid_eul_type");
				if (eul_type_found == params.end()) eul_type = "321";
				else if (check_eul_validity(eul_type_found->second.data()))	eul_type = eul_type_found->second;
				else THROW_FILE_LINE("");

				auto pe_mat = target.model->calculator().calculateExpression(cmd_param.second);
				if (pe_mat.size() != 6)THROW_FILE_LINE("");
				aris::dynamic::s_nv(3, ori_unit, pe_mat.data() + 3);
				aris::dynamic::s_pe2pq(pe_mat.data(), end_pq_out, eul_type.data());
				aris::dynamic::s_nv(3, pos_unit, end_pq_out);
				return true;
			}
		}

		THROW_FILE_LINE("No end pose input");
	}
	void slerp(double starting[4], double ending[4], double result[4], double t)
	{
		double cosa = starting[0] * ending[0] + starting[1] * ending[1] + starting[2] * ending[2] + starting[3] * ending[3];

		// If the dot product is negative, the quaternions have opposite handed-ness and slerp won't take
		// the shorter path. Fix by reversing one quaternion.
		if (cosa < 0.0f)
		{
			ending[0] = -ending[0];
			ending[1] = -ending[1];
			ending[2] = -ending[2];
			ending[3] = -ending[3];
			cosa = -cosa;
		}

		double k0, k1;

		// If the inputs are too close for comfort, linearly interpolate
		if (cosa > 0.9995f)
		{
			k0 = 1.0f - t;
			k1 = t;
		}
		else
		{
			double sina = sqrt(1.0f - cosa * cosa);
			double a = atan2(sina, cosa);
			k0 = sin((1.0f - t)*a) / sina;
			k1 = sin(t*a) / sina;
		}
		result[0] = starting[0] * k0 + ending[0] * k1;
		result[1] = starting[1] * k0 + ending[1] * k1;
		result[2] = starting[2] * k0 + ending[2] * k1;
		result[3] = starting[3] * k0 + ending[3] * k1;
	}
	struct MoveCParam
	{
		std::vector<double> joint_vel, joint_acc, joint_dec, ee_begin_pq, ee_mid_pq, ee_end_pq, joint_pos_begin, joint_pos_end;
		Size total_count[6];
		double acc, vel, dec, jerk;
		double angular_acc, angular_vel, angular_dec, angular_jerk;
		aris::dynamic::Marker *tool, *wobj;
	};
	struct MoveC::Imp {};
	auto MoveC::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		MoveCParam mvc_param;
		mvc_param.ee_begin_pq.resize(7);
		mvc_param.ee_mid_pq.resize(7);
		mvc_param.ee_end_pq.resize(7);
		if (!find_mid_pq(params, target, mvc_param.ee_mid_pq.data()))THROW_FILE_LINE("");
		if (!find_end_pq(params, target, mvc_param.ee_end_pq.data()))THROW_FILE_LINE("");

		auto tool_is_string = params.at("tool");
		auto wobj_is_string = params.at("wobj");

		mvc_param.tool = &*target.model->generalMotionPool()[0].makI().fatherPart().markerPool().findByName(params.at("tool"));
		mvc_param.wobj = &*target.model->generalMotionPool()[0].makJ().fatherPart().markerPool().findByName(params.at("wobj"));

		for (auto cmd_param : params)
		{
			if (cmd_param.first == "acc")
			{
				mvc_param.acc = std::stod(cmd_param.second);
			}
			else if (cmd_param.first == "vel")
			{
				mvc_param.vel = std::stod(cmd_param.second);
			}
			else if (cmd_param.first == "dec")
			{
				mvc_param.dec = std::stod(cmd_param.second);
			}
			else if (cmd_param.first == "jerk")
			{
				mvc_param.jerk = std::stod(cmd_param.second);
			}
			else if (cmd_param.first == "angular_acc")
			{
				mvc_param.angular_acc = std::stod(cmd_param.second);
			}
			else if (cmd_param.first == "angular_vel")
			{
				mvc_param.angular_vel = std::stod(cmd_param.second);
			}
			else if (cmd_param.first == "angular_dec")
			{
				mvc_param.angular_dec = std::stod(cmd_param.second);
			}
			else if (cmd_param.first == "angular_jerk")
			{
				mvc_param.angular_jerk = std::stod(cmd_param.second);
			}
		}

		for (auto &option : target.mot_options)	option |= Plan::USE_TARGET_POS;
		target.param = mvc_param;

		std::vector<std::pair<std::string, std::any>> ret;
		target.ret = ret;

	}
	auto MoveC::executeRT(PlanTarget &target)->int
	{
		auto &mvc_param = std::any_cast<MoveCParam &>(target.param);
		auto controller = target.controller;

		// 取得起始位置 //
		static double pos_ratio, ori_ratio;
		static double A[9], b[3], C[3], R, theta, ori_theta;
		double p, v, a, j;
		aris::Size pos_total_count, ori_total_count;

		if (target.count == 1)
		{
			//target.model->generalMotionPool().at(0).getMpq(mvc_param.ee_begin_pq.data());
			mvc_param.tool->getPq(*mvc_param.wobj, mvc_param.ee_begin_pq.data());

			//排除3点共线的情况//		
			std::vector<double> mul_cross(3);
			std::vector<double> p1(3);
			std::vector<double> p2(3);
			std::vector<double> p3(3);
			std::copy(mvc_param.ee_begin_pq.data(), mvc_param.ee_begin_pq.data() + 3, p1.data());
			std::copy(mvc_param.ee_mid_pq.data(), mvc_param.ee_mid_pq.data() + 3, p2.data());
			std::copy(mvc_param.ee_end_pq.data(), mvc_param.ee_end_pq.data() + 3, p3.data());
			s_vs(3, p2.data(), p3.data());
			s_vs(3, p1.data(), p2.data());
			s_c3(p2.data(), p3.data(), mul_cross.data());
			double normv = aris::dynamic::s_norm(3, mul_cross.data());
			if (normv <= 1e-10) return -1;
			
			//通过AC=b 解出圆心C//
			A[0] = 2 * (mvc_param.ee_begin_pq[0] - mvc_param.ee_mid_pq[0]);
			A[1] = 2 * (mvc_param.ee_begin_pq[1] - mvc_param.ee_mid_pq[1]);
			A[2] = 2 * (mvc_param.ee_begin_pq[2] - mvc_param.ee_mid_pq[2]);
			A[3] = 2 * (mvc_param.ee_mid_pq[0] - mvc_param.ee_end_pq[0]);
			A[4] = 2 * (mvc_param.ee_mid_pq[1] - mvc_param.ee_end_pq[1]);
			A[5] = 2 * (mvc_param.ee_mid_pq[2] - mvc_param.ee_end_pq[2]);
			A[6] = (A[1] * A[5] - A[4] * A[2]) / 4;
			A[7] = -(A[0] * A[5] - A[3] * A[2]) / 4;
			A[8] = (A[0] * A[4] - A[3] * A[1]) / 4;
			b[0] = pow(mvc_param.ee_begin_pq[0], 2) + pow(mvc_param.ee_begin_pq[1], 2) + pow(mvc_param.ee_begin_pq[2], 2) - pow(mvc_param.ee_mid_pq[0], 2) - pow(mvc_param.ee_mid_pq[1], 2) - pow(mvc_param.ee_mid_pq[2], 2);
			b[1] = pow(mvc_param.ee_mid_pq[0], 2) + pow(mvc_param.ee_mid_pq[1], 2) + pow(mvc_param.ee_mid_pq[2], 2) - pow(mvc_param.ee_end_pq[0], 2) - pow(mvc_param.ee_end_pq[1], 2) - pow(mvc_param.ee_end_pq[2], 2);
			b[2] = A[6] * mvc_param.ee_begin_pq[0] + A[7] * mvc_param.ee_begin_pq[1] + A[8] * mvc_param.ee_begin_pq[2];
			
			//解线性方程组
			{
				double pinv[9];
				std::vector<double> U_vec(9);
				auto U = U_vec.data();
				double tau[3];
				aris::Size p[3];
				aris::Size rank;
				s_householder_utp(3, 3, A, U, tau, p, rank, 1e-10);
				double tau2[3];
				s_householder_utp2pinv(3, 3, rank, U, tau, p, pinv, tau2, 1e-10);
				//获取圆心
				s_mm(3, 1, 3, pinv, b, C);
				//获取半径
				R = sqrt(pow(mvc_param.ee_begin_pq[0] - C[0], 2) + pow(mvc_param.ee_begin_pq[1] - C[1], 2) + pow(mvc_param.ee_begin_pq[2] - C[2], 2));
			}
				
			//求旋转角度theta//	
			double u, v, w;
			double u1, v1, w1;
			u = A[6];
			v = A[7];
			w = A[8];
			u1 = (mvc_param.ee_begin_pq[1] - C[1])*(mvc_param.ee_end_pq[2] - mvc_param.ee_begin_pq[2]) - (mvc_param.ee_begin_pq[2] - C[2])*(mvc_param.ee_end_pq[1] - mvc_param.ee_begin_pq[1]);
			v1 = (mvc_param.ee_begin_pq[2] - C[2])*(mvc_param.ee_end_pq[0] - mvc_param.ee_begin_pq[0]) - (mvc_param.ee_begin_pq[0] - C[0])*(mvc_param.ee_end_pq[2] - mvc_param.ee_begin_pq[2]);
			w1 = (mvc_param.ee_begin_pq[0] - C[0])*(mvc_param.ee_end_pq[1] - mvc_param.ee_begin_pq[1]) - (mvc_param.ee_begin_pq[1] - C[1])*(mvc_param.ee_end_pq[0] - mvc_param.ee_begin_pq[0]);

			double H = u * u1 + v * v1 + w * w1;
			
			// 判断theta 与 pi 的关系
			if (H >= 0)
			{
				theta = 2 * asin(sqrt(pow((mvc_param.ee_end_pq[0] - mvc_param.ee_begin_pq[0]), 2) + pow((mvc_param.ee_end_pq[1] - mvc_param.ee_begin_pq[1]), 2) + pow((mvc_param.ee_end_pq[2] - mvc_param.ee_begin_pq[2]), 2)) / (2 * R));
			}
			else
			{
				theta = 2 * 3.1415 - 2 * asin(sqrt(pow((mvc_param.ee_end_pq[0] - mvc_param.ee_begin_pq[0]), 2) + pow((mvc_param.ee_end_pq[1] - mvc_param.ee_begin_pq[1]), 2) + pow((mvc_param.ee_end_pq[2] - mvc_param.ee_begin_pq[2]), 2)) / (2 * R));
			}
			
			double normv_begin_pq = aris::dynamic::s_norm(4, mvc_param.ee_begin_pq.data() + 3);
			double normv_end_pq = aris::dynamic::s_norm(4, mvc_param.ee_end_pq.data() + 3);

			ori_theta = std::abs((mvc_param.ee_begin_pq[3] * mvc_param.ee_end_pq[3] + mvc_param.ee_begin_pq[4] * mvc_param.ee_end_pq[4] + mvc_param.ee_begin_pq[5] * mvc_param.ee_end_pq[5] + mvc_param.ee_begin_pq[6] * mvc_param.ee_end_pq[6])/ (normv_begin_pq* normv_end_pq));

			//aris::plan::moveAbsolute(target.count, 0.0, theta, mvc_param.vel / 1000 / R, mvc_param.acc / 1000 / 1000 / R, mvc_param.dec / 1000 / 1000 / R, p, v, a, pos_total_count);
			//aris::plan::moveAbsolute(target.count, 0.0, 1.0, mvc_param.angular_vel / 1000 / ori_theta / 2.0, mvc_param.angular_acc / 1000 / 1000 / ori_theta / 2.0, mvc_param.angular_dec / 1000 / 1000 / ori_theta / 2.0, p, v, a, ori_total_count);

			traplan::sCurve(target.count, 0.0, theta, mvc_param.vel / 1000 / R, mvc_param.acc / 1000 / 1000 / R, mvc_param.jerk / 1000 / 1000 /1000 / R, p, v, a, j, pos_total_count);
			traplan::sCurve(target.count, 0.0, 1.0, mvc_param.angular_vel / 1000 / ori_theta / 2.0, mvc_param.angular_acc / 1000 / 1000 / ori_theta / 2.0, mvc_param.angular_jerk / 1000 / 1000 /1000 / ori_theta / 2.0, p, v, a, j, ori_total_count);

			pos_ratio = pos_total_count < ori_total_count ? double(pos_total_count) / ori_total_count : 1.0;
			ori_ratio = ori_total_count < pos_total_count ? double(ori_total_count) / pos_total_count : 1.0;
		}

		//位置规划//
		double w[3], pmr[16], pqt[7];
		aris::dynamic::s_vc(3, A + 6, w);
		auto normv = aris::dynamic::s_norm(3, w);
		if (std::abs(normv) > 1e-10)aris::dynamic::s_nv(3, 1 / normv, w); //数乘
		//aris::plan::moveAbsolute(target.count, 0.0, theta, mvc_param.vel / 1000 / R * pos_ratio, mvc_param.acc / 1000 / 1000 / R * pos_ratio * pos_ratio, mvc_param.dec / 1000 / 1000 / R * pos_ratio* pos_ratio, p, v, a, pos_total_count);
		traplan::sCurve(target.count, 0.0, theta, mvc_param.vel / 1000 / R * pos_ratio, mvc_param.acc / 1000 / 1000 / R * pos_ratio * pos_ratio, mvc_param.jerk / 1000 / 1000 / 1000 / R * pos_ratio* pos_ratio* pos_ratio, p, v, a, j, pos_total_count);

		double pqr[7]{ C[0], C[1], C[2], w[0] * sin(p / 2.0), w[1] * sin(p / 2.0), w[2] * sin(p / 2.0), cos(p / 2.0) };
		double pos[4]{ mvc_param.ee_begin_pq[0] - C[0], mvc_param.ee_begin_pq[1] - C[1], mvc_param.ee_begin_pq[2] - C[2], 1};
		aris::dynamic::s_pq2pm(pqr, pmr);
		s_mm(4, 1, 4, pmr, aris::dynamic::RowMajor{ 4 }, pos, 1, pqt, 1);

		//姿态规划//
		//aris::plan::moveAbsolute(target.count, 0.0, 1.0, mvc_param.angular_vel / 1000 / ori_theta / 2.0 * ori_ratio, mvc_param.angular_acc / 1000 / 1000 / ori_theta / 2.0 * ori_ratio * ori_ratio, mvc_param.angular_dec / 1000 / 1000 / ori_theta / 2.0* ori_ratio * ori_ratio, p, v, a, ori_total_count);
		traplan::sCurve(target.count, 0.0, 1.0, mvc_param.angular_vel / 1000 / ori_theta / 2.0 * ori_ratio, mvc_param.angular_acc / 1000 / 1000 / ori_theta / 2.0 * ori_ratio * ori_ratio, mvc_param.angular_jerk / 1000 / 1000 /1000 / ori_theta / 2.0* ori_ratio * ori_ratio* ori_ratio, p, v, a, j, ori_total_count);
		slerp(mvc_param.ee_begin_pq.data() + 3, mvc_param.ee_end_pq.data() + 3, pqt + 3, p);
		
		//雅克比矩阵判断奇异点//
		{
			auto &fwd = dynamic_cast<aris::dynamic::ForwardKinematicSolver&>(target.model->solverPool()[1]);
			fwd.cptJacobiWrtEE();
			//QR分解求方程的解
			double U[36], tau[6];
			aris::Size p[6];
			Size rank;
			//auto inline s_householder_utp(Size m, Size n, const double *A, AType a_t, double *U, UType u_t, double *tau, TauType tau_t, Size *p, Size &rank, double zero_check = 1e-10)noexcept->void
			//A为输入
			s_householder_utp(6, 6, fwd.Jf(), U, tau, p, rank, 1e-3);
			if(rank < 6)return -1002;
		}

		// set目标位置，并进行运动学反解 //
		//target.model->generalMotionPool().at(0).setMpq(pqt);
		mvc_param.tool->setPq(*mvc_param.wobj, pqt);
		target.model->generalMotionPool().at(0).updMpm();

		if (target.model->solverPool().at(0).kinPos())return -1;

		////////////////////////////////////// log ///////////////////////////////////////
		auto &lout = controller->lout();
		{
			lout << target.count << " " << pqt[0] << " " << pqt[1] << " " << pqt[2] << " " << pqt[3] << " " << pqt[4] << " " << pqt[5] << " " << pqt[6] << "  ";
			/*
			for (Size i = 0; i < 6; i++)
			{
				lout << controller->motionAtAbs(i).targetPos() << ",";
				lout << controller->motionAtAbs(i).actualPos() << ",";
				lout << controller->motionAtAbs(i).actualVel() << ",";
			}
			*/
			lout << std::endl;
		}
		
		if (g_move_stop)
		{
			return -4;
		}
		return std::max(pos_total_count, ori_total_count) > target.count ? 1 : 0;
	}
	MoveC::~MoveC() = default;
	MoveC::MoveC(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"mvc\">"
			"	<GroupParam>"
			"		<Param name=\"pos_unit\" default=\"m\"/>"
			"		<UniqueParam default=\"mid_pq\">"
			"			<Param name=\"mid_pq\" default=\"{0,0,0,0,0,0,1}\"/>"
			"			<Param name=\"mid_pm\" default=\"{1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1}\"/>"
			"			<GroupParam>"
			"				<Param name=\"mid_pe\" default=\"{0,0,0,0,0,0}\"/>"
			"				<Param name=\"mid_ori_unit\" default=\"rad\"/>"
			"				<Param name=\"mid_eul_type\" default=\"321\"/>"
			"			</GroupParam>"
			"		</UniqueParam>"
			"		<UniqueParam default=\"end_pq\">"
			"			<Param name=\"end_pq\" default=\"{0,0,0,0,0,0,1}\"/>"
			"			<Param name=\"end_pm\" default=\"{1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1}\"/>"
			"			<GroupParam>"
			"				<Param name=\"end_pe\" default=\"{0,0,0,0,0,0}\"/>"
			"				<Param name=\"end_ori_unit\" default=\"rad\"/>"
			"				<Param name=\"end_eul_type\" default=\"321\"/>"
			"			</GroupParam>"
			"		</UniqueParam>"
			"		<Param name=\"acc\" default=\"0.1\"/>"
			"		<Param name=\"vel\" default=\"0.1\"/>"
			"		<Param name=\"dec\" default=\"0.1\"/>"
			"		<Param name=\"jerk\" default=\"0.1\"/>"
			"		<Param name=\"angular_acc\" default=\"0.1\"/>"
			"		<Param name=\"angular_vel\" default=\"0.1\"/>"
			"		<Param name=\"angular_dec\" default=\"0.1\"/>"
			"		<Param name=\"angular_jerk\" default=\"0.1\"/>"
			"		<Param name=\"tool\" default=\"tool0\"/>"
			"		<Param name=\"wobj\" default=\"wobj0\"/>"
			"	</GroupParam>"
			"</Command>");
	}
	ARIS_DEFINE_BIG_FOUR_CPP(MoveC);

	
	//复现文件位置//
	struct MoveFParam
	{
		std::vector<Size> total_count_vec;
		std::vector<double> axis_begin_pos_vec;
		std::vector<double> axis_first_pos_vec;
		std::vector<std::vector<double>> pos_vec;
		double vel, acc, dec;
		std::string path;
	};
	auto MoveF::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		MoveFParam param;
		param.vel = std::stod(params.at("vel"));
		param.acc = std::stod(params.at("acc"));
		param.dec = std::stod(params.at("dec"));
		param.path = params.at("path");

        std::cout<<param.path;
		param.total_count_vec.resize(6, 0);
		param.axis_begin_pos_vec.resize(6, 0.0);
		param.axis_first_pos_vec.resize(6, 0.0);

		param.pos_vec.resize(6, std::vector<double>(1, 0.0));
		//std::cout << "size:" << param.pos_vec.size() << std::endl;
		//初始化pos_vec//
		for (int j = 0; j < param.pos_vec.size(); j++)
		{
			param.pos_vec[j].clear();
		}

		//定义读取log文件的输入流oplog//
		std::ifstream oplog;
		int cal = 0;
		oplog.open(param.path);

		//以下检查是否成功读取文件//
		if (!oplog)
		{
			throw std::runtime_error("fail to open the file");
		}
		while (!oplog.eof())
		{
			for (int j = 0; j < param.pos_vec.size(); j++)
			{
				double data;
				oplog >> data;
				param.pos_vec[j].push_back(data);
			}
		}
		oplog.close();
		//oplog.clear();
		for (int j = 0; j < param.pos_vec.size(); j++)
		{
			param.pos_vec[j].pop_back();
			param.axis_first_pos_vec[j] = param.pos_vec[j][0];
		}

		target.param = param;
		std::fill(target.mot_options.begin(), target.mot_options.end(),
			Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER|
			Plan::NOT_CHECK_POS_FOLLOWING_ERROR);
		std::vector<std::pair<std::string, std::any>> ret;
		target.ret = ret;
	}
	auto MoveF::executeRT(PlanTarget &target)->int
	{
		auto controller = target.controller;
		auto &param = std::any_cast<MoveFParam&>(target.param);
		
		double p, v, a;
		aris::Size t_count;
		static aris::Size first_total_count = 1;
		aris::Size total_count = 1;
		aris::Size return_value = 0;

		// 获取6个电机初始位置 //
		if (target.count == 1)
		{
			for (int i = 0; i < 6; i++)
			{
				param.axis_begin_pos_vec[i] = target.model->motionPool().at(i).mp();
			}
			for (int i = 0; i < 6; i++)
			{
				// 梯形规划到log开始点 //
				aris::plan::moveAbsolute(target.count, param.axis_begin_pos_vec[i], param.axis_first_pos_vec[i], param.vel / 1000, param.acc / 1000 / 1000, param.dec / 1000 / 1000, p, v, a, t_count);
				first_total_count = std::max(first_total_count, t_count);
			}
		}
		
		// 机械臂走到log开始点 //
		if (target.count <= first_total_count)
		{
			for (int i = 0; i < 6; i++)
			{
				// 在第一个周期走梯形规划复位
				aris::plan::moveAbsolute(target.count, param.axis_begin_pos_vec[i], param.axis_first_pos_vec[i], param.vel / 1000, param.acc / 1000 / 1000, param.dec / 1000 / 1000, p, v, a, t_count);
				controller->motionAtAbs(i).setTargetPos(p);
				target.model->motionPool().at(i).setMp(p);
			}
		}
		
		// 机械臂开始从头到尾复现log中点 //
		if (target.count > first_total_count)
		{
			for (int i = 0; i < 6; i++)
			{
				controller->motionAtAbs(i).setTargetPos(param.pos_vec[i][target.count - first_total_count]);
				target.model->motionPool().at(i).setMp(param.pos_vec[i][target.count - first_total_count]);
			}
		}
		if (target.model->solverPool().at(1).kinPos())return -1;

		//输出6个轴的实时位置log文件//
		auto &lout = controller->lout();
		for (int i = 0; i < 6; i++)
		{
			lout << controller->motionAtAbs(i).actualPos() << " ";//第一列数字必须是位置
		}
		lout << std::endl;

		return target.count > (first_total_count - 2 + param.pos_vec[0].size()) ? 0 : 1;

	}
	auto MoveF::collectNrt(PlanTarget &target)->void {}
	MoveF::MoveF(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"mvf\">"
			"	<GroupParam>"
            "		<Param name=\"path\" default=\"/home/kaanh/Desktop/build-Kaanh-gk-Desktop_Qt_5_11_2_GCC_64bit-Default/log/motion_replay.txt\"/>"
            "		<Param name=\"vel\" default=\"0.5\" abbreviation=\"v\"/>"
            "		<Param name=\"acc\" default=\"0.6\" abbreviation=\"a\"/>"
            "		<Param name=\"dec\" default=\"0.6\" abbreviation=\"d\"/>"
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
		
		std::vector<std::pair<std::string, std::any>> ret;
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
				if (imp_->increase_count < 0 || imp_->increase_count>1e5)THROW_FILE_LINE("");

				auto mat = target.model->calculator().calculateExpression(params.at("vel"));
				if (mat.size() != 6)THROW_FILE_LINE("");
				std::copy(mat.begin(), mat.end(), imp_->vel);

				mat = target.model->calculator().calculateExpression(params.at("acc"));
				if (mat.size() != 6)THROW_FILE_LINE("");
				std::copy(mat.begin(), mat.end(), imp_->acc);

				mat = target.model->calculator().calculateExpression(params.at("dec"));
				if (mat.size() != 6)THROW_FILE_LINE("");
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
				velocity = std::max(std::min(100, velocity), 0);
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

		std::vector<std::pair<std::string, std::any>> ret;
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
				if (imp_->increase_count < 0 || imp_->increase_count>1e5)THROW_FILE_LINE("");
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
				velocity = std::max(std::min(100, velocity), 0);
				imp_->s2_nrt.vel_percent = velocity;

				imp_->s2_nrt.is_increase.assign(imp_->s2_nrt.is_increase.size(), 0);
				imp_->s2_nrt.is_increase[std::stoi(params.at("motion_id"))] = std::max(std::min(1, std::stoi(params.at("direction"))), -1) * imp_->increase_count;

				imp_->s2_nrt.jogj_is_running = true;

				target.option |= NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION | NOT_PRINT_CMD_INFO | NOT_LOG_CMD_INFO;
				jogj_is_changing = true;
				while (jogj_is_changing.load())std::this_thread::sleep_for(std::chrono::milliseconds(1));
			}
		}

	}
	auto JogJ::executeRT(PlanTarget &target)->int
	{
		//获取驱动//
		auto controller = target.controller;

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
		for (int i = 0; i < imp_->p_now.size(); i++)
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

		// 运动学正解//
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
			cout << "------------------------------------------" << std::endl;
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


#define JOGJ_PARAM_STRING \
		"	<UniqueParam>"\
		"		<GroupParam>"\
		"			<Param name=\"increase_count\" default=\"500\"/>"\
		"			<Param name=\"vel\" default=\"1\" abbreviation=\"v\"/>"\
		"			<Param name=\"acc\" default=\"5\" abbreviation=\"a\"/>"\
		"			<Param name=\"dec\" default=\"5\" abbreviation=\"d\"/>"\
		"			<Param name=\"vel_percent\" default=\"10\"/>"\
		"			<Param name=\"direction\" default=\"1\"/>"\
		"		</GroupParam>"\
		"		<Param name=\"stop\"/>"\
		"	</UniqueParam>"
	// 示教运动--关节1点动 //
	struct JogJParam 
	{
		int motion_id;
		double vel, acc, dec;
		double p_now, v_now, a_now, target_pos, max_vel;
		int increase_status;
		int increase_count;
		int vel_percent;
		static std::atomic_int32_t j1_count, j2_count, j3_count, j4_count, j5_count, j6_count, j7_count;
	};
	template<typename JogType>
	auto set_jogj_input_param(JogType* this_p, const std::map<std::string, std::string> &cmd_params, PlanTarget &target, JogJParam &param, std::atomic_int32_t& j_count)->void
	{
		auto&cs = aris::server::ControlServer::instance();
		auto c = target.controller;
		param.p_now = 0.0;
		param.v_now = 0.0;
		param.a_now = 0.0;
		param.target_pos = 0.0;
		param.max_vel = 0.0;
		param.increase_status = 0;
		param.vel_percent = 0;

		for (auto &p : cmd_params)
		{
			if (p.first == "increase_count")
			{
				param.increase_count = std::stoi(cmd_params.at("increase_count"));
				if (param.increase_count < 0 || param.increase_count>1e5)THROW_FILE_LINE("");

				param.vel = c->motionPool().at(param.motion_id).maxVel()*g_vel.getspeed().w_percent;
				param.acc = std::min(std::max(std::stod(cmd_params.at("acc")), 0.0), 1.0)*c->motionPool().at(param.motion_id).maxAcc();
				param.dec = std::min(std::max(std::stod(cmd_params.at("dec")), 0.0), 1.0)*c->motionPool().at(param.motion_id).maxAcc();

				auto velocity = std::stoi(cmd_params.at("vel_percent"));
				velocity = std::max(std::min(100, velocity), 0);
				param.vel_percent = velocity;
				param.increase_status = std::max(std::min(1, std::stoi(cmd_params.at("direction"))), -1);

				std::shared_ptr<aris::plan::PlanTarget> planptr = cs.currentExecuteTarget();
				//当前有指令在执行//
				if (planptr && planptr->plan.get() != this_p)throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + "Other command is running");
				if (j_count.exchange(param.increase_count))
				{
					target.option |= aris::plan::Plan::Option::NOT_RUN_EXECUTE_FUNCTION | aris::plan::Plan::Option::NOT_RUN_COLLECT_FUNCTION;
				}
				else
				{
					std::fill(target.mot_options.begin(), target.mot_options.end(), aris::plan::Plan::MotionOption::NOT_CHECK_POS_FOLLOWING_ERROR | aris::plan::Plan::MotionOption::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER | aris::plan::Plan::MotionOption::USE_TARGET_POS | aris::plan::Plan::MotionOption::NOT_CHECK_ENABLE);
					target.mot_options[param.motion_id] = aris::plan::Plan::MotionOption::NOT_CHECK_POS_FOLLOWING_ERROR | aris::plan::Plan::MotionOption::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER | aris::plan::Plan::MotionOption::USE_TARGET_POS;
				}
			}
			else if (p.first == "stop")
			{
				j_count.exchange(0);
				target.option |= aris::plan::Plan::Option::NOT_RUN_EXECUTE_FUNCTION | aris::plan::Plan::Option::NOT_RUN_COLLECT_FUNCTION;
			}
		}
	}
	std::atomic_int32_t JogJParam::j1_count = 0;
	auto JogJ1::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{	
		JogJParam param;
		std::vector<std::pair<std::string, std::any>> ret;
		target.ret = ret;

		param.motion_id = 0;

		set_jogj_input_param(this, params, target, param, param.j1_count);

		target.param = param;
	}
	auto JogJ1::executeRT(PlanTarget &target)->int
	{
		//获取驱动//
		auto controller = target.controller;
		auto &param = std::any_cast<JogJParam&>(target.param);

		// get current pos //
		if (target.count == 1)
		{
			/*
			imp_->target_pos = controller->motionAtAbs(imp_->motion_id).actualPos();
			imp_->p_now = controller->motionAtAbs(imp_->motion_id).actualPos();
			imp_->v_now = controller->motionAtAbs(imp_->motion_id).actualVel();
			imp_->a_now = 0.0;
			*/
			param.target_pos = target.model->motionPool().at(param.motion_id).mp();
			param.p_now = target.model->motionPool()[param.motion_id].mp();
			param.v_now = target.model->motionPool()[param.motion_id].mv();
			param.a_now = 0.0;

		}

		// init status and calculate target pos and max vel //
		param.max_vel = param.vel*1.0*param.vel_percent / 100.0;
		
		param.target_pos += aris::dynamic::s_sgn(param.increase_status)*param.max_vel * 1e-3;

		// 梯形轨迹规划 //
		static double p_next, v_next, a_next;

		aris::Size t;
		auto finished = aris::plan::moveAbsolute2(param.p_now, param.v_now, param.a_now
			, param.target_pos, 0.0, 0.0
			, param.max_vel, param.acc, param.dec
			, 1e-3, 1e-10, p_next, v_next, a_next, t);

		if (param.j1_count == 0)
		{
			param.increase_status = 0;
		}
		else
		{
			--param.j1_count;
		}

		target.model->motionPool().at(param.motion_id).setMp(p_next);
		//controller->motionAtAbs(imp_->motion_id).setTargetPos(p_next);
		param.p_now = p_next;
		param.v_now = v_next;
		param.a_now = a_next;

		// 运动学正解//
		if (target.model->solverPool().at(1).kinPos())return -1;

		// 打印 //
		auto &cout = controller->mout();
		if (target.count % 10 == 0)
		{
			cout << param.j1_count << "  ";
			cout << param.target_pos << "  ";
			cout << param.p_now << "  ";
			cout << param.v_now << "  ";
			cout << param.a_now << "  ";
			cout << "------------------------------------------" << std::endl;
		}

		// log //
		auto &lout = controller->lout();
		{
			lout << param.target_pos << " ";
			lout << param.p_now << " ";
			lout << param.v_now << " ";
			lout << param.a_now << " ";
			lout << controller->motionAtAbs(param.motion_id).actualPos() << " ";
			lout << controller->motionAtAbs(param.motion_id).actualVel() << " ";
			lout << std::endl;
		}

		return finished;
	}
	auto JogJ1::collectNrt(PlanTarget &target)->void 
	{
		JogJParam::j1_count = 0;
		if (target.ret_code < 0)
		{
			JogJParam::j1_count.store(0);
		}
	}
	JogJ1::~JogJ1() = default;
	JogJ1::JogJ1(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"j1\">"
				JOGJ_PARAM_STRING
			"</Command>");
	}


	// 示教运动--关节2点动 //
	std::atomic_int32_t JogJParam::j2_count = 0;
	auto JogJ2::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		JogJParam param;
		std::vector<std::pair<std::string, std::any>> ret;
		target.ret = ret;

		param.motion_id = 1;

		set_jogj_input_param(this, params, target, param, param.j2_count);

		target.param = param;
	}
	auto JogJ2::executeRT(PlanTarget &target)->int
	{
		//获取驱动//
		auto controller = target.controller;
		auto &param = std::any_cast<JogJParam&>(target.param);

		// get current pos //
		if (target.count == 1)
		{
			/*
			imp_->target_pos = controller->motionAtAbs(imp_->motion_id).actualPos();
			imp_->p_now = controller->motionAtAbs(imp_->motion_id).actualPos();
			imp_->v_now = controller->motionAtAbs(imp_->motion_id).actualVel();
			imp_->a_now = 0.0;
			*/
			param.target_pos = target.model->motionPool().at(param.motion_id).mp();
			param.p_now = target.model->motionPool()[param.motion_id].mp();
			param.v_now = target.model->motionPool()[param.motion_id].mv();
			param.a_now = 0.0;

		}

		// init status and calculate target pos and max vel //
		param.max_vel = param.vel*1.0*param.vel_percent / 100.0;

		param.target_pos += aris::dynamic::s_sgn(param.increase_status)*param.max_vel * 1e-3;

		// 梯形轨迹规划 //
		static double p_next, v_next, a_next;

		aris::Size t;
		auto finished = aris::plan::moveAbsolute2(param.p_now, param.v_now, param.a_now
			, param.target_pos, 0.0, 0.0
			, param.max_vel, param.acc, param.dec
			, 1e-3, 1e-10, p_next, v_next, a_next, t);

		if (param.j2_count == 0)
		{
			param.increase_status = 0;
		}
		else
		{
			--param.j2_count;
		}

		target.model->motionPool().at(param.motion_id).setMp(p_next);
		//controller->motionAtAbs(imp_->motion_id).setTargetPos(p_next);
		param.p_now = p_next;
		param.v_now = v_next;
		param.a_now = a_next;

		// 运动学正解//
		if (target.model->solverPool().at(1).kinPos())return -1;

		// 打印 //
		auto &cout = controller->mout();
		if (target.count % 10 == 0)
		{
			cout << param.j2_count << "  ";
			cout << param.target_pos << "  ";
			cout << param.p_now << "  ";
			cout << param.v_now << "  ";
			cout << param.a_now << "  ";
			cout << "------------------------------------------" << std::endl;
		}

		// log //
		auto &lout = controller->lout();
		{
			lout << param.target_pos << " ";
			lout << param.p_now << " ";
			lout << param.v_now << " ";
			lout << param.a_now << " ";
			lout << controller->motionAtAbs(param.motion_id).actualPos() << " ";
			lout << controller->motionAtAbs(param.motion_id).actualVel() << " ";
			lout << std::endl;
		}

		return finished;
	}
	auto JogJ2::collectNrt(PlanTarget &target)->void 
	{
		JogJParam::j2_count = 0;
		if (target.ret_code < 0)
		{
			JogJParam::j2_count.store(0);
		}
	}
	JogJ2::~JogJ2() = default;
	JogJ2::JogJ2(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"j2\">"
				JOGJ_PARAM_STRING
			"</Command>");
	}


	// 示教运动--关节3点动 //
	std::atomic_int32_t JogJParam::j3_count = 0;
	auto JogJ3::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		JogJParam param;
		std::vector<std::pair<std::string, std::any>> ret;
		target.ret = ret;

		param.motion_id = 2;
		set_jogj_input_param(this, params, target, param, param.j3_count);

		target.param = param;
	}
	auto JogJ3::executeRT(PlanTarget &target)->int
	{
		//获取驱动//
		auto controller = target.controller;
		auto &param = std::any_cast<JogJParam&>(target.param);

		// get current pos //
		if (target.count == 1)
		{
			/*
			imp_->target_pos = controller->motionAtAbs(imp_->motion_id).actualPos();
			imp_->p_now = controller->motionAtAbs(imp_->motion_id).actualPos();
			imp_->v_now = controller->motionAtAbs(imp_->motion_id).actualVel();
			imp_->a_now = 0.0;
			*/
			param.target_pos = target.model->motionPool().at(param.motion_id).mp();
			param.p_now = target.model->motionPool()[param.motion_id].mp();
			param.v_now = target.model->motionPool()[param.motion_id].mv();
			param.a_now = 0.0;

		}

		// init status and calculate target pos and max vel //
		param.max_vel = param.vel*1.0*param.vel_percent / 100.0;

		param.target_pos += aris::dynamic::s_sgn(param.increase_status)*param.max_vel * 1e-3;

		// 梯形轨迹规划 //
		static double p_next, v_next, a_next;

		aris::Size t;
		auto finished = aris::plan::moveAbsolute2(param.p_now, param.v_now, param.a_now
			, param.target_pos, 0.0, 0.0
			, param.max_vel, param.acc, param.dec
			, 1e-3, 1e-10, p_next, v_next, a_next, t);

		if (param.j3_count == 0)
		{
			param.increase_status = 0;
		}
		else
		{
			--param.j3_count;
		}

		target.model->motionPool().at(param.motion_id).setMp(p_next);
		//controller->motionAtAbs(imp_->motion_id).setTargetPos(p_next);
		param.p_now = p_next;
		param.v_now = v_next;
		param.a_now = a_next;

		// 运动学正解//
		if (target.model->solverPool().at(1).kinPos())return -1;

		// 打印 //
		auto &cout = controller->mout();
		if (target.count % 10 == 0)
		{
			cout << param.j3_count << "  ";
			cout << param.target_pos << "  ";
			cout << param.p_now << "  ";
			cout << param.v_now << "  ";
			cout << param.a_now << "  ";
			cout << "------------------------------------------" << std::endl;
		}

		// log //
		auto &lout = controller->lout();
		{
			lout << param.target_pos << " ";
			lout << param.p_now << " ";
			lout << param.v_now << " ";
			lout << param.a_now << " ";
			lout << controller->motionAtAbs(param.motion_id).actualPos() << " ";
			lout << controller->motionAtAbs(param.motion_id).actualVel() << " ";
			lout << std::endl;
		}

		return finished;
	}
	auto JogJ3::collectNrt(PlanTarget &target)->void 
	{
		JogJParam::j3_count = 0;
		if (target.ret_code < 0)
		{
			JogJParam::j3_count.store(0);
		}
	}
	JogJ3::~JogJ3() = default;
	JogJ3::JogJ3(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"j3\">"
				JOGJ_PARAM_STRING
			"</Command>");
	}


	// 示教运动--关节4点动 //
	std::atomic_int32_t JogJParam::j4_count = 0;
	auto JogJ4::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		JogJParam param;
		std::vector<std::pair<std::string, std::any>> ret;
		target.ret = ret;
    
		param.motion_id = 3;
		set_jogj_input_param(this, params, target, param, param.j4_count);

		target.param = param;
	}
	auto JogJ4::executeRT(PlanTarget &target)->int
	{
		//获取驱动//
		auto controller = target.controller;
		auto &param = std::any_cast<JogJParam&>(target.param);

		// get current pos //
		if (target.count == 1)
		{
			/*
			imp_->target_pos = controller->motionAtAbs(imp_->motion_id).actualPos();
			imp_->p_now = controller->motionAtAbs(imp_->motion_id).actualPos();
			imp_->v_now = controller->motionAtAbs(imp_->motion_id).actualVel();
			imp_->a_now = 0.0;
			*/
			param.target_pos = target.model->motionPool().at(param.motion_id).mp();
			param.p_now = target.model->motionPool()[param.motion_id].mp();
			param.v_now = target.model->motionPool()[param.motion_id].mv();
			param.a_now = 0.0;

		}

		// init status and calculate target pos and max vel //
		param.max_vel = param.vel*1.0*param.vel_percent / 100.0;

		param.target_pos += aris::dynamic::s_sgn(param.increase_status)*param.max_vel * 1e-3;

		// 梯形轨迹规划 //
		static double p_next, v_next, a_next;

		aris::Size t;
		auto finished = aris::plan::moveAbsolute2(param.p_now, param.v_now, param.a_now
			, param.target_pos, 0.0, 0.0
			, param.max_vel, param.acc, param.dec
			, 1e-3, 1e-10, p_next, v_next, a_next, t);

		if (param.j4_count == 0)
		{
			param.increase_status = 0;
		}
		else
		{
			--param.j4_count;
		}

		target.model->motionPool().at(param.motion_id).setMp(p_next);
		//controller->motionAtAbs(imp_->motion_id).setTargetPos(p_next);
		param.p_now = p_next;
		param.v_now = v_next;
		param.a_now = a_next;

		// 运动学正解//
		if (target.model->solverPool().at(1).kinPos())return -1;

		// 打印 //
		auto &cout = controller->mout();
		if (target.count % 10 == 0)
		{
			cout << param.j4_count << "  ";
			cout << param.target_pos << "  ";
			cout << param.p_now << "  ";
			cout << param.v_now << "  ";
			cout << param.a_now << "  ";
			cout << "------------------------------------------" << std::endl;
		}

		// log //
		auto &lout = controller->lout();
		{
			lout << param.target_pos << " ";
			lout << param.p_now << " ";
			lout << param.v_now << " ";
			lout << param.a_now << " ";
			lout << controller->motionAtAbs(param.motion_id).actualPos() << " ";
			lout << controller->motionAtAbs(param.motion_id).actualVel() << " ";
			lout << std::endl;
		}

		return finished;
	}
	auto JogJ4::collectNrt(PlanTarget &target)->void 
	{
		JogJParam::j4_count = 0;
		if (target.ret_code < 0)
		{
			JogJParam::j4_count.store(0);
		}
	}
	JogJ4::~JogJ4() = default;
	JogJ4::JogJ4(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"j4\">"
				JOGJ_PARAM_STRING
			"</Command>");
	}


	// 示教运动--关节5点动 //
	std::atomic_int32_t JogJParam::j5_count = 0;
	auto JogJ5::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		JogJParam param;
		std::vector<std::pair<std::string, std::any>> ret;
		target.ret = ret;

		param.motion_id = 4;
		set_jogj_input_param(this, params, target, param, param.j5_count);

		target.param = param;
	}
	auto JogJ5::executeRT(PlanTarget &target)->int
	{
		//获取驱动//
		auto controller = target.controller;
		auto &param = std::any_cast<JogJParam&>(target.param);

		// get current pos //
		if (target.count == 1)
		{
			/*
			imp_->target_pos = controller->motionAtAbs(imp_->motion_id).actualPos();
			imp_->p_now = controller->motionAtAbs(imp_->motion_id).actualPos();
			imp_->v_now = controller->motionAtAbs(imp_->motion_id).actualVel();
			imp_->a_now = 0.0;
			*/
			param.target_pos = target.model->motionPool().at(param.motion_id).mp();
			param.p_now = target.model->motionPool()[param.motion_id].mp();
			param.v_now = target.model->motionPool()[param.motion_id].mv();
			param.a_now = 0.0;

		}

		// init status and calculate target pos and max vel //
		param.max_vel = param.vel*1.0*param.vel_percent / 100.0;

		param.target_pos += aris::dynamic::s_sgn(param.increase_status)*param.max_vel * 1e-3;

		// 梯形轨迹规划 //
		static double p_next, v_next, a_next;

		aris::Size t;
		auto finished = aris::plan::moveAbsolute2(param.p_now, param.v_now, param.a_now
			, param.target_pos, 0.0, 0.0
			, param.max_vel, param.acc, param.dec
			, 1e-3, 1e-10, p_next, v_next, a_next, t);

		if (param.j5_count == 0)
		{
			param.increase_status = 0;
		}
		else
		{
			--param.j5_count;
		}

		target.model->motionPool().at(param.motion_id).setMp(p_next);
		//controller->motionAtAbs(imp_->motion_id).setTargetPos(p_next);
		param.p_now = p_next;
		param.v_now = v_next;
		param.a_now = a_next;

		// 运动学正解//
		if (target.model->solverPool().at(1).kinPos())return -1;

		// 打印 //
		auto &cout = controller->mout();
		if (target.count % 10 == 0)
		{
			cout << param.j5_count << "  ";
			cout << param.target_pos << "  ";
			cout << param.p_now << "  ";
			cout << param.v_now << "  ";
			cout << param.a_now << "  ";
			cout << "------------------------------------------" << std::endl;
		}

		// log //
		auto &lout = controller->lout();
		{
			lout << param.target_pos << " ";
			lout << param.p_now << " ";
			lout << param.v_now << " ";
			lout << param.a_now << " ";
			lout << controller->motionAtAbs(param.motion_id).actualPos() << " ";
			lout << controller->motionAtAbs(param.motion_id).actualVel() << " ";
			lout << std::endl;
		}

		return finished;
	}
	auto JogJ5::collectNrt(PlanTarget &target)->void 
	{
		JogJParam::j5_count = 0;
		if (target.ret_code < 0)
		{
			JogJParam::j5_count.store(0);
		}
	}
	JogJ5::~JogJ5() = default;
	JogJ5::JogJ5(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"j5\">"
				JOGJ_PARAM_STRING
			"</Command>");
	}


	// 示教运动--关节6点动 //
	std::atomic_int32_t JogJParam::j6_count = 0;
	auto JogJ6::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		JogJParam param;
		std::vector<std::pair<std::string, std::any>> ret;
		target.ret = ret;

		param.motion_id = 5;
		set_jogj_input_param(this, params, target, param, param.j6_count);

		target.param = param;
	}
	auto JogJ6::executeRT(PlanTarget &target)->int
	{
		//获取驱动//
		auto controller = target.controller;
		auto &param = std::any_cast<JogJParam&>(target.param);

		// get current pos //
		if (target.count == 1)
		{
			/*
			imp_->target_pos = controller->motionAtAbs(imp_->motion_id).actualPos();
			imp_->p_now = controller->motionAtAbs(imp_->motion_id).actualPos();
			imp_->v_now = controller->motionAtAbs(imp_->motion_id).actualVel();
			imp_->a_now = 0.0;
			*/
			param.target_pos = target.model->motionPool().at(param.motion_id).mp();
			param.p_now = target.model->motionPool()[param.motion_id].mp();
			param.v_now = target.model->motionPool()[param.motion_id].mv();
			param.a_now = 0.0;

		}

		// init status and calculate target pos and max vel //
		param.max_vel = param.vel*1.0*param.vel_percent / 100.0;
		param.target_pos += aris::dynamic::s_sgn(param.increase_status)*param.max_vel * 1e-3;

		// 梯形轨迹规划 //
		static double p_next, v_next, a_next;

		aris::Size t;
		auto finished = aris::plan::moveAbsolute2(param.p_now, param.v_now, param.a_now
			, param.target_pos, 0.0, 0.0
			, param.max_vel, param.acc, param.dec
			, 1e-3, 1e-10, p_next, v_next, a_next, t);

		//当计数器j6_count等于0时，increase_status=0，即目标位置不再变更//
		if (param.j6_count == 0)
		{
			param.increase_status = 0;
		}
		else
		{
			--param.j6_count;
		}

		target.model->motionPool().at(param.motion_id).setMp(p_next);
		//controller->motionAtAbs(imp_->motion_id).setTargetPos(p_next);
		param.p_now = p_next;
		param.v_now = v_next;
		param.a_now = a_next;

		// 运动学正解//
		if (target.model->solverPool().at(1).kinPos())return -1;

		// 打印 //
		auto &cout = controller->mout();
		if (target.count % 10 == 0)
		{
			cout << param.j6_count << "  ";
			cout << param.target_pos << "  ";
			cout << param.p_now << "  ";
			cout << param.v_now << "  ";
			cout << param.a_now << "  ";
			cout << "------------------------------------------" << std::endl;
		}

		// log //
		auto &lout = controller->lout();
		{
			lout << param.target_pos << " ";
			lout << param.p_now << " ";
			lout << param.v_now << " ";
			lout << param.a_now << " ";
			lout << controller->motionAtAbs(param.motion_id).actualPos() << " ";
			lout << controller->motionAtAbs(param.motion_id).actualVel() << " ";
			lout << std::endl;
		}

		return finished;
	}
	auto JogJ6::collectNrt(PlanTarget &target)->void 
	{
		JogJParam::j6_count = 0;
		if (target.ret_code < 0)
		{
			JogJParam::j6_count.store(0);
		}
	}
	JogJ6::~JogJ6() = default;
	JogJ6::JogJ6(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"j6\">"
				JOGJ_PARAM_STRING
			"</Command>");
	}


	// 示教运动--外部轴点动 //
	std::atomic_int32_t JogJParam::j7_count = 0;
	auto JogJ7::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		auto&cs = aris::server::ControlServer::instance();
		auto c = target.controller;
		JogJParam param;

		std::vector<std::pair<std::string, std::any>> ret;
		target.ret = ret;

		param.motion_id = 6;
		param.p_now = 0.0;
		param.v_now = 0.0;
		param.a_now = 0.0;
		param.target_pos = 0.0;
		param.max_vel = 0.0;
		param.increase_status = 0;
		param.vel_percent = 0;

		for (auto &p : params)
		{
			if (p.first == "increase_count")
			{
				param.increase_count = std::stoi(params.at("increase_count"));
				if (param.increase_count < 0 || param.increase_count>1e5)THROW_FILE_LINE("");

				param.vel = std::min(std::max(std::stod(params.at("vel")), 0.0), 1.0)*c->motionPool().at(param.motion_id).maxVel();
				param.acc = std::min(std::max(std::stod(params.at("acc")), 0.0), 1.0)*c->motionPool().at(param.motion_id).maxAcc();
				param.dec = std::min(std::max(std::stod(params.at("dec")), 0.0), 1.0)*c->motionPool().at(param.motion_id).maxAcc();

				auto velocity = std::stoi(params.at("vel_percent"));
				velocity = std::max(std::min(100, velocity), 0);
				param.vel_percent = velocity;
				param.increase_status = std::max(std::min(1, std::stoi(params.at("direction"))), -1);

				std::shared_ptr<aris::plan::PlanTarget> planptr = cs.currentExecuteTarget();
				//当前有指令在执行//
				if (planptr && planptr->plan.get() != this)throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + "Other command is running");

				if (param.j7_count.exchange(param.increase_count))
				{
					target.option |= NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION;
				}
				else
				{
					std::fill(target.mot_options.begin(), target.mot_options.end(), NOT_CHECK_POS_FOLLOWING_ERROR | NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER | NOT_CHECK_ENABLE);
					target.mot_options[param.motion_id] = NOT_CHECK_POS_FOLLOWING_ERROR | NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
				}
			}
			else if (p.first == "stop")
			{
				param.j7_count.exchange(0);
				target.option |= aris::plan::Plan::Option::NOT_RUN_EXECUTE_FUNCTION | aris::plan::Plan::Option::NOT_RUN_COLLECT_FUNCTION;
			}
		}

		target.param = param;
	}
	auto JogJ7::executeRT(PlanTarget &target)->int
	{
		//获取驱动//
		auto controller = target.controller;
		auto &param = std::any_cast<JogJParam&>(target.param);

		// get current pos //
		if (target.count == 1)
		{
			param.target_pos = controller->motionAtAbs(param.motion_id).actualPos();
			param.p_now = controller->motionAtAbs(param.motion_id).actualPos();
			param.v_now = controller->motionAtAbs(param.motion_id).actualVel();
			param.a_now = 0.0;
		}

		// init status and calculate target pos and max vel //
		param.max_vel = param.vel*1.0*g_vel_percent.load() / 100.0;
		param.target_pos += aris::dynamic::s_sgn(param.increase_status)*param.max_vel * 1e-3;

		// 梯形轨迹规划 //
		static double p_next, v_next, a_next;

		aris::Size t;
		auto finished = aris::plan::moveAbsolute2(param.p_now, param.v_now, param.a_now
			, param.target_pos, 0.0, 0.0
			, param.max_vel, param.acc, param.dec
			, 1e-3, 1e-10, p_next, v_next, a_next, t);

		//当计数器j7_count等于0时，increase_status=0，即目标位置不再变更//
		if (param.j7_count == 0)
		{
			param.increase_status = 0;
		}
		else
		{
			--param.j7_count;
		}

		controller->motionAtAbs(param.motion_id).setTargetPos(p_next);
		param.p_now = p_next;
		param.v_now = v_next;
		param.a_now = a_next;

		// 打印 //
		auto &cout = controller->mout();
		if (target.count % 10 == 0)
		{
			cout << param.j7_count << "  ";
			cout << param.target_pos << "  ";
			cout << param.p_now << "  ";
			cout << param.v_now << "  ";
			cout << param.a_now << "  ";
			cout << "------------------------------------------" << std::endl;
		}

		// log //
		auto &lout = controller->lout();
		{
			lout << param.target_pos << " ";
			lout << param.p_now << " ";
			lout << param.v_now << " ";
			lout << param.a_now << " ";
			lout << controller->motionAtAbs(param.motion_id).actualPos() << " ";
			lout << controller->motionAtAbs(param.motion_id).actualVel() << " ";
			lout << std::endl;
		}

		return finished;
	}
	auto JogJ7::collectNrt(PlanTarget &target)->void
	{
		JogJParam::j7_count = 0;
		if (target.ret_code < 0)
		{
			JogJParam::j7_count.store(0);
		}
	}
	JogJ7::~JogJ7() = default;
	JogJ7::JogJ7(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"j7\">"
			"	<GroupParam>"
			"		<UniqueParam>"
			"			<GroupParam>"
			"				<Param name=\"increase_count\" default=\"500\"/>"
			"				<Param name=\"vel\" default=\"1\" abbreviation=\"v\"/>"
			"				<Param name=\"acc\" default=\"5\" abbreviation=\"a\"/>"
			"				<Param name=\"dec\" default=\"5\" abbreviation=\"d\"/>"
			"				<Param name=\"vel_percent\" default=\"10\"/>"
			"				<Param name=\"direction\" default=\"1\"/>"
			"			</GroupParam>"
			"			<Param name=\"stop\"/>"
			"		</UniqueParam>"
			"	</GroupParam>"
			"</Command>");
	}



#define JOGC_PARAM_STRING \
		"	<UniqueParam>"\
		"		<GroupParam>"\
		"			<Param name=\"increase_count\" default=\"500\"/>"\
		"			<Param name=\"vel\" default=\"{0.05,0.05,0.05,0.25,0.25,0.25}\"/>"\
		"			<Param name=\"acc\" default=\"{0.2,0.2,0.2,1,1,1}\"/>"\
		"			<Param name=\"dec\" default=\"{0.2,0.2,0.2,1,1,1}\"/>"\
		"			<Param name=\"cor\" default=\"0\"/>"\
		"			<Param name=\"tool\" default=\"tool0\"/>"\
		"			<Param name=\"wobj\" default=\"wobj0\"/>"\
		"			<Param name=\"vel_percent\" default=\"20\"/>"\
		"			<Param name=\"direction\" default=\"1\"/>"\
		"		</GroupParam>"\
		"		<Param name=\"stop\"/>"\
		"	</UniqueParam>"
	// 示教运动--jogx //
	struct JCParam 
	{
		std::vector<double> pm_target;
		double vel[6], acc[6], dec[6];
		int increase_count;
		int cor_system;
		int vel_percent;
		int moving_type;
		int increase_status[6]{0,0,0,0,0,0};
		static std::atomic_int32_t jx_count, jy_count, jz_count, jrx_count, jry_count, jrz_count;
		aris::dynamic::Marker *tool, *wobj;
	};
	template<typename JogType>
	auto set_jogc_input_param(JogType* this_p, const std::map<std::string, std::string> &cmd_params, PlanTarget &target, JCParam &param, std::atomic_int32_t& j_count)->void
	{
		auto&cs = aris::server::ControlServer::instance();
		auto c = target.controller;

		for (auto &p : cmd_params)
		{
			if (p.first == "increase_count")
			{
				auto tool_is_string = cmd_params.at("tool");
				auto wobj_is_string = cmd_params.at("wobj");

				param.tool = &*target.model->generalMotionPool()[0].makI().fatherPart().markerPool().findByName(cmd_params.at("tool"));
				param.wobj = &*target.model->generalMotionPool()[0].makJ().fatherPart().markerPool().findByName(cmd_params.at("wobj"));
				
				param.increase_count = std::stoi(cmd_params.at("increase_count"));
				if (param.increase_count < 0 || param.increase_count>1e5)THROW_FILE_LINE("");

				param.cor_system = std::stoi(cmd_params.at("cor"));
				auto velocity = std::stoi(cmd_params.at("vel_percent"));
				velocity = std::max(std::min(100, velocity), 0);
				param.vel_percent = velocity;

				auto mat = target.model->calculator().calculateExpression(cmd_params.at("vel"));
				if (mat.size() != 6)THROW_FILE_LINE("");
				std::copy(mat.begin(), mat.end(), param.vel);
				std::fill(param.vel, param.vel + 3, g_vel.getspeed().v_tcp);
				std::fill(param.vel + 3, param.vel + 6, g_vel.getspeed().w_tcp);

				mat = target.model->calculator().calculateExpression(cmd_params.at("acc"));
				if (mat.size() != 6)THROW_FILE_LINE("");
				std::copy(mat.begin(), mat.end(), param.acc);

				mat = target.model->calculator().calculateExpression(cmd_params.at("dec"));
				if (mat.size() != 6)THROW_FILE_LINE("");
				std::copy(mat.begin(), mat.end(), param.dec);

				param.increase_status[param.moving_type] = std::max(std::min(1, std::stoi(cmd_params.at("direction"))), -1);

				std::shared_ptr<aris::plan::PlanTarget> planptr = cs.currentExecuteTarget();

				//当前有指令在执行//
				if (planptr && planptr->plan.get() != this_p)throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + "Other command is running");

				if (j_count.exchange(param.increase_count))
				{
					target.option |= aris::plan::Plan::Option::NOT_RUN_EXECUTE_FUNCTION | aris::plan::Plan::Option::NOT_RUN_COLLECT_FUNCTION;
				}
				else
				{
					std::fill(target.mot_options.begin(), target.mot_options.end(), aris::plan::Plan::MotionOption::NOT_CHECK_POS_FOLLOWING_ERROR | aris::plan::Plan::MotionOption::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER | aris::plan::Plan::MotionOption::USE_TARGET_POS);
				}
			}
			else if (p.first == "stop")
			{
				j_count.exchange(0);
				target.option |= aris::plan::Plan::Option::NOT_RUN_EXECUTE_FUNCTION | aris::plan::Plan::Option::NOT_RUN_COLLECT_FUNCTION;
			}
		}
	}
	std::atomic_int32_t JCParam::jx_count = 0;
	auto JX::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		JCParam param;

		param.pm_target.resize(16, 0.0);
		param.moving_type = 0;//0,1,2,3,4,5分别表示沿x,y,z,Rx,Ry,Rz动作//

		std::vector<std::pair<std::string, std::any>> ret;
		target.ret = ret;

		set_jogc_input_param(this, params, target, param, param.jx_count);

		target.param = param;

	}
	auto JX::executeRT(PlanTarget &target)->int
	{
		//获取驱动//
		auto controller = target.controller;
		auto &param = std::any_cast<JCParam&>(target.param);
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
		double max_vel[6]{ 0,0,0,0,0,0 };

		// calculate target pos and max vel //
		for (int i = 0; i < 6; i++)
		{
			max_vel[i] = param.vel[i] * 1.0 * param.vel_percent / 100.0;
			target_p[i] += aris::dynamic::s_sgn(param.increase_status[i]) * max_vel[i] * 1e-3;
		}
		// 梯形轨迹规划 calculate real value //
		double p_next[6]{ 0,0,0,0,0,0 }, v_next[6]{ 0,0,0,0,0,0 }, a_next[6]{ 0,0,0,0,0,0 };
		int finished[6]{0,0,0,0,0,0};
		for (int i = 0; i < 6; i++)
		{
			aris::Size t;
			finished[i] = aris::plan::moveAbsolute2(p_now[i], v_now[i], a_now[i]
				, target_p[i], 0.0, 0.0
				, max_vel[i], param.acc[i], param.dec[i]
				, 1e-3, 1e-10, p_next[i], v_next[i], a_next[i], t);
		}

		//当计数器jx_count等于0时，increase_status=0，即目标位置不再变更//
		if (param.jx_count == 0)
		{
			param.increase_status[param.moving_type] = 0;
		}
		else
		{
			--param.jx_count;
		}

		//将欧拉角转换成四元数，求绕任意旋转轴转动的旋转矩阵//
		double w[3], pm[16];
		aris::dynamic::s_vc(3, v_next + 3, w);
		auto normv = aris::dynamic::s_norm(3, w);
		if (std::abs(normv) > 1e-10)aris::dynamic::s_nv(3, 1 / normv, w); //数乘
		auto theta = normv * 1e-3;
		double pq[7]{ p_next[0] - p_now[0], p_next[1] - p_now[1], p_next[2] - p_now[2], w[0] * sin(theta / 2.0), w[1] * sin(theta / 2.0), w[2] * sin(theta / 2.0), cos(theta / 2.0) };
		s_pq2pm(pq, pm);

		// 获取当前位姿矩阵 //
		double pm_now[16];
		//target.model->generalMotionPool()[0].getMpm(pm_now);
		param.tool->getPm(*param.wobj, pm_now);

		// 保存下个周期的copy //
		s_vc(6, p_next, p_now);
		s_vc(6, v_next, v_now);
		s_vc(6, a_next, a_now);

		//绝对坐标系
		if (param.cor_system == 0)
		{
			s_pm_dot_pm(pm, pm_now, param.pm_target.data());
		}
		//工具坐标系
		else if (param.cor_system == 1)
		{
			s_pm_dot_pm(pm_now, pm, param.pm_target.data());
		}

		//target.model->generalMotionPool().at(0).setMpm(param.pm_target.data());
		param.tool->setPm(*param.wobj, param.pm_target.data());
		target.model->generalMotionPool().at(0).updMpm();

		// 运动学反解 //
		if (target.model->solverPool().at(0).kinPos())return -1;

		// 打印 //
		auto &cout = controller->mout();
		if (target.count % 100 == 0)
		{
			cout << "jx_count:" << std::endl;
			cout << param.jx_count << "  ";
			cout << std::endl;
			cout << "increase_status:" << std::endl;
			cout << param.increase_status[param.moving_type] << "  ";
			cout << std::endl;
			cout << "p_next:" << std::endl;
			cout << p_next[param.moving_type] << "  ";
			cout << std::endl;
			cout << "v_next:" << std::endl;
			cout << v_next[param.moving_type] << "  ";
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

		return finished[param.moving_type];
	}
	auto JX::collectNrt(PlanTarget &target)->void 
	{
		JCParam::jx_count = 0;
		if (target.ret_code < 0)
		{
			JCParam::jx_count.store(0);
		}
	}
	JX::~JX() = default;
	JX::JX(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"jx\">"
				JOGC_PARAM_STRING
			"</Command>");
	}
	

	// 示教运动--jogy //
	std::atomic_int32_t JCParam::jy_count = 0;
	auto JY::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		JCParam param;

		param.pm_target.resize(16, 0.0);
		param.moving_type = 1;//0,1,2,3,4,5分别表示沿x,y,z,Rx,Ry,Rz动作//

		std::vector<std::pair<std::string, std::any>> ret;
		target.ret = ret;

		set_jogc_input_param(this, params, target, param, param.jy_count);

		target.param = param;

	}
	auto JY::executeRT(PlanTarget &target)->int
	{
		//获取驱动//
		auto controller = target.controller;
		auto &param = std::any_cast<JCParam&>(target.param);
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
		double max_vel[6]{ 0,0,0,0,0,0 };

		// calculate target pos and max vel //
		for (int i = 0; i < 6; i++)
		{
			max_vel[i] = param.vel[i] * 1.0 * param.vel_percent / 100.0;
			target_p[i] += aris::dynamic::s_sgn(param.increase_status[i]) * max_vel[i] * 1e-3;
		}
		// 梯形轨迹规划 calculate real value //
		double p_next[6]{ 0,0,0,0,0,0 }, v_next[6]{ 0,0,0,0,0,0 }, a_next[6]{ 0,0,0,0,0,0 };
		int finished[6]{ 0,0,0,0,0,0 };
		for (int i = 0; i < 6; i++)
		{
			aris::Size t;
			finished[i] = aris::plan::moveAbsolute2(p_now[i], v_now[i], a_now[i]
				, target_p[i], 0.0, 0.0
				, max_vel[i], param.acc[i], param.dec[i]
				, 1e-3, 1e-10, p_next[i], v_next[i], a_next[i], t);
		}

		//当计数器jy_count等于0时，increase_status=0，即目标位置不再变更//
		if (param.jy_count == 0)
		{
			param.increase_status[param.moving_type] = 0;
		}
		else
		{
			--param.jy_count;
		}

		//将欧拉角转换成四元数，求绕任意旋转轴转动的旋转矩阵//
		double w[3], pm[16];
		aris::dynamic::s_vc(3, v_next + 3, w);
		auto normv = aris::dynamic::s_norm(3, w);
		if (std::abs(normv) > 1e-10)aris::dynamic::s_nv(3, 1 / normv, w); //数乘
		auto theta = normv * 1e-3;
		double pq[7]{ p_next[0] - p_now[0], p_next[1] - p_now[1], p_next[2] - p_now[2], w[0] * sin(theta / 2.0), w[1] * sin(theta / 2.0), w[2] * sin(theta / 2.0), cos(theta / 2.0) };
		s_pq2pm(pq, pm);

		// 获取当前位姿矩阵 //
		double pm_now[16];
		param.tool->getPm(*param.wobj, pm_now);

		// 保存下个周期的copy //
		s_vc(6, p_next, p_now);
		s_vc(6, v_next, v_now);
		s_vc(6, a_next, a_now);

		//绝对坐标系
		if (param.cor_system == 0)
		{
			s_pm_dot_pm(pm, pm_now, param.pm_target.data());
		}
		//工具坐标系
		else if (param.cor_system == 1)
		{
			s_pm_dot_pm(pm_now, pm, param.pm_target.data());
		}

		param.tool->setPm(*param.wobj, param.pm_target.data());
		target.model->generalMotionPool().at(0).updMpm();

		// 运动学反解 //
		if (target.model->solverPool().at(0).kinPos())return -1;

		// 打印 //
		auto &cout = controller->mout();
		if (target.count % 10 == 0)
		{
			cout << "jy_count:" << std::endl;
			cout << param.jy_count << "  ";
			cout << std::endl;
			cout << "increase_status:" << std::endl;
			cout << param.increase_status[param.moving_type] << "  ";
			cout << std::endl;
			cout << "p_next:" << std::endl;
			cout << p_next[param.moving_type] << "  ";
			cout << std::endl;
			cout << "v_next:" << std::endl;
			cout << v_next[param.moving_type] << "  ";
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

		return finished[param.moving_type];
	}
	auto JY::collectNrt(PlanTarget &target)->void 
	{
		JCParam::jy_count = 0;
		if (target.ret_code < 0)
		{
			JCParam::jy_count.store(0);
		}
	}
	JY::~JY() = default;
	JY::JY(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"jy\">"
				JOGC_PARAM_STRING
			"</Command>");
	}


	// 示教运动--jogz //
	std::atomic_int32_t JCParam::jz_count = 0;
	auto JZ::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		JCParam param;

		param.pm_target.resize(16, 0.0);
		param.moving_type = 2;//0,1,2,3,4,5分别表示沿x,y,z,Rx,Ry,Rz动作//

		std::vector<std::pair<std::string, std::any>> ret;
		target.ret = ret;

		set_jogc_input_param(this, params, target, param, param.jz_count);

		target.param = param;

	}
	auto JZ::executeRT(PlanTarget &target)->int
	{
		//获取驱动//
		auto controller = target.controller;
		auto &param = std::any_cast<JCParam&>(target.param);
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
		double max_vel[6]{ 0,0,0,0,0,0 };

		// calculate target pos and max vel //
		for (int i = 0; i < 6; i++)
		{
			max_vel[i] = param.vel[i] * 1.0 * param.vel_percent / 100.0;
			target_p[i] += aris::dynamic::s_sgn(param.increase_status[i]) * max_vel[i] * 1e-3;
		}
		// 梯形轨迹规划 calculate real value //
		double p_next[6]{ 0,0,0,0,0,0 }, v_next[6]{ 0,0,0,0,0,0 }, a_next[6]{ 0,0,0,0,0,0 };
		int finished[6]{ 0,0,0,0,0,0 };
		for (int i = 0; i < 6; i++)
		{
			aris::Size t;
			finished[i] = aris::plan::moveAbsolute2(p_now[i], v_now[i], a_now[i]
				, target_p[i], 0.0, 0.0
				, max_vel[i], param.acc[i], param.dec[i]
				, 1e-3, 1e-10, p_next[i], v_next[i], a_next[i], t);
		}

		//当计数器jz_count等于0时，increase_status=0，即目标位置不再变更//
		if (param.jz_count == 0)
		{
			param.increase_status[param.moving_type] = 0;
		}
		else
		{
			--param.jz_count;
		}

		//将欧拉角转换成四元数，求绕任意旋转轴转动的旋转矩阵//
		double w[3], pm[16];
		aris::dynamic::s_vc(3, v_next + 3, w);
		auto normv = aris::dynamic::s_norm(3, w);
		if (std::abs(normv) > 1e-10)aris::dynamic::s_nv(3, 1 / normv, w); //数乘
		auto theta = normv * 1e-3;
		double pq[7]{ p_next[0] - p_now[0], p_next[1] - p_now[1], p_next[2] - p_now[2], w[0] * sin(theta / 2.0), w[1] * sin(theta / 2.0), w[2] * sin(theta / 2.0), cos(theta / 2.0) };
		s_pq2pm(pq, pm);

		// 获取当前位姿矩阵 //
		double pm_now[16];
		param.tool->getPm(*param.wobj, pm_now);

		// 保存下个周期的copy //
		s_vc(6, p_next, p_now);
		s_vc(6, v_next, v_now);
		s_vc(6, a_next, a_now);

		//绝对坐标系
		if (param.cor_system == 0)
		{
			s_pm_dot_pm(pm, pm_now, param.pm_target.data());
		}
		//工具坐标系
		else if (param.cor_system == 1)
		{
			s_pm_dot_pm(pm_now, pm, param.pm_target.data());
		}

		param.tool->setPm(*param.wobj, param.pm_target.data());
		target.model->generalMotionPool().at(0).updMpm();

		// 运动学反解 //
		if (target.model->solverPool().at(0).kinPos())return -1;

		// 打印 //
		auto &cout = controller->mout();
		if (target.count % 10 == 0)
		{
			cout << "jz_count:" << std::endl;
			cout << param.jz_count << "  ";
			cout << std::endl;
			cout << "increase_status:" << std::endl;
			cout << param.increase_status[param.moving_type] << "  ";
			cout << std::endl;
			cout << "p_next:" << std::endl;
			cout << p_next[param.moving_type] << "  ";
			cout << std::endl;
			cout << "v_next:" << std::endl;
			cout << v_next[param.moving_type] << "  ";
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

		return finished[param.moving_type];
	}
	auto JZ::collectNrt(PlanTarget &target)->void 
	{
		JCParam::jz_count = 0;
		if (target.ret_code < 0)
		{
			JCParam::jz_count.store(0);
		}
	}
	JZ::~JZ() = default;
	JZ::JZ(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"jz\">"
				JOGC_PARAM_STRING
			"</Command>");
	}


	// 示教运动--jogrx //
	std::atomic_int32_t JCParam::jrx_count = 0;
	auto JRX::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		JCParam param;

		param.pm_target.resize(16, 0.0);
		param.moving_type = 3;//0,1,2,3,4,5分别表示沿x,y,z,Rx,Ry,Rz动作//

		std::vector<std::pair<std::string, std::any>> ret;
		target.ret = ret;

		set_jogc_input_param(this, params, target, param, param.jrx_count);

		target.param = param;

	}
	auto JRX::executeRT(PlanTarget &target)->int
	{
		//获取驱动//
		auto controller = target.controller;
		auto &param = std::any_cast<JCParam&>(target.param);
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
		double max_vel[6]{ 0,0,0,0,0,0 };

		// calculate target pos and max vel //
		for (int i = 0; i < 6; i++)
		{
			max_vel[i] = param.vel[i] * 1.0 * param.vel_percent / 100.0;
			target_p[i] += aris::dynamic::s_sgn(param.increase_status[i]) * max_vel[i] * 1e-3;
		}
		// 梯形轨迹规划 calculate real value //
		double p_next[6]{ 0,0,0,0,0,0 }, v_next[6]{ 0,0,0,0,0,0 }, a_next[6]{ 0,0,0,0,0,0 };
		int finished[6]{ 0,0,0,0,0,0 };
		for (int i = 0; i < 6; i++)
		{
			aris::Size t;
			finished[i] = aris::plan::moveAbsolute2(p_now[i], v_now[i], a_now[i]
				, target_p[i], 0.0, 0.0
				, max_vel[i], param.acc[i], param.dec[i]
				, 1e-3, 1e-10, p_next[i], v_next[i], a_next[i], t);
		}

		//当计数器jrx_count等于0时，increase_status=0，即目标位置不再变更//
		if (param.jrx_count == 0)
		{
			param.increase_status[param.moving_type] = 0;
		}
		else
		{
			--param.jrx_count;
		}

		//将欧拉角转换成四元数，求绕任意旋转轴转动的旋转矩阵//
		double w[3], pm[16];
		aris::dynamic::s_vc(3, v_next + 3, w);
		auto normv = aris::dynamic::s_norm(3, w);
		if (std::abs(normv) > 1e-10)aris::dynamic::s_nv(3, 1 / normv, w); //数乘
		auto theta = normv * 1e-3;
		double pq[7]{ p_next[0] - p_now[0], p_next[1] - p_now[1], p_next[2] - p_now[2], w[0] * sin(theta / 2.0), w[1] * sin(theta / 2.0), w[2] * sin(theta / 2.0), cos(theta / 2.0) };
		s_pq2pm(pq, pm);

		// 获取当前位姿矩阵 //
		double pm_now[16];
		param.tool->getPm(*param.wobj, pm_now);

		// 保存下个周期的copy //
		s_vc(6, p_next, p_now);
		s_vc(6, v_next, v_now);
		s_vc(6, a_next, a_now);

		//绝对坐标系
		if (param.cor_system == 0)
		{
			s_pm_dot_pm(pm, pm_now, param.pm_target.data());
		}
		//工具坐标系
		else if (param.cor_system == 1)
		{
			s_pm_dot_pm(pm_now, pm, param.pm_target.data());
		}

		param.tool->setPm(*param.wobj, param.pm_target.data());
		target.model->generalMotionPool().at(0).updMpm();

		// 运动学反解 //
		if (target.model->solverPool().at(0).kinPos())return -1;

		// 打印 //
		auto &cout = controller->mout();
		if (target.count % 10 == 0)
		{
			cout << "jrx_count:" << std::endl;
			cout << param.jrx_count << "  ";
			cout << std::endl;
			cout << "increase_status:" << std::endl;
			cout << param.increase_status[param.moving_type] << "  ";
			cout << std::endl;
			cout << "p_next:" << std::endl;
			cout << p_next[param.moving_type] << "  ";
			cout << std::endl;
			cout << "v_next:" << std::endl;
			cout << v_next[param.moving_type] << "  ";
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

		return finished[param.moving_type];
	}
	auto JRX::collectNrt(PlanTarget &target)->void 
	{
		JCParam::jrx_count = 0;
		if (target.ret_code < 0)
		{
			JCParam::jrx_count.store(0);
		}
	}
	JRX::~JRX() = default;
	JRX::JRX(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"jrx\">"
				JOGC_PARAM_STRING
			"</Command>");
	}


	// 示教运动--jogry //
	std::atomic_int32_t JCParam::jry_count = 0;
	auto JRY::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		JCParam param;

		param.pm_target.resize(16, 0.0);
		param.moving_type = 4;//0,1,2,3,4,5分别表示沿x,y,z,Rx,Ry,Rz动作//

		std::vector<std::pair<std::string, std::any>> ret;
		target.ret = ret;

		set_jogc_input_param(this, params, target, param, param.jry_count);

		target.param = param;

	}
	auto JRY::executeRT(PlanTarget &target)->int
	{
		//获取驱动//
		auto controller = target.controller;
		auto &param = std::any_cast<JCParam&>(target.param);
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
		double max_vel[6]{ 0,0,0,0,0,0 };

		// calculate target pos and max vel //
		for (int i = 0; i < 6; i++)
		{
			max_vel[i] = param.vel[i] * 1.0 * param.vel_percent / 100.0;
			target_p[i] += aris::dynamic::s_sgn(param.increase_status[i]) * max_vel[i] * 1e-3;
		}
		// 梯形轨迹规划 calculate real value //
		double p_next[6]{ 0,0,0,0,0,0 }, v_next[6]{ 0,0,0,0,0,0 }, a_next[6]{ 0,0,0,0,0,0 };
		int finished[6]{ 0,0,0,0,0,0 };
		for (int i = 0; i < 6; i++)
		{
			aris::Size t;
			finished[i] = aris::plan::moveAbsolute2(p_now[i], v_now[i], a_now[i]
				, target_p[i], 0.0, 0.0
				, max_vel[i], param.acc[i], param.dec[i]
				, 1e-3, 1e-10, p_next[i], v_next[i], a_next[i], t);
		}

		//当计数器jry_count等于0时，increase_status=0，即目标位置不再变更//
		if (param.jry_count == 0)
		{
			param.increase_status[param.moving_type] = 0;
		}
		else
		{
			--param.jry_count;
		}

		//将欧拉角转换成四元数，求绕任意旋转轴转动的旋转矩阵//
		double w[3], pm[16];
		aris::dynamic::s_vc(3, v_next + 3, w);
		auto normv = aris::dynamic::s_norm(3, w);
		if (std::abs(normv) > 1e-10)aris::dynamic::s_nv(3, 1 / normv, w); //数乘
		auto theta = normv * 1e-3;
		double pq[7]{ p_next[0] - p_now[0], p_next[1] - p_now[1], p_next[2] - p_now[2], w[0] * sin(theta / 2.0), w[1] * sin(theta / 2.0), w[2] * sin(theta / 2.0), cos(theta / 2.0) };
		s_pq2pm(pq, pm);

		// 获取当前位姿矩阵 //
		double pm_now[16];
		param.tool->getPm(*param.wobj, pm_now);

		// 保存下个周期的copy //
		s_vc(6, p_next, p_now);
		s_vc(6, v_next, v_now);
		s_vc(6, a_next, a_now);

		//绝对坐标系
		if (param.cor_system == 0)
		{
			s_pm_dot_pm(pm, pm_now, param.pm_target.data());
		}
		//工具坐标系
		else if (param.cor_system == 1)
		{
			s_pm_dot_pm(pm_now, pm, param.pm_target.data());
		}

		param.tool->setPm(*param.wobj, param.pm_target.data());
		target.model->generalMotionPool().at(0).updMpm();

		// 运动学反解 //
		if (target.model->solverPool().at(0).kinPos())return -1;

		// 打印 //
		auto &cout = controller->mout();
		if (target.count % 10 == 0)
		{
			cout << "jry_count:" << std::endl;
			cout << param.jry_count << "  ";
			cout << std::endl;
			cout << "increase_status:" << std::endl;
			cout << param.increase_status[param.moving_type] << "  ";
			cout << std::endl;
			cout << "p_next:" << std::endl;
			cout << p_next[param.moving_type] << "  ";
			cout << std::endl;
			cout << "v_next:" << std::endl;
			cout << v_next[param.moving_type] << "  ";
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

		return finished[param.moving_type];
	}
	auto JRY::collectNrt(PlanTarget &target)->void 
	{
		JCParam::jry_count = 0;
		if (target.ret_code < 0)
		{
			JCParam::jry_count.store(0);
		}
	}
	JRY::~JRY() = default;
	JRY::JRY(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"jry\">"
				JOGC_PARAM_STRING
			"</Command>");
	}


	// 示教运动--jogrz //
	std::atomic_int32_t JCParam::jrz_count = 0;
	auto JRZ::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		JCParam param;

		param.pm_target.resize(16, 0.0);
		param.moving_type = 5;//0,1,2,3,4,5分别表示沿x,y,z,Rx,Ry,Rz动作//

		std::vector<std::pair<std::string, std::any>> ret;
		target.ret = ret;

		set_jogc_input_param(this, params, target, param, param.jrz_count);

		target.param = param;

	}
	auto JRZ::executeRT(PlanTarget &target)->int
	{
		//获取驱动//
		auto controller = target.controller;
		auto &param = std::any_cast<JCParam&>(target.param);
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
		double max_vel[6]{ 0,0,0,0,0,0 };

		// calculate target pos and max vel //
		for (int i = 0; i < 6; i++)
		{
			max_vel[i] = param.vel[i] * 1.0 * param.vel_percent / 100.0;
			target_p[i] += aris::dynamic::s_sgn(param.increase_status[i]) * max_vel[i] * 1e-3;
		}
		// 梯形轨迹规划 calculate real value //
		double p_next[6]{ 0,0,0,0,0,0 }, v_next[6]{ 0,0,0,0,0,0 }, a_next[6]{ 0,0,0,0,0,0 };
		int finished[6]{ 0,0,0,0,0,0 };
		for (int i = 0; i < 6; i++)
		{
			aris::Size t;
			finished[i] = aris::plan::moveAbsolute2(p_now[i], v_now[i], a_now[i]
				, target_p[i], 0.0, 0.0
				, max_vel[i], param.acc[i], param.dec[i]
				, 1e-3, 1e-10, p_next[i], v_next[i], a_next[i], t);
		}

		//当计数器jrz_count等于0时，increase_status=0，即目标位置不再变更//
		if (param.jrz_count == 0)
		{
			param.increase_status[param.moving_type] = 0;
		}
		else
		{
			--param.jrz_count;
		}

		//将欧拉角转换成四元数，求绕任意旋转轴转动的旋转矩阵//
		double w[3], pm[16];
		aris::dynamic::s_vc(3, v_next + 3, w);
		auto normv = aris::dynamic::s_norm(3, w);
		if (std::abs(normv) > 1e-10)aris::dynamic::s_nv(3, 1 / normv, w); //数乘
		auto theta = normv * 1e-3;
		double pq[7]{ p_next[0] - p_now[0], p_next[1] - p_now[1], p_next[2] - p_now[2], w[0] * sin(theta / 2.0), w[1] * sin(theta / 2.0), w[2] * sin(theta / 2.0), cos(theta / 2.0) };
		s_pq2pm(pq, pm);

		// 获取当前位姿矩阵 //
		double pm_now[16];
		param.tool->getPm(*param.wobj, pm_now);

		// 保存下个周期的copy //
		s_vc(6, p_next, p_now);
		s_vc(6, v_next, v_now);
		s_vc(6, a_next, a_now);

		//绝对坐标系
		if (param.cor_system == 0)
		{
			s_pm_dot_pm(pm, pm_now, param.pm_target.data());
		}
		//工具坐标系
		else if (param.cor_system == 1)
		{
			s_pm_dot_pm(pm_now, pm, param.pm_target.data());
		}

		param.tool->setPm(*param.wobj, param.pm_target.data());
		target.model->generalMotionPool().at(0).updMpm();

		// 运动学反解 //
		if (target.model->solverPool().at(0).kinPos())return -1;

		// 打印 //
		auto &cout = controller->mout();
		if (target.count % 10 == 0)
		{
			cout << "jrz_count:" << std::endl;
			cout << param.jrz_count << "  ";
			cout << std::endl;
			cout << "increase_status:" << std::endl;
			cout << param.increase_status[param.moving_type] << "  ";
			cout << std::endl;
			cout << "p_next:" << std::endl;
			cout << p_next[param.moving_type] << "  ";
			cout << std::endl;
			cout << "v_next:" << std::endl;
			cout << v_next[param.moving_type] << "  ";
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

		return finished[param.moving_type];
	}
	auto JRZ::collectNrt(PlanTarget &target)->void 
	{
		JCParam::jrz_count = 0;
		if (target.ret_code < 0)
		{
			JCParam::jrz_count.store(0);
		}
	}
	JRZ::~JRZ() = default;
	JRZ::JRZ(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"jrz\">"
				JOGC_PARAM_STRING
			"</Command>");
	}
	

	// 清理slavepool //
	auto ClearCon::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		auto&cs = aris::server::ControlServer::instance();
		if (cs.running())throw std::runtime_error("cs is running, please stop the cs using cs_stop!");

		auto &controller = target.controller;
		controller->slavePool().clear();

		std::vector<std::pair<std::string, std::any>> ret;
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
		if (cs.running())throw std::runtime_error("cs is running, please stop the cs using cs_stop!");

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

		std::vector<std::pair<std::string, std::any>> ret;
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
		if (cs.running())throw std::runtime_error("cs is running, please stop the cs using cs_stop!");

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

		std::vector<std::pair<std::string, std::any>> ret;
		target.ret = ret;
		target.option = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;

	}
	SetDH::SetDH(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"setdh\">"
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
		if (cs.running())throw std::runtime_error("cs is running, please stop the cs using cs_stop!");

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

		std::vector<std::pair<std::string, std::any>> ret;
		target.ret = ret;
		target.option = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;

	}
	SetPG::SetPG(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"setpgpath\">"
			"	<GroupParam>"
			"		<Param name=\"name\" default=\"test\"/>"
			"		<Param name=\"pe\" default=\"{0, 0, 0, -0, 0, -0}\"/>"
			"		<Param name=\"part_id\" default=\"6\"/>"
			"		<Param name=\"file_path\" default=\"/RobotGallery/Rokae/XB4/l0.data\"/>"
			"	</GroupParam>"
			"</Command>");
	}
	

	// 配置PG参数 //
	struct SetPPathParam
	{
		std::string name;
		std::vector<double> pe;
		std::vector<std::string> file_path;
	};
	auto SetPPath::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		auto&cs = aris::server::ControlServer::instance();
		//if (cs.running())throw std::runtime_error("cs is running, please stop the cs using cs_stop!");

		SetPPathParam param;
		param.pe.clear();
		param.pe.resize(6, 0.0);
		param.file_path.clear();

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
			else if (p.first == "file_path")
			{
				auto data = p.second;
				char *s_input = (char *)data.c_str();
				const char *split = ";";
				// 以‘;’为分隔符拆分字符串
				char *sp_input = strtok(s_input, split);
				std::string s_data;
				while (sp_input != NULL)
				{
					s_data = sp_input;
					param.file_path.push_back(s_data);
					sp_input = strtok(NULL, split);
				}
			}
		}

		for (int i = 0; i < param.file_path.size(); i++)
		{
			target.model->partPool().at(i).geometryPool().clear();
			target.model->partPool().at(i).geometryPool().add<FileGeometry>(param.name, param.file_path[i], param.pe.data());
		}

		std::vector<std::pair<std::string, std::any>> ret;
		target.ret = ret;
		target.option = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;

	}
	SetPPath::SetPPath(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"setppath\">"
			"	<GroupParam>"
			"		<Param name=\"name\" default=\"test\"/>"
			"		<Param name=\"pe\" default=\"{0.0, 0.0, 0.0, -0.0, 0.0, -0.0}\"/>"
			"		<Param name=\"file_path\" default=\"/RobotGallery/Rokae/XB4/l0.data;/RobotGallery/Rokae/XB4/l1.data;/RobotGallery/Rokae/XB4/l2.data;/RobotGallery/Rokae/XB4/l3.data;/RobotGallery/Rokae/XB4/l4.data;/RobotGallery/Rokae/XB4/l5.data;/RobotGallery/Rokae/XB4/l6.data\"/>"
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
		if (cs.running())throw std::runtime_error("cs is running, please stop the cs using cs_stop!");

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

		std::vector<std::pair<std::string, std::any>> ret;
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
		if (cs.running())throw std::runtime_error("cs is running, please stop the cs using cs_stop!");
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
		std::vector<std::pair<std::string, std::any>> ret;
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


	// 从硬件扫描从站 //
	auto ScanSlave::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		auto&cs = aris::server::ControlServer::instance();
		if (cs.running())throw std::runtime_error("cs is running, please stop the cs using cs_stop!");

		aris::control::EthercatMaster mst;
		mst.scan();

		std::vector<std::pair<std::string, std::any>> ret;
		ret.push_back(std::make_pair<std::string, std::any>("controller_xml", mst.xmlString()));

		target.ret = ret;
		target.option = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;
	}
	ScanSlave::ScanSlave(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"scanslave\">"
			"</Command>");
	}


	// 根据ESI补全从站信息 //
	struct GetEsiPdoListParam
	{
		int vendor_id;
		int product_code;
		int revision_num;
	};
	auto GetEsiPdoList::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		auto&cs = aris::server::ControlServer::instance();
		if (cs.running())throw std::runtime_error("cs is running, please stop the cs using cs_stop!");
		
		GetEsiPdoListParam param;
		for (auto &p : params)
		{
			if (p.first == "vendor_id")
			{
				param.vendor_id = std::stoi(p.second);
			}
			if (p.first == "product_code")
			{
				param.product_code = std::stoi(p.second);
			}
			if (p.first == "revision_num")
			{
				param.revision_num = std::stoi(p.second);
			}
		}

		aris::control::EthercatMaster mst;
		auto pdolist = mst.getPdoList(param.vendor_id, param.product_code, param.revision_num);
		
		std::vector<std::pair<std::string, std::any>> ret;
		ret.push_back(std::make_pair<std::string, std::any>("pdo_list_xml", pdolist));

		target.ret = ret;
		target.option = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;
	}
	GetEsiPdoList::GetEsiPdoList(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"getesipdolist\">"
			"	<GroupParam>"
			"		<Param name=\"vendor_id\" default=\"0\"/>"
			"		<Param name=\"product_code\" default=\"0\"/>"
			"		<Param name=\"revision_num\" default=\"0\"/>"
			"	</GroupParam>"
			"</Command>");
	}


	// 选择ESI路径 //
	struct SetEsiPathParam
	{
		std::vector<std::filesystem::path> path;
	};
	auto SetEsiPath::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		auto&cs = aris::server::ControlServer::instance();
		if (cs.running())throw std::runtime_error("cs is running, please stop the cs using cs_stop!");
		
		SetEsiPathParam param;
		param.path.clear();

		for (auto &p : params)
		{
			if (p.first == "path")
			{
				for (auto &filepath : std::filesystem::directory_iterator(p.second))
				{
					if (filepath.is_regular_file())param.path.push_back(filepath.path());
				}
			}
		}

		aris::control::EthercatMaster mst;
		mst.setEsiDirs(param.path);
		auto device_list = mst.getDeviceList();

		std::vector<std::pair<std::string, std::any>> ret;
		ret.push_back(std::make_pair<std::string, std::any>("device_list_xml", device_list));

		target.ret = ret;
		target.option = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;
	}
	SetEsiPath::SetEsiPath(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"setesipath\">"
			"	<GroupParam>"
			"		<Param name=\"path\" default=\"\"/>"
			"	</GroupParam>"
			"</Command>");
	}


	// update controller //
	auto Update::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		system("bash /home/kaanh/Document/kaanh/update_arislib.sh");
		std::vector<std::pair<std::string, std::any>> ret;
		target.ret = ret;
		target.option = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;
	}
	Update::Update(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"update_controller\">"
			"</Command>");
	}


	// 保存配置 //
	struct SaveXmlParam
	{
		std::string path;
	};
	auto SaveXml::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		auto&cs = aris::server::ControlServer::instance();
		if (cs.running())throw std::runtime_error("cs is running, please stop the cs using cs_stop!");

		SaveXmlParam param;
		for (auto &p : params)
		{
			if (p.first == "path")
			{
				param.path = p.second;
			}
		}
		const std::string xmlfile = "kaanh.xml";
		param.path = param.path + '/' + xmlfile;

		std::cout << "path:" << param.path << std::endl;
		cs.saveXmlFile(param.path.c_str());

		std::vector<std::pair<std::string, std::any>> ret;
		target.ret = ret;
		target.option = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;
	}
	SaveXml::SaveXml(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"savexml\">"
			"	<GroupParam>"
			"		<Param name=\"path\" default=\"\"/>"
			"	</GroupParam>"
			"</Command>");
	}


	auto GetXml::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		std::vector<std::pair<std::string, std::any>> ret_value;
		ret_value.push_back(std::make_pair(std::string("configure_xml"), target.server->xmlString()));
		target.ret = ret_value;
		target.option |= NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION;
	}
	GetXml::~GetXml() = default;
	GetXml::GetXml(const std::string &name) : Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"get_xml\">"
			"</Command>");
	}


	auto SetXml::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		// remove all symbols "{" "}"
		if (target.server->running())THROW_FILE_LINE("server is running, can't set xml");
		auto xml_str = params.at("xml").substr(1, params.at("xml").size() - 2);
		// 这一句要小心，此时 this 已被销毁，后面不能再调用this了 //
		target.server->loadXmlStr(xml_str);
		target.option |= NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION;

		std::vector<std::pair<std::string, std::any>> ret_value;
		target.ret = ret_value;
	}
	SetXml::~SetXml() = default;
	SetXml::SetXml(const std::string &name) : Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"set_xml\">"
			"	<Param name=\"xml\"/>"
			"</Command>");
	}


	auto Start::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		target.server->start();
		target.option |= NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION;

		std::vector<std::pair<std::string, std::any>> ret_value;
		target.ret = ret_value;
	}
	Start::~Start() = default;
	Start::Start(const std::string &name) : Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"cs_start\">"
			"</Command>");
	}


	auto Stop::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		target.server->stop();
		target.option |= NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION;

		std::vector<std::pair<std::string, std::any>> ret_value;
		target.ret = ret_value;
	}
	Stop::~Stop() = default;
	Stop::Stop(const std::string &name) : Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"cs_stop\">"
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
		
		std::vector<std::pair<std::string, std::any>> ret;
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
	

	//设置全局速度//
	struct SetVelParam
	{
		int vel_percent;
	};
	auto SetVel::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		SetVelParam param;
		for (auto &p : params)
		{
			if (p.first == "vel_percent")
			{
				param.vel_percent = std::stoi(p.second);
			}
		}
		g_vel_percent.store(param.vel_percent);

		std::vector<std::pair<std::string, std::any>> ret;
		target.ret = ret;
		target.option = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;
	}
	SetVel::SetVel(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"setvel\">"
			"	<GroupParam>"
			"		<Param name=\"vel_percent\" abbreviation=\"p\" default=\"0\"/>"
			"	</GroupParam>"
			"</Command>");
	}


	//var//
	struct VarParam
	{
		std::string name;
		std::string value;
		std::string type;
	};
	auto Var::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		VarParam param;
		for (auto &p : params)
		{
			if(p.first == "name")
			{
				param.name = p.second;
			}
			else if (p.first == "value")
			{
				param.value = p.second;
			}
			else if (p.first == "type")
			{
				param.type = p.second;
			}
		}
		aris::core::Calculator c;
		if ((param.type == "int") || (param.type == "double") || (param.type == "bool") || (param.type == "array"))
		{
			c.addVariable(param.name, target.model->calculator().calculateExpression(param.value));
		}
		else if (param.type == "string")
		{

		}

		std::vector<std::pair<std::string, std::any>> ret;
		target.ret = ret;
		target.option = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;
	}
	Var::Var(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"var\">"
			"	<GroupParam>"
			"		<Param name=\"name\" default=\"0\"/>"
			"		<Param name=\"value\" default=\"0\"/>"
			"		<Param name=\"type\" default=\"0\"/>"
			"	</GroupParam>"
			"</Command>");
	}


	//运行指令//
	struct RunParam
	{
		int goto_cmd_id;
		std::thread run;
	};
	std::mutex mymutex;
	std::mutex runmutex;
	bool is_running = false;
	auto is_auto_executing()->bool
	{
		std::unique_lock<std::mutex>runmutex;
		return is_running;
	}
	auto set_is_auto_executing(bool param)->bool
	{
		std::unique_lock<std::mutex>runmutex;
		bool now = is_running;
		is_running = param;
		return now;
	}
	auto Run::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		std::vector<std::pair<std::string, std::any>> run_ret;
		auto param = std::make_shared<RunParam>();		

		for (auto &p : params)
		{
			if (p.first == "forward")
			{
				//有指令在执行//
				if (is_auto_executing())
				{
					target.option = Plan::NOT_RUN_COLLECT_FUNCTION| Plan::NOT_RUN_EXECUTE_FUNCTION;
				}
				//没有指令在执行//
				else
				{
					const bool is_forward = true;
					set_is_auto_executing(is_forward);
					param->run = std::thread([&]()->void
					{
						try
						{
							auto&cs = aris::server::ControlServer::instance();
							{
								std::unique_lock<std::mutex> run_lock(mymutex);
								auto iter = cmdparam.cmd_vec.find(cmdparam.current_cmd_id);
								if (iter == cmdparam.cmd_vec.end())
								{
									set_is_auto_executing(false);
									return;
								}
								else
								{
									cmdparam.current_plan_id = iter->first;
								}			
							}
							cs.executeCmd(aris::core::Msg(cmdparam.cmd_vec[cmdparam.current_cmd_id]), [&](aris::plan::PlanTarget &target)->void
							{
								std::unique_lock<std::mutex> run_lock(mymutex);
								auto iter = cmdparam.cmd_vec.find(cmdparam.current_cmd_id);
								iter++;
								std::cout << "current_cmd_id:" << iter->second << std::endl;
								if (iter != cmdparam.cmd_vec.end())
								{
									cmdparam.current_plan_id = iter->first;
									cmdparam.current_cmd_id = iter->first;
								}
								else
								{
									cmdparam.current_plan_id = cmdparam.cmd_vec.rbegin()->first + 1;
								}
								set_is_auto_executing(false);
							});
						}
						catch (std::exception &e)
						{
							std::cout << e.what() << std::endl;
							LOG_ERROR << e.what() << std::endl;
						}
					});
				}	
			}
			else if (p.first == "goto")
			{
				param->goto_cmd_id = std::stoi(p.second);
				//有指令在执行//
				if (is_auto_executing())
				{
					target.option = Plan::NOT_RUN_COLLECT_FUNCTION | Plan::NOT_RUN_EXECUTE_FUNCTION;
				}
				//没有指令在执行//
				else
				{
					const bool is_goto = true;
					set_is_auto_executing(is_goto);
					param->run = std::thread([&]()->void
					{
						bool is_existed = false;
						{
							std::unique_lock<std::mutex> run_lock(mymutex);
							for (auto iter = cmdparam.cmd_vec.begin(); iter != cmdparam.cmd_vec.end(); ++iter)
							{
								if ((iter->first == param->goto_cmd_id) && (cmdparam.current_cmd_id != iter->first))
								{
									auto temp_iter = iter;
									temp_iter--;
									cmdparam.current_cmd_id = temp_iter->first;
									cmdparam.current_cmd_id = std::max(cmdparam.current_cmd_id, 0);
									is_existed = true;
									cmdparam.current_plan_id = cmdparam.cmd_vec.find(cmdparam.current_cmd_id)->first;
								}
							}
						}
						if (!is_existed)
						{
							set_is_auto_executing(false);
							return;
						}
						else
						{
							try
							{	
								aris::server::ControlServer::instance().executeCmd(aris::core::Msg(cmdparam.cmd_vec[cmdparam.current_cmd_id]), [&](aris::plan::PlanTarget &target)->void
								{
									std::unique_lock<std::mutex> run_lock(mymutex);
									auto iter = cmdparam.cmd_vec.find(cmdparam.current_cmd_id);
									iter++;
									if (iter != cmdparam.cmd_vec.end())
									{
										cmdparam.current_plan_id = iter->first;
										cmdparam.current_cmd_id = iter->first;
									}
									else
									{
										cmdparam.current_plan_id = cmdparam.cmd_vec.rbegin()->first + 1;
									}
									const bool is_not_goto = false;
									set_is_auto_executing(is_not_goto);
								});
							}
							catch (std::exception &e)
							{
								std::cout << e.what() << std::endl;
								LOG_ERROR << e.what() << std::endl;
							}
						}
					});
				}

			}
			else if (p.first == "start")
			{	
				g_is_auto.store(true);
				//有指令在执行//
				if (is_auto_executing())
				{
					target.option = Plan::NOT_RUN_COLLECT_FUNCTION | Plan::NOT_RUN_EXECUTE_FUNCTION;
				}
				//没有指令在执行//
				else
				{
					const bool is_start = true;
					set_is_auto_executing(is_start);
					param->run = std::thread([&]()->void
					{
						try 
						{
							int begin_cmd_id = 0, end_cmd_id = 0;
							auto&cs = aris::server::ControlServer::instance();
							{
								std::unique_lock<std::mutex> run_lock(mymutex);
								auto iter = cmdparam.cmd_vec.find(cmdparam.current_cmd_id);
								if (iter == cmdparam.cmd_vec.end())
								{
									set_is_auto_executing(false);
									return;
								}
								else
								{
									cmdparam.current_plan_id = iter->first;
								}
							}
							for (auto iter = cmdparam.cmd_vec.find(cmdparam.current_cmd_id); iter != cmdparam.cmd_vec.end(); ++iter)
							{
								cs.executeCmd(aris::core::Msg(iter->second), [iter](aris::plan::PlanTarget &target)->void
								{
									std::unique_lock<std::mutex> run_lock(mymutex);
									auto temp_iter = iter;
									temp_iter++;
									if (temp_iter != cmdparam.cmd_vec.end())
									{
										cmdparam.current_plan_id = temp_iter->first;
										cmdparam.current_cmd_id = temp_iter->first;
									}
									else
									{
										cmdparam.current_plan_id = cmdparam.cmd_vec.rbegin()->first + 1;
										const bool is_not_start = false;
										set_is_auto_executing(is_not_start);
									}
									g_is_auto.store(false);
								});
							}
						}
						catch (std::exception &e)
						{
							std::cout << e.what() << std::endl;
							LOG_ERROR << e.what() << std::endl;
						}
					});
				}	
			}
			else if (p.first == "pause") 
			{ 
				std::unique_lock<std::mutex> run_lock(mymutex);
				const bool is_pause = false;
				set_is_auto_executing(is_pause);

				target.option = Plan::NOT_RUN_COLLECT_FUNCTION | Plan::NOT_RUN_EXECUTE_FUNCTION;
			}
			else if (p.first == "stop")
			{
				std::unique_lock<std::mutex> run_lock(mymutex);
				cmdparam.current_cmd_id = 0;
				cmdparam.current_plan_id = -1;
				const bool is_stop = false;
				set_is_auto_executing(is_stop);

				target.option = Plan::NOT_RUN_COLLECT_FUNCTION | Plan::NOT_RUN_EXECUTE_FUNCTION;
			}
		}
		target.ret = run_ret;
		target.param = param;
	}
	auto Run::collectNrt(aris::plan::PlanTarget &target)->void
	{
		auto param = std::any_cast<std::shared_ptr<RunParam>>(target.param);
		param->run.join();
	}
	Run::Run(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"run\">"
			"	<GroupParam>"
			"		<UniqueParam>"
			"			<Param name=\"forward\"/>"
			"			<Param name=\"goto\" default=\"2\"/>"
			"			<Param name=\"start\"/>"
			"			<Param name=\"pause\"/>"
			"			<Param name=\"stop\"/>"
			"		</UniqueParam>"
			"	</GroupParam>"
			"</Command>");
	}


	//编程界面指令//
	bool splitString(std::string spCharacter, const std::string& objString, std::map<int, std::string>& stringVector)
	{
		if (objString.length() == 0)
		{
			return true;
		}
		size_t posBegin = 0;
		size_t posEnd = 0;

		while (posEnd != std::string::npos)
		{
			posBegin = posEnd;
			posEnd = objString.find(spCharacter, posBegin);

			if (posBegin == posEnd)
			{
				posEnd += spCharacter.size();
				continue;
			}
			if (posEnd == std::string::npos)
			{
				break;
			}
			std::string str = objString.substr(posBegin, posEnd - posBegin);
			auto sep_pos = str.find(":");
			auto id = str.substr(0, sep_pos);
			auto command = str.substr(sep_pos + 1);
			stringVector.insert(std::pair<int, std::string>(std::stoi(id), command.c_str()));
			posEnd += spCharacter.size();
		}
		return true;
	}
	auto onReceivedMsg(aris::core::Socket *socket, aris::core::Msg &msg)->int
	{
		std::string msg_data = msg.toString();
		
		{
			LOG_INFO << "receive cmd:"
				<< msg.header().msg_size_ << "&"
				<< msg.header().msg_id_ << "&"
				<< msg.header().msg_type_ << "&"
				<< msg.header().reserved1_ << "&"
				<< msg.header().reserved2_ << "&"
				<< msg.header().reserved3_ << ":"
				<< msg_data << std::endl;
		}

		//std::string data = "collectcmd --cmdlist = {1:moveJ\r\n2:moveJ\r\n3:moveL\r\n}";
		std::string s = " --";
		auto cmd_name_pos = msg_data.find(s);
		auto cmd_name = msg_data.substr(0, cmd_name_pos);

		//解析指令失败//
		if (cmd_name.empty())
		{
			aris::core::Msg ret_msg(msg);
			std::vector<std::pair<std::string, std::any>> *js;
			js->push_back(std::make_pair<std::string, std::any>("return_code", -3));
			js->push_back(std::make_pair<std::string, std::any>("return_message", std::string("PARSE_EXCEPTION")));
			ret_msg.copy(aris::server::parse_ret_value(*js));
			// return back to source
			try
			{
				socket->sendMsg(ret_msg);
			}
			catch (std::exception &e)
			{
				std::cout << e.what() << std::endl;
				LOG_ERROR << e.what() << std::endl;
			}
		}
		else
		{
			if (strcmp(cmd_name.c_str(), "collectcmd") == 0)
			{
				{
					std::unique_lock<std::mutex> run_lock(mymutex);
					cmdparam.cmd_vec.clear();
					cmdparam.current_cmd_id = 0;
					cmdparam.current_plan_id = -1;
				}

				auto begin_pos = msg_data.find("{");
				auto end_pos = msg_data.rfind("}");
				auto cmd_str = msg_data.substr(begin_pos + 1, end_pos - 1 - begin_pos);
				const std::string split = "\\r\\n";
				splitString(split, cmd_str, cmdparam.cmd_vec);

				/*
				char *cmd_input = (char *)cmd_str.c_str();
				const char *split = "\\r\\n";
				char *cmd = strtok(cmd_input, split);
				int i = 0;
				while (cmd != NULL)
				{
					std::string str = cmd;
					auto sep_pos = str.find(":");
					auto id = str.substr(0, sep_pos);
					auto command = str.substr(sep_pos + 1);
					cmdparam.cmd_vec.push_back(std::make_pair<std::string, std::string>(id.c_str(), command.c_str()));
					cmd = strtok(NULL, split);
				}
				*/
				
				auto iter = cmdparam.cmd_vec.begin();
				if (iter != cmdparam.cmd_vec.end())
				{
					cmdparam.current_cmd_id = iter->first;
					std::cout << "cmd_vec:" << iter->second << std::endl;
				}
				else
				{
					std::cout << "cmd_vec is null" << std::endl;
				}	
			}
			else
			{
				try
				{
					aris::server::ControlServer::instance().executeCmd(aris::core::Msg(msg), [socket, msg](aris::plan::PlanTarget &target)->void
					{
						// make return msg
						aris::core::Msg ret_msg(msg);
						// only copy if it is a str
						if (auto str = std::any_cast<std::string>(&target.ret))
						{
							ret_msg.copy(*str);
						}
						else if (auto js = std::any_cast<std::vector<std::pair<std::string, std::any>>>(&target.ret))
						{
							js->push_back(std::make_pair<std::string, std::any>("return_code", target.ret_code));
							js->push_back(std::make_pair<std::string, std::any>("return_message", std::string(target.ret_msg)));
							ret_msg.copy(aris::server::parse_ret_value(*js));
						}

						// return back to source
						try
						{
							socket->sendMsg(ret_msg);
						}
						catch (std::exception &e)
						{
							std::cout << e.what() << std::endl;
							LOG_ERROR << e.what() << std::endl;
						}
					});
				}
				catch (std::exception &e)
				{
					std::vector<std::pair<std::string, std::any>> ret_pair;
					ret_pair.push_back(std::make_pair<std::string, std::any>("return_code", int(aris::plan::PlanTarget::PARSE_EXCEPTION)));
					ret_pair.push_back(std::make_pair<std::string, std::any>("return_message", std::string(e.what())));
					std::string ret_str = aris::server::parse_ret_value(ret_pair);

					//std::cout << ret_str << std::endl;
					LOG_ERROR << ret_str << std::endl;

					try
					{
						aris::core::Msg m = msg;
						m.copy(ret_str);
						socket->sendMsg(m);
					}
					catch (std::exception &e)
					{
						std::cout << e.what() << std::endl;
						LOG_ERROR << e.what() << std::endl;
					}
				}
			}
		}

		return 0;
	}
	auto onReceivedConnection(aris::core::Socket *sock, const char *ip, int port)->int
	{
		std::cout << "socket receive connection" << std::endl;
		LOG_INFO << "socket receive connection:\n"
			<< std::setw(aris::core::LOG_SPACE_WIDTH) << "|" << "  ip:" << ip << "\n"
			<< std::setw(aris::core::LOG_SPACE_WIDTH) << "|" << "port:" << port << std::endl;
		return 0;
	}
	auto onLoseConnection(aris::core::Socket *socket)->int
	{
		std::cout << "socket lose connection" << std::endl;
		LOG_INFO << "socket lose connection" << std::endl;
		for (;;)
		{
			try
			{
				socket->startServer(socket->port());
				break;
			}
			catch (std::runtime_error &e)
			{
				std::cout << e.what() << std::endl << "will try to restart server socket in 1s" << std::endl;
				LOG_ERROR << e.what() << std::endl << "will try to restart server socket in 1s" << std::endl;
				std::this_thread::sleep_for(std::chrono::seconds(1));
			}
		}
		std::cout << "socket restart successful" << std::endl;
		LOG_INFO << "socket restart successful" << std::endl;

		return 0;
	}
	auto ProInterface::open()->void { sock_->startServer(); }
	auto ProInterface::close()->void { sock_->stop(); }
	auto ProInterface::loadXml(const aris::core::XmlElement &xml_ele)->void
	{
		Interface::loadXml(xml_ele);
		this->sock_ = findOrInsertType<aris::core::Socket>("socket", "", "5866", aris::core::Socket::WEB);

		sock_->setOnReceivedMsg(onReceivedMsg);
		sock_->setOnReceivedConnection(onReceivedConnection);
		sock_->setOnLoseConnection(onLoseConnection);
	}
	ProInterface::ProInterface(const std::string &name, const std::string &port, aris::core::Socket::TYPE type) :Interface(name)
	{
		sock_ = &add<aris::core::Socket>("socket", "", port, type);

		sock_->setOnReceivedMsg(onReceivedMsg);
		sock_->setOnReceivedConnection(onReceivedConnection);
		sock_->setOnLoseConnection(onLoseConnection);
	}


	// 停止指令 //
	auto MoveSt::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		g_move_stop = true;
		std::vector<std::pair<std::string, std::any>> ret;
		target.ret = ret;
		target.option = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;
	}
	MoveSt::MoveSt(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"mvstop\">"
			"</Command>");
	}


	// 手动--自动模式切换； manual=1，手动;manual=0，自动//
	struct SwitchParam
	{
		bool is_manual;
	};
	auto Switch::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		SwitchParam param;
		for (auto &p : params)
		{
			if (p.first == "manual")
			{
				param.is_manual = std::stoi(p.second);
			}
		}
		g_is_manual.store(param.is_manual);
		std::vector<std::pair<std::string, std::any>> ret;
		target.ret = ret;
		target.option = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;
	}
	Switch::Switch(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"switch\">"
			"		<Param name=\"manual\" default=\"1\"/>"
			"</Command>");
	}

    auto createPlanRoot()->std::unique_ptr<aris::plan::PlanRoot>
	{
        std::unique_ptr<aris::plan::PlanRoot> plan_root(new aris::plan::PlanRoot);

        plan_root->planPool().add<kaanh::Enable>();
        plan_root->planPool().add<kaanh::Disable>();
        plan_root->planPool().add<kaanh::Home>();
        plan_root->planPool().add<kaanh::Mode>();
        plan_root->planPool().add<kaanh::Sleep>();
        plan_root->planPool().add<kaanh::Recover>();
        auto &rs = plan_root->planPool().add<kaanh::Reset>();
        rs.command().findParam("pos")->setDefaultValue("{0.5,0.3925,0.7899,0.5,0.5,0.5}");
		
        //qifan//
        //rs.command().findParam("pos")->setDefaultValue("{0.5,0.353,0.5,0.5,0.5,0.5}");

        plan_root->planPool().add<kaanh::MoveAbsJ>();
        plan_root->planPool().add<kaanh::MoveL>();
        plan_root->planPool().add<kaanh::MoveJ>();
		plan_root->planPool().add<kaanh::MoveSt>();

		plan_root->planPool().add<kaanh::Get>();
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
		plan_root->planPool().add<kaanh::Start>();
		plan_root->planPool().add<kaanh::Stop>();
		plan_root->planPool().add<kaanh::SetCT>();
		plan_root->planPool().add<kaanh::SetVel>();
		plan_root->planPool().add<kaanh::Run>();
		plan_root->planPool().add<kaanh::MoveF>();
		plan_root->planPool().add<kaanh::Switch>();

		plan_root->planPool().add<MoveXYZ>();
		plan_root->planPool().add<MoveJoint>();
		plan_root->planPool().add<MoveDistal>();
        plan_root->planPool().add<DistalTest>();
		plan_root->planPool().add<MovePressure>();
		plan_root->planPool().add<MoveFeed>();
		plan_root->planPool().add<MovePressureToolYZ>();
		plan_root->planPool().add<MovePressureToolXY>();

		plan_root->planPool().add<MovePressureToolXLine>();
		plan_root->planPool().add<MovePressureToolYLine>();
		plan_root->planPool().add<MovePressureToolXSine>();
		plan_root->planPool().add<MoveForceXSine>();
		plan_root->planPool().add<MoveForceCircle>();
		plan_root->planPool().add<MoveForceCurve>();

		plan_root->planPool().add<GetForce>();
        plan_root->planPool().add<MoveSeriesGK>();
        plan_root->planPool().add<ForceDirect>();

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

		plan_root->planPool().add<DoubleSPlan>();

		plan_root->planPool().add<cplan::MoveCircle>();
		plan_root->planPool().add<cplan::MoveTroute>();
		plan_root->planPool().add<cplan::RemoveFile>();
		plan_root->planPool().add<cplan::MoveinModel>();
		plan_root->planPool().add<cplan::FMovePath>();
		plan_root->planPool().add<cplan::OpenFile>();
		
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
