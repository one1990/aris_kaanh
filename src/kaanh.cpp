#include <algorithm>
#include"kaanh.h"
#include"iir.h"
#include"forcecontrol.h"


using namespace aris::dynamic;
using namespace aris::plan;

extern double fce_data[buffer_length];
extern int data_num, data_num_send;
extern std::atomic_int which_di;
extern std::atomic_bool is_automatic;

namespace kaanh
{
	auto createControllerRokaeXB4()->std::unique_ptr<aris::control::Controller>	/*函数返回的是一个类指针，指针指向Controller,controller的类型是智能指针std::unique_ptr*/
	{
		std::unique_ptr<aris::control::Controller> controller(new aris::control::EthercatController);/*创建std::unique_ptr实例*/

		for (aris::Size i = 0; i < 6; ++i)
		{
			
#ifdef WIN32
			double pos_offset[6]
			{
				0,   0,	  0,   0,   0,   0
			};
#endif
#ifdef UNIX
			double pos_offset[6]
			{
				0.00293480352126769,0.317555328381088,-0.292382537944081,0.0582675097338009,1.53363576057128,17.1269434336436
			};
#endif
			double pos_factor[6]
			{
				131072.0 * 81 / 2 / PI, 131072.0 * 81 / 2 / PI, 131072.0 * 81 / 2 / PI, 131072.0 * 72.857 / 2 / PI, 131072.0 * 81 / 2 / PI, 131072.0 * 50 / 2 / PI
			};
			double max_pos[6]
			{
				170.0 / 360 * 2 * PI, 130.0 / 360 * 2 * PI,	50.0 / 360 * 2 * PI, 170.0 / 360 * 2 * PI, 117.0 / 360 * 2 * PI, 360.0 / 360 * 2 * PI,
			};
			double min_pos[6]
			{
				-170.0 / 360 * 2 * PI, - 84.0 / 360 * 2 * PI, - 188.0 / 360 * 2 * PI, - 170.0 / 360 * 2 * PI, - 117.0 / 360 * 2 * PI, - 360.0 / 360 * 2 * PI
			};
			double max_vel[6]
			{
				310.0 / 360 * 2 * PI, 240.0 / 360 * 2 * PI, 310.0 / 360 * 2 * PI, 250.0 / 360 * 2 * PI, 295.0 / 360 * 2 * PI, 500.0 / 360 * 2 * PI,
			};
			double max_acc[6]
			{
				1500.0 / 360 * 2 * PI, 1500.0 / 360 * 2 * PI, 1500.0 / 360 * 2 * PI, 1750.0 / 360 * 2 * PI, 1500.0 / 360 * 2 * PI, 2500.0 / 360 * 2 * PI,
			};
			
			std::string xml_str =
                "<m" + std::to_string(i) + " type=\"EthercatMotion\" phy_id=\"" + std::to_string(i) + "\" product_code=\"0x0\""
				" vendor_id=\"0x000002E1\" revision_num=\"0x29001\" dc_assign_activate=\"0x0300\""
				" min_pos=\"" + std::to_string(min_pos[i]) + "\" max_pos=\"" + std::to_string(max_pos[i]) + "\" max_vel=\"" + std::to_string(max_vel[i]) + "\" min_vel=\"" + std::to_string(-max_vel[i]) + "\""
				" max_acc=\"" + std::to_string(max_acc[i]) + "\" min_acc=\"" + std::to_string(-max_acc[i]) + "\" max_pos_following_error=\"0.1\" max_vel_following_error=\"0.5\""
				" home_pos=\"0\" pos_factor=\"" + std::to_string(pos_factor[i]) + "\" pos_offset=\"" + std::to_string(pos_offset[i]) + "\">"
				"	<sm_pool type=\"SyncManagerPoolObject\">"
				"		<sm type=\"SyncManager\" is_tx=\"false\"/>"
				"		<sm type=\"SyncManager\" is_tx=\"true\"/>"
				"		<sm type=\"SyncManager\" is_tx=\"false\">"
				"			<index_1600 type=\"Pdo\" default_child_type=\"PdoEntry\" index=\"0x1600\" is_tx=\"false\">"
				"				<control_word index=\"0x6040\" subindex=\"0x00\" size=\"16\"/>"
				"				<mode_of_operation index=\"0x6060\" subindex=\"0x00\" size=\"8\"/>"
				"				<target_pos index=\"0x607A\" subindex=\"0x00\" size=\"32\"/>"
				"				<target_vel index=\"0x60FF\" subindex=\"0x00\" size=\"32\"/>"
				"				<offset_vel index=\"0x60B1\" subindex=\"0x00\" size=\"32\"/>"
				"				<target_tor index=\"0x6071\" subindex=\"0x00\" size=\"16\"/>"
				"			</index_1600>"
				"		</sm>"
				"		<sm type=\"SyncManager\" is_tx=\"true\">"
				"			<index_1a00 type=\"Pdo\" default_child_type=\"PdoEntry\" index=\"0x1A00\" is_tx=\"true\">"
				"				<status_word index=\"0x6041\" subindex=\"0x00\" size=\"16\"/>"
				"				<mode_of_display index=\"0x6061\" subindex=\"0x00\" size=\"8\"/>"
				"				<pos_actual_value index=\"0x6064\" subindex=\"0x00\" size=\"32\"/>"
				"				<vel_actual_value index=\"0x606c\" subindex=\"0x00\" size=\"32\"/>"
				"				<cur_actual_value index=\"0x6078\" subindex=\"0x00\" size=\"16\"/>"
				"			</index_1a00>"
				"		</sm>"
				"	</sm_pool>"
				"	<sdo_pool type=\"SdoPoolObject\" default_child_type=\"Sdo\">"
				"	</sdo_pool>"
				"</m" + std::to_string(i) + ">";

			controller->slavePool().add<aris::control::EthercatMotion>().loadXmlStr(xml_str);
		}

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
		double pq_ee_i[]{ 0.398, 0.0, 0.6295, 0.0, 0.0, 0.0, 1.0 };
		double pm_ee_i[16];
		double pm_ee_j[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };

		s_pq2pm(pq_ee_i, pm_ee_i);

		auto &makI = p6.markerPool().add<Marker>("ee_makI", pm_ee_i);
		auto &makJ = model->ground().markerPool().add<Marker>("ee_makJ", pm_ee_j);
		auto &ee = model->generalMotionPool().add<aris::dynamic::GeneralMotion>("ee", &makI, &makJ, false);

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
	// 获取驱动器当前位置，并设置为起始位置 //
	class MoveInit : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
		{
			target.option |=
				Plan::USE_TARGET_POS |
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

		}
		auto virtual executeRT(PlanTarget &target)->int
		{
			// 访问主站 //
			auto controller = dynamic_cast<aris::control::EthercatController*>(target.master);

			for (Size i = 0; i < 6; ++i)
			{
				target.model->motionPool().at(i).setMp(controller->motionAtAbs(i).actualPos());
			}

			if (!target.model->solverPool().at(1).kinPos())return -1;
			return 1000-target.count;
		}
		auto virtual collectNrt(PlanTarget &target)->void {}
		explicit MoveInit(const std::string &name = "MoveInit_plan"): Plan(name)
		{
			command().loadXmlStr(
				"<moveInit>"
				"</moveInit>");
		}
	};

	// 末端四元数xyz方向余弦轨迹；速度前馈//
	struct MoveXParam
	{
		double x, y, z;
		double time;
	};
	class MoveX : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void 
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

			target.option |=
				//用于使用模型轨迹驱动电机//
				Plan::USE_TARGET_POS |
				Plan::USE_VEL_OFFSET |
#ifdef WIN32
				Plan::NOT_CHECK_POS_MIN |
				Plan::NOT_CHECK_POS_MAX |
				Plan::NOT_CHECK_POS_CONTINUOUS |
				Plan::NOT_CHECK_POS_CONTINUOUS_AT_START |
				Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER|
				Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START|
				Plan::NOT_CHECK_POS_FOLLOWING_ERROR|
#endif
				Plan::NOT_CHECK_VEL_MIN |
				Plan::NOT_CHECK_VEL_MAX |
				Plan::NOT_CHECK_VEL_CONTINUOUS |
				Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START |
				Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;

		}
		auto virtual executeRT(PlanTarget &target)->int 
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

			if (!target.model->solverPool().at(0).kinPos())return -1;
			target.model->solverPool().at(0).kinVel();

			// 访问主站 //
			auto controller = dynamic_cast<aris::control::Controller*>(target.master);

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
		auto virtual collectNrt(PlanTarget &target)->void {}

		explicit MoveX(const std::string &name = "MoveX_plan") :Plan(name)
		{
			command().loadXmlStr(
				"<moveX>"
				"	<group type=\"GroupParam\" default_child_type=\"Param\">"
				"		<unique_pos type=\"UniqueParam\" default_child_type=\"Param\" default=\"x\">"
				"			<x default=\"0.1\"/>"
				"			<y default=\"0.1\"/>"
				"			<z default=\"0.1\"/>"
				"		</unique_pos>"
				"		<time default=\"1.0\" abbreviation=\"t\"/>"
				"	</group>"
				"</moveX>");
		}

	};

	// 单关节正弦往复轨迹 //
	struct MoveJSParam
	{
		double j[6];
		double time;
		uint32_t timenum;
		std::vector<bool> joint_active_vec;
	};
	class MoveJS : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
		{
			//MoveJSParam param = {{0.0,0.0,0.0,0.0,0.0,0.0},0.0,0};
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

			target.option |=
				Plan::USE_TARGET_POS |
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

		}
		auto virtual executeRT(PlanTarget &target)->int
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

			if (!target.model->solverPool().at(1).kinPos())return -1;

			// 访问主站 //
			auto controller = dynamic_cast<aris::control::Controller*>(target.master);

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
		auto virtual collectNrt(PlanTarget &target)->void {}

		explicit MoveJS(const std::string &name = "MoveJS_plan") :Plan(name)
		{
			command().loadXmlStr(
				"<moveJS>"
				"	<group type=\"GroupParam\" default_child_type=\"Param\">"
				"		<j1 default=\"current_pos\"/>"
				"		<j2 default=\"current_pos\"/>"
				"		<j3 default=\"current_pos\"/>"
				"		<j4 default=\"current_pos\"/>"
				"		<j5 default=\"current_pos\"/>"
				"		<j6 default=\"current_pos\"/>"
				"		<time default=\"1.0\" abbreviation=\"t\"/>"
				"		<timenum default=\"2\" abbreviation=\"n\"/>"
				"	</group>"
				"</moveJS>");
		}
	};

	// 任意关节正弦往复轨迹 //
	struct MoveJSNParam
	{
		std::vector<double> axis_pos_vec;
		std::vector<double> axis_time_vec;
		std::vector<bool> joint_active_vec;
		uint32_t timenum;
	};
	class MoveJSN : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
		{
			MoveJSNParam param;
			auto c = dynamic_cast<aris::control::Controller*>(target.master);
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

			target.option |=
				Plan::USE_TARGET_POS |
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

		}
		auto virtual executeRT(PlanTarget &target)->int
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

			if (!target.model->solverPool().at(1).kinPos())return -1;

			// 访问主站 //
			auto controller = dynamic_cast<aris::control::Controller*>(target.master);

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
		auto virtual collectNrt(PlanTarget &target)->void {}

		explicit MoveJSN(const std::string &name = "MoveJSN_plan") :Plan(name)
		{
			command().loadXmlStr(
				"<moveJSN>"
				"	<group type=\"GroupParam\" default_child_type=\"Param\">"
				"		<pos default=\"{0.1,0.2,0.2,0.2,0.2,0.2}\" abbreviation=\"p\"/>"
				"		<time default=\"{1.0,1.0,1.0,1.0,1.0,1.0}\" abbreviation=\"t\"/>"
				"		<timenum default=\"2\" abbreviation=\"n\"/>"
				"	</group>"
				"</moveJSN>");
		}
	};

	// 单关节相对运动轨迹--输入单个关节，角度位置；关节按照梯形速度轨迹执行；速度前馈//
	struct MoveJRParam
	{
		double vel, acc, dec;
		std::vector<double> joint_pos_vec, begin_joint_pos_vec;
		std::vector<bool> joint_active_vec;
	};
	class MoveJR : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
		{
			auto c = dynamic_cast<aris::control::Controller*>(target.master);
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

			target.option |=
				Plan::USE_TARGET_POS |
				Plan::USE_VEL_OFFSET |
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

		}
		auto virtual executeRT(PlanTarget &target)->int
		{
			auto &param = std::any_cast<MoveJRParam&>(target.param);
			auto controller = dynamic_cast<aris::control::Controller *>(target.master);

			if (target.count == 1)
			{
				for (Size i = 0; i < param.joint_active_vec.size(); ++i)
				{
					if (param.joint_active_vec[i])
					{
						param.begin_joint_pos_vec[i] = target.model->motionPool()[i].mp();
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
					target.model->motionPool().at(i).setMp(p);
					target.model->motionPool().at(i).setMv(v*1000);
					total_count = std::max(total_count, t_count);
				}
			}

			if (!target.model->solverPool().at(1).kinPos())return -1;

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

			return total_count - target.count;
		}
		auto virtual collectNrt(PlanTarget &target)->void {}

		explicit MoveJR(const std::string &name = "MoveJR_plan") :Plan(name)
		{
			command().loadXmlStr(
				"<moveJR default_child_type=\"Param\">"
				"	<group type=\"GroupParam\" default_child_type=\"Param\">"
				"		<limit_time default=\"5000\"/>"
				"		<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"all\">"
				"			<all abbreviation=\"a\"/>"
				"			<motion_id abbreviation=\"m\" default=\"0\"/>"
				"			<physical_id abbreviation=\"p\" default=\"0\"/>"
				"			<slave_id abbreviation=\"s\" default=\"0\"/>"
				"		</unique>"
				"		<pos default=\"0\"/>"
				"		<vel default=\"0.5\"/>"
				"		<acc default=\"1\"/>"
				"		<dec default=\"1\"/>"
				"		<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_all\">"
				"			<check_all/>"
				"			<check_none/>"
				"			<group type=\"GroupParam\" default_child_type=\"Param\">"
				"				<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos\">"
				"					<check_pos/>"
				"					<not_check_pos/>"
				"					<group type=\"GroupParam\" default_child_type=\"Param\">"
				"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_max\">"
				"							<check_pos_max/>"
				"							<not_check_pos_max/>"
				"						</unique>"
				"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_min\">"
				"							<check_pos_min/>"
				"							<not_check_pos_min/>"
				"						</unique>"
				"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_continuous\">"
				"							<check_pos_continuous/>"
				"							<not_check_pos_continuous/>"
				"						</unique>"
				"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_continuous_at_start\">"
				"							<check_pos_continuous_at_start/>"
				"							<not_check_pos_continuous_at_start/>"
				"						</unique>"
				"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_continuous_second_order\">"
				"							<check_pos_continuous_second_order/>"
				"							<not_check_pos_continuous_second_order/>"
				"						</unique>"
				"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_continuous_second_order_at_start\">"
				"							<check_pos_continuous_second_order_at_start/>"
				"							<not_check_pos_continuous_second_order_at_start/>"
				"						</unique>"
				"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_following_error\">"
				"							<check_pos_following_error/>"
				"							<not_check_pos_following_error />"
				"						</unique>"
				"					</group>"
				"				</unique>"
				"				<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel\">"
				"					<check_vel/>"
				"					<not_check_vel/>"
				"					<group type=\"GroupParam\" default_child_type=\"Param\">"
				"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel_max\">"
				"							<check_vel_max/>"
				"							<not_check_vel_max/>"
				"						</unique>"
				"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel_min\">"
				"							<check_vel_min/>"
				"							<not_check_vel_min/>"
				"						</unique>"
				"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel_continuous\">"
				"							<check_vel_continuous/>"
				"							<not_check_vel_continuous/>"
				"						</unique>"
				"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel_continuous_at_start\">"
				"							<check_vel_continuous_at_start/>"
				"							<not_check_vel_continuous_at_start/>"
				"						</unique>"
				"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel_following_error\">"
				"							<check_vel_following_error/>"
				"							<not_check_vel_following_error />"
				"						</unique>"
				"					</group>"
				"				</unique>"
				"			</group>"
				"		</unique>"
				"	</group>"
				"</moveJR>");
		}
	};

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
	class MoveTTT : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
		{
			auto c = dynamic_cast<aris::control::Controller*>(target.master);
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

			target.option |=
				Plan::USE_TARGET_POS |
				Plan::USE_VEL_OFFSET |
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

		}
		auto virtual executeRT(PlanTarget &target)->int
		{
			auto &param = std::any_cast<MoveTTTParam&>(target.param);
			auto controller = dynamic_cast<aris::control::Controller *>(target.master);

			if (target.count == 1)
			{
				for (Size i = 0; i < param.joint_active_vec.size(); ++i)
				{
					if (param.joint_active_vec[i])
					{
						param.begin_joint_pos_vec[i] = controller->motionPool()[i].actualPos();
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
					auto result = aris::plan::moveAbsolute2(param.begin_joint_pos_vec[i], param.begin_axis_vel_vec[i], param.begin_axis_acc_vec[i], param.joint_pos_vec[i], param.axis_vel_vec[i], param.axis_acc_vec[i], controller->motionPool()[i].maxVel(), controller->motionPool()[i].maxAcc(), controller->motionPool()[i].maxAcc(), 1e-3, 1e-10, p, v, a, t_count);
					controller->motionAtAbs(i).setTargetPos(p);
					total_count = std::max(total_count, t_count);

					param.begin_joint_pos_vec[i] = p;
					param.begin_axis_vel_vec[i] = v;
					param.begin_axis_acc_vec[i] = a;
				}
			}

			//if (!target.model->solverPool().at(1).kinPos())return -1;

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

			return total_count - target.count;
		}
		auto virtual collectNrt(PlanTarget &target)->void {}

		explicit MoveTTT(const std::string &name = "MoveTTT_plan") :Plan(name)
		{
			command().loadXmlStr(
				"<moveTTT default_child_type=\"Param\">"
				"	<group type=\"GroupParam\" default_child_type=\"Param\">"
				"		<limit_time default=\"5000\"/>"
				"		<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"all\">"
				"			<all abbreviation=\"a\"/>"
				"			<motion_id abbreviation=\"m\" default=\"0\"/>"
				"			<physical_id abbreviation=\"p\" default=\"0\"/>"
				"			<slave_id abbreviation=\"s\" default=\"0\"/>"
				"		</unique>"
				"		<pos default=\"0\"/>"
				"		<vel default=\"0.1\"/>"
				"		<acc default=\"0.2\"/>"
				"		<dec default=\"0.2\"/>"
				"		<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_all\">"
				"			<check_all/>"
				"			<check_none/>"
				"			<group type=\"GroupParam\" default_child_type=\"Param\">"
				"				<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos\">"
				"					<check_pos/>"
				"					<not_check_pos/>"
				"					<group type=\"GroupParam\" default_child_type=\"Param\">"
				"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_max\">"
				"							<check_pos_max/>"
				"							<not_check_pos_max/>"
				"						</unique>"
				"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_min\">"
				"							<check_pos_min/>"
				"							<not_check_pos_min/>"
				"						</unique>"
				"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_continuous\">"
				"							<check_pos_continuous/>"
				"							<not_check_pos_continuous/>"
				"						</unique>"
				"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_continuous_at_start\">"
				"							<check_pos_continuous_at_start/>"
				"							<not_check_pos_continuous_at_start/>"
				"						</unique>"
				"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_continuous_second_order\">"
				"							<check_pos_continuous_second_order/>"
				"							<not_check_pos_continuous_second_order/>"
				"						</unique>"
				"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_continuous_second_order_at_start\">"
				"							<check_pos_continuous_second_order_at_start/>"
				"							<not_check_pos_continuous_second_order_at_start/>"
				"						</unique>"
				"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_following_error\">"
				"							<check_pos_following_error/>"
				"							<not_check_pos_following_error />"
				"						</unique>"
				"					</group>"
				"				</unique>"
				"				<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel\">"
				"					<check_vel/>"
				"					<not_check_vel/>"
				"					<group type=\"GroupParam\" default_child_type=\"Param\">"
				"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel_max\">"
				"							<check_vel_max/>"
				"							<not_check_vel_max/>"
				"						</unique>"
				"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel_min\">"
				"							<check_vel_min/>"
				"							<not_check_vel_min/>"
				"						</unique>"
				"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel_continuous\">"
				"							<check_vel_continuous/>"
				"							<not_check_vel_continuous/>"
				"						</unique>"
				"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel_continuous_at_start\">"
				"							<check_vel_continuous_at_start/>"
				"							<not_check_vel_continuous_at_start/>"
				"						</unique>"
				"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel_following_error\">"
				"							<check_vel_following_error/>"
				"							<not_check_vel_following_error />"
				"						</unique>"
				"					</group>"
				"				</unique>"
				"			</group>"
				"		</unique>"
				"	</group>"
				"</moveTTT>");
		}
	};

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
	class MoveJM : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
		{
			auto c = dynamic_cast<aris::control::Controller*>(target.master);
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

			target.option |=
				Plan::USE_VEL_OFFSET |
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

		}
		auto virtual executeRT(PlanTarget &target)->int
		{
			//获取驱动//
			auto controller = dynamic_cast<aris::control::Controller*>(target.master);
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
				
			if (!target.model->solverPool().at(1).kinPos())return -1;

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
		auto virtual collectNrt(PlanTarget &target)->void {}

		explicit MoveJM(const std::string &name = "MoveJM_plan") :Plan(name)
		{
			command().loadXmlStr(
				"<moveJM>"
				"	<group type=\"GroupParam\" default_child_type=\"Param\">"
				"		<pos default=\"current_pos\"/>"
				"		<vel default=\"{0.2,0.2,0.2,0.2,0.2,0.2}\" abbreviation=\"v\"/>"
				"		<acc default=\"{0.1,0.1,0.1,0.1,0.1,0.1}\" abbreviation=\"a\"/>"
				"		<dec default=\"{0.1,0.1,0.1,0.1,0.1,0.1}\" abbreviation=\"d\"/>"
				"		<ab default=\"1\"/>"
				"		<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_all\">"
				"			<check_all/>"
				"			<check_none/>"
				"			<group type=\"GroupParam\" default_child_type=\"Param\">"
				"				<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos\">"
				"					<check_pos/>"
				"					<not_check_pos/>"
				"					<group type=\"GroupParam\" default_child_type=\"Param\">"
				"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_max\">"
				"							<check_pos_max/>"
				"							<not_check_pos_max/>"
				"						</unique>"
				"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_min\">"
				"							<check_pos_min/>"
				"							<not_check_pos_min/>"
				"						</unique>"
				"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_continuous\">"
				"							<check_pos_continuous/>"
				"							<not_check_pos_continuous/>"
				"						</unique>"
				"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_continuous_at_start\">"
				"							<check_pos_continuous_at_start/>"
				"							<not_check_pos_continuous_at_start/>"
				"						</unique>"
				"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_continuous_second_order\">"
				"							<check_pos_continuous_second_order/>"
				"							<not_check_pos_continuous_second_order/>"
				"						</unique>"
				"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_continuous_second_order_at_start\">"
				"							<check_pos_continuous_second_order_at_start/>"
				"							<not_check_pos_continuous_second_order_at_start/>"
				"						</unique>"
				"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_following_error\">"
				"							<check_pos_following_error/>"
				"							<not_check_pos_following_error />"
				"						</unique>"
				"					</group>"
				"				</unique>"
				"				<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel\">"
				"					<check_vel/>"
				"					<not_check_vel/>"
				"					<group type=\"GroupParam\" default_child_type=\"Param\">"
				"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel_max\">"
				"							<check_vel_max/>"
				"							<not_check_vel_max/>"
				"						</unique>"
				"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel_min\">"
				"							<check_vel_min/>"
				"							<not_check_vel_min/>"
				"						</unique>"
				"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel_continuous\">"
				"							<check_vel_continuous/>"
				"							<not_check_vel_continuous/>"
				"						</unique>"
				"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel_continuous_at_start\">"
				"							<check_vel_continuous_at_start/>"
				"							<not_check_vel_continuous_at_start/>"
				"						</unique>"
				"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel_following_error\">"
				"							<check_vel_following_error/>"
				"							<not_check_vel_following_error />"
				"						</unique>"
				"					</group>"
				"				</unique>"
				"			</group>"
				"		</unique>"
				"	</group>"
				"</moveJM>");
		}
	};

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
	class MoveJI : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
		{
			auto c = dynamic_cast<aris::control::Controller*>(target.master);
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

			target.option |=
				Plan::USE_VEL_OFFSET |
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

		}
		auto virtual executeRT(PlanTarget &target)->int
		{
			//获取驱动//
			auto controller = dynamic_cast<aris::control::Controller*>(target.master);
			auto &param = std::any_cast<MoveJIParam&>(target.param);
			static double begin_pos[6];
			static double pos[6];
			// 取得起始位置 //
			if (target.count == 1)
			{
				target.model->generalMotionPool().at(0).setMpq(param.pq.data());	//generalMotionPool()指模型末端，at(0)表示第1个末端，对于6足就有6个末端，对于机器人只有1个末端
				if (!target.model->solverPool().at(0).kinPos())return -1;
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
			if (!target.model->solverPool().at(1).kinPos())return -1;

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
		auto virtual collectNrt(PlanTarget &target)->void {}

		explicit MoveJI(const std::string &name = "MoveJI_plan") :Plan(name)
		{
			command().loadXmlStr(
				"<moveJI>"
				"	<group type=\"GroupParam\" default_child_type=\"Param\">"
				"		<pq default=\"current_pos\"/>"
				"		<vel default=\"{0.2,0.2,0.2,0.2,0.2,0.2}\" abbreviation=\"v\"/>"
				"		<acc default=\"{0.1,0.1,0.1,0.1,0.1,0.1}\" abbreviation=\"a\"/>"
				"		<dec default=\"{0.1,0.1,0.1,0.1,0.1,0.1}\" abbreviation=\"d\"/>"
				"		<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_all\">"
				"			<check_all/>"
				"			<check_none/>"
				"			<group type=\"GroupParam\" default_child_type=\"Param\">"
				"				<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos\">"
				"					<check_pos/>"
				"					<not_check_pos/>"
				"					<group type=\"GroupParam\" default_child_type=\"Param\">"
				"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_max\">"
				"							<check_pos_max/>"
				"							<not_check_pos_max/>"
				"						</unique>"
				"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_min\">"
				"							<check_pos_min/>"
				"							<not_check_pos_min/>"
				"						</unique>"
				"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_continuous\">"
				"							<check_pos_continuous/>"
				"							<not_check_pos_continuous/>"
				"						</unique>"
				"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_continuous_at_start\">"
				"							<check_pos_continuous_at_start/>"
				"							<not_check_pos_continuous_at_start/>"
				"						</unique>"
				"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_continuous_second_order\">"
				"							<check_pos_continuous_second_order/>"
				"							<not_check_pos_continuous_second_order/>"
				"						</unique>"
				"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_continuous_second_order_at_start\">"
				"							<check_pos_continuous_second_order_at_start/>"
				"							<not_check_pos_continuous_second_order_at_start/>"
				"						</unique>"
				"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_following_error\">"
				"							<check_pos_following_error/>"
				"							<not_check_pos_following_error />"
				"						</unique>"
				"					</group>"
				"				</unique>"
				"				<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel\">"
				"					<check_vel/>"
				"					<not_check_vel/>"
				"					<group type=\"GroupParam\" default_child_type=\"Param\">"
				"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel_max\">"
				"							<check_vel_max/>"
				"							<not_check_vel_max/>"
				"						</unique>"
				"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel_min\">"
				"							<check_vel_min/>"
				"							<not_check_vel_min/>"
				"						</unique>"
				"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel_continuous\">"
				"							<check_vel_continuous/>"
				"							<not_check_vel_continuous/>"
				"						</unique>"
				"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel_continuous_at_start\">"
				"							<check_vel_continuous_at_start/>"
				"							<not_check_vel_continuous_at_start/>"
				"						</unique>"
				"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel_following_error\">"
				"							<check_vel_following_error/>"
				"							<not_check_vel_following_error />"
				"						</unique>"
				"					</group>"
				"				</unique>"
				"			</group>"
				"		</unique>"
				"	</group>"
				"</moveJI>");
		}
	};
	
	// 夹爪控制 //
	struct GraspParam
	{
		bool status;
	};
	class Grasp : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
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

		auto virtual executeRT(PlanTarget &target)->int
		{
			auto &param = std::any_cast<GraspParam&>(target.param);
			// 访问主站 //
			auto controller = dynamic_cast<aris::control::EthercatController*>(target.master);
			static std::uint8_t dq = 0x01;
			if (param.status)
			{
				dq = 0x01;
				controller->ecSlavePool().at(7).writePdo(0x7001, 0x01, dq);
			}
			else
			{
				dq = 0x00;
				controller->ecSlavePool().at(7).writePdo(0x7001, 0x01, dq);
			}	
			//std::cout << int(a) << std::endl;
			return 0;
		}
		auto virtual collectNrt(PlanTarget &target)->void {}

		explicit Grasp(const std::string &name = "Grasp_plan") :Plan(name)
		{
			command().loadXmlStr(
				"<grasp>"
				"	<group_switch type=\"GroupParam\" default_child_type=\"Param\">"
				"		<status default=\"1\"/>"
				"	</group_switch>"
				"</grasp>");
		}
	};

	// 监听DI信号 //
	class ListenDI : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void 
		{
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
		auto virtual executeRT(PlanTarget &target)->int
		{
			if (is_automatic)
			{
				// 访问主站 //
				auto controller = dynamic_cast<aris::control::EthercatController*>(target.master);
				static std::uint8_t di = 0x00;
				static std::int16_t di_delay[6] = { 0,0,0,0,0,0 };
				controller->ecSlavePool().at(7).readPdo(0x6001, 0x01, di);
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
		explicit ListenDI(const std::string &name = "ListenDI_plan") :Plan(name)
		{
			command().loadXmlStr(
				"<listenDI>"
				"</listenDI>");
		}
	};

	// 电缸余弦往复轨迹 //
	struct MoveEAParam
	{
		double s;
		double time;
	};
	class MoveEA : public aris::plan::Plan
	{
	public:
		//平均值速度滤波、摩擦力滤波器初始化//
		std::vector<double> fore_vel;
		IIR_FILTER::IIR iir;
		double tempforce;

		auto virtual prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
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

			target.option |=
				Plan::USE_TARGET_POS |
#ifdef WIN32
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
				Plan::NOT_CHECK_VEL_MIN |
				Plan::NOT_CHECK_VEL_MAX |
				Plan::NOT_CHECK_VEL_CONTINUOUS |
				Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START |
				Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;

		}
		auto virtual executeRT(PlanTarget &target)->int
		{
			auto &param = std::any_cast<MoveEAParam&>(target.param);

			auto time = static_cast<int>(param.time * 1000);
			static double begin_p;

			// 访问主站 //
			auto controller = dynamic_cast<aris::control::Controller*>(target.master);

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
		auto virtual collectNrt(PlanTarget &target)->void {}

		explicit MoveEA(const std::string &name = "MoveEA_plan"):Plan(name), fore_vel(FORE_VEL_LENGTH + 1), tempforce(0)
		{
			command().loadXmlStr(
				"<moveEA>"
				"	<group type=\"GroupParam\" default_child_type=\"Param\">"
				"		<s default=\"0.1\"/>"
				"		<time default=\"1.0\" abbreviation=\"t\"/>"
				"	</group>"
				"</moveEA>");

			std::vector<double> num_data(IIR_FILTER::num, IIR_FILTER::num + 20);
			std::vector<double> den_data(IIR_FILTER::den, IIR_FILTER::den + 20);
			iir.setPara(num_data, den_data);

		}
	};

	// 电缸运动轨迹；速度前馈 //
	struct MoveEAPParam
	{
		double begin_pos, pos, vel, acc, dec;
		bool ab;
	};
	class MoveEAP : public aris::plan::Plan
	{
	public:
		//平均值速度滤波、摩擦力滤波器初始化//
		std::vector<double> fore_vel;
		IIR_FILTER::IIR iir;
		double tempforce;
		
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
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

			target.option |=
				Plan::USE_TARGET_POS |
				Plan::USE_VEL_OFFSET |
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

		}
		auto virtual executeRT(PlanTarget &target)->int
		{
			auto &param = std::any_cast<MoveEAPParam&>(target.param);
			// 访问主站 //
			auto controller = dynamic_cast<aris::control::Controller*>(target.master);

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
		auto virtual collectNrt(PlanTarget &target)->void {}

		explicit MoveEAP(const std::string &name = "MoveEAP_plan") :Plan(name), fore_vel(FORE_VEL_LENGTH + 1), tempforce(0)
		{
			command().loadXmlStr(
				"<moveEAP>"
				"	<group type=\"GroupParam\" default_child_type=\"Param\">"
				"		<begin_pos default=\"0.1\" abbreviation=\"b\"/>"
				"		<pos default=\"0.1\"/>"
				"		<vel default=\"0.02\"/>"
				"		<acc default=\"0.3\"/>"
				"		<dec default=\"-0.3\"/>"
				"		<ab default=\"0\"/>"
				"	</group>"
				"</moveEAP>");

			std::vector<double> num_data(IIR_FILTER::num, IIR_FILTER::num + 20);
			std::vector<double> den_data(IIR_FILTER::den, IIR_FILTER::den + 20);
			iir.setPara(num_data, den_data);
			

		}
	};

	auto createPlanRootRokaeXB4()->std::unique_ptr<aris::plan::PlanRoot>
	{
		std::unique_ptr<aris::plan::PlanRoot> plan_root(new aris::plan::PlanRoot);

		plan_root->planPool().add<aris::plan::EnablePlan>();
		plan_root->planPool().add<aris::plan::DisablePlan>();
		plan_root->planPool().add<aris::plan::ModePlan>();
		plan_root->planPool().add<aris::plan::RecoverPlan>();
		plan_root->planPool().add<aris::plan::SleepPlan>();
		auto &rs = plan_root->planPool().add<aris::plan::ResetPlan>();
		rs.command().findByName("group")->findByName("pos")->loadXmlStr("<pos default=\"{0.5,0.392523364485981,0.789915966386555,0.5,0.5,0.5}\" abbreviation=\"p\"/>");

		plan_root->planPool().add<aris::plan::MovePlan>();
		plan_root->planPool().add<aris::plan::MoveJ>();
		plan_root->planPool().add<aris::plan::Show>();
		plan_root->planPool().add<kaanh::MoveInit>();
		plan_root->planPool().add<MoveX>();
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
		plan_root->planPool().add<forcecontrol::MoveStop>();
		plan_root->planPool().add<forcecontrol::MoveSPQ>();
		plan_root->planPool().add<kaanh::MoveJM>();
		plan_root->planPool().add<kaanh::MoveJI>();
		plan_root->planPool().add<kaanh::Grasp>();
		plan_root->planPool().add<kaanh::ListenDI>();
		plan_root->planPool().add<kaanh::MoveEA>();
		plan_root->planPool().add<kaanh::MoveEAP>();

	/*	auto &dm1 = plan_root->planPool().add<aris::plan::MoveJ>();
		dm1.command().findByName("group")->findByName("unique_pos")->findByName("pq")->loadXmlStr("<pq default=\"{0.444,-0,0.562,0.642890516,0.000011540,0.765958083,-0.000008196}\"/>");
		dm1.command().findByName("group")->findByName("unique_pos")->findByName("pq")->loadXmlStr("<pq default=\"{0.406,0.334,0.032,-0.018301280,-0.327458252,0.944444310,-0.021473281}\"/>");
		dm1.command().findByName("group")->findByName("unique_pos")->findByName("pq")->loadXmlStr("<pq default=\"{0.406,0.334,0.398,-0.018332796,-0.327460720,0.944442425,-0.021491655}\"/>");
		dm1.command().findByName("group")->findByName("unique_pos")->findByName("pq")->loadXmlStr("<pq default=\"{0.406,-0.344,0.390,-0.025825354,-0.327485510,0.944191478,-0.024264042}\"/>");
		dm1.command().findByName("group")->findByName("unique_pos")->findByName("pq")->loadXmlStr("<pq default=\"{0.406,-0.344,0.085,-0.025828364,-0.327501842,0.944186337,-0.024240465}\"/>");
		dm1.command().findByName("group")->findByName("unique_pos")->findByName("pq")->loadXmlStr("<pq default=\"{0.406,-0.344,0.272,-0.025848482,-0.327498467,0.944187605,-0.024215228}\"/>");
		dm1.command().findByName("group")->findByName("unique_pos")->findByName("pq")->loadXmlStr("<pq default=\"{0.444,-0,0.562,0.642890516,0.000011540,0.765958083,-0.000008196}\"/>");*/
		return plan_root;
	}
}
