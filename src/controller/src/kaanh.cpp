#include <algorithm>
#include"kaanh.h"

using namespace aris::dynamic;
using namespace aris::plan;

namespace kaanh
{
    //configuring controller
	auto createControllerRokaeXB4()->std::unique_ptr<aris::control::Controller>
	{
		std::unique_ptr<aris::control::Controller> controller(new aris::control::EthercatController);

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
				0.00293480352126769,   -2.50023777179214,   -0.292382537944081,   0.0582675097338009,   1.53363576057128,   26.3545454214145
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
				-170.0 / 360 * 2 * PI, -84.0 / 360 * 2 * PI, -188.0 / 360 * 2 * PI, -170.0 / 360 * 2 * PI, -117.0 / 360 * 2 * PI, -360.0 / 360 * 2 * PI
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
				"<EthercatMotion phy_id=\"" + std::to_string(i) + "\" product_code=\"0x00\""
                " vendor_id=\"0x2e1\" revision_num=\"0x00029001\" dc_assign_activate=\"0x0300\""
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
				"				<PdoEntry name=\"cur_actual_value\" index=\"0x6078\" subindex=\"0x00\" size=\"16\"/>"
				"			</Pdo>"
				"		</SyncManager>"
				"	</SyncManagerPoolObject>"
				"</EthercatMotion>";

			controller->slavePool().add<aris::control::EthercatMotion>().loadXmlStr(xml_str);

#ifndef WIN32
			dynamic_cast<aris::control::EthercatMotion&>(controller->slavePool().back()).scanInfoForCurrentSlave();
#endif
	}
		return controller;
};
    //set DH parameters
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
				pq_pe_j_vec = std::make_any<std::vector<double> >(3 * target.model->motionPool().size());
				target.server->getRtData([](aris::server::ControlServer& cs, std::any& data)
				{
					for (aris::Size i = 0; i < cs.model().motionPool().size(); i++)
					{
						std::any_cast<std::vector<double>&>(data)[i] = cs.model().motionPool()[i].mp();
					}
					for (aris::Size i = cs.model().motionPool().size(); i < 2 * cs.model().motionPool().size(); i++)
					{
						std::any_cast<std::vector<double>&>(data)[i] = cs.model().motionPool()[i - cs.model().motionPool().size()].mv();
					}
					for (aris::Size i = 2 * cs.model().motionPool().size(); i < 3 * cs.model().motionPool().size(); i++)
					{
						std::any_cast<std::vector<double>&>(data)[i] = cs.model().motionPool()[i - 2 * cs.model().motionPool().size()].ma();
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
			pq_pe_j[i] = std::floor(pq_pe_j[i] * 1000.0f + 0.5) / 1000.0f;
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


	struct MoveJRParam
	{
		double begin_pos, target_pos, vel, acc, dec;
		std::vector<bool> joint_active_vec;
	};
	auto MoveJR::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		auto c = target.controller;
		MoveJRParam param;

        param.joint_active_vec.clear();

		for (auto cmd_param : params)
		{
			if (cmd_param.first == "motion_id")
			{
				param.joint_active_vec.resize(target.model->motionPool().size(), false);
				param.joint_active_vec.at(std::stoi(cmd_param.second)) = true;
			}
			else if (cmd_param.first == "pos")
			{
				param.target_pos = std::stod(cmd_param.second);
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

		target.param = param;

        //std::fill(target.mot_options.begin(), target.mot_options.end(), Plan::USE_TARGET_POS);
	}
	auto MoveJR::executeRT(PlanTarget &target)->int
	{
		auto &param = std::any_cast<MoveJRParam&>(target.param);
		auto controller = target.controller;
		

		//获取第一个count时，电机的当前角度位置//
		if (target.count == 1)
		{
			for (Size i = 0; i < param.joint_active_vec.size(); ++i)
			{
				if (param.joint_active_vec[i])
				{
                    param.begin_pos = controller->motionAtAbs(i).actualPos();
				}
			}
		}

		//梯形轨迹//
		aris::Size total_count{ 1 };
		for (Size i = 0; i < param.joint_active_vec.size(); ++i)
		{
			if (param.joint_active_vec[i])
			{
				double p, v, a;
				aris::Size t_count;
				aris::plan::moveAbsolute(target.count, param.begin_pos, param.begin_pos + param.target_pos, param.vel / 1000, param.acc / 1000 / 1000, param.dec / 1000 / 1000, p, v, a, t_count);
				controller->motionAtAbs(i).setTargetPos(p);
				target.model->motionPool().at(i).setMp(p);
				total_count = std::max(total_count, t_count);
			}
		}
		//3D模型同步
        if (target.model->solverPool().at(1).kinPos())return -1;

		// 打印 //
		auto &cout = controller->mout();
		if (target.count % 100 == 0)
		{
			cout << "target_pos" << ":" << param.target_pos << " ";
			cout << "vel" << ":" << param.vel << " ";
			cout << "acc" << ":" << param.acc << " ";
			cout << "dec"  << ":" << param.dec << " ";
			
			for (Size i = 0; i < param.joint_active_vec.size(); ++i)
			{
				if (param.joint_active_vec[i])
				{
					cout << "actualPos" << ":" << controller->motionAtAbs(i).actualPos() << " ";
                    cout << "actualVel" << ":" << controller->motionAtAbs(i).actualVel() << " ";
                    cout << "actualCur" << ":" << controller->motionAtAbs(i).actualCur() << " ";
				}
			}

			cout << std::endl;
		}

		// log //
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
	auto MoveJR::collectNrt(PlanTarget &target)->void {}
	MoveJR::MoveJR(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"moveJR\">"
			"	<GroupParam>"
			"		<Param name=\"motion_id\" abbreviation=\"m\" default=\"0\"/>"
			"		<Param name=\"pos\" default=\"0\"/>"
			"		<Param name=\"vel\" abbreviation=\"v\" default=\"0.5\"/>"
			"		<Param name=\"acc\" default=\"1\"/>"
			"		<Param name=\"dec\" default=\"1\"/>"
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
				imp_->s2_nrt.cor_system = 0;
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
			max_vel[i] = imp_->vel[i] * 1.0*imp_->s1_rt.vel_percent / 100.0;
			target_p[i] += aris::dynamic::s_sgn(increase_status[i])*max_vel[i] * 1e-3;
			increase_status[i] -= aris::dynamic::s_sgn(increase_status[i]);
		}
		//std::copy_n(target_pos, 6, imp_->pe_start);

		// 梯形轨迹规划 calculate real value //
		double p_next[6]{ 0,0,0,0,0,0 }, v_next[6]{ 0,0,0,0,0,0 }, a_next[6]{ 0,0,0,0,0,0 };
		for (int i = 0; i < 6; i++)
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
		double pq[7]{ p_next[0] - p_now[0], p_next[1] - p_now[1], p_next[2] - p_now[2], w[0] * sin(theta / 2.0), w[1] * sin(theta / 2.0), w[2] * sin(theta / 2.0), cos(theta / 2.0) };
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


	auto createPlanRootRokaeXB4()->std::unique_ptr<aris::plan::PlanRoot>
	{
        std::unique_ptr<aris::plan::PlanRoot> plan_root(new aris::plan::PlanRoot);

		plan_root->planPool().add<aris::plan::Enable>();
		plan_root->planPool().add<aris::plan::Disable>();
		plan_root->planPool().add<aris::plan::Mode>();
		plan_root->planPool().add<aris::plan::Recover>();
		auto &rs = plan_root->planPool().add<aris::plan::Reset>();
        rs.command().findParam("pos")->setDefaultValue("{0.5,0.39252,0.7899,0.5,0.5,0.5}");

		plan_root->planPool().add<aris::plan::MoveL>();
        plan_root->planPool().add<aris::plan::GetXml>();
        plan_root->planPool().add<aris::plan::GetPartPq>();
        plan_root->planPool().add<aris::plan::SetXml>();
		plan_root->planPool().add<aris::plan::MoveJ>();
		plan_root->planPool().add<aris::plan::Show>();
		plan_root->planPool().add<kaanh::Get>();
		plan_root->planPool().add<kaanh::MoveJR>();
		plan_root->planPool().add<kaanh::JogC>();
		plan_root->planPool().add<kaanh::JogJ>();

		return plan_root;
	}
	
}
