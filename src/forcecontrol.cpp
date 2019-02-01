﻿#include"forcecontrol.h"
#include <atomic>


using namespace aris::dynamic;
using namespace aris::plan;

namespace forcecontrol
{
	// 力控拖动——单关节或者6个轨迹相对运动轨迹--输入单个关节，角度位置；关节按照梯形速度轨迹执行；速度前馈；电流控制 //
	struct MoveJRCParam
	{
		double kp_p, kp_v, ki_v;
		double vel, acc, dec;
		std::vector<double> joint_pos_vec, begin_joint_pos_vec;
		std::vector<bool> joint_active_vec;
	};
	static std::atomic_bool enable_moveJRC = true;
	auto MoveJRC::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		auto c = dynamic_cast<aris::control::Controller*>(target.master);
		MoveJRCParam param;
		enable_moveJRC = true;
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
			else if (cmd_param.first == "kp_p")
			{
				param.kp_p = std::stod(cmd_param.second);
			}
			else if (cmd_param.first == "kp_v")
			{
				param.kp_v = std::stod(cmd_param.second);
			}
			else if (cmd_param.first == "ki_v")
			{
				param.ki_v = std::stod(cmd_param.second);
			}
		}

		param.begin_joint_pos_vec.resize(target.model->motionPool().size());

		target.param = param;

		target.option |=
			Plan::USE_TARGET_POS |
			Plan::USE_VEL_OFFSET |
			//#ifdef WIN32
			Plan::NOT_CHECK_POS_MIN |
			Plan::NOT_CHECK_POS_MAX |
			Plan::NOT_CHECK_POS_CONTINUOUS |
			Plan::NOT_CHECK_POS_CONTINUOUS_AT_START |
			Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
			Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START |
			Plan::NOT_CHECK_POS_FOLLOWING_ERROR |
			//#endif
			Plan::NOT_CHECK_VEL_MIN |
			Plan::NOT_CHECK_VEL_MAX |
			Plan::NOT_CHECK_VEL_CONTINUOUS |
			Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START |
			Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;

	}
	auto MoveJRC::executeRT(PlanTarget &target)->int
	{
		auto &param = std::any_cast<MoveJRCParam&>(target.param);
		auto controller = dynamic_cast<aris::control::Controller *>(target.master);
		static bool is_running{ true };
		static double vinteg[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
		double pqa[7] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
		bool ds_is_all_finished{ true };
		bool md_is_all_finished{ true };

		//第一个周期，将目标电机的控制模式切换到电流控制模式
		if (target.count == 1)
		{

			is_running = true;
			for (Size i = 0; i < param.joint_active_vec.size(); ++i)
			{
				if (param.joint_active_vec[i])
				{
					param.begin_joint_pos_vec[i] = target.model->motionPool()[i].mp();
					controller->motionPool().at(i).setModeOfOperation(10);	//切换到电流控制
				}
			}
		}

		//最后一个周期将目标电机去使能
		if (!enable_moveJRC)
		{
			is_running = false;
		}
		if (!is_running)
		{
			for (Size i = 0; i < param.joint_active_vec.size(); ++i)
			{
				if (param.joint_active_vec[i])
				{
					//controller->motionPool().at(i).setModeOfOperation(8);
					//controller->motionPool().at(i).setTargetPos(controller->motionAtAbs(i).actualPos());
					//target.model->motionPool().at(i).setMp(controller->motionAtAbs(i).actualPos());
					auto ret = controller->motionPool().at(i).disable();
					if (ret)
					{
						ds_is_all_finished = false;
					}
				}
			}
		}

		//将目标电机由电流模式切换到位置模式
		if (!is_running&&ds_is_all_finished)
		{
			for (Size i = 0; i < param.joint_active_vec.size(); ++i)
			{
				if (param.joint_active_vec[i])
				{
					controller->motionPool().at(i).setModeOfOperation(8);
					auto ret = controller->motionPool().at(i).mode(8);
					if (ret)
					{
						md_is_all_finished = false;
					}
				}
			}
		}

		//动力学
		for (int i = 0; i < 6; ++i)
		{
			target.model->motionPool()[i].setMp(controller->motionPool()[i].actualPos());
			target.model->motionPool().at(i).setMv(controller->motionAtAbs(i).actualVel());
			target.model->motionPool().at(i).setMa(0.0);
		}

		target.model->solverPool()[1].kinPos();
		target.model->solverPool()[1].kinVel();
		target.model->solverPool()[2].dynAccAndFce();

		if (is_running)
		{
			for (Size i = 0; i < param.joint_active_vec.size(); ++i)
			{
				if (param.joint_active_vec[i])
				{
					double p, v, pa, vt, va, voff, ft, foff, ft_offset;
					p = controller->motionAtAbs(i).actualPos();
					v = 0.0;
					pa = controller->motionAtAbs(i).actualPos();
					va = controller->motionAtAbs(i).actualVel();
					voff = v * 1000;
					foff = 0.0;
					vt = param.kp_p*(p - pa) + voff;
					//限制速度的范围在-1.0~1.0之间
					vt = std::max(-1.0, vt);
					vt = std::min(1.0, vt);

					vinteg[i] = vinteg[i] + vt - va;
					ft = param.kp_v*(vt - va) + param.ki_v*vinteg[i] + foff;
					//限制电流的范围在-400~400(千分数：额定电流是1000)之间
					ft = std::max(-400.0, ft);
					ft = std::min(400.0, ft);

					//拖动示教
					//constexpr double f_static[6] = { 9.349947583,11.64080253,4.770140543,3.631416685,2.58310847,1.783739862 };
					//constexpr double f_vel[6] = { 7.80825641,13.26518528,7.856443575,3.354615249,1.419632126,0.319206404 };
					//constexpr double f_acc[6] = { 0,3.555679326,0.344454603,0.148247716,0.048552673,0.033815455 };
					//constexpr double f2c_index[6] = { 9.07327526291993, 9.07327526291993, 17.5690184835913, 39.0310903520972, 66.3992503259041, 107.566785527965 };
					//constexpr double max_static_vel[6] = { 0.1, 0.1, 0.1, 0.05, 0.05, 0.075 };
					//constexpr double f_static_index[6] = { 0.5, 0.5, 0.5, 0.85, 0.95, 0.8 };

					auto real_vel = std::max(std::min(max_static_vel[i], controller->motionAtAbs(i).actualVel()), -max_static_vel[i]);
					ft_offset = (f_vel[i] * controller->motionAtAbs(i).actualVel() + f_static_index[i] * f_static[i] * real_vel / max_static_vel[i])*f2c_index[i];

					ft_offset = std::max(-500.0, ft_offset);
					ft_offset = std::min(500.0, ft_offset);

					controller->motionAtAbs(i).setTargetCur(ft_offset + target.model->motionPool()[i].mfDyn()*f2c_index[i]);

					//打印PID控制结果
					/*
					auto &cout = controller->mout();
					if (target.count % 100 == 0)
					{
						//cout << "ft:" << ft << "  " << "vt:" << vt << "  " << "va:" << va << "  " << "param.kp_v*(vt - va):" << param.kp_v*(vt - va) << "  " << "param.ki_v*vinteg[i]:" << param.ki_v*vinteg[i] << "    ";
						cout << "feedbackf:" << std::setw(10) << controller->motionAtAbs(i).actualCur()
							 << "f:" << std::setw(10) << ft_offset
							 << "p:" << std::setw(10) << p
							 << "pa:" << std::setw(10) << pa
							 << "va:" << std::setw(10) << va << std::endl;
					}
					*/
				}
			}
		}

		if (!target.model->solverPool().at(1).kinPos())return -1;
		target.model->generalMotionPool().at(0).getMpq(pqa);

		// 打印电流 //
		auto &cout = controller->mout();
		if (target.count % 100 == 0)
		{
			for (Size i = 0; i < param.joint_active_vec.size(); i++)
			{
				if (param.joint_active_vec[i])
				{
					cout << "pos" << i + 1 << ":" << std::setw(6) << controller->motionAtAbs(i).actualPos() << "  ";
					cout << "vel" << i + 1 << ":" << std::setw(6) << controller->motionAtAbs(i).actualVel() << "  ";
					cout << "cur" << i + 1 << ":" << std::setw(6) << controller->motionAtAbs(i).actualCur() << "  ";
				}
			}
			cout << "pq: ";
			for (Size i = 0; i < 7; i++)
			{
				cout << std::setw(6) << pqa[i] << " ";
			}
			cout << std::endl;
		}

		// log 位置、速度、电流 //
		auto &lout = controller->lout();
		for (Size i = 0; i < param.joint_active_vec.size(); i++)
		{
            //lout << std::setw(10) << controller->motionAtAbs(i).targetCur() << ",";
            //lout << std::setw(10) << controller->motionAtAbs(i).actualPos() << ",";
            //lout << std::setw(10) << controller->motionAtAbs(i).actualVel() << ",";
            //lout << std::setw(10) << controller->motionAtAbs(i).actualCur() << " | ";

            lout << controller->motionAtAbs(i).actualPos() << " ";
            lout << controller->motionAtAbs(i).actualVel() << " ";
            lout << controller->motionAtAbs(i).actualCur() << " ";	
		}
		
		// log 末端pq值 //
		for (Size i = 0; i < 7; i++)
		{
			lout << pqa[i] << " ";
		}
		lout << std::endl;

		return (!is_running&&ds_is_all_finished&&md_is_all_finished) ? 0 : 1;
	}
	auto MoveJRC::collectNrt(PlanTarget &target)->void {}
	MoveJRC::MoveJRC(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<moveJRC default_child_type=\"Param\">"
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
			"		<kp_p default=\"1\"/>"
			"		<kp_v default=\"100\"/>"
			"		<ki_v default=\"0.1\"/>"
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
			"</moveJRC>");
	}


	// 力控末端空间——输入末端pq姿态；先末端PID算法，再末端力反解到轴空间，然后控制每个电机——优点是能够控制末端力；速度前馈；电流控制 //
	struct MovePQCrashParam
	{
		std::vector<double> kp_p;
		std::vector<double> kp_v;
		std::vector<double> ki_v;

		std::vector<double> pqt;
		std::vector<double> pqa;
		std::vector<double> vt;
		std::vector<double> va;

		std::vector<double> ft;
		std::vector<double> ft_pid;
	};
	static std::atomic_bool enable_movePQCrash = true;
	auto MovePQCrash::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
		{
			auto c = dynamic_cast<aris::control::Controller*>(target.master);
			MovePQCrashParam param;
			enable_movePQCrash = true;
			param.kp_p.resize(7, 0.0);
			param.kp_v.resize(6, 0.0);
			param.ki_v.resize(6, 0.0);

			param.pqt.resize(7, 0.0);
			param.pqa.resize(7, 0.0);
			param.vt.resize(7, 0.0);
			param.va.resize(7, 0.0);
			param.ft.resize(6, 0.0);
			param.ft_pid.resize(6, 0.0);
			//params.at("pqt")
			for (auto &p : params)
			{
				if (p.first == "pqt")
				{
					auto pqarray = target.model->calculator().calculateExpression(p.second);
					param.pqt.assign(pqarray.begin(), pqarray.end());
				}
				else if (p.first == "kp_p")
				{
					auto v = target.model->calculator().calculateExpression(p.second);
					if (v.size() == 1)
					{
						std::fill(param.kp_p.begin(), param.kp_p.end(), v.toDouble());
					}
					else if (v.size() == param.kp_p.size())
					{
						param.kp_p.assign(v.begin(), v.end());
					}
					else
					{
						throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
					}

					for (Size i = 0; i < param.kp_p.size(); ++i)
					{
						if (param.kp_p[i] > 10000.0)
						{
							param.kp_p[i] = 10000.0;
						}
						if (param.kp_p[i] < 0.01)
						{
							param.kp_p[i] = 0.01;
						}
					}
				}
				else if (p.first == "kp_v")
				{
					auto a = target.model->calculator().calculateExpression(p.second);
					if (a.size() == 1)
					{
						std::fill(param.kp_v.begin(), param.kp_v.end(), a.toDouble());
					}
					else if (a.size() == param.kp_v.size())
					{
						param.kp_v.assign(a.begin(), a.end());
					}
					else
					{
						throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
					}

					for (Size i = 0; i < param.kp_v.size(); ++i)
					{
						if (param.kp_v[i] > 10000.0)
						{
							param.kp_v[i] = 10000.0;
						}
						if (param.kp_v[i] < 0.01)
						{
							param.kp_v[i] = 0.01;
						}
					}
				}
				else if (p.first == "ki_v")
				{
					auto d = target.model->calculator().calculateExpression(p.second);
					if (d.size() == 1)
					{
						std::fill(param.ki_v.begin(), param.ki_v.end(), d.toDouble());
					}
					else if (d.size() == param.ki_v.size())
					{
						param.ki_v.assign(d.begin(), d.end());
					}
					else
					{
						throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
					}

					for (Size i = 0; i < param.ki_v.size(); ++i)
					{
						if (param.ki_v[i] > 10000.0)
						{
							param.ki_v[i] = 10000.0;
						}
						if (param.ki_v[i] < 0.001)
						{
							param.ki_v[i] = 0.001;
						}
					}
				}

			}
			target.param = param;

			target.option |=
				Plan::USE_VEL_OFFSET |
				//#ifdef WIN32
				Plan::NOT_CHECK_POS_MIN |
				Plan::NOT_CHECK_POS_MAX |
				Plan::NOT_CHECK_POS_CONTINUOUS |
				Plan::NOT_CHECK_POS_CONTINUOUS_AT_START |
				Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
				Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START |
				Plan::NOT_CHECK_POS_FOLLOWING_ERROR |
				//#endif
				Plan::NOT_CHECK_VEL_MIN |
				Plan::NOT_CHECK_VEL_MAX |
				Plan::NOT_CHECK_VEL_CONTINUOUS |
				Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START |
				Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;

		}
	auto MovePQCrash::executeRT(PlanTarget &target)->int
		{
			auto &param = std::any_cast<MovePQCrashParam&>(target.param);
			auto controller = dynamic_cast<aris::control::Controller *>(target.master);
			static bool is_running{ true };
			static double vinteg[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
			static double vproportion[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
			bool ds_is_all_finished{ true };
			bool md_is_all_finished{ true };

			//第一个周期，将目标电机的控制模式切换到电流控制模式
			if (target.count == 1)
			{
				is_running = true;

				for (Size i = 0; i < param.ft.size(); ++i)
				{
					controller->motionPool().at(i).setModeOfOperation(10);	//切换到电流控制
				}
			}

			//最后一个周期将目标电机去使能
			if (!enable_movePQCrash)
			{
				is_running = false;
			}
			if (!is_running)
			{
				for (Size i = 0; i < param.ft.size(); ++i)
				{
					auto ret = controller->motionPool().at(i).disable();
					if (ret)
					{
						ds_is_all_finished = false;
					}
				}
			}

			//将目标电机由电流模式切换到位置模式
			if (!is_running&&ds_is_all_finished)
			{
				for (Size i = 0; i < param.ft.size(); ++i)
				{
					controller->motionPool().at(i).setModeOfOperation(8);
					//auto ret = controller->motionPool().at(i).modeOfDisplay();
					auto ret = controller->motionPool().at(i).mode(8);
					if (ret)
					{
						md_is_all_finished = false;
					}
				}
			}

			//运动学、动力学正解
			for (int i = 0; i < param.ft.size(); ++i)
			{
				target.model->motionPool()[i].setMp(controller->motionPool()[i].actualPos());
				target.model->motionPool().at(i).setMv(controller->motionAtAbs(i).actualVel());
				target.model->motionPool().at(i).setMa(0.0);
			}

			target.model->solverPool()[1].kinPos();
			target.model->solverPool()[1].kinVel();
			double ee_acc[6]{ 0,0,0,0,0,0 };
			target.model->generalMotionPool()[0].setMaa(ee_acc);
			target.model->solverPool()[0].dynAccAndFce();
			target.model->solverPool()[2].dynAccAndFce();

			double ft_friction[6];
			double ft_offset[6];
			double real_vel[6];
			double ft_friction1[6], ft_friction2[6], ft_dynamic[6], ft_pid[6];
			static double ft_friction2_index[6] = { 3.0, 3.0, 3.0, 3.0, 3.0, 3.0 };
			if (is_running)
			{
				//位置环PID+速度限制
				target.model->generalMotionPool().at(0).getMpq(param.pqa.data());
				for (Size i = 0; i < param.kp_p.size(); ++i)
				{
					param.vt[i] = param.kp_p[i] * (param.pqt[i] - param.pqa[i]);
					param.vt[i] = std::max(std::min(param.vt[i], vt_limit[i]), -vt_limit[i]);
				}

				//速度环PID+力及力矩的限制
				target.model->generalMotionPool().at(0).getMvq(param.va.data());
				for (Size i = 0; i < param.ft.size(); ++i)
				{
					double vinteg_limit;

					//vproportion[i] = std::min(r2*limit, std::max(-r2*limit, param.kp_v[i] * (param.vt[i] - va[i])));
					vproportion[i] = param.kp_v[i] * (param.vt[i] - param.va[i]);
					vinteg_limit = std::max(0.0, mt_limit[i] - vproportion[i]);
					vinteg[i] = std::min(vinteg_limit, std::max(-vinteg_limit, vinteg[i] + param.ki_v[i] * (param.vt[i] - param.va[i])));

					param.ft[i] = vproportion[i] + vinteg[i];
					//力的限制
					//if (i < 3)
					//{
					//	param.ft[i] = std::max(std::min(param.ft[i], ft_limit), -ft_limit);
					//}
					//力矩的限制
					//else
					//{
					//	param.ft[i] = std::max(std::min(param.ft[i], Mt_limit), -Mt_limit);
					//}
				}

				s_c3a(param.pqa.data(), param.ft.data(), param.ft.data() + 3);

				//通过雅克比矩阵将param.ft转换到关节param.ft_pid
				auto &fwd = dynamic_cast<aris::dynamic::ForwardKinematicSolver&>(target.model->solverPool()[1]);

				fwd.cptJacobi();
				/*		double U[36], tau[6], tau2[6], J_fce[36];
						Size p[6], rank;

						s_householder_utp(6, 6, inv.Jf(), U, tau, p, rank);
						s_householder_utp2pinv(6, 6, rank, U, tau, p, J_fce, tau2);*/
	
				s_mm(6, 1, 6, fwd.Jf(), aris::dynamic::ColMajor{ 6 }, param.ft.data(), 1, param.ft_pid.data(), 1);

				//动力学载荷
				for (Size i = 0; i < param.ft.size(); ++i)
				{
					//double ft_friction1, ft_friction2, ft_dynamic, ft_pid;

					//动力学参数
					//constexpr double f_static[6] = { 9.349947583,11.64080253,4.770140543,3.631416685,2.58310847,1.783739862 };
					//constexpr double f_vel[6] = { 7.80825641,13.26518528,7.856443575,3.354615249,1.419632126,0.319206404 };
					//constexpr double f_acc[6] = { 0,3.555679326,0.344454603,0.148247716,0.048552673,0.033815455 };
					//constexpr double f2c_index[6] = { 9.07327526291993, 9.07327526291993, 17.5690184835913, 39.0310903520972, 66.3992503259041, 107.566785527965 };
					//constexpr double f_static_index[6] = {0.5, 0.5, 0.5, 0.85, 0.95, 0.8};

					//静摩擦力+动摩擦力=ft_friction

					real_vel[i] = std::max(std::min(max_static_vel[i], controller->motionAtAbs(i).actualVel()), -max_static_vel[i]);
					ft_friction1[i] = 0.8*(f_static[i] * real_vel[i] / max_static_vel[i]);

					double ft_friction2_max = std::max(0.0, controller->motionAtAbs(i).actualVel() >= 0 ? f_static[i] - ft_friction1[i] : f_static[i] + ft_friction1[i]);
					double ft_friction2_min = std::min(0.0, controller->motionAtAbs(i).actualVel() >= 0 ? -f_static[i] + ft_friction1[i] : -f_static[i] - ft_friction1[i]);

					ft_friction2[i] = std::max(ft_friction2_min, std::min(ft_friction2_max, ft_friction2_index[i] * param.ft_pid[i]));

					ft_friction[i] = ft_friction1[i] + ft_friction2[i] + f_vel[i] * controller->motionAtAbs(i).actualVel();

					//auto real_vel = std::max(std::min(max_static_vel[i], controller->motionAtAbs(i).actualVel()), -max_static_vel[i]);
					//ft_friction = (f_vel[i] * controller->motionAtAbs(i).actualVel() + f_static_index[i] * f_static[i] * real_vel / max_static_vel[i])*f2c_index[i];

					ft_friction[i] = std::max(-500.0, ft_friction[i]);
					ft_friction[i] = std::min(500.0, ft_friction[i]);

					//动力学载荷=ft_dynamic
					ft_dynamic[i] = target.model->motionPool()[i].mfDyn();

					//PID输入=ft_pid
					ft_pid[i] = param.ft_pid[i];
					//ft_pid = 0.0;

					ft_offset[i] = (ft_friction[i] + ft_dynamic[i] + ft_pid[i])*f2c_index[i];
					controller->motionAtAbs(i).setTargetCur(ft_offset[i]);
				}
			}

			//打印//
			auto &cout = controller->mout();
			if (target.count % 1000 == 0)
			{
				cout << "friction1 and friction2:";
				for (Size i = 0; i < 6; i++)
				{
					cout << ft_friction1[i] << "  ";
					cout << ft_friction2[i] << "  ";
				}
				cout << std::endl;
				cout << "vt:";
				for (Size i = 0; i < 6; i++)
				{
					cout << param.vt[i] << "  ";
				}
				cout << std::endl;

				cout << "ft:";
				for (Size i = 0; i < 6; i++)
				{
					cout << param.ft[i] << "  ";
				}
				cout << std::endl;

				cout << "fi:";
				for (Size i = 0; i < 6; i++)
				{
					cout << param.ft_pid[i] * f2c_index[i] << "  ";
				}
				cout << std::endl;


				cout << "------------------------------------------------" << std::endl;

				/*
				for (Size i = 0; i < 6; i++)
				{
					cout << std::setw(6) << "pos" << i + 1 << ":" << controller->motionAtAbs(i).actualPos();
					cout << std::setw(6) << "vel" << i + 1 << ":" << controller->motionAtAbs(i).actualVel();
					cout << std::setw(6) << "cur" << i + 1 << ":" << controller->motionAtAbs(i).actualCur();
				}
				cout << std::endl;*/
			}

			// log //
			auto &lout = controller->lout();
			for (Size i = 0; i < param.kp_p.size(); i++)
			{
				lout << param.kp_p[i] << ",";
				lout << param.kp_v[i] << ",";
				lout << param.ki_v[i] << ",";
				lout << param.pqt[i] << ",";
				lout << param.pqa[i] << ",";
				lout << param.vt[i] << ",";
				lout << param.va[i] << ",";
			}
			for (Size i = 0; i < param.ft.size(); i++)
			{
				lout << vproportion[i] << ",";
				lout << vinteg[i] << ",";
				lout << param.ft[i] << ",";
				lout << param.ft_pid[i] << ",";
				lout << ft_friction1[i] << ",";
				lout << ft_friction2[i] << ",";
				lout << ft_friction[i] << ",";
				lout << ft_dynamic[i] << ",";
				lout << ft_offset[i] << ",";
				lout << controller->motionAtAbs(i).targetCur() << ",";
				lout << controller->motionAtAbs(i).actualPos() << ",";
				lout << controller->motionAtAbs(i).actualVel() << ",";
				lout << controller->motionAtAbs(i).actualCur();
			}
			lout << std::endl;

			return (!is_running&&ds_is_all_finished&&md_is_all_finished) ? 0 : 1;
		}
	auto MovePQCrash::collectNrt(PlanTarget &target)->void {}
	MovePQCrash::MovePQCrash(const std::string &name) :Plan(name)
		{
			command().loadXmlStr(
				"<movePQCrash>"
				"	<group type=\"GroupParam\" default_child_type=\"Param\">"
				"		<pqt default=\"{0.42,0.0,0.55,0.0,0.0,0.0,1.0}\" abbreviation=\"p\"/>"
				"		<kp_p default=\"{1.0,1.0,1.0,1.0,1.0,1.0,1.0}\"/>"
				"		<kp_v default=\"0.1*{100,100,100,100,100,100}\"/>"
				"		<ki_v default=\"30*{1,1,1,1,1,1}\"/>"
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
				"</movePQCrash>");
		}

	
	// 力控末端跟随——末端pq由MoveSetPQ给定，前三根轴执行末端PID控制，保证末端执行到指定位置；最后三根轴通过轴空间PID控制，并保持末端姿态不变——速度前馈；电流控制 //
	struct MovePQBParam
	{
		std::vector<double> kp_p;
		std::vector<double> kp_v;
		std::vector<double> ki_v;

		std::vector<double> pqt;
		std::vector<double> pqa;
		std::vector<double> pqb;
		std::vector<double> vqb;
		std::vector<double> vqt;
		std::vector<double> pt;
		std::vector<double> pa;
		std::vector<double> vt;
		std::vector<double> va;
		std::vector<double> vfwd;

		std::vector<double> ft;
		std::vector<double> ft_pid;
        std::vector<double> vinteg;
        std::vector<double> vproportion;

        aris::Size start_count = 1;
        aris::Size actual_count = 1;

	};
	static std::atomic_bool enable_movePQB = true;
	static std::atomic<std::array<double, 7> > setpqPQB;
	std::atomic_bool one_time_counter = false;
	std::atomic_int16_t which_func = 1;
	std::function<std::array<double, 7>(aris::Size count, aris::Size &start_count)> func[2];
	//pq接口函数
	std::array<double, 7> load_pq1(aris::Size count, aris::Size &start_count){return setpqPQB.load(); }
	std::array<double, 7> load_pq3(aris::Size count, aris::Size &start_count)
	{
        double target_pq[5][7] = { {-0.122203,0.386206,0.0139912,-0.492466,0.474288,0.511942,0.520041},{-0.122203,0.466206,0.0139912,-0.492466,0.474288,0.511942,0.520041},{0.162203,0.466206,0.0139912,-0.492466,0.474288,0.511942,0.520041},{0.162203,0.386206,0.0139912,-0.492466,0.474288,0.511942,0.520041},{-0.122203,0.386206,0.0139912,-0.492466,0.474288,0.511942,0.520041} };
        double vel = 0.04, acc = 0.08, dec = 0.08;
		static aris::Size total_count[4] = { 1,1,1,1 };
		
        std::array<double, 7> temp = {-0.122203,0.386206,0.0139912,-0.492466,0.474288,0.511942,0.520041};
		
		//获取每段梯形轨迹的时间
		double p, v, a;
		aris::Size t_count;
		if (count == start_count)
		{
			for (aris::Size j = 0; j < 4; j++)
			{
				for (aris::Size i = 0; i < 3; i++)
				{
					aris::plan::moveAbsolute(count - start_count + 1, target_pq[j][i], target_pq[j+1][i], vel / 1000, acc / 1000 / 1000, dec / 1000 / 1000, temp[i], v, a, t_count);
					total_count[j] = std::max(total_count[j], t_count);
				}
			}
		}
		
		//获取根据输入的target.count选择执行哪一段梯形轨迹
		if (count <= start_count + total_count[0])
		{
			for (aris::Size i = 0; i < 3; i++)
			{
				aris::plan::moveAbsolute(count - start_count + 1, target_pq[0][i], target_pq[1][i], vel / 1000, acc / 1000 / 1000, dec / 1000 / 1000, temp[i], v, a, t_count);
			}
		}
		else if (count > start_count + total_count[0] && count <= start_count + total_count[0] + total_count[1])
		{
			for (aris::Size i = 0; i < 3; i++)
			{
				aris::plan::moveAbsolute(count - start_count + 1, target_pq[1][i], target_pq[2][i], vel / 1000, acc / 1000 / 1000, dec / 1000 / 1000, temp[i], v, a, t_count);
			}
		}
		else if (count > start_count + total_count[0] + total_count[1] && count <= start_count + total_count[0] + total_count[1] + total_count[2])
		{
			for (aris::Size i = 0; i < 3; i++)
			{
				aris::plan::moveAbsolute(count - start_count + 1, target_pq[2][i], target_pq[3][i], vel / 1000, acc / 1000 / 1000, dec / 1000 / 1000, temp[i], v, a, t_count);
			}
		}
		else if (count > start_count + total_count[0] + total_count[1] + total_count[2] && count <= start_count + total_count[0] + total_count[1] + total_count[2] + total_count[3])
		{
			for (aris::Size i = 0; i < 3; i++)
			{
				aris::plan::moveAbsolute(count - start_count + 1, target_pq[3][i], target_pq[4][i], vel / 1000, acc / 1000 / 1000, dec / 1000 / 1000, temp[i], v, a, t_count);
			}
		}	
		
		return temp; 
	}
	//电机控制模式切换函数
	auto motor_control_mode(PlanTarget &target, bool &is_running, bool &ds_is_all_finished, bool &md_is_all_finished)
	{
		auto &param = std::any_cast<MovePQBParam&>(target.param);
		auto controller = dynamic_cast<aris::control::Controller *>(target.master);
		//第一个周期，将目标电机的控制模式切换到电流控制模式		
		if (target.count == 1)
		{
			//is_running = true;
			for (Size i = 0; i < param.ft.size(); ++i)
			{
				controller->motionPool().at(i).setModeOfOperation(10);	//切换到电流控制
			}
		}

		//最后一个周期将目标电机去使能
		if (!enable_movePQB)
		{
			is_running = false;
		}
		if (!is_running)
		{
			for (Size i = 0; i < param.ft.size(); ++i)
			{
				auto ret = controller->motionPool().at(i).disable();
				if (ret)
				{
					ds_is_all_finished = false;
				}
			}
		}

		//将目标电机由电流模式切换到位置模式
		if (!is_running&&ds_is_all_finished)
		{
			for (Size i = 0; i < param.ft.size(); ++i)
			{
				controller->motionPool().at(i).setModeOfOperation(8);
				auto ret = controller->motionPool().at(i).mode(8);
				if (ret)
				{
					md_is_all_finished = false;
				}
			}
		}

	}
	//加载pq函数
	auto load_func(PlanTarget &target, std::function<std::array<double, 7>(aris::Size count, aris::Size &start_count)> func)->void
	{
		auto &param = std::any_cast<MovePQBParam&>(target.param);
		std::array<double, 7> temp;
        temp = func(param.actual_count, param.start_count);
		std::copy(temp.begin(), temp.end(), param.pqt.begin());
	}
	//力控算法函数
    auto force_control_algorithm(PlanTarget &target, bool is_running)->void
	{
		auto &param = std::any_cast<MovePQBParam&>(target.param);
		auto controller = dynamic_cast<aris::control::Controller *>(target.master);

		//求目标位置pq的运动学反解，获取电机实际位置、实际速度
		target.model->generalMotionPool().at(0).setMpq(param.pqt.data());
        target.model->solverPool().at(0).kinPos();
		for (Size i = 0; i < param.pt.size(); ++i)
		{
			param.pt[i] = target.model->motionPool().at(i).mp();		//motionPool()指模型驱动器，at(0)表示第1个驱动器
			param.pa[i] = controller->motionPool().at(i).actualPos();
			param.va[i] = controller->motionPool().at(i).actualVel();
		}

		//模型运动学正解、动力学正解
		for (int i = 0; i < param.ft.size(); ++i)
		{
			target.model->motionPool()[i].setMp(controller->motionPool()[i].actualPos());
			target.model->motionPool().at(i).setMv(controller->motionAtAbs(i).actualVel());
			target.model->motionPool().at(i).setMa(0.0);
		}
		target.model->solverPool()[1].kinPos();
		target.model->solverPool()[1].kinVel();
		target.model->solverPool()[2].dynAccAndFce();

		//末端空间PID开始位置
		target.model->generalMotionPool().at(0).getMpq(param.pqb.data());
		target.model->generalMotionPool().at(0).getMvq(param.vqb.data());

		//角度不变，位置变化
		target.model->generalMotionPool().at(0).getMpq(param.pqa.data());
		//std::array<double, 4> q = { 0.0,0.0,0.0,1.0 };
		std::copy(param.pqt.begin() + 3, param.pqt.end(), param.pqa.begin() + 3);

		target.model->generalMotionPool().at(0).setMpq(param.pqa.data());
        target.model->solverPool().at(0).kinPos();
		for (Size i = 3; i < param.pt.size(); ++i)
		{
			param.pt[i] = target.model->motionPool().at(i).mp();		//motionPool()指模型驱动器，at(0)表示第1个驱动器
		}

		double ft_friction[6];
		double ft_offset[6];
		double real_vel[6];
		double ft_friction1[6], ft_friction2[6], ft_dynamic[6];
		static double ft_friction2_index[6] = { 5.0, 5.0, 5.0, 5.0, 5.0, 3.0 };
		if (is_running)
		{
			//前三轴，末端空间——位置环PID+速度限制
			for (Size i = 0; i < 3; ++i)
			{
				param.vt[i] = param.kp_p[i] * (param.pqt[i] - param.pqb[i]);
				param.vt[i] = param.vt[i] + param.vfwd[i];
				//param.vt[i] = std::max(std::min(param.vt[i], vt_limit_PQB[i]), -vt_limit_PQB[i]);
			}
			//前三轴，限制末端空间vt向量的模的大小
			double normv = aris::dynamic::s_norm(3, param.vt.data());
			double normv_limit = std::max(std::min(normv, vt_normv_limit), -vt_normv_limit);
			aris::dynamic::s_vc(3, normv_limit / normv, param.vt.data(), param.vt.data());
			std::array<double, 4> vq = { 0.0,0.0,0.0,0.0 };
			std::copy(param.vt.begin(), param.vt.begin() + 3, param.vqt.begin());
			std::copy(vq.begin(), vq.end(), param.vqt.begin() + 3);

			//前三轴，限制末端空间vqt向量的大小
			for (Size i = 0; i < 3; ++i)
			{
				param.vqt[i] = std::max(std::min(param.vt[i], vt_limit_PQB[i]), -vt_limit_PQB[i]);
			}

			target.model->generalMotionPool().at(0).setMvq(param.vqt.data());
			target.model->solverPool()[0].kinVel();
			for (int i = 0; i < 3; ++i)
			{
				param.vt[i] = target.model->motionPool()[i].mv();	//motionPool()指模型驱动器，at(0)表示第1个驱动器
			}

			/*前三轴，末端空间速度环----------------------------------------------------------------------------------------start
			//前三轴，末端空间——速度环PID+力及力矩的限制
			for (Size i = 0; i < 3; ++i)
			{
				param.vproportion[i] = param.kp_v[i] * (param.vt[i] - param.vqb[i]);
				param.vinteg[i] = param.vinteg[i] + param.ki_v[i] * (param.vt[i] - param.vqb[i]);
				//vinteg[i] = std::min(vinteg[i], fi_limit_PQB[i]);
				//vinteg[i] = std::max(vinteg[i], -fi_limit_PQB[i]);
			}
			//前三轴，限制末端空间vinteg向量的模的大小
			double normvi = aris::dynamic::s_norm(3, param.vinteg.data());
			double normvi_limit = std::max(std::min(normvi, fi_limit_PQB[0]), -fi_limit_PQB[0]);
			aris::dynamic::s_vc(3, normvi_limit / normvi, param.vinteg.data(), param.vinteg.data());

			for (Size i = 0; i < 3; ++i)
			{
				param.ft[i] = param.vproportion[i] + param.vinteg[i];
				//param.ft[i] = std::min(param.ft[i], ft_limit_PQB[i]);
				//param.ft[i] = std::max(param.ft[i], -ft_limit_PQB[i]);
			}

			//前三轴，限制末端空间ft向量的模的大小
			double normf = aris::dynamic::s_norm(3, param.ft.data());
			double normf_limit = std::max(std::min(normf, ft_limit_PQB[0]), -ft_limit_PQB[0]);
			aris::dynamic::s_vc(3, normf_limit / normf, param.ft.data(), param.ft.data());

			//前三轴，末端力向量平移到大地坐标系
			s_c3(param.pqb.data(), param.ft.data(), param.ft.data() + 3);

			//前三轴，通过力雅克比矩阵将param.ft转换到关节param.ft_pid
			auto &fwd = dynamic_cast<aris::dynamic::ForwardKinematicSolver&>(target.model->solverPool()[1]);
			fwd.cptJacobi();
			s_mm(6, 1, 6, fwd.Jf(), aris::dynamic::ColMajor{ 6 }, param.ft.data(), 1, param.ft_pid.data(), 1);
			前三轴，末端空间PID----------------------------------------------------------------------------------------------end */


			//后三轴，轴空间——位置环PID+速度限制
			for (Size i = 3; i < param.ft_pid.size(); ++i)
			{
				param.vt[i] = param.kp_p[i] * (param.pt[i] - param.pa[i]);
				param.vt[i] = std::max(std::min(param.vt[i], vt_limit_PQB[i]), -vt_limit_PQB[i]);
				param.vt[i] = param.vt[i] + param.vfwd[i];
			}

			////后三轴，轴空间——速度环PID+力及力矩的限制
			//for (Size i = 3; i < param.ft_pid.size(); ++i)

			//6根轴，轴空间——速度环PID+力及力矩的限制
			for (Size i = 0; i < param.ft_pid.size(); ++i)
			{
				param.vproportion[i] = param.kp_v[i] * (param.vt[i] - param.va[i]);
				param.vinteg[i] = param.vinteg[i] + param.ki_v[i] * (param.vt[i] - param.va[i]);
				param.vinteg[i] = std::min(param.vinteg[i], fi_limit_JFB[i]);
				param.vinteg[i] = std::max(param.vinteg[i], -fi_limit_JFB[i]);

				param.ft_pid[i] = param.vproportion[i] + param.vinteg[i];
				param.ft_pid[i] = std::min(param.ft_pid[i], ft_limit_JFB[i]);
				param.ft_pid[i] = std::max(param.ft_pid[i], -ft_limit_JFB[i]);
			}

			//动力学载荷
			for (Size i = 0; i < param.ft_pid.size(); ++i)
			{
				//动力学参数
				//constexpr double f_static[6] = { 9.349947583,11.64080253,4.770140543,3.631416685,2.58310847,1.783739862 };
				//constexpr double f_vel[6] = { 7.80825641,13.26518528,7.856443575,3.354615249,1.419632126,0.319206404 };
				//constexpr double f_acc[6] = { 0,3.555679326,0.344454603,0.148247716,0.048552673,0.033815455 };
				//constexpr double f2c_index[6] = { 9.07327526291993, 9.07327526291993, 17.5690184835913, 39.0310903520972, 66.3992503259041, 107.566785527965 };
				//constexpr double f_static_index[6] = {0.5, 0.5, 0.5, 0.85, 0.95, 0.8};

				//静摩擦力+动摩擦力=ft_friction

				real_vel[i] = std::max(std::min(max_static_vel[i], controller->motionAtAbs(i).actualVel()), -max_static_vel[i]);
				ft_friction1[i] = 0.8*(f_static[i] * real_vel[i] / max_static_vel[i]);

				//double ft_friction2_max = std::max(0.0, controller->motionAtAbs(i).actualVel() >= 0 ? f_static[i] - ft_friction1[i] : f_static[i] + ft_friction1[i]);
				//double ft_friction2_min = std::min(0.0, controller->motionAtAbs(i).actualVel() >= 0 ? -f_static[i] + ft_friction1[i] : -f_static[i] - ft_friction1[i]);
				//ft_friction2[i] = std::max(ft_friction2_min, std::min(ft_friction2_max, ft_friction2_index[i] * param.ft[i]));
				//ft_friction[i] = ft_friction1[i] + ft_friction2[i] + f_vel[i] * controller->motionAtAbs(i).actualVel();

				ft_friction[i] = ft_friction1[i] + f_vel[i] * controller->motionAtAbs(i).actualVel();

				ft_friction[i] = std::max(-500.0, ft_friction[i]);
				ft_friction[i] = std::min(500.0, ft_friction[i]);

				//动力学载荷=ft_dynamic
				ft_dynamic[i] = target.model->motionPool()[i].mfDyn();

				ft_offset[i] = (ft_friction[i] + ft_dynamic[i] + param.ft_pid[i])*f2c_index[i];
				controller->motionAtAbs(i).setTargetCur(ft_offset[i]);
			}
		}

		//print//
		auto &cout = controller->mout();
		if (target.count % 1000 == 0)
		{
			cout << "pt:";
			for (Size i = 0; i < 6; i++)
			{
				cout << std::setw(10) << param.pt[i] << "  ";
			}
			cout << std::endl;

			cout << "pa:";
			for (Size i = 0; i < 6; i++)
			{
				cout << std::setw(10) << param.pa[i] << "  ";
			}
			cout << std::endl;

			cout << "vt:";
			for (Size i = 0; i < 6; i++)
			{
				cout << std::setw(10) << param.vt[i] << "  ";
			}
			cout << std::endl;

			cout << "va:";
			for (Size i = 0; i < 6; i++)
			{
				cout << std::setw(10) << param.va[i] << "  ";
			}
			cout << std::endl;

			cout << "vproportion:";
			for (Size i = 0; i < 6; i++)
			{
				cout << std::setw(10) << param.vproportion[i] << "  ";
			}
			cout << std::endl;

			cout << "vinteg:";
			for (Size i = 0; i < 6; i++)
			{
				cout << std::setw(10) << param.vinteg[i] << "  ";
			}
			cout << std::endl;

			cout << "ft:";
			for (Size i = 0; i < 6; i++)
			{
				cout << std::setw(10) << param.ft[i] << "  ";
			}
			cout << std::endl;

			cout << "ft_pid:";
			for (Size i = 0; i < 6; i++)
			{
				cout << std::setw(10) << param.ft_pid[i] << "  ";
			}
			cout << std::endl;
			cout << "friction1:";
			for (Size i = 0; i < 6; i++)
			{
				cout << std::setw(10) << ft_friction1[i] << "  ";
			}
			cout << std::endl;
			cout << "friction:";
			for (Size i = 0; i < 6; i++)
			{
				cout << std::setw(10) << ft_friction[i] << "  ";
			}
			cout << std::endl;
			cout << "ft_dynamic:";
			for (Size i = 0; i < 6; i++)
			{
				cout << std::setw(10) << ft_dynamic[i] << "  ";
			}
			cout << std::endl;
			cout << "ft_offset:";
			for (Size i = 0; i < 6; i++)
			{
				cout << std::setw(10) << ft_offset[i] << "  ";
			}
			cout << std::endl;
			cout << "------------------------------------------------" << std::endl;
		}

		//log//
		auto &lout = controller->lout();
		for (Size i = 0; i < param.ft_pid.size(); i++)
		{
			lout << param.pt[i] << ",";
			lout << controller->motionAtAbs(i).actualPos() << ",";
			lout << param.vt[i] << ",";
			lout << controller->motionAtAbs(i).actualVel() << ",";
			lout << ft_offset[i] << ",";
			lout << controller->motionAtAbs(i).actualCur() << ",";
			lout << param.vproportion[i] << ",";
			lout << param.vinteg[i] << ",";
			lout << param.ft[i] << ",";
			lout << param.ft_pid[i] << ",";
			lout << ft_friction1[i] << ",";
			lout << ft_friction[i] << ",";
			lout << ft_dynamic[i] << ",";
		}
		lout << std::endl;
	}
	//MovePQB成员函数实现
	auto MovePQB::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		auto c = dynamic_cast<aris::control::Controller*>(target.master);
		MovePQBParam param;

		enable_movePQB = true;
		param.kp_p.resize(6, 0.0);
		param.kp_v.resize(6, 0.0);
		param.ki_v.resize(6, 0.0);

		param.pqt.resize(7, 0.0);
		param.pqa.resize(7, 0.0);
		param.pqb.resize(7, 0.0);
		param.vqb.resize(7, 0.0);
		param.vqt.resize(7, 0.0);
		param.pt.resize(6, 0.0);
		param.pa.resize(6, 0.0);
		param.vt.resize(6, 0.0);
		param.va.resize(6, 0.0);
		param.vfwd.resize(6, 0.0);
		param.ft.resize(6, 0.0);
		param.ft_pid.resize(6, 0.0);
        param.vinteg.resize(6, 0.0);
        param.vproportion.resize(6, 0.0);

		for (auto &p : params)
		{
			if (p.first == "pqt")
			{
				auto pqarray = target.model->calculator().calculateExpression(p.second);
				param.pqt.assign(pqarray.begin(), pqarray.end());
				std::array<double, 7> temp;
				std::copy(pqarray.begin(), pqarray.end(), temp.begin());
				setpqPQB.store(temp);
				func[0] = load_pq1;
				func[1] = load_pq1;
			}
			else if (p.first == "kp_p")
			{
				auto v = target.model->calculator().calculateExpression(p.second);
				if (v.size() == 1)
				{
					std::fill(param.kp_p.begin(), param.kp_p.end(), v.toDouble());
				}
				else if (v.size() == param.kp_p.size())
				{
					param.kp_p.assign(v.begin(), v.end());
				}
				else
				{
					throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
				}

				for (Size i = 0; i < param.kp_p.size(); ++i)
				{
					if (param.kp_p[i] > 10000.0)
					{
						param.kp_p[i] = 10000.0;
					}
					if (param.kp_p[i] < 0.01)
					{
						param.kp_p[i] = 0.01;
					}
				}
			}
			else if (p.first == "kp_v")
			{
				auto a = target.model->calculator().calculateExpression(p.second);
				if (a.size() == 1)
				{
					std::fill(param.kp_v.begin(), param.kp_v.end(), a.toDouble());
				}
				else if (a.size() == param.kp_v.size())
				{
					param.kp_v.assign(a.begin(), a.end());
				}
				else
				{
					throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
				}

				for (Size i = 0; i < param.kp_v.size(); ++i)
				{
					if (param.kp_v[i] > 10000.0)
					{
						param.kp_v[i] = 10000.0;
					}
					if (param.kp_v[i] < 0.01)
					{
						param.kp_v[i] = 0.01;
					}
				}
			}
			else if (p.first == "ki_v")
			{
				auto d = target.model->calculator().calculateExpression(p.second);
				if (d.size() == 1)
				{
					std::fill(param.ki_v.begin(), param.ki_v.end(), d.toDouble());
				}
				else if (d.size() == param.ki_v.size())
				{
					param.ki_v.assign(d.begin(), d.end());
				}
				else
				{
					throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
				}

				for (Size i = 0; i < param.ki_v.size(); ++i)
				{
					if (param.ki_v[i] > 10000.0)
					{
						param.ki_v[i] = 10000.0;
					}
					if (param.ki_v[i] < 0.001)
					{
						param.ki_v[i] = 0.001;
					}
				}
			}
		}
		target.param = param;

		target.option |=
			Plan::USE_VEL_OFFSET |
			//#ifdef WIN32
			Plan::NOT_CHECK_POS_MIN |
			Plan::NOT_CHECK_POS_MAX |
			Plan::NOT_CHECK_POS_CONTINUOUS |
			Plan::NOT_CHECK_POS_CONTINUOUS_AT_START |
			Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
			Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START |
			Plan::NOT_CHECK_POS_FOLLOWING_ERROR |
			//#endif
			Plan::NOT_CHECK_VEL_MIN |
			Plan::NOT_CHECK_VEL_MAX |
			Plan::NOT_CHECK_VEL_CONTINUOUS |
			Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START |
			Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;

	}
	auto MovePQB::executeRT(PlanTarget &target)->int
	{
		auto &param = std::any_cast<MovePQBParam&>(target.param);
		auto controller = dynamic_cast<aris::control::Controller *>(target.master);
		controller->logFile("movePQB.txt");
		static bool is_running{ true };
		bool ds_is_all_finished{ true };
		bool md_is_all_finished{ true };

		//切换电机控制模式
		motor_control_mode(target, is_running, ds_is_all_finished, md_is_all_finished);

		//函数选择判断
        if(which_func == 1)
        {
            func[0] = load_pq1;
        }
        else if(which_func != 1)
        {
			func[0] = func[1];
            if(one_time_counter)
            {
                param.start_count = target.count;
                one_time_counter = false;
            }
        }
        param.actual_count = target.count;
		//加载数据
		load_func(target, func[0]);

		//力控算法
		force_control_algorithm(target, is_running);
        auto &cout = controller->mout();
        if (target.count % 1000 == 0)
        {
            cout <<"one_time_counter:  "<<one_time_counter<<std::endl;
        }

		return (!is_running&&ds_is_all_finished&&md_is_all_finished) ? 0 : 1;
	}
	auto MovePQB::collectNrt(PlanTarget &target)->void {}
	MovePQB::MovePQB(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<movePQB>"
			"	<group type=\"GroupParam\" default_child_type=\"Param\">"
            "		<pqt default=\"{0.42,0.0,0.55,0,0,0,1}\" abbreviation=\"p\"/>"
            "		<kp_p default=\"{4,6,4,3,3,2}\"/>"
            "		<kp_v default=\"{170,270,90,60,35,16}\"/>"
            "		<ki_v default=\"{2,15,10,0.2,0.2,0.18}\"/>"
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
			"</movePQB>");
	}


	// 力控轴空间——输入末端pq姿态；先对末端pq进行梯形轨迹规划，然后规划好的末端pq反解到轴空间，最后通过轴空间PID算法控制每个电机——好实现，抖动小；速度前馈；电流控制 //
	struct MoveJCrashParam
	{
		std::vector<double> kp_p;
		std::vector<double> kp_v;
		std::vector<double> ki_v;

		std::vector<double> pqt;
		std::vector<double> pqb;
		std::vector<double> vqt;
		std::vector<double> xyz;
		double vel, acc, dec;
		int which_dir;
		std::vector<double> pt;
		std::vector<double> pa;
		std::vector<double> vt;
		std::vector<double> va;
		std::vector<double> vfwd;

		std::vector<double> ft;
	};
	static std::atomic_bool enable_moveJCrash = true;
	auto MoveJCrash::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
		{
			auto c = dynamic_cast<aris::control::Controller*>(target.master);
			MoveJCrashParam param;
			enable_moveJCrash = true;
			param.kp_p.resize(6, 0.0);
			param.kp_v.resize(6, 0.0);
			param.ki_v.resize(6, 0.0);

			param.pqt.resize(7, 0.0);
			param.pqb.resize(7, 0.0);
			param.vqt.resize(7, 0.0);
			param.xyz.resize(3, 0.0);
			param.which_dir = 0;
			param.pt.resize(6, 0.0);
			param.pa.resize(6, 0.0);
			param.vt.resize(6, 0.0);
			param.va.resize(6, 0.0);
			param.vfwd.resize(6, 0.0);
			param.ft.resize(6, 0.0);
			//params.at("pqt")
			for (auto &p : params)
			{
				if (p.first == "pqt")
				{
					auto pqarray = target.model->calculator().calculateExpression(p.second);
					param.pqt.assign(pqarray.begin(), pqarray.end());
					param.which_dir = 0;
				}
				else if (p.first == "xyz")
				{
					auto xyzarray = target.model->calculator().calculateExpression(p.second);
					param.xyz.assign(xyzarray.begin(), xyzarray.end());
					param.which_dir = 1;
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
				else if (p.first == "kp_p")
				{
					auto v = target.model->calculator().calculateExpression(p.second);
					if (v.size() == 1)
					{
						std::fill(param.kp_p.begin(), param.kp_p.end(), v.toDouble());
					}
					else if (v.size() == param.kp_p.size())
					{
						param.kp_p.assign(v.begin(), v.end());
					}
					else
					{
						throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
					}

					for (Size i = 0; i < param.kp_p.size(); ++i)
					{
						if (param.kp_p[i] > 10000.0)
						{
							param.kp_p[i] = 10000.0;
						}
						if (param.kp_p[i] < 0.01)
						{
							param.kp_p[i] = 0.01;
						}
					}
				}
				else if (p.first == "kp_v")
				{
					auto a = target.model->calculator().calculateExpression(p.second);
					if (a.size() == 1)
					{
						std::fill(param.kp_v.begin(), param.kp_v.end(), a.toDouble());
					}
					else if (a.size() == param.kp_v.size())
					{
						param.kp_v.assign(a.begin(), a.end());
					}
					else
					{
						throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
					}

					for (Size i = 0; i < param.kp_v.size(); ++i)
					{
						if (param.kp_v[i] > 10000.0)
						{
							param.kp_v[i] = 10000.0;
						}
						if (param.kp_v[i] < 0.01)
						{
							param.kp_v[i] = 0.01;
						}
					}
				}
				else if (p.first == "ki_v")
				{
					auto d = target.model->calculator().calculateExpression(p.second);
					if (d.size() == 1)
					{
						std::fill(param.ki_v.begin(), param.ki_v.end(), d.toDouble());
					}
					else if (d.size() == param.ki_v.size())
					{
						param.ki_v.assign(d.begin(), d.end());
					}
					else
					{
						throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
					}

					for (Size i = 0; i < param.ki_v.size(); ++i)
					{
						if (param.ki_v[i] > 10000.0)
						{
							param.ki_v[i] = 10000.0;
						}
						if (param.ki_v[i] < 0.001)
						{
							param.ki_v[i] = 0.001;
						}
					}
				}

			}
			target.param = param;

			target.option |=
				Plan::USE_VEL_OFFSET |
				//#ifdef WIN32
				Plan::NOT_CHECK_POS_MIN |
				Plan::NOT_CHECK_POS_MAX |
				Plan::NOT_CHECK_POS_CONTINUOUS |
				Plan::NOT_CHECK_POS_CONTINUOUS_AT_START |
				Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
				Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START |
				Plan::NOT_CHECK_POS_FOLLOWING_ERROR |
				//#endif
				Plan::NOT_CHECK_VEL_MIN |
				Plan::NOT_CHECK_VEL_MAX |
				Plan::NOT_CHECK_VEL_CONTINUOUS |
				Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START |
				Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;

		}
	auto MoveJCrash::executeRT(PlanTarget &target)->int
		{
			auto &param = std::any_cast<MoveJCrashParam&>(target.param);
			auto controller = dynamic_cast<aris::control::Controller *>(target.master);
			static bool is_running{ true };
			static double vinteg[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
			static double vproportion[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
			bool ds_is_all_finished{ true };
			bool md_is_all_finished{ true };

			//第一个周期，将目标电机的控制模式切换到电流控制模式		
			if (target.count == 1)
			{
				is_running = true;
				if (param.which_dir == 0)
				{
					target.model->generalMotionPool().at(0).getMpq(param.pqb.data());
				}
				else
				{
					target.model->generalMotionPool().at(0).getMpq(param.pqb.data());
					param.pqt.assign(param.pqb.begin(), param.pqb.end());
				}

				for (Size i = 0; i < param.ft.size(); ++i)
				{
					controller->motionPool().at(i).setModeOfOperation(10);	//切换到电流控制
				}
			}

			//最后一个周期将目标电机去使能
			if (!enable_moveJCrash)
			{
				is_running = false;
			}
			if (!is_running)
			{
				for (Size i = 0; i < param.ft.size(); ++i)
				{
					auto ret = controller->motionPool().at(i).disable();
					if (ret)
					{
						ds_is_all_finished = false;
					}
				}
			}

			//将目标电机由电流模式切换到位置模式//
			if (!is_running&&ds_is_all_finished)
			{
				for (Size i = 0; i < param.ft.size(); ++i)
				{
					controller->motionPool().at(i).setModeOfOperation(8);
					auto ret = controller->motionPool().at(i).mode(8);
					if (ret)
					{
						md_is_all_finished = false;
					}
				}
			}

			//轨迹规划
			aris::Size total_count{ 1 };
			if (param.which_dir == 0)
			{
				//求目标位置pq的运动学反解，获取电机实际位置、实际速度
				target.model->generalMotionPool().at(0).setMpq(param.pqt.data());
				if (!target.model->solverPool().at(0).kinPos())return -1;
				for (Size i = 0; i < param.pt.size(); ++i)
				{
					param.pt[i] = target.model->motionPool().at(i).mp();		//motionPool()指模型驱动器，at(0)表示第1个驱动器
					param.pa[i] = controller->motionPool().at(i).actualPos();
					param.va[i] = controller->motionPool().at(i).actualVel();
				}
			}
			else
			{
				//x,y,z方向梯形轨迹规划
				double norm, p, v, a;
				norm = std::sqrt(param.xyz[0] * param.xyz[0] + param.xyz[1] * param.xyz[1] + param.xyz[2] * param.xyz[2]);
				aris::plan::moveAbsolute(target.count, 0.0, norm, param.vel / 1000, param.acc / 1000 / 1000, param.dec / 1000 / 1000, p, v, a, total_count);

				//double norm = aris::dynamic::s_norm(3, param.xyz.data());
				//aris::dynamic::s_vc(7, param.pqb.data(), param.pqt.data());
				//aris::dynamic::s_va(3, p / norm, param.xyz.data(), param.pqt.data());
				//aris::dynamic::s_vc(3, v / norm * 1000, param.xyz.data(), param.vqt.data());

				for (Size i = 0; i < param.xyz.size(); i++)
				{
					param.pqt[i] = param.pqb[i] + param.xyz[i] * p / norm;
					param.vqt[i] = param.xyz[i] * v / norm * 1000;
				}
				target.model->generalMotionPool().at(0).setMpq(param.pqt.data());
				if (!target.model->solverPool().at(0).kinPos())return -1;
				target.model->generalMotionPool().at(0).setMvq(param.vqt.data());
				target.model->solverPool().at(0).kinVel();

				for (Size i = 0; i < param.pt.size(); ++i)
				{
					param.pt[i] = target.model->motionPool().at(i).mp();		//motionPool()指模型驱动器，at(0)表示第1个驱动器
					param.vfwd[i] = target.model->motionPool().at(i).mv();
					param.pa[i] = controller->motionPool().at(i).actualPos();
					param.va[i] = controller->motionPool().at(i).actualVel();
				}
			}

			//模型正解//
			for (int i = 0; i < param.ft.size(); ++i)
			{
				target.model->motionPool()[i].setMp(controller->motionPool()[i].actualPos());
				target.model->motionPool().at(i).setMv(controller->motionAtAbs(i).actualVel());
				target.model->motionPool().at(i).setMa(0.0);
			}
			target.model->solverPool()[1].kinPos();
			target.model->solverPool()[1].kinVel();
			target.model->solverPool()[2].dynAccAndFce();

			double ft_friction[6];
			double ft_offset[6];
			double real_vel[6];
			double ft_friction1[6], ft_friction2[6], ft_dynamic[6];
			static double ft_friction2_index[6] = { 5.0, 5.0, 5.0, 5.0, 5.0, 3.0 };
			if (is_running)
			{
				//位置环PID+速度限制
				for (Size i = 0; i < param.kp_p.size(); ++i)
				{
					param.vt[i] = param.kp_p[i] * (param.pt[i] - param.pa[i]);
					param.vt[i] = std::max(std::min(param.vt[i], vt_limit[i]), -vt_limit[i]);
					param.vt[i] = param.vt[i] + param.vfwd[i];
				}

				//速度环PID+力及力矩的限制
				for (Size i = 0; i < param.ft.size(); ++i)
				{
					vproportion[i] = param.kp_v[i] * (param.vt[i] - param.va[i]);
					vinteg[i] = vinteg[i] + param.ki_v[i] * (param.vt[i] - param.va[i]);
					vinteg[i] = std::min(vinteg[i], fi_limit[i]);
					vinteg[i] = std::max(vinteg[i], -fi_limit[i]);

					param.ft[i] = vproportion[i] + vinteg[i];
					param.ft[i] = std::min(param.ft[i], ft_limit[i]);
					param.ft[i] = std::max(param.ft[i], -ft_limit[i]);
				}

				//动力学载荷
				for (Size i = 0; i < param.ft.size(); ++i)
				{
					//动力学参数
					//constexpr double f_static[6] = { 9.349947583,11.64080253,4.770140543,3.631416685,2.58310847,1.783739862 };
					//constexpr double f_vel[6] = { 7.80825641,13.26518528,7.856443575,3.354615249,1.419632126,0.319206404 };
					//constexpr double f_acc[6] = { 0,3.555679326,0.344454603,0.148247716,0.048552673,0.033815455 };
					//constexpr double f2c_index[6] = { 9.07327526291993, 9.07327526291993, 17.5690184835913, 39.0310903520972, 66.3992503259041, 107.566785527965 };
					//constexpr double f_static_index[6] = {0.5, 0.5, 0.5, 0.85, 0.95, 0.8};

					//静摩擦力+动摩擦力=ft_friction

					real_vel[i] = std::max(std::min(max_static_vel[i], controller->motionAtAbs(i).actualVel()), -max_static_vel[i]);
					ft_friction1[i] = 0.8*(f_static[i] * real_vel[i] / max_static_vel[i]);

					//double ft_friction2_max = std::max(0.0, controller->motionAtAbs(i).actualVel() >= 0 ? f_static[i] - ft_friction1[i] : f_static[i] + ft_friction1[i]);
					//double ft_friction2_min = std::min(0.0, controller->motionAtAbs(i).actualVel() >= 0 ? -f_static[i] + ft_friction1[i] : -f_static[i] - ft_friction1[i]);
					//ft_friction2[i] = std::max(ft_friction2_min, std::min(ft_friction2_max, ft_friction2_index[i] * param.ft[i]));
					//ft_friction[i] = ft_friction1[i] + ft_friction2[i] + f_vel[i] * controller->motionAtAbs(i).actualVel();

					ft_friction[i] = ft_friction1[i] + f_vel[i] * controller->motionAtAbs(i).actualVel();

					//auto real_vel = std::max(std::min(max_static_vel[i], controller->motionAtAbs(i).actualVel()), -max_static_vel[i]);
					//ft_friction = (f_vel[i] * controller->motionAtAbs(i).actualVel() + f_static_index[i] * f_static[i] * real_vel / max_static_vel[i])*f2c_index[i];

					ft_friction[i] = std::max(-500.0, ft_friction[i]);
					ft_friction[i] = std::min(500.0, ft_friction[i]);

					//动力学载荷=ft_dynamic
					ft_dynamic[i] = target.model->motionPool()[i].mfDyn();

					ft_offset[i] = (ft_friction[i] + ft_dynamic[i] + param.ft[i])*f2c_index[i];
					controller->motionAtAbs(i).setTargetCur(ft_offset[i]);
				}
			}

			//print//
			auto &cout = controller->mout();
			if (target.count % 1000 == 0)
			{
				cout << "pt:";
				for (Size i = 0; i < 6; i++)
				{
					cout << std::setw(10) << param.pt[i] << "  ";
				}
				cout << std::endl;

				cout << "pa:";
				for (Size i = 0; i < 6; i++)
				{
					cout << std::setw(10) << param.pa[i] << "  ";
				}
				cout << std::endl;

				cout << "vt:";
				for (Size i = 0; i < 6; i++)
				{
					cout << std::setw(10) << param.vt[i] << "  ";
				}
				cout << std::endl;

				cout << "va:";
				for (Size i = 0; i < 6; i++)
				{
					cout << std::setw(10) << param.va[i] << "  ";
				}
				cout << std::endl;

				cout << "vproportion:";
				for (Size i = 0; i < 6; i++)
				{
					cout << std::setw(10) << vproportion[i] << "  ";
				}
				cout << std::endl;

				cout << "vinteg:";
				for (Size i = 0; i < 6; i++)
				{
					cout << std::setw(10) << vinteg[i] << "  ";
				}
				cout << std::endl;

				cout << "ft:";
				for (Size i = 0; i < 6; i++)
				{
					cout << std::setw(10) << param.ft[i] << "  ";
				}
				cout << std::endl;
				cout << "friction1:";
				for (Size i = 0; i < 6; i++)
				{
					cout << std::setw(10) << ft_friction1[i] << "  ";
				}
				cout << std::endl;
				cout << "friction:";
				for (Size i = 0; i < 6; i++)
				{
					cout << std::setw(10) << ft_friction[i] << "  ";
				}
				cout << std::endl;
				cout << "ft_dynamic:";
				for (Size i = 0; i < 6; i++)
				{
					cout << std::setw(10) << ft_dynamic[i] << "  ";
				}
				cout << std::endl;
				cout << "ft_offset:";
				for (Size i = 0; i < 6; i++)
				{
					cout << std::setw(10) << ft_offset[i] << "  ";
				}
				cout << std::endl;
				cout << "------------------------------------------------" << std::endl;
			}

			//log//
			auto &lout = controller->lout();
			for (Size i = 0; i < param.ft.size(); i++)
			{
				lout << param.pt[i] << ",";
				lout << controller->motionAtAbs(i).actualPos() << ",";
				lout << param.vt[i] << ",";
				lout << controller->motionAtAbs(i).actualVel() << ",";
				lout << ft_offset[i] << ",";
				lout << controller->motionAtAbs(i).actualCur() << ",";
				lout << vproportion[i] << ",";
				lout << vinteg[i] << ",";
				lout << param.ft[i] << ",";
				lout << ft_friction1[i] << ",";
				lout << ft_friction[i] << ",";
				lout << ft_dynamic[i] << ",";
			}
			lout << std::endl;

			return (!is_running&&ds_is_all_finished&&md_is_all_finished) ? 0 : 1;
		}
	auto MoveJCrash::collectNrt(PlanTarget &target)->void {}
	MoveJCrash::MoveJCrash(const std::string &name) :Plan(name)
		{
			command().loadXmlStr(
				"<moveJCrash>"
				"	<group type=\"GroupParam\" default_child_type=\"Param\">"
				"		<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"pqt\">"
				"			<pqt default=\"{0.42,0.0,0.55,0,0,0,1}\" abbreviation=\"p\"/>"
				"			<xyz default=\"{0.01,0.0,0.0}\"/>"
				"		</unique>"
				"		<vel default=\"0.05\"/>"
				"		<acc default=\"0.1\"/>"
				"		<dec default=\"0.1\"/>"
				"		<kp_p default=\"{10,12,70,4,6,3}\"/>"
				"		<kp_v default=\"{200,360,120,100,60,20}\"/>"
				"		<ki_v default=\"{2,18,20,0.6,0.5,0.4}\"/>"
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
				"</moveJCrash>");
		}


	// 力控跟随——末端pq由MoveSetPQ给定，然后，末端pq反解到轴空间，通过轴空间PID控制电机动作——速度前馈；电流控制 //
	struct MoveJFParam
	{
		std::vector<double> kp_p;
		std::vector<double> kp_v;
		std::vector<double> ki_v;

		std::vector<double> pqt;
		std::vector<double> pt;
		std::vector<double> pa;
		std::vector<double> vt;
		std::vector<double> va;
		std::vector<double> vfwd;

		std::vector<double> ft;
	};
	static std::atomic_bool enable_moveJF = true;
    static std::atomic<std::array<double, 7> > setpqJF;
	auto MoveJF::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		auto c = dynamic_cast<aris::control::Controller*>(target.master);
		MoveJFParam param;
		enable_moveJF = true;
		param.kp_p.resize(6, 0.0);
		param.kp_v.resize(6, 0.0);
		param.ki_v.resize(6, 0.0);

		param.pqt.resize(7, 0.0);
		param.pt.resize(6, 0.0);
		param.pa.resize(6, 0.0);
		param.vt.resize(6, 0.0);
		param.va.resize(6, 0.0);
		param.vfwd.resize(6, 0.0);
		param.ft.resize(6, 0.0);

		for (auto &p : params)
		{
			if (p.first == "pqt")
			{
				auto pqarray = target.model->calculator().calculateExpression(p.second);
				param.pqt.assign(pqarray.begin(), pqarray.end());
				std::array<double, 7> temp;
				std::copy(pqarray.begin(), pqarray.end(), temp.begin());
				setpqJF.store(temp);
			}
			else if (p.first == "kp_p")
			{
				auto v = target.model->calculator().calculateExpression(p.second);
				if (v.size() == 1)
				{
					std::fill(param.kp_p.begin(), param.kp_p.end(), v.toDouble());
				}
				else if (v.size() == param.kp_p.size())
				{
					param.kp_p.assign(v.begin(), v.end());
				}
				else
				{
					throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
				}

				for (Size i = 0; i < param.kp_p.size(); ++i)
				{
					if (param.kp_p[i] > 10000.0)
					{
						param.kp_p[i] = 10000.0;
					}
					if (param.kp_p[i] < 0.01)
					{
						param.kp_p[i] = 0.01;
					}
				}
			}
			else if (p.first == "kp_v")
			{
				auto a = target.model->calculator().calculateExpression(p.second);
				if (a.size() == 1)
				{
					std::fill(param.kp_v.begin(), param.kp_v.end(), a.toDouble());
				}
				else if (a.size() == param.kp_v.size())
				{
					param.kp_v.assign(a.begin(), a.end());
				}
				else
				{
					throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
				}

				for (Size i = 0; i < param.kp_v.size(); ++i)
				{
					if (param.kp_v[i] > 10000.0)
					{
						param.kp_v[i] = 10000.0;
					}
					if (param.kp_v[i] < 0.01)
					{
						param.kp_v[i] = 0.01;
					}
				}
			}
			else if (p.first == "ki_v")
			{
				auto d = target.model->calculator().calculateExpression(p.second);
				if (d.size() == 1)
				{
					std::fill(param.ki_v.begin(), param.ki_v.end(), d.toDouble());
				}
				else if (d.size() == param.ki_v.size())
				{
					param.ki_v.assign(d.begin(), d.end());
				}
				else
				{
					throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
				}

				for (Size i = 0; i < param.ki_v.size(); ++i)
				{
					if (param.ki_v[i] > 10000.0)
					{
						param.ki_v[i] = 10000.0;
					}
					if (param.ki_v[i] < 0.001)
					{
						param.ki_v[i] = 0.001;
					}
				}
			}

		}
		target.param = param;

		target.option |=
			Plan::USE_VEL_OFFSET |
			//#ifdef WIN32
			Plan::NOT_CHECK_POS_MIN |
			Plan::NOT_CHECK_POS_MAX |
			Plan::NOT_CHECK_POS_CONTINUOUS |
			Plan::NOT_CHECK_POS_CONTINUOUS_AT_START |
			Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
			Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START |
			Plan::NOT_CHECK_POS_FOLLOWING_ERROR |
			//#endif
			Plan::NOT_CHECK_VEL_MIN |
			Plan::NOT_CHECK_VEL_MAX |
			Plan::NOT_CHECK_VEL_CONTINUOUS |
			Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START |
			Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;

	}
	auto MoveJF::executeRT(PlanTarget &target)->int
	{
		auto &param = std::any_cast<MoveJFParam&>(target.param);
		auto controller = dynamic_cast<aris::control::Controller *>(target.master);
		static bool is_running{ true };
		static double vinteg[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
		static double vproportion[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
		bool ds_is_all_finished{ true };
		bool md_is_all_finished{ true };

		//第一个周期，将目标电机的控制模式切换到电流控制模式		
		if (target.count == 1)
		{
			is_running = true;

			for (Size i = 0; i < param.ft.size(); ++i)
			{
				controller->motionPool().at(i).setModeOfOperation(10);	//切换到电流控制
			}
		}

		//最后一个周期将目标电机去使能
		if (!enable_moveJF)
		{
			is_running = false;
		}
		if (!is_running)
		{
			for (Size i = 0; i < param.ft.size(); ++i)
			{
				auto ret = controller->motionPool().at(i).disable();
				if (ret)
				{
					ds_is_all_finished = false;
				}
			}
		}

		//将目标电机由电流模式切换到位置模式
		if (!is_running&&ds_is_all_finished)
		{
			for (Size i = 0; i < param.ft.size(); ++i)
			{
				controller->motionPool().at(i).setModeOfOperation(8);
				auto ret = controller->motionPool().at(i).mode(8);
				if (ret)
				{
					md_is_all_finished = false;
				}
			}
		}

		//通过moveSPQ设置实时目标PQ位置
		std::array<double, 7> temp;
		temp = setpqJF.load();
		std::copy(temp.begin(), temp.end(), param.pqt.begin());
		
		//求目标位置pq的运动学反解，获取电机实际位置、实际速度
		target.model->generalMotionPool().at(0).setMpq(param.pqt.data());
		if (!target.model->solverPool().at(0).kinPos())return -1;
		for (Size i = 0; i < param.pt.size(); ++i)
		{
			param.pt[i] = target.model->motionPool().at(i).mp();		//motionPool()指模型驱动器，at(0)表示第1个驱动器
			param.pa[i] = controller->motionPool().at(i).actualPos();
			param.va[i] = controller->motionPool().at(i).actualVel();
		}
		
		//模型正解
		for (int i = 0; i < param.ft.size(); ++i)
		{
			target.model->motionPool()[i].setMp(controller->motionPool()[i].actualPos());
			target.model->motionPool().at(i).setMv(controller->motionAtAbs(i).actualVel());
			target.model->motionPool().at(i).setMa(0.0);
		}
		target.model->solverPool()[1].kinPos();
		target.model->solverPool()[1].kinVel();
		target.model->solverPool()[2].dynAccAndFce();

		double ft_friction[6];
		double ft_offset[6];
		double real_vel[6];
		double ft_friction1[6], ft_friction2[6], ft_dynamic[6];
		static double ft_friction2_index[6] = { 5.0, 5.0, 5.0, 5.0, 5.0, 3.0 };
		if (is_running)
		{
			//位置环PID+速度限制
			for (Size i = 0; i < param.kp_p.size(); ++i)
			{
				param.vt[i] = param.kp_p[i] * (param.pt[i] - param.pa[i]);
				param.vt[i] = std::max(std::min(param.vt[i], vt_limit[i]), -vt_limit[i]);
				param.vt[i] = param.vt[i] + param.vfwd[i];
			}

			//速度环PID+力及力矩的限制
			for (Size i = 0; i < param.ft.size(); ++i)
			{
				vproportion[i] = param.kp_v[i] * (param.vt[i] - param.va[i]);
				vinteg[i] = vinteg[i] + param.ki_v[i] * (param.vt[i] - param.va[i]);
				vinteg[i] = std::min(vinteg[i], fi_limit[i]);
				vinteg[i] = std::max(vinteg[i], -fi_limit[i]);

				param.ft[i] = vproportion[i] + vinteg[i];
				param.ft[i] = std::min(param.ft[i], ft_limit[i]);
				param.ft[i] = std::max(param.ft[i], -ft_limit[i]);
			}

			//动力学载荷
			for (Size i = 0; i < param.ft.size(); ++i)
			{
				//动力学参数
				//constexpr double f_static[6] = { 9.349947583,11.64080253,4.770140543,3.631416685,2.58310847,1.783739862 };
				//constexpr double f_vel[6] = { 7.80825641,13.26518528,7.856443575,3.354615249,1.419632126,0.319206404 };
				//constexpr double f_acc[6] = { 0,3.555679326,0.344454603,0.148247716,0.048552673,0.033815455 };
				//constexpr double f2c_index[6] = { 9.07327526291993, 9.07327526291993, 17.5690184835913, 39.0310903520972, 66.3992503259041, 107.566785527965 };
				//constexpr double f_static_index[6] = {0.5, 0.5, 0.5, 0.85, 0.95, 0.8};

				//静摩擦力+动摩擦力=ft_friction

				real_vel[i] = std::max(std::min(max_static_vel[i], controller->motionAtAbs(i).actualVel()), -max_static_vel[i]);
				ft_friction1[i] = 0.8*(f_static[i] * real_vel[i] / max_static_vel[i]);

				//double ft_friction2_max = std::max(0.0, controller->motionAtAbs(i).actualVel() >= 0 ? f_static[i] - ft_friction1[i] : f_static[i] + ft_friction1[i]);
				//double ft_friction2_min = std::min(0.0, controller->motionAtAbs(i).actualVel() >= 0 ? -f_static[i] + ft_friction1[i] : -f_static[i] - ft_friction1[i]);
				//ft_friction2[i] = std::max(ft_friction2_min, std::min(ft_friction2_max, ft_friction2_index[i] * param.ft[i]));
				//ft_friction[i] = ft_friction1[i] + ft_friction2[i] + f_vel[i] * controller->motionAtAbs(i).actualVel();

				ft_friction[i] = ft_friction1[i] + f_vel[i] * controller->motionAtAbs(i).actualVel();

				//auto real_vel = std::max(std::min(max_static_vel[i], controller->motionAtAbs(i).actualVel()), -max_static_vel[i]);
				//ft_friction = (f_vel[i] * controller->motionAtAbs(i).actualVel() + f_static_index[i] * f_static[i] * real_vel / max_static_vel[i])*f2c_index[i];

				ft_friction[i] = std::max(-500.0, ft_friction[i]);
				ft_friction[i] = std::min(500.0, ft_friction[i]);

				//动力学载荷=ft_dynamic
				ft_dynamic[i] = target.model->motionPool()[i].mfDyn();

				ft_offset[i] = (ft_friction[i] + ft_dynamic[i] + param.ft[i])*f2c_index[i];
				controller->motionAtAbs(i).setTargetCur(ft_offset[i]);
			}
		}

		//print//
		auto &cout = controller->mout();
		if (target.count % 1000 == 0)
		{
			cout << "pt:";
			for (Size i = 0; i < 6; i++)
			{
				cout << std::setw(10) << param.pt[i] << "  ";
			}
			cout << std::endl;

			cout << "pa:";
			for (Size i = 0; i < 6; i++)
			{
				cout << std::setw(10) << param.pa[i] << "  ";
			}
			cout << std::endl;

			cout << "vt:";
			for (Size i = 0; i < 6; i++)
			{
				cout << std::setw(10) << param.vt[i] << "  ";
			}
			cout << std::endl;

			cout << "va:";
			for (Size i = 0; i < 6; i++)
			{
				cout << std::setw(10) << param.va[i] << "  ";
			}
			cout << std::endl;

			cout << "vproportion:";
			for (Size i = 0; i < 6; i++)
			{
				cout << std::setw(10) << vproportion[i] << "  ";
			}
			cout << std::endl;

			cout << "vinteg:";
			for (Size i = 0; i < 6; i++)
			{
				cout << std::setw(10) << vinteg[i] << "  ";
			}
			cout << std::endl;

			cout << "ft:";
			for (Size i = 0; i < 6; i++)
			{
				cout << std::setw(10) << param.ft[i] << "  ";
			}
			cout << std::endl;
			cout << "friction1:";
			for (Size i = 0; i < 6; i++)
			{
				cout << std::setw(10) << ft_friction1[i] << "  ";
			}
			cout << std::endl;
			cout << "friction:";
			for (Size i = 0; i < 6; i++)
			{
				cout << std::setw(10) << ft_friction[i] << "  ";
			}
			cout << std::endl;
			cout << "ft_dynamic:";
			for (Size i = 0; i < 6; i++)
			{
				cout << std::setw(10) << ft_dynamic[i] << "  ";
			}
			cout << std::endl;
			cout << "ft_offset:";
			for (Size i = 0; i < 6; i++)
			{
				cout << std::setw(10) << ft_offset[i] << "  ";
			}
			cout << std::endl;
			cout << "------------------------------------------------" << std::endl;
		}

		// log //
		auto &lout = controller->lout();
		for (Size i = 0; i < param.ft.size(); i++)
		{
			lout << param.pt[i] << ",";
			lout << controller->motionAtAbs(i).actualPos() << ",";
			lout << param.vt[i] << ",";
			lout << controller->motionAtAbs(i).actualVel() << ",";
			lout << ft_offset[i] << ",";
			lout << controller->motionAtAbs(i).actualCur() << ",";
			lout << vproportion[i] << ",";
			lout << vinteg[i] << ",";
			lout << param.ft[i] << ",";
			lout << ft_friction1[i] << ",";
			lout << ft_friction[i] << ",";
			lout << ft_dynamic[i] << ",";
		}
		lout << std::endl;

		return (!is_running&&ds_is_all_finished&&md_is_all_finished) ? 0 : 1;
	}
	auto MoveJF::collectNrt(PlanTarget &target)->void {}
	MoveJF::MoveJF(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<moveJF>"
			"	<group type=\"GroupParam\" default_child_type=\"Param\">"
			"		<pqt default=\"{0.42,0.0,0.55,0,0,0,1}\" abbreviation=\"p\"/>"
            "		<kp_p default=\"{10,12,20,3,4,3}\"/>"
            "		<kp_v default=\"{200,360,120,80,40,20}\"/>"
            "		<ki_v default=\"{2,18,20,0.4,0.3,0.4}\"/>"
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
			"</moveJF>");
	}


	// 力控跟随——末端pq由MoveSetPQ给定，最后三个轴保持不动——速度前馈；电流控制 //
	struct MoveJFBParam
	{
		std::vector<double> kp_p;
		std::vector<double> kp_v;
		std::vector<double> ki_v;

		std::vector<double> pqt;
		std::vector<double> pqa;
		std::vector<double> pt;
		std::vector<double> pa;
		std::vector<double> vt;
		std::vector<double> va;
		std::vector<double> vfwd;

		std::vector<double> ft;
	};
	static std::atomic_bool enable_moveJFB = true;
	static std::atomic<std::array<double, 7> > setpqJFB;
	auto MoveJFB::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		auto c = dynamic_cast<aris::control::Controller*>(target.master);
		MoveJFBParam param;
		enable_moveJFB = true;
		param.kp_p.resize(6, 0.0);
		param.kp_v.resize(6, 0.0);
		param.ki_v.resize(6, 0.0);

		param.pqt.resize(7, 0.0);
		param.pqa.resize(7, 0.0);
		param.pt.resize(6, 0.0);
		param.pa.resize(6, 0.0);
		param.vt.resize(6, 0.0);
		param.va.resize(6, 0.0);
		param.vfwd.resize(6, 0.0);
		param.ft.resize(6, 0.0);

		for (auto &p : params)
		{
			if (p.first == "pqt")
			{
				auto pqarray = target.model->calculator().calculateExpression(p.second);
				param.pqt.assign(pqarray.begin(), pqarray.end());
				std::array<double, 7> temp;
				std::copy(pqarray.begin(), pqarray.end(), temp.begin());
				setpqJFB.store(temp);
			}
			else if (p.first == "kp_p")
			{
				auto v = target.model->calculator().calculateExpression(p.second);
				if (v.size() == 1)
				{
					std::fill(param.kp_p.begin(), param.kp_p.end(), v.toDouble());
				}
				else if (v.size() == param.kp_p.size())
				{
					param.kp_p.assign(v.begin(), v.end());
				}
				else
				{
					throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
				}

				for (Size i = 0; i < param.kp_p.size(); ++i)
				{
					if (param.kp_p[i] > 10000.0)
					{
						param.kp_p[i] = 10000.0;
					}
					if (param.kp_p[i] < 0.01)
					{
						param.kp_p[i] = 0.01;
					}
				}
			}
			else if (p.first == "kp_v")
			{
				auto a = target.model->calculator().calculateExpression(p.second);
				if (a.size() == 1)
				{
					std::fill(param.kp_v.begin(), param.kp_v.end(), a.toDouble());
				}
				else if (a.size() == param.kp_v.size())
				{
					param.kp_v.assign(a.begin(), a.end());
				}
				else
				{
					throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
				}

				for (Size i = 0; i < param.kp_v.size(); ++i)
				{
					if (param.kp_v[i] > 10000.0)
					{
						param.kp_v[i] = 10000.0;
					}
					if (param.kp_v[i] < 0.01)
					{
						param.kp_v[i] = 0.01;
					}
				}
			}
			else if (p.first == "ki_v")
			{
				auto d = target.model->calculator().calculateExpression(p.second);
				if (d.size() == 1)
				{
					std::fill(param.ki_v.begin(), param.ki_v.end(), d.toDouble());
				}
				else if (d.size() == param.ki_v.size())
				{
					param.ki_v.assign(d.begin(), d.end());
				}
				else
				{
					throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
				}

				for (Size i = 0; i < param.ki_v.size(); ++i)
				{
					if (param.ki_v[i] > 10000.0)
					{
						param.ki_v[i] = 10000.0;
					}
					if (param.ki_v[i] < 0.001)
					{
						param.ki_v[i] = 0.001;
					}
				}
			}

		}
		target.param = param;

		target.option |=
			Plan::USE_VEL_OFFSET |
			//#ifdef WIN32
			Plan::NOT_CHECK_POS_MIN |
			Plan::NOT_CHECK_POS_MAX |
			Plan::NOT_CHECK_POS_CONTINUOUS |
			Plan::NOT_CHECK_POS_CONTINUOUS_AT_START |
			Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
			Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START |
			Plan::NOT_CHECK_POS_FOLLOWING_ERROR |
			//#endif
			Plan::NOT_CHECK_VEL_MIN |
			Plan::NOT_CHECK_VEL_MAX |
			Plan::NOT_CHECK_VEL_CONTINUOUS |
			Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START |
			Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;

	}
	auto MoveJFB::executeRT(PlanTarget &target)->int
	{
		auto &param = std::any_cast<MoveJFBParam&>(target.param);
		auto controller = dynamic_cast<aris::control::Controller *>(target.master);
		static bool is_running{ true };
		static double vinteg[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
		static double vproportion[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
		bool ds_is_all_finished{ true };
		bool md_is_all_finished{ true };

		//第一个周期，将目标电机的控制模式切换到电流控制模式		
		if (target.count == 1)
		{
			is_running = true;

			for (Size i = 0; i < param.ft.size(); ++i)
			{
				controller->motionPool().at(i).setModeOfOperation(10);	//切换到电流控制
			}
		}

		//最后一个周期将目标电机去使能
		if (!enable_moveJFB)
		{
			is_running = false;
		}
		if (!is_running)
		{
			for (Size i = 0; i < param.ft.size(); ++i)
			{
				auto ret = controller->motionPool().at(i).disable();
				if (ret)
				{
					ds_is_all_finished = false;
				}
			}
		}

		//将目标电机由电流模式切换到位置模式
		if (!is_running&&ds_is_all_finished)
		{
			for (Size i = 0; i < param.ft.size(); ++i)
			{
				controller->motionPool().at(i).setModeOfOperation(8);
				auto ret = controller->motionPool().at(i).mode(8);
				if (ret)
				{
					md_is_all_finished = false;
				}
			}
		}

		//通过moveSPQ设置实时目标PQ位置
		std::array<double, 7> temp;
		temp = setpqJFB.load();
		std::copy(temp.begin(), temp.end(), param.pqt.begin());

		//求目标位置pq的运动学反解，获取电机实际位置、实际速度
		target.model->generalMotionPool().at(0).setMpq(param.pqt.data());
		if (!target.model->solverPool().at(0).kinPos())return -1;
		for (Size i = 0; i < param.pt.size(); ++i)
		{
			param.pt[i] = target.model->motionPool().at(i).mp();		//motionPool()指模型驱动器，at(0)表示第1个驱动器
			param.pa[i] = controller->motionPool().at(i).actualPos();
			param.va[i] = controller->motionPool().at(i).actualVel();
		}

		//模型运动学正解、动力学正解
		for (int i = 0; i < param.ft.size(); ++i)
		{
			target.model->motionPool()[i].setMp(controller->motionPool()[i].actualPos());
			target.model->motionPool().at(i).setMv(controller->motionAtAbs(i).actualVel());
			target.model->motionPool().at(i).setMa(0.0);
		}
		target.model->solverPool()[1].kinPos();
		target.model->solverPool()[1].kinVel();
		target.model->solverPool()[2].dynAccAndFce();

		//角度不变，位置变化
		target.model->generalMotionPool().at(0).getMpq(param.pqa.data());
		std::array<double, 4> q = {0.0,0.0,0.0,1.0};
		std::copy(q.begin(), q.end(), param.pqa.begin()+3);
		target.model->generalMotionPool().at(0).setMpq(param.pqa.data());
        if (!target.model->solverPool().at(0).kinPos())return -1;
		for (Size i = 3; i < param.pt.size(); ++i)
		{
			param.pt[i] = target.model->motionPool().at(i).mp();		//motionPool()指模型驱动器，at(0)表示第1个驱动器
			param.pa[i] = controller->motionPool().at(i).actualPos();
			param.va[i] = controller->motionPool().at(i).actualVel();
		}

		double ft_friction[6];
		double ft_offset[6];
		double real_vel[6];
		double ft_friction1[6], ft_friction2[6], ft_dynamic[6];
		static double ft_friction2_index[6] = { 5.0, 5.0, 5.0, 5.0, 5.0, 3.0 };
		if (is_running)
		{
			//位置环PID+速度限制
			for (Size i = 0; i < param.kp_p.size(); ++i)
			{
				param.vt[i] = param.kp_p[i] * (param.pt[i] - param.pa[i]);
				param.vt[i] = std::max(std::min(param.vt[i], vt_limit_JFB[i]), -vt_limit_JFB[i]);
				param.vt[i] = param.vt[i] + param.vfwd[i];
			}

			//速度环PID+力及力矩的限制
			for (Size i = 0; i < param.ft.size(); ++i)
			{
				vproportion[i] = param.kp_v[i] * (param.vt[i] - param.va[i]);
				vinteg[i] = vinteg[i] + param.ki_v[i] * (param.vt[i] - param.va[i]);
				vinteg[i] = std::min(vinteg[i], fi_limit_JFB[i]);
				vinteg[i] = std::max(vinteg[i], -fi_limit_JFB[i]);

				param.ft[i] = vproportion[i] + vinteg[i];
				param.ft[i] = std::min(param.ft[i], ft_limit_JFB[i]);
				param.ft[i] = std::max(param.ft[i], -ft_limit_JFB[i]);
			}

			//动力学载荷
			for (Size i = 0; i < param.ft.size(); ++i)
			{
				//动力学参数
				//constexpr double f_static[6] = { 9.349947583,11.64080253,4.770140543,3.631416685,2.58310847,1.783739862 };
				//constexpr double f_vel[6] = { 7.80825641,13.26518528,7.856443575,3.354615249,1.419632126,0.319206404 };
				//constexpr double f_acc[6] = { 0,3.555679326,0.344454603,0.148247716,0.048552673,0.033815455 };
				//constexpr double f2c_index[6] = { 9.07327526291993, 9.07327526291993, 17.5690184835913, 39.0310903520972, 66.3992503259041, 107.566785527965 };
				//constexpr double f_static_index[6] = {0.5, 0.5, 0.5, 0.85, 0.95, 0.8};

				//静摩擦力+动摩擦力=ft_friction

				real_vel[i] = std::max(std::min(max_static_vel[i], controller->motionAtAbs(i).actualVel()), -max_static_vel[i]);
				ft_friction1[i] = 0.8*(f_static[i] * real_vel[i] / max_static_vel[i]);

				//double ft_friction2_max = std::max(0.0, controller->motionAtAbs(i).actualVel() >= 0 ? f_static[i] - ft_friction1[i] : f_static[i] + ft_friction1[i]);
				//double ft_friction2_min = std::min(0.0, controller->motionAtAbs(i).actualVel() >= 0 ? -f_static[i] + ft_friction1[i] : -f_static[i] - ft_friction1[i]);
				//ft_friction2[i] = std::max(ft_friction2_min, std::min(ft_friction2_max, ft_friction2_index[i] * param.ft[i]));
				//ft_friction[i] = ft_friction1[i] + ft_friction2[i] + f_vel[i] * controller->motionAtAbs(i).actualVel();

				ft_friction[i] = ft_friction1[i] + f_vel[i] * controller->motionAtAbs(i).actualVel();

				//auto real_vel = std::max(std::min(max_static_vel[i], controller->motionAtAbs(i).actualVel()), -max_static_vel[i]);
				//ft_friction = (f_vel[i] * controller->motionAtAbs(i).actualVel() + f_static_index[i] * f_static[i] * real_vel / max_static_vel[i])*f2c_index[i];

				ft_friction[i] = std::max(-500.0, ft_friction[i]);
				ft_friction[i] = std::min(500.0, ft_friction[i]);

				//动力学载荷=ft_dynamic
				ft_dynamic[i] = target.model->motionPool()[i].mfDyn();

				ft_offset[i] = (ft_friction[i] + ft_dynamic[i] + param.ft[i])*f2c_index[i];
				controller->motionAtAbs(i).setTargetCur(ft_offset[i]);
			}
		}

		//print//
		auto &cout = controller->mout();
		if (target.count % 1000 == 0)
		{
			cout << "pt:";
			for (Size i = 0; i < 6; i++)
			{
				cout << std::setw(10) << param.pt[i] << "  ";
			}
			cout << std::endl;

			cout << "pa:";
			for (Size i = 0; i < 6; i++)
			{
				cout << std::setw(10) << param.pa[i] << "  ";
			}
			cout << std::endl;

			cout << "vt:";
			for (Size i = 0; i < 6; i++)
			{
				cout << std::setw(10) << param.vt[i] << "  ";
			}
			cout << std::endl;

			cout << "va:";
			for (Size i = 0; i < 6; i++)
			{
				cout << std::setw(10) << param.va[i] << "  ";
			}
			cout << std::endl;

			cout << "vproportion:";
			for (Size i = 0; i < 6; i++)
			{
				cout << std::setw(10) << vproportion[i] << "  ";
			}
			cout << std::endl;

			cout << "vinteg:";
			for (Size i = 0; i < 6; i++)
			{
				cout << std::setw(10) << vinteg[i] << "  ";
			}
			cout << std::endl;

			cout << "ft:";
			for (Size i = 0; i < 6; i++)
			{
				cout << std::setw(10) << param.ft[i] << "  ";
			}
			cout << std::endl;
			cout << "friction1:";
			for (Size i = 0; i < 6; i++)
			{
				cout << std::setw(10) << ft_friction1[i] << "  ";
			}
			cout << std::endl;
			cout << "friction:";
			for (Size i = 0; i < 6; i++)
			{
				cout << std::setw(10) << ft_friction[i] << "  ";
			}
			cout << std::endl;
			cout << "ft_dynamic:";
			for (Size i = 0; i < 6; i++)
			{
				cout << std::setw(10) << ft_dynamic[i] << "  ";
			}
			cout << std::endl;
			cout << "ft_offset:";
			for (Size i = 0; i < 6; i++)
			{
				cout << std::setw(10) << ft_offset[i] << "  ";
			}
			cout << std::endl;
			cout << "------------------------------------------------" << std::endl;
		}

		//log//
		auto &lout = controller->lout();
		for (Size i = 0; i < param.ft.size(); i++)
		{
			lout << param.pt[i] << ",";
			lout << controller->motionAtAbs(i).actualPos() << ",";
			lout << param.vt[i] << ",";
			lout << controller->motionAtAbs(i).actualVel() << ",";
			lout << ft_offset[i] << ",";
			lout << controller->motionAtAbs(i).actualCur() << ",";
			lout << vproportion[i] << ",";
			lout << vinteg[i] << ",";
			lout << param.ft[i] << ",";
			lout << ft_friction1[i] << ",";
			lout << ft_friction[i] << ",";
			lout << ft_dynamic[i] << ",";
		}
		lout << std::endl;

		return (!is_running&&ds_is_all_finished&&md_is_all_finished) ? 0 : 1;
	}
	auto MoveJFB::collectNrt(PlanTarget &target)->void {}
	MoveJFB::MoveJFB(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<moveJFB>"
			"	<group type=\"GroupParam\" default_child_type=\"Param\">"
			"		<pqt default=\"{0.42,0.0,0.55,0,0,0,1}\" abbreviation=\"p\"/>"
            "		<kp_p default=\"{8,12,20,3,3,2}\"/>"
            "		<kp_v default=\"{170,360,120,60,35,16}\"/>"
            "		<ki_v default=\"{2,18,20,0.2,0.2,0.18}\"/>"
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
			"</moveJFB>");
	}


	// 力控PID参数整定——输入单个或者全部轴空间位置，然后分别对各个轴空间进行PID整定；速度前馈；电流控制 //
	struct MoveJPIDParam
	{
		std::vector<bool> joint_active_vec;
		std::vector<double> kp_p;
		std::vector<double> kp_v;
		std::vector<double> ki_v;
		std::vector<double> kd_v;

		std::vector<double> pt;
		std::vector<double> pa;
		std::vector<double> vt;
		std::vector<double> va;

		std::vector<double> ft;
	};
	static std::atomic_bool enable_moveJPID = true;
	auto MoveJPID::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
		{
			auto c = dynamic_cast<aris::control::Controller*>(target.master);
			MoveJPIDParam param;
			enable_moveJPID = true;
			param.kp_p.resize(6, 0.0);
			param.kp_v.resize(6, 0.0);
			param.ki_v.resize(6, 0.0);
			param.kd_v.resize(6, 0.0);

			param.pa.resize(6, 0.0);
			param.vt.resize(6, 0.0);
			param.va.resize(6, 0.0);
			param.ft.resize(6, 0.0);

			for (auto &p : params)
			{
				if (p.first == "all")
				{
					param.joint_active_vec.resize(target.model->motionPool().size(), true);
				}
				else if (p.first == "none")
				{
					param.joint_active_vec.resize(target.model->motionPool().size(), false);
				}
				else if (p.first == "motion_id")
				{
					param.joint_active_vec.resize(target.model->motionPool().size(), false);
					param.joint_active_vec.at(std::stoi(p.second)) = true;
				}
				else if (p.first == "physical_id")
				{
					param.joint_active_vec.resize(c->motionPool().size(), false);
					param.joint_active_vec.at(c->motionAtPhy(std::stoi(p.second)).phyId()) = true;
				}
				else if (p.first == "slave_id")
				{
					param.joint_active_vec.resize(c->motionPool().size(), false);
					param.joint_active_vec.at(c->motionAtPhy(std::stoi(p.second)).slaId()) = true;
				}
				else if (p.first == "pos")
				{
					aris::core::Matrix mat = target.model->calculator().calculateExpression(p.second);
					if (mat.size() == 1)param.pt.resize(c->motionPool().size(), mat.toDouble());
					else
					{
						param.pt.resize(mat.size());
						std::copy(mat.begin(), mat.end(), param.pt.begin());
					}
				}
				else if (p.first == "kp_p")
				{
					auto v = target.model->calculator().calculateExpression(p.second);
					if (v.size() == 1)
					{
						std::fill(param.kp_p.begin(), param.kp_p.end(), v.toDouble());
					}
					else if (v.size() == param.kp_p.size())
					{
						param.kp_p.assign(v.begin(), v.end());
					}
					else
					{
						throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
					}

					for (Size i = 0; i < param.kp_p.size(); ++i)
					{
						if (param.kp_p[i] > 10000.0)
						{
							param.kp_p[i] = 10000.0;
						}
						if (param.kp_p[i] < 0.01)
						{
							param.kp_p[i] = 0.01;
						}
					}
				}
				else if (p.first == "kp_v")
				{
					auto a = target.model->calculator().calculateExpression(p.second);
					if (a.size() == 1)
					{
						std::fill(param.kp_v.begin(), param.kp_v.end(), a.toDouble());
					}
					else if (a.size() == param.kp_v.size())
					{
						param.kp_v.assign(a.begin(), a.end());
					}
					else
					{
						throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
					}

					for (Size i = 0; i < param.kp_v.size(); ++i)
					{
						if (param.kp_v[i] > 10000.0)
						{
							param.kp_v[i] = 10000.0;
						}
						if (param.kp_v[i] < 0.01)
						{
							param.kp_v[i] = 0.01;
						}
					}
				}
				else if (p.first == "ki_v")
				{
					auto d = target.model->calculator().calculateExpression(p.second);
					if (d.size() == 1)
					{
						std::fill(param.ki_v.begin(), param.ki_v.end(), d.toDouble());
					}
					else if (d.size() == param.ki_v.size())
					{
						param.ki_v.assign(d.begin(), d.end());
					}
					else
					{
						throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
					}

					for (Size i = 0; i < param.ki_v.size(); ++i)
					{
						if (param.ki_v[i] > 10000.0)
						{
							param.ki_v[i] = 10000.0;
						}
						if (param.ki_v[i] < 0.001)
						{
							param.ki_v[i] = 0.001;
						}
					}
				}
				else if (p.first == "kd_v")
				{
					auto d = target.model->calculator().calculateExpression(p.second);
					if (d.size() == 1)
					{
						std::fill(param.kd_v.begin(), param.kd_v.end(), d.toDouble());
					}
					else if (d.size() == param.kd_v.size())
					{
						param.kd_v.assign(d.begin(), d.end());
					}
					else
					{
						throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
					}

					for (Size i = 0; i < param.kd_v.size(); ++i)
					{
						if (param.kd_v[i] > 1000.0)
						{
							param.kd_v[i] = 1000.0;
						}
						if (param.kd_v[i] < 0.0)
						{
							param.kd_v[i] = 0.0;
						}
					}
				}
			}
			target.param = param;

			target.option |=
				Plan::USE_VEL_OFFSET |
				//#ifdef WIN32
				Plan::NOT_CHECK_POS_MIN |
				Plan::NOT_CHECK_POS_MAX |
				Plan::NOT_CHECK_POS_CONTINUOUS |
				Plan::NOT_CHECK_POS_CONTINUOUS_AT_START |
				Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
				Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START |
				Plan::NOT_CHECK_POS_FOLLOWING_ERROR |
				//#endif
				Plan::NOT_CHECK_VEL_MIN |
				Plan::NOT_CHECK_VEL_MAX |
				Plan::NOT_CHECK_VEL_CONTINUOUS |
				Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START |
				Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;

		}
	auto MoveJPID::executeRT(PlanTarget &target)->int
		{
			auto &param = std::any_cast<MoveJPIDParam&>(target.param);
			auto controller = dynamic_cast<aris::control::Controller *>(target.master);
			static bool is_running{ true };
			static double vproportion[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
			static double vinteg[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
			static double vdiff[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
			static double vt_va_last[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
			static double vt_va_this[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
			bool ds_is_all_finished{ true };
			bool md_is_all_finished{ true };

			//第一个周期，将目标电机的控制模式切换到电流控制模式		
			if (target.count == 1)
			{
				is_running = true;
				for (Size i = 0; i < param.joint_active_vec.size(); ++i)
				{
					if (param.joint_active_vec[i])
					{
						controller->motionPool().at(i).setModeOfOperation(10);	//切换到电流控制
					}
				}
			}

			//最后一个周期将目标电机去使能
			if (!enable_moveJPID)
			{
				is_running = false;
			}
			if (!is_running)
			{
				for (Size i = 0; i < param.ft.size(); ++i)
				{
					auto ret = controller->motionPool().at(i).disable();
					if (ret)
					{
						ds_is_all_finished = false;
					}
				}
			}

			//将目标电机由电流模式切换到位置模式
			if (!is_running&&ds_is_all_finished)
			{
				for (Size i = 0; i < param.ft.size(); ++i)
				{
					controller->motionPool().at(i).setModeOfOperation(8);
					auto ret = controller->motionPool().at(i).mode(8);
					if (ret)
					{
						md_is_all_finished = false;
					}
				}
			}

			//获取电机实际位置、实际速度
			for (Size i = 0; i < param.joint_active_vec.size(); ++i)
			{
				if (param.joint_active_vec[i])
				{
					param.pa[i] = controller->motionPool().at(i).actualPos();	//motionPool()指模型驱动器，at(0)表示第1个驱动器
					param.va[i] = controller->motionPool().at(i).actualVel();
				}
			}

			//模型正解
			for (int i = 0; i < param.ft.size(); ++i)
			{
				target.model->motionPool()[i].setMp(controller->motionPool()[i].actualPos());
				target.model->motionPool().at(i).setMv(controller->motionAtAbs(i).actualVel());
				target.model->motionPool().at(i).setMa(0.0);
			}
			target.model->solverPool()[1].kinPos();
			target.model->solverPool()[1].kinVel();
			target.model->solverPool()[2].dynAccAndFce();

			double ft_friction[6];
			double ft_offset[6];
			double real_vel[6];
			double ft_friction1[6], ft_friction2[6], ft_dynamic[6];
			static double ft_friction2_index[6] = { 5.0, 5.0, 5.0, 5.0, 5.0, 3.0 };
			if (is_running)
			{
				//位置环PID+速度限制
				for (Size i = 0; i < param.joint_active_vec.size(); ++i)
				{
					if (param.joint_active_vec[i])
					{
						param.vt[i] = param.kp_p[i] * (param.pt[i] - param.pa[i]);
						param.vt[i] = std::max(std::min(param.vt[i], vt_limit[i]), -vt_limit[i]);
					}
				}

				//速度环PID+力及力矩的限制
				for (Size i = 0; i < param.joint_active_vec.size(); ++i)
				{
					if (param.joint_active_vec[i])
					{
						vproportion[i] = param.kp_v[i] * (param.vt[i] - param.va[i]);

						vinteg[i] = vinteg[i] + param.ki_v[i] * (param.vt[i] - param.va[i]);
						vinteg[i] = std::min(vinteg[i], fi_limit[i]);
						vinteg[i] = std::max(vinteg[i], -fi_limit[i]);

						vt_va_this[i] = param.vt[i] - param.va[i];
						vdiff[i] = param.kd_v[i] * (vt_va_this[i] - vt_va_last[i]);
						vt_va_last[i] = param.vt[i] - param.va[i];

						param.ft[i] = vproportion[i] + vinteg[i] + vdiff[i];
					}
				}

				//动力学载荷
				for (Size i = 0; i < param.joint_active_vec.size(); ++i)
				{
					if (param.joint_active_vec[i])
					{
						//动力学参数
						//constexpr double f_static[6] = { 9.349947583,11.64080253,4.770140543,3.631416685,2.58310847,1.783739862 };
						//constexpr double f_vel[6] = { 7.80825641,13.26518528,7.856443575,3.354615249,1.419632126,0.319206404 };
						//constexpr double f_acc[6] = { 0,3.555679326,0.344454603,0.148247716,0.048552673,0.033815455 };
						//constexpr double f2c_index[6] = { 9.07327526291993, 9.07327526291993, 17.5690184835913, 39.0310903520972, 66.3992503259041, 107.566785527965 };
						//constexpr double f_static_index[6] = {0.5, 0.5, 0.5, 0.85, 0.95, 0.8};

						//静摩擦力+动摩擦力=ft_friction

						real_vel[i] = std::max(std::min(max_static_vel[i], controller->motionAtAbs(i).actualVel()), -max_static_vel[i]);
						ft_friction1[i] = 0.8*(f_static[i] * real_vel[i] / max_static_vel[i]);

						//double ft_friction2_max = std::max(0.0, controller->motionAtAbs(i).actualVel() >= 0 ? f_static[i] - ft_friction1[i] : f_static[i] + ft_friction1[i]);
						//double ft_friction2_min = std::min(0.0, controller->motionAtAbs(i).actualVel() >= 0 ? -f_static[i] + ft_friction1[i] : -f_static[i] - ft_friction1[i]);
						//ft_friction2[i] = std::max(ft_friction2_min, std::min(ft_friction2_max, ft_friction2_index[i] * param.ft[i]));
						//ft_friction[i] = ft_friction1[i] + ft_friction2[i] + f_vel[i] * controller->motionAtAbs(i).actualVel();

						ft_friction[i] = ft_friction1[i] + f_vel[i] * controller->motionAtAbs(i).actualVel();

						//auto real_vel = std::max(std::min(max_static_vel[i], controller->motionAtAbs(i).actualVel()), -max_static_vel[i]);
						//ft_friction = (f_vel[i] * controller->motionAtAbs(i).actualVel() + f_static_index[i] * f_static[i] * real_vel / max_static_vel[i])*f2c_index[i];

						ft_friction[i] = std::max(-500.0, ft_friction[i]);
						ft_friction[i] = std::min(500.0, ft_friction[i]);

						//动力学载荷=ft_dynamic
						ft_dynamic[i] = target.model->motionPool()[i].mfDyn();

						ft_offset[i] = (ft_friction[i] + ft_dynamic[i] + param.ft[i])*f2c_index[i];
						controller->motionAtAbs(i).setTargetCur(ft_offset[i]);
					}
				}
			}

			//print//
			auto &cout = controller->mout();
			if (target.count % 1000 == 0)
			{
				cout << "pt:";
				for (Size i = 0; i < 6; i++)
				{
					if (param.joint_active_vec[i])
					{
						cout << std::setw(10) << param.pt[i] << "  ";
					}
				}
				cout << std::endl;

				cout << "pa:";
				for (Size i = 0; i < 6; i++)
				{
					if (param.joint_active_vec[i])
					{
						cout << std::setw(10) << param.pa[i] << "  ";
					}
				}
				cout << std::endl;

				cout << "vt:";
				for (Size i = 0; i < 6; i++)
				{
					if (param.joint_active_vec[i])
					{
						cout << std::setw(10) << param.vt[i] << "  ";
					}
				}
				cout << std::endl;

				cout << "va:";
				for (Size i = 0; i < 6; i++)
				{
					if (param.joint_active_vec[i])
					{
						cout << std::setw(10) << param.va[i] << "  ";
					}
				}
				cout << std::endl;

				cout << "vproportion:";
				for (Size i = 0; i < 6; i++)
				{
					if (param.joint_active_vec[i])
					{
						cout << std::setw(10) << vproportion[i] << "  ";
					}
				}
				cout << std::endl;

				cout << "vinteg:";
				for (Size i = 0; i < 6; i++)
				{
					if (param.joint_active_vec[i])
					{
						cout << std::setw(10) << vinteg[i] << "  ";
					}
				}
				cout << std::endl;

				cout << "vdiff:";
				for (Size i = 0; i < 6; i++)
				{
					if (param.joint_active_vec[i])
					{
						cout << std::setw(10) << vdiff[i] << "  ";
						cout << std::setw(10) << param.kd_v[i] << "  ";

					}
				}
				cout << std::endl;

				cout << "ft:";
				for (Size i = 0; i < 6; i++)
				{
					if (param.joint_active_vec[i])
					{
						cout << std::setw(10) << param.ft[i] << "  ";
					}
				}
				cout << std::endl;
				cout << "friction1:";
				for (Size i = 0; i < 6; i++)
				{
					if (param.joint_active_vec[i])
					{
						cout << std::setw(10) << ft_friction1[i] << "  ";
					}
				}
				cout << std::endl;
				cout << "friction:";
				for (Size i = 0; i < 6; i++)
				{
					if (param.joint_active_vec[i])
					{
						cout << std::setw(10) << ft_friction[i] << "  ";
					}
				}
				cout << std::endl;
				cout << "ft_dynamic:";
				for (Size i = 0; i < 6; i++)
				{
					if (param.joint_active_vec[i])
					{
						cout << std::setw(10) << ft_dynamic[i] << "  ";
					}
				}
				cout << std::endl;
				cout << "ft_offset:";
				for (Size i = 0; i < 6; i++)
				{
					if (param.joint_active_vec[i])
					{
						cout << std::setw(10) << ft_offset[i] << "  ";
					}
				}
				cout << std::endl;

				cout << "------------------------------------------------" << std::endl;
			}

			// log //
			auto &lout = controller->lout();
			for (Size i = 0; i < param.ft.size(); i++)
			{
				lout << param.pt[i] << ",";
				lout << controller->motionAtAbs(i).actualPos() << ",";
				lout << param.vt[i] << ",";
				lout << controller->motionAtAbs(i).actualVel() << ",";
				lout << ft_offset[i] << ",";
				lout << controller->motionAtAbs(i).actualCur() << ",";
				lout << vproportion[i] << ",";
				lout << vinteg[i] << ",";
				lout << vdiff[i] << ",";
				lout << param.ft[i] << ",";
				lout << ft_friction1[i] << ",";
				lout << ft_friction[i] << ",";
				lout << ft_dynamic[i] << ",";
			}
			lout << std::endl;

			return (!is_running&&ds_is_all_finished&&md_is_all_finished) ? 0 : 1;
		}
	auto MoveJPID::collectNrt(PlanTarget &target)->void {}
	MoveJPID::MoveJPID(const std::string &name) :Plan(name)
		{
			command().loadXmlStr(
				"<moveJPID>"
				"	<group type=\"GroupParam\" default_child_type=\"Param\">"
				"		<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"all\">"
				"			<all abbreviation=\"a\"/>"
				"			<motion_id abbreviation=\"m\" default=\"0\"/>"
				"			<physical_id abbreviation=\"p\" default=\"0\"/>"
				"			<slave_id abbreviation=\"s\" default=\"0\"/>"
				"		</unique>"
				"		<pos default=\"0.0\"/>"
				"		<kp_p default=\"{10,12,70,5,6,3}\"/>"
				"		<kp_v default=\"{270,360,120,120,60,25}\"/>"
				"		<ki_v default=\"{2,18,20.0,0.6,0.5,0.5}\"/>"
				"		<kd_v default=\"0\"/>"
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
				"</moveJPID>");
		}


	// 力控停止指令——停止MoveStop，去使能电机 //
	auto MoveStop::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
		{
			enable_moveJRC = false;
			enable_movePQCrash = false;
			enable_movePQB = false;
			enable_moveJCrash = false;
			enable_moveJF = false;
			enable_moveJFB = false;
			enable_moveJPID = false;
			target.option = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;
		}
	MoveStop::MoveStop(const std::string &name) :Plan(name)
		{
			command().loadXmlStr(
				"<moveStop>"
				"</moveStop>");
		}
	

	// 力控跟随指令——实时给定末端pq值 //
	auto MoveSPQ::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		for (auto &p : params)
		{
			if (p.first == "setpqJF")
			{
				auto pqarray = target.model->calculator().calculateExpression(p.second);
				std::array<double, 7> temp;
				std::copy(pqarray.begin(), pqarray.end(), temp.begin());
				setpqJF.store(temp);
			}
			else if (p.first == "setpqJFB")
			{
				auto pqarray = target.model->calculator().calculateExpression(p.second);
				std::array<double, 7> temp;
				std::copy(pqarray.begin(), pqarray.end(), temp.begin());
				setpqJFB.store(temp);
			}
			else if (p.first == "setpqPQB")
			{
				auto pqarray = target.model->calculator().calculateExpression(p.second);
				std::array<double, 7> temp;
				std::copy(pqarray.begin(), pqarray.end(), temp.begin());
				setpqPQB.store(temp);
                which_func == 1;
                func[1] = load_pq1;
			}
			else if (p.first == "which_func")
			{
				which_func = std::stoi(p.second);
				if (which_func == 1)
				{
					func[1] = load_pq1;
					one_time_counter = false;
				}
				else if (which_func == 2)
				{
					func[1] = load_pq2;
					one_time_counter = true;
				}
				else if (which_func == 3)
				{
					func[1] = load_pq3;
					one_time_counter = true;
				}
			}
		}
		target.option = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;
	}
	MoveSPQ::MoveSPQ(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<moveSPQ>"
			"	<group type=\"GroupParam\" default_child_type=\"Param\">"
			"		<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"setpqJFB\">"
			"			<setpqJF default=\"{0.42,0.0,0.55,0,0,0,1}\"/>"
			"			<setpqJFB default=\"{0.42,0.0,0.55,0,0,0,1}\"/>"
			"			<setpqPQB default=\"{0.42,0.0,0.55,0,0,0,1}\"/>"
			"			<which_func default=\"1\"/>"
			"		</unique>"
			"	</group>"
			"</moveSPQ>");
	}

}
