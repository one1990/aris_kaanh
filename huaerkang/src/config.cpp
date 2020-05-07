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
 
        for (aris::Size i = 0; i < 2; ++i)
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
                243.0*2048/2/PI, -243.0*2048/2/PI, 243.0*2048/2/PI, 131072.0 * 101 / 2 / PI
            };
			//关节最大位置，角度单位转换成弧度单位
            double max_pos[4]
            {
                170000.0 / 360 * 2 * PI, 170000.0 / 360 * 2 * PI,	170000.0 / 360 * 2 * PI,  150.0 / 360 * 2 * PI
            };
//            double max_pos[4]
//            {
//                170000.0 / 360 * 2 * PI, 400000.0 / 360 * 2 * PI,	1500000.0 / 360 * 2 * PI,  150.0 / 360 * 2 * PI
//            };
			//关节最小位置，角度单位转换成弧度单位
            double min_pos[4]
            {
                -170000.0 / 360 * 2 * PI, -170000.0 / 360 * 2 * PI, -170000.0 / 360 * 2 * PI, -125.0 / 360 * 2 * PI
            };
			//关节最大速度，角度单位转换成弧度单位
//            double max_vel[4]
//            {
//                230.0 / 360 * 2 * PI, 300.0 / 360 * 2 * PI, 300.0 / 360 * 2 * PI, 300.0 / 360 * 2 * PI
//            };
            double max_vel[4]
            {
                300.0 / 360 * 2 * PI, 300.0 / 360 * 2 * PI, 300.0 / 360 * 2 * PI, 300.0 / 360 * 2 * PI
            };
			//关节最大加速度，角度单位转换成弧度单位
//            double max_acc[4]
//            {
//                1150.0 / 360 * 2 * PI, 1500.0 / 360 * 2 * PI, 1500.0 / 360 * 2 * PI, 1500.0 / 360 * 2 * PI
//            };
            double max_acc[4]
            {
                1500.0 / 360 * 2 * PI, 1500.0 / 360 * 2 * PI, 1500.0 / 360 * 2 * PI, 1500.0 / 360 * 2 * PI
            };

            //根据从站的ESI文件，以及上述参数配置从站信息，格式是xml格式
            std::string xml_str =
                "<EthercatMotor phy_id=\"" + std::to_string(i) + "\" product_code=\"0x60500000\""
                " vendor_id=\"251\" revision_num=\"0x01500000\" dc_assign_activate=\"0x0300\" sync0_shift_ns=\"800000\""
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
	
    double g_count = 0.0;
    std::atomic_int g_vel_percent = 100;

    template<typename MoveType>
    auto updatecount(MoveType *plan)->void
    {
        if (plan->count() == 1)
        {
            g_count = 0.0;
        }
        //渐变调速
        static double g_vel_percent_last = g_vel_percent.load();
        static double g_vel_percent_now = g_vel_percent.load();
        g_vel_percent_now = g_vel_percent.load();
        if (g_vel_percent_now - g_vel_percent_last >= 0.5)
        {
            g_vel_percent_last = g_vel_percent_last + 0.5;
        }
        else if (g_vel_percent_now - g_vel_percent_last <= -0.5 )
        {
            g_vel_percent_last = g_vel_percent_last - 0.5;
        }
        else
        {
            g_vel_percent_last = g_vel_percent_now;
        }

        g_count = g_count + 2*g_vel_percent_last / 100.0;
    }

    //设置全局速度//
    struct GVelParam
    {
        int vel_percent;
    };
    auto GVel::prepareNrt()->void
    {
        GVelParam param;
        for (auto &p : cmdParams())
        {
            if (p.first == "vel_percent")
            {
                param.vel_percent = int32Param(p.first);
            }
        }
        g_vel_percent.store(param.vel_percent);

        std::vector<std::pair<std::string, std::any>> ret_value;
        ret() = ret_value;
        option() = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;
    }
    GVel::GVel(const std::string &name) :Plan(name)
    {
        command().loadXmlStr(
            "<Command name=\"gvel\">"
            "	<GroupParam>"
            "		<Param name=\"vel_percent\" abbreviation=\"p\" default=\"0\"/>"
            "	</GroupParam>"
            "</Command>");
    }


	//MoveAbs的指令参数结构体，长度单位是m，角度单位是rad
	//每条指令的执行顺序
	//1、先执行prepareNrt，每条指令只执行一次
	//2、然后执行executeRT,executeRT每一个循环周期(默认1ms)会被实时核调用一次，执行的总时间由用户给定
	//3、执行结束后，本指令会被析构
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
                    std::cout << "param.axis_vel_vec[i]:" << param.axis_vel_vec[i] << "maxvel:"<< controller()->motionPool()[i].maxVel() << std::endl;
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
            controller()->logFileRawName("motion_replay");//log name

            for (Size i = 0; i < param->active_motor.size(); ++i)
			{
				if (param->active_motor[i])
				{
					param->axis_begin_pos_vec[i] = controller()->motionPool()[i].targetPos();


				}
			}
		}

        updatecount(this);
		aris::Size total_count{ 1 };
        double p, v, a, j;
		for (Size i = 0; i < param->active_motor.size(); ++i)
		{
			if (param->active_motor[i])
			{

//                Size t_count;
//				//梯形轨迹规划
//                aris::plan::moveAbsolute(g_count,
//                    param->axis_begin_pos_vec[i], param->axis_pos_vec[i],
//                    param->axis_vel_vec[i] / 1000, param->axis_acc_vec[i] / 1000 / 1000, param->axis_dec_vec[i] / 1000 / 1000,
//                    p, v, a, t_count);

                //s形规划//
                double t_count;
                traplan::sCurved(g_count, param->axis_begin_pos_vec[i], param->axis_pos_vec[i],
                    param->axis_vel_vec[i] / 1000.0, param->axis_acc_vec[i] / 1000.0 / 1000.0, param->axis_jerk_vec[i] / 1000.0 / 1000.0 / 1000.0,
                    p, v, a, j, t_count);
                controller()->motionPool()[i].setTargetPos(p);
                //controller()->motionPool()[i].setTargetVel(v*500);
                total_count = std::max(total_count, aris::Size(t_count));
			}
		}

        auto &cout = controller()->mout();
        if(count()%100 ==0)
        {
            for(int i=0; i<param->active_motor.size();i++)
            {
                cout << "targetpos:" << controller()->motionPool()[i].targetPos()<<"  ";
                cout << "actualpos:" << controller()->motionPool()[i].actualPos()<<"  ";
                cout << "targetvel:" << 1000.0*v <<"  ";
                cout << "actualvel:" << controller()->motionPool()[i].actualVel()<<"  ";

            }
            cout << std::endl;
        }

        //for(int i=0; i<param->active_motor.size();i++)
        //{
        //   lout() <<std::setprecision(10) << controller()->motionPool()[i].targetPos()<<"  ";
        //    lout() << controller()->motionPool()[i].actualPos()<<"  ";
        //}
        //controller()->lout() << std::endl;
		//返回0表示正常结束，返回负数表示报错，返回正数表示正在执行
        return (total_count%2==1)?(total_count+1 - int(g_count)):(total_count - int(g_count));
	}
	auto MoveAbs::collectNrt()->void {}
	MoveAbs::~MoveAbs() = default;
	MoveAbs::MoveAbs(const std::string &name) : Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"moveabs\">"
			"	<GroupParam>"
			"		<Param name=\"pos\" default=\"0.0\"/>"
            "		<Param name=\"vel\" default=\"0.5\"/>"
            "		<Param name=\"acc\" default=\"10\"/>"
            "		<Param name=\"dec\" default=\"10\"/>"
            "		<Param name=\"jerk\" default=\"100.0\"/>"
			"		<UniqueParam default=\"all\">"\
			"			<Param name=\"all\" abbreviation=\"a\"/>"\
			"			<Param name=\"motion_id\" abbreviation=\"m\" default=\"0\"/>"
			"		</UniqueParam>"
			"	</GroupParam>"
			"</Command>");
	}
	

	//MoveLine的指令参数结构体，长度单位是m，角度单位是rad
	//每条指令的执行顺序
	//1、先执行prepareNrt，每条指令只执行一次
	//2、然后执行executeRT,executeRT每一个循环周期(默认1ms)会被实时核调用一次，执行的总时间由用户给定
	//3、执行结束后，本指令会被析构
	struct MoveLineParam
	{
		std::vector<double> ee;					//目标姿态+外部轴角度
		std::string eul_type;					//欧拉角类型，默认是321
		double ori_theta;						//姿态规划角
		double vel;								//姿态速度
		double acc;								//姿态加速度
		double dec;								//姿态加速度
		double jerk;							//姿态跃度
		std::vector<double> ext_pos_begin;		//外部轴起始位置
		std::vector<double> ext_pos;			//外部轴目标位置
		std::vector<double> ext_vel;			//外部轴速度
		std::vector<double> ext_acc;			//外部轴加速度
		std::vector<double> ext_dec;			//外部轴减速度
		std::vector<double> ext_jerk;			//外部轴跃度
		Size max_total_count;					//总规划数
		double pos_ratio;						//外部轴规划系数
		double ori_ratio;						//姿态规划系数
	};
	auto MoveLine::prepareNrt()->void
	{
		MoveLineParam param;
		param.ext_pos_begin.resize(controller()->motionPool().size() - model()->motionPool().size(), 0.0);

		//解析指令参数，单位为m或rad
		for (auto cmd_param : cmdParams())
		{
			if (cmd_param.first == "ee")
			{
				auto pe_mat = matrixParam(cmd_param.first);
				if (pe_mat.size() != controller()->motionPool().size())THROW_FILE_LINE("Input ee out of range");
				param.ee.assign(pe_mat.begin(), pe_mat.begin() + 3);
				param.ext_pos.assign(pe_mat.begin() + 3, pe_mat.end());
			}
			else if (cmd_param.first == "eul_type")
			{
				param.eul_type = std::string(cmd_param.second);
				if (!kaanh::check_eul_validity(param.eul_type))THROW_FILE_LINE("Input eul_type error");
			}
			else if (cmd_param.first == "acc")
			{
				param.acc = doubleParam(cmd_param.first);

				//线加速度的最大最小值需要用户自己设定，这里默认设置为50
				if (param.acc > 50 || param.acc < -50)
					THROW_FILE_LINE("input acc beyond range");


			}
			else if (cmd_param.first == "vel")
			{
				param.vel = doubleParam(cmd_param.first);

				//线加速度的最大最小值需要用户自己设定，这里默认设置为10
				if (param.vel > 10 || param.vel < -10)
					THROW_FILE_LINE("input vel beyond range");
			}
			else if (cmd_param.first == "dec")
			{
				param.dec = doubleParam(cmd_param.first);

				//线加速度的最大最小值需要用户自己设定
				if (param.dec > 50 || param.dec < -50)
					THROW_FILE_LINE("input dec beyond range");
			}
			else if (cmd_param.first == "jerk")
			{
				param.jerk = doubleParam(cmd_param.first);

				//线加速度的最大最小值需要用户自己设定
				if (param.jerk > 100 || param.jerk < -100)
					THROW_FILE_LINE("input jerk beyond range");
			}
			else if (cmd_param.first == "ext_acc")
			{
				auto a = matrixParam(cmd_param.first);
				if (a.size() == controller()->motionPool().size() - model()->motionPool().size())
				{
					param.ext_acc.assign(a.begin(), a.end());
				}
				else
				{
					THROW_FILE_LINE("ext_acc input error");
				}
				for (Size i = 0; i < param.ext_acc.size(); ++i)
				{
					if (param.ext_acc[i] > controller()->motionPool()[i+ model()->motionPool().size()].maxAcc() || param.ext_acc[i] < controller()->motionPool()[i+ model()->motionPool().size()].minAcc())
						THROW_FILE_LINE("input acc beyond range");
				}
			}
			else if (cmd_param.first == "ext_vel")
			{
				auto v = matrixParam(cmd_param.first);

				if (v.size() == controller()->motionPool().size() - model()->motionPool().size())
				{
					param.ext_vel.assign(v.begin(), v.end());
				}
				else
				{
					THROW_FILE_LINE("ext_vel input error");
				}
				for (Size i = 0; i < param.ext_vel.size(); ++i)
				{
					if (param.ext_vel[i] > controller()->motionPool()[i + model()->motionPool().size()].maxVel() || param.ext_vel[i] < controller()->motionPool()[i + model()->motionPool().size()].minVel())
						THROW_FILE_LINE("input vel beyond range");
				}
			}
			else if (cmd_param.first == "ext_dec")
			{
				auto d = matrixParam(cmd_param.first);

				if (d.size() == controller()->motionPool().size() - model()->motionPool().size())
				{
					param.ext_dec.assign(d.begin(), d.end());
				}
				else
				{
					THROW_FILE_LINE("ext_dec input error");
				}
				for (Size i = 0; i < param.ext_dec.size(); ++i)
				{
					if (param.ext_dec[i] > controller()->motionPool()[i+ model()->motionPool().size()].maxAcc() || param.ext_dec[i] < controller()->motionPool()[i+ model()->motionPool().size()].minAcc())
						THROW_FILE_LINE("input dec beyond range");
				}
			}
			else if (cmd_param.first == "ext_jerk")//跃度值为加速度值的倍数,超过100倍加速度值，认为输入超限，可以也可以自己设置
			{
				auto d = matrixParam(cmd_param.first);
				if (d.size() == controller()->motionPool().size() - model()->motionPool().size())
				{
					param.ext_jerk.assign(d.begin(), d.end());
				}
				else
				{
					THROW_FILE_LINE("ext_jerk input error");
				}
				for (Size i = 0; i < param.ext_jerk.size(); ++i)
				{
					if (param.ext_jerk[i] > controller()->motionPool()[i + model()->motionPool().size()].maxAcc()*100.0 || param.ext_jerk[i] < controller()->motionPool()[i + model()->motionPool().size()].minAcc()*100.0)
						THROW_FILE_LINE("input jerk beyond range");
				}
			}

		}

		this->param() = param;
		for (auto &option : motorOptions())	option |= Plan::USE_TARGET_POS;
		std::vector<std::pair<std::string, std::any>> ret_value;
		ret() = ret_value;
	}
	auto MoveLine::executeRT()->int
	{
		auto param = std::any_cast<MoveLineParam>(&this->param());
		// 获得求解器，求反解 //
		auto &solver = dynamic_cast<aris::dynamic::Serial3InverseKinematicSolver&>(model()->solverPool()[0]);
		solver.setWhichRoot(4);	//设置解，一共4个，设为4时会选最优解
		double pq_begin[7];
		double pq_end[7];
		static double q_begin[4];
		static double q_end[4];
		double q_target[4];

		double p, v, a, j;
		Size ext_max_count{ 1 }, ori_total_count{ 1 }, ext_total_count{ 1 };

		if (count() == 1)
		{
            controller()->logFileRawName("motion_replay");//log name
			//获取外部轴的起始位置
			for (int i = 0; i < param->ext_pos_begin.size(); i++)
			{
				param->ext_pos_begin[i] = controller()->motionPool().at(i + model()->motionPool().size()).targetPos();
			}
			//获取起始姿态
			for (int i = 0; i < model()->motionPool().size(); i++)
			{
				model()->motionPool().at(i).setMp(controller()->motionPool().at(i).targetPos());
			}
			auto &forward = dynamic_cast<aris::dynamic::ForwardKinematicSolver&>(model()->solverPool()[1]);	// 获取正解求解器
			forward.kinPos();										//求正解
			model()->generalMotionPool().at(0).updMpm();			//更新模型
			model()->generalMotionPool().at(0).getMpq(pq_begin);	//获取起始时刻的末端位姿
			std::copy(pq_begin + 3, pq_begin + 7, q_begin);			//提取起始时刻的末端姿态

			//获取目标姿态
			double pe[6]{0,0,0,param->ee[0],param->ee[1],param->ee[2]};//根据目标姿态构造位置为0的位姿矩阵
			aris::dynamic::s_pe2pq(pe, pq_end, "321");						//获取目标姿态
			std::copy(pq_end + 3, pq_end + 7, q_end);				//提取目标姿态

			//计算姿态规划角
			kaanh::cal_ori_theta(q_begin, q_end, param->ori_theta);

			//姿态规划//
			traplan::sCurve(count(), 0.0, 1.0, param->vel / 1000.0 / param->ori_theta / 2.0 , param->acc / 1000.0 / 1000.0 / param->ori_theta / 2.0,
				param->jerk / 1000.0 / 1000.0 / 1000.0 / param->ori_theta / 2.0, p, v, a, j, ori_total_count);
			//外部轴规划
			for (int i = 0; i < param->ext_pos_begin.size(); i++)
			{
				traplan::sCurve(count(), param->ext_pos_begin[i], param->ext_pos[i], param->ext_vel[i] / 1000.0, param->ext_acc[i] / 1000.0 / 1000.0,
					param->ext_jerk[i] / 1000.0 / 1000.0 / 1000.0, p, v, a, j, ext_total_count);
				ext_max_count = std::max(ext_total_count, ext_max_count);
			}
			param->pos_ratio = ext_max_count < ori_total_count ? double(ext_max_count) / ori_total_count : 1.0;//外部轴规划系数
			param->ori_ratio = ori_total_count < ext_max_count ? double(ori_total_count) / ext_max_count : 1.0;//姿态规划系数

			//获取最大的规划count数
			//姿态带系数规划
			traplan::sCurve(count(), 0.0, 1.0, param->vel / 1000.0 / param->ori_theta / 2.0*param->ori_ratio, param->acc / 1000.0 / 1000.0 / param->ori_theta / 2.0*param->ori_ratio*param->ori_ratio,
				param->jerk / 1000.0 / 1000.0 / 1000.0 / param->ori_theta / 2.0*param->ori_ratio*param->ori_ratio*param->ori_ratio, p, v, a, j, ori_total_count);
			//外部轴带系数规划
			for (int i = 0; i < param->ext_pos_begin.size(); i++)
			{
				traplan::sCurve(count(), param->ext_pos_begin[i], param->ext_pos[i], param->ext_vel[i] / 1000.0*param->pos_ratio, param->ext_acc[i] / 1000.0 / 1000.0*param->pos_ratio*param->pos_ratio,
					param->ext_jerk[i] / 1000.0 / 1000.0 / 1000.0*param->pos_ratio*param->pos_ratio*param->pos_ratio, p, v, a, j, ext_total_count);
				ext_max_count = std::max(ext_total_count, ext_max_count);
			}
			param->max_total_count = std::max(ori_total_count, ext_max_count);
		}

        updatecount(this);
		//姿态规划
        traplan::sCurve(g_count, 0.0, 1.0, param->vel / 1000.0 / param->ori_theta / 2.0*param->ori_ratio, param->acc / 1000.0 / 1000.0 / param->ori_theta / 2.0*param->ori_ratio*param->ori_ratio,
			param->jerk / 1000.0 / 1000.0 / 1000.0 / param->ori_theta / 2.0*param->ori_ratio*param->ori_ratio*param->ori_ratio, p, v, a, j, ori_total_count);
		kaanh::slerp(q_begin, q_end, q_target, p);//求每个count规划的四元数
		solver.setQuaternionAngle(q_target);//发送四元数给model
		if (solver.kinPos())return -1;//求反解，模型会相应的给电机发指令

		//外部轴规划
		for (int i = 0; i < param->ext_pos_begin.size(); i++)
		{
            traplan::sCurve(g_count, param->ext_pos_begin[i], param->ext_pos[i], param->ext_vel[i] / 1000.0*param->pos_ratio, param->ext_acc[i] / 1000.0 / 1000.0*param->pos_ratio*param->pos_ratio,
				param->ext_jerk[i] / 1000.0 / 1000.0 / 1000.0*param->pos_ratio*param->pos_ratio*param->pos_ratio, p, v, a, j, ext_total_count);
			controller()->motionPool().at(i + model()->motionPool().size()).setTargetPos(p);
		}

		//返回0表示正常结束，返回负数表示报错，返回正数表示正在执行
        return (param->max_total_count%2==1)?(param->max_total_count+1 - int(g_count)):(param->max_total_count - int(g_count));

	}
	auto MoveLine::collectNrt()->void {}
	MoveLine::~MoveLine() = default;
	MoveLine::MoveLine(const std::string &name) : Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"moveline\">"
			"	<GroupParam>"
			"		<Param name=\"ee\" default=\"{0.0, 0.0, 0.0, 0.0}\"/>"
			"		<Param name=\"eul_type\" default=\"321\"/>"
			"		<Param name=\"vel\" default=\"1.0\"/>"
			"		<Param name=\"acc\" default=\"1.0\"/>"
			"		<Param name=\"dec\" default=\"1.0\"/>"
			"		<Param name=\"jerk\" default=\"10.0\"/>"
			"		<Param name=\"ext_vel\" default=\"1.0\"/>"
			"		<Param name=\"ext_acc\" default=\"1.0\"/>"
			"		<Param name=\"ext_dec\" default=\"1.0\"/>"
			"		<Param name=\"ext_jerk\" default=\"10.0\"/>"
			"	</GroupParam>"
			"</Command>");
	}


	struct MoveJParam	//MoveJ指令的参数结构体
	{
		std::vector<double> axis_vel_vec;		//关节速度
		std::vector<double> axis_acc_vec;		//关节加速度
		std::vector<double> axis_dec_vec;		//关节减速度
		std::vector<double> axis_jerk_vec;		//关节跃度
		std::vector<double> axis_begin_pos_vec;	//关节起始位置
		std::vector<double> axis_pos_vec;		//关节目标位置
		std::string eul_type;					//欧拉角类型，默认是321
		std::vector<double> ee;					//目标姿态+外部轴角度
		std::vector<Size> total_count;			//总规划count数
	};
	auto MoveJ::prepareNrt()->void
	{
		MoveJParam param;

		param.ee.resize(controller()->motionPool().size(), 0.0);//前3维是姿态，后面的是外部轴
		param.axis_begin_pos_vec.resize(controller()->motionPool().size(), 0.0);
		param.axis_pos_vec.resize(controller()->motionPool().size(), 0.0);
		param.total_count.resize(controller()->motionPool().size(), 0);//每个关节的规划count保存数组

		// 提取指令输入的数值姿态欧拉角ee及外部轴位置，欧拉角类型，关节速度、加速度、减速度、跃度
		for (auto cmd_param : cmdParams())
		{
			if (cmd_param.first == "ee")
			{
				auto pe_mat = matrixParam(cmd_param.first);
				if (pe_mat.size() != controller()->motionPool().size())THROW_FILE_LINE("Input ee out of range");
				param.ee.assign(pe_mat.begin(), pe_mat.end());
			}
			else if (cmd_param.first == "eul_type")
			{
				param.eul_type = std::string(cmd_param.second);
				if (!kaanh::check_eul_validity(param.eul_type))THROW_FILE_LINE("Input eul_type error");
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
			else if (cmd_param.first == "jerk")//跃度值为加速度值的倍数
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

		this->param() = param;//把参数传递给MoveJ类，方便后面引用
		std::vector<std::pair<std::string, std::any>> ret_value;//定义每条指令的返回值，用户可以按照这种格式添加
		ret() = ret_value;//将返回值添加到ret()函数，指令结束后，会将返回值传递给发送指令端(terminal或socket client或websocket端)
	}
	auto MoveJ::executeRT()->int
	{
		auto mvj_param = std::any_cast<MoveJParam>(&this->param());

		// 取得起始位置 //
		double p, v, a, j;
		static Size max_total_count;
		//在第一个周期，获取3个关节的起始角度值，同时根据目标姿态ee求反解，得出3个关节的目标角度值
		if (count() == 1)
		{
            controller()->logFileRawName("motion_replay");//log name
			// 获得求解器 //
			auto &solver = dynamic_cast<aris::dynamic::Serial3InverseKinematicSolver&>(model()->solverPool()[0]);
			solver.setEulaAngle(mvj_param->ee.data(), mvj_param->eul_type.c_str());

			// 求反解 //
			solver.kinPos();

			// 获取关节起始角度值，目标角度值 //
			for (Size i = 0; i < controller()->motionPool().size(); ++i)
			{
				mvj_param->axis_begin_pos_vec[i] = controller()->motionPool()[i].targetPos();//获取关节起始角度值
				if (i < model()->motionPool().size())
				{
					mvj_param->axis_pos_vec[i] = model()->motionPool()[i].mp();//根据model求反解结果得到对应3个关节目标角度值
				}
				else
				{
					mvj_param->axis_pos_vec[i] = mvj_param->ee[i];//获取外部轴的目标角度值
				}			
				
				//S形轨迹规划//
                traplan::sCurve(count(), mvj_param->axis_begin_pos_vec[i], mvj_param->axis_pos_vec[i],
					mvj_param->axis_vel_vec[i] / 1000, mvj_param->axis_acc_vec[i] / 1000 / 1000, mvj_param->axis_jerk_vec[i] / 1000 / 1000 / 1000,
					p, v, a, j, mvj_param->total_count[i]);
			}

			//第一个周期就可以得到每个关节规划的总时间，这里取最大值是为了保证后续每个周期的同步
			max_total_count = *std::max_element(mvj_param->total_count.begin(), mvj_param->total_count.end());
		}

        updatecount(this);

		//根据T型规划或者S型规划，控制每个关节运动
		for (Size i = 0; i < controller()->motionPool().size(); ++i)
		{
			//S形轨迹规划//
            traplan::sCurve(g_count * mvj_param->total_count[i] / max_total_count,
				mvj_param->axis_begin_pos_vec[i], mvj_param->axis_pos_vec[i],
				mvj_param->axis_vel_vec[i] / 1000, mvj_param->axis_acc_vec[i] / 1000 / 1000, mvj_param->axis_jerk_vec[i] / 1000 / 1000 / 1000,
				p, v, a, j, mvj_param->total_count[i]);

			controller()->motionPool()[i].setTargetPos(p);
		}

        if(max_total_count == 0)
        {
            return 0;
        }
        else
        {
            return (max_total_count%2==1)?(max_total_count+1 - int(g_count)):(max_total_count - int(g_count));
        }

	}
	auto MoveJ::collectNrt()->void{}
	MoveJ::~MoveJ() = default;
	MoveJ::MoveJ(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
            "<Command name=\"movejoint\">"
			"	<GroupParam>"
			"		<Param name=\"ee\" default=\"{0,0,0,0}\"/>"
			"		<Param name=\"eul_type\" default=\"321\"/>"
			"		<Param name=\"acc\" default=\"0.1\"/>"
			"		<Param name=\"vel\" default=\"0.1\"/>"
			"		<Param name=\"dec\" default=\"0.1\"/>"
			"		<Param name=\"jerk\" default=\"10.0\"/>"
			"	</GroupParam>"
			"</Command>");
	}


	//每一条用户开发的指令都要添加到如下planPool()中，格式参考其他指令
	auto createPlanRoot()->std::unique_ptr<aris::plan::PlanRoot>
	{
		std::unique_ptr<aris::plan::PlanRoot> plan_root(new aris::plan::PlanRoot);
		//for 华尔康
		plan_root->planPool().add<config::MoveAbs>();
		plan_root->planPool().add<config::MoveLine>();
		plan_root->planPool().add<config::MoveJ>();
        plan_root->planPool().add<config::GVel>();


		plan_root->planPool().add<aris::plan::Enable>();
		plan_root->planPool().add<aris::plan::Disable>();
		plan_root->planPool().add<aris::plan::Start>();
		plan_root->planPool().add<aris::plan::Stop>();
		plan_root->planPool().add<aris::plan::Mode>();
        plan_root->planPool().add<aris::plan::Show>();
		plan_root->planPool().add<aris::plan::Clear>();
        plan_root->planPool().add<kaanh::Recover>();
        plan_root->planPool().add<kaanh::Reset>();

        plan_root->planPool().add<kaanh::SetPdo>();
        plan_root->planPool().add<kaanh::Get>();
        plan_root->planPool().add<kaanh::SetVel>();
        plan_root->planPool().add<kaanh::JogJ1>();
        plan_root->planPool().add<kaanh::JogJ2>();
        plan_root->planPool().add<kaanh::JogJ3>();
        plan_root->planPool().add<kaanh::JogJ4>();
        plan_root->planPool().add<kaanh::JogJ5>();
        plan_root->planPool().add<kaanh::JogJ6>();
		return plan_root;
	}
}
