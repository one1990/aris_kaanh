#ifndef FORCECONTROL_H_
#define FORCECONTROL_H_


#include <memory>
#include <aris.hpp>
#include <atomic>
#include"cplan.h"


namespace forcecontrol
{
	//电缸力检测参数声明
	constexpr double ea_a = 3765.8, ea_b = 1334.8, ea_c = 45.624, ea_gra = 24, ea_index = -6, ea_gra_index = 36;  //电缸电流换算压力的系数，ea_k表示比例系数，ea_b表示截距，ea_offset表示重力影响量，ea_index表示电流扭矩系数=额定扭矩*6.28*减速比/导程/1000//

	//机器人力控参数声明
	constexpr double f_static[6] = { 9.349947583,11.64080253,4.770140543,3.631416685,2.58310847,1.783739862 };
    constexpr double f_vel_JRC[6] = { 7,11,6,2.5,0.5,0.319206404 };
    constexpr double f_vel[6] = { 7.80825641,13.26518528,7.856443575,3.354615249,1.419632126,0.319206404 };
    //constexpr double f_vel[6] = { 7.80825641,13.26518528,7.856443575,2.7,1.419632126,0.319206404 };
	constexpr double f_acc[6] = { 0,3.555679326,0.344454603,0.148247716,0.048552673,0.033815455 };
	constexpr double f2c_index[6] = { 9.07327526291993, 9.07327526291993, 17.5690184835913, 39.0310903520972, 66.3992503259041, 107.566785527965 };
	constexpr double max_static_vel[6] = { 0.12, 0.12, 0.1, 0.05, 0.05, 0.075 };
    constexpr double f_static_index_JRC[6] = { 1.0, 1.0, 0.8, 1.0, 1.2, 0.8 };
    constexpr double f_static_index[6] = { 0.5, 0.5, 0.5, 0.6, 0.95, 0.8 };
    constexpr double fi_limit[6] = { 15,15,10,10,5,5 };
    constexpr double vt_limit[6] = { 0.1,0.1,0.1,0.1,0.1,0.1 };
    constexpr double ft_limit[6] = { 15,15,10,10,5,5 };
	constexpr double mt_limit[6] = { 15,10,10,1,1,1 };

    constexpr double fi_limit_JFB[6] = { 33,18,10,10,5,5 };
    constexpr double vt_limit_JFB[6] = { 0.1,0.1,0.1,0.6,0.6,0.6 };
    constexpr double ft_limit_JFB[6] = { 33,18,10,15,10,10 };
	
    constexpr double fi_limit_PQB[6] = { 35,40,40,15,10,10 };
    constexpr double vt_limit_PQB[6] = { 0.2,0.2,0.09,0.6,0.6,0.6 };
    constexpr double vt_normv_limit = 0.3;
    constexpr double ft_limit_PQB[6] = { 35,40,40,15,10,10 };
	constexpr double ft_limit_PQB_P[6] = { 35,40,40,15,10,10 };

	//其他参数和函数声明 
	using Size = std::size_t;
	constexpr double PI = 3.141592653589793;

	class MoveJRC : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		explicit MoveJRC(const std::string &name = "MoveJRC_plan");
		ARIS_REGISTER_TYPE(MoveJRC);
	};

	class MovePQCrash : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		explicit MovePQCrash(const std::string &name = "MovePQCrash_plan");
		ARIS_REGISTER_TYPE(MovePQCrash);
	};

	class MovePQB : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		explicit MovePQB(const std::string &name = "MovePQB_plan");
		ARIS_REGISTER_TYPE(MovePQB);
	};

	class MoveJCrash : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		explicit MoveJCrash(const std::string &name = "MoveJCrash_plan");
		ARIS_REGISTER_TYPE(MoveJCrash);
	};

	class MoveJF : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		explicit MoveJF(const std::string &name = "MoveJF_plan");
		ARIS_REGISTER_TYPE(MoveJF);
	};

	class MoveJFB : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		explicit MoveJFB(const std::string &name = "MoveJFB_plan");
		ARIS_REGISTER_TYPE(MoveJFB);
	};

	class MoveJPID : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		explicit MoveJPID(const std::string &name = "MoveJPID_plan");
		ARIS_REGISTER_TYPE(MoveJPID);
	};

	class MoveEAC : public aris::plan::Plan
	{
	public:
		double tempforce;
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		explicit MoveEAC(const std::string &name = "MoveEAC_plan");
		ARIS_REGISTER_TYPE(MoveEAC);
	};

	class MoveStop : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		explicit MoveStop(const std::string &name = "MoveStop_plan");
		ARIS_REGISTER_TYPE(MoveStop);
	};

	class MoveSPQ : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		explicit MoveSPQ(const std::string &name = "MoveSPQ_plan");
		ARIS_REGISTER_TYPE(MoveSPQ);
	};
}

#endif
