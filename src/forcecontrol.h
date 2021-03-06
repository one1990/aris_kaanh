#ifndef FORCECONTROL_H_
#define FORCECONTROL_H_


#include <memory>
#include <aris_control.h>
#include <aris_dynamic.h>
#include <aris_plan.h>
#include <tinyxml2.h>
#include <atomic>

namespace forcecontrol
{
	//机器人力控参数声明
	constexpr double f_static[6] = { 9.349947583,11.64080253,4.770140543,3.631416685,2.58310847,1.783739862 };
    constexpr double f_vel[6] = { 7.80825641,13.26518528,7.856443575,3.354615249,1.419632126,0.319206404 };
    //constexpr double f_vel[6] = { 7.80825641,13.26518528,7.856443575,2.7,1.419632126,0.319206404 };
	constexpr double f_acc[6] = { 0,3.555679326,0.344454603,0.148247716,0.048552673,0.033815455 };
	constexpr double f2c_index[6] = { 9.07327526291993, 9.07327526291993, 17.5690184835913, 39.0310903520972, 66.3992503259041, 107.566785527965 };
	constexpr double max_static_vel[6] = { 0.12, 0.12, 0.1, 0.05, 0.05, 0.075 };
    constexpr double f_static_index[6] = { 0.5, 0.5, 0.5, 0.75, 0.95, 0.8 };
    constexpr double fi_limit[6] = { 15,15,10,10,5,5 };
    constexpr double vt_limit[6] = { 0.1,0.1,0.1,0.1,0.1,0.1 };
    constexpr double ft_limit[6] = { 15,15,10,10,5,5 };
	constexpr double mt_limit[6] = { 15,10,10,1,1,1 };

    constexpr double fi_limit_JFB[6] = { 18,11,9,10,5,5 };
    constexpr double vt_limit_JFB[6] = { 0.1,0.1,0.1,0.6,0.6,0.6 };
    constexpr double ft_limit_JFB[6] = { 18,11,9,15,10,10 };
	
    constexpr double fi_limit_PQB[6] = { 35,35,35,15,10,10 };
    constexpr double vt_limit_PQB[6] = { 0.12,0.12,0.12,0.6,0.6,0.6 };
    constexpr double ft_limit_PQB[6] = { 35,35,35,15,10,10 };

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
	};

	class MovePQCrash : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		explicit MovePQCrash(const std::string &name = "MovePQCrash_plan");
	};

	class MovePQB : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		explicit MovePQB(const std::string &name = "MovePQB_plan");
	};

	class MoveJCrash : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		explicit MoveJCrash(const std::string &name = "MoveJCrash_plan");
	};

	class MoveJF : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		explicit MoveJF(const std::string &name = "MoveJF_plan");
	};

	class MoveJFB : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		explicit MoveJFB(const std::string &name = "MoveJFB_plan");
	};

	class MoveJPID : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		explicit MoveJPID(const std::string &name = "MoveJPID_plan");
	};

	class MoveStop : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		explicit MoveStop(const std::string &name = "MoveStop_plan");
	};

	class MoveSPQ : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		explicit MoveSPQ(const std::string &name = "MoveSPQ_plan");
	};
}

#endif
