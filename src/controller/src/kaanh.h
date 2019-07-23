#ifndef KAANH_H_
#define KAANH_H_

#include <memory>
#include <aris.hpp>


// \brief 机器人命名空间
// \ingroup aris

namespace kaanh
{
	//其他参数和函数声明 
	using Size = std::size_t;
	constexpr double PI = 3.141592653589793;
	
	auto createControllerRokaeXB4()->std::unique_ptr<aris::control::Controller>;
	auto createModelRokae()->std::unique_ptr<aris::dynamic::Model>;
	auto createPlanRootRokaeXB4()->std::unique_ptr<aris::plan::PlanRoot>;
	

	class Get : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		explicit Get(const std::string &name = "Get_plan");
		ARIS_REGISTER_TYPE(Get);


	};

	class MoveJR : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		explicit MoveJR(const std::string &name = "MoveJR_plan");
		ARIS_REGISTER_TYPE(MoveJR);
	};

	class JogC : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		virtual ~JogC();
		explicit JogC(const std::string &name = "JogC_plan");
		ARIS_REGISTER_TYPE(JogC);
		JogC(const JogC &);
		JogC(JogC &);
		JogC& operator=(const JogC &);
		JogC& operator=(JogC &&);


	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};

	class JogJ : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		virtual ~JogJ();
		explicit JogJ(const std::string &name = "JogJ_plan");
		ARIS_REGISTER_TYPE(JogJ);
		JogJ(const JogJ &);
		JogJ(JogJ &);
		JogJ& operator=(const JogJ &);
		JogJ& operator=(JogJ &&);


	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};

}

#endif
