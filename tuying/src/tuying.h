#ifndef TUYING_H_
#define TUYING_H_

#include <memory>
#include <aris.hpp>
#include "kaanh.h"
#include "json.hpp"

namespace tuying
{
	using Size = std::size_t;
	constexpr double PI = 3.141592653589793;

	struct SpeedParam
	{
		double w_percent;	//关节速度百分比
		double v_tcp;	//TCP线速度mm/s
		double w_tcp;	//空间旋转速度°/s
		double w_ext;	//外部轴角速度°/s
		double v_ext;	//外部轴线速度mm/s
	};

	auto createControllerQifan()->std::unique_ptr<aris::control::Controller>;
	auto createModelQifan()->std::unique_ptr<aris::dynamic::Model>;

	auto createControllerRokaeXB4()->std::unique_ptr<aris::control::Controller>;
	auto createModelRokae()->std::unique_ptr<aris::dynamic::Model>;

	auto createPlanRootRokaeXB4()->std::unique_ptr<aris::plan::PlanRoot>;

	struct CmdListParam
	{
		std::vector<std::pair<std::string, std::string>> cmd_vec;
		int current_cmd_id = 0;
		int current_plan_id = -1;
	};

	class Enable : public aris::plan::Plan
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;

		virtual ~Enable();
		explicit Enable(const std::string &name = "enable_plan");
        ARIS_REGISTER_TYPE(tuying::Enable);
        ARIS_DECLARE_BIG_FOUR(Enable);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};

	class Disable : public aris::plan::Plan
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;

		virtual ~Disable();
		explicit Disable(const std::string &name = "enable_plan");
        ARIS_REGISTER_TYPE(tuying::Disable);
		ARIS_DECLARE_BIG_FOUR(Disable);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};

	class Get : public aris::plan::Plan
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual collectNrt()->void;

		explicit Get(const std::string &name = "Get_plan");
		ARIS_REGISTER_TYPE(Get);
	};

    class Getp : public aris::plan::Plan
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual collectNrt()->void;

        explicit Getp(const std::string &name = "Getp_plan");
        ARIS_REGISTER_TYPE(Getp);
    };

	class MoveT : public aris::plan::Plan
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;
		auto virtual collectNrt()->void;

		explicit MoveT(const std::string &name = "MoveT_plan");
		ARIS_REGISTER_TYPE(MoveT);
	};

	class MoveE0 : public aris::plan::Plan
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;
		auto virtual collectNrt()->void;

		explicit MoveE0(const std::string &name = "MoveE0_plan");
		ARIS_REGISTER_TYPE(MoveE0);
	};

	class MoveE : public aris::plan::Plan
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;
		auto virtual collectNrt()->void;

		explicit MoveE(const std::string &name = "MoveE_plan");
		ARIS_REGISTER_TYPE(MoveE);
	};

	class MoveAbJ : public aris::plan::Plan
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;
		auto virtual collectNrt()->void;

		explicit MoveAbJ(const std::string &name = "MoveAbJ_plan");
		ARIS_REGISTER_TYPE(MoveAbJ);
	};

	class MoveJM : public aris::plan::Plan
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;
		auto virtual collectNrt()->void;

		explicit MoveJM(const std::string &name = "MoveJM_plan");
		ARIS_REGISTER_TYPE(MoveJM);
	};

	class Xbox : public aris::plan::Plan
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual collectNrt()->void;
		explicit Xbox(const std::string &name = "Xbox_plan");
		ARIS_REGISTER_TYPE(Xbox);
	};

	class DMode : public aris::plan::Plan
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual collectNrt()->void;
		explicit DMode(const std::string &name = "DMode_plan");
		ARIS_REGISTER_TYPE(DMode);
	};

	class DEnable : public aris::plan::Plan
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual collectNrt()->void;
		explicit DEnable(const std::string &name = "DEnable_plan");
		ARIS_REGISTER_TYPE(DEnable);
	};

	class DDisable : public aris::plan::Plan
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual collectNrt()->void;
		explicit DDisable(const std::string &name = "DDisable_plan");
		ARIS_REGISTER_TYPE(DDisable);
	};

	class DJ1 : public aris::plan::Plan
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual collectNrt()->void;
		explicit DJ1(const std::string &name = "DJ1_plan");
		ARIS_REGISTER_TYPE(DJ1);
	};

	class DJ2 : public aris::plan::Plan
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual collectNrt()->void;
		explicit DJ2(const std::string &name = "DJ2_plan");
		ARIS_REGISTER_TYPE(DJ2);
	};

	class DJ3 : public aris::plan::Plan
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual collectNrt()->void;
		explicit DJ3(const std::string &name = "DJ3_plan");
		ARIS_REGISTER_TYPE(DJ3);
	};

    class DHome : public aris::plan::Plan
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual collectNrt()->void;
        explicit DHome(const std::string &name = "DHome_plan");
        ARIS_REGISTER_TYPE(DHome);
    };

	class DMoveAbsJ : public aris::plan::Plan
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual collectNrt()->void;
		explicit DMoveAbsJ(const std::string &name = "DMoveAbsJ_plan");
		ARIS_REGISTER_TYPE(DMoveAbsJ);
	};

	class SaveHome : public aris::plan::Plan
	{
	public:
		auto virtual prepareNrt()->void;
		explicit SaveHome(const std::string &name = "SaveHome_plan");
		ARIS_REGISTER_TYPE(SaveHome);
	};

	class SetHome : public aris::plan::Plan
	{
	public:
		auto virtual prepareNrt()->void;
		explicit SetHome(const std::string &name = "SetHome_plan");
		ARIS_REGISTER_TYPE(SetHome);
	};

	class ToHome : public aris::plan::Plan
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;
		auto virtual collectNrt()->void;

		explicit ToHome(const std::string &name = "ToHome_plan");
		ARIS_REGISTER_TYPE(ToHome);
	};

	class SaveP : public aris::plan::Plan
	{
	public:
		auto virtual prepareNrt()->void;
		explicit SaveP(const std::string &name = "SaveP_plan");
        auto virtual collectNrt()->void;
		ARIS_REGISTER_TYPE(SaveP);
	};

}

#endif
