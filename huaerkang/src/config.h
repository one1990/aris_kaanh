#ifndef CONFIG_H_
#define CONFIG_H_

#include <memory>
#include <aris.hpp>
#include "kaanh/kaanhconfig.h"
#include "kaanh.h"

//global time speed array//
extern double timespeed[101];

namespace config
{
	//其他参数和函数声明 
	using Size = std::size_t;
	constexpr double PI = 3.141592653589793;

    class GVel : public aris::plan::Plan
    {
    public:
        auto virtual prepareNrt()->void;
        explicit GVel(const std::string &name = "GVel");
        ARIS_REGISTER_TYPE(GVel);

    };

	class MoveAbs :public aris::plan::Plan
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;
		auto virtual collectNrt()->void;

		virtual ~MoveAbs();
        explicit MoveAbs(const std::string &name = "moveabs");
        ARIS_REGISTER_TYPE(MoveAbs);
	};

	class MoveLine :public aris::plan::Plan
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;
		auto virtual collectNrt()->void;

		virtual ~MoveLine();
        explicit MoveLine(const std::string &name = "moveline");
        ARIS_REGISTER_TYPE(MoveLine);
	};

	class MoveJ : public aris::plan::Plan
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;
		auto virtual collectNrt()->void;

		virtual ~MoveJ();
        explicit MoveJ(const std::string &name = "movejoint");
        ARIS_REGISTER_TYPE(MoveJ);

	};

    auto createController()->std::unique_ptr<aris::control::Controller>;
    auto createModel(const double *robot_pm = nullptr)->std::unique_ptr<aris::dynamic::Model>;
	auto createPlanRoot()->std::unique_ptr<aris::plan::PlanRoot>;
}

#endif
