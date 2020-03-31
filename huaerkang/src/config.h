#ifndef CONFIG_H_
#define CONFIG_H_

#include <memory>
#include <aris.hpp>

//global time speed array//
extern double timespeed[101];

namespace config
{
	//其他参数和函数声明 
	using Size = std::size_t;
	constexpr double PI = 3.141592653589793;

    auto createController()->std::unique_ptr<aris::control::Controller>;
    auto createModel(const double *robot_pm = nullptr)->std::unique_ptr<aris::dynamic::Model>;

	auto createPlanRoot()->std::unique_ptr<aris::plan::PlanRoot>;
}

#endif
