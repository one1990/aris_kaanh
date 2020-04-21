#ifndef WULINGCONFIG_H_
#define WULINGCONFIG_H_

#include <memory>
#include <aris.hpp>

//global time speed array//
extern double timespeed[101];

namespace wulingconfig
{
	//其他参数和函数声明 
	using Size = std::size_t;
	constexpr double PI = 3.141592653589793;

    auto createControllerWuling()->std::unique_ptr<aris::control::Controller>;
    auto createModelWuling(const double *robot_pm = nullptr)->std::unique_ptr<aris::dynamic::Model>;

	auto createUserDataType(aris::core::Calculator &cal)->void;

    auto createPauseTimeSpeed()->void;

	auto createPlanRoot()->std::unique_ptr<aris::plan::PlanRoot>;
}

#endif
