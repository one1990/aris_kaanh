#ifndef KAANHCONFIG_H_
#define KAANHCONFIG_H_

#include <memory>
#include <aris.hpp>

//global time speed array//
extern double timespeed[101];

namespace kaanhconfig
{
	//其他参数和函数声明 
	using Size = std::size_t;
	constexpr double PI = 3.141592653589793;

	auto createControllerEA()->std::unique_ptr<aris::control::Controller>;

	auto createModelRokaeXB4(const double *robot_pm = nullptr)->std::unique_ptr<aris::dynamic::Model>;
	auto createControllerRokaeXB4()->std::unique_ptr<aris::control::Controller>;
	auto createModelRokae()->std::unique_ptr<aris::dynamic::Model>;
	
	auto createControllerSanXiang()->std::unique_ptr<aris::control::Controller>;
	auto createModelSanXiang()->std::unique_ptr<aris::dynamic::Model>;

	auto createControllerQifan()->std::unique_ptr<aris::control::Controller>;
	auto createModelQifan()->std::unique_ptr<aris::dynamic::Model>;

    auto createControllerDaye()->std::unique_ptr<aris::control::Controller>;
    auto createModelDaye(const double *robot_pm = nullptr)->std::unique_ptr<aris::dynamic::Model>;

	auto createUserDataType(aris::core::Calculator &cal)->void;

	auto createPauseTimeSpeed()->void;

	auto createPlanRoot()->std::unique_ptr<aris::plan::PlanRoot>;
}

#endif
