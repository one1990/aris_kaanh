#ifndef ROBOTPLAN_H_
#define ROBOTPLAN_H_

#include <aris.hpp>
#include <iostream>
#include <fstream>
#include <typeinfo>
#include <vector>
#include <memory>
#include <atomic>
#include <string> 
#include <algorithm>


class DoubleSPlan : public aris::plan::Plan
{
public:
	auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
	auto virtual executeRT(aris::plan::PlanTarget &target)->int;
	auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

	explicit DoubleSPlan(const std::string &name = "DoubleSPlan");
	ARIS_REGISTER_TYPE(DoubleSPlan);
};








#endif
