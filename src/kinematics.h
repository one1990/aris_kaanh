#ifndef KINEMATICS_H_
#define KINEMATICS_H_

#include <aris.hpp>
#include <iostream>
#include <fstream>
#include <typeinfo>
#include <vector>
#include <memory>
#include <atomic>
#include <string> 
#include <algorithm>


class FourPoints : public aris::plan::Plan
{
public:
	auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;

	explicit FourPoints(const std::string &name = "FourPoints");
	ARIS_REGISTER_TYPE(FourPoints);
};

class SetTool : public aris::plan::Plan
{
public:
	auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;

	explicit SetTool(const std::string &name = "SetTool");
	ARIS_REGISTER_TYPE(SetTool);
};

#endif
