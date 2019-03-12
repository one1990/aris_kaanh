#ifndef SIXDISTALFC_H_
#define SIXDISTALFC_H_

#include <aris.h>
#include <iostream>
#include <fstream>
#include <typeinfo>
#include <vector>
#include <memory>
#include <aris_control.h>
#include <aris_dynamic.h>
#include <aris_plan.h>
#include <tinyxml2.h>
#include <atomic>
#include <string> 
#include <algorithm>

# define FORE_VEL_LENGTH 20	//速度平均值滤波buffer长度
# define MEDIAN_LENGTH 41	//中值滤波buffer长度

class MoveDistal : public aris::plan::Plan
{
public:
	auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
	auto virtual executeRT(aris::plan::PlanTarget &target)->int;
	//auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

	explicit MoveDistal(const std::string &name = "MoveDistal");
};

class MoveXYZ : public aris::plan::Plan
{
public:
    auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
    auto virtual executeRT(aris::plan::PlanTarget &target)->int;
    //auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

    explicit MoveXYZ(const std::string &name = "MoveXYZ");
};






class MovePressure: public aris::plan::Plan
{
public:
    auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
    auto virtual executeRT(aris::plan::PlanTarget &target)->int;
    //auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

    explicit MovePressure(const std::string &name = "MovePressure");
};


#endif