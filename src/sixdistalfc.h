﻿#ifndef SIXDISTALFC_H_
#define SIXDISTALFC_H_

#include <aris.hpp>
#include <iostream>
#include <fstream>
#include <typeinfo>
#include <vector>
#include <memory>
#include <atomic>
#include <string> 
#include <algorithm>

constexpr double f2c_index[6] = { 9.07327526291993, 9.07327526291993, 17.5690184835913, 39.0310903520972, 66.3992503259041, 107.566785527965 };

class MoveDistal : public aris::plan::Plan
{
public:
	auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
	auto virtual executeRT(aris::plan::PlanTarget &target)->int;
	auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

	explicit MoveDistal(const std::string &name = "MoveDistal");
	ARIS_REGISTER_TYPE(MoveDistal);
};

class MoveXYZ : public aris::plan::Plan
{
public:
    auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
    auto virtual executeRT(aris::plan::PlanTarget &target)->int;
    //auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

    explicit MoveXYZ(const std::string &name = "MoveXYZ");
	ARIS_REGISTER_TYPE(MoveXYZ);
};

class MoveJoint : public aris::plan::Plan
{
public:
	auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
	auto virtual executeRT(aris::plan::PlanTarget &target)->int;
	//auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

	explicit MoveJoint(const std::string &name = "MoveJoint");
	ARIS_REGISTER_TYPE(MoveJoint);
};



class SetTool : public aris::plan::Plan
{
public:
	auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
	auto virtual executeRT(aris::plan::PlanTarget &target)->int;
	auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

	explicit SetTool(const std::string &name = "SetTool");
	ARIS_REGISTER_TYPE(SetTool);
};






class MovePressure: public aris::plan::Plan
{
public:
    auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
    auto virtual executeRT(aris::plan::PlanTarget &target)->int;
    //auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

    explicit MovePressure(const std::string &name = "MovePressure");
	ARIS_REGISTER_TYPE(MovePressure);
};

class MovePressureTool : public aris::plan::Plan
{
public:
	auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
	auto virtual executeRT(aris::plan::PlanTarget &target)->int;
	//auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

	explicit MovePressureTool(const std::string &name = "MovePressureTool");
	ARIS_REGISTER_TYPE(MovePressureTool);
};

class MoveFeed : public aris::plan::Plan
{
public:
	auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
	auto virtual executeRT(aris::plan::PlanTarget &target)->int;
	//auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

	explicit MoveFeed(const std::string &name = "MoveFeed");
    ARIS_REGISTER_TYPE(MoveFeed);
};


//与Meng交换数据
class GetForce : public aris::plan::Plan
{
public:
	auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
    auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

	explicit GetForce(const std::string &name = "GetForce");
	ARIS_REGISTER_TYPE(GetForce);
};




#endif