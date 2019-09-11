#ifndef SIXDISTALFC_H_
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

class MoveDistalSave : public aris::plan::Plan
{
public:
	auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;

	explicit MoveDistalSave(const std::string &name = "MoveDistalSave");
	ARIS_REGISTER_TYPE(MoveDistalSave);
};

class DistalTest : public aris::plan::Plan
{
public:
    auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
    auto virtual executeRT(aris::plan::PlanTarget &target)->int;

    explicit DistalTest(const std::string &name = "DistalTest");
    ARIS_REGISTER_TYPE(DistalTest);
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
    auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

	explicit MoveJoint(const std::string &name = "MoveJoint");
	ARIS_REGISTER_TYPE(MoveJoint);
};

class ForceDirect : public aris::plan::Plan
{
public:
	auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
	auto virtual executeRT(aris::plan::PlanTarget &target)->int;
    auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

	explicit ForceDirect(const std::string &name = "ForceDirect");
	ARIS_REGISTER_TYPE(ForceDirect);
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

class MovePressureToolYZ : public aris::plan::Plan
{
public:
	auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
	auto virtual executeRT(aris::plan::PlanTarget &target)->int;
	//auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

	explicit MovePressureToolYZ(const std::string &name = "MovePressureToolYZ");
	ARIS_REGISTER_TYPE(MovePressureToolYZ);
};

class MovePressureToolXY : public aris::plan::Plan
{
public:
	auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
	auto virtual executeRT(aris::plan::PlanTarget &target)->int;
    auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

	explicit MovePressureToolXY(const std::string &name = "MovePressureToolXY");
	ARIS_REGISTER_TYPE(MovePressureToolXY);
};

class MovePressureToolXLine : public aris::plan::Plan
{
public:
    auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
    auto virtual executeRT(aris::plan::PlanTarget &target)->int;
    auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

    explicit MovePressureToolXLine(const std::string &name = "MovePressureToolXLine");
    ARIS_REGISTER_TYPE(MovePressureToolXLine);
};

class MovePressureToolYLine : public aris::plan::Plan
{
public:
	auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
	auto virtual executeRT(aris::plan::PlanTarget &target)->int;
	auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

	explicit MovePressureToolYLine(const std::string &name = "MovePressureToolYLine");
	ARIS_REGISTER_TYPE(MovePressureToolYLine);
};

class MovePressureToolXSine : public aris::plan::Plan
{
public:
	auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
	auto virtual executeRT(aris::plan::PlanTarget &target)->int;
	auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

	explicit MovePressureToolXSine(const std::string &name = "MovePressureToolXSine");
	ARIS_REGISTER_TYPE(MovePressureToolXSine);
};

class MoveForceXSine : public aris::plan::Plan
{
public:
	auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
	auto virtual executeRT(aris::plan::PlanTarget &target)->int;
	auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

	explicit MoveForceXSine(const std::string &name = "MoveForceXSine");
	ARIS_REGISTER_TYPE(MoveForceXSine);
};

class MoveForceCircle : public aris::plan::Plan
{
public:
	auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
	auto virtual executeRT(aris::plan::PlanTarget &target)->int;
	auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

	explicit MoveForceCircle(const std::string &name = "MoveForceCircle");
	ARIS_REGISTER_TYPE(MoveForceCircle);
};

class MoveForceCurve : public aris::plan::Plan
{
public:
	auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
	auto virtual executeRT(aris::plan::PlanTarget &target)->int;
	auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

	explicit MoveForceCurve(const std::string &name = "MoveForceCurve");
	ARIS_REGISTER_TYPE(MoveForceCurve);
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
