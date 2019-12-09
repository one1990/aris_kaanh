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

class MoveJoint : public aris::plan::Plan
{
public:
	auto virtual prepairNrt()->void;
	auto virtual executeRT()->int;
    auto virtual collectNrt()->void;

	explicit MoveJoint(const std::string &name = "MoveJoint");
	ARIS_REGISTER_TYPE(MoveJoint);
};

class Replay : public aris::plan::Plan
{
public:
    auto virtual prepairNrt()->void;
    auto virtual executeRT()->int;
    auto virtual collectNrt()->void;

    explicit Replay(const std::string &name = "Replay");
    ARIS_REGISTER_TYPE(Replay);
};

class ForceDirect : public aris::plan::Plan
{
public:
	auto virtual prepairNrt()->void;
	auto virtual executeRT()->int;
    auto virtual collectNrt()->void;

	explicit ForceDirect(const std::string &name = "ForceDirect");
	ARIS_REGISTER_TYPE(ForceDirect);
};

class FCStop : public aris::plan::Plan
{
public:
    auto virtual prepairNrt()->void;
    explicit FCStop(const std::string &name = "FCStop");
    ARIS_REGISTER_TYPE(FCStop);
};

#endif