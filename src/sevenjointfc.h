#ifndef SEVENJOINTFC_H_
#define SEVENJOINTFC_H_

#include <aris.hpp>
#include <iostream>
#include <fstream>
#include <typeinfo>
#include <vector>
#include <memory>
#include <atomic>
#include <string> 
#include <algorithm>


class SevenJointDyna : public aris::plan::Plan
{
public:
	auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
	auto virtual executeRT(aris::plan::PlanTarget &target)->int;
	auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

	explicit SevenJointDyna(const std::string &name = "SevenJointDyna");
	ARIS_REGISTER_TYPE(SevenJointDyna);
};

class SevenJointTest : public aris::plan::Plan
{
public:
	auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
	auto virtual executeRT(aris::plan::PlanTarget &target)->int;

	explicit SevenJointTest(const std::string &name = "SevenJointTest");
	ARIS_REGISTER_TYPE(SevenJointTest);
};

class SevenDragTeach : public aris::plan::Plan
{
public:
	auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
	auto virtual executeRT(aris::plan::PlanTarget &target)->int;

	explicit SevenDragTeach(const std::string &name = "SevenDragTeach");
	ARIS_REGISTER_TYPE(SevenDragTeach);
};



class SevenLoadDyna : public aris::plan::Plan
{
public:
	auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
	auto virtual executeRT(aris::plan::PlanTarget &target)->int;
	auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

	explicit SevenLoadDyna(const std::string &name = "SevenLoadDyna");
	ARIS_REGISTER_TYPE(SevenLoadDyna);
};




/*
//与Meng交换数据
class GetError : public aris::plan::Plan
{
public:
	auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
	auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

	explicit GetError(const std::string &name = "GetError");
	ARIS_REGISTER_TYPE(GetError);
};*/




#endif
