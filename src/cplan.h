#ifndef ROKAE_CPLAN_H_
#define ROKAE_CPLAN_H_

#include <memory>
#include <aris_control.h>
#include <aris_dynamic.h>
#include <aris_plan.h>
#include <tinyxml2.h>
#include <atomic>


class MoveCircle : public aris::plan::Plan
{
public:
    auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
    auto virtual executeRT(aris::plan::PlanTarget &target)->int;
    //auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

    explicit MoveCircle(const std::string &name = "MoveCircle");
};



class MoveTroute : public aris::plan::Plan
{
public:
    auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
    auto virtual executeRT(aris::plan::PlanTarget &target)->int;
    //auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

    explicit MoveTroute(const std::string &name = "MoveTroute");
};


class MoveFile : public aris::plan::Plan
{
public:
    auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
    auto virtual executeRT(aris::plan::PlanTarget &target)->int;
    //auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

    explicit MoveFile(const std::string &name = "MoveFile");
};


#endif
