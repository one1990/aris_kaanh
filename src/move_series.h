#ifndef MOVESERIES_H_
#define MOVESERIES_H_

#include <aris.hpp>
#include <iostream>
#include <fstream>
#include <typeinfo>
#include <vector>
#include <memory>
#include <atomic>
#include <string> 
#include <algorithm>


class MoveSeries :public aris::plan::Plan
{
public:
	auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
	auto virtual executeRT(aris::plan::PlanTarget &target)->int;

	virtual ~MoveSeries();
	explicit MoveSeries(const std::string &name = "move_series");
	ARIS_REGISTER_TYPE(MoveSeries);


};

#endif

