#ifndef ROBOT_H_
#define ROBOT_H_

#include <memory>
#include <aris.hpp>

namespace robot
{
	//其他参数和函数声明 
	using Size = std::size_t;
	constexpr double PI = 3.141592653589793;
	class MoveS :public aris::plan::Plan
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;
		auto virtual collectNrt()->void;

		virtual ~MoveS();
		explicit MoveS(const std::string &name = "mvs");
		ARIS_REGISTER_TYPE(MoveS);
	};

	auto createPlanRoot()->std::unique_ptr<aris::plan::PlanRoot>;
}

#endif
