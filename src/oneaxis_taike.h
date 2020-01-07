#ifndef FORCECONTROL_H_
#define FORCECONTROL_H_

#include <memory>
#include <aris.hpp>
#include <atomic>

//其他参数和函数声明 
namespace taike
{
	using Size = std::size_t;
	constexpr double PI = 3.141592653589793;

	class MoveJS : public aris::plan::Plan
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;
		auto virtual collectNrt()->void;

		explicit MoveJS(const std::string &name = "MoveJS_plan");
		ARIS_REGISTER_TYPE(MoveJS);
	};

	class MoveEAP : public aris::plan::Plan
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;
		auto virtual collectNrt()->void;

		explicit MoveEAP(const std::string &name = "MoveEAP_plan");
		ARIS_REGISTER_TYPE(MoveEAP);
	};

	class Status : public aris::plan::Plan
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;
		auto virtual collectNrt()->void;

		explicit Status(const std::string &name = "Status_plan");
		ARIS_REGISTER_TYPE(Status);
	};

	class Clear : public aris::plan::Plan
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual collectNrt()->void;

		explicit Clear(const std::string &name = "Clear_plan");
		ARIS_REGISTER_TYPE(Clear);
	};

	class SetBrake : public aris::plan::Plan
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual collectNrt()->void;

		explicit SetBrake(const std::string &name = "SetBrake_plan");
		ARIS_REGISTER_TYPE(SetBrake);
	};

	class MotorMode : public aris::plan::Plan
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual collectNrt()->void;

		explicit MotorMode(const std::string &name = "MotorMode_plan");
		ARIS_REGISTER_TYPE(MotorMode);
	};

	class EnableMotor : public aris::plan::Plan
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;
		auto virtual collectNrt()->void;

		explicit EnableMotor(const std::string &name = "EnableMotor_plan");
		ARIS_REGISTER_TYPE(EnableMotor);
	};
}
#endif
